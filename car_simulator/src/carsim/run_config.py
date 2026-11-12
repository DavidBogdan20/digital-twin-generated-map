"""
Per-run files of run_scenario.py: the weather and ego-driver overrides, the copy
of the scenario XML that ScenarioRunner is given, the --record path and the
run.json metadata, and the metrics file the metrics block writes.
"""

import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

from carsim.blocks.spec import EGO_DEFAULT_MODE, EGO_DEFAULT_TARGET_SPEED, EGO_MODES, ending_marker
from carsim.paths import SCENARIO_RUNNER_ROOT

XML_WEATHER = "xml"  # --weather value meaning "as configured in the XML"
XML_DRIVE = "xml"  # --drive value meaning "the variant's <ego_control mode>"
DRIVE_CHOICES = (XML_DRIVE,) + EGO_MODES  # --drive values: xml, autopilot, manual

# Order of the attributes in the written <weather> element; fields the carla build has beyond these are appended
_FIELD_ORDER = (
    "cloudiness", "precipitation", "precipitation_deposits", "wind_intensity", "sun_azimuth_angle",
    "sun_altitude_angle", "fog_density", "fog_distance", "fog_falloff", "wetness", "scattering_intensity",
    "mie_scattering_scale", "rayleigh_scattering_scale", "dust_storm",
)


def weather_presets():
    """Names of the carla.WeatherParameters presets (ClearNoon, HardRainNight, ...)."""
    names = []
    for name in dir(carla.WeatherParameters):
        if name[:1].isupper() and isinstance(getattr(carla.WeatherParameters, name), carla.WeatherParameters):
            names.append(name)
    return sorted(names)


def _numeric_fields(params):
    """{attribute: value} of every numeric field of a carla.WeatherParameters, rounded to 6 digits."""
    values = {}
    for name in dir(params):
        if name.startswith("_"):
            continue
        value = getattr(params, name)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            continue  # methods, __doc__, ...
        values[name] = round(float(value), 6)
    ordered = {name: values.pop(name) for name in _FIELD_ORDER if name in values}
    ordered.update(sorted(values.items()))
    return ordered


def weather_values(preset):
    """The numeric fields of a preset, e.g. weather_values("HardRainNight")."""
    if preset not in weather_presets():
        raise ValueError(f"Unknown weather preset '{preset}'. Available: {', '.join(weather_presets())}")
    return _numeric_fields(getattr(carla.WeatherParameters, preset))


def effective_ego_mode(xml_params, drive=XML_DRIVE):
    """
    The ego_control mode a run uses: `drive` when it is "autopilot" or "manual", else the variant's own
    <ego_control mode> (`xml_params` is Variant.params), which is the autopilot when the variant has no
    <ego_control> element or no mode attribute.
    """
    if drive not in DRIVE_CHOICES:
        raise ValueError(f"Unknown drive option '{drive}'. Available: {', '.join(DRIVE_CHOICES)}")
    if drive != XML_DRIVE:
        return drive
    return xml_params.get("ego_control", {}).get("mode", EGO_DEFAULT_MODE)


def _apply_drive(node, drive):
    """
    Set `mode` on the variant's <ego_control> element (created when missing). Switching to the autopilot also
    makes sure the element has a target_speed: the XML's own is kept, else the block's default is written.
    """
    element = node.find("ego_control")
    if element is None:
        element = ET.SubElement(node, "ego_control")  # the block order does not depend on the XML order
    element.set("mode", drive)
    if drive == "autopilot" and "target_speed" not in element.attrib:
        element.set("target_speed", format(EGO_DEFAULT_TARGET_SPEED, ".6g"))


def _apply_stop_file(node, stop_file):
    """
    Make the variant's <run_control stop_file="..."/> element the one of this run: any existing one is removed,
    and with a `stop_file` a new one is added. A stale stop file is deleted, so the run does not end at once, and so is
    a stale ending marker, so the window's watcher does not take it for this run's.
    """
    for stale in node.findall("run_control"):
        node.remove(stale)
    if stop_file is None:
        return
    stop_path = Path(os.path.abspath(stop_file))
    stop_path.unlink(missing_ok=True)
    Path(ending_marker(stop_path)).unlink(missing_ok=True)
    ET.SubElement(node, "run_control", {"stop_file": str(stop_path)})


def write_run_config(source_xml, variant_name, weather, dest_xml, drive=XML_DRIVE, stop_file=None, ego_z=None):
    """
    Write a copy of the scenario XML to `dest_xml`. With `weather` a preset name, the variant's <weather>
    element is replaced by one with every numeric field of that preset, so nothing of the XML's own weather
    survives (ScenarioRunner only overrides the attributes the element has). With `weather` "xml" the
    element is kept. With `drive` "autopilot" or "manual" the variant's <ego_control> gets that mode (the
    element is created when missing, and the autopilot keeps the XML's target_speed or gets the block's
    default); with "xml" it is left alone. With `stop_file` (a path) the variant gets
    <run_control stop_file="<absolute path>"/>, which ends a scenario without a time limit cleanly once that file
    exists (a stale file is deleted here); without it the variant has no <run_control> in the copy. With `ego_z` the
    variant's <ego_vehicle> spawns at that z (the road surface of a generated map can sit above the XML's). Other
    variants are not touched. Comments are not kept. Returns the weather values the copy configures: those of
    the preset, or for "xml" the attributes of the XML's element (fields it does not set keep ScenarioRunner's
    defaults).
    """
    if drive not in DRIVE_CHOICES:
        raise ValueError(f"Unknown drive option '{drive}'. Available: {', '.join(DRIVE_CHOICES)}")
    tree = ET.parse(source_xml)
    node = next((n for n in tree.getroot().iter("scenario") if n.get("name") == variant_name), None)
    if node is None:
        raise ValueError(f"Variant '{variant_name}' not found in {source_xml}")
    old = node.findall("weather")
    if drive != XML_DRIVE:
        _apply_drive(node, drive)
    _apply_stop_file(node, stop_file)
    if ego_z is not None and node.find("ego_vehicle") is not None:
        node.find("ego_vehicle").set("z", format(ego_z, ".3f"))

    if weather == XML_WEATHER:
        used = {}
        for element in old:
            for key, value in element.attrib.items():
                try:
                    used[key] = float(value)
                except ValueError:
                    pass
    else:
        used = weather_values(weather)
        probe = carla.WeatherParameters()
        unknown = [key for key in used if not hasattr(probe, key)]
        if unknown:  # ScenarioRunner would ignore these with a warning
            raise ValueError(f"Weather fields {unknown} are not attributes of carla.WeatherParameters")
        element = ET.Element("weather", {key: format(value, ".6g") for key, value in used.items()})
        index = list(node).index(old[0]) if old else len(node)
        for stale in old:
            node.remove(stale)
        node.insert(index, element)

    if hasattr(ET, "indent"):  # Python 3.9+; on 3.8 (the VM) the copy stays as parsed
        ET.indent(tree, space="    ")
    Path(dest_xml).parent.mkdir(parents=True, exist_ok=True)
    tree.write(dest_xml, encoding="utf-8", xml_declaration=True)
    return used


def record_argument(run_dir, variant_name):
    """
    ScenarioRunner's --record value that makes CARLA's recorder write into `run_dir`, and the file it will
    write there.

    scenario_runner.py builds "{SCENARIO_RUNNER_ROOT}/{record}/{scenario name}.log" (the environment
    variable is set by runner.scenario_runner_env to paths.SCENARIO_RUNNER_ROOT), so `record` is the run
    folder relative to that root, e.g. "..\\..\\outputs\\20261029_222630_Name". The CARLA server, which
    writes the file, resolves the ".." parts. Raises ValueError when no such relative path exists (the run
    folder is on another drive than ScenarioRunner).
    """
    root = os.path.abspath(SCENARIO_RUNNER_ROOT)
    run_dir = os.path.abspath(run_dir)
    try:
        record = os.path.relpath(run_dir, root)
    except ValueError as e:
        raise ValueError(f"--record needs {run_dir} to be on the same drive as ScenarioRunner ({root}): {e}")
    log = os.path.join(run_dir, f"{variant_name}.log")
    joined = "{}/{}/{}.log".format(root, record, variant_name)  # as in scenario_runner.py
    if os.path.normcase(os.path.normpath(joined)) != os.path.normcase(log):
        raise ValueError(f"Recorder path '{joined}' does not resolve to {log}")
    return record, Path(log)


def scenario_runner_random_seed():
    """The seed CarlaDataProvider fixes for ScenarioRunner's random numbers (2000 in v0.9.15 and v0.9.16), or None."""
    path = SCENARIO_RUNNER_ROOT / "srunner" / "scenariomanager" / "carla_data_provider.py"
    try:
        match = re.search(r"^\s*_random_seed\s*=\s*(\d+)", path.read_text(encoding="utf-8"), re.M)
    except OSError:
        return None
    return int(match.group(1)) if match else None


def scenario_runner_command(sr_args):
    """The command line runner.run_scenario_runner starts for `sr_args`, as a list of strings."""
    script = SCENARIO_RUNNER_ROOT / "scenario_runner.py"
    return [sys.executable, str(script)] + [str(a) for a in sr_args]


def write_run_json(run_dir, info):
    """Write (or rewrite) run.json in the run folder; `info` is a dict of JSON-serializable values."""
    path = Path(run_dir) / "run.json"
    path.write_text(json.dumps(info, indent=2, default=str) + "\n", encoding="utf-8")
    return path


def read_metrics(path):
    """The metrics dict the metrics block wrote to `path`, or None when the file is missing or not valid JSON."""
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def metrics_summary(metrics):
    """One line for the console from a metrics dict: impact, first visible TTC, brake onset TTC, min distance."""
    def number(value, unit):
        return "n/a" if value is None else f"{value:.1f} {unit}"

    impact = metrics.get("impact") or {}
    if impact.get("happened"):
        speed = impact.get("speed")
        hit = "impact YES" + ("" if speed is None else f" at {speed * 3.6:.1f} km/h")
    elif impact.get("happened") is None:
        hit = "impact unknown"  # the collision sensor could not be spawned
    else:
        hit = "impact no"
    visible = metrics.get("first_visible") or {}
    onset = metrics.get("brake_onset") or {}
    return (f"Metrics: {hit} | first visible TTC {number(visible.get('ttc'), 's')} | "
            f"brake onset TTC {number(onset.get('ttc'), 's')} | min distance {number(metrics.get('min_distance'), 'm')}")


def command_text(command):
    """A copy-pasteable string for a command list."""
    return subprocess.list2cmdline(command)
