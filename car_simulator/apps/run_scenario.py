#!/usr/bin/env python
"""
Run a scenario from the scenarios/ folder with ScenarioRunner.

    python apps/run_scenario.py pedestrian_crossing
    python apps/run_scenario.py occluded_crossing --variant OccludedCrossing_Crowd_Town10HD_1 --debug
    python apps/run_scenario.py pedestrian_crossing --weather HardRainNight --record
    python apps/run_scenario.py pedestrian_crossing --drive manual
    python apps/run_scenario.py occluded_crossing --no-view

Before handing over to ScenarioRunner this checks the scenario's naming
rules, loads its map (if it is not loaded yet), moves the spectator camera
to the ego start and waits until the road surface has loaded there (and at
the pedestrian spawn, for a variant with a pedestrian, crowd or cyclists).
Results go to outputs/<time>_<variant>/. Pass/fail is in the printed result
table and the JSON file; the exit code is 0 even when a criterion fails.
That folder also gets scenario.xml (the copy of the scenario XML that
ScenarioRunner runs, with the --weather and --drive overrides applied) and
run.json (what was run, with which settings, seed and command line, and the "metrics" of the run). The metrics
block writes metrics.json there (impact and impact speed, first visible and brake onset with their TTC, speed
reduction, min distance and TTC, duration); run.json gets a copy and the console a one-line summary.

--weather NAME replaces the weather of the variant with a CARLA preset
(ClearNoon, HardRainNight, ...); the default, xml, keeps the XML's <weather>.
--record records the run with CARLA's recorder to <variant>.log in the run
folder; play it back with apps/replay.py.

--drive says who drives the ego. xml (default) keeps the variant's
<ego_control mode>; autopilot makes the scenario drive it; manual lets you
drive it. With manual, ScenarioRunner's manual_control.py is started
automatically in its own console once the scenario is running (the ego exists
then): click into its window and drive with W/A/S/D. A manual run has no time
limit and no distance end: it ends when you close the driving window (Esc or its
X; the scenario then ends cleanly and saves its results) or press Ctrl-C here.
Do not use the window's Ctrl+R, Ctrl+P or P keys (own recorder, replay, CARLA
autopilot). --no-driver-window starts nothing and prints the command to start
it yourself; the run then ends with Ctrl-C. The autopilot run stays a single
pass that ends by itself. --drive other than xml needs a scenario built from
blocks (no .py of its own). run.json records how the run ended (end_reason:
stop_file = window closed, restart or quit = the chase view's buttons, ctrl_c, or scenario_end).

An autopilot run also opens a chase view (apps/chase_view.py, own console): a window that follows the ego, with a
HUD, and the control panel of the run. Its Restart button (R) ends the run cleanly (through a stop file, as the
driving window does) and starts it again in this process, same variant and settings, the map stays loaded, each run
with its own outputs/<time>_<variant>/ folder. Its Quit button (Esc, the X) ends the run cleanly and this command
exits. When a run ends by itself the window stays open with the last frame and the result and metrics line, and
waits for R or Quit. --no-view skips the window (a single run, as before). Ctrl-C here ends everything.

An autopilot run is synchronous by default (ScenarioRunner v0.9.15 fixed-step mode, steps of 0.05 s, fixed):
repeatable, and the scenario and the chase view cannot starve each other, but it runs as fast as the PC allows, not
in real time. --async opts out (real time, not repeatable). A manual run is asynchronous by default (you drive it in
real time); --sync forces the synchronous mode.

Large maps (WolfsburgHighway, Town12): after loading, the streaming distances are set to --stream-distance and
--active-distance (load_world resets them, and the defaults crashed a 0.9.15 server), the spectator travels to the
start in short steps, and when the road surface there sits higher than the XML's ego z (the generated road mesh waves
about +-1.2 m around the OpenDRIVE height) the ego is spawned that much higher. ScenarioRunner's Traffic Manager uses
--tm-port (8100: 8000 is taken on the VM; its first start on a large map can take minutes). --view-spectator makes the
chase view move the spectator instead of attaching a camera sensor to the ego (see apps/chase_view.py). On Linux the
chase view and the driving window log to chase_view.log and manual_control.log in the run folder.
"""

import argparse
import os
import shutil
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import carla  # noqa: E402

from carsim.blocks.spec import CROSSING_BLOCKS, ending_marker  # noqa: E402
from carsim.paths import OUTPUTS_DIR, REPO_ROOT, list_scenario_names  # noqa: E402
from carsim.road import CrosswalkIndex, crosswalks, deformed_map, find_crossing, surface_offset  # noqa: E402
from carsim.run_config import (DRIVE_CHOICES, XML_DRIVE, XML_WEATHER, command_text, effective_ego_mode,  # noqa: E402
                               metrics_summary, read_metrics, record_argument, scenario_runner_command,
                               scenario_runner_random_seed, weather_presets, write_run_config, write_run_json)
from carsim.runner import (EGO_DESTROYED_LINE, QUIT_REQUEST, RESTART_REQUEST, SCENARIO_ENDING_LINE,  # noqa: E402
                           SCENARIO_RUNNING_LINE, clear_request, clear_result, manual_control_arguments, next_step,
                           parse_result_line, pending_request, run_scenario_runner, start_chase_view,
                           start_manual_control, stop_process, wait_for_request, watch_requests, write_request,
                           write_result)
from carsim.scenario_files import COMPOSED_SCENARIO_FILE, config_file, scenario_file, validate, variants  # noqa: E402
from carsim.sim import (ACTOR_ACTIVE_DISTANCE, GROUND_LABELS, TILE_STREAM_DISTANCE, TRAFFIC_MANAGER_PORT,  # noqa: E402
                        another_client_ticks, connect, ensure_map, focus_spectator, reset_world, wait_for_ground)


EGO_Z_ABOVE_ROAD = 0.3  # ego start z in the XML = road z + this [m]
PEDESTRIAN_Z_ABOVE_GROUND = 1.0  # road.pedestrian_path spawns the pedestrian this high [m]
SPECTATOR_AT_START = {"behind": 12.0, "height": 6.0, "pitch": -20.0}  # the CARLA window starts as a chase view of the ego

TRAFFIC_MANAGER_SEED = 0  # ScenarioRunner's --trafficManagerSeed default; run_scenario does not change it


SR_FRAME_RATE = 20.0  # ScenarioRunner v0.9.15's synchronous steps per second, fixed (it has no --frameRate option)


def scenario_runner_arguments(variant, scenario_py, scenario_xml, host, port, timeout, tm_port, run_dir, sync,
                              record=None, debug=False):
    """The command-line arguments of scenario_runner.py (v0.9.15) for one run."""
    sr_args = [
        "--scenario", variant,
        "--additionalScenario", scenario_py,
        "--configFile", scenario_xml,
        "--host", host,
        "--port", port,
        "--timeout", timeout,
        "--trafficManagerPort", tm_port,
        "--output", "--json", "--junit",
        "--outputDir", run_dir,
    ]
    if sync:
        sr_args.append("--sync")
    if record is not None:
        sr_args += ["--record", record]
    if debug:
        sr_args.append("--debug")
    return sr_args


def manual_control_command(host, port, tm_port=TRAFFIC_MANAGER_PORT):
    """The command that opens the driving window of a manual-mode run, as run from the repository root."""
    python = "python -m uv run python" if os.name == "nt" else "python3"  # the repo's command forms (README, PORTING)
    cmd = [python, os.path.join("apps", "manual_control_tm.py"), "--tm-port", str(tm_port)]
    return " ".join(cmd + manual_control_arguments(host, port))


class DrivingWindow:

    """
    The manual_control.py process of a manual-mode run: started when ScenarioRunner reports that the scenario is
    running (the ego exists by then), stopped when the run ends. Pass `feed` to run_scenario_runner as on_line and
    `mark_ending` as on_interrupt.

    The window is started with --keep_ego_vehicle, so closing it (Esc, its X, Ctrl+C in its console, a crash) leaves
    the ego to ScenarioRunner. A watcher thread turns that into the end of the run: it creates `stop_file`, which the
    scenario watches (<run_control stop_file>), so the scenario ends cleanly and the results are saved. Without a
    `stop_file` (a scenario that does not read one) it only tells the user to press Ctrl-C.

    A window that exits because the run is ending is not the user closing it. The scenario says so twice: with its
    output line (see `feed`), which may not have been read yet when the console holds the output back, and with the
    marker file next to the stop file that ComposedScenario.terminate() creates, which the watcher checks too.
    """

    POLL_SECONDS = 0.2  # how often the watcher looks at the window
    ENDING_WAIT = 1.5  # s: a window that exited waits this long for the scenario's ending marker before it counts as closed
    ENDING_POLL = 0.1  # s: ... looking this often
    EXIT_WAIT = 5.0  # s: stop() lets the window exit by itself (it does when the ego is gone) before terminating it

    def __init__(self, host, port, stop_file=None, log_file=None, tm_port=TRAFFIC_MANAGER_PORT):
        self.host = host
        self.port = port
        self.tm_port = tm_port
        self.log_file = log_file  # its output on Linux
        self.stop_file = Path(stop_file) if stop_file is not None else None
        self.proc = None
        self.tried = False
        self.window_closed = False  # the user closed the window while the run was going (the watcher acted)
        self.stop_file_written = False  # the watcher created the stop file (the only way end_reason is stop_file)
        self._lock = threading.Lock()
        self._ending = threading.Event()  # the run is ending on its own (scenario ending, Ctrl-C) or has ended (stop())

    def feed(self, line):
        # The scenario's own end marker comes before the ego is destroyed at every end; "Destroying ego vehicle" only
        # when the ego still exists at ScenarioRunner's cleanup (not at a normal end). The window exits by itself
        # right after the ego is gone: not a user closing it.
        if SCENARIO_ENDING_LINE in line or EGO_DESTROYED_LINE in line:
            self.mark_ending()
        if self.tried or SCENARIO_RUNNING_LINE not in line:
            return
        self.tried = True  # one attempt per run
        try:
            self.proc = start_manual_control(self.host, self.port, self.log_file, self.tm_port)
        except OSError as e:  # includes FileNotFoundError
            print(f"\nCould not start the driving window ({e}). Start it yourself:\n"
                  f"    {manual_control_command(self.host, self.port, self.tm_port)}\n"
                  "Press Ctrl-C here to end the run.\n")
            return
        print("\nOpened the driving window (manual_control.py, own console): click into its window and drive "
              "with W/A/S/D. Closing it ends the run.\n")
        threading.Thread(target=self._watch, args=(self.proc,), name="driving-window-watch", daemon=True).start()

    def mark_ending(self):
        """The run is ending by itself (Ctrl-C, the ego is being destroyed): a window that exits now is no request to end."""
        self._ending.set()

    def _marked_ending(self):
        """True once the scenario has created its ending marker (False without a stop file, or when it cannot be told)."""
        if self.stop_file is None:
            return False
        try:
            return os.path.exists(ending_marker(self.stop_file))
        except OSError:
            return False

    def _watch(self, proc):
        """End the run once when `proc` exits while the run is still going."""
        while proc.poll() is None:
            if self._ending.wait(self.POLL_SECONDS):
                return
            if self._marked_ending():
                self.mark_ending()
                return
        # The window also exits when the ego is destroyed (EgoGone, a crash of the run), a few ticks before the scenario
        # ends and creates its marker: give it a moment before this counts as the user closing the window
        deadline = time.monotonic() + self.ENDING_WAIT
        while time.monotonic() < deadline:
            if self._ending.wait(self.ENDING_POLL) or self._marked_ending():
                self.mark_ending()
                return
        self._window_closed()

    def _window_closed(self):
        with self._lock:
            if self._ending.is_set() or self.window_closed:
                return
            if self._marked_ending():  # the scenario has ended, the output line has just not been read yet
                self._ending.set()
                return
            self.window_closed = True
        if self.stop_file is None:
            print("\nDriving window closed. This scenario does not read a stop file, so it keeps running: "
                  "press Ctrl-C here to end the run.\n")
            return
        try:
            self.stop_file.parent.mkdir(parents=True, exist_ok=True)
            self.stop_file.touch()
        except OSError as e:
            print(f"\nDriving window closed, but the stop file could not be written ({e}): "
                  "press Ctrl-C here to end the run.\n")
            return
        self.stop_file_written = True
        print("\nDriving window closed: ending the run (results are being saved)\n")

    def stop(self):
        """
        Close the window if it is still open. It exits by itself once the ego is gone, so it gets EXIT_WAIT seconds
        for that, then it is terminated (killed after 2 s). Call this before reset_world.
        """
        self._ending.set()
        if self.proc is None:
            return
        try:
            stopped = stop_process(self.proc, grace=self.EXIT_WAIT)
        except KeyboardInterrupt:  # Ctrl+C while waiting: do not wait any longer
            stopped = stop_process(self.proc)
        if stopped:
            print("Closed the driving window.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scenario", choices=list_scenario_names(), help="scenario folder under scenarios/")
    parser.add_argument("--variant", help="variant name in the XML (default, also when empty: the first)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=60.0,
                        help="CARLA client timeout, also ScenarioRunner's no-tick watchdog [s]")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--sync", dest="sync", action="store_true",
                      help="run ScenarioRunner's synchronous fixed-step mode: repeatable, but it runs as fast as the "
                           "PC allows, not in real time (the default for an autopilot run)")
    mode.add_argument("--async", dest="sync", action="store_false",
                      help="run the simulation asynchronously, in real time, not repeatable (the default for a "
                           "manual run; an autopilot run is synchronous unless you pass this)")
    parser.set_defaults(sync=None)
    parser.add_argument("--frame-rate", type=float, default=SR_FRAME_RATE,
                        help="simulation steps per second in synchronous mode: ScenarioRunner v0.9.15 has 20 fixed, "
                             "any other value is refused")
    parser.add_argument("--drive", default=XML_DRIVE, choices=DRIVE_CHOICES,
                        help="who drives the ego: 'xml' (default) keeps the variant's <ego_control mode>, "
                             "'autopilot' lets the scenario drive it, 'manual' lets you drive it (the driving window, "
                             "manual_control.py, is started automatically)")
    parser.add_argument("--no-driver-window", action="store_true",
                        help="with --drive manual: do not start manual_control.py; print the command to start it yourself")
    parser.add_argument("--weather", default=XML_WEATHER, choices=[XML_WEATHER] + weather_presets(), metavar="NAME",
                        help="weather: 'xml' (default) keeps the variant's <weather> from the XML, or a CARLA preset: "
                             + ", ".join(weather_presets()))
    parser.add_argument("--record", action="store_true",
                        help="record the run with CARLA's recorder to <variant>.log in the run folder "
                             "(replay it with apps/replay.py)")
    parser.add_argument("--debug", action="store_true", help="draw the crossing and print the behavior tree")
    parser.add_argument("--no-view", action="store_true",
                        help="do not open the chase view window that an autopilot run gets (apps/chase_view.py)")
    parser.add_argument("--view-spectator", action="store_true",
                        help="the chase view moves the server's spectator behind the ego instead of attaching a camera "
                             "sensor (the picture is then in the CARLA window)")
    parser.add_argument("--tm-port", type=int, default=TRAFFIC_MANAGER_PORT,
                        help=f"ScenarioRunner's Traffic Manager port (default {TRAFFIC_MANAGER_PORT})")
    parser.add_argument("--stream-distance", type=float, default=None,
                        help=f"large maps: tiles load within this distance of the ego [m] (default {TILE_STREAM_DISTANCE:g}; "
                             "3000 on WolfsburgHighway, which keeps every tile loaded: sim.KEEP_LOADED)")
    parser.add_argument("--active-distance", type=float, default=None,
                        help=f"large maps: actors farther from the ego sleep [m] (default {ACTOR_ACTIVE_DISTANCE:g}; 3000 on WolfsburgHighway)")
    args = parser.parse_args()

    try:
        problems = validate(args.scenario)
        all_variants = variants(args.scenario)
    except (OSError, ValueError, ET.ParseError) as e:
        sys.exit(f"Scenario '{args.scenario}' is broken: {e}")
    if problems:
        sys.exit(f"Scenario '{args.scenario}' has problems:\n" + "\n".join(f"  - {p}" for p in problems))
    if not all_variants:
        sys.exit(f"No <scenario> entries in {config_file(args.scenario)}")
    variant = next((v for v in all_variants if v.name == args.variant), None) if args.variant else all_variants[0]
    if variant is None:
        sys.exit(f"Variant '{args.variant}' not found. Available: {', '.join(v.name for v in all_variants)}")
    # An occluder needs the ego in the lane left of the kerb lane
    spots = f"python apps/find_crossing_spots.py --scenario {args.scenario}" + (
        " --lane inner" if "occluder" in variant.params else "")
    if variant.ego.get("x", 0.0) == 0.0 and variant.ego.get("y", 0.0) == 0.0:
        sys.exit(f"The ego start in the XML is still (0, 0). Pick one with:\n    {spots} --apply 0")

    # --drive rewrites the <ego_control> of the XML copy, which only a scenario built from blocks (the shared
    # ComposedScenario) reads. A scenario with a .py of its own keeps the XML's behaviour.
    composed = scenario_file(args.scenario) == COMPOSED_SCENARIO_FILE
    if not composed and args.drive != XML_DRIVE:
        sys.exit(f"--drive {args.drive} needs a scenario built from blocks; '{args.scenario}' has its own scenario "
                 f"file ({scenario_file(args.scenario).name}), which does not read <ego_control>. Use --drive {XML_DRIVE}.")

    # The ego_control mode the run uses: --drive overrides the variant's own (write_run_config puts it in the XML copy)
    if composed:
        ego_mode = effective_ego_mode(variant.params, args.drive)
    else:
        # Best effort: the driving window opens only when the XML says manual, as the custom scenario may or may not
        # wait for a human driver
        ego_mode = variant.params.get("ego_control", {}).get("mode")
    manual = ego_mode == "manual"
    sync = args.sync if args.sync is not None else ego_mode == "autopilot"  # autopilot: sync unless --async
    if manual and not composed:
        print(f"Note: '{args.scenario}' has its own scenario file; the driving window opens because its XML says "
              "<ego_control mode=\"manual\">, but the scenario itself decides how that is used.")
    driver_window = manual and not args.no_driver_window
    if args.no_driver_window and not manual:
        print("Note: --no-driver-window only matters for manual driving; ignored.")
    if manual and sync:
        print("WARNING: manual control with --sync: the simulation does not run in real time.")
    if args.frame_rate != SR_FRAME_RATE:
        sys.exit(f"--frame-rate {args.frame_rate:g}: ScenarioRunner v0.9.15 steps a synchronous run at "
                 f"{SR_FRAME_RATE:g} per second and has no option to change it.")
    if args.record and args.host not in ("127.0.0.1", "localhost"):
        print(f"WARNING: --record: the recording is written by the server on {args.host}, "
              "so the path in the run folder only exists if that machine shares this file system.")

    ego = variant.ego
    start = carla.Location(ego["x"], ego["y"], ego["z"])
    try:
        client = connect(args.host, args.port, args.timeout)
        # Two runs on one server step the world twice per frame and the reset below would delete the other's actors
        if another_client_ticks(client.get_world()):
            sys.exit("Another scenario is running on this server (the world is synchronous and still advancing). "
                     "End it first (close its window or press Ctrl-C there), or run apps/reset_world.py if it hangs.")
        world = ensure_map(client, variant.town, timeout=args.timeout, tile_stream_distance=args.stream_distance,
                           actor_active_distance=args.active_distance)
        # Start from a clean world: a replay or a killed run leaves its actors (ego, pedestrians, cars) behind,
        # and they block spawns and the ego's route. This also switches to asynchronous mode, which
        # wait_for_ground needs (it polls with wait_for_tick). No Traffic Manager here: it would be hosted by this
        # process and ScenarioRunner's would then be a client of it, never stepped (background traffic frozen).
        reset_world(client, traffic_manager_port=None)

        # The CARLA window's camera starts behind the ego, looking along its way: fly freely from there
        focus_spectator(world, start, **SPECTATOR_AT_START)
        cmap = world.get_map()
        # A generated map's road mesh waves around the OpenDRIVE heights by a known function: the ego spawns on it
        deformed = deformed_map(cmap.name)
        ego_z = None
        if deformed:
            road_z = cmap.get_waypoint(start).transform.location.z + surface_offset(cmap, start)
            ego_z = road_z + EGO_Z_ABOVE_ROAD
            print(f"Generated map: the road surface at the ego start is at z={road_z:.2f}, the ego spawns at "
                  f"z={ego_z:.2f} (the XML says {start.z:.2f}).")
            surfaces = [carla.Location(start.x, start.y, road_z)]
        else:
            surfaces = [carla.Location(start.x, start.y, start.z - EGO_Z_ABOVE_ROAD)]

        # Only a variant with an occluder, pedestrian, crowd or cyclists uses the crossing: compute the one the scenario will
        # compute from the same ego start, to know where the pedestrian spawns. It is within about 100 m, so it is
        # in a tile that streams in with the ego's, but check it as well.
        if any(tag in variant.params for tag in CROSSING_BLOCKS):
            crossing = find_crossing(cmap, cmap.get_waypoint(start), CrosswalkIndex(crosswalks(cmap)))
            if crossing is None:
                sys.exit(f"No pedestrian crossing found from the ego start. Pick another start with:\n    {spots} --apply 0")
            ped = crossing.spawn.location
            surfaces.append(carla.Location(ped.x, ped.y, ped.z - PEDESTRIAN_Z_ABOVE_GROUND
                                           + (surface_offset(cmap, ped) if deformed else 0.0)))
        ground = wait_for_ground(world, surfaces)[0]
    except RuntimeError as e:  # includes CARLA's timeout when no server is running
        sys.exit(f"{e}")

    # Elsewhere the ego spawns at the XML's z: raise it where the road (not a tree or a prop) is higher, never lower it
    if (not deformed and ground is not None and ground.label in GROUND_LABELS
            and ground.location.z + EGO_Z_ABOVE_ROAD > start.z + 0.05):
        ego_z = ground.location.z + EGO_Z_ABOVE_ROAD
        print(f"The road surface at the ego start is at z={ground.location.z:.2f}: the ego spawns at z={ego_z:.2f} "
              f"instead of the XML's {start.z:.2f}.")

    # An autopilot run with the chase view: the window is the control panel (Restart, Quit) and stays open across runs.
    # Its requests and each run's result travel as small files in this folder
    chase = None  # the chase view process
    chase_view = ego_mode == "autopilot" and not args.no_view
    control = tempfile.mkdtemp(prefix="carsim_control_") if chase_view else None
    run_number = 0
    try:
        while True:
            run_number += 1
            if run_number > 1:  # a restart: back behind the ego start, wherever the camera was flown to
                focus_spectator(world, start, settle_seconds=0.0, **SPECTATOR_AT_START)
            started = datetime.now().astimezone()
            run_dir = OUTPUTS_DIR / f"{started:%Y%m%d_%H%M%S}_{variant.name}"
            run_dir.mkdir(parents=True, exist_ok=True)

            # ScenarioRunner runs a copy of the XML with the weather and ego-driver overrides applied
            # A manual run with the driving window has no time limit: the scenario ends when this file exists, which the
            # window's watcher creates when the window is closed (a scenario with its own .py does not read it)
            scenario_xml = run_dir / "scenario.xml"
            # An autopilot run with the chase view has one too: the window's Restart and Quit end the run through it
            stop_file = run_dir / "stop" if (driver_window or control is not None) and composed else None
            try:
                weather_values = write_run_config(config_file(args.scenario), variant.name, args.weather, scenario_xml,
                                                  drive=args.drive, stop_file=stop_file, ego_z=ego_z)
            except (OSError, ValueError, ET.ParseError) as e:
                sys.exit(f"Could not write the run configuration: {e}")

            record_log = None
            record_arg = None
            if args.record:
                try:
                    record_arg, record_log = record_argument(run_dir, variant.name)
                except ValueError as e:
                    sys.exit(str(e))

            sr_args = scenario_runner_arguments(variant.name, scenario_file(args.scenario), scenario_xml, args.host,
                                                args.port, args.timeout, args.tm_port, run_dir, sync, record_arg,
                                                args.debug)

            command = scenario_runner_command(sr_args)
            metrics_file = run_dir / "metrics.json"  # written by the metrics block (CARSIM_METRICS_FILE), merged below
            info = {
                "scenario": args.scenario,
                "variant": variant.name,
                "town": variant.town,
                "weather": {
                    "name": args.weather,
                    "values": weather_values,
                    **({"note": "xml: only the attributes the XML's <weather> sets; the rest keep ScenarioRunner's defaults"}
                       if args.weather == XML_WEATHER else {}),
                },
                "drive": {"requested": args.drive, "effective": ego_mode},
                "driver_window": driver_window,
                "stop_file": str(stop_file) if stop_file else None,
                "run_in_session": run_number,  # 2 and up: restarted from the chase view
                "sync": sync,
                "frame_rate": args.frame_rate if sync else None,
                "record": str(record_log) if record_log else None,
                "scenario_runner_command": command,
                "scenario_runner_command_line": command_text(command),
                "started": started.isoformat(timespec="seconds"),
                "random_seed": scenario_runner_random_seed(),  # CarlaDataProvider's, fixed
                "traffic_manager_seed": TRAFFIC_MANAGER_SEED,
                "traffic_manager_port": args.tm_port,
                "ego_z": ego_z,  # the ego's spawn z when raised above the XML's (null: the XML's)
                "run_dir": str(run_dir),
                "scenario_xml": str(scenario_xml),
                "metrics_file": str(metrics_file),
                "metrics": None,  # the metrics block's JSON, set when the run is over (null when it wrote none)
                "exit_code": None,
                "end_reason": None,  # stop_file (driving window closed), restart / quit (chase view), ctrl_c or scenario_end; set when the run is over
            }
            write_run_json(run_dir, info)

            window = None
            if manual:
                if driver_window:
                    window = DrivingWindow(args.host, args.port, stop_file, run_dir / "manual_control.log", args.tm_port)
                    end_note = ("No time limit: close the driving window (Esc) or press Ctrl-C here to end the run."
                                if composed else
                                "This scenario has its own file and decides when it ends; press Ctrl-C here to end the run.")
                    print("\nManual ego control. The driving window (manual_control.py) opens by itself once the scenario is "
                          "running.\nClick into its window and drive with W/A/S/D (Q reverse, Space handbrake).\n"
                          f"{end_note}\n"
                          "Do not use its Ctrl+R (own recorder), Ctrl+P (replay) or P (CARLA autopilot).\n")
                else:
                    print("\nManual ego control. When ScenarioRunner has spawned the ego, drive it from a second terminal.\n"
                          f"Start manual_control.py yourself with: {manual_control_command(args.host, args.port, args.tm_port)}; "
                          "press Ctrl-C here to end the run.\n")

            code = None
            interrupted = False
            reported = None  # ScenarioRunner's result (SUCCESS / FAILURE ...) of this run, from its output
            ended_by = None  # 'restart' / 'quit': the chase view's request that made this run end (its stop file was created)

            def start_view(line):
                nonlocal chase, chase_view
                if not chase_view or SCENARIO_RUNNING_LINE not in line:
                    return
                chase_view = False  # one attempt per session: the window stays open across restarts
                try:
                    chase = start_chase_view(args.host, args.port, control, args.view_spectator,
                                             run_dir / "chase_view.log")
                except OSError as e:  # includes FileNotFoundError
                    print(f"\nCould not start the chase view ({e}); the run goes on without it.\n")
                    return
                print("\nOpened the chase view (own console): R restarts the run, Esc quits, C cycles the view, H toggles the HUD. "
                      "The window stays open when the run ends. --no-view skips it.\n")

            def on_line(line):
                nonlocal reported
                if window is not None:
                    window.feed(line)
                start_view(line)
                reported = parse_result_line(line) or reported

            def on_request(request):
                """The chase view asked to restart or quit while the run is going: end it cleanly through the stop file."""
                nonlocal ended_by
                if stop_file is None:
                    print(f"\nChase view: {request} requested, but '{args.scenario}' has its own scenario file and cannot be "
                          "ended from outside; it is taken when the run has ended (or press Ctrl-C here).\n")
                    return
                try:
                    stop_file.parent.mkdir(parents=True, exist_ok=True)
                    stop_file.touch()
                except OSError as e:
                    print(f"\nChase view: {request} requested, but the stop file could not be written ({e}): "
                          "press Ctrl-C here to end the run.\n")
                    return
                ended_by = request
                print(f"\nChase view: {request} requested: ending the run (results are being saved)\n")

            request_done = threading.Event()  # set when the run is over: stops the watcher
            if control is not None:
                clear_result(control)  # the window shows the result of the run that just ended, never the one before
                threading.Thread(target=watch_requests, args=(control, on_request, request_done), name="chase-view-requests",
                                 daemon=True).start()

            def on_interrupt():
                nonlocal interrupted
                interrupted = True
                if window is not None:
                    window.mark_ending()  # the window exits when the ego is destroyed: that is not a request to end

            try:
                code = run_scenario_runner(sr_args, log_file=run_dir / "scenario_runner.log",
                                           on_line=on_line, on_interrupt=on_interrupt,
                                           metrics_file=metrics_file)
            except KeyboardInterrupt:
                interrupted = True
                raise
            finally:
                # The driving window would wait for an ego that no longer exists: close it first. A second Ctrl+C here
                # must not skip the reset and the run.json below.
                if window is not None:
                    try:
                        window.stop()
                    except KeyboardInterrupt:
                        print("Interrupted while closing the driving window; continuing with the cleanup.")
                request_done.set()
                # A stopped or killed ScenarioRunner leaves the server synchronous (frozen window) and its actors behind,
                # and with --record the recorder running
                try:
                    client = connect(args.host, args.port, args.timeout)
                    if record_log is not None:
                        client.stop_recorder()
                    # No Traffic Manager here: more runs may follow in this process (see the pre-run reset), and
                    # ScenarioRunner's ended with it (reset_world's default)
                    reset_world(client)
                except RuntimeError as e:
                    print(f"Could not reset the server, run apps/reset_world.py: {e}")
                info["exit_code"] = code
                if window is not None and window.stop_file_written:
                    info["end_reason"] = "stop_file"
                elif ended_by is not None and not interrupted:
                    info["end_reason"] = ended_by
                elif interrupted:
                    info["end_reason"] = "ctrl_c"
                else:
                    info["end_reason"] = "scenario_end"
                info["finished"] = datetime.now().astimezone().isoformat(timespec="seconds")
                if record_log is not None:
                    info["record_found"] = record_log.is_file()
                info["metrics"] = read_metrics(metrics_file)
                write_run_json(run_dir, info)
            summary = metrics_summary(info["metrics"]) if info["metrics"] is not None else None
            if summary is not None:
                print("\n" + summary)
            print(f"\nResults and log: {run_dir}")
            if record_log is not None:
                if record_log.is_file():
                    print(f"Recording: {record_log}\n"
                          "    replay it with: python -m uv run python apps\\replay.py")
                else:
                    print(f"Recording expected at {record_log}, but it was not written "
                          "(did the scenario load? is the server on this machine?)")

            # What next: a window that is open stays so; it asks for a restart (the same variant and settings, the map stays
            # loaded) or to quit. Without a window (or Ctrl-C) this was the only run.
            if control is None:
                break
            write_result(control, run_number, reported, summary, run_dir)
            step = next_step(pending_request(control), interrupted, chase is not None and chase.poll() is None)
            if step == "wait":
                print("\nThe chase view stays open: R restarts the run, Esc quits (or press Ctrl-C here).")
                try:
                    step = wait_for_request(control, lambda: chase.poll() is None)
                except KeyboardInterrupt:
                    step = "end"
            if step != "restart":
                break
            clear_request(control, RESTART_REQUEST)
            print("\nRestarting the run (the map stays loaded)...\n")
    finally:
        if control is not None:
            # The window closes itself on the quit request; the camera goes with it. Then the final reset
            write_request(control, QUIT_REQUEST)
            try:
                stop_process(chase, grace=5.0)
            except KeyboardInterrupt:  # Ctrl+C while waiting: do not wait any longer
                stop_process(chase)
            try:
                reset_world(connect(args.host, args.port, args.timeout))
            except RuntimeError as e:
                print(f"Could not reset the server, run apps/reset_world.py: {e}")
            shutil.rmtree(control, ignore_errors=True)
    sys.exit(code)


if __name__ == "__main__":
    main()
