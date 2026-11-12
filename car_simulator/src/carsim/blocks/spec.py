"""
Which blocks exist and which XML attributes each takes, with their types. Plain data with no CARLA or ScenarioRunner
imports, so scenario_files.validate() can read it without loading the blocks themselves.
"""

import math

# Blocks are built, spawned and run in this order
BLOCK_ORDER = ("ego_control", "spawning", "run_control", "occluder", "pedestrian", "crowd", "cyclists", "cut_in",
               "background", "end", "metrics")

# Blocks that exist even without an XML element (with all-default parameters): the ego always needs a driver
# (autopilot unless the XML says manual) and the scenario always needs an end condition
IMPLICIT_BLOCKS = ("ego_control", "end", "metrics")

# Blocks that use the pedestrian crossing (the occluder stands before it): apps/run_scenario.py runs its crossing pre-check for a variant that has
# one of these tags, so keep the names in step
CROSSING_BLOCKS = ("occluder", "pedestrian", "crowd", "cyclists")

# Attribute types: "count" a whole number >= 0, "positive" a finite number > 0 (speeds, distances, radius, times),
# "delay" a finite number >= 0 (a wait of 0 s is fine), "angle" a finite number from 0 to 180 (degrees),
# "text" anything (checked by the block itself, if at all)
# tag -> {attribute it reads from config.other_parameters[tag]: type}
PARAM_TYPES = {
    "ego_control": {"mode": "text", "target_speed": "positive", "brake": "text"},
    "pedestrian": {"model": "text", "age": "text", "speed": "positive", "trigger_distance": "positive",
                   "walk_in": "delay", "overlap": "delay",
                   "flow_radius": "positive"},
    # A vehicle stopped in the kerb lane just before the crosswalk, hiding it from the ego in the lane to its left
    "occluder": {"model": "text", "gap": "delay", "queue": "count", "prop": "text"},
    "crowd": {"count": "count", "min_speed": "positive", "max_speed": "positive", "max_delay": "delay",
              "trigger_distance": "positive", "flow_radius": "positive"},
    "cyclists": {"count": "count", "min_speed": "positive", "max_speed": "positive", "max_delay": "delay",
                 "trigger_distance": "positive"},
    "background": {"vehicles": "count", "motorcycles": "count", "bicycles": "count", "walkers": "count",
                   "radius": "positive"},
    # A parked car pulling out, or a cyclist changing lanes, in front of the ego; hidden by a vehicle until then
    "cut_in": {"kind": "text", "distance": "positive", "trigger_distance": "positive", "speed": "positive",
               "blocker": "text"},
    # Per-run measurements written for run.json (always on; no attributes)
    "metrics": {},
    "end": {"distance_after_crossing": "positive", "distance": "positive", "timeout": "positive"},
    # Where actors may appear and vanish while the scenario runs (see carsim.road.hidden_from_ego)
    "spawning": {"hide_distance": "positive", "behind_distance": "delay", "behind_angle": "angle",
                 "far_distance": "positive", "side_angle": "angle"},
    # Written by apps/run_scenario.py into its copy of the XML for a manual run with the driving window; not by hand
    "run_control": {"stop_file": "text"},
}

# tag -> attributes it reads from config.other_parameters[tag]
BLOCK_PARAMS = {tag: tuple(types) for tag, types in PARAM_TYPES.items()}

# (min_speed, max_speed) defaults of the blocks that have them, to compare a lone min_speed or max_speed against
SPEED_DEFAULTS = {"crowd": (1.1, 1.8), "cyclists": (3.0, 5.0)}

# Defaults of the <spawning> distances that must stay in order (see carsim.road.hidden_from_ego)
HIDE_DEFAULTS = {"hide_distance": 50.0, "far_distance": 120.0}

EGO_MODES = ("autopilot", "manual")
EGO_DEFAULT_MODE = "autopilot"  # the mode of a variant without an <ego_control> element
EGO_DEFAULT_TARGET_SPEED = 30.0  # km/h, the autopilot's speed when <ego_control> has no target_speed
# none: the autopilot never brakes; sight: it stops for crossers and brakes for what is in its path, as far as it
# can see them (carsim.blocks.driver). Ignored in manual mode
BRAKE_MODES = ("none", "sight")
WALKER_AGES = ("adult", "child")  # values of the walker blueprints' age attribute
CUT_IN_KINDS = ("parking", "cyclist")

# s, ComposedScenario's timeout for a manual-driving run: there is none to speak of (7 days). A manual run is endless:
# it ends when the stop file of <run_control> appears (the driving window was closed) or with Ctrl-C.
# ScenarioRunner's TimeOut needs a number, and a result writer that reports a duration under it still says SUCCESS.
MANUAL_DEFAULT_TIMEOUT = 7 * 24 * 3600

# ComposedScenario.terminate() creates this file next to the stop file when the run ends on its own terms, and
# apps/run_scenario.py's driving window watcher looks for it before it treats a window that closed as the user's request
# to end the run: unlike the "scenario ending" line of the output, which the watcher only sees once it has read it,
# the file is there as soon as the scenario has written it
ENDING_MARKER_SUFFIX = ".ending"


def ending_marker(stop_file):
    """The path of the marker file of the stop file `stop_file` (a path or a string)."""
    return str(stop_file) + ENDING_MARKER_SUFFIX


def _check_value(kind, value):
    """None if `value` (a string from the XML) is fine for `kind`, else what it should have been."""
    if kind == "text":
        return None
    try:
        number = float(value)
    except ValueError:
        return "a number"
    if not math.isfinite(number):
        return "a finite number"
    if kind == "count":
        return None if number >= 0 and number == int(number) else "a whole number, 0 or more"
    if kind == "delay":
        return None if number >= 0 else "a number, 0 or more"
    if kind == "angle":
        return None if 0 <= number <= 180 else "a number from 0 to 180"
    return None if number > 0 else "a number greater than 0"


def check_params(params, duplicates=(), other_actors=0):
    """
    Problems with the block elements of one variant, as a list of strings: unknown tags, unknown attributes, values
    that are not of the attribute's type (see PARAM_TYPES), min_speed above max_speed, a bad ego_control mode, a tag
    used more than once, and other_actor elements.

    `params` is Variant.params (tag -> attributes). `duplicates` are the tags that appear more than once in the
    variant (ScenarioRunner keeps only the last element of a tag, so the others would be ignored silently).
    `other_actors` is the number of <other_actor> elements, which ComposedScenario does not use.
    """
    problems = []
    for tag in duplicates:
        problems.append(f"<{tag}> is used more than once; ScenarioRunner keeps only the last one, so the others "
                        "would be ignored")
    if other_actors:
        problems.append(f"<other_actor> is ignored by ComposedScenario ({other_actors} found): it spawns what the "
                        "blocks ask for; use a block (crowd, cyclists, background) instead")
    for tag, attributes in params.items():
        if tag not in BLOCK_PARAMS:
            problems.append(f"unknown block <{tag}>; known blocks: {', '.join(BLOCK_ORDER)} "
                            "(weather, ego_vehicle and route are ScenarioRunner's own elements)")
            continue
        valid = {}
        for attribute, value in attributes.items():
            if attribute not in BLOCK_PARAMS[tag]:
                problems.append(f"<{tag}> has an unknown attribute '{attribute}'; known: "
                                f"{', '.join(BLOCK_PARAMS[tag])}")
                continue
            expected = _check_value(PARAM_TYPES[tag][attribute], value)
            if expected is not None:
                problems.append(f"<{tag}> {attribute}=\"{value}\" must be {expected}")
            elif PARAM_TYPES[tag][attribute] != "text":
                valid[attribute] = float(value)
        if tag in SPEED_DEFAULTS and ("min_speed" in valid or "max_speed" in valid):
            low = valid.get("min_speed", SPEED_DEFAULTS[tag][0])
            high = valid.get("max_speed", SPEED_DEFAULTS[tag][1])
            if low > high:
                both = "min_speed" in valid and "max_speed" in valid
                problems.append(f"<{tag}> min_speed {low:g} is above max_speed {high:g}"
                                + ("" if both else " (the one you did not set has its default)"))
    spawning = params.get("spawning")
    if spawning and ("hide_distance" in spawning or "far_distance" in spawning):
        try:
            hide = float(spawning.get("hide_distance", HIDE_DEFAULTS["hide_distance"]))
            far = float(spawning.get("far_distance", HIDE_DEFAULTS["far_distance"]))
        except ValueError:
            hide = far = 0.0  # already reported as not a number
        if 0.0 < far < hide:  # (a value that is not positive was reported above)
            problems.append(f"<spawning> far_distance {far:g} is below hide_distance {hide:g}"
                            + ("" if "far_distance" in spawning else " (the one you did not set has its default)"))
    mode = params.get("ego_control", {}).get("mode")
    if mode is not None and mode not in EGO_MODES:
        problems.append(f"<ego_control> mode '{mode}' must be one of: {', '.join(EGO_MODES)}")
    brake = params.get("ego_control", {}).get("brake")
    if brake is not None and brake not in BRAKE_MODES:
        problems.append(f"<ego_control> brake '{brake}' must be one of: {', '.join(BRAKE_MODES)}")
    pedestrian = params.get("pedestrian", {})
    if "age" in pedestrian:
        if pedestrian["age"] not in WALKER_AGES:
            problems.append(f"<pedestrian> age '{pedestrian['age']}' must be one of: {', '.join(WALKER_AGES)}")
        if "model" in pedestrian:
            problems.append("<pedestrian> has both model and age; age picks the model, so set only one")
    if "overlap" in pedestrian:
        try:
            overlap = float(pedestrian["overlap"])
        except ValueError:
            overlap = None  # already reported as not a number
        if overlap is not None and overlap > 100:
            problems.append(f"<pedestrian> overlap {overlap:g} must be a percentage from 0 to 100")
        if "trigger_distance" in pedestrian:
            problems.append("<pedestrian> has both overlap and trigger_distance; overlap starts the walk by time, "
                            "so set only one")
    if "cut_in" in params:
        kind = params["cut_in"].get("kind")
        if kind not in CUT_IN_KINDS:
            problems.append(f"<cut_in> kind {'is missing' if kind is None else repr(kind) + ' is unknown'}; "
                            f"it must be one of: {', '.join(CUT_IN_KINDS)}")
    return problems
