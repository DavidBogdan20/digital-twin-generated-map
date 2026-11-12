#!/usr/bin/env python
"""
Offline check of the composable scenario blocks, no CARLA server needed.

  - the block registry: BLOCK_ORDER, the parameter lists and the block classes agree, and the crossing blocks
    (occluder, pedestrian, crowd, cyclists; run_scenario.py's crossing pre-check uses them) are the ones that need
    the crossing
  - scenario_files: scenario_file() gives the shared composed file for a folder without a .py of its own, the folder's
    own .py otherwise; list_scenario_names() does not list _composed; validate() accepts the shipped scenarios and
    flags unknown block tags, unknown attributes, bad values (type, range, min_speed above max_speed), a bad ego
    mode, a bad brake, a bad age, age together with model, a missing or bad cut_in kind, a tag used twice and
    <other_actor>
  - a mocked build of ComposedScenario for every variant of pedestrian_crossing.xml: the real ScenarioRunner XML
    parser, BasicScenario and py_trees, on an offline map built from the OpenDRIVE file, with CarlaDataProvider and
    the world mocked (spawns are recorded, nothing is spawned anywhere). It prints the behavior tree and checks that
    every block was created, what was spawned (models, counts, bicycle blueprints only at the crosswalk, motorcycles
    only in traffic), the trigger distance, the timeout (a default that grows with the distance to drive) and the
    cleanup. Cyclists must be spawned on the sidewalk (physics off, not on a driving lane), be set down at their
    start with physics on, ride a planned distance that leaves room to stop, and ride on (RideOn) instead of being
    destroyed in view; the main pedestrian and every crowd walker have an AI controller and end in a WalkOn, and
    without a navigation mesh (Town12) in a WaitForever as before; a waiting spot whose
    spawn fails is retried further back, then skipped; a crowd walker whose spawn fails is retried further back;
    background bicycles get a Traffic Manager speed difference.
  - a mocked build of both occluded_crossing variants (the ego on an inner-lane start from find_candidates, not the
    XML's placeholder: the truck first, then the queue car, both on the kerb lane; the child's model has age child;
    the crowd and cyclists of the crowd variant) and of both cut_in variants (the ego on a start that fits the kind;
    the actors and the tree of each kind), all with the cautious driver of brake="sight"
  - a manual ego: the endless flows (pedestrians, cyclists, background bubble) replace the single pass, no distance
    end, a timeout of 7 days, no start trigger (also when <background> is the only flow: the trigger is decided before
    the behaviours are made); the run_control stop file ends the tree; no navigation mesh falls back to the single
    pass; validation of the new <spawning> (five numbers) and <run_control> elements
  - the behaviours of the flows against a fake world: tests/check_flows_offline.py

    python tests/check_blocks_offline.py

Exits with 1 if any check fails.
"""

import fnmatch
import importlib.util
import itertools
import os
import re
import sys
import tempfile
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))
sys.path.insert(0, str(ROOT / "apps"))

import carla  # noqa: E402
import py_trees  # noqa: E402

from check_crossings_offline import xodr_path  # noqa: E402
from find_crossing_spots import find_candidates  # noqa: E402
from carsim import blocks as carsim_blocks  # noqa: E402
from carsim import scenario_files  # noqa: E402
from carsim.blocks import spec  # noqa: E402
from carsim.paths import list_scenario_names  # noqa: E402
from carsim.road import (CYCLE_DECELERATION, HIDE_BEHIND_ANGLE, HIDE_BEHIND_DISTANCE, HIDE_DISTANCE,  # noqa: E402
                         HIDE_FAR_DISTANCE, HIDE_SIDE_ANGLE, HideRule, cyclist_speed_limit, find_sidewalk)
from carsim.scenario_files import COMPOSED_SCENARIO_FILE, config_file, scenario_file, validate, variants  # noqa: E402
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider  # noqa: E402
from srunner.scenariomanager.timer import TimeOut  # noqa: E402
from srunner.tools.scenario_parser import ScenarioConfigurationParser  # noqa: E402

FAILURES = []
SCENARIOS = ("pedestrian_crossing", "occluded_crossing", "cut_in")  # the scenarios that ship with the repository


def check(condition, message):
    if not condition:
        FAILURES.append(message)
        print("  FAIL " + message)
    return condition


# --- fakes ---------------------------------------------------------------------------------------------------------

# Vehicle blueprints of CARLA 0.9.16: (id, base_type, number_of_wheels), read from a running server. The 4-wheel
# vehicles are a sample, the two-wheelers are complete.
VEHICLES = [
    ("vehicle.tesla.model3", "car", 4), ("vehicle.audi.a2", "car", 4), ("vehicle.lincoln.mkz_2017", "car", 4),
    ("vehicle.mini.cooper_s", "", 4), ("vehicle.volkswagen.t2", "van", 4), ("vehicle.carlamotors.carlacola", "truck", 4),
    ("vehicle.carlamotors.european_hgv", "truck", 6),
    ("vehicle.harley-davidson.low_rider", "motorcycle", 2), ("vehicle.yamaha.yzf", "motorcycle", 2),
    ("vehicle.kawasaki.ninja", "motorcycle", 2), ("vehicle.vespa.zx125", "motorcycle", 2),
    ("vehicle.bh.crossbike", "bicycle", 2), ("vehicle.gazelle.omafiets", "bicycle", 2),
    ("vehicle.diamondback.century", "bicycle", 2),
]
BICYCLES = {v[0] for v in VEHICLES if v[1] == "bicycle"}
MOTORCYCLES = {v[0] for v in VEHICLES if v[1] == "motorcycle"}
WALKERS = ["walker.pedestrian.%04d" % i for i in range(1, 11)]
CHILDREN = {"walker.pedestrian.0009", "walker.pedestrian.0010"}  # the walkers with age child


class FakeAttribute:
    def __init__(self, value):
        self.value = value

    def as_str(self):
        return str(self.value)

    def as_int(self):
        return int(self.value)


class FakeBlueprint:
    def __init__(self, bp_id, **attributes):
        self.id = bp_id
        self.attributes = {k: FakeAttribute(v) for k, v in attributes.items()}

    def has_attribute(self, name):
        return name in self.attributes

    def get_attribute(self, name):
        return self.attributes[name]


class FakeLibrary:
    def __init__(self):
        self.blueprints = [FakeBlueprint(i, base_type=b, number_of_wheels=n) for i, b, n in VEHICLES]
        self.blueprints += [FakeBlueprint(i, age="child" if i in CHILDREN else "adult") for i in WALKERS] + [FakeBlueprint("controller.ai.walker")]
        self.blueprints += [FakeBlueprint("static.prop.busstop")]  # the occluder prop of the Child variant (occluder.py checks it exists)

    def filter(self, pattern):
        return [bp for bp in self.blueprints if fnmatch.fnmatch(bp.id, pattern)]

    def find(self, bp_id):
        return next(bp for bp in self.blueprints if bp.id == bp_id)


class FakeActor:
    """Enough of a carla.Actor for the behaviours' constructors."""
    ids = itertools.count(1000)

    def __init__(self, type_id, transform):
        self.id = next(self.ids)
        self.type_id = type_id
        self.transform = transform
        self.is_alive = True
        self.stopped = False
        self.physics = True
        self.destroy_count = 0

    def get_transform(self):
        return self.transform

    def get_location(self):
        return self.transform.location

    def get_control(self):
        return carla.WalkerControl() if self.type_id.startswith("walker") else carla.VehicleControl()

    def apply_control(self, control):
        pass

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def go_to_location(self, location):
        pass

    def set_max_speed(self, speed):
        pass

    def set_simulate_physics(self, enabled=True):
        self.physics = bool(enabled)

    def set_location(self, location):
        self.transform = carla.Transform(location, self.transform.rotation)

    def set_transform(self, transform):
        self.transform = transform

    def set_target_velocity(self, velocity):
        self.target_velocity = velocity

    def set_target_angular_velocity(self, velocity):
        pass

    def destroy(self):
        self.destroy_count += 1
        self.is_alive = False
        return True


class FakeSettings:
    def __init__(self, step):
        self.fixed_delta_seconds = step


class FakeTrafficManager:
    def __init__(self):
        self.speed_differences = []  # (actor, percentage)
        self.lane_changes = []  # (actor, enabled)

    def vehicle_percentage_speed_difference(self, actor, percentage):
        self.speed_differences.append((actor, percentage))

    def auto_lane_change(self, actor, enabled):
        self.lane_changes.append((actor, enabled))

    def update_vehicle_lights(self, actor, enabled):
        self.lights = getattr(self, "lights", []) + [(actor, enabled)]


class FakeClient:
    def __init__(self):
        self.manager = FakeTrafficManager()
        self.ports = []

    def get_trafficmanager(self, port):
        self.ports.append(port)
        return self.manager


class MapProxy:
    """
    The offline map with spawn points (an OpenDRIVE-only carla.Map has none): a sample of driving waypoints, the
    way the server's map would give lane-aligned vehicle spawn points.
    """

    def __init__(self, cmap, around, radius=250.0):
        self._map = cmap
        self.spawn_points = [wp.transform for wp in cmap.generate_waypoints(15.0)
                             if wp.transform.location.distance(around) <= radius]

    def get_spawn_points(self):
        return list(self.spawn_points)

    def __getattr__(self, name):
        return getattr(self._map, name)


class FakeWorld:
    def __init__(self, cmap, around, step=0.05):
        self.step = step  # fixed_delta_seconds of the settings; None like an asynchronous world
        self.client = FakeClient()
        self.library = FakeLibrary()
        self.debug = mock.MagicMock()
        self.ticks = 0
        self.controllers = []
        self.nav_points = []
        for wp in cmap.generate_waypoints(6.0):
            if wp.transform.location.distance(around) <= 150.0:
                sidewalk = find_sidewalk(wp, "right")
                if sidewalk is not None:
                    self.nav_points.append(sidewalk.transform.location)
        self.nav_index = 0

    def get_blueprint_library(self):
        return self.library

    def get_settings(self):
        return FakeSettings(self.step)

    def get_random_location_from_navigation(self):
        if not self.nav_points:
            return None
        self.nav_index = (self.nav_index * 7 + 3) % len(self.nav_points)
        return self.nav_points[self.nav_index]

    def spawn_actor(self, blueprint, transform, attach_to=None):
        controller = FakeActor(blueprint.id, transform)
        self.controllers.append(controller)
        return controller

    def tick(self):
        self.ticks += 1

    def wait_for_tick(self):
        self.ticks += 1

    def set_weather(self, weather):
        self.weather = weather

    def set_pedestrians_seed(self, seed):
        self.pedestrians_seed = seed


class Spawns:
    """CarlaDataProvider.request_new_actor, recorded. `fail` is a list of categories whose first N requests fail."""

    def __init__(self, fail=None, fail_at=()):
        self.requests = []  # dicts: model, category, transform, autopilot, tick, ok
        self.fail = dict(fail or {})
        self.fail_at = set(fail_at)  # indices (0-based) of requests that fail, whatever the category

    def __call__(self, model, spawn_point=None, rolename="scenario", autopilot=False, random_location=False,
                 color=None, actor_category="car", attribute_filter=None, tick=True):
        ok = True
        if self.fail.get(actor_category, 0) > 0:
            self.fail[actor_category] -= 1
            ok = False
        if len(self.requests) in self.fail_at:
            ok = False
        self.requests.append(dict(model=model, category=actor_category, transform=spawn_point, autopilot=autopilot,
                                  tick=tick, ok=ok))
        return FakeActor(model, spawn_point) if ok else None

    def models(self, category, ok_only=True):
        return [r["model"] for r in self.requests if r["category"] == category and (r["ok"] or not ok_only)]


def load_scenario_module(path):
    """The way ScenarioRunner loads it: by basename, with the file's folder at the front of sys.path."""
    sys.path.insert(0, str(path.parent))
    try:
        spec_ = importlib.util.spec_from_file_location(path.stem, path)
        module = importlib.util.module_from_spec(spec_)
        sys.modules[path.stem] = module
        spec_.loader.exec_module(module)
    finally:
        sys.path.pop(0)
    return module


def build(module, variant_name, xml_file, cmap, fail=None, edit=None, step=0.05, fail_at=(), nav=True):
    """
    Build ComposedScenario for one variant with everything CARLA mocked. Returns (scenario, world, spawns, config);
    world.client is the fake client whose Traffic Manager records the speed differences.
    """
    CarlaDataProvider.get_random_seed().seed(2000)  # the same random layout in every build
    config = ScenarioConfigurationParser.parse_scenario_configuration(variant_name, str(xml_file))[0]
    if edit is not None:
        edit(config)
    ego_loc = config.trigger_points[0].location
    around = ego_loc
    world = FakeWorld(cmap, around, step)
    if not nav:
        world.nav_points = []  # like Town12
    spawns = Spawns(fail, fail_at)
    ego = FakeActor("vehicle.lincoln.mkz_2017", config.ego_vehicles[0].transform)
    proxy = MapProxy(cmap, around)
    patches = [
        mock.patch.object(CarlaDataProvider, "get_map", staticmethod(lambda: proxy)),
        mock.patch.object(CarlaDataProvider, "get_world", staticmethod(lambda: world)),
        mock.patch.object(CarlaDataProvider, "get_client", staticmethod(lambda: world.client)),
        mock.patch.object(CarlaDataProvider, "request_new_actor", staticmethod(spawns)),
        mock.patch.object(CarlaDataProvider, "is_sync_mode", staticmethod(lambda: True)),
        mock.patch.object(CarlaDataProvider, "is_runtime_init_mode", staticmethod(lambda: False)),
    ]
    for patch in patches:
        patch.start()
    try:
        scenario = module.ComposedScenario(world, [ego], config, debug_mode=False)
    finally:
        for patch in patches:
            patch.stop()
    return scenario, world, spawns, config


def tree_text(root):
    return py_trees.display.ascii_tree(root)


def nodes(root):
    return list(root.iterate())


def node_names(root):
    return [node.name for node in nodes(root)]


# --- checks --------------------------------------------------------------------------------------------------------

def check_registry():
    print("block registry")
    classes = carsim_blocks.block_classes()
    check(tuple(classes) == spec.BLOCK_ORDER, "block_classes() is not in BLOCK_ORDER")
    check(set(spec.BLOCK_PARAMS) == set(spec.BLOCK_ORDER), "BLOCK_PARAMS and BLOCK_ORDER have different tags")
    for tag, cls in classes.items():
        check(cls.tag == tag, f"{cls.__name__}.tag is {cls.tag!r}, registered as {tag!r}")
    check(spec.BLOCK_ORDER == ("ego_control", "spawning", "run_control", "occluder", "pedestrian", "crowd", "cyclists",
                               "cut_in", "background", "end", "metrics"), "BLOCK_ORDER changed")
    check(spec.BLOCK_ORDER[-1] == "metrics" and "metrics" in spec.IMPLICIT_BLOCKS,
          "metrics must be the last block and an implicit one (it exists without an XML element)")
    check(spec.IMPLICIT_BLOCKS == ("ego_control", "end", "metrics"), f"IMPLICIT_BLOCKS {spec.IMPLICIT_BLOCKS}")
    check(spec.PARAM_TYPES["metrics"] == {} and spec.BLOCK_PARAMS["metrics"] == (), "metrics takes no attributes")
    check(not classes["metrics"].needs_crossing, "metrics must not need the crossing")
    check(spec.check_params({"metrics": {"x": "1"}}) != [], "an attribute on <metrics> was accepted")
    check(set(spec.CROSSING_BLOCKS) == {tag for tag, cls in classes.items() if cls.needs_crossing},
          "CROSSING_BLOCKS and the blocks with needs_crossing differ")
    check(set(spec.CROSSING_BLOCKS) == {"occluder", "pedestrian", "crowd", "cyclists"},
          "the crossing tags must be occluder, pedestrian, crowd, cyclists (run_scenario.py's pre-check)")
    check(not classes["cut_in"].needs_crossing, "cut_in needs the crossing")
    try:
        carsim_blocks.create_blocks(mock.MagicMock(), {"nonsense": {}})
        check(False, "create_blocks accepted an unknown tag")
    except ValueError as error:
        check("nonsense" in str(error), "unknown tag error does not name the tag")


def check_files():
    print("scenario files")
    names = list_scenario_names()
    check("_composed" not in names, "list_scenario_names() lists _composed")
    check(all(n in names for n in SCENARIOS), f"scenarios listed: {names}")
    for name in SCENARIOS:
        check(scenario_file(name) == COMPOSED_SCENARIO_FILE, f"{name}: scenario_file is not the composed file")
        check(validate(name) == [], f"{name}: validate: {validate(name)}")
        check(not list((ROOT / "scenarios" / name).glob("*.py")), f"{name} has a .py of its own")
    check(not (ROOT / "scenarios" / "pedestrian_crossing" / "junction_pedestrian_crossing.py").exists(),
          "junction_pedestrian_crossing.py still exists")
    all_names = [v.name for n in names for v in variants(n)]
    check(len(all_names) == len(set(all_names)), "variant names collide between scenarios")
    check("PedestrianCrossing_Town10HD_1" in all_names, "variant PedestrianCrossing_Town10HD_1 missing")

    # The new elements: <spawning> (the hide rule) and <run_control> (the stop file, written by run_scenario.py)
    check(spec.check_params({"spawning": {"hide_distance": "50", "behind_distance": "15", "behind_angle": "110"},
                             "run_control": {"stop_file": "C:\\runs\\one\\stop"},
                             "pedestrian": {"flow_radius": "60"}, "crowd": {"count": "3", "flow_radius": "40"}}) == [],
          "<spawning>, <run_control> or flow_radius reported as problems")
    check(spec.check_params({"spawning": {"behind_distance": "0", "behind_angle": "180"}}) == [],
          "behind_distance 0 or behind_angle 180 flagged")
    problems = spec.check_params({"spawning": {"hide_distance": "0", "behind_distance": "-1", "behind_angle": "200",
                                               "colour": "red"},
                                  "run_control": {"stop_file": "x", "other": "1"}})
    for what in ('<spawning> hide_distance="0"', '<spawning> behind_distance="-1"', '<spawning> behind_angle="200"',
                 "'colour'", "'other'"):
        check(any(what in p for p in problems), f"{what} not reported: {problems}")
    check(len(problems) == 5, f"expected 5 problems for <spawning>/<run_control>, got {problems}")
    # far_distance and side_angle of <spawning>; the order of hide_distance and far_distance
    check(spec.check_params({"spawning": {"far_distance": "150", "side_angle": "45", "hide_distance": "50"}}) == [],
          "far_distance / side_angle reported as problems")
    check(spec.check_params({"spawning": {"side_angle": "0"}}) == [] and spec.check_params({"spawning": {"side_angle": "180"}}) == [],
          "side_angle 0 or 180 flagged")
    problems = spec.check_params({"spawning": {"far_distance": "0", "side_angle": "181"}})
    check(len(problems) == 2 and any("far_distance" in p for p in problems) and any("side_angle" in p for p in problems),
          f"bad far_distance / side_angle: {problems}")
    problems = spec.check_params({"spawning": {"far_distance": "40"}})
    check(len(problems) == 1 and "far_distance 40 is below hide_distance 50" in problems[0], f"far below default hide: {problems}")
    problems = spec.check_params({"spawning": {"hide_distance": "200"}})
    check(len(problems) == 1 and "far_distance 120 is below hide_distance 200" in problems[0]
          and "default" in problems[0], f"hide above default far: {problems}")
    check(spec.check_params({"spawning": {"hide_distance": "200", "far_distance": "300"}}) == [], "far above hide flagged")
    check(spec.HIDE_DEFAULTS == {"hide_distance": HIDE_DISTANCE, "far_distance": HIDE_FAR_DISTANCE},
          "spec.HIDE_DEFAULTS differs from carsim.road")
    check(HideRule() == HideRule(HIDE_DISTANCE, HIDE_BEHIND_DISTANCE, HIDE_BEHIND_ANGLE, HIDE_FAR_DISTANCE, HIDE_SIDE_ANGLE)
          and (HIDE_DISTANCE, HIDE_FAR_DISTANCE, HIDE_SIDE_ANGLE, HIDE_BEHIND_DISTANCE, HIDE_BEHIND_ANGLE)
          == (50.0, 120.0, 60.0, 15.0, 110.0), "the default hide rule is not 50, 120, 60, 15, 110")

    problems = spec.check_params({"ego_control": {"mode": "fly"}, "crowd": {"cnt": "3"}, "weird": {}, "end": {}})
    check(any("weird" in p for p in problems), "unknown tag not reported")
    check(any("cnt" in p for p in problems), "unknown attribute not reported")
    check(any("fly" in p for p in problems), "bad ego mode not reported")
    check(spec.check_params({"ego_control": {"mode": "manual"}, "cyclists": {"count": "2"}}) == [],
          "good parameters reported as problems")
    # The cautious driver and the occluded crossing / cut-in blocks
    check(spec.check_params({"ego_control": {"mode": "autopilot", "brake": "sight"}, "pedestrian": {"age": "child"},
                             "occluder": {"model": "vehicle.carlamotors.european_hgv", "gap": "1.5", "queue": "1"},
                             "cut_in": {"kind": "cyclist", "distance": "25", "trigger_distance": "12", "speed": "4"}}) == [],
          "the occluder, cut_in, brake or age parameters reported as problems")
    for what, bad in (("brake 'maybe'", {"ego_control": {"brake": "maybe"}}), ("age 'old'", {"pedestrian": {"age": "old"}}),
                      ("age with model", {"pedestrian": {"age": "child", "model": "walker.pedestrian.0001"}}),
                      ("missing kind", {"cut_in": {"distance": "30"}}), ("kind 'bus'", {"cut_in": {"kind": "bus"}}),
                      ("occluder queue -1", {"occluder": {"queue": "-1"}}),
                      ("cut_in speed 0", {"cut_in": {"kind": "parking", "speed": "0"}})):
        check(len(spec.check_params(bad)) == 1, f"{what}: expected one problem, got {spec.check_params(bad)}")
    check(spec.check_params({"cyclists": {"count": "4", "min_speed": "3.0", "max_speed": "5", "max_delay": "0",
                                          "trigger_distance": "25"},
                             "end": {"distance_after_crossing": "30", "timeout": "90"},
                             "background": {"vehicles": "0", "radius": "120"}}) == [],
          "good typed parameters reported as problems")
    bad = {"crowd": {"count": "2.5"}, "cyclists": {"count": "-1", "min_speed": "0", "max_speed": "fast"},
           "background": {"vehicles": "many", "radius": "-5"}, "end": {"distance": "nan", "timeout": "inf"},
           "pedestrian": {"speed": "-1.4", "trigger_distance": "0"}, "ego_control": {"target_speed": "0"}}
    problems = spec.check_params(bad)
    for what in ('<crowd> count="2.5"', '<cyclists> count="-1"', '<cyclists> min_speed="0"',
                 '<cyclists> max_speed="fast"', '<background> vehicles="many"', '<background> radius="-5"',
                 '<end> distance="nan"', '<end> timeout="inf"', '<pedestrian> speed="-1.4"',
                 '<pedestrian> trigger_distance="0"', '<ego_control> target_speed="0"'):
        check(any(what in p for p in problems), f"bad value {what} not reported: {problems}")
    check(len(problems) == 11, f"expected 11 value problems, got {len(problems)}: {problems}")
    problems = spec.check_params({"crowd": {"min_speed": "2.0", "max_speed": "1.0"}})
    check(len(problems) == 1 and "min_speed 2 is above max_speed 1" in problems[0], f"min above max: {problems}")
    problems = spec.check_params({"cyclists": {"min_speed": "6"}})  # the default max_speed is 5
    check(len(problems) == 1 and "min_speed 6 is above max_speed 5" in problems[0], f"min above default: {problems}")
    check(spec.check_params({"crowd": {"min_speed": "1.5", "max_speed": "1.5"}}) == [], "equal min and max flagged")
    check(spec.check_params({"crowd": {"max_delay": "0"}}) == [], "max_delay 0 flagged")
    problems = spec.check_params({"crowd": {"count": "2"}}, duplicates=("crowd",), other_actors=2)
    check(any("<crowd> is used more than once" in p for p in problems), f"duplicate tag not reported: {problems}")
    check(any("<other_actor> is ignored by ComposedScenario" in p for p in problems)
          and not any("unknown block" in p for p in problems), f"other_actor not reported clearly: {problems}")


def check_overlap_validation():
    """<pedestrian overlap>: a percentage, and not together with trigger_distance."""
    print("pedestrian overlap validation")
    check(spec.check_params({"pedestrian": {"overlap": "50"}}) == [], "overlap 50 was rejected")
    check(spec.check_params({"pedestrian": {"overlap": "0"}}) == [], "overlap 0 was rejected")
    check(spec.check_params({"pedestrian": {"overlap": "100"}}) == [], "overlap 100 was rejected")
    problems = spec.check_params({"pedestrian": {"overlap": "150"}})
    check(len(problems) == 1 and "overlap" in problems[0], f"overlap 150 not reported: {problems}")
    problems = spec.check_params({"pedestrian": {"overlap": "-5"}})
    check(len(problems) == 1 and "overlap" in problems[0], f"overlap -5 not reported: {problems}")
    problems = spec.check_params({"pedestrian": {"overlap": "abc"}})
    check(len(problems) == 1 and "overlap" in problems[0], f"overlap abc not reported once: {problems}")
    problems = spec.check_params({"pedestrian": {"overlap": "50", "trigger_distance": "47"}})
    check(len(problems) == 1 and "overlap and trigger_distance" in problems[0], f"both not reported: {problems}")
    check(spec.check_params({"pedestrian": {"trigger_distance": "47"}}) == [], "trigger_distance alone was rejected")


def check_validate_in_temp_folders():
    """validate() on made-up scenario folders: bad blocks in a composed one, a custom scenario with its own .py."""
    print("validate() on temporary scenario folders")
    composed_xml = """<scenarios>
        <scenario name="TempBlocks_Town10HD_1" type="ComposedScenario" town="Town10HD_Opt">
            <ego_vehicle x="1" y="2" z="0.3" yaw="0"/>
            <ego_control mode="fly"/>
            <cyclists count="2" colour="red"/>
            <unicorn/>
            <weather cloudiness="20"/>
        </scenario></scenarios>"""
    values_xml = """<scenarios>
        <scenario name="TempValues_Town10HD_1" type="ComposedScenario" town="Town10HD_Opt">
            <ego_vehicle x="1" y="2" z="0.3" yaw="0"/>
            <other_actor x="5" y="6" z="0.3" yaw="0" model="vehicle.audi.a2"/>
            <crowd count="5"/>
            <crowd count="3" max_speed="1.0" min_speed="2.0"/>
            <cyclists count="two"/>
            <weather cloudiness="20"/>
            <weather cloudiness="30"/>
        </scenario></scenarios>"""
    custom_xml = """<scenarios>
        <scenario name="TempCustom_Town10HD_1" type="TempCustomScenario" town="Town10HD_Opt">
            <ego_vehicle x="1" y="2" z="0.3" yaw="0"/>
            <whatever_the_class_wants a="1"/>
        </scenario></scenarios>"""
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        for folder, xml in (("temp_blocks", composed_xml), ("temp_custom", custom_xml), ("temp_two", custom_xml),
                            ("temp_values", values_xml)):
            (root / folder).mkdir()
            (root / folder / f"{folder}.xml").write_text(xml, encoding="utf-8")
        source = "class TempCustomScenario:\n    pass\n"
        (root / "temp_custom" / "temp_custom_scenario.py").write_text(source, encoding="utf-8")
        for stem in ("one", "two"):
            (root / "temp_two" / f"{stem}.py").write_text(source, encoding="utf-8")
        with mock.patch.object(scenario_files, "scenario_dir", lambda name: root / name):
            check(scenario_files.scenario_file("temp_blocks") == COMPOSED_SCENARIO_FILE, "no .py: composed file")
            problems = validate("temp_blocks")
            check(any("<unicorn>" in p for p in problems), f"unknown block not reported: {problems}")
            check(any("colour" in p for p in problems), f"unknown attribute not reported: {problems}")
            check(any("fly" in p for p in problems), f"bad ego mode not reported: {problems}")
            check(len(problems) == 3, f"expected 3 problems, got {problems}")
            problems = validate("temp_values")
            check(any("<crowd> is used more than once" in p for p in problems), f"duplicate not reported: {problems}")
            check(any("<other_actor> is ignored by ComposedScenario" in p for p in problems),
                  f"other_actor not reported: {problems}")
            check(any('<cyclists> count="two"' in p for p in problems), f"bad count not reported: {problems}")
            check(any("min_speed 2 is above max_speed 1" in p for p in problems), f"speeds not reported: {problems}")
            check(not any("unknown block" in p for p in problems), f"other_actor called an unknown block: {problems}")
            check(not any("weather" in p for p in problems), f"weather (ScenarioRunner's own) reported: {problems}")
            check(len(problems) == 4, f"expected 4 problems, got {problems}")
            variant = variants("temp_values")[0]
            check(variant.duplicates == ("crowd",) and variant.other_actors == 1,
                  f"Variant.duplicates {variant.duplicates} / other_actors {variant.other_actors}")
            check(scenario_files.scenario_file("temp_custom") == root / "temp_custom" / "temp_custom_scenario.py",
                  "a folder with its own .py must use it")
            check(validate("temp_custom") == [], f"custom scenario: {validate('temp_custom')}")
            try:
                scenario_files.scenario_file("temp_two")
                check(False, "two .py files in a folder were accepted")
            except FileNotFoundError:
                pass


def check_cyclist_plan(block, tree, spawns, cmap):
    """The cyclists wait on the sidewalk, are set down on a driving lane, ride a planned distance, stop and are removed."""
    step = block.step
    bikes = block.bikes
    check(step == 0.05, f"cyclists' step {step}")
    # Spawned on the sidewalk: not on a driving lane, physics off, beyond the kerb on the cycle line, nothing hidden
    requests = [r for r in spawns.requests if r["category"] == "bicycle" and not r["autopilot"]]
    check(len(requests) >= len(bikes), "fewer bicycle spawn requests than bicycles")
    for bike, delay, speed, distance, start in bikes:
        wait = block.waits[bike.id]
        check(not bike.physics, "a cyclist's physics is on while it waits")
        check(cmap.get_waypoint(bike.transform.location, project_to_road=False,
                                lane_type=carla.LaneType.Driving) is None,
              "a waiting cyclist is on a driving lane")
        lift = block.ctx.surface(wait.location).z - wait.location.z  # a deformed map's road mesh (ctx.surface)
        check(abs(bike.transform.location.z - (wait.location.z + 0.05 + lift)) < 1e-3,
              f"a waiting cyclist is not set down on its spot: z {bike.transform.location.z:.2f} vs {wait.location.z:.2f}")
        check(bike.transform.location.distance(start.location) < 12.0 and bike.transform.location.z < start.location.z + 1.0,
              "a waiting cyclist is far from its start")
        # It faces the way it will ride
        check(abs((bike.transform.rotation.yaw - start.rotation.yaw + 180.0) % 360.0 - 180.0) < 1.0,
              "a waiting cyclist does not face its crossing direction")
    for r in requests:
        check(cmap.get_waypoint(r["transform"].location, project_to_road=False,
                                lane_type=carla.LaneType.Driving) is None,
              "a cyclist spawn request is on a driving lane")
    # Behaviour chain per bicycle: Idle, set down (physics on), KeepVelocity (forced speed), RideOn (no stop: roll on to the
    # far sidewalk, ride on along it, removed once hidden: nothing is parked or destroyed in view)
    chains = [node for node in nodes(tree) if node.name.startswith("Cyclist")]
    check(len(chains) == len(bikes), f"{len(chains)} cyclist chains for {len(bikes)} bicycles")
    for node, (bike, delay, speed, distance, start) in zip(chains, bikes):
        kinds = [type(c).__name__ for c in node.children]
        check(kinds == ["Idle", "ActorTransformSetter", "KeepVelocity", "RideOn"], f"{node.name}: {kinds}")
        if len(node.children) < 4:
            continue
        idle, setter, ride, leave = node.children[:4]
        check(setter._actor is bike and ride._actor is bike and leave.bike.actor is bike,
              f"{node.name}: behaviours for different actors")
        check(leave.bike in leave.flow.bikes and leave.bike.lane.exit is not None,
              f"{node.name}: no far sidewalk spot to ride on to, or the bicycle is not in the helper flow")
        check(leave.bike.lane is block.records[bike.id].lane and leave.flow is block.leaver, f"{node.name}: RideOn state")
        # The single pass never parks the bicycle: it rides on along the far sidewalk, off the driving lanes (so off the ego's route lanes)
        check(len(leave.route) >= 2 and leave.bike.glide >= 2.0, f"{node.name}: no run along the far sidewalk ({len(leave.route)} points)")
        for point in leave.route:  # the ego's route lanes are driving lanes
            check(cmap.get_waypoint(point, project_to_road=False, lane_type=carla.LaneType.Driving) is None,
                  f"{node.name}: a run point {point} is on a driving lane")
        check(setter._physics is True, f"{node.name}: the set-down does not switch physics on")
        check(setter._transform is start, f"{node.name}: set down somewhere else than the start")
        check(cmap.get_waypoint(setter._transform.location, project_to_road=False,
                                lane_type=carla.LaneType.Driving) is not None,
              f"{node.name}: the start is not on a driving lane")
        check(ride._forced_speed is True, f"{node.name}: KeepVelocity without force_speed")
        check(3.0 - 1e-9 <= speed <= 5.0 + 1e-9 and delay >= 0.0 and distance > 1.0,
              f"{node.name}: delay {delay} / speed {speed} / distance {distance}")
        # The push stops a step late and the brake needs room: together they stay within the ride
        planned = ride._target_distance
        stopping = planned + speed * step + speed ** 2 / (2.0 * CYCLE_DECELERATION)
        check(0.0 < planned and stopping <= distance + 1e-6,
              f"{node.name}: forced ride {planned:.2f} m + overshoot + braking = {stopping:.2f} m of {distance:.2f} m")
        check(speed <= max(cyclist_speed_limit(distance, step), 0.0) + 1e-9, f"{node.name}: speed above the cap")

def check_no_double_destroy(block, scenario):
    """RideOn removes a bicycle through the context (out of the pool); the scenario's own removal must then skip it."""
    pool = CarlaDataProvider._carla_actor_pool
    bikes = [bike for bike, *_ in block.bikes]
    for bike in bikes:
        pool[bike.id] = bike
    try:
        gone = bikes[0]
        record = block.records[gone.id]
        check(sum(1 for node in nodes(scenario.scenario_tree) if node.name.startswith("RideOn") and node.bike is record) == 1,
              "not one RideOn node for the bicycle")
        check(not any(type(node).__name__ == "ActorDestroy" and node._actor is gone for node in nodes(scenario.scenario_tree)),
              "a bicycle is still destroyed by an ActorDestroy node (in view)")
        block.leaver._remove(record)  # what RideOn does once the bicycle is hidden
        check(gone.destroy_count == 1 and gone.id not in pool, "the removal did not destroy the bicycle once")
        check(gone not in scenario.other_actors, "the removed bicycle is still in the scenario's actor list")
        check(record not in block.leaver.bikes, "RideOn would go on with a removed bicycle")
        # A second removal of the same id is harmless (prints a note, returns None)
        check(CarlaDataProvider.remove_actor_by_id(gone.id) is None, "removing a removed actor id did not return None")
        check(gone.destroy_count == 1, "a removed bicycle was destroyed again by remove_actor_by_id")
        scenario.remove_all_actors()  # must skip the removed one, and remove the others once each
        check(all(bike.destroy_count == 1 for bike in bikes),
              f"destroy counts after remove_all_actors: {[b.destroy_count for b in bikes]}")
    finally:
        for bike in bikes:
            pool.pop(bike.id, None)
        scenario.other_actors = []


def check_walk_on(block_ped, block_crowd, tree, world, nav):
    """The main pedestrian and every crowd walker: an AI controller, the walk across that ends off the line, a WalkOn."""
    names = node_names(tree)
    adversary = next((n for n in nodes(tree) if n.name == "Adversary"), None)
    ped_kinds = [type(c).__name__ for c in adversary.children] if adversary is not None else []
    if block_ped is not None:
        check(ped_kinds[-1] == ("WalkOn" if nav else "WaitForever"), f"the pedestrian's chain ends in {ped_kinds}")
        check(("WalkOn" in names) == nav, "WalkOn nodes without (or missing with) a navigation mesh")
        check((block_ped.controller is not None) == nav, "the main pedestrian's controller")
        if nav:
            walk = next(c for c in adversary.children if c.name == "WalkAcross")
            check(walk._line is not None, "the walk across has no off-line guard")
            node = adversary.children[-1]
            check(node.walker is block_ped.walker and node.controller is block_ped.controller and node.speed == block_ped.speed,
                  "WalkOn of the main pedestrian: wrong walker, controller or speed")
            check(not getattr(block_ped.controller, "started", False), "the pedestrian's controller is started at load")
            check(block_ped.controller in world.controllers, "the pedestrian's controller is not in the world")
    if block_crowd is not None:
        for i, (walker, delay, speed, distance, spot, controller) in enumerate(block_crowd.walkers):
            chain = next(n for n in nodes(tree) if n.name == "Crowd{}".format(i + 1))
            kinds = [type(c).__name__ for c in chain.children]
            check(kinds == (["Idle", "WalkLine", "WalkOn"] if nav else ["Idle", "WalkLine", "WaitForever"]),
                  f"crowd chain {kinds}")
            check((controller is not None) == nav, "a crowd walker's controller")
            if nav:
                check(chain.children[1]._line is not None, "a crowd walk without the off-line guard")
                check(chain.children[2].walker is walker and chain.children[2].controller is controller,
                      "WalkOn of a crowd walker: wrong walker or controller")


def check_variant(module, name, variant, cmap):
    xml_file = config_file(name)
    print(f"mocked build: {name} / {variant.name}")
    scenario, world, spawns, config = build(module, variant.name, xml_file, cmap)
    params = config.other_parameters
    by_tag = {block.tag: block for block in scenario._blocks}

    expected = [tag for tag in spec.BLOCK_ORDER if tag in params or tag in spec.IMPLICIT_BLOCKS]
    check([b.tag for b in scenario._blocks] == expected, f"blocks {[b.tag for b in scenario._blocks]} != {expected}")
    check(all(tag in by_tag for tag in params), "a block of the XML was not created")

    tree = scenario.scenario_tree
    names = node_names(tree)
    text = tree_text(tree)
    print(text)
    check(scenario.behavior_tree.name == "ComposedScenario", f"tree name {scenario.behavior_tree.name!r}")
    check(config.weather is not None and world.weather is config.weather, "weather not applied")

    uses_crossing = any(tag in params for tag in spec.CROSSING_BLOCKS)
    check(scenario._ctx.uses_crossing == uses_crossing, "uses_crossing wrong")
    check(("WaitForEgo" in names) == uses_crossing, "WaitForEgo presence")
    check(("EgoPassedCrossing" in names) == uses_crossing, "EgoPassedCrossing presence")
    brake = params.get("ego_control", {}).get("brake")
    driver = "EgoAutopilot" if brake is None else "EgoCautiousDriver"
    check(driver in names, f"{driver} missing in autopilot mode")
    check(names.count("MetricsSampler") == 1 and "metrics" in by_tag, "the metrics sampler is not in the tree once")
    speed = float(params.get("ego_control", {}).get("target_speed", 30.0)) / 3.6
    want = max(60.0, scenario._ctx.total_drive_distance() / speed * 1.5 + 20.0
               + (module.CAUTIOUS_WAIT if brake == "sight" else 0.0))
    explicit = params.get("end", {}).get("timeout")
    check("TimeOut" in names and abs(scenario.timeout - (float(explicit) if explicit else want)) < 1e-6,
          f"timeout {scenario.timeout}, expected {float(explicit) if explicit else want}")
    check("CollisionTest" in node_names(scenario.criteria_tree), "CollisionTest criterion missing")

    # Trigger distance: pedestrian, else the first block that sets one, else 25
    want = 25.0
    for tag in ("pedestrian", "crowd", "cyclists"):
        if tag in params and "trigger_distance" in params[tag]:
            want = float(params[tag]["trigger_distance"])
            break
    check(scenario._ctx.trigger_distance == want, f"trigger distance {scenario._ctx.trigger_distance} != {want}")

    ego_start = scenario._ctx.ego_start_wp.transform.location
    # --- the main pedestrian
    if "pedestrian" in params:
        check(spawns.models("pedestrian")[0] == params["pedestrian"].get("model", "walker.pedestrian.0001"),
              "main pedestrian model")
        check(spawns.requests[0]["tick"] is True and spawns.requests[0]["category"] == "pedestrian",
              "the main pedestrian must spawn first and with its own tick")
        check("Adversary" in names and "RemovePedestrian" in names, "pedestrian nodes missing")
        check(next(n for n in nodes(tree) if n.name == "RemovePedestrian")._actor is by_tag["pedestrian"].walker,
              "the cleanup does not remove the main pedestrian")
        check(scenario.other_actors[0] is by_tag["pedestrian"].walker, "main pedestrian is not other_actors[0]")
    else:
        check("Adversary" not in names and "RemovePedestrian" not in names, "pedestrian nodes without the block")
    # --- crowd
    if "crowd" in params:
        wanted = int(params["crowd"]["count"])
        placed = len(by_tag["crowd"].walkers)
        check(wanted - 2 <= placed <= wanted, f"crowd placed {placed} of {wanted}")
        check(sum(1 for n in names if n.startswith("Crowd")) == placed, "crowd behaviours != crowd walkers")
        check(all(not r["tick"] for r in spawns.requests[1:] if r["category"] == "pedestrian"),
              "scenery pedestrians must spawn without a tick of their own")
    # --- cyclists
    if "cyclists" in params:
        wanted = int(params["cyclists"]["count"])
        bikes = spawns.models("bicycle")
        crossing_bikes = len(by_tag["cyclists"].bikes)
        check(wanted - 2 <= crossing_bikes <= wanted, f"cyclists placed {crossing_bikes} of {wanted}")
        check(sum(1 for n in names if n.startswith("Cyclist")) == crossing_bikes, "cyclist behaviours != bikes")
        check(all(m in BICYCLES for m in bikes), f"a non-bicycle blueprint used for bicycles: {set(bikes) - BICYCLES}")
        check(not any(m in MOTORCYCLES for r in spawns.requests if r["category"] == "bicycle" and not r["autopilot"]
                      for m in [r["model"]]), "a motorcycle was spawned at the crosswalk")
        check_cyclist_plan(by_tag["cyclists"], tree, spawns, cmap)
    else:
        check(not any(n.startswith("Cyclist") for n in names), "cyclist nodes without the block")
    # --- background
    if "background" in params:
        bg = params["background"]
        cars = [r for r in spawns.requests if r["category"] == "car"]
        motos = [r for r in spawns.requests if r["category"] == "motorbike"]
        traffic_bikes = [r for r in spawns.requests if r["category"] == "bicycle" and r["autopilot"]]
        for label, got, key in (("cars", cars, "vehicles"), ("motorcycles", motos, "motorcycles"),
                                ("bicycles", traffic_bikes, "bicycles")):
            want_n = int(bg.get(key, 0))
            check(len(got) == want_n, f"background {label}: {len(got)} spawned, {want_n} wanted")
            check(all(r["autopilot"] and not r["tick"] for r in got), f"background {label}: autopilot / tick=False")
        check(all(r["model"] in MOTORCYCLES for r in motos), "background motorcycle models")
        check(all(r["model"] in BICYCLES for r in traffic_bikes), "background bicycle models")
        check(all(r["model"] not in MOTORCYCLES | BICYCLES for r in cars), "background car models are two-wheelers")
        check(all(dict((m, n) for m, _, n in [(v[0], v[1], v[2]) for v in VEHICLES])[r["model"]] == 4 for r in cars),
              "background cars are not 4-wheel")
        route = scenario._ctx.route_lanes()
        for r in cars + motos + traffic_bikes:
            wp = cmap.get_waypoint(r["transform"].location)
            check((wp.road_id, wp.lane_id) not in route, "background vehicle spawned in an ego lane")
            check(r["transform"].location.distance(ego_start) >= 25.0, "background vehicle within 25 m of the ego")
        # The background walkers' controllers are started; the crossing pedestrians' wait for their walk on
        walking_on = (1 if "pedestrian" in params else 0) + (len(by_tag["crowd"].walkers) if "crowd" in params else 0)
        started = [c for c in world.controllers if getattr(c, "started", False)]
        check(len(world.controllers) == int(bg.get("walkers", 0)) + walking_on
              and len(started) == int(bg.get("walkers", 0)),
              f"AI controllers: {len(world.controllers)} ({len(started)} started) for {bg.get('walkers')} background "
              f"walkers and {walking_on} walking on")
        check(hasattr(world, "pedestrians_seed") or int(bg.get("walkers", 0)) == 0, "pedestrians seed not set")
        slowed = world.client.manager.speed_differences
        check(len(slowed) == len(traffic_bikes) and all(pct == 50.0 for _, pct in slowed)
              and all(a.type_id in BICYCLES for a, _ in slowed),
              f"background bicycles must get a 50% Traffic Manager speed difference, and only they: "
              f"{[(a.type_id, p) for a, p in slowed]}")
        lane_changes = world.client.manager.lane_changes
        check(not any(enabled for _, enabled in lane_changes)
              and {a.id for a, _ in slowed} <= {a.id for a, _ in lane_changes}
              and (lane_changes or not int(bg.get("vehicles", 0))),
              f"background vehicles must have Traffic Manager lane changes off (no cut-in into the ego's lane): "
              f"{[(a.type_id, e) for a, e in lane_changes]}")
        check(set(world.client.ports) <= {CarlaDataProvider.get_traffic_manager_port()},
              f"Traffic Manager port {world.client.ports}")
    if "pedestrian" in params or "crowd" in params:
        check_walk_on(by_tag.get("pedestrian"), by_tag.get("crowd"), tree, world, True)
    check(world.ticks >= 2, "the extra world tick after the spawns is missing")
    check(all(a.is_alive for a in scenario.other_actors), "an actor is dead after the build")

    # --- cyclists are removed by ActorDestroy: that, remove_all_actors and the cleanup must not destroy anything twice
    if "cyclists" in params:
        check_no_double_destroy(by_tag["cyclists"], scenario)

    # --- cleanup: controllers go, calling it again is harmless
    scenario._cleanup_blocks()
    scenario._cleanup_blocks()
    check(all(not c.is_alive for c in world.controllers), "AI controllers not destroyed by the cleanup")
    scenario.other_actors.clear()  # nothing real to remove


def check_manual_and_plain(module, cmap):
    """Variants of the shipped XMLs with other settings: manual ego, no crossing block, a failing spawn, bad params."""
    xml = config_file("pedestrian_crossing")
    name = "PedestrianCrossing_Town10HD_1"
    print("mocked build: manual ego control")
    scenario, world, spawns, config = build(module, name, xml, cmap,
                                            edit=lambda c: c.other_parameters.__setitem__(
                                                "ego_control", {"mode": "manual"}))
    names = node_names(scenario.scenario_tree)
    print(tree_text(scenario.scenario_tree))
    check("EgoAutopilot" not in names, "EgoAutopilot in manual mode")
    check(scenario._ctx.endless, "manual mode is not endless")
    check(scenario.timeout == spec.MANUAL_DEFAULT_TIMEOUT == 7 * 24 * 3600.0, f"manual-mode timeout {scenario.timeout}")
    check(isinstance(scenario.timeout_node, TimeOut) and scenario.timeout_node._timeout_value == scenario.timeout,
          "manual mode: the timeout node is not a TimeOut of the 7 days")
    check(scenario._setup_scenario_trigger(config) is None, "a manual run with flows waits for the ego to move")
    for flow in ("PedestrianFlow", "CyclistFlow", "BackgroundBubble", "ManualRun"):
        check(flow in names, f"manual mode: {flow} missing from the tree")
    for gone in ("EgoPassedCrossing", "EgoDroveDistance", "WaitForEgo", "Adversary", "StopFileExists"):
        check(gone not in names, f"manual mode: {gone} in the tree")
    check(not any(re.match(r"(Crowd|Cyclist)[0-9]", n) for n in names), "manual mode: single-pass behaviours in the tree")
    check(scenario._block_by_tag["pedestrian"].walker is None and not scenario._block_by_tag["crowd"].walkers,
          "manual mode spawned the main pedestrian or the crowd at load (the flow does that during the run)")
    check(sum(1 for r in spawns.requests if r["category"] == "bicycle" and not r["autopilot"]) == 2,
          "manual mode: the two cyclists are not placed at load")
    scenario.other_actors.clear()

    print("mocked build: manual ego control with a stop file")
    stop_file = Path(tempfile.gettempdir()) / "carsim_check_blocks_stop_file"
    stop_file.unlink(missing_ok=True)

    def edit(c):
        c.other_parameters["ego_control"] = {"mode": "manual"}
        c.other_parameters["run_control"] = {"stop_file": str(stop_file)}
        c.other_parameters["end"] = {"timeout": "90"}
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit)
    names = node_names(scenario.scenario_tree)
    check("StopFileExists" in names, "run_control did not add the stop file condition")
    check(scenario.timeout == 90.0, f"an explicit <end timeout> is ignored in manual mode: {scenario.timeout}")
    scenario.other_actors.clear()

    print("mocked build: manual ego control on a map without a navigation mesh falls back to the single pass")

    def edit(c):
        c.other_parameters["ego_control"] = {"mode": "manual"}
        c.other_parameters["run_control"] = {"stop_file": str(stop_file)}
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit, nav=False)
    names = node_names(scenario.scenario_tree)
    check("PedestrianFlow" not in names and "Adversary" in names and any(n.startswith("Crowd") for n in names),
          f"no navigation mesh: the pedestrian single pass is missing: {names}")
    check("CyclistFlow" in names, "no navigation mesh: the cyclist flow does not need one")
    check("BackgroundBubble" in names, "no navigation mesh: the background vehicles still follow the ego")
    check("EgoPassedCrossing" not in names, "manual mode has a distance end")
    scenario.other_actors.clear()

    print("mocked build: autopilot on a map without a navigation mesh: the pedestrians stay on the far kerb as before")
    scenario, world, spawns, config = build(module, name, xml, cmap, nav=False)
    check_walk_on(scenario._block_by_tag["pedestrian"], scenario._block_by_tag["crowd"], scenario.scenario_tree, world, False)
    check(not [c for c in world.controllers if not getattr(c, "started", False)], "controllers for the walkers without a navigation mesh")
    check(scenario._ctx.walk_on is None, "walk-on points without a navigation mesh")
    scenario.other_actors.clear()

    print("mocked build: manual, no flow at all: the start trigger stays, and the stop file can end the wait")

    def edit(c):
        c.other_parameters.clear()
        c.other_parameters.update({"ego_control": {"mode": "manual"}, "run_control": {"stop_file": str(stop_file)},
                                   "pedestrian": {}, "crowd": {"count": "2"}})
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit, nav=False)
    check(not scenario._ctx.flow_active, "a flow without any flow block")
    trigger = scenario._setup_scenario_trigger(config)
    check(trigger is not None and "StopFileExists" in node_names(trigger) and "TimeToArrival" in node_names(trigger)
          and "EgoGone" in node_names(trigger),
          f"the start trigger of a manual run without a flow: {node_names(trigger) if trigger is not None else None}")
    scenario.other_actors.clear()

    print("mocked build: manual, <background> is the only flow: no start trigger either")

    def edit(c):
        c.other_parameters.clear()
        c.other_parameters.update({"ego_control": {"mode": "manual"}, "run_control": {"stop_file": str(stop_file)},
                                   "background": {"vehicles": "3", "walkers": "2", "radius": "120"}})
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit)
    tree_names = node_names(scenario.behavior_tree)
    check(scenario._ctx.flow_active, "a bubble of background actors did not set flow_active in spawn()")
    check("TimeToArrival" not in tree_names and "EgoMovesOrEnds" not in tree_names
          and len(scenario.behavior_tree.children) == 1,
          f"a manual run whose only flow is <background> kept the start trigger: {tree_names}")
    check("BackgroundBubble" in tree_names and "StopFileExists" in tree_names and "EgoGone" in tree_names,
          f"the bubble or the end conditions are missing: {tree_names}")
    scenario.other_actors.clear()

    # Walkers only and no navigation mesh: the bubble has nothing to do, so the trigger stays (and ends with the stop file)
    scenario, world, spawns, config = build(module, name, xml, cmap, nav=False, edit=lambda c: (
        c.other_parameters.clear(), c.other_parameters.update(
            {"ego_control": {"mode": "manual"}, "run_control": {"stop_file": str(stop_file)},
             "background": {"walkers": "4"}})))
    tree_names = node_names(scenario.behavior_tree)
    check(not scenario._ctx.flow_active and "TimeToArrival" in tree_names and "BackgroundBubble" not in tree_names,
          f"background walkers without a navigation mesh: flow_active {scenario._ctx.flow_active}, {tree_names}")
    scenario.other_actors.clear()

    # An autopilot run never has the start trigger, with or without a background element
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=lambda c: (
        c.other_parameters.clear(), c.other_parameters.update({"background": {"vehicles": "2"}})))
    check("TimeToArrival" not in node_names(scenario.behavior_tree), "an autopilot run kept the start trigger")
    scenario.other_actors.clear()

    print("mocked build: explicit timeout, trigger_distance on the pedestrian wins over the crowd's")

    def edit(c):
        c.other_parameters["end"] = {"timeout": "42"}
        c.other_parameters["crowd"] = {"count": "2", "trigger_distance": "9"}
        c.other_parameters["pedestrian"] = {"trigger_distance": "17"}
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit)
    check(scenario.timeout == 42.0, f"timeout {scenario.timeout}")
    check(scenario._ctx.trigger_distance == 17.0, f"trigger distance {scenario._ctx.trigger_distance}")
    scenario.other_actors.clear()

    print("mocked build: no crossing block (ego_control, background, end distance)")

    def edit(c):
        c.other_parameters.clear()
        c.other_parameters.update({"background": {"vehicles": "3", "bicycles": "1"}, "end": {"distance": "80"}})
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit)
    names = node_names(scenario.scenario_tree)
    print(tree_text(scenario.scenario_tree))
    check(scenario._ctx._crossing is None, "the crossing was computed although no block needs it")
    check("EgoDroveDistance" in names and "WaitForEgo" not in names and "EgoAutopilot" in names, f"tree {names}")
    drive = [n for n in nodes(scenario.scenario_tree) if n.name == "EgoDroveDistance"]
    check(drive and drive[0]._target_distance == 80.0, "DriveDistance is not the end distance")
    check(len(spawns.requests) == 4, f"{len(spawns.requests)} spawn requests for 3 cars and 1 bicycle")
    scenario.other_actors.clear()

    print("mocked build: the main pedestrian's first spawns fail, a retry succeeds")
    scenario, world, spawns, config = build(module, name, xml, cmap, fail={"pedestrian": 2})
    check(len(spawns.requests) > 2 and not spawns.requests[0]["ok"] and not spawns.requests[1]["ok"]
          and spawns.requests[2]["ok"], "the pedestrian spawn was not retried at the next offset")
    scenario.other_actors.clear()

    check_timeout_default(module, xml, name, cmap)
    check_crowd_retry(module, xml, name, cmap)
    check_cyclist_waiting(module, xml, name, cmap)

    print("unknown block tag raises")
    try:
        build(module, name, xml, cmap, edit=lambda c: c.other_parameters.__setitem__("unicorn", {}))
        check(False, "an unknown block tag did not raise")
    except ValueError as error:
        check("unicorn" in str(error), "unknown tag error does not name the tag")


def check_timeout_default(module, xml, name, cmap):
    """Autopilot without <end timeout>: max(60, total drive distance / speed * 1.5 + 20); manual 7 days; explicit wins."""
    print("mocked build: the default timeout grows with the distance to drive")
    speed = 30.0 / 3.6

    def edit(c):
        c.other_parameters["end"] = {"distance_after_crossing": "600"}
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit)
    drive = scenario._ctx.crossing.distance + 600.0
    wait = module.CAUTIOUS_WAIT if config.other_parameters.get("ego_control", {}).get("brake") == "sight" else 0.0
    want = max(60.0, drive / speed * 1.5 + 20.0 + wait)
    check(want > 60.0 and abs(scenario.timeout - want) < 1e-6,
          f"timeout {scenario.timeout:.1f} s, expected {want:.1f} s for {drive:.0f} m at {speed:.2f} m/s")
    scenario.other_actors.clear()

    def edit(c):
        c.other_parameters.clear()
        c.other_parameters.update({"end": {"distance": "2000"}, "ego_control": {"target_speed": "60"}})
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit)
    want = 2000.0 / (60.0 / 3.6) * 1.5 + 20.0
    check(abs(scenario.timeout - want) < 1e-6, f"no crossing: timeout {scenario.timeout:.1f} s, expected {want:.1f} s")
    check(scenario._ctx._crossing is None, "the timeout computed a crossing for a scenario without one")
    scenario.other_actors.clear()

    def edit(c):
        c.other_parameters["end"] = {"distance_after_crossing": "600", "timeout": "75"}
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit)
    check(scenario.timeout == 75.0, f"explicit timeout {scenario.timeout}")
    scenario.other_actors.clear()

    def edit(c):
        c.other_parameters["ego_control"] = {"mode": "manual"}
        c.other_parameters["end"] = {"distance_after_crossing": "600"}
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit)
    check(scenario.timeout == spec.MANUAL_DEFAULT_TIMEOUT, f"manual timeout {scenario.timeout}")
    scenario.other_actors.clear()


def check_crowd_retry(module, xml, name, cmap):
    """A crowd walker whose spawn fails is tried 0.25 m and 0.5 m further back on its own line, then skipped."""
    print("mocked build: a crowd walker that cannot be spawned is retried further back")
    only_crowd = {"pedestrian": {}, "crowd": {"count": "6"}}

    def edit(c):
        c.other_parameters.clear()
        c.other_parameters.update(only_crowd)
    scenario, world, spawns0, config = build(module, name, xml, cmap, edit=edit)
    base = scenario._block_by_tag["crowd"].walkers
    base_requests = spawns0.requests
    check(len(base) == 6, f"baseline crowd {len(base)}")
    scenario.other_actors.clear()

    for failing, label in (({1}, "the first start"), ({1, 2}, "the first two tries")):
        scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit, fail_at=failing)
        crowd = scenario._block_by_tag["crowd"]
        back = 0.25 * (1 + len(failing) - 1)
        check(len(crowd.walkers) == 6, f"{label} failed: {len(crowd.walkers)} of 6 placed")
        first, retry = spawns.requests[1], spawns.requests[1 + len(failing)]
        check(retry["ok"] and not first["ok"], f"{label} failed: the spawn was not retried")
        a, b = first["transform"].location, retry["transform"].location
        fwd = first["transform"].get_forward_vector()
        along = (b.x - a.x) * fwd.x + (b.y - a.y) * fwd.y
        sideways = abs((b.x - a.x) * -fwd.y + (b.y - a.y) * fwd.x)
        check(abs(along + back) < 1e-3 and sideways < 1e-3, f"{label} failed: moved {along:.3f} m along, {sideways:.3f} sideways")
        check(retry["model"] == first["model"], "the retry changed the walker model")
        # Same target and column: the walk is longer by the move, the rest of the crowd unchanged
        check(abs(crowd.walkers[0][3] - (base[0][3] + back)) < 1e-6,
              f"walk distance {crowd.walkers[0][3]:.3f}, expected {base[0][3] + back:.3f}")
        check([w[3] for w in crowd.walkers[1:]] == [w[3] for w in base[1:]], "other walks changed")
        check(len(spawns.requests) == len(base_requests) + len(failing), "extra requests beyond the retries")
        scenario.other_actors.clear()

    # All three tries fail: the walker is skipped, the others stay
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit, fail_at={1, 2, 3})
    crowd = scenario._block_by_tag["crowd"]
    check(len(crowd.walkers) == 5, f"a walker with three failing tries: {len(crowd.walkers)} placed, expected 5")
    scenario.other_actors.clear()


def check_cyclist_waiting(module, xml, name, cmap):
    """The world's fixed step is used; a sidewalk spot that is refused is retried further back, then skipped."""
    print("mocked build: cyclists use the world's step and wait on the sidewalk even when a spawn is refused")
    only_bikes = {"cyclists": {"count": "2"}}

    def edit(c):
        c.other_parameters.clear()
        c.other_parameters.update(only_bikes)
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit, step=0.02)
    check(scenario._block_by_tag["cyclists"].step == 0.02, "the cyclists ignored fixed_delta_seconds")
    scenario.other_actors.clear()
    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit, step=None)
    check(abs(scenario._block_by_tag["cyclists"].step - 0.05) < 1e-12, "no fixed step: the default was not used")
    scenario.other_actors.clear()

    scenario, world, spawns0, config = build(module, name, xml, cmap, edit=edit)
    base = scenario._block_by_tag["cyclists"]
    base_wait = {bike.id: wait for bike, wait in ((b[0], base.waits[b[0].id]) for b in base.bikes)}
    first_wait = spawns0.requests[0]["transform"]
    scenario.other_actors.clear()

    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit, fail_at={0})
    block = scenario._block_by_tag["cyclists"]
    check(len(block.bikes) == 2, f"{len(block.bikes)} bicycles after a refused first spawn")
    first, second = spawns.requests[0], spawns.requests[1]
    check(not first["ok"] and second["ok"], "the refused sidewalk spot was not retried")
    a, b = first["transform"].location, second["transform"].location
    fwd = first["transform"].get_forward_vector()
    check(abs((b.x - a.x) * fwd.x + (b.y - a.y) * fwd.y + 0.5) < 1e-3
          and abs(-(b.x - a.x) * fwd.y + (b.y - a.y) * fwd.x) < 1e-3,
          "the retry is not 0.5 m further back along the cycle line")
    check(first["model"] == second["model"], "the retry changed the bicycle model")
    scenario.other_actors.clear()

    scenario, world, spawns, config = build(module, name, xml, cmap, edit=edit, fail_at={0, 1, 2})
    block = scenario._block_by_tag["cyclists"]
    check(len(block.bikes) == 1, f"a bicycle with three failing sidewalk spots must be skipped: {len(block.bikes)} placed")
    scenario.other_actors.clear()

def check_occluded_crossing(module, cmap, tmp):
    """Both occluded_crossing variants on an inner-lane start, built from the shipped XML with its placeholder ego replaced."""
    starts = find_candidates(cmap, 60.0, lane="inner")
    check(len(starts) >= 1, "no inner-lane start on Town10HD")
    start = starts[0][0].transform
    pose = f'x="{start.location.x}" y="{start.location.y}" z="{start.location.z + 0.3}" yaw="{start.rotation.yaw}"'
    xml = Path(tmp) / "occluded.xml"
    xml.write_text(config_file("occluded_crossing").read_text(encoding="utf-8").replace('x="0" y="0" z="0" yaw="0"', pose),
                   encoding="utf-8")
    cars = {v[0] for v in VEHICLES if v[1] == "car"} | {"vehicle.mini.cooper_s_2021"}
    for variant in [v for v in variants("occluded_crossing") if v.town == "Town10HD_Opt"]:
        print(f"mocked build: occluded_crossing / {variant.name}")
        scenario, world, spawns, config = build(module, variant.name, xml, cmap)
        params = config.other_parameters
        names = node_names(scenario.scenario_tree)
        by_tag = {block.tag: block for block in scenario._blocks}
        expected = [tag for tag in spec.BLOCK_ORDER if tag in params or tag in spec.IMPLICIT_BLOCKS]
        check([b.tag for b in scenario._blocks] == expected, f"blocks {[b.tag for b in scenario._blocks]} != {expected}")
        check(scenario._ctx.uses_crossing and "EgoCautiousDriver" in names and "EgoAutopilot" not in names,
              "the occluded crossing must use the crossing and the cautious driver")
        # The truck first, then the queue, all standing on the kerb lane right of the ego lane
        queue = int(params["occluder"]["queue"])
        models = spawns.models("car")
        traffic = int(params.get("background", {}).get("vehicles", 0))  # background cars spawn after the occluder
        check(models[0] == "vehicle.carlamotors.european_hgv" and len(models) == 1 + queue + traffic
              and all(m in cars for m in models[1:1 + queue]), f"occluder spawns {models}")
        prop = params["occluder"].get("prop", "none")
        if prop != "none":
            check(prop in spawns.models("misc"), f"the occluder prop {prop} was not spawned: {spawns.models('misc')}")
        route = scenario._ctx.route_lanes()
        for vehicle in scenario.other_actors[:1 + queue]:
            wp = cmap.get_waypoint(vehicle.transform.location, project_to_road=False, lane_type=carla.LaneType.Driving)
            left = wp.get_left_lane() if wp is not None else None
            check(left is not None and (left.road_id, left.lane_id) in route,
                  f"{vehicle.type_id} is not on the kerb lane right of the ego lane")
        if "trigger_distance" in params["pedestrian"]:  # with `overlap` the walk starts by time (tests/check_trigger_offline.py)
            want = float(params["pedestrian"]["trigger_distance"])
            check(scenario._ctx.trigger_distance == want, f"trigger distance {scenario._ctx.trigger_distance} != {want}")
        if "age" in params["pedestrian"]:
            check(by_tag["pedestrian"].model in CHILDREN, f"the child walker is {by_tag['pedestrian'].model}")
        if "crowd" in params:
            wanted = int(params["crowd"]["count"])
            check(wanted - 2 <= len(by_tag["crowd"].walkers) <= wanted, f"crowd placed {len(by_tag['crowd'].walkers)}")
            check(len(by_tag["cyclists"].bikes) >= 1, "no crossing cyclists")
        scenario.other_actors.clear()


def check_cut_in(module, cmap, tmp):
    """Both cut_in variants, from the shipped XML with its placeholder ego replaced by a start that fits the kind."""
    # Junction free and straight for what the kind needs; found by tests/check_cut_in_offline.py (x, y, z, yaw)
    starts = {"CutIn_Parking_Town10HD_1": (-8.21, 140.66, 0.30, 0.35), "CutIn_Cyclist_Town10HD_1": (19.35, 137.46, 0.30, 0.32)}
    kinds = {"CutIn_Parking_Town10HD_1": ("ParkingCutIn", 2), "CutIn_Cyclist_Town10HD_1": ("CyclistCutIn", 3)}
    for variant in [v for v in variants("cut_in") if v.name in starts]:
        print(f"mocked build: cut_in / {variant.name}")
        x, y, z, yaw = starts[variant.name]
        xml = Path(tmp) / "cut_in.xml"
        xml.write_text(config_file("cut_in").read_text(encoding="utf-8").replace(
            'x="0" y="0" z="0" yaw="0"', f'x="{x}" y="{y}" z="{z}" yaw="{yaw}"'), encoding="utf-8")
        scenario, world, spawns, config = build(module, variant.name, xml, cmap)
        names = node_names(scenario.scenario_tree)
        tree_node, actors = kinds[variant.name]
        check(tree_node in names and "EgoCautiousDriver" in names, f"{tree_node} / EgoCautiousDriver missing in the tree")
        check(len(scenario._block_by_tag["cut_in"].actors) == actors, "the cut_in actors")
        check(not scenario._ctx.uses_crossing and "WaitForEgo" not in names, "cut_in must not use the crossing")
        scenario.other_actors.clear()


def check_surface():
    """
    BlockContext.surface: unchanged on a map that is not deformed; on a deformed one the z goes up by
    road.surface_offset, a spawn point is rebased on the lane under it; and every spawn and set_transform in the blocks
    goes through it, except the pedestrians spawned at navigation points (the navigation mesh has their height).
    """
    from types import SimpleNamespace

    from carsim import road
    from carsim.blocks.base import SPAWN_ABOVE, BlockContext

    print("spawn heights on deformed maps")
    lane = SimpleNamespace(transform=carla.Transform(carla.Location(0, 0, 2.0)))
    cmap = SimpleNamespace(name="/Game/Carla/Maps/Town10HD_Opt",
                           get_waypoint=lambda loc, project_to_road=True, lane_type=None: None if lane_type else lane)
    tf = carla.Transform(carla.Location(100.0, -50.0, 3.0), carla.Rotation(yaw=30.0))
    ctx = BlockContext(None, cmap, None, None, None)
    check(ctx.surface(tf) is tf, "a Town map must not be shifted")
    cmap.name = "/Game/CustomMaps/Wolfsburg2/Wolfsburg2"  # built with the stock tool's waves
    ctx = BlockContext(None, cmap, None, None, None)
    with mock.patch.dict(os.environ, {road.DEFORMED_ENV: ""}):
        moved = ctx.surface(tf)
        rebased = ctx.surface(tf, above=SPAWN_ABOVE)
        point = ctx.surface(tf.location)
    dz = road.deformation(100.0, -50.0)
    check(abs(dz) > 0.01, "the test point must lie off a wave's zero")
    check(abs(moved.location.z - 3.0 - dz) < 1e-6 and moved.rotation.yaw == 30.0 and moved.location.x == 100.0,
          f"deformed: z {moved.location.z}, want {3.0 + dz}")
    check(abs(rebased.location.z - 2.0 - SPAWN_ABOVE - dz) < 1e-6, f"a spawn point rebased on its lane: {rebased.location.z}")
    check(isinstance(point, carla.Location) and abs(point.z - moved.location.z) < 1e-6, "a Location is shifted the same")
    cmap.name = "/Game/CustomMaps/WolfsburgHighway/WolfsburgHighway"  # flat roads (road.FLAT_MAPS): a lane keeps its z
    with mock.patch.dict(os.environ, {road.DEFORMED_ENV: ""}):
        flat = BlockContext(None, cmap, None, None, None).surface(tf)
    check(abs(flat.location.z - 3.0) < 1e-6, f"flat map: z {flat.location.z}, want 3.0")

    # navigation_point: on a deformed map only a point on a Sidewalk lane (others crashed the 0.9.15 server), retried
    from carsim.blocks.base import NAVIGATION_SIDEWALK_TRIES
    drawn = iter([carla.Location(1, 0, 0), carla.Location(2, 0, 0), carla.Location(3, 0, 0)])
    world = SimpleNamespace(get_random_location_from_navigation=lambda: next(drawn, None))
    walk = SimpleNamespace(name=cmap.name, get_waypoint=lambda loc, project_to_road=True, lane_type=None:
                           lane if lane_type == carla.LaneType.Sidewalk and loc.x == 3 else None)
    with mock.patch.dict(os.environ, {road.DEFORMED_ENV: ""}):
        nav = BlockContext(world, walk, None, None, None)
        got = nav.navigation_point()
        check(got is not None and got.x == 3, f"navigation_point must skip points off the sidewalk: {got}")
        check(nav.on_sidewalk(carla.Location(3, 0, 0)) and not nav.on_sidewalk(carla.Location(1, 0, 0)),
              "on_sidewalk on a deformed map")
        never = BlockContext(SimpleNamespace(get_random_location_from_navigation=lambda: carla.Location(1, 0, 0)),
                             walk, None, None, None)
        check(never.navigation_point() is None, "navigation_point must give up (None) after its tries")
    check(NAVIGATION_SIDEWALK_TRIES > 1, "navigation_point must retry")
    town = BlockContext(SimpleNamespace(get_random_location_from_navigation=lambda: carla.Location(1, 0, 0)),
                        SimpleNamespace(name="/Game/Carla/Maps/Town10HD_Opt", get_waypoint=walk.get_waypoint), None, None, None)
    check(town.navigation_point().x == 1 and town.on_sidewalk(carla.Location(1, 0, 0)), "a Town map is not filtered")

    for path in sorted((ROOT / "src" / "carsim" / "blocks").glob("*.py")):
        text = path.read_text(encoding="utf-8")
        for match in re.finditer(r"(request_new_actor|\.set_transform)\(", text):
            call = text[match.end():match.end() + 160]  # the arguments, about two lines
            navigation = re.search(r"\bloc(ation)?\.z \+ 1\.0", call)  # a pedestrian at a navigation point
            if "surface(" not in call and not navigation:
                line = text.count("\n", 0, match.start()) + 1
                check(False, f"{path.name}:{line}: a spawn or set_transform that does not go through ctx.surface")


def check_other_maps(module):
    """
    A mocked build of every variant of the other scenarios on a map other than Town10HD (WolfsburgHighway), on that
    map's own OpenDRIVE file (tests/check_crossings_offline.xodr_path: CARSIM_XODR_DIRS or the CARLA install); skipped
    when it is not found. The build places every actor of the blocks, so a start that does not fit them is an error.
    """
    maps = {}
    for name in ("occluded_crossing", "cut_in"):
        for variant in [v for v in variants(name) if v.town != "Town10HD_Opt"]:
            path = xodr_path(variant.town)
            if not path.exists():
                print(f"  SKIP {variant.name}: no {path.name} (put its folder in CARSIM_XODR_DIRS)")
                continue
            print(f"mocked build: {name} / {variant.name}")
            if variant.town not in maps:
                maps[variant.town] = carla.Map(variant.town, path.read_text(encoding="utf-8"))
            try:
                scenario, _, _, _ = build(module, variant.name, config_file(name), maps[variant.town])
                scenario.other_actors.clear()
            except ValueError as error:
                check(False, f"{variant.name} does not build on its map: {error}")


def main():
    check_registry()
    check_overlap_validation()
    check_files()
    check_validate_in_temp_folders()

    module = load_scenario_module(COMPOSED_SCENARIO_FILE)
    check(hasattr(module, "ComposedScenario"), "composed_scenario.py has no ComposedScenario")
    maps = {}
    for name in ("pedestrian_crossing",):
        for variant in variants(name):
            town = {"Town10HD_Opt": "Town10HD"}.get(variant.town, variant.town)
            if town not in maps:
                path = xodr_path(town)
                if not path.exists():
                    print(f"  SKIP {variant.name}: no {path.name} (put its folder in CARSIM_XODR_DIRS)")
                    continue
                maps[town] = carla.Map(town, path.read_text(encoding="utf-8"))
            check_variant(module, name, variant, maps[town])
    if "Town10HD" not in maps:
        maps["Town10HD"] = carla.Map("Town10HD", xodr_path("Town10HD").read_text(encoding="utf-8"))
    check_manual_and_plain(module, maps["Town10HD"])
    with tempfile.TemporaryDirectory() as tmp:
        check_occluded_crossing(module, maps["Town10HD"], tmp)
        check_cut_in(module, maps["Town10HD"], tmp)
    check_other_maps(module)
    check_surface()

    print("FAILED" if FAILURES else "PASSED")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
