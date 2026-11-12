#!/usr/bin/env python
"""
Offline check of the cut_in block (src/carsim/blocks/cut_in.py), no CARLA server needed.

For each kind a mocked build of ComposedScenario (the fakes of tests/check_blocks_offline.py, with bounding boxes added
here; an offline Town10HD map) from a temporary scenario XML without a `brake` attribute, with the ego started on a lane
that fits the kind (found on the map: junction free and straight for as long as the cut-in needs). Checked:

  - the actors are on the right lanes (parking: both on the Shoulder; cyclist: all three on the Driving lane right of
    the ego lane), in the right order (parking: blocker nearer the ego than the car; cyclist: van B, cyclist, car A),
    with the clear gaps (parking 1.5 m; cyclist 3 m ahead, 1.5 m behind), the parked ones with the hand brake on;
    the blocker is the default of the kind (firetruck for the cyclist) or the one of the XML
  - the block has no say in the shared trigger distance (trigger_distance() is None)
  - the tree: the trigger, a Launch and the pull-out agent (parking), or the ride and LaneChange (cyclist), and a last
    follower without a plan (they never end: nothing finite directly under Main but the ego's own nodes); the parking
    Launch gives the car `speed` along its heading; the cyclist convoy has a never ending HoldConvoySpeed under Main
    that forces all three actors to `speed` every tick (a kinematic check: same speed, gaps constant); the blocker does not avoid collisions (it would never move 1.5 m behind
    the cyclist), car A and the blocker follow without a plan
  - ctx.target is the cut-in actor (parking: the car, cyclist: the bicycle)
  - the cut-in actor is hidden behind its blocker from the ego lane at the trigger distance (a 2D ray against the
    blocker's box), and the LaneChange of the cyclist has a plan at the place it will start from
  - errors: a bad or missing kind, an unknown blocker, a start without the lane the kind needs, a spawn that fails
  - parking without a Shoulder (maps such as WolfsburgHighway): is_kerb_lane on fake lanes, and on a cyclist start (the
    ego in the lane left of the kerb lane) the car and the blocker stand double-parked in the kerb Driving lane, logged

    python tests/check_cut_in_offline.py

Exits with 1 if any check fails.
"""

import math
import sys
import tempfile
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))

import carla  # noqa: E402
import py_trees  # noqa: E402

import check_blocks_offline as fakes  # noqa: E402
from check_blocks_offline import FakeActor, Spawns, build, load_scenario_module, nodes, tree_text, xodr_path  # noqa: E402
from carsim.blocks import cut_in  # noqa: E402
from carsim.road import lane_point  # noqa: E402
from carsim.scenario_files import COMPOSED_SCENARIO_FILE  # noqa: E402
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import (BasicAgentBehavior, LaneChange,  # noqa: E402
                                                                      WaypointFollower)
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle  # noqa: E402
from srunner.tools.scenario_helper import generate_target_waypoint_list_multilane  # noqa: E402

FAILURES = []

EGO_SPEED = 30 / 3.6  # the scenarios' target_speed of 30 km/h [m/s]
SPRINTER, FIRETRUCK = "vehicle.mercedes.sprinter", "vehicle.carlamotors.firetruck"
EXTENTS = {cut_in.CAR: (1.9, 0.9), SPRINTER: (2.95, 1.0), FIRETRUCK: (4.25, 1.45),
           "vehicle.carlamotors.european_hgv": (3.95, 1.45)}  # half length, half width of the vehicles [m], measured on 0.9.16
BIKE_EXTENT, CAR_EXTENT = (0.9, 0.4), (2.3, 0.95)  # bicycles and the other cars
GAP_TOLERANCE = 0.1  # [m]
START_SPOTS = 2  # ego starts tried per kind


def check(condition, message):
    if not condition:
        FAILURES.append(message)
        print("  FAIL " + message)
    return condition


# --- fakes ---------------------------------------------------------------------------------------------------------

class BoxedSpawns(Spawns):
    """Spawns whose actors have a bounding box (half sizes as above) and record the controls applied to them."""

    def __call__(self, model, *args, **kwargs):
        actor = super().__call__(model, *args, **kwargs)
        if actor is not None:
            x, y = EXTENTS.get(model) or (BIKE_EXTENT if model in fakes.BICYCLES else CAR_EXTENT)
            actor.bounding_box = carla.BoundingBox(carla.Location(), carla.Vector3D(x, y, 1.0))
            actor.controls = []
            actor.apply_control = actor.controls.append
            actor.velocities = []
            actor.set_target_velocity = actor.velocities.append
        return actor


def build_cut_in(module, cmap, tmp, kind, start, extra="", **kwargs):
    """ComposedScenario with <cut_in kind=...> and the ego at the waypoint `start` (no brake attribute)."""
    t = start.transform
    xml = Path(tmp) / f"cut_in_{kind}.xml"
    xml.write_text(f"""<scenarios>
        <scenario name="TempCutIn_{kind}_Town10HD_1" type="ComposedScenario" town="Town10HD_Opt">
            <ego_vehicle x="{t.location.x}" y="{t.location.y}" z="{t.location.z + 0.3}" yaw="{t.rotation.yaw}"
                         model="vehicle.lincoln.mkz_2017" rolename="hero"/>
            <ego_control mode="autopilot" target_speed="30"/>
            <cut_in kind="{kind}"{extra}/>
            <end distance="120" timeout="90"/>
            <weather cloudiness="20"/>
        </scenario></scenarios>""", encoding="utf-8")
    with mock.patch.object(fakes, "Spawns", BoxedSpawns):
        return build(module, f"TempCutIn_{kind}_Town10HD_1", xml, cmap, **kwargs)


# --- map: where the ego can start ----------------------------------------------------------------------------------

def straight_free(wp, free, straight):
    """True when the lane is free of junctions for `free` m ahead of `wp` and turns less than 5 degrees in `straight` m."""
    if any(lane_point(wp, s).is_junction for s in range(0, int(free) + 1, 4)):
        return False
    turn = lane_point(wp, straight).transform.rotation.yaw - wp.transform.rotation.yaw
    return abs((turn + 180.0) % 360.0 - 180.0) < 5.0


def catch_up(distance, trigger, speed):
    """Where along the lane the ego reaches the cyclist [m from the ego start]."""
    return distance + speed * (distance - trigger) / (EGO_SPEED - speed)


def lane_changes_left(side_wp, distance, speed):
    """True when the cyclist, riding in the kerb lane `side_wp` (at `distance`), can change left where it starts to."""
    start = lane_point(side_wp, catch_up(distance, cut_in.DEFAULTS["cyclist"][1], speed) - distance)
    plan, _ = generate_target_waypoint_list_multilane(start, "left", 2, 10, 8, check=True)
    return plan is not None


def find_starts(cmap, kind, count=START_SPOTS):
    """Ego starts (waypoints) for `kind`, one per lane: parking needs a Shoulder right of the lane at the default
    distance and junction free for 60 m more; cyclist a Driving lane of the same direction right of it, junction free
    until 20 m past the catch-up point, and a lane change that has a plan. Straight (under 5 degrees) up to 15 m past
    the cut-in actor, as the checks measure along the ego's heading."""
    distance, trigger, speed = cut_in.DEFAULTS[kind]
    ahead = distance + 60.0 if kind == "parking" else catch_up(distance, trigger, speed) + 20.0
    starts, seen = [], set()
    for wp in cmap.generate_waypoints(2.0):
        key = (wp.road_id, wp.lane_id)
        if key in seen or wp.lane_type != carla.LaneType.Driving or wp.is_junction:
            continue
        right = wp.get_right_lane()
        if right is None:
            continue
        if kind == "parking":
            ok = right.lane_type == carla.LaneType.Shoulder
        else:
            ok = right.lane_type == carla.LaneType.Driving and (right.lane_id > 0) == (wp.lane_id > 0)
        if not ok or not straight_free(wp, ahead, distance + 15.0):
            continue
        ref = lane_point(wp, distance).get_right_lane()
        if kind == "parking":
            ok = ref is not None and ref.lane_type == carla.LaneType.Shoulder
        else:
            ok = ref is not None and ref.lane_type == carla.LaneType.Driving and lane_changes_left(ref, distance, speed)
        if ok:
            seen.add(key)
            starts.append(wp)
            if len(starts) >= count:
                break
    return starts


# --- 2D geometry ---------------------------------------------------------------------------------------------------

def forward(wp):
    return wp.transform.get_forward_vector()


def along(origin, vector, location):
    """Distance of `location` from `origin` along the unit `vector` (2D) [m]."""
    return (location.x - origin.x) * vector.x + (location.y - origin.y) * vector.y


def hits_box(a, b, actor):
    """True when the segment a-b (carla.Location, 2D) crosses the actor's box (slab test in the box's frame)."""
    yaw = math.radians(actor.transform.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    ext = actor.bounding_box.extent

    def local(p):
        dx, dy = p.x - actor.transform.location.x, p.y - actor.transform.location.y
        return (dx * c + dy * s, -dx * s + dy * c)

    (ax, ay), (bx, by) = local(a), local(b)
    t0, t1 = 0.0, 1.0
    for p, d, e in ((ax, bx - ax, ext.x), (ay, by - ay, ext.y)):
        if abs(d) < 1e-9:
            if abs(p) > e:
                return False
            continue
        lo, hi = sorted(((-e - p) / d, (e - p) / d))
        t0, t1 = max(t0, lo), min(t1, hi)
        if t0 > t1:
            return False
    return True


def ego_at_trigger(start, distance, actor, trigger):
    """The ego's place on its lane where its distance to `actor` is `trigger` (the lane point nearest to that)."""
    best = min((abs(lane_point(start, s / 2.0).transform.location.distance(actor.transform.location) - trigger),
                s / 2.0) for s in range(0, int(distance * 2)))
    return lane_point(start, best[1]).transform.location


# --- the checks ----------------------------------------------------------------------------------------------------

def check_actor_lanes(cmap, scenario, block, kind, start):
    actors = block.actors
    spawns = scenario._spawns
    if kind == "parking":
        check(spawns.models("car") == [cut_in.CAR, SPRINTER], f"parking spawns {spawns.models('car')}")
        for role in ("car", "blocker"):
            loc = actors[role].transform.location
            wp = cmap.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Shoulder)
            left = wp.get_left_lane() if wp is not None else None
            check(left is not None and (left.road_id, left.lane_id) in scenario._ctx.route_lanes(),
                  f"{role} is not on the Shoulder right of the ego lane")
        order = ("blocker", "car")
        gaps = [("car", "blocker", cut_in.PARKED_GAP)]
    else:
        check(spawns.models("bicycle")[0] in fakes.BICYCLES, f"cyclist model {spawns.models('bicycle')}")
        check(spawns.models("car")[1] == FIRETRUCK, f"blocker model {spawns.models('car')}")
        for role in ("cyclist", "car_a", "blocker"):
            wp = cmap.get_waypoint(actors[role].transform.location, project_to_road=False,
                                   lane_type=carla.LaneType.Driving)
            left = wp.get_left_lane() if wp is not None else None
            check(left is not None and (left.road_id, left.lane_id) in scenario._ctx.route_lanes(),
                  f"{role} is not on the Driving lane right of the ego lane")
        order = ("blocker", "cyclist", "car_a")
        gaps = [("car_a", "cyclist", cut_in.AHEAD_GAP), ("cyclist", "blocker", cut_in.BEHIND_GAP)]
    ego = start.transform.location
    fwd = forward(start)
    pos = {role: along(ego, fwd, actors[role].transform.location) for role in actors}
    check(all(pos[a] < pos[b] for a, b in zip(order, order[1:])), f"{kind} order along the lane: {pos}")
    for front, back, want in gaps:
        gap = pos[front] - pos[back] - cut_in.half_length(actors[front]) - cut_in.half_length(actors[back])
        check(abs(gap - want) < GAP_TOLERANCE, f"{kind}: clear gap {back} to {front} is {gap:.2f} m, want {want:g}")
    check(abs(pos["car" if kind == "parking" else "cyclist"] - block.distance) < 1.0,
          f"{kind}: the cut-in actor is {pos} m ahead, not {block.distance:g}")
    if kind == "parking":
        for role in ("car", "blocker"):
            check(any(c.hand_brake for c in actors[role].controls), f"{role} has no hand brake")


def check_launch(launch, actors, speed, hold=False):
    """One tick of the Launch gives every actor `speed` along its heading; it succeeds, or with `hold` keeps running."""
    check(launch.actors == actors and launch.speed == speed and launch.hold == hold,
          "the Launch has the wrong actors, speed or hold")
    want_status = py_trees.common.Status.RUNNING if hold else py_trees.common.Status.SUCCESS
    status = launch.update()
    check(status == want_status, f"the Launch returned {status}, want {want_status}")
    for actor in actors:
        want = actor.get_transform().get_forward_vector()
        got = actor.velocities[-1:] or [None]
        check(got[0] is not None and abs(got[0].x - want.x * speed) < 1e-6 and abs(got[0].y - want.y * speed) < 1e-6
              and abs(got[0].z - want.z * speed) < 1e-6,
              f"{actor.type_id} velocity {got[0]}, want {speed:g} m/s along its heading")


def check_convoy_kinematics(hold, block, steps=100, dt=0.05):
    """The convoy moved by the hold node's commanded velocities alone keeps the same speed, so the gaps stay."""
    actors = [block.actors[role] for role in ("blocker", "cyclist", "car_a")]
    gaps = lambda: [b.transform.location.distance(a.transform.location) for a, b in zip(actors, actors[1:])]
    before = gaps()
    for _ in range(steps):
        hold.update()
        for actor in actors:
            v, loc = actor.velocities[-1], actor.transform.location
            check(abs(math.hypot(v.x, v.y) - block.speed) < 1e-6, f"{actor.type_id} commanded {math.hypot(v.x, v.y):g} m/s")
            loc.x += v.x * dt
            loc.y += v.y * dt
    check(all(abs(a - b) < 1e-3 for a, b in zip(before, gaps())), f"convoy gaps changed {before} -> {gaps()}")


def check_tree(scenario, block, kind):
    tree = scenario.scenario_tree
    print(tree_text(tree))
    check(block.trigger_distance() is None, "cut_in has a say in the trigger distance")
    check(scenario._ctx.trigger_distance == 25.0, f"shared trigger distance {scenario._ctx.trigger_distance}")
    main = next(node for node in nodes(tree) if node.name == "Main")
    mine = [child for child in main.children if child.name in ("ParkingCutIn", "CyclistCutIn", "CarAFollows",
                                                               "BlockerFollows", "HoldConvoySpeed")]
    check(len(mine) == (1 if kind == "parking" else 4), f"{kind}: {len(mine)} cut-in nodes under Main")
    # Nothing finite directly under Main: followers without a plan, sequences that end in one, the never ending hold
    for node in mine:
        if node.name == "HoldConvoySpeed":
            continue
        last = node.children[-1] if node.children else node
        check(isinstance(last, WaypointFollower) and last._plan is None,
              f"{node.name} does not end in a follower without a plan")
    sequence = next(child for child in mine if child.name.endswith("CutIn"))
    types = [type(child) for child in sequence.children]
    if kind == "parking":
        check(types == [InTriggerDistanceToVehicle, cut_in.Launch, BasicAgentBehavior, WaypointFollower],
              f"parking tree {types}")
        check(bool(sequence.children[2]._plan), "the pull-out agent has no plan")
        check_launch(sequence.children[1], [block.actors["car"]], block.speed)
        check(sequence.children[0]._distance == 15.0 and sequence.children[0]._actor is block.actors["car"]
              and sequence.children[0]._reference_actor is scenario._ctx.ego, "parking trigger: ego, car, 15 m")
    else:
        ride = sequence.children[0]
        check(types[1:] == [LaneChange, WaypointFollower] and isinstance(ride, py_trees.composites.Parallel),
              f"cyclist tree {types}")
        hold = next(child for child in mine if child.name == "HoldConvoySpeed")
        check(type(hold) is cut_in.Launch, f"HoldConvoySpeed is a {type(hold).__name__}")
        check_launch(hold, [block.actors[r] for r in ("cyclist", "car_a", "blocker")], block.speed, hold=True)
        check_convoy_kinematics(hold, block)
        check(ride.policy == py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, "the ride Parallel is not SUCCESS_ON_ONE")
        check([type(child) for child in ride.children] == [WaypointFollower, InTriggerDistanceToVehicle]
              and ride.children[0]._plan is None and ride.children[1]._distance == cut_in.DEFAULTS["cyclist"][1],
              "the ride Parallel is not a plain follower plus the trigger")
        lane_change = sequence.children[1]
        check((lane_change._direction, lane_change._distance_same_lane, lane_change._distance_other_lane,
               lane_change._distance_lane_change) == ("left", 2, 10, 8), "LaneChange arguments")
        blocker = next(child for child in mine if child.name == "BlockerFollows")
        check(not blocker._avoid_collision and blocker._plan is None, "the blocker must not avoid collisions, no plan")
        check(not next(child for child in mine if child.name == "CarAFollows")._avoid_collision, "car A avoids")
    for node in nodes(sequence):
        check(node.name not in ("Idle", "StopVehicle", "ActorDestroy") and "DriveDistance" not in type(node).__name__,
              f"{node.name} could end or stop the cut-in actor")


def check_hidden(start, block, kind):
    """The cut-in actor is hidden behind its blocker from the ego lane at the trigger distance (2D rays)."""
    actors = block.actors
    target, blocker = ("car", "blocker") if kind == "parking" else ("cyclist", "blocker")
    ego = ego_at_trigger(start, block.distance, actors[target], block.trigger_at)
    centre = actors[target].transform.location
    check(hits_box(ego, centre, actors[blocker]), f"{kind}: the {target} is not hidden behind the {blocker} "
                                                  f"from the ego {block.trigger_at:g} m away")
    if kind == "cyclist":  # the blocker is what hides it: car A must not
        check(not hits_box(ego, centre, actors["car_a"]), "car A is in the way of the cyclist's centre")


def check_kind(module, cmap, tmp, kind):
    print(f"cut_in kind={kind}")
    starts = find_starts(cmap, kind)
    check(len(starts) >= START_SPOTS, f"{kind}: only {len(starts)} ego starts on the map")
    for start in starts:
        t = start.transform
        print(f"  start x={t.location.x:.2f} y={t.location.y:.2f} z={t.location.z + 0.3:.2f} yaw={t.rotation.yaw:.2f} "
              f"(road {start.road_id} lane {start.lane_id})")
    for i, start in enumerate(starts):
        scenario, world, spawns, config = build_cut_in(module, cmap, tmp, kind, start)
        scenario._spawns = spawns
        block = scenario._block_by_tag["cut_in"]
        check(block.distance == cut_in.DEFAULTS[kind][0] and block.trigger_at == cut_in.DEFAULTS[kind][1]
              and block.speed == cut_in.DEFAULTS[kind][2], f"{kind}: defaults {block.distance, block.trigger_at}")
        check(not any("brake" in v for v in config.other_parameters.values()), "the temporary XML has a brake")
        check_actor_lanes(cmap, scenario, block, kind, start)
        check(scenario._ctx.target is block.actors["car" if kind == "parking" else "cyclist"],
              f"{kind}: ctx.target is {scenario._ctx.target}")
        if i == 0:
            check_tree(scenario, block, kind)
        check_hidden(start, block, kind)
        scenario.other_actors.clear()
    return starts


def check_overrides(module, cmap, tmp, starts):
    """Values from the XML win over the defaults."""
    scenario, _, _, _ = build_cut_in(module, cmap, tmp, "parking", starts["parking"][0],
                                     extra=' distance="70" trigger_distance="18" speed="3"')
    block = scenario._block_by_tag["cut_in"]
    check((block.distance, block.trigger_at, block.speed) == (70.0, 18.0, 3.0),
          f"overrides {(block.distance, block.trigger_at, block.speed)}")
    scenario.other_actors.clear()
    for kind, want in (("parking", "vehicle.carlamotors.european_hgv"), ("cyclist", "vehicle.carlamotors.carlacola")):
        scenario, _, spawns, _ = build_cut_in(module, cmap, tmp, kind, starts[kind][0], extra=f' blocker="{want}"')
        check(scenario._block_by_tag["cut_in"].blocker == want and want in spawns.models("car"),
              f"{kind}: blocker override {scenario._block_by_tag['cut_in'].blocker}, spawned {spawns.models('car')}")
        scenario.other_actors.clear()


def check_errors(module, cmap, tmp, starts):
    print("errors")
    for bad in ("", "parking_lot"):
        extra_xml = Path(tmp) / "bad_kind.xml"
        start = starts["parking"][0]
        t = start.transform
        extra_xml.write_text(f"""<scenarios>
            <scenario name="TempBadKind_Town10HD_1" type="ComposedScenario" town="Town10HD_Opt">
                <ego_vehicle x="{t.location.x}" y="{t.location.y}" z="{t.location.z + 0.3}" yaw="{t.rotation.yaw}"/>
                <cut_in {'kind="%s"' % bad if bad else ''}/>
            </scenario></scenarios>""", encoding="utf-8")
        try:
            with mock.patch.object(fakes, "Spawns", BoxedSpawns):
                build(module, "TempBadKind_Town10HD_1", extra_xml, cmap)
            check(False, f"kind {bad!r} was accepted")
        except ValueError as error:
            check("kind" in str(error), f"kind {bad!r}: error does not name the kind: {error}")
    for kind in ("parking", "cyclist"):
        try:
            build_cut_in(module, cmap, tmp, kind, starts[kind][0], extra=' blocker="vehicle.no.such_truck"')
            check(False, f"{kind}: an unknown blocker was accepted")
        except ValueError as error:
            check("vehicle.no.such_truck" in str(error), f"{kind}: error does not name the blocker: {error}")
    # A start without the lane the kind needs (parking: a cyclist start whose Driving lane on the right is taken for no
    # kerb lane, as with a third lane right of it; Town10HD has no such start)
    for kind, start in (("parking", starts["cyclist"][0]), ("cyclist", starts["parking"][0])):
        try:
            with mock.patch.object(cut_in, "is_kerb_lane", lambda wp, side: False):
                build_cut_in(module, cmap, tmp, kind, start)
            check(False, f"kind {kind} accepted a start without the lane it needs")
        except ValueError as error:
            want = ("Shoulder", "kerb Driving lane") if kind == "parking" else ("Driving lane",)
            check(all(w in str(error) for w in want), f"{kind} error: {error}")
    # A failing spawn
    for kind in ("parking", "cyclist"):
        for index in (0, 1, 2) if kind == "cyclist" else (0, 1):
            try:
                build_cut_in(module, cmap, tmp, kind, starts[kind][0], fail_at={index})
                check(False, f"{kind}: a failed spawn (request {index}) was accepted")
            except ValueError as error:
                check("could not spawn" in str(error), f"{kind} spawn failure {index}: {error}")


def check_kerb_parking(module, cmap, tmp, starts):
    """Parking without a Shoulder: double-parked in the kerb Driving lane right of the ego lane."""
    print("parking in the kerb lane")

    class Lane:
        def __init__(self, lane_id, lane_type=carla.LaneType.Driving, right=None):
            self.lane_id, self.lane_type, self.right = lane_id, lane_type, right

        def get_right_lane(self):
            return self.right

    ego = Lane(-1)
    check(cut_in.is_kerb_lane(ego, Lane(-2)), "a same-direction Driving lane with nothing right of it is the kerb lane")
    check(cut_in.is_kerb_lane(ego, Lane(-2, right=Lane(-3, carla.LaneType.Sidewalk))), "a sidewalk right of it: kerb lane")
    check(not cut_in.is_kerb_lane(ego, Lane(-2, right=Lane(-3))), "a Driving lane right of it: not the kerb lane")
    check(not cut_in.is_kerb_lane(ego, Lane(2)), "the other direction is not the kerb lane")
    check(not cut_in.is_kerb_lane(ego, Lane(-2, carla.LaneType.Sidewalk)), "a sidewalk is not the kerb lane")
    check(not cut_in.is_kerb_lane(ego, None), "no lane is not the kerb lane")

    start = next((wp for wp in starts["cyclist"] if cut_in.is_kerb_lane(
        lane_point(wp, 60.0), lane_point(wp, 60.0).get_right_lane())), None)
    if not check(start is not None, "no cyclist start has a kerb Driving lane on its right"):
        return
    with mock.patch("builtins.print") as printed:
        scenario, _, _, _ = build_cut_in(module, cmap, tmp, "parking", start)
    lines = " ".join(str(call.args[0]) for call in printed.call_args_list if call.args)
    block = scenario._block_by_tag["cut_in"]
    for role in ("car", "blocker"):
        wp = cmap.get_waypoint(block.actors[role].transform.location, project_to_road=False,
                               lane_type=carla.LaneType.Driving)
        left = wp.get_left_lane() if wp is not None else None
        check(left is not None and (left.road_id, left.lane_id) in scenario._ctx.route_lanes(),
              f"{role} is not in the Driving lane right of the ego lane")
    check("double-parked in the kerb Driving lane" in lines, f"the double parking is not logged: {lines[:300]}")
    check(scenario._ctx.target is block.actors["car"], "ctx.target is the car")
    scenario.other_actors.clear()


def main():
    module = load_scenario_module(COMPOSED_SCENARIO_FILE)
    cmap = carla.Map("Town10HD", xodr_path("Town10HD").read_text(encoding="utf-8"))
    starts = {}
    with tempfile.TemporaryDirectory() as tmp:
        for kind in cut_in.CUT_IN_KINDS:
            starts[kind] = check_kind(module, cmap, tmp, kind)
        if all(starts.values()):
            check_overrides(module, cmap, tmp, starts)
            check_errors(module, cmap, tmp, starts)
            check_kerb_parking(module, cmap, tmp, starts)
    print("FAILED" if FAILURES else "PASSED")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
