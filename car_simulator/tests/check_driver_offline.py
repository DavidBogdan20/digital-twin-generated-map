#!/usr/bin/env python
"""
Offline check of the cautious ego driver (src/carsim/blocks/driver.py, <ego_control brake="sight"/>), no CARLA server
needed. Checked:

  - ray clearance: an occluder nearer than the target does not let it be seen, a hit on the target alone, no hit at all,
    a hit just inside the margin and hits beyond the target do; the order of the hits does not matter; a hit labelled
    NONE (an invisible volume) in front of the target does not block; can_see needs one clear ray of three
  - the crosser test on the real Town10HD map: on the crosswalk in the road is a crosser, waiting on the kerb, finished
    on the far kerb and in the road 10 m from the crosswalk are not
  - the in-path prediction (a walker walking into the route is, one walking along beside it is not, one standing on it
    is at once) and the speed decision (cruise, stop before the crosswalk, emergency, lead follow, resume when clear)
  - CautiousDriver ticks against a fake world whose rays a wall can block: an occluded child in the path is not braked
    for, the same child in view is (brake 1.0, the log says first sighting and emergency brake), a crosser in the other
    lane stops the ego before the crosswalk and it drives on when the crosser has left the road, a seen actor is
    still known for 2 s behind the wall, a slow lead is followed, update() is always RUNNING
  - round 2: the eye comes from the live ego pose and is never inside its bounding box (a cache 3 m behind), a ray hit
    on the ego's own box does not block, at most MAX_RAYS cast_ray calls a tick (nearest first, actors far off the
    route are not cast at), a hidden actor is carried on from its last seen state (not its live pose), the time window
    (crossed already / arrives after the ego has passed: no brake, in the path when the ego arrives: brake), a full
    brake bites BRAKE_DELAY after the decision, ctx.metrics events (first_visible, brake_onset, emergency,
    emergency_count) are written once with the contract keys; initialise() sets the ego to the cruise speed
  - __init__ touches no world (the actor has no get_world); ego_control: no brake is the WaypointFollower, sight the
    CautiousDriver, an explicit none the CautiousDriver with hazards ignored, an unknown value is a ValueError, a
    manual ego ignores it
  - fix cycle 1: SpeedControl (feed-forward, deadband brake, integral limits, anti-windup, closed loop on a point mass
    calibrated on the live mkz: +-0.3 m/s, no chatter), the counterfactual (hazards=False never looks or brakes), the
    ray budget (least recently looked at first: a crosser behind 6 hidden walkers is known in 3 ticks), brake_onset
    only for a real reaction (lead braking included, with TTC) and first_hazard, no brake release on escalating to an
    emergency, the path width across the route (a turned stopped car), the time window at the resume speed, a
    bicycle riding along the road is no crosser, a late crosser is passed and never held on the crosswalk
  - line of sight on maps where cast_ray sees nothing (CARLA 0.9.15 generated maps): ray_box_entry, BoxRays (nearest hit
    first, labelled), the startup self-test (a labelled hit on the ego's box, not a NONE one, not one elsewhere, not one
    behind a labelled hit, no hit, a RuntimeError) and choose_sight (CARSIM_SIGHT wins); a CautiousDriver whose self-test fails logs the choice and sees
    through the actors' boxes: a child behind a parked truck is not braked for, in view once the truck has gone it is.
    The other driver checks force cast_ray (CARSIM_SIGHT), as their fake world never hits the ego

    python tests/check_driver_offline.py

Exits with 1 if any check fails.
"""

import math
import os
import sys
from collections import namedtuple
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))

import carla  # noqa: E402
import py_trees  # noqa: E402

from check_blocks_offline import FAILURES, FakeActor, check  # noqa: E402
from check_crossings_offline import xodr_path  # noqa: E402
from carsim.blocks import driver  # noqa: E402
from carsim.blocks.ego_control import EgoControl  # noqa: E402
from carsim.road import CrosswalkIndex, crosswalks, find_crossing, lane_point, straight_route  # noqa: E402
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaypointFollower  # noqa: E402
from srunner.scenariomanager.timer import GameTime  # noqa: E402

CRUISE = 30 / 3.6  # m/s
Hit = namedtuple("Hit", "location label")
NONE, TRUCK, PEDESTRIANS = (carla.CityObjectLabel.NONE, carla.CityObjectLabel.Truck,
                               carla.CityObjectLabel.Pedestrians)


def loc(x, y=0.0, z=0.0):
    return carla.Location(x, y, z)


# --- ray clearance -------------------------------------------------------------------------------------------------

def check_rays():
    eye = loc(0, 0, 1.3)
    ahead = lambda d, label=TRUCK: Hit(loc(d, 0, 1.3), label)  # noqa: E731
    check(not driver.ray_clear(eye, [ahead(5)], 10.0), "an occluder at 5 m must hide a target at 10 m")
    check(not driver.ray_clear(eye, [ahead(9.5), ahead(10.2, PEDESTRIANS)], 10.0), "an occluder 0.5 m in front hides")
    check(not driver.ray_clear(eye, [ahead(10.2, PEDESTRIANS), ahead(5)], 10.0), "the order of the hits must not matter")
    check(driver.ray_clear(eye, [ahead(10.1, PEDESTRIANS)], 10.0), "a hit on the target alone must be clear")
    check(driver.ray_clear(eye, [], 10.0), "no hit at all must be clear")
    check(driver.ray_clear(eye, [ahead(9.8)], 10.0), "a hit 0.2 m in front (inside the margin) must be clear")
    check(driver.ray_clear(eye, [ahead(12)], 10.0), "a hit beyond the target must be clear")
    check(driver.ray_clear(eye, [ahead(0.2, NONE), ahead(9.0, NONE), ahead(10.1, PEDESTRIANS)], 10.0),
          "hits labelled NONE in front of the target must not block")
    check(not driver.ray_clear(eye, [ahead(0.2, NONE), ahead(5)], 10.0), "a NONE hit must not hide a real occluder")

    check(abs(driver.box_distance((10, 0, 0), (12, 0, 0), 0.0, (1, 1, 1)) - 1.0) < 1e-9, "box distance along x")
    check(abs(driver.box_distance((10, 0, 0), (10, 1.5, 0), math.pi / 2, (2, 1, 1))) < 1e-9, "inside a turned box is 0")

    class World:
        def __init__(self, script):
            self.script = list(script)

        def cast_ray(self, a, b):
            return self.script.pop(0)

    walker = (loc(10, 0, 0.9), 0.0, (0.2, 0.2, 0.9))
    wall = [Hit(loc(5, 0, 1.0), TRUCK)]
    check(driver.can_see(World([wall, wall, []]), eye, *walker), "one clear ray of three must be enough")
    check(not driver.can_see(World([wall, wall, wall]), eye, *walker), "three blocked rays must hide the target")
    check(driver.can_see(World([[]]), eye, *walker), "a clear first ray must not need the others")

    # a hit on the ego's own box is not an occluder, a real one outside it still is
    ego = Actor("vehicle.lincoln.mkz_2017", 0, 0, extent=(2.4, 1.0, 0.8))
    own, eye = driver.box_of(ego), loc(2.6, 0, 1.3)
    body = Hit(loc(1.0, 0.0, 1.0), carla.CityObjectLabel.Car)
    check(not driver.ray_clear(eye, [body], 10.0), "without the ego's box a hit 1.6 m from the eye hides the target")
    check(driver.ray_clear(eye, [body], 10.0, own), "a hit on the ego's own bounding box must not block")
    check(not driver.ray_clear(eye, [body, Hit(loc(7.6, 0, 1.3), TRUCK)], 10.0, own), "a real occluder still blocks")


# --- the pure decisions, on the real map ---------------------------------------------------------------------------

def make_geometry():
    cmap = carla.Map("Town10HD", xodr_path("Town10HD").read_text(encoding="utf-8"))
    start = cmap.get_waypoint(carla.Location(28.74, -67.91, 0.3))  # the ego start of pedestrian_crossing.xml
    crossing = find_crossing(cmap, start, CrosswalkIndex(crosswalks(cmap)))
    plan = [(wp, None) for wp in straight_route(start, crossing.distance + 50.0)]
    points = [(start.transform.location.x, start.transform.location.y)] + [
        (wp.transform.location.x, wp.transform.location.y) for wp, _ in plan]
    return cmap, start, crossing, plan, driver.Route(points)


def on_driving_lane(cmap, x, y):
    return cmap.get_waypoint(loc(x, y), project_to_road=False, lane_type=carla.LaneType.Driving) is not None


def check_decisions(cmap, crossing, route):
    center, axis, near, far = driver.crosswalk_span(crossing, route)
    check(0 < near < far and far - near < 6.0 and abs(near - crossing.distance) < 6.0,
          f"crosswalk span {near:.1f}..{far:.1f} vs crossing at {crossing.distance:.1f}")

    def crosser(x, y):
        return driver.is_crosser(x, y, on_driving_lane(cmap, x, y), center, axis)

    walk_x = center[0]
    s, offset, _ = route.project(walk_x, crossing.wp.transform.location.y)
    check(abs(s - crossing.distance) < 6.0 and offset < 1.0, "the crossing waypoint must be on the route")
    check(crosser(walk_x, crossing.wp.transform.location.y), "a walker on the crosswalk in the ego lane is a crosser")
    other = next((t for t in range(-9, 10) if on_driving_lane(cmap, walk_x, center[1] + t)
                  and route.project(walk_x, center[1] + t)[1] > 3.0), None)
    check(other is not None, "no other driving lane on the crosswalk")
    check(crosser(walk_x, center[1] + other), "a walker on the crosswalk in another lane is a crosser")
    check(not crosser(crossing.spawn.location.x, crossing.spawn.location.y), "waiting on the kerb is not crossing")
    check(not crosser(crossing.end.x, crossing.end.y), "finished on the far kerb is not crossing")
    check(not crosser(walk_x + 10.0, crossing.wp.transform.location.y), "10 m from the crosswalk is not crossing")

    # in-path prediction: a route point 20 m ahead of the start, the walker 5 m to the side
    x0, y0 = route.points[0][0] - 20.0, route.points[0][1]
    check(driver.predict_conflict(route, (x0, y0 + 4.0), (0.0, -2.5), 2.5) is not None,
          "a walker walking into the route must conflict")
    s_hit, t_hit, direction = driver.predict_conflict(route, (x0, y0 + 4.0), (0.0, -2.5), 2.5)
    check(0.25 <= t_hit <= 2.0 and abs(s_hit - 20.0) < 1.0 and direction[0] < -0.9, f"conflict at s={s_hit:.1f} t={t_hit}")
    check(driver.predict_conflict(route, (x0, y0 + 4.0), (-2.0, 0.0), 2.5) is None,
          "a walker walking along beside the route must not conflict")
    check(driver.predict_conflict(route, (x0, y0), (0.0, 0.0), 2.5)[1] == 0.0, "an actor on the route conflicts at once")
    check(driver.predict_conflict(route, (x0, y0 + 20.0), (0.0, -2.5), 2.5) is None, "a conflict beyond 2 s is none")

    # the speed decision
    d = driver.decide_speed
    check(d(CRUISE, CRUISE, [], []) == (CRUISE, False), "cruise with nothing in the way")
    check(d(CRUISE, CRUISE, [60.0], [])[0] == CRUISE, "a far stop point does not slow the ego yet")
    target, emergency = d(CRUISE, CRUISE, [10.0], [])
    planned = 10.0 - CRUISE * driver.BRAKE_DELAY - driver.STOP_MARGIN  # the delay's coasting and the margin come off the gap
    check(abs(target - math.sqrt(6.0 * planned)) < 1e-9 and not emergency, f"a stop point at 10 m: {target:.2f} {emergency}")
    check(d(0.0, CRUISE, [0.0], []) == (0.0, False), "stopped at the stop point: target 0, no emergency")
    check(d(CRUISE, CRUISE, [3.0], [])[1] and d(CRUISE, CRUISE, [3.0], [])[0] < 1.5, "a stop point nearer than v^2/12 is an emergency (the delay eats the gap)")
    check(d(CRUISE, CRUISE, [], [1.0])[1] and not d(CRUISE, CRUISE, [], [3.0])[1], "TTC 1.0 s brakes, 3.0 s does not")
    check(d(4.0, CRUISE, [], [], lead=(4.0, 8.0)) == (4.0, False), "a lead at the kept gap (4 m + 4 m): its speed")
    check(d(CRUISE, CRUISE, [], [], lead=(4.0, 12.0)) == (4.0 + 0.3 * 4.0, False), "a lead 4 m too far: a bit faster")
    check(d(CRUISE, CRUISE, [], [], lead=(4.0, 30.0)) == (CRUISE, False), "a lead far ahead: cruise (catch up)")
    close = d(CRUISE, CRUISE, [], [], lead=(4.0, 6.0))[0]
    check(abs(close - 3.4) < 1e-9, f"a lead closer than its gap: slower than it ({close:.2f})")
    check(d(4.0, CRUISE, [], [], lead=(4.0, 0.0))[0] == 1.6, "a lead touching: still a soft target, no stop point")
    check(d(CRUISE, CRUISE, [], [], lead=(4.0, 5.0)) == (4.0 - 0.9, True), "a lead 5 m ahead at 4 m/s closing: emergency")
    check(not d(4.0, CRUISE, [], [], lead=(4.0, 5.0))[1], "following at its speed, even closer than kept: no emergency")
    check(d(CRUISE, CRUISE, [], [], lead=(10.0, 30.0))[0] == CRUISE, "a faster lead does not slow the ego")
    check(d(CRUISE, CRUISE, [], [])[0] == CRUISE, "clear again: cruise (resume)")
    return center, axis, near, other


# --- CautiousDriver against fakes ----------------------------------------------------------------------------------

class Actor(FakeActor):
    """A FakeActor with the bounding box, velocity and controls the driver reads."""

    def __init__(self, type_id, x, y, yaw=0.0, vx=0.0, vy=0.0, extent=(0.2, 0.2, 0.9), attributes=None):
        super().__init__(type_id, carla.Transform(loc(x, y), carla.Rotation(yaw=yaw)))
        self.velocity = carla.Vector3D(vx, vy, 0.0)
        self.bounding_box = carla.BoundingBox(loc(0, 0, extent[2]), carla.Vector3D(*extent))
        self.attributes = attributes or {}
        self.control = None
        self.target_velocity = None  # what set_target_velocity was given

    def get_velocity(self):
        return self.velocity

    def apply_control(self, control):
        self.control = control


class World:
    """cast_ray: an invisible NONE volume 0.2 m from the eye, and a vehicle hit at 5 m for every aim near a hidden spot."""

    def __init__(self):
        self.hidden = []  # (x, y) spots behind the occluder
        self.eyes, self.aims = [], []  # every cast_ray call

    def cast_ray(self, eye, aim):
        self.eyes.append(eye)
        self.aims.append(aim)
        hits = [Hit(loc(eye.x, eye.y, eye.z), NONE)]
        if any(math.hypot(aim.x - x, aim.y - y) < 2.0 for x, y in self.hidden):
            hits.append(Hit(loc(eye.x + 0.5 * (aim.x - eye.x), eye.y + 0.5 * (aim.y - eye.y), eye.z), TRUCK))
        return hits


class Ctx:
    """The parts of BlockContext the driver and ego_control read."""

    def __init__(self, cmap, start, crossing, plan, ego, uses_crossing=True):
        self.world, self.map, self.ego_start_wp, self.crossing, self.uses_crossing = World(), cmap, start, crossing, uses_crossing
        self.plan, self.ego, self.things, self.speed, self.endless = plan, ego, [], 0.0, False
        self.lines, self.target, self.metrics, self.cached = [], None, {}, None

    def ego_plan(self):
        return self.plan

    def ego_transform(self):
        return self.cached or self.ego.get_transform()  # the cache may lag the live pose

    def ego_forward_speed(self):
        return self.speed

    def solid_things(self):
        return [(a, "walker" if a.type_id.startswith("walker") else "vehicle", a.get_location(), 1.0)
                for a in self.things]

    def log(self, who, message):
        self.lines.append(message)


class Planner:
    """LocalPlanner stand-in: remembers the speed it was given (km/h) and drives at full throttle."""
    created = []

    def __init__(self, vehicle, opt_dict=None):
        self.opt, self.speed, self.plan, self.released = opt_dict, None, None, False
        Planner.created.append(self)

    def set_global_plan(self, plan):
        self.plan = plan

    def set_speed(self, kmh):
        self.speed = kmh

    def run_step(self, debug=False):
        return carla.VehicleControl(throttle=0.5, brake=0.0)

    def reset_vehicle(self):
        self.released = True


class Clock:
    now = 0.0


def make_driver(cmap, start, crossing, plan, s_ego, speed, uses_crossing=True, hazards=True):
    waypoint = lane_point(start, s_ego)
    transform = waypoint.transform
    ego = Actor("vehicle.lincoln.mkz_2017", transform.location.x, transform.location.y, transform.rotation.yaw,
                extent=(2.4, 1.0, 0.8))
    ctx = Ctx(cmap, start, crossing, plan, ego, uses_crossing)
    ctx.speed = speed
    behaviour = driver.CautiousDriver(ctx, CRUISE, hazards=hazards)
    with mock.patch.object(driver, "LocalPlanner", Planner):
        behaviour.initialise()
    return ctx, behaviour


def tick(behaviour, ctx, dt=0.05):
    Clock.now += dt
    with mock.patch.object(GameTime, "get_time", staticmethod(lambda: Clock.now)):
        status = behaviour.update()
    check(status == py_trees.common.Status.RUNNING, f"update() returned {status}")
    return ctx.ego.control


def tick_for(behaviour, ctx, seconds, dt=0.05):
    for _ in range(round(seconds / dt)):
        control = tick(behaviour, ctx, dt)
    return control


def lane_xy(start, s, crossing):
    wp = lane_point(start, s)
    return wp.transform.location.x, wp.transform.location.y


def check_driver(cmap, start, crossing, plan, route, center, axis, near, other):
    # __init__ touches no world: the actor has no get_world
    ego = Actor("vehicle.lincoln.mkz_2017", 0, 0)
    check(not hasattr(ego, "get_world"), "the fake ego must have no get_world")
    behaviour = driver.CautiousDriver(Ctx(cmap, start, crossing, plan, ego), CRUISE)
    check(behaviour.name == "EgoCautiousDriver", f"name {behaviour.name}")

    # an occluded child in the path is not braked for, the same child in view is
    ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE)
    check(Planner.created[-1].opt["target_speed"] == CRUISE * 3.6 and Planner.created[-1].plan is plan,
          "the planner is made with the cruise speed in km/h and the plan")
    cx, cy = lane_xy(start, 22.0, crossing)  # 12 m ahead of the ego, on its lane
    child = Actor("walker.pedestrian.0001", cx, cy, yaw=90.0, vy=0.0)
    ctx.things = [child]
    ctx.world.hidden = [(cx, cy)]
    control = tick(behaviour, ctx)
    check(control.brake == 0.0 and Planner.created[-1].speed == CRUISE * 3.6, "an occluded child must not be braked for")
    check(not [line for line in ctx.lines if "test speed" not in line and "line of sight" not in line],
          f"nothing to log for an unseen child: {ctx.lines}")
    ctx.world.hidden = []
    control = tick_for(behaviour, ctx, driver.BRAKE_DELAY + 0.05)  # the decision, then the brake delay
    check(control.brake == 1.0 and control.throttle == 0.0, f"a visible child in the path: brake {control.brake}")
    check(any("first visible" in line for line in ctx.lines) and any("hazard" in line for line in ctx.lines)
          and any("emergency brake" in line for line in ctx.lines), f"log: {ctx.lines}")
    # it stays known for 2 s behind the wall, then it is forgotten
    ctx.world.hidden = [(cx, cy)]
    Clock.now += 1.0
    check(tick(behaviour, ctx).brake == 1.0, "a child seen 1 s ago is still known")
    Clock.now += 1.5
    control = tick(behaviour, ctx)
    check(control.brake == 0.0 and any("clear, driving on" in line for line in ctx.lines),
          f"a child unseen for 2.5 s is forgotten: brake {control.brake}, {ctx.lines}")
    check(len([line for line in ctx.lines if "hazard" in line]) == 1, "the hazard is logged once")

    # a crosser in another lane: the ego stops before the crosswalk, and drives on when it has left the road
    gap_to_stop = 10.0
    s_ego = near - driver.CROSSER_STOP - gap_to_stop - 2.4  # the bumper is 2.4 m ahead of the ego's origin
    ctx, behaviour = make_driver(cmap, start, crossing, plan, s_ego, 8.0)
    walker = Actor("walker.pedestrian.0002", center[0], center[1] + other, yaw=90.0)
    ctx.things = [walker]
    control = tick(behaviour, ctx)
    want = math.sqrt(2 * driver.COMFORT_DECEL * (gap_to_stop - 8.0 * driver.BRAKE_DELAY - driver.STOP_MARGIN)) * 3.6
    check(abs(Planner.created[-1].speed - want) < 0.5 and 0.0 < control.brake < 1.0 and
          any("stopping for crosser" in line for line in ctx.lines),
          f"a crosser: planner speed {Planner.created[-1].speed} want {want:.1f}, {ctx.lines}")
    ctx.speed = 0.0
    ctx.ego.transform = lane_point(start, near - driver.CROSSER_STOP - 2.0).transform  # at the stop point, 0.4 m past it
    control = tick(behaviour, ctx)
    check(control.brake == 1.0 and control.throttle == 0.0, "held at the stop point while the crosser is on the road")
    walker.transform = carla.Transform(crossing.end, walker.transform.rotation)
    control = tick(behaviour, ctx)
    check(control.brake == 0.0 and Planner.created[-1].speed == CRUISE * 3.6 and
          any("clear, driving on" in line for line in ctx.lines), f"the crosser left: brake {control.brake}")

    # no crossing in the scenario: a walker on the crosswalk in another lane is no reason to stop
    ctx, behaviour = make_driver(cmap, start, crossing, plan, s_ego, 8.0, uses_crossing=False)
    ctx.things = [Actor("walker.pedestrian.0002", center[0], center[1] + other, yaw=90.0)]
    tick(behaviour, ctx)
    check(Planner.created[-1].speed == CRUISE * 3.6, "without a crossing there is no crosser")

    # a slow lead vehicle in the lane is followed, and the ego resumes when it is gone
    ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE)
    lx, ly = lane_xy(start, 24.0, crossing)  # ~9 m of gap
    lead = Actor("vehicle.audi.a2", lx, ly, yaw=180.0, vx=-4.0, extent=(2.3, 0.9, 0.7))
    ctx.things = [lead]
    control = tick(behaviour, ctx)
    check(4.0 * 3.6 - 0.5 < Planner.created[-1].speed < 6.0 * 3.6 and control.throttle == 0.0 and control.brake > 0.0 and
          not any("emergency" in line for line in ctx.lines),
          f"follow a lead at 4 m/s: planner speed {Planner.created[-1].speed}, brake {control.brake}")
    ctx.things = []
    Clock.now += 3.0
    tick(behaviour, ctx)
    check(Planner.created[-1].speed == CRUISE * 3.6, "the lead is gone: cruise again")

    # a bicycle is a crosser too (it is a vehicle whose base_type says so)
    ctx, behaviour = make_driver(cmap, start, crossing, plan, s_ego, 8.0)
    ctx.things = [Actor("vehicle.bh.crossbike", center[0], center[1] + other, yaw=90.0, vy=2.0,
                        attributes={"base_type": "bicycle"})]
    tick(behaviour, ctx)
    check(any("stopping for crosser" in line for line in ctx.lines), f"a crossing bicycle: {ctx.lines}")

    behaviour.terminate(py_trees.common.Status.INVALID)
    check(Planner.created[-1].released and ctx.ego.control.throttle == 0.0 and ctx.ego.control.brake == 0.0,
          "terminate releases the planner and the controls")


def check_ego_control(cmap, start, crossing, plan):
    def build(params):
        ctx = Ctx(cmap, start, crossing, plan, Actor("vehicle.lincoln.mkz_2017", 0, 0))
        return EgoControl(ctx, params)

    plain = build({"mode": "autopilot", "target_speed": "30"}).from_start()
    check(len(plain) == 1 and isinstance(plain[0], WaypointFollower) and plain[0].name == "EgoAutopilot",
          "autopilot without brake is today's WaypointFollower")
    none = build({"mode": "autopilot", "target_speed": "30", "brake": "none"}).from_start()
    check(len(none) == 1 and isinstance(none[0], driver.CautiousDriver) and not none[0].hazards
          and not isinstance(none[0], WaypointFollower), "an explicit brake none is the CautiousDriver, hazards ignored")
    check(build({"brake": "sight"}).from_start()[0].hazards, "brake sight looks")
    ego_block = build({"mode": "autopilot", "target_speed": "23"})
    with mock.patch.object(WaypointFollower, "initialise", lambda self: True):
        ego_block.from_start()[0].initialise()
    v, forward = ego_block.ctx.ego.target_velocity, ego_block.ctx.ego.get_transform().get_forward_vector()
    check(v is not None and abs(v.x - forward.x * 23 / 3.6) < 1e-3 and abs(v.y - forward.y * 23 / 3.6) < 1e-3,
          f"the autopilot ego starts at the test speed too: {v}")
    sight = build({"mode": "autopilot", "target_speed": "30", "brake": "sight"}).from_start()
    check(len(sight) == 1 and isinstance(sight[0], driver.CautiousDriver) and abs(sight[0].speed - CRUISE) < 1e-9,
          "brake sight is the CautiousDriver at the target speed")
    try:
        build({"brake": "fast"})
        check(False, "an unknown brake value must be a ValueError")
    except ValueError as error:
        check("brake" in str(error) and "sight" in str(error), f"brake error message: {error}")
    manual = build({"mode": "manual", "brake": "sight"})
    check(manual.from_start() == [] and manual.ctx.endless, "a manual ego ignores brake")
    # cleanup() removes the chase view's camera on the ego (before ScenarioRunner destroys the ego)
    from carsim.blocks import ego_control
    with mock.patch.object(ego_control, "destroy_sensors_on", return_value=1) as destroy:
        manual.cleanup()
    check(destroy.call_args[0] == (manual.ctx.world, manual.ctx.ego) and any("chase view" in l for l in manual.ctx.lines),
          f"ego_control cleanup: {destroy.call_args}")


# --- closed loop: following a slow cyclist ------------------------------------------------------------------------

def ego_accel(v, throttle, brake):
    """
    The point-mass ego of the closed loop, calibrated on the live mkz held in 2nd gear: a steady speed needs throttle
    0.34 + 0.013 v (0.38 at 4 m/s, 0.41 at 6.4, 0.45 at 8.4), 8 m/s^2 per unit of throttle above it, and 8 per unit brake.
    """
    return 8.0 * (throttle - (0.34 + 0.013 * v)) - 8.0 * brake if v > 0.0 or throttle > 0.34 else 0.0


def follow_run(lead_speed=4.0, wobble=0.0, edge=0.0, seconds=30.0, ego_speed=CRUISE, gap0=12.0, cruise=CRUISE):
    """A point-mass ego (see ego_accel) behind a cyclist on a straight road, 20 Hz. gap0 > RANGE: no lead in sight."""
    dt, ego_half, lead_half = 0.05, 2.4, 0.9
    wp = lambda x: (type("WP", (), {"transform": carla.Transform(loc(x, 0.0, 0.3))})(), None)  # noqa: E731
    ctx = Ctx(None, wp(0.0)[0], None, [wp(x) for x in range(2, 400, 2)], None, uses_crossing=False)
    ctx.ego = Actor("vehicle.lincoln.mkz_2017", 0.0, 0.0, extent=(ego_half, 1.0, 0.8))
    lead = Actor("vehicle.bh.crossbike", ego_half + gap0 + lead_half, 0.0, vx=lead_speed, extent=(lead_half, 0.3, 0.9),
                 attributes={"base_type": "bicycle"})
    ctx.things = [lead]
    sim = {"v": ego_speed, "x": 0.0}
    behaviour = driver.CautiousDriver(ctx, cruise)
    with mock.patch.object(driver, "LocalPlanner", Planner):
        behaviour.initialise()
    rows, x_lead = [], lead.get_location().x
    for i in range(int(seconds / dt)):
        t = (i + 1) * dt
        x_lead += lead_speed * dt
        lead.transform = carla.Transform(loc(x_lead, edge + wobble * math.sin(t * 2.0)))  # sways across the path edge
        ctx.speed = sim["v"]
        control = tick(behaviour, ctx, dt)
        sim["v"] = max(0.0, sim["v"] + ego_accel(sim["v"], control.throttle, control.brake) * dt)
        sim["x"] += sim["v"] * dt
        ctx.ego.transform = carla.Transform(loc(sim["x"], 0.0))
        rows.append((t, sim["v"], x_lead - lead_half - (sim["x"] + ego_half), control))
    return rows, ctx


def brake_throttle_switches(rows, after):
    states = ["b" if c.brake > 0.05 else "t" for t, _, _, c in rows if t > after and (c.brake > 0.05 or c.throttle > 0.05)]
    return sum(a != b for a, b in zip(states, states[1:]))


def check_crosser_stop(cmap, start, crossing, plan, center, other, near):
    """Closed loop on the real route: a crosser in the road 40 m ahead, the ego stops before its stop point, no emergency."""
    stop_line = near - driver.CROSSER_STOP
    for kmh in (23, 30):
        Clock.now = 0.0
        s0 = stop_line - 2.4 - 40.0
        ctx, behaviour = make_driver(cmap, start, crossing, plan, s0, kmh / 3.6, uses_crossing=True)
        ctx.things = [Actor("walker.pedestrian.0002", center[0], center[1] + other, yaw=90.0)]
        sim = {"v": kmh / 3.6}
        with mock.patch.object(driver, "LocalPlanner", Planner):
            behaviour = driver.CautiousDriver(ctx, kmh / 3.6)
            behaviour.initialise()
        s, front_max = s0, s0 + 2.4
        for _ in range(int(30.0 / 0.05)):
            ctx.speed = sim["v"]
            control = tick(behaviour, ctx)
            sim["v"] = max(0.0, sim["v"] + ego_accel(sim["v"], control.throttle, control.brake) * 0.05)
            s += sim["v"] * 0.05
            ctx.ego.transform = lane_point(start, s).transform
            front_max = max(front_max, s + 2.4)
        check(not any("emergency" in line for line in ctx.lines) and any("stopping for crosser" in line for line in ctx.lines),
              f"crosser stop from {kmh} km/h: log {ctx.lines}")
        check(front_max <= stop_line and sim["v"] < 0.05,
              f"crosser stop from {kmh} km/h: front reaches {front_max - stop_line:+.2f} m of the stop point, v {sim['v']:.2f}")


def check_follow():
    for edge, wobble in ((0.0, 0.0), (1.7, 0.25)):  # the second sways about the edge of the ego's path corridor (1.8 m)
        Clock.now = 0.0
        rows, ctx = follow_run(edge=edge, wobble=wobble)
        tail = [r for r in rows if r[0] > 10.0]
        tag = f"cyclist at 4 m/s ({edge} m off, sway {wobble} m)"
        check(min(r[2] for r in rows) > 1.0, f"{tag}: the closest gap {min(r[2] for r in rows):.1f} m")
        check(all(abs(r[1] - 4.0) < 0.3 for r in tail),
              f"{tag}: speed after 10 s within 4 +- 0.3, got {min(r[1] for r in tail):.2f}..{max(r[1] for r in tail):.2f}")
        check(all(abs(r[2] - 8.0) < 2.0 for r in tail),
              f"{tag}: gap after 10 s near 8 m, got {min(r[2] for r in tail):.1f}..{max(r[2] for r in tail):.1f}")
        switches = brake_throttle_switches(rows, 10.0)
        check(switches <= 2, f"{tag}: {switches} brake/throttle switches after 10 s")
        check(not any("emergency" in line for line in ctx.lines), f"{tag}: log {ctx.lines}")

    # no hazard: the ego holds the test speed (live: within +-0.1 m/s, the LocalPlanner PID swung 5.0 to 7.7 around 6.4)
    for kmh in (20, 23, 25):
        Clock.now = 0.0
        rows, ctx = follow_run(gap0=200.0, seconds=20.0, ego_speed=kmh / 3.6, cruise=kmh / 3.6)
        tail = [r[1] for r in rows if r[0] > 3.0]
        check(all(abs(v - kmh / 3.6) < 0.3 for v in tail),
              f"cruise hold at {kmh} km/h: {min(tail):.2f}..{max(tail):.2f} m/s, want {kmh / 3.6:.2f} +- 0.3")

    # a parked car pulling out (barely moving) first seen 7 m ahead at 30 km/h is still an emergency brake
    Clock.now = 0.0
    rows, ctx = follow_run(lead_speed=0.0, gap0=7.0, seconds=0.6)
    first = next((r[0] for r in rows if r[3].brake == 1.0), None)  # the brake bites BRAKE_DELAY after the decision
    check(any("emergency brake" in line for line in ctx.lines) and rows[0][3].brake == 0.0
          and first is not None and abs(first - rows[0][0] - driver.BRAKE_DELAY) < 0.06,
          f"a stationary car 7 m ahead at 30 km/h: emergency, brake at {first}, {ctx.lines}")


# --- round 2: eye, ray budget, last seen state, time window, brake delay, metrics ---------------------------------

def frame(start, s):
    """(x, y, forward, left, yaw [deg]) of the ego lane at route distance s."""
    t = lane_point(start, s).transform
    yaw = math.radians(t.rotation.yaw)
    return t.location.x, t.location.y, (math.cos(yaw), math.sin(yaw)), (-math.sin(yaw), math.cos(yaw)), t.rotation.yaw


def walker_at(start, s, lateral, v_lateral=0.0, type_id="walker.pedestrian.0001"):
    """A walker at route distance s, `lateral` m left of the lane, walking `v_lateral` m/s to the left."""
    x, y, _, left, yaw = frame(start, s)
    return Actor(type_id, x + lateral * left[0], y + lateral * left[1], yaw=yaw + 90.0,
                 vx=v_lateral * left[0], vy=v_lateral * left[1])


def check_start_speed(cmap, start, crossing, plan):
    ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False)
    forward = ctx.ego.get_transform().get_forward_vector()
    v = ctx.ego.target_velocity
    check(v is not None and abs(v.x - forward.x * CRUISE) < 1e-3 and abs(v.y - forward.y * CRUISE) < 1e-3
          and abs(v.z - forward.z * CRUISE) < 1e-3, f"the ego is set to the cruise speed along its heading: {v}")
    check(sum("test speed" in line for line in ctx.lines) == 1, f"logged once: {ctx.lines}")
    tick(behaviour, ctx)
    check(sum("test speed" in line for line in ctx.lines) == 1, "not logged again on the ticks")


def check_eye_and_budget(cmap, start, crossing, plan):
    # the eye comes from the live pose: the cache is 3 m behind, and an eye from it would be inside the ego's box
    ctx, behaviour = make_driver(cmap, start, crossing, plan, 20.0, CRUISE, uses_crossing=False)
    stale = lane_point(start, 17.0).transform
    ctx.cached = stale
    ctx.things = [walker_at(start, 35.0, 0.0)]
    tick(behaviour, ctx)
    center, yaw, extent = driver.box_of(ctx.ego)
    inside = lambda eye: driver.box_distance((eye.x, eye.y, eye.z), (center.x, center.y, center.z), yaw, extent) == 0.0  # noqa: E731
    stale_eye = driver.eye_position(stale.location, math.radians(stale.rotation.yaw), 0.0, extent[0])
    check(inside(stale_eye), "an eye from the lagging cache would be inside the ego's box (the test is sensitive)")
    check(ctx.world.eyes and not any(inside(eye) for eye in ctx.world.eyes),
          f"the eye must be outside the live ego box: {len(ctx.world.eyes)} rays")
    check(abs(ctx.world.eyes[0].z - ctx.ego.get_transform().location.z - driver.EYE_HEIGHT) < 1e-3, "eye height 1.3 m")

    # ray budget: 30 hidden walkers in the lane ahead, at most MAX_RAYS calls, the nearest first
    ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False)
    ctx.things = [walker_at(start, 14.0 + i, 0.0) for i in range(30)]
    ctx.world.hidden = [(a.get_location().x, a.get_location().y) for a in ctx.things]
    tick(behaviour, ctx)
    near = ctx.things[0].get_location()
    check(len(ctx.world.aims) == driver.MAX_RAYS, f"{len(ctx.world.aims)} rays for 30 hidden walkers, cap {driver.MAX_RAYS}")
    check(math.hypot(ctx.world.aims[0].x - near.x, ctx.world.aims[0].y - near.y) < 0.1, "the nearest candidate is cast at first")
    ctx.world.hidden, ctx.world.aims = [], []
    tick(behaviour, ctx)
    check(0 < len(ctx.world.aims) <= driver.MAX_RAYS, f"{len(ctx.world.aims)} rays for 30 visible walkers")

    # an actor far off the route (and not moving towards it) could not matter: no ray
    ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False)
    ctx.things = [walker_at(start, 25.0, 40.0)]
    tick(behaviour, ctx)
    check(not ctx.world.aims and not behaviour._known, "a walker 40 m off the route is not looked at")


def check_window_and_memory(cmap, start, crossing, plan):
    w = driver.in_time_window
    check(w(0.0, math.inf, 12.0, 5.2, CRUISE), "an actor in the path for good conflicts")
    check(not w(0.0, 0.25, 12.0, 5.2, CRUISE), "an actor that leaves the path long before the ego arrives does not")
    check(not w(1.75, 4.0, 1.0, 5.2, CRUISE), "an actor that enters after the ego's tail has passed does not")
    check(w(1.25, 3.5, 12.0, 5.2, CRUISE), "an actor in the path when the ego arrives conflicts")
    check(w(0.0, math.inf, 5.0, 5.2, 0.0), "a stopped ego (taken at WINDOW_MIN_SPEED) in front of a blocked path conflicts")

    def run(s_child, lateral, v_lateral, seconds=0.5):
        ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False)
        ctx.things = [walker_at(start, s_child, lateral, v_lateral)]
        control = tick_for(behaviour, ctx, seconds)
        return ctx, behaviour, control

    # the bumper is at route distance 12.4, so s = 24.6 is 12 m (centre minus the walker's 0.2 half size) ahead at 8.3 m/s
    ctx, _, control = run(24.6, 1.5, 1.4)  # still at the path edge, walking away: gone before the ego arrives
    check(control.brake == 0.0 and Planner.created[-1].speed == CRUISE * 3.6 and not any("hazard" in l for l in ctx.lines),
          f"a child that has crossed already: brake {control.brake}, {ctx.lines}")
    ctx, _, control = run(13.6, 4.0, -1.5)  # enters the path 1.75 s from now, the ego has passed it after 0.7 s
    check(control.brake == 0.0 and Planner.created[-1].speed == CRUISE * 3.6 and not any("hazard" in l for l in ctx.lines),
          f"a child that enters after the ego has passed: brake {control.brake}, {ctx.lines}")
    ctx, _, control = run(24.6, 3.5, -1.5, seconds=driver.BRAKE_DELAY + 0.1)  # in the path 1.25 to 3.5 s, the ego is there at 1.4 s
    check(control.brake == 1.0 and any("hazard" in l for l in ctx.lines), f"a child in the path when the ego arrives: {ctx.lines}")

    # a hidden actor is carried on from its last seen state: the live pose (moved to a harmless spot) is not read
    ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False)
    child = walker_at(start, 24.6, 3.5, -1.5)
    ctx.things = [child]
    tick(behaviour, ctx)
    seen = dict(behaviour._known[child.id])
    live = walker_at(start, 24.6, 9.0)  # 9 m to the side, standing: harmless, and a candidate (within CANDIDATE_BAND)
    child.transform, child.velocity = live.transform, live.velocity
    ctx.world.hidden = [(live.get_location().x, live.get_location().y)]
    control = tick_for(behaviour, ctx, driver.BRAKE_DELAY + 0.1)
    check(behaviour._known[child.id]["center"] == seen["center"] and behaviour._known[child.id]["vel"] == seen["vel"],
          "the last seen state is kept while the actor is hidden")
    check(control.brake == 1.0 and not any("clear, driving on" in l for l in ctx.lines),
          f"the hidden child is still braked for by its extrapolated track, not its live pose: {ctx.lines}")


def check_delay_and_metrics(cmap, start, crossing, plan):
    ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False)
    child = walker_at(start, 22.0, 0.0)  # 12 m ahead of the ego's origin, 9.4 m bumper to surface, standing in the lane
    ctx.things, ctx.target = [child], child
    brakes = []
    for _ in range(10):
        control = tick(behaviour, ctx)
        brakes.append((Clock.now, control.brake))
    decided, applied = brakes[0][0], next(t for t, b in brakes if b == 1.0)
    check(brakes[0][1] == 0.0 and abs(applied - decided - driver.BRAKE_DELAY) < 0.06,
          f"the full brake bites {applied - decided:.2f} s after the decision, not at once")
    m = ctx.metrics
    check(set(m) == {"first_visible", "first_hazard", "brake_onset", "emergency", "emergency_count"}, f"metrics keys {sorted(m)}")
    check(set(m["first_hazard"]) == {"t", "actor", "kind", "distance", "ttc"} and m["first_hazard"]["actor"] == child.id
          and m["first_hazard"]["t"] == m["first_visible"]["t"], f"first_hazard {m['first_hazard']}")
    check(set(m["first_visible"]) == {"t", "actor", "kind", "distance", "ttc"} and m["first_visible"]["actor"] == child.id
          and m["first_visible"]["kind"] == "walker" and abs(m["first_visible"]["distance"] - 9.4) < 0.2
          and m["first_visible"]["ttc"] is not None, f"first_visible {m['first_visible']}")
    check(set(m["emergency"]) == {"t", "speed", "distance"} and m["emergency"]["t"] == decided
          and m["emergency"]["speed"] == CRUISE, f"emergency {m['emergency']}")
    check(set(m["brake_onset"]) == {"t", "speed", "ttc", "distance"} and abs(m["brake_onset"]["t"] - applied) < 1e-9
          and m["brake_onset"]["ttc"] is not None and m["emergency_count"] == 1, f"brake_onset {m['brake_onset']}")
    check(sum("first visible" in l for l in ctx.lines) == 1 and sum("hazard" in l for l in ctx.lines) == 1, f"log {ctx.lines}")
    first = {key: dict(value) for key, value in m.items() if isinstance(value, dict)}
    tick_for(behaviour, ctx, 1.0)
    check({key: m[key] for key in first} == first and m["emergency_count"] == 1, "the events are written once")


# --- fix cycle 1: speed control, counterfactual, ray budget, brake onset, escalation, path width, window, crossers ---

def check_speed_control():
    sc = driver.SpeedControl
    c = sc()
    throttle, brake = c.step(CRUISE, CRUISE, 0.05)
    check(abs(throttle - driver.FF_THROTTLE * CRUISE) < 1e-9 and brake == 0.0, f"on target: the feed-forward throttle only, {throttle}")
    check(c.step(CRUISE, CRUISE - 3.0, 0.05) == (1.0, 0.0), "far below the target: full throttle, no brake")
    check(c.step(CRUISE, CRUISE + 0.2, 0.05)[1] == 0.0 and c.step(CRUISE, CRUISE + 0.2, 0.05)[0] > 0.0,
          "a little over the target (inside the deadband): backs off the throttle, no brake")
    check(c.step(CRUISE, CRUISE + 1.3, 0.05)[0] == 0.0
          and abs(c.step(CRUISE, CRUISE + 1.3, 0.05)[1] - driver.BRAKE_GAIN * (1.3 - driver.BRAKE_DEADBAND)) < 1e-9,
          "over the deadband: brake in proportion, no throttle")
    check(c.step(CRUISE, CRUISE + 9.0, 0.05) == (0.0, 1.0), "far over: full brake")
    c = sc()
    for _ in range(1200):
        c.step(10.0, 9.7, 0.05)  # a little under the target for a minute: the integral grows to its limit, not beyond
    check(0.0 < c.integral <= driver.INTEGRAL_MAX + 1e-9, f"the integral is limited: {c.integral}")
    c = sc()
    for _ in range(1200):
        c.step(40.0, 39.8, 0.05)  # the throttle is saturated (feed-forward 2.4): the integral must not wind up
    check(c.integral == 0.0, f"no windup while saturated: {c.integral}")
    c = sc()
    for _ in range(100):
        c.step(CRUISE, CRUISE - 2.0, 0.05)  # a start-up dip (error beyond INTEGRAL_BAND): nothing is collected
    check(c.integral == 0.0, f"nothing collected beyond the band: {c.integral}")
    # closed loop on the calibrated point mass: from a standing start, within +-0.3 once settled, no brake and no throttle chatter
    sim, c, rows = {"v": 0.0}, sc(), []
    for i in range(int(40.0 / 0.05)):
        throttle, brake = c.step(CRUISE, sim["v"], 0.05)
        sim["v"] = max(0.0, sim["v"] + ego_accel(sim["v"], throttle, brake) * 0.05)
        rows.append((i * 0.05, sim["v"], throttle, brake))
    tail = [r for r in rows if r[0] > 20.0]
    check(all(abs(r[1] - CRUISE) < 0.3 for r in tail) and not any(r[3] > 0.0 for r in tail),
          f"cruise from a standing start: {min(r[1] for r in tail):.2f}..{max(r[1] for r in tail):.2f} m/s")


def check_counterfactual(cmap, start, crossing, plan):
    ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False, hazards=False)
    cx, cy = lane_xy(start, 22.0, crossing)
    ctx.things = [Actor("walker.pedestrian.0001", cx, cy, yaw=90.0)]
    control = tick_for(behaviour, ctx, 1.0)
    check(control.brake == 0.0 and control.throttle > 0.0 and not ctx.world.aims and not ctx.metrics
          and not any("hazard walker" in line or "emergency" in line for line in ctx.lines),
          f"brake none: a child standing in the lane 9 m ahead is not looked at, never braked for: {ctx.lines}, {ctx.metrics}")
    check(any("hazards ignored" in line for line in ctx.lines), f"the log says hazards are ignored: {ctx.lines}")


def check_ray_budget(cmap, start, crossing, plan, center, other, near):
    # 6 walkers hidden in the lane ahead cost 18 rays (the cap is 12): nearest first starved the visible crosser further
    # off for good, with the least recently looked at first it is known within a few ticks
    s_ego = near - driver.CROSSER_STOP - 30.0 - 2.4
    ctx, behaviour = make_driver(cmap, start, crossing, plan, s_ego, CRUISE)
    ctx.things = [walker_at(start, s_ego + 8.0 + i, 0.0) for i in range(6)]
    ctx.world.hidden = [(a.get_location().x, a.get_location().y) for a in ctx.things]
    crosser = Actor("walker.pedestrian.0002", center[0], center[1] + other, yaw=90.0)
    ctx.things.append(crosser)
    for n in range(1, 6):
        tick(behaviour, ctx)
        if crosser.id in behaviour._known:
            break
    check(crosser.id in behaviour._known and n <= 3, f"the crosser behind 6 hidden walkers is known after {n} ticks")
    check(any("stopping for crosser" in line for line in ctx.lines), f"and the ego stops for it: {ctx.lines}")


def check_brake_onset():
    Clock.now = 0.0
    rows, ctx = follow_run(lead_speed=4.0, gap0=12.0, seconds=6.0)  # braking for a lead writes brake_onset, with the TTC to it
    onset = ctx.metrics.get("brake_onset")
    check(onset is not None and onset["ttc"] is not None and onset["ttc"] > 0.0 and onset["t"] > 0.0
          and set(onset) == {"t", "speed", "ttc", "distance"}, f"a lead braking writes brake_onset with a TTC: {onset}")
    check("first_hazard" in ctx.metrics and ctx.metrics["first_hazard"]["kind"] == "bicycle",
          f"first_hazard {ctx.metrics.get('first_hazard')}")
    Clock.now = 0.0
    rows, ctx = follow_run(gap0=200.0, seconds=3.0, ego_speed=CRUISE + 3.0)  # a brake with no hazard is no onset
    check(any(r[3].brake >= 0.2 for r in rows) and "brake_onset" not in ctx.metrics and "first_hazard" not in ctx.metrics,
          f"a brake without a hazard is not a brake onset: {ctx.metrics}")


def check_escalation(cmap, start, crossing, plan):
    # braking for a slow lead, then it stops dead 4 m ahead of its old place: the emergency must not let go of the brake
    Clock.now = 0.0
    ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False)
    lx, ly = lane_xy(start, 24.0, crossing)
    lead = Actor("vehicle.audi.a2", lx, ly, yaw=180.0, vx=-4.0, extent=(2.3, 0.9, 0.7))
    ctx.things = [lead]
    check(tick_for(behaviour, ctx, 0.3).brake >= 0.2, "braking for the slow lead")
    lx, ly = lane_xy(start, 20.0, crossing)
    lead.transform, lead.velocity = carla.Transform(loc(lx, ly), carla.Rotation(yaw=180.0)), carla.Vector3D(0.0, 0.0, 0.0)
    brakes = [tick(behaviour, ctx).brake for _ in range(8)]
    check(any("emergency brake" in line for line in ctx.lines) and min(brakes) >= 0.2 and brakes[-1] == 1.0,
          f"escalating to an emergency keeps braking: {brakes}, {ctx.lines}")


def check_path_width_and_window(cmap, start, crossing, plan, route):
    # C1: a car stopped across the route (turned 90 degrees), its centre 3.0 m off it: its nose is in the path
    x0, y0 = route.points[0][0] - 20.0, route.points[0][1]
    box = (math.pi / 2, (2.3, 0.9, 1.0))
    reach = 1.0 + driver.PATH_MARGIN
    check(driver.predict_conflict(route, (x0, y0 + 3.0), (0.0, 0.0), reach) is None
          and driver.predict_conflict(route, (x0, y0 + 3.0), (0.0, 0.0), reach, box) is not None,
          "a turned car's length across the route counts (its width alone would miss it)")
    check(driver.predict_conflict(route, (x0, y0 + 4.0), (0.0, 0.0), reach, box) is None, "4.0 m off, its nose is clear of the path")
    check(driver.leaves_route(route, (x0, y0 + 3.0), (0.0, 0.0), reach, 0.0, box) == math.inf
          and driver.leaves_route(route, (x0, y0 + 3.0), (0.0, -3.0), reach, 0.0, box) < math.inf,
          "leaves_route uses the same width")

    ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False)
    x, y, _, left, yaw = frame(start, 26.0)
    ctx.things = [Actor("vehicle.audi.a2", x + 3.0 * left[0], y + 3.0 * left[1], yaw=yaw + 90.0, extent=(2.3, 0.9, 0.7))]
    tick(behaviour, ctx)
    check(any("hazard vehicle" in line for line in ctx.lines), f"C1: a stopped car across the route, nose 0.7 m into it, is a hazard: {ctx.lines}")

    # C2: a stopped ego and a walker leaving the path within 1 s, 4.3 m ahead: the window is taken at the speed the ego resumes to
    Clock.now = 0.0
    ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False)
    ctx.speed = 0.0
    ctx.things = [walker_at(start, 12.4 + 4.3 + 0.2, 0.0, 2.0)]
    tick(behaviour, ctx)
    check(any("hazard" in line for line in ctx.lines), f"a walker leaving the path 4.3 m ahead of a stopped ego is a hazard: {ctx.lines}")


def check_crossers(cmap, start, crossing, plan, center, axis, near, other):
    along = (-axis[1], axis[0])  # along the road
    at = (center[0], center[1], True)
    check(driver.is_crosser(*at, center, axis, (1.4 * axis[0], 1.4 * axis[1])) and driver.is_crosser(*at, center, axis)
          and not driver.is_crosser(*at, center, axis, (5.0 * along[0], 5.0 * along[1])),
          "S1: crossing and standing is a crosser, riding along the road is not")
    s_ego = near - driver.CROSSER_STOP - 10.0 - 2.4
    ctx, behaviour = make_driver(cmap, start, crossing, plan, s_ego, 8.0)
    ctx.things = [Actor("vehicle.bh.crossbike", center[0], center[1] + other, yaw=0.0, vx=5.0 * along[0], vy=5.0 * along[1],
                        attributes={"base_type": "bicycle"})]
    tick(behaviour, ctx)
    check(not any("crosser" in line for line in ctx.lines) and Planner.created[-1].speed == CRUISE * 3.6,
          f"S1: a bicycle riding along the road through the crosswalk is not stopped for: {ctx.lines}")

    # S2: the front 1 m before the crosswalk at 8.3 m/s (it cannot stop short of it): it passes, no emergency hold
    walker = lambda: Actor("walker.pedestrian.0002", center[0], center[1] + other, yaw=90.0)  # noqa: E731
    Clock.now = 0.0
    ctx, behaviour = make_driver(cmap, start, crossing, plan, near - 1.0 - 2.4, CRUISE)
    ctx.things = [walker()]
    control = tick(behaviour, ctx)
    check(not any("crosser" in line or "emergency" in line for line in ctx.lines) and control.brake == 0.0,
          f"S2: a late crosser the ego cannot stop for is passed, not an emergency: {ctx.lines}")
    ctx, behaviour = make_driver(cmap, start, crossing, plan, near - 1.0 - 2.4, 0.0)  # stopped short of the crosswalk: held
    ctx.things = [walker()]
    check(tick(behaviour, ctx).brake == 1.0 and any("crosser" in line for line in ctx.lines), "S2: stopped short of the crosswalk it holds")
    ctx, behaviour = make_driver(cmap, start, crossing, plan, near + 1.0 - 2.4, 0.0)  # stopped on the crosswalk: it does not hold there
    ctx.things = [walker()]
    check(tick(behaviour, ctx).brake == 0.0 and Planner.created[-1].speed == CRUISE * 3.6, "S2: stopped on the crosswalk it does not hold")


def check_sight(cmap, start, crossing, plan):
    print("line of sight without cast_ray")
    entry = driver.ray_box_entry
    box = ((10.0, 0.0, 1.0), 0.0, (1.0, 1.0, 1.0))
    check(abs(entry(loc(0, 0, 1), loc(20, 0, 1), *box) - 0.45) < 1e-9, "the segment enters the box at 9 m of 20")
    check(entry(loc(0, 3, 1), loc(20, 3, 1), *box) is None, "a segment beside the box misses it")
    check(entry(loc(0, 0, 1), loc(5, 0, 1), *box) is None, "a segment that ends before the box misses it")
    check(entry(loc(10, 0, 1), loc(20, 0, 1), *box) == 0.0, "a segment that starts inside enters at once")
    turned = ((10.0, 0.0, 1.0), math.pi / 4, (3.0, 0.2, 1.0))  # a thin box turned 45 degrees across the x axis
    check(entry(loc(0, 0, 1), loc(20, 0, 1), *turned) is not None and entry(loc(0, 2.0, 1), loc(20, 2.0, 1), *turned)
          is not None and entry(loc(0, 2.5, 1), loc(20, 2.5, 1), *turned) is None, "a turned box")
    rays = driver.BoxRays([((20.0, 0.0, 1.0), 0.0, (1.0, 1.0, 1.0), TRUCK), ((10.0, 0.0, 1.0), 0.0, (1.0, 1.0, 1.0), PEDESTRIANS)])
    hits = rays.cast_ray(loc(0, 0, 1), loc(30, 0, 1))
    check([h.label for h in hits] == [PEDESTRIANS, TRUCK] and abs(hits[0].location.x - 9.0) < 1e-9,
          f"BoxRays: nearest first, at the box surface: {hits}")

    ego = Actor("vehicle.lincoln.mkz_2017", 0, 0, extent=(2.4, 1.0, 0.8))

    class Scripted:
        def __init__(self, hits):
            self.hits = hits

        def cast_ray(self, a, b):
            if isinstance(self.hits, Exception):
                raise self.hits
            return self.hits

    on_ego, beside = Hit(loc(2.4, 0, 0.8), carla.CityObjectLabel.Car), Hit(loc(6.0, 0, 0.8), carla.CityObjectLabel.Car)
    for name, hits, want in (("a labelled hit on the ego", [Hit(loc(5, 0, 0.8), NONE), on_ego], True),
                             ("no hit", [], False), ("a NONE hit on the ego", [Hit(loc(2.4, 0, 0.8), NONE)], False),
                             ("a hit away from the ego", [beside], False), ("an error", RuntimeError("x"), False),
                             ("a road hit before the ego", [on_ego, Hit(loc(4.0, 0, 0.8), carla.CityObjectLabel.Roads)], False)):
        check(driver.cast_ray_sees_actors(Scripted(hits), ego) == want, f"self-test with {name}")
    check(driver.choose_sight(Scripted([]), ego)[0] == "boxes", "no hit: boxes")
    check(driver.choose_sight(Scripted([on_ego]), ego)[0] == "cast_ray", "a hit on the ego: cast_ray")
    check(driver.choose_sight(Scripted([]), ego, "cast_ray")[0] == "cast_ray"
          and driver.choose_sight(Scripted([on_ego]), ego, "boxes")[0] == "boxes", "CARSIM_SIGHT wins")

    # the driver picks the boxes by itself (the fake world's cast_ray never hits the ego) and sees through them
    with mock.patch.dict(os.environ):
        os.environ.pop(driver.SIGHT_ENV, None)
        ctx, behaviour = make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False)
    check(behaviour._sight == "boxes" and any("bounding boxes" in line for line in ctx.lines), f"the choice: {ctx.lines}")
    seen = []  # the self-test runs before the ego is set moving (an async client's pose would be stale)
    with mock.patch.object(driver, "choose_sight", lambda world, ego, forced=None: seen.append(ego.target_velocity)
                           or ("boxes", "test")):
        make_driver(cmap, start, crossing, plan, 10.0, CRUISE, uses_crossing=False)
    check(seen == [None], f"the self-test ran after set_target_velocity: {seen}")
    child = walker_at(start, 24.6, 3.5, -1.5)  # in the path when the ego arrives (see check_window_and_memory)
    x, y, fwd, left, yaw = frame(start, 19.0)  # a truck parked between, 3.2 m left of the lane: out of the path
    truck = Actor("vehicle.carlamotors.european_hgv", x + 3.2 * left[0], y + 3.2 * left[1], yaw=yaw, extent=(4.0, 1.45, 1.75))
    ctx.things, ctx.world.aims = [child], []  # the self-test cast one ray
    with mock.patch.object(driver.CarlaDataProvider, "_carla_actor_pool", {a.id: a for a in (ctx.ego, child, truck)}):
        control = tick(behaviour, ctx)
        check(control.brake == 0.0 and child.id not in behaviour._known and not ctx.world.aims,
              f"the child behind the truck's box is unseen and World.cast_ray unused: brake {control.brake}")
    with mock.patch.object(driver.CarlaDataProvider, "_carla_actor_pool", {a.id: a for a in (ctx.ego, child)}):
        control = tick_for(behaviour, ctx, driver.BRAKE_DELAY + 0.1)
        check(control.brake == 1.0 and child.id in behaviour._known, f"in view once the truck has gone: {ctx.lines}")


def main():
    os.environ[driver.SIGHT_ENV] = "cast_ray"  # the fake World of these checks never hits the ego (see check_sight)
    cmap, start, crossing, plan, route = make_geometry()
    check_rays()
    center, axis, near, other = check_decisions(cmap, crossing, route)
    check_driver(cmap, start, crossing, plan, route, center, axis, near, other)
    check_ego_control(cmap, start, crossing, plan)
    check_follow()
    check_crosser_stop(cmap, start, crossing, plan, center, other, near)
    check_start_speed(cmap, start, crossing, plan)
    check_eye_and_budget(cmap, start, crossing, plan)
    check_window_and_memory(cmap, start, crossing, plan)
    check_delay_and_metrics(cmap, start, crossing, plan)
    check_speed_control()
    check_counterfactual(cmap, start, crossing, plan)
    check_ray_budget(cmap, start, crossing, plan, center, other, near)
    check_brake_onset()
    check_escalation(cmap, start, crossing, plan)
    check_path_width_and_window(cmap, start, crossing, plan, route)
    check_crossers(cmap, start, crossing, plan, center, axis, near, other)
    check_sight(cmap, start, crossing, plan)

    print("FAILED" if FAILURES else "PASSED")
    sys.exit(1 if FAILURES else 0)


if __name__ == "__main__":
    main()
