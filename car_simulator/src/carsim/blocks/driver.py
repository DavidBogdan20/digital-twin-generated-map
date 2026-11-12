"""
The cautious autopilot of <ego_control brake="sight"/>: it brakes for what it can see, and only for that.

Every decision is a plain function of numbers (ray clearance, crosser test, in-path prediction, time window, speed
decision, SpeedControl), so tests/check_driver_offline.py runs them without a server; CautiousDriver only reads the
world and applies the control. It is a fair baseline: what it knows of an actor is the state it saw last (extrapolated
while hidden), and a full brake bites BRAKE_DELAY after the decision. The same driver with hazards=False
(<ego_control brake="none"/>) never looks and never brakes: the counterfactual run, at exactly the same speed control.

Perception is the `cast_ray(start, end)` the driver looks through (can_see): World.cast_ray where it sees actors, else
BoxRays, the same rays against the bounding boxes of the scenario's actors. A startup self-test (cast_ray_sees_actors:
one ray at the ego itself) picks one and logs it: on some generated maps (CARLA 0.9.15 Digital Twin maps) cast_ray
returned no hits at all. The environment variable CARSIM_SIGHT=cast_ray|boxes forces one.
"""

import math
import os
from collections import namedtuple

import carla
import py_trees

from agents.navigation.local_planner import LocalPlanner
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior
from srunner.scenariomanager.timer import GameTime

RANGE = 50.0  # only actors this close and ahead are looked at [m]
EYE_HEIGHT = 1.3  # [m] above the ego's ground point
EYE_AHEAD = 0.1  # the eye is this far in front of the ego's bumper [m], so never inside its own bounding box
EGO_HIT_MARGIN = 0.3  # a ray hit this close to the ego's own bounding box is the ego, not an occluder [m]
CANDIDATE_BAND = 15.0  # only actors whose track comes this close to the route are looked at (crossers on a wide road) [m]
MAX_RAYS = 12  # cast_ray calls per tick, the candidate looked at longest ago first (a crowd made a tick last 0.1 to 0.5 s)
RAY_MARGIN = 0.3  # a hit this much nearer than the target's box (or further) does not hide it [m]
TOP_INSET, BOTTOM_INSET = 0.1, 0.2  # the high and low ray end this far inside the target's box [m]
KNOWN_FOR = 2.0  # an actor stays known this long after it was last seen [s]
CROSSER_BAND = 4.0  # a walker or bicycle this close to the crosswalk's centre line, on a driving lane, is crossing [m]
CROSSER_STOP = 2.0  # the ego stops this far before the crosswalk [m]
PREDICT_TIME, PREDICT_STEP = 2.0, 0.25  # constant-velocity look-ahead [s]
WINDOW_TIME = 6.0  # how long an actor's stay in the path is followed, to know when it leaves [s]
TIME_MARGIN = 0.5  # ego and actor must be at the same place within this much time to conflict [s]
WINDOW_MIN_SPEED = 1.0  # a slower (or stopped) ego is taken to arrive at this speed [m/s]
BRAKE_DELAY = 0.3  # a full brake is applied this long after the decision, the ego coasts meanwhile [s]
PATH_MARGIN = 0.5  # on top of the two half widths, an actor this close to the route is in its path [m]
PATH_STOP = 4.0  # the ego stops this far before an actor in its path [m]
LEAD_MIN_SPEED = 1.0  # a vehicle in the lane moving along it faster than this is followed, not stopped for [m/s]
LEAD_GAP, LEAD_TIME = 4.0, 1.0  # the gap kept to it: LEAD_GAP + LEAD_TIME * its speed [m, s]
LEAD_GAIN = 0.3  # [1/s] the ego is this much faster than the lead per metre of gap over the kept gap
LEAD_HYSTERESIS = 0.5  # a followed vehicle stays in the path this much further out [m]
COMFORT_DECEL = 3.0  # [m/s^2], the braking a stop target is approached with
STOP_MARGIN = 0.3  # a comfort stop aims this much short of its stop point, for the last bit of tracking lag [m]
EMERGENCY_DECEL = 6.0  # [m/s^2], the ego brakes at full when it could not stop within the gap at this deceleration
EMERGENCY_TTC = 1.6  # [s]
HOLD_SPEED = 0.1  # a target speed this low is a full brake [m/s]
MOVING = 0.5  # slower than this the ego is stopped, not in an emergency [m/s]
FF_THROTTLE = 0.06  # throttle that holds a steady speed, per m/s of target (drag and rolling resistance of the mkz)
KP, KI = 0.3, 0.3  # throttle per m/s of speed error, and per m/s*s of its integral
INTEGRAL_BAND = 0.5  # the integral only collects while the speed is this close to the target (not in a start-up dip or a stop) [m/s]
INTEGRAL_MAX = 0.3  # the integral term's limit, in throttle (anti-windup, with freezing it while the throttle is saturated)
BRAKE_DEADBAND = 0.3  # the ego is only braked when it is this much over the target speed, below that it coasts [m/s]
BRAKE_GAIN = 0.5  # brake per m/s over the target speed beyond the deadband
GEAR_2_SPEEDS = (2.5, 3.5, 10.0)  # held in 2nd gear above the second speed until the third, back to the autobox below the first [m/s]
SELFTEST_AHEAD = 3.0  # the self-test ray starts this far in front of the ego's box, at its centre height [m]
SIGHT_ENV = "CARSIM_SIGHT"  # cast_ray | boxes: force the perception instead of the self-test
BOX_PREFIXES = ("vehicle.", "walker.", "static.prop.")  # the actors BoxRays sees (and hides behind)


# --- geometry and decisions, all pure ---------------------------------------------------------------------------

def box_distance(point, center, yaw, extent):
    """Distance from `point` (x, y, z) to the box at `center` (x, y, z), turned by `yaw` [rad], half sizes `extent`."""
    dx, dy = point[0] - center[0], point[1] - center[1]
    local = (dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw), point[2] - center[2])
    return math.sqrt(sum(max(0.0, abs(v) - e) ** 2 for v, e in zip(local, extent)))


def ray_clear(eye, hits, target_distance, ego_box=None):
    """
    True when no hit of a cast_ray lies nearer to `eye` than the target's box (`target_distance`) less RAY_MARGIN.
    Ordering and whether the target itself is hit do not matter. Hits labelled NONE do not count: they are invisible
    volumes (one ~0.2 m from the eye on every ray, a capsule around each walker), real occluders are labelled. Nor do
    hits within EGO_HIT_MARGIN of `ego_box` (a `box_of` result): that is the ego's own body.
    """
    limit = target_distance - RAY_MARGIN

    def ignored(hit):
        if hit.label == carla.CityObjectLabel.NONE:
            return True
        if ego_box is None:
            return False
        c, yaw, extent = ego_box
        return box_distance((hit.location.x, hit.location.y, hit.location.z), (c.x, c.y, c.z), yaw, extent) <= EGO_HIT_MARGIN

    return all(ignored(hit) or eye.distance(hit.location) >= limit for hit in hits)


def can_see(world, eye, center, yaw, extent, ego_box=None):
    """True when one of three rays from `eye` (a Location) to the target's box is clear: centre first, then near top, near bottom."""
    target = box_distance((eye.x, eye.y, eye.z), (center.x, center.y, center.z), yaw, extent)
    for dz in (0.0, extent[2] - TOP_INSET, BOTTOM_INSET - extent[2]):
        aim = carla.Location(center.x, center.y, center.z + dz)
        if ray_clear(eye, world.cast_ray(eye, aim), target, ego_box):
            return True
    return False


Hit = namedtuple("Hit", "location label")  # what BoxRays.cast_ray returns, like carla.LabelledPoint


def ray_box_entry(start, end, center, yaw, extent):
    """
    Where the segment `start` -> `end` (carla.Location) first meets the box at `center` (x, y, z), turned by `yaw`
    [rad], half sizes `extent`: the fraction 0..1 along the segment (0 when `start` is inside), or None (slab test).
    """
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy, dz = start.x - center[0], start.y - center[1], start.z - center[2]
    ex, ey, ez = end.x - start.x, end.y - start.y, end.z - start.z
    origin = (dx * c + dy * s, -dx * s + dy * c, dz)
    direction = (ex * c + ey * s, -ex * s + ey * c, ez)
    low, high = 0.0, 1.0
    for o, d, e in zip(origin, direction, extent):
        if abs(d) < 1e-12:
            if abs(o) > e:
                return None
            continue
        t1, t2 = (-e - o) / d, (e - o) / d
        low, high = max(low, min(t1, t2)), min(high, max(t1, t2))
        if low > high:
            return None
    return low


class BoxRays:

    """
    cast_ray for maps where World.cast_ray sees no actors: the segment against `boxes`, a list of (centre (x, y, z),
    yaw [rad], half sizes, label), nearest hit first. Buildings and other map meshes are not in it.
    """

    def __init__(self, boxes):
        self.boxes = boxes

    def cast_ray(self, start, end):
        hits = []
        for center, yaw, extent, label in self.boxes:
            t = ray_box_entry(start, end, center, yaw, extent)
            if t is not None:
                hits.append((t, Hit(carla.Location(start.x + (end.x - start.x) * t, start.y + (end.y - start.y) * t,
                                                   start.z + (end.z - start.z) * t), label)))
        return [hit for _, hit in sorted(hits, key=lambda item: item[0])]


def box_label(actor):
    """A CityObjectLabel for BoxRays: anything but NONE counts as an occluder."""
    if actor.type_id.startswith("walker."):
        return carla.CityObjectLabel.Pedestrians
    if actor.type_id.startswith("static."):
        return carla.CityObjectLabel.Static
    return carla.CityObjectLabel.Car


def actor_boxes(actors, skip_id=None):
    """BoxRays' boxes of the vehicles, walkers and static props among `actors` (all but `skip_id`); gone ones are left out."""
    boxes = []
    for actor in actors:
        try:
            if actor is None or actor.id == skip_id or not actor.type_id.startswith(BOX_PREFIXES):
                continue
            center, yaw, extent = box_of(actor)
        except (AttributeError, RuntimeError):
            continue
        boxes.append(((center.x, center.y, center.z), yaw, extent, box_label(actor)))
    return boxes


def cast_ray_sees_actors(world, actor):
    """
    The startup self-test: True when World.cast_ray, cast from SELFTEST_AHEAD m in front of `actor` (the ego) at its
    centre height back to its centre, has a labelled hit on its box (within EGO_HIT_MARGIN) and none before it (a road
    mesh that waves above the car would hide what the driver looks at too).
    """
    center, yaw, extent = box_of(actor)
    ahead = extent[0] + SELFTEST_AHEAD
    start = carla.Location(center.x + ahead * math.cos(yaw), center.y + ahead * math.sin(yaw), center.z)
    try:
        hits = world.cast_ray(start, center)
    except RuntimeError:
        return False
    labelled = sorted((hit for hit in hits if hit.label != carla.CityObjectLabel.NONE),
                      key=lambda hit: start.distance(hit.location))
    return bool(labelled) and box_distance((labelled[0].location.x, labelled[0].location.y, labelled[0].location.z),
                                           (center.x, center.y, center.z), yaw, extent) <= EGO_HIT_MARGIN


def choose_sight(world, ego, forced=None):
    """("cast_ray" or "boxes", why): `forced` (CARSIM_SIGHT) when it names one, else the self-test's verdict."""
    if forced in ("cast_ray", "boxes"):
        return forced, "forced by {}".format(SIGHT_ENV)
    if cast_ray_sees_actors(world, ego):
        return "cast_ray", "the self-test ray hit the ego"
    return "boxes", "World.cast_ray did not see the ego in the self-test"


class Rays:

    """A world's (or BoxRays') cast_ray that counts its calls."""

    def __init__(self, world):
        self.world, self.count = world, 0

    def cast_ray(self, start, end):
        self.count += 1
        return self.world.cast_ray(start, end)


class Route:

    """The ego's route as a polyline of (x, y): where a point projects onto it."""

    def __init__(self, points):
        self.points = points
        self.cumulative = [0.0]
        for a, b in zip(points, points[1:]):
            self.cumulative.append(self.cumulative[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))

    def project(self, x, y):
        """(s, offset, (tx, ty)): route distance to the nearest point, its distance from (x, y), the direction there."""
        best = None
        for i, (a, b) in enumerate(zip(self.points, self.points[1:])):
            dx, dy = b[0] - a[0], b[1] - a[1]
            length = math.hypot(dx, dy)
            if length == 0.0:
                continue
            u = min(1.0, max(0.0, ((x - a[0]) * dx + (y - a[1]) * dy) / (length * length)))
            offset = math.hypot(x - (a[0] + u * dx), y - (a[1] + u * dy))
            if best is None or offset < best[1]:
                best = (self.cumulative[i] + u * length, offset, (dx / length, dy / length))
        return best


def crosswalk_span(crossing, route):
    """
    (center, axis, near, far) of the crosswalk: its centre line (a point and the direction across the road) and the
    route distances of its near and far edge [m]. Without a painted crosswalk it is 4 m deep at the crossing's waypoint.
    """
    crosswalk = crossing.crosswalk
    if crosswalk is not None:
        center, axis = (crosswalk.center.x, crosswalk.center.y), crosswalk.axis
        half = max(centre_line_offset(p.x, p.y, center, axis) for p in crosswalk.points)
    else:
        location, yaw = crossing.wp.transform.location, math.radians(crossing.wp.transform.rotation.yaw)
        center, axis, half = (location.x, location.y), (-math.sin(yaw), math.cos(yaw)), 2.0
    s = route.project(*center)[0]
    return center, axis, s - half, s + half


def centre_line_offset(x, y, center, axis):
    """How far (x, y) is from the line through `center` along `axis`, i.e. along the road for a crosswalk."""
    return abs(-(x - center[0]) * axis[1] + (y - center[1]) * axis[0])


def is_crosser(x, y, on_driving_lane, center, axis, velocity=(0.0, 0.0)):
    """
    A walker or bicycle at (x, y) that is on a driving lane, within CROSSER_BAND of the crosswalk's centre line and not
    moving along the road (its velocity across the crosswalk is at least its velocity along it: a standing one is across).
    """
    across = abs(velocity[0] * axis[0] + velocity[1] * axis[1])
    along = abs(-velocity[0] * axis[1] + velocity[1] * axis[0])
    return on_driving_lane and centre_line_offset(x, y, center, axis) <= CROSSER_BAND and across >= along


def half_across(box, direction):
    """Half size [m] of `box` ((yaw [rad], half sizes) or None) across the route, which runs along `direction`."""
    return 0.0 if box is None else half_along(box[0], box[1], (-direction[1], direction[0]))


def predict_conflict(route, position, velocity, reach, box=None):
    """
    (s, t, direction) where the constant-velocity track of an actor first comes within `reach` (plus the actor's half
    size across the route, from its `box`) of the route (the route distance, the time [s] and the route's direction
    there), or None within PREDICT_TIME.
    """
    for i in range(int(round(PREDICT_TIME / PREDICT_STEP)) + 1):
        t = i * PREDICT_STEP
        s, offset, direction = route.project(position[0] + velocity[0] * t, position[1] + velocity[1] * t)
        if offset < reach + half_across(box, direction):
            return s, t, direction
    return None


def leaves_route(route, position, velocity, reach, t_in, box=None):
    """The time [s] at which a constant-velocity track that is in the path at `t_in` has left it, inf when it still is at WINDOW_TIME."""
    t = t_in
    while t <= WINDOW_TIME:
        _, offset, direction = route.project(position[0] + velocity[0] * t, position[1] + velocity[1] * t)
        if offset >= reach + half_across(box, direction):
            return t
        t += PREDICT_STEP
    return math.inf


def in_time_window(t_in, t_out, gap, span, speed):
    """
    True when the ego is at the actor's place while the actor is in the path (`t_in` to `t_out` [s], TIME_MARGIN on
    both sides). The ego's front reaches the actor after `gap` m and its tail has passed after `gap + span` m, at
    `speed` (at least WINDOW_MIN_SPEED), `span` being the ego's length plus the actor's extent along the route.
    """
    v = max(speed, WINDOW_MIN_SPEED)
    return t_in - TIME_MARGIN <= (gap + span) / v and t_out + TIME_MARGIN >= gap / v


def half_along(yaw, extent, direction):
    """Half size [m] of a box (turned by `yaw` [rad], half sizes `extent`) along `direction`, a unit (x, y)."""
    angle = yaw - math.atan2(direction[1], direction[0])
    return abs(math.cos(angle)) * extent[0] + abs(math.sin(angle)) * extent[1]


def time_to_collision(gap, ego_speed, along_speed):
    """Seconds until a gap [m] closes, the ego driving `ego_speed` and the other `along_speed` the same way; inf if never."""
    closing = ego_speed - along_speed
    return gap / closing if closing > 0.1 else math.inf


def decide_speed(speed, cruise, stop_gaps, ttcs, lead=None):
    """
    (target speed, emergency) in m/s. `stop_gaps`: distances [m] to the points the ego must stop at, `ttcs`: the time
    to collision with each actor in its path, `lead`: (speed, gap) of the vehicle ahead in the lane, or None.
    The target is the slowest of cruise, the speed from which each stop point is reached at COMFORT_DECEL after the
    ego has coasted BRAKE_DELAY at its speed (the brake does not bite at once) and STOP_MARGIN short, and the
    lead's speed plus LEAD_GAIN per metre of gap over the kept gap (a smooth follow, no stop point for a moving lead).
    Emergency: a collision within EMERGENCY_TTC, or a stop point nearer than the ego needs at EMERGENCY_DECEL, or a
    lead closer than the kept gap (or needing more than COMFORT_DECEL to match) within EMERGENCY_TTC.
    """
    target = cruise
    for gap in stop_gaps:
        target = min(target, math.sqrt(2.0 * COMFORT_DECEL * max(0.0, gap - speed * BRAKE_DELAY - STOP_MARGIN)))
    lead_closing = False
    if lead is not None:
        lead_speed, gap = lead
        kept = LEAD_GAP + LEAD_TIME * lead_speed
        target = min(target, max(0.0, lead_speed + LEAD_GAIN * (gap - kept)))
        closing = speed - lead_speed
        lead_closing = (closing > 0.1 and gap / closing < EMERGENCY_TTC
                        and (gap < kept or closing * closing / (2.0 * max(gap, 0.1)) > COMFORT_DECEL))
    braking_distance = speed * speed / (2.0 * EMERGENCY_DECEL)
    emergency = speed > MOVING and (lead_closing or any(t < EMERGENCY_TTC for t in ttcs)
                                    or any(g < braking_distance for g in stop_gaps))
    return target, emergency


class SpeedControl:

    """
    The longitudinal control, in m/s (LocalPlanner's PID works on a km/h error with a 10-sample "integral" and
    chattered between full throttle and full brake): throttle = FF_THROTTLE * target (a steady cruise needs it) + KP *
    error + an integral of KI * error, collected within INTEGRAL_BAND of the target only, limited to INTEGRAL_MAX and
    frozen while the throttle is saturated and the error pushes further (anti-windup). Brake only when the ego is more than BRAKE_DEADBAND over the target, BRAKE_GAIN
    per m/s beyond it (no throttle then); inside the deadband the throttle just backs off.
    """

    def __init__(self):
        self.integral = 0.0

    def step(self, target, speed, dt):
        """(throttle, brake) for a `target` speed at `speed` [m/s], `dt` s after the last step."""
        error = target - speed
        if error < -BRAKE_DEADBAND:
            return 0.0, min(1.0, BRAKE_GAIN * (-error - BRAKE_DEADBAND))
        raw = FF_THROTTLE * target + KP * error + self.integral
        throttle = min(1.0, max(0.0, raw))
        if abs(error) < INTEGRAL_BAND and (raw == throttle or (raw - throttle) * error < 0.0):
            self.integral = max(-INTEGRAL_MAX, min(INTEGRAL_MAX, self.integral + KI * error * dt))
        return throttle, 0.0


def eye_position(location, yaw, bbox_x, extent_x):
    """The eye of the ego (carla.Location at its ground point, `yaw` [rad], bbox centre offset and half length along x)."""
    ahead = bbox_x + extent_x + EYE_AHEAD
    return carla.Location(location.x + ahead * math.cos(yaw), location.y + ahead * math.sin(yaw),
                          location.z + EYE_HEIGHT)


def box_of(actor, transform=None):
    """(centre carla.Location, yaw [rad], (x, y, z) half sizes) of an actor's bounding box now (at `transform` if given)."""
    transform, box = transform or actor.get_transform(), actor.bounding_box
    yaw = math.radians(transform.rotation.yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    center = carla.Location(transform.location.x + box.location.x * c - box.location.y * s,
                            transform.location.y + box.location.x * s + box.location.y * c,
                            transform.location.z + box.location.z)
    return center, yaw, (box.extent.x, box.extent.y, box.extent.z)


def kind_of(actor, kind):
    """"walker", "bicycle" or "vehicle" for an actor `solid_things` called `kind`."""
    if kind == "vehicle" and getattr(actor, "attributes", {}).get("base_type") == "bicycle":
        return "bicycle"
    return kind


class CautiousDriver(AtomicBehavior):

    """
    The ego drives its route at `speed` [m/s] like WaypointFollower (LocalPlanner steers, SpeedControl does throttle and
    brake), but looks: every tick it casts rays at the walkers and vehicles near its route ahead (`can_see`, at most
    MAX_RAYS calls, the candidate looked at longest ago first, so none is starved) and knows those it saw within
    the last KNOWN_FOR seconds, by the state it saw them in (a hidden actor is carried on at its last velocity, its live
    pose is never read). It stops before the crosswalk while a known walker or bicycle crosses it (not one riding along
    the road; and not when it can no longer stop short of the crosswalk: then it passes and never holds on it), brakes at full for a
    known actor that will be in its path when the ego gets there (its constant-velocity track within PREDICT_TIME comes
    close to the route, see `in_time_window`) when a stop is not comfortable any more, and follows a known vehicle that
    is ahead in its lane and moves. A full brake is applied BRAKE_DELAY after it was decided. An actor behind something
    is not known: the ego does not brake for it until it comes into view. It never ends (the End block does): update()
    is always RUNNING. initialise() sets the ego to `speed` along its heading. ctx.metrics gets first_visible (the
    target seen), first_hazard (the first actor that is one), brake_onset (the first brake for a hazard: emergency, stop
    target or slower lead), emergency and emergency_count (once, the first time). With hazards=False it never looks:
    the speed control alone, the counterfactual run.
    """

    def __init__(self, ctx, speed, hazards=True, name="EgoCautiousDriver"):
        super().__init__(name, ctx.ego)
        self.ctx = ctx
        self.speed = speed
        self.hazards = hazards
        self._speed_control = SpeedControl()
        self._last = None  # game time of the last tick
        self._second_gear = False  # held in 2nd gear: the mkz's automatic gearbox hunts between 1st and 2nd at 5 to 7 m/s
        self._applied = 0.0  # the brake applied on the last tick
        self._cast_at = {}  # actor id -> game time rays were last cast at it
        self._planner = None
        self._sight = "cast_ray"  # set by the self-test in initialise(): cast_ray | boxes
        self._route = None
        self._crosswalk = None  # crosswalk_span(...), or None without a crossing
        self._leading = set()  # ids followed as a lead on the last tick
        self._known = {}  # actor id -> its last seen state: kind, t (game time), center (x, y, z), yaw, extent, vel (x, y)
        self._reported = set()  # ids of the hazards already logged
        self._mode = "drive"  # drive | crosser | emergency, to log the changes
        self._braking_since = None  # game time a full brake was decided, None while there is none

    def initialise(self):
        super().initialise()
        ctx = self.ctx
        self._planner = LocalPlanner(ctx.ego, opt_dict={
            "target_speed": self.speed * 3.6,
            "lateral_control_dict": {"K_P": 1.0, "K_D": 0.01, "K_I": 0.0, "dt": 0.05},
            "max_throttle": 1.0,
            "max_brake": 1.0})
        plan = ctx.ego_plan()
        self._planner.set_global_plan(plan)  # steering only: the planner's own PID throttle and brake are replaced
        start = ctx.ego_start_wp.transform.location
        self._route = Route([(start.x, start.y)] + [(wp.transform.location.x, wp.transform.location.y) for wp, _ in plan])
        if ctx.uses_crossing:
            self._crosswalk = crosswalk_span(ctx.crossing, self._route)
        if self.hazards:  # before the ego is set moving: the self-test aims at its pose as the client knows it
            self._sight, why = choose_sight(ctx.world, ctx.ego, os.environ.get(SIGHT_ENV))
        # as in NCAP the ego is at its test speed when the test starts, not accelerating from rest
        forward = ctx.ego.get_transform().get_forward_vector()
        ctx.ego.set_target_velocity(carla.Vector3D(forward.x * self.speed, forward.y * self.speed, forward.z * self.speed))
        ctx.log("ego_control", "starting at the test speed, {:.1f} m/s{}".format(
            self.speed, "" if self.hazards else ", hazards ignored (brake none)"))
        if self.hazards:
            ctx.log("ego_control", "line of sight: {} ({})".format(
                "World.cast_ray" if self._sight == "cast_ray" else "rays against the actors' bounding boxes", why))

    def update(self):
        ctx, now = self.ctx, GameTime.get_time()
        transform = ctx.ego.get_transform()  # live: ctx.ego_transform() is a cache that lags 0.4 to 3 m
        yaw = math.radians(transform.rotation.yaw)
        box = ctx.ego.bounding_box
        extent = box.extent
        speed = ctx.ego_forward_speed() or 0.0
        s_ego = self._route.project(transform.location.x, transform.location.y)[0]
        front = s_ego + extent.x
        ahead = box.location.x + extent.x
        bumper = (transform.location.x + ahead * math.cos(yaw), transform.location.y + ahead * math.sin(yaw))

        if self.hazards:
            self._look(now, transform, yaw)
        stop_gaps, ttcs, lead_ttcs, leads, leading, distances = [], [], [], [], set(), []
        crossing_now = False
        for actor_id, known in list(self._known.items()):
            age = now - known["t"]
            if age > KNOWN_FOR:
                del self._known[actor_id]
                continue
            kind, size, (vx, vy), z = known["kind"], known["extent"], known["vel"], known["center"][2]
            x, y = known["center"][0] + vx * age, known["center"][1] + vy * age  # carried on at the last seen velocity
            surface = box_distance((bumper[0], bumper[1], z), (x, y, z), known["yaw"], size)
            ttc = math.inf
            hazard = False

            # a crosser is a stop target only while the ego can still stop short of the crosswalk (or is stopped before
            # it): else it passes, and only the in-path prediction below can brake, it never holds on the crosswalk
            if self._crosswalk is not None and kind != "vehicle" and s_ego < self._crosswalk[3]:
                room = self._crosswalk[2] - front
                lane = ctx.map.get_waypoint(carla.Location(x, y, z), project_to_road=False,
                                            lane_type=carla.LaneType.Driving)
                if is_crosser(x, y, lane is not None, self._crosswalk[0], self._crosswalk[1], (vx, vy)) and room > (
                        0.0 if speed <= MOVING else speed * BRAKE_DELAY + speed * speed / (2.0 * EMERGENCY_DECEL)):
                    stop_gaps.append(self._crosswalk[2] - CROSSER_STOP - front)
                    crossing_now = hazard = True

            reach = extent.y + PATH_MARGIN + (LEAD_HYSTERESIS if actor_id in self._leading else 0.0)
            box = (known["yaw"], size)
            conflict = predict_conflict(self._route, (x, y), (vx, vy), reach, box)
            if conflict is not None and conflict[0] > s_ego:
                s, t, direction = conflict
                along = vx * direction[0] + vy * direction[1]
                half = half_along(known["yaw"], size, direction)
                gap = max(0.0, s - half - front)
                lateral = abs(vx * direction[1] - vy * direction[0])
                # a moving vehicle drifting in at the lane edge is followed too (t > 0), not stopped for
                if kind != "walker" and along > LEAD_MIN_SPEED and (t == 0.0 or lateral < along / 2.0):
                    leads.append((gap, along))
                    leading.add(actor_id)
                    ttc, hazard = time_to_collision(gap, speed, along), True
                    lead_ttcs.append(ttc)
                elif in_time_window(t, leaves_route(self._route, (x, y), (vx, vy), reach, t, box), gap,
                                    2.0 * (extent.x + half), max(speed, self.speed)):  # at the speed it would resume to
                    stop_gaps.append(gap - PATH_STOP)
                    ttc = time_to_collision(gap, speed, along)
                    ttcs.append(ttc)
                    hazard = True
            if hazard:
                distances.append(surface)
            ttc_text = "{:.1f} s".format(ttc) if ttc < math.inf else "none"
            if "first_visible" not in ctx.metrics and (
                    (ctx.target is not None and actor_id == ctx.target.id and age == 0.0)
                    or (ctx.target is None and hazard)):
                ctx.metrics["first_visible"] = {"t": now, "actor": actor_id, "kind": kind, "distance": surface,
                                                "ttc": ttc if ttc < math.inf else None}
                ctx.log("ego_control", "t={:.1f}s first visible {} {}: {:.1f} m ahead (bumper to surface), TTC {}".format(
                    now, kind, actor_id, surface, ttc_text))
            if hazard and actor_id not in self._reported:
                self._reported.add(actor_id)
                ctx.metrics.setdefault("first_hazard", {"t": now, "actor": actor_id, "kind": kind, "distance": surface,
                                                        "ttc": ttc if ttc < math.inf else None})
                ctx.log("ego_control", "t={:.1f}s hazard {} {}: {:.1f} m ahead (bumper to surface), TTC {}".format(
                    now, kind, actor_id, surface, ttc_text))

        self._leading = leading
        lead = None
        if leads:
            gap, along = min(leads)
            lead = (along, gap)
        target, emergency = decide_speed(speed, self.speed, stop_gaps, ttcs, lead)
        nearest = min(distances, default=None)
        self._log_mode(now, "emergency" if emergency else ("crosser" if crossing_now else "drive"), speed, stop_gaps,
                       nearest)

        self._planner.set_speed(target * 3.6)
        control = self._planner.run_step(debug=False)  # the steering
        dt = 0.05 if self._last is None else min(0.2, max(0.0, now - self._last))
        self._last = now
        control.throttle, control.brake = self._speed_control.step(target, speed, dt)
        self._second_gear = GEAR_2_SPEEDS[0] < speed < GEAR_2_SPEEDS[2] and (self._second_gear or speed > GEAR_2_SPEEDS[1])
        control.manual_gear_shift, control.gear = self._second_gear, 2 if self._second_gear else 0
        if emergency or target <= HOLD_SPEED:
            if self._braking_since is None:
                self._braking_since = now
            # a stopped ego holds at once, one already braking does not let go for the delay
            bites = speed <= MOVING or self._applied >= 0.2 or now - self._braking_since >= BRAKE_DELAY - 1e-6
            control.throttle, control.brake = 0.0, 1.0 if bites else 0.0
        else:
            self._braking_since = None
        # only a reaction to a hazard counts: an emergency, or a target below cruise (a stop point or a slower lead)
        if control.brake >= 0.2 and (emergency or target < self.speed) and "brake_onset" not in ctx.metrics:
            ctx.metrics["brake_onset"] = {"t": now, "speed": speed, "ttc": min(ttcs + lead_ttcs, default=None),
                                          "distance": nearest}
        self._applied = control.brake
        ctx.ego.apply_control(control)
        return py_trees.common.Status.RUNNING

    def _look(self, now, transform, yaw):
        """Cast the rays at the candidates (near the route, or already known), the one looked at longest ago first (then the nearest), and note the state of those that are seen."""
        ctx = self.ctx
        ego_box = ctx.ego.bounding_box
        eye = eye_position(transform.location, yaw, ego_box.location.x, ego_box.extent.x)
        own = box_of(ctx.ego, transform)
        location, forward = transform.location, (math.cos(yaw), math.sin(yaw))
        candidates = []
        for actor, kind, loc, _ in ctx.solid_things():
            dx, dy = loc.x - location.x, loc.y - location.y
            if actor.id != ctx.ego.id and dx * forward[0] + dy * forward[1] > 0.0 and math.hypot(dx, dy) <= RANGE:
                candidates.append((self._cast_at.get(actor.id, -math.inf), math.hypot(dx, dy), actor.id, actor, kind, loc))
        rays = Rays(ctx.world if self._sight == "cast_ray" else BoxRays(
            actor_boxes(list(CarlaDataProvider._carla_actor_pool.values()), ctx.ego.id)))
        for _, _, _, actor, kind, loc in sorted(candidates, key=lambda c: c[:3]):
            if rays.count + 3 > MAX_RAYS:
                break
            try:
                velocity = actor.get_velocity()
                if actor.id not in self._known and predict_conflict(
                        self._route, (loc.x, loc.y), (velocity.x, velocity.y), CANDIDATE_BAND) is None:
                    continue
                center, actor_yaw, size = box_of(actor)
            except RuntimeError:
                continue
            self._cast_at[actor.id] = now
            if can_see(rays, eye, center, actor_yaw, size, own):
                self._known[actor.id] = {"kind": kind_of(actor, kind), "t": now, "center": (center.x, center.y, center.z),
                                         "yaw": actor_yaw, "extent": size, "vel": (velocity.x, velocity.y)}

    def _log_mode(self, now, mode, speed, stop_gaps, distance):
        if mode == self._mode:
            return
        if mode == "emergency":
            self.ctx.log("ego_control", "t={:.1f}s emergency brake at {:.1f} m/s, {:.1f} m to the nearest stop point".format(
                now, speed, min(stop_gaps, default=math.inf)))
            metrics = self.ctx.metrics
            metrics["emergency_count"] = metrics.get("emergency_count", 0) + 1
            metrics.setdefault("emergency", {"t": now, "speed": speed, "distance": distance})
        elif mode == "crosser":
            self.ctx.log("ego_control", "t={:.1f}s stopping for crosser".format(now))
        else:
            self.ctx.log("ego_control", "t={:.1f}s clear, driving on".format(now))
        self._mode = mode

    def terminate(self, new_status):
        """Release the controls and the planner."""
        if self._planner is not None:
            self._planner.reset_vehicle()
            self._planner = None
            if self.ctx.ego.is_alive:
                self.ctx.ego.apply_control(carla.VehicleControl())
        super().terminate(new_status)
