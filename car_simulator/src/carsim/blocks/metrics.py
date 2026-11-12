"""
Per-run measurements for run.json (implicit block, no XML element, no attributes).

A from_start sampler (it never ends, so it cannot end the scenario under Main's SUCCESS_ON_ONE) looks at the ego every
tick: speed, brake command and, when a block set `ctx.target`, the distance from the ego's bounding box to the target's
and the time to collision. A collision sensor on the ego (spawned when the sampler starts, destroyed in cleanup())
gives the impact. cleanup() runs from ComposedScenario.terminate(), before the ego is destroyed, and writes the JSON to
the file named by the environment variable CARSIM_METRICS_FILE (set by apps/run_scenario.py, which merges it into
run.json under "metrics"). Without the variable nothing is written. Missing data is null, never an error.

The measurements are kept as running aggregates, so a long manual run costs no memory. Definitions (see
.agents/plans/ncap-data-review.md for the NCAP ones):

  distance        the gap between the ego's and the target's bounding boxes [m], 0 when they overlap
  ttc             gap along the ego's heading / closing speed, measured to the part of the target's box that lies inside
                  the ego's width band (a turned car beside the bumper counts only from where it enters the band), null
                  while nothing of it is inside (a child still on the pavement is not on a collision course)
  test_speed      the ego's target speed when the scenario tells it (`ctx.ego_target_speed`), else its top speed up to
                  the first braking
  speed_reduction test_speed - impact speed (the impact speed is 0 when there was no impact), as NCAP scores it;
                  null when the collision sensor could not be spawned (impact unknown: impact.happened is null too)
  first_visible   the first sighting of the target (driver, ctx.metrics["first_visible"])
  first_hazard    the first time the driver judged anything a hazard: {"t", "actor", "kind", "distance", "ttc"}
                  (driver, ctx.metrics["first_hazard"]); null without one
  brake_onset / emergency   written by the driver into ctx.metrics (see BlockContext.metrics); brake_onset falls back
                  to the first tick with brake >= BRAKE_ONSET AND a hazard (ttc < HAZARD_TTC) only when the driver
                  wrote no event at all (no driver); a start-up cruise brake without a hazard is not a brake onset
"""

import json
import math
import os
from pathlib import Path

import carla
import py_trees

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior
from srunner.scenariomanager.timer import GameTime

from carsim.blocks.base import HALF_LENGTH_DEFAULTS, Block
from carsim.runner import METRICS_FILE_ENV

DRIVER_EVENTS = ("first_visible", "first_hazard", "brake_onset", "emergency")
BRAKE_ONSET = 0.2  # a brake command at or above this is the start of braking
HAZARD_TTC = 4.0  # [s] the fallback brake onset needs a target with a ttc below this
MIN_CLOSING_SPEED = 0.1  # [m/s] slower than this the time to collision is not defined
COLLISION_SENSOR = "sensor.other.collision"


# --- pure parts (no CARLA) ------------------------------------------------------------------------------------------

def box_corners(box):
    """The 4 corners of a box (centre x, centre y, yaw [rad], half length, half width) as (x, y) pairs."""
    cx, cy, yaw, hx, hy = box
    cos, sin = math.cos(yaw), math.sin(yaw)
    return [(cx + sx * hx * cos - sy * hy * sin, cy + sx * hx * sin + sy * hy * cos)
            for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def _point_segment(point, a, b):
    px, py = point
    dx, dy = b[0] - a[0], b[1] - a[1]
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 == 0 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / length2))
    return math.hypot(px - (a[0] + t * dx), py - (a[1] + t * dy))


def _axes(corners):
    return [(corners[i][1] - corners[(i + 1) % 4][1], corners[(i + 1) % 4][0] - corners[i][0]) for i in range(2)]


def box_distance(a, b):
    """Gap between two oriented boxes [m]: 0 when they touch or overlap."""
    ca, cb = box_corners(a), box_corners(b)
    separated = False
    for ax, ay in _axes(ca) + _axes(cb):
        pa = [x * ax + y * ay for x, y in ca]
        pb = [x * ax + y * ay for x, y in cb]
        if max(pa) < min(pb) or max(pb) < min(pa):
            separated = True
            break
    if not separated:
        return 0.0
    pairs = [(p, cb) for p in ca] + [(p, ca) for p in cb]
    return min(_point_segment(p, q[i], q[(i + 1) % 4]) for p, q in pairs for i in range(4))


def _clip_band(points, hy):
    """The polygon `points` clipped to |y| <= hy (Sutherland-Hodgman); [] when nothing is inside."""
    for sign in (1.0, -1.0):
        clipped = []
        for i, (x1, y1) in enumerate(points):
            x2, y2 = points[(i + 1) % len(points)]
            in1, in2 = sign * y1 <= hy, sign * y2 <= hy
            if in1 != in2:
                k = (sign * hy - y1) / (y2 - y1)
                clipped.append((x1 + k * (x2 - x1), y1 + k * (y2 - y1)))
            if in2:
                clipped.append((x2, y2))
        points = clipped
        if not points:
            break
    return points


def time_to_collision(ego, target, ego_velocity, target_velocity):
    """
    Constant-speed time to collision [s] of the ego with a target box, or None. Both boxes are as in box_corners,
    velocities are (vx, vy) [m/s]. Defined while the target is ahead and part of it lies inside the ego's width band:
    the gap along the ego's heading to that part (0 when the front already reached it) over the closing speed.
    """
    cx, cy, yaw, hx, hy = ego
    cos, sin = math.cos(yaw), math.sin(yaw)
    points = [((x - cx) * cos + (y - cy) * sin, -(x - cx) * sin + (y - cy) * cos) for x, y in box_corners(target)]
    points = _clip_band(points, hy)
    if not points or max(p[0] for p in points) <= 0:
        return None  # beside its path, or behind the ego
    gap = max(0.0, min(p[0] for p in points) - hx)
    closing = (ego_velocity[0] - target_velocity[0]) * cos + (ego_velocity[1] - target_velocity[1]) * sin
    if gap < 1e-6:
        return 0.0
    return gap / closing if closing > MIN_CLOSING_SPEED else None


class Accumulator:
    """The running aggregates of the per-tick samples (speed [m/s], brake 0..1, distance [m] or None, ttc [s] or None)."""

    def __init__(self):
        self.t0 = self.t = None
        self.ticks = 0
        self.speed = None  # the last speed
        self.peak_speed = None  # top speed until the first braking
        self.braked = False  # a brake >= BRAKE_ONSET was seen (ends the top-speed window)
        self.onset = None  # first tick with brake >= BRAKE_ONSET and a hazard (ttc < HAZARD_TTC)
        self.min_distance = self.min_ttc = None

    def add(self, t, speed, brake, distance=None, ttc=None):
        if self.t0 is None:
            self.t0 = t
        self.t = t
        self.ticks += 1
        if speed is not None:
            self.speed = speed
            if not self.braked:
                self.peak_speed = speed if self.peak_speed is None else max(self.peak_speed, speed)
        if brake is not None and brake >= BRAKE_ONSET:
            self.braked = True
            if self.onset is None and ttc is not None and ttc < HAZARD_TTC:
                self.onset = {"t": t, "speed": speed, "ttc": ttc, "distance": distance}
        if distance is not None and (self.min_distance is None or distance < self.min_distance):
            self.min_distance = distance
        if ttc is not None and (self.min_ttc is None or ttc < self.min_ttc):
            self.min_ttc = ttc


def _round(value, digits=3):
    return round(value, digits) if isinstance(value, (int, float)) and not isinstance(value, bool) else value


def _rounded(event):
    return None if event is None else {key: _round(value) for key, value in event.items()}


def build_metrics(acc, events=None, impact=None, test_speed=None, impact_known=True):
    """
    The metrics dict (the JSON of the file) from an Accumulator, the driver's `events` (ctx.metrics), the first
    collision `impact` ({"t", "speed", "other_type", "other_is_target"} or None) and the ego's target speed [m/s].
    Every value that could not be measured is None; with `impact_known` False (no collision sensor) the impact and
    the speed reduction are None, not "no impact".
    """
    events = events or {}
    # The sampler's brake is only a fallback for runs without a driver (a driver writes at least one event)
    onset = events.get("brake_onset") or (None if any(events.get(k) for k in DRIVER_EVENTS) else acc.onset)
    if test_speed is None:
        test_speed = acc.peak_speed
    impact_speed = impact.get("speed") if impact else 0.0
    reduction = None if test_speed is None or impact_speed is None or not impact_known else test_speed - impact_speed
    return {
        "test_speed": _round(test_speed),
        "first_visible": _rounded(events.get("first_visible")),
        "first_hazard": _rounded(events.get("first_hazard")),
        "brake_onset": _rounded(onset),
        "emergency": _rounded(events.get("emergency")),
        "emergency_count": int(events.get("emergency_count") or 0),
        "impact": {"happened": impact is not None if impact_known else None, "speed": _round(impact.get("speed")) if impact else None,
                   "t": _round(impact.get("t")) if impact else None,
                   "other_type": impact.get("other_type") if impact else None,
                   "other_is_target": impact.get("other_is_target") if impact else None},
        "speed_reduction": _round(reduction),
        "min_distance": _round(acc.min_distance),
        "min_ttc": _round(acc.min_ttc),
        "duration": None if acc.t0 is None else _round(acc.t - acc.t0),
    }


# --- the block ------------------------------------------------------------------------------------------------------

def actor_box(actor, transform):
    """(centre x, centre y, yaw [rad], half length, half width) of an actor's bounding box at `transform`."""
    yaw = math.radians(transform.rotation.yaw)
    try:
        extent, offset = actor.bounding_box.extent, actor.bounding_box.location
    except (AttributeError, RuntimeError):
        extent = offset = None
    if extent is None or extent.x == 0.0:
        half = HALF_LENGTH_DEFAULTS["vehicle" if actor.type_id.startswith("vehicle.") else "walker"]
        return transform.location.x, transform.location.y, yaw, half, half
    cos, sin = math.cos(yaw), math.sin(yaw)
    return (transform.location.x + offset.x * cos - offset.y * sin, transform.location.y + offset.x * sin + offset.y * cos,
            yaw, extent.x, extent.y)


class Sampler(AtomicBehavior):

    """Records the ego (and `ctx.target`) every tick; never succeeds or fails. finish() writes the file once."""

    def __init__(self, ctx, name="MetricsSampler"):
        super().__init__(name)
        self.ctx = ctx
        self.acc = Accumulator()
        self.sensor = None
        self.impact = None
        self.finished = False
        self.sensor_failed = False  # no collision sensor: the impact is unknown
        self.failed = False  # an error in update() was already reported

    def initialise(self):
        try:
            blueprint = self.ctx.library.find(COLLISION_SENSOR)
            self.sensor = self.ctx.world.spawn_actor(blueprint, carla.Transform(), attach_to=self.ctx.ego)
            self.sensor.listen(self._on_collision)
        except Exception as error:  # no sensor: no impact data, the rest still works
            self.sensor = None
            self.sensor_failed = True
            self.ctx.log("metrics", "WARNING no collision sensor ({}): impact is reported as unknown (null)".format(error))

    def _on_collision(self, event):
        # Runs on the client's thread: only the first contact is kept, with the speed of the tick before it
        if self.impact is not None:
            return
        try:
            other = event.other_actor
            target = self.ctx.target
            self.impact = {"t": self.acc.t, "speed": self.acc.speed, "other_type": other.type_id,
                           "other_is_target": target is not None and other.id == target.id}
        except Exception:
            self.impact = {"t": self.acc.t, "speed": self.acc.speed, "other_type": None, "other_is_target": None}

    def update(self):
        try:
            self._sample()
        except Exception as error:  # a failed read (an actor gone) must neither end nor crash the scenario
            if not self.failed:
                self.failed = True
                self.ctx.log("metrics", "WARNING sampling failed ({}); measurements may be incomplete".format(error))
        return py_trees.common.Status.RUNNING

    def _sample(self):
        ego = self.ctx.ego
        velocity = ego.get_velocity()
        speed = math.hypot(velocity.x, velocity.y)
        brake = ego.get_control().brake
        distance = ttc = None
        target = self.ctx.target
        # The main pedestrian walks on after crossing and is removed once hidden: stop measuring it then
        if target is not None and getattr(target, "is_alive", True):
            ego_box = actor_box(ego, ego.get_transform())
            target_box = actor_box(target, target.get_transform())
            target_velocity = target.get_velocity()
            distance = box_distance(ego_box, target_box)
            ttc = time_to_collision(ego_box, target_box, (velocity.x, velocity.y),
                                    (target_velocity.x, target_velocity.y))
        self.acc.add(GameTime.get_time(), speed, brake, distance, ttc)

    def finish(self):
        """Destroy the sensor and write the file; safe to call repeatedly (it acts once)."""
        sensor, self.sensor = self.sensor, None
        if sensor is not None:
            for action in ("stop", "destroy"):
                try:
                    if sensor.is_alive:
                        getattr(sensor, action)()
                except RuntimeError:
                    pass
        if self.finished or self.acc.ticks == 0:
            return
        self.finished = True
        events = dict(self.ctx.metrics)
        test_speed = getattr(self.ctx, "ego_target_speed", None)
        metrics = build_metrics(self.acc, events, self.impact, test_speed, impact_known=not self.sensor_failed)
        target = self.ctx.target
        metrics["target"] = getattr(target, "type_id", None)
        self.ctx.log("metrics", json.dumps(metrics))
        path = os.environ.get(METRICS_FILE_ENV)
        if not path:
            return
        try:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            Path(path).write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
        except OSError as error:
            self.ctx.log("metrics", "WARNING could not write {}: {}".format(path, error))


class Metrics(Block):

    """The sampler and what it needs to write the file at the end (see the module docstring)."""

    tag = "metrics"

    def __init__(self, ctx, params):
        super().__init__(ctx, params)
        self.sampler = Sampler(ctx)

    def from_start(self):
        return [self.sampler]

    def cleanup(self):
        self.sampler.finish()
