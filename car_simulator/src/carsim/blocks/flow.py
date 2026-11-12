"""
The endless flows of a manual run, as py_trees behaviours that never finish (they always return RUNNING).

A manual run has no distance end and no time limit, so the crossing must stay busy for as long as the user drives:
pedestrians, cyclists and background traffic keep arriving and leaving. Nothing may pop up in view, so every spawn
and every removal during the run follows carsim.road.hidden_from_ego (the ego cannot see the spot), the only
exceptions being a cyclist that waits visibly on the sidewalk until its turn and the safety cap of a cyclist that has
finished its ride where the far side has no sidewalk spot to leave to (see CyclistFlow). Actors are spawned without a
tick of their own and removed through CarlaDataProvider.remove_actor_by_id (BlockContext.remove_actor), so
ScenarioRunner's pool stays consistent and its final cleanup removes whatever is left; AI controllers are registered
with the context and destroyed in terminate().

    PedestrianFlow      pedestrians that arrive at the crosswalk, cross it, walk on and are removed out of sight
    CyclistFlow         bicycles that wait on the sidewalk, ride across the cycle lane, leave to the far sidewalk and
                        are removed out of sight
    BackgroundBubble    background traffic and wandering pedestrians kept within a radius of the moving ego

Each behaviour does at most MAX_SPAWNS_PER_TICK spawns per tick and holds its count: it never spawns above it. A
CARLA error (RuntimeError) in one tick does not end the run: the flow logs it (at most every 10 s), waits a second
and carries on. An actor the server would not destroy is not forgotten: BlockContext.retry_removals() (called by every
flow once per tick) tries it again, and the flows count it against their capacity until it is gone. The pedestrian and
cyclist flows pause (no new spawns, nobody new leaves the sidewalk) while the ego is more than twice the hide rule's
far distance from the crossing: what is moving finishes, what is waiting waits.
"""

import math
from dataclasses import dataclass, field

import carla
import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorDestroy, AtomicBehavior, KeepVelocity
from srunner.scenariomanager.timer import GameTime

from carsim.blocks.base import SPAWN_ABOVE
from carsim.road import CYCLE_DECELERATION, SIDEWALK_RETRY_BACK, cyclist_forced_ride, cyclist_wait_retries

MAX_SPAWNS_PER_TICK = 2
FLOW_ERROR_BACKOFF = 1.0  # a flow that hit a CARLA error does nothing for this long [s]
FLOW_ERROR_LOG_INTERVAL = 10.0  # ... and logs it at most this often [s]
FLOW_ACTOR_ERRORS = 3  # a pedestrian or bicycle that is still alive but whose update fails this often is removed
FLOW_PAUSE_FACTOR = 2.0  # pedestrians and cyclists pause while the ego is farther than this times far_distance away

# --- PedestrianFlow ---
FLOW_RADIUS = 60.0  # pedestrians spawn within this distance of the crossing [m]
FLOW_MIN_CROSSING_DISTANCE = 15.0  # ... and at least this far from it [m]
FLOW_SPAWN_INTERVAL = (1.5, 4.0)  # a new pedestrian at most this often: a random time in this range [s]
FLOW_RETRY_DELAY = 0.5  # wait after a spawn that found no spot or failed [s]
FLOW_ARRIVE = 1.5  # a pedestrian has arrived within this distance of its target [m]
FLOW_STUCK_TIME = 10.0  # no progress towards the target for this long: skip to the next leg [s]
FLOW_PROGRESS = 0.5  # progress that counts: the distance to the target shrinks by this much [m]
FLOW_LEG_MAX = 120.0  # the longest a leg may take [s]
FLOW_BEYOND = (30.0, 60.0)  # the final point is this far beyond the far end of the crosswalk [m]
FLOW_LATERAL = 1.0  # targets at the crosswalk ends are spread this far to either side of the walk line [m]
FLOW_LINGER = 8.0  # a pedestrian that finished in view is sent further after this long [s]
FLOW_REPICK_RETRY = 1.0  # a finished pedestrian in view for which no hidden point was found asks again after this long [s]
FLOW_MAX_REPICKS = 20  # a finished pedestrian that is still in view is sent on this many times, then it stays put
FLOW_SAMPLES_PER_TICK = 10  # navigation points drawn per tick while the pools are not full
FLOW_POOL = 60  # navigation points kept to spawn at, and to send a pedestrian to (per crosswalk end)
FLOW_PICKS = FLOW_POOL  # points of a pool looked at for one that is out of sight: all of them, it is cheap
FLOW_REFILL_DRY = 5  # this many passes in a row that added nothing: the pools cannot fill, rest for FLOW_REFILL_COOLDOWN
FLOW_REFILL_COOLDOWN = 5.0  # [s]
FLOW_INITIAL = 4  # pedestrians placed at load, already walking, so the crossing is not empty at first
FLOW_INITIAL_RADIUS = 35.0  # ... at navigation points within this distance of the crossing [m]
FLOW_INITIAL_TRIES = 100  # navigation points drawn per initial pedestrian

# --- CyclistFlow ---
CYCLE_WAIT = (3.0, 12.0)  # a bicycle waits on the sidewalk this long (random) before it rolls onto the road [s]
CYCLE_ROLL_SPEED = 2.0  # speed of the kinematic rolls: sidewalk to the start of the ride, and end of the ride to the far sidewalk [m/s]
CYCLE_LINGER_CAP = 5.0  # no far sidewalk spot: a bicycle that has stopped at the far side is removed after this long, in sight or not [s]
CYCLE_RESPAWN = (0.5, 2.0)  # pause between two bicycle spawns [s]
CYCLE_SLOT_BACKOFF = 5.0  # a waiting spot where the spawn failed is left alone this long [s]
CYCLE_STOP_MAX = 3.0  # braking takes at most this long before the bicycle counts as stopped [s]
CYCLE_BRAKE = 1.0
CYCLE_STOPPED_SPEED = 0.05  # [m/s]
CYCLE_RIDE_SLACK = 6.0  # a ride that takes this much longer than planned is given up (the bicycle was knocked over) [s]
CYCLE_SIDEWALK_DROP = 0.05  # a waiting bicycle floats this far above its spot (physics off); the spawn lifts it 0.2 [m]
CYCLE_LAUNCH_CLEAR = {"vehicle": 3.0, "walker": 0.75}  # a lane does not launch while one of these is this near the ride [m]
CYCLE_LAUNCH_CLEAR_BIKE = 1.0  # ... or a bicycle of the same lane is this near it [m]
CYCLE_RIDE_AHEAD = 4.0  # a riding bicycle brakes for something this far ahead on its line (plus half its length) [m]
CYCLE_ROLL_AHEAD = 2.5  # a rolling bicycle waits for something this far ahead on its line (plus half its length) [m]
CYCLE_LATERAL = {"vehicle": 1.6, "walker": 0.75}  # a thing this near the line, sideways, is on it [m]
CYCLE_BIKE_LATERAL = 0.75  # the same for a bicycle of the same lane [m]
CYCLE_HOLD_MAX = 8.0  # a ride (or the way to the far sidewalk) held up this long in total by something on its line is given up [s]

# --- single pass ---
WALK_ON_POINTS = 20  # navigation points kept beyond each end of the crosswalk, for the single pass to walk on to
WALK_ON_TRIES = 2000  # navigation points drawn at load to find them
WALK_OFF_LINE = 1.0  # a walker this far off its line (knocked by a bicycle, say) stops marching on it and walks on [m]
RUN_LENGTH = 100.0  # a bicycle that has crossed rides on along the far sidewalk for up to this far [m]
RUN_STEP = 2.0  # ... in points this far apart [m]
RUN_MIN = 20.0  # the way on (away from the ego) is taken when it is at least this long, else the longer way [m]
SIDEWALK_BRIDGE = 6.0  # where a sidewalk lane ends, the next one is looked for this far straight ahead [m]

# --- BackgroundBubble ---
BUBBLE_PERIOD = 1.0  # the bubble is looked after this often [s]
BUBBLE_KEEP = 1.3  # a background actor farther than this times the radius from the ego, and out of sight, is removed
BUBBLE_FREE_DISTANCE = 8.0  # a vehicle spawns only where no vehicle is within this distance [m]
BUBBLE_VEHICLE_TRIES = 6  # spawn points tried per vehicle spawn
BUBBLE_WALKER_TRIES = 8  # navigation points tried per pedestrian spawn
WALKER_SPEED = (0.9, 1.7)  # max speed of the wandering pedestrians [m/s]
BICYCLE_SPEED_DIFFERENCE = 50.0  # background bicycles ride this many percent below the speed limit (Traffic Manager)


def make_walker_flow(ctx):
    """The one pedestrian flow of the scenario, from what the pedestrian and crowd blocks asked for."""
    return PedestrianFlow(ctx, ctx.walker_flow_slots, radius=ctx.walker_flow_radius,
                          speed=ctx.walker_flow_speed or (1.1, 1.8))


def _beyond(point, end, outward):
    """True when `point` is FLOW_BEYOND metres from `end`, the end of a crosswalk, on the side `outward` (a unit vector) points to."""
    dx, dy = point.x - end.x, point.y - end.y
    return FLOW_BEYOND[0] <= math.hypot(dx, dy) <= FLOW_BEYOND[1] and dx * outward[0] + dy * outward[1] > 0.0


def dist2d(a, b):
    return math.hypot(a.x - b.x, a.y - b.y)


def segment_distance(point, a, b):
    """2D distance from `point` to the segment a-b (Locations)."""
    dx, dy = b.x - a.x, b.y - a.y
    length2 = dx * dx + dy * dy
    t = 0.0 if length2 <= 0.0 else max(0.0, min(1.0, ((point.x - a.x) * dx + (point.y - a.y) * dy) / length2))
    return math.hypot(point.x - (a.x + dx * t), point.y - (a.y + dy * t))


def _alive(actor):
    try:
        return actor.is_alive
    except RuntimeError:
        return False


class _Flow(AtomicBehavior):

    """
    A behaviour that never ends. update() runs _update(now) and survives a CARLA error: a RuntimeError (an actor that
    is gone, a spawn or a Traffic Manager call the server refused) is logged, at most once per FLOW_ERROR_LOG_INTERVAL,
    the flow then does nothing for FLOW_ERROR_BACKOFF seconds and carries on. It always returns RUNNING.
    """

    def __init__(self, ctx, name):
        super().__init__(name)
        self.ctx = ctx
        self.errors = 0
        self._backoff_until = 0.0
        self._error_logged = -math.inf

    def _update(self, now):
        raise NotImplementedError

    def update(self):
        now = GameTime.get_time()
        self.ctx.retry_removals()
        if now < self._backoff_until:
            return py_trees.common.Status.RUNNING
        try:
            self._update(now)
        except RuntimeError as error:
            self.errors += 1
            self._backoff_until = now + FLOW_ERROR_BACKOFF
            if now - self._error_logged >= FLOW_ERROR_LOG_INTERVAL:
                self._error_logged = now
                self.ctx.log(self.name, "WARNING a CARLA call failed ({}), pausing {:g} s".format(
                    error, FLOW_ERROR_BACKOFF))
        return py_trees.common.Status.RUNNING

    def _paused(self, ego, center):
        """True while the ego is too far from `center` for anyone to see the flow: no new spawns, no new departures."""
        return dist2d(ego.location, center) > FLOW_PAUSE_FACTOR * self.ctx.hide.far_distance


# --------------------------------------------------------------------------------------------------------------------
# Pedestrians


@dataclass
class _Walker:
    actor: object
    controller: object
    speed: float
    legs: list  # targets to walk to, in order; the last one is the final point
    far: int  # index of the far end of the crosswalk (the pedestrian crosses to it)
    leg: int = 0  # the leg it walks first (an initial pedestrian may start on the crosswalk)
    pending: bool = True  # spawned, but the controller is started a tick later
    has_destination: bool = False
    done: bool = False
    done_at: float = 0.0
    repicks: int = 0
    leg_start: float = 0.0
    progress_at: float = 0.0
    best: float = float("inf")
    errors: int = 0


class PedestrianFlow(_Flow):

    """
    Keeps up to `capacity` pedestrians around the crossing. A new one is spawned when there are fewer, at most one every
    1.5 to 4 s, at a navigation point within `radius` of the crossing and at least 15 m from it that the ego cannot see
    (hidden_from_ego), with an AI walker controller. Its route: to the end of the crosswalk on its own side (the nearer
    one), across to the far end, then to a random navigation point 30 to 60 m beyond the crosswalk, preferably one that is
    out of the ego's sight. A leg ends within 1.5 m of its target, or when the pedestrian makes no progress for 10 s
    (stuck: it skips to the next leg). Once it has reached its final point, or got stuck there, it is removed as soon as
    it is hidden_from_ego; if it stays in view for 8 s it is sent on to another point out of sight, again and again (up
    to 20 times), so it never freezes in view holding a slot.

    The controller is started, sent to its first target and given its speed one tick after the spawn (`pending`).
    Navigation points are drawn a few per tick into pools, so a tick stays cheap; a pool that cannot fill (nothing of the
    map is in range) is not sampled every tick forever but rests for a while. Which points are hidden changes with the
    ego, so a full spawn pool is kept fresh with hidden points while a spawn finds none. At load `populate()` places a
    few pedestrians near the crossing, already on their way (load-time spawns are exempt from the hide rule). While the
    ego is far from the crossing (twice the rule's far distance) nobody new is spawned.
    """

    def __init__(self, ctx, capacity, radius=None, speed=(1.1, 1.8), name="PedestrianFlow"):
        super().__init__(ctx, name)
        self.capacity = capacity
        self.radius = radius if radius is not None else FLOW_RADIUS
        self.speed = (min(speed), max(speed))
        crossing = ctx.crossing
        a = crossing.spawn.location
        self.ends = [carla.Location(a.x, a.y, a.z - 1.0), crossing.end]  # ground height; spawn.z is 1 m up
        self.center = crossing.wp.transform.location
        length = dist2d(self.ends[0], self.ends[1]) or 1.0
        self.axis = ((self.ends[1].x - self.ends[0].x) / length, (self.ends[1].y - self.ends[0].y) / length)
        self.walkers = []
        self.spawn_pool = []
        self.destination_pools = ([], [])  # per far end of the crosswalk
        self.next_spawn = 0.0
        self.spawned = 0
        self.removed = 0
        self.initial = 0  # placed at load by populate()
        self._starved = False  # the last spawn found no hidden point in the spawn pool
        self._dry = 0  # passes of _refill in a row that added nothing
        self._cooldown_until = 0.0

    # --- the pools of navigation points ---

    def _outward(self, far):
        """Unit vector pointing away from the crosswalk past its end `far`."""
        sign = 1.0 if far == 1 else -1.0
        return self.axis[0] * sign, self.axis[1] * sign

    def _add(self, pool, point, ego=None):
        """
        Put `point` in `pool`; True when that made progress: the pool grew, or (with `ego`, for the spawn pool) a point
        in view was replaced by a hidden one. A full spawn pool that is starved takes hidden points only.
        """
        if len(pool) < FLOW_POOL:
            pool.append(point)
            return True
        if ego is not None:
            hidden = self.ctx.is_hidden(ego, point)
            visible = [i for i, other in enumerate(pool) if not self.ctx.is_hidden(ego, other)]
            if hidden and visible:
                pool[self.ctx.rng.choice(visible)] = point
                return True
            if not hidden and self._starved:
                return False
        pool[self.ctx.rng.randrange(len(pool))] = point
        return False

    def _refill(self, now, ego):
        if now < self._cooldown_until:
            return
        needs = len(self.spawn_pool) < FLOW_POOL or any(len(pool) < FLOW_POOL // 2 for pool in self.destination_pools)
        if not needs and not self._starved:
            return
        world = self.ctx.world
        progress = False
        for _ in range(FLOW_SAMPLES_PER_TICK):
            point = self.ctx.navigation_point()
            if point is None:
                continue
            if FLOW_MIN_CROSSING_DISTANCE <= dist2d(point, self.center) <= self.radius:
                progress |= self._add(self.spawn_pool, point, ego)
            for far in (0, 1):
                end = self.ends[far]
                if _beyond(point, end, self._outward(far)):
                    progress |= self._add(self.destination_pools[far], point)
        if progress:
            self._dry = 0
        else:
            self._dry += 1
            if self._dry >= FLOW_REFILL_DRY:
                self._dry = 0
                self._cooldown_until = now + FLOW_REFILL_COOLDOWN

    def _pick_hidden(self, pool, ego, avoid=None):
        """A random point of `pool` the ego cannot see (and, with `avoid`, at least 30 m from it), or None."""
        if not pool:
            return None
        for point in self.ctx.rng.sample(pool, min(FLOW_PICKS, len(pool))):
            if self.ctx.is_hidden(ego, point) and (avoid is None or dist2d(point, avoid) >= FLOW_BEYOND[0]):
                return point
        return None

    # --- the walkers ---

    def _legs(self, point):
        """Targets near each end of the crosswalk, spread across its width: (legs, index of the far end)."""
        near = 0 if dist2d(point, self.ends[0]) <= dist2d(point, self.ends[1]) else 1
        far = 1 - near
        nx, ny = -self.axis[1], self.axis[0]
        legs = []
        for index in (near, far):
            lateral = self.ctx.rng.uniform(-FLOW_LATERAL, FLOW_LATERAL)
            end = self.ends[index]
            legs.append(carla.Location(x=end.x + nx * lateral, y=end.y + ny * lateral, z=end.z))
        return legs, far

    def populate(self, count=FLOW_INITIAL):
        """
        Place up to `count` pedestrians at load, at navigation points within FLOW_INITIAL_RADIUS of the crossing (a
        third of them on the crosswalk itself), each on its usual route; their controllers are started by the first
        update, so they are walking when the run begins. Returns how many were placed. Call it after the blocks have
        spawned and before the behaviour tree runs; spawns at load are not bound by the hide rule.
        """
        wanted = min(count, self.capacity)
        world, rng = self.ctx.world, self.ctx.rng
        tries = 0
        while len(self.walkers) < wanted and tries < wanted * FLOW_INITIAL_TRIES:
            tries += 1
            point = self.ctx.navigation_point()
            if point is None or dist2d(point, self.center) > FLOW_INITIAL_RADIUS:
                continue
            legs, far = self._legs(point)
            spot, leg = point, 0
            if rng.random() < 1.0 / 3.0:
                fraction = rng.uniform(0.15, 0.85)
                spot = carla.Location(legs[0].x + (legs[1].x - legs[0].x) * fraction,
                                      legs[0].y + (legs[1].y - legs[0].y) * fraction, legs[0].z)
                leg = 1
            walker, controller = self.ctx.spawn_walker(spot)
            if walker is None:
                continue
            self.walkers.append(_Walker(walker, controller, rng.uniform(*self.speed), legs, far, leg=leg))
            self.spawned += 1
            self.initial += 1
        return self.initial

    def _spawn(self, now, ego):
        spawned = 0
        while (spawned < MAX_SPAWNS_PER_TICK and len(self.walkers) + self.ctx.pending_removals(self) < self.capacity
               and now >= self.next_spawn):
            point = self._pick_hidden(self.spawn_pool, ego)
            if point is None:
                self._starved = True
                self.next_spawn = now + FLOW_RETRY_DELAY
                return
            self._starved = False
            walker, controller = self.ctx.spawn_walker(point)
            if walker is None:
                self.next_spawn = now + FLOW_RETRY_DELAY
                return
            legs, far = self._legs(point)
            self.walkers.append(_Walker(walker, controller, self.ctx.rng.uniform(*self.speed), legs, far))
            self.spawned += 1
            spawned += 1
            self.next_spawn = now + self.ctx.rng.uniform(*FLOW_SPAWN_INTERVAL)

    def _go(self, walker, now, leg):
        """Send the pedestrian to leg `leg`; it is on that leg only once the controller took the order."""
        walker.controller.go_to_location(walker.legs[leg])
        walker.leg = leg
        walker.leg_start = walker.progress_at = now
        walker.best = float("inf")

    def _start(self, walker, now):
        walker.controller.start()
        walker.controller.go_to_location(walker.legs[walker.leg])
        walker.controller.set_max_speed(walker.speed)
        walker.pending = False
        walker.leg_start = walker.progress_at = now

    def _next_leg(self, walker, now, ego):
        leg = walker.leg + 1
        if leg == 2 and not walker.has_destination:
            walker.has_destination = True
            pool = self.destination_pools[walker.far]
            point = self._pick_hidden(pool, ego) or (self.ctx.rng.choice(pool) if pool else None)
            if point is not None:
                walker.legs.append(point)
        if leg >= len(walker.legs):
            walker.done, walker.done_at = True, now
            return
        self._go(walker, now, leg)

    def _send_further(self, walker, now, ego):
        """A finished pedestrian still in view walks on to another point out of sight."""
        pool = self.spawn_pool + self.destination_pools[0] + self.destination_pools[1]
        point = self._pick_hidden(pool, ego, avoid=self.ctx.location_of(walker.actor))
        if point is None:
            walker.done_at = now - FLOW_LINGER + FLOW_REPICK_RETRY  # none out of sight right now: ask again soon
            return
        walker.legs.append(point)
        try:
            self._go(walker, now, len(walker.legs) - 1)
        except RuntimeError:
            walker.legs.pop()
            raise
        walker.repicks += 1
        walker.done_at = now
        walker.done = False

    def _remove(self, walker):
        if walker in self.walkers:
            self.walkers.remove(walker)
            self.removed += 1
        self.ctx.remove_actor(walker.actor, walker.controller, group=self)

    def _drop(self, walker):
        """A pedestrian that cannot be looked after (gone, or failing): out of the flow and off the server."""
        if walker in self.walkers:
            self.walkers.remove(walker)
        self.ctx.remove_actor(walker.actor, walker.controller, group=self)

    def _step(self, walker, now, ego):
        if walker.pending:
            return
        location = self.ctx.location_of(walker.actor)
        if walker.done:
            if self.ctx.is_hidden(ego, location):
                self._remove(walker)
            elif now - walker.done_at >= FLOW_LINGER and walker.repicks < FLOW_MAX_REPICKS:
                self._send_further(walker, now, ego)
            return
        distance = dist2d(location, walker.legs[walker.leg])
        if distance < walker.best - FLOW_PROGRESS:
            walker.best, walker.progress_at = distance, now
        if (distance < FLOW_ARRIVE or now - walker.progress_at > FLOW_STUCK_TIME
                or now - walker.leg_start > FLOW_LEG_MAX):
            self._next_leg(walker, now, ego)

    def _guarded(self, walker, action):
        """Run `action(walker)`; a CARLA error drops a pedestrian that is gone or keeps failing, else it tries again."""
        try:
            action(walker)
        except RuntimeError:
            walker.errors += 1
            if not _alive(walker.actor) or walker.errors >= FLOW_ACTOR_ERRORS:
                self._drop(walker)

    def _update(self, now):
        ego = self.ctx.ego_transform()
        for walker in list(self.walkers):
            if not _alive(walker.actor):
                self._drop(walker)
            elif walker.pending:
                self._guarded(walker, lambda w: self._start(w, now))
        for walker in list(self.walkers):
            if walker in self.walkers:
                self._guarded(walker, lambda w: self._step(w, now, ego))
        if self._paused(ego, self.center):
            return
        self._refill(now, ego)
        self._spawn(now, ego)


# --------------------------------------------------------------------------------------------------------------------
# Cyclists


@dataclass
class CycleLane:
    """
    One direction of the cycle crossing: `waits` are the waiting spots on the sidewalk (index = queue position, 0 the
    front), `start` the transform on the road where the ride starts, `speed` the lane's one riding speed and `push`
    how far the bicycle is pushed at that speed before it brakes (cyclist_forced_ride). `ride` is the length of the ride
    [m] and `exit` the Transform on the far sidewalk (road.cyclist_exit_spot) a bicycle leaves to after its ride, or
    None when there is no such spot. `bikes` is the occupied waiting spots, queue position -> _Bike.
    """
    side: str
    waits: list
    start: carla.Transform
    speed: float
    push: float
    ride: float = 0.0
    exit: object = None
    bikes: dict = field(default_factory=dict)
    active: object = None  # the bicycle on the road (rolling, riding or at the far side), or None
    blocked_until: dict = field(default_factory=dict)  # queue position -> time until which the spawn is not retried

    @property
    def yaw(self):
        return math.radians(self.start.rotation.yaw)

    @property
    def end(self):
        """Where the ride ends, on the road."""
        return carla.Location(self.start.location.x + math.cos(self.yaw) * self.ride,
                              self.start.location.y + math.sin(self.yaw) * self.ride, self.start.location.z)


@dataclass
class _Bike:
    actor: object
    lane: CycleLane
    position: int  # queue position on the sidewalk
    origin: object = None  # the transform it was actually spawned at (a retry spot may be further back than the lane's)
    ready_at: float = None  # when it may leave (None: drawn at the first tick; set when spawned during the run)
    state: str = "waiting"  # waiting, rolling, riding, stopping, leaving, parked (a held-up leave ends so too, and glides on), lingering
    since: float = 0.0
    rolled: float = 0.0
    last: object = None
    ridden: float = 0.0
    glide: float = CYCLE_ROLL_SPEED  # speed of its kinematic glides [m/s] (the single pass rides on at the lane's speed)
    held: float = 0.0  # how long something on its line has held it up, during the ride or on the way to the far sidewalk [s]
    leave_from: object = None
    leave_to: object = None
    errors: int = 0

    @property
    def start_point(self):
        """Where the roll to the road starts: the spot it was spawned at."""
        return (self.origin if self.origin is not None else self.lane.waits[self.position]).location


def spawn_waiting_bike(ctx, cmap, model, wait, ego=None):
    """
    Spawn a bicycle on the sidewalk at `wait` (a Transform, see road.cyclist_waiting_spots) with its physics off, so it
    cannot fall or roll; if that spot cannot be spawned, a spot further back along the line is tried
    (road.cyclist_wait_retries). With `ego` (a Transform) only a spot that ctx.is_hidden from it is tried: the retry
    spots are not the one the caller looked at. Returns (actor, the transform used), or (None, None). Not registered
    with the scenario: the caller does that (a load-time bicycle with ctx.add_actor, a bicycle of a flow stays in the
    data provider's pool).
    """
    tries = [(0.0, wait)] + list(cyclist_wait_retries(cmap, wait, SIDEWALK_RETRY_BACK))
    for extra, tf in tries:
        if ego is not None and not ctx.is_hidden(ego, tf.location):
            continue
        bike = CarlaDataProvider.request_new_actor(model, ctx.surface(tf), actor_category="bicycle", tick=False)
        if bike is None:
            continue
        try:
            bike.set_simulate_physics(False)
            # request_new_actor lifted it 0.2 m; with the physics off it would float, so set it down on the spot
            bike.set_transform(ctx.surface(carla.Transform(
                carla.Location(tf.location.x, tf.location.y, tf.location.z + CYCLE_SIDEWALK_DROP), tf.rotation)))
        except RuntimeError:
            ctx.remove_actor(bike)  # it would fall over with its physics on: do not keep it
            continue
        return bike, tf
    return None, None


class CyclistFlow(_Flow):

    """
    Keeps `count` bicycles cycling. A bicycle waits visibly on a sidewalk spot beyond the kerb (physics off), for a random
    3 to 12 s, then rolls to the start of the ride just inside the kerb (a short kinematic roll, physics off), has its
    physics switched on and is pushed across at the lane's speed with a forced velocity (as KeepVelocity does), braked
    to a stop (as StopVehicle does) before the far kerb. Then it does not vanish in view: its physics go off again and it
    rolls kinematically along its own line to the far sidewalk (the waiting spot of the opposite direction, road.
    cyclist_exit_spot), the lane is free for the next one at once, and the bicycle is removed once it is hidden_from_ego
    (a lane without such a spot keeps the old rule: removed once hidden, or after at most 5 s at the far side). A new
    bicycle is spawned on a free waiting spot of either side, only when the spot is hidden_from_ego, the spot it is
    actually spawned at (a retry spot a little further back included; load-time bicycles are placed by the block
    before the run starts).

    Rules that keep the bicycles apart: a lane has one bicycle on the road at a time (the next one leaves when the
    previous one has been removed, or has left the road for the far sidewalk and is no longer on the way: one that
    is still leaving or parked there, up to the exit spot, holds the next bicycle on the sidewalk), and the bicycle
    nearest the kerb of a lane leaves first, so nobody rolls through a waiting one. A bicycle on its way to the far
    sidewalk that something has held up for CYCLE_HOLD_MAX seconds is parked where it is until it is hidden (it
    glides on if its line clears first). The lanes are the ones of road.cyclist_spawns, which keep clear of each other and of the crowd.

    The flow looks at what else is around (ego, vehicles, pedestrians, from CarlaDataProvider's cache): a lane does not
    launch while the ego or another vehicle is within about 3 m (plus half its length), or a pedestrian within 0.75 m,
    of the ride from the waiting spot to its end (to the exit spot on the far sidewalk, where there is one); a riding
    bicycle stops pushing and brakes while something is on its line within 4 m ahead, and goes on when it is clear; a
    rolling one waits. The bicycles of the other lane do not count (the lanes are apart by design), and neither do
    pedestrians crossing the zebra beside the lane. While the ego is far from the crossing (twice the rule's far
    distance) nobody leaves the sidewalk and nobody is spawned.
    """

    def __init__(self, ctx, lanes, count, models, name="CyclistFlow"):
        super().__init__(ctx, name)
        self.lanes = lanes
        self.count = count
        self.models = models
        self.bikes = []
        self.next_spawn = 0.0
        self.last_time = None
        self.spawned = 0
        self.removed = 0
        self.held_up = 0  # launches put off and rides held up because something was in the way (counted per tick)

    # --- placing bicycles ---

    def free_spots(self, now):
        """(lane, queue position) of the lowest free waiting spot of each lane that is not backed off."""
        spots = []
        for lane in self.lanes:
            if lane.active is not None and lane.active.state == "rolling":
                continue  # it rolls over the spots of its queue
            for position, wait in enumerate(lane.waits):
                if position in lane.bikes:
                    continue
                if lane.blocked_until.get(position, 0.0) <= now:
                    spots.append((lane, position))
                break
        return spots

    def place(self, lane, position, now, ready_at=None, ego=None):
        """
        Spawn a bicycle on a waiting spot (with `ego`, only at a spot hidden from it); True when it worked, else the spot
        is backed off and a line is logged.
        """
        ctx = self.ctx
        bike, used = spawn_waiting_bike(ctx, ctx.map, ctx.rng.choice(self.models), lane.waits[position], ego=ego)
        if bike is None:
            lane.blocked_until[position] = now + CYCLE_SLOT_BACKOFF
            ctx.log("cyclists", "no bicycle could be spawned on the sidewalk spot {} of lane {}: skipped".format(
                position, lane.side))
            return False
        record = _Bike(bike, lane, position, origin=used, ready_at=ready_at, since=now)
        lane.bikes[position] = record
        self.bikes.append(record)
        self.spawned += 1
        return True

    def _spawn(self, now, ego):
        spawned = 0
        while (spawned < MAX_SPAWNS_PER_TICK and len(self.bikes) + self.ctx.pending_removals(self) < self.count
               and now >= self.next_spawn):
            options = [(lane, position) for lane, position in self.free_spots(now)
                       if self.ctx.is_hidden(ego, lane.waits[position].location)]
            if not options:
                return
            lane, position = self.ctx.rng.choice(options)
            self.next_spawn = now + self.ctx.rng.uniform(*CYCLE_RESPAWN)
            if self.place(lane, position, now, ready_at=now + self.ctx.rng.uniform(*CYCLE_WAIT), ego=ego):
                spawned += 1
            else:
                return

    # --- what is in the way ---

    def _own(self):
        return {bike.actor.id: bike for bike in self.bikes}

    def _in_the_way(self, bike, here, fx, fy, ahead):
        """Something on the line from `here` along the unit vector (fx, fy), within `ahead` metres (plus half its length)."""
        own = self._own()
        for actor, kind, location, half in self.ctx.solid_things():
            if actor.id == bike.actor.id:
                continue
            other = own.get(actor.id)
            if other is not None and other.lane is not bike.lane:
                continue  # the other lane is clear of this one by design
            dx, dy = location.x - here.x, location.y - here.y
            along = dx * fx + dy * fy
            side = abs(-dx * fy + dy * fx)
            lateral = CYCLE_BIKE_LATERAL if other is not None else CYCLE_LATERAL[kind]
            if -0.5 <= along <= ahead + half and side <= lateral:
                return True
        return False

    def _ride_blocked(self, bike):
        """
        Something the bicycle waiting at `bike` would ride into: the ego, a vehicle or a pedestrian near its ride, or a
        bicycle of its lane near it. The ride counts up to the exit spot on the far sidewalk where there is one, as the
        bicycle goes on there after the ride: one that is still leaving or parked on the way holds it back.
        """
        lane = bike.lane
        a, b = bike.start_point, (lane.end if lane.exit is None else lane.exit.location)
        own = self._own()
        for actor, kind, location, half in self.ctx.solid_things():
            if actor.id == bike.actor.id:
                continue
            other = own.get(actor.id)
            if other is not None and other.lane is not lane:
                continue
            clear = CYCLE_LAUNCH_CLEAR_BIKE if other is not None else (
                CYCLE_LAUNCH_CLEAR[kind] + (half if kind == "vehicle" else 0.0))
            if segment_distance(location, a, b) <= clear:
                return True
        return False

    # --- one bicycle ---

    def _forget(self, bike):
        if bike in self.bikes:
            self.bikes.remove(bike)
        if bike.lane.bikes.get(bike.position) is bike:
            del bike.lane.bikes[bike.position]
        if bike.lane.active is bike:
            bike.lane.active = None

    def _remove(self, bike):
        if bike in self.bikes:
            self.removed += 1
        self._forget(bike)
        self.ctx.remove_actor(bike.actor, group=self)

    def _glide(self, bike, dt, start, goal, rotation, ahead):
        """
        Move the bicycle kinematically (physics off) along start -> goal by CYCLE_ROLL_SPEED * dt, from where it is
        (`bike.rolled` metres along); it waits while something is in its way. True once it is at `goal`.
        """
        length = dist2d(start, goal)
        if length > 0.0 and bike.rolled < length:
            f = bike.rolled / length
            here = carla.Location(start.x + (goal.x - start.x) * f, start.y + (goal.y - start.y) * f, start.z)
            if self._in_the_way(bike, here, (goal.x - start.x) / length, (goal.y - start.y) / length, ahead):
                self.held_up += 1
                return False
        bike.rolled = length if length <= 0.0 else min(length, bike.rolled + bike.glide * dt)
        f = bike.rolled / length if length > 0.0 else 1.0
        bike.actor.set_transform(self.ctx.surface(carla.Transform(
            carla.Location(start.x + (goal.x - start.x) * f, start.y + (goal.y - start.y) * f,
                           start.z + (goal.z - start.z) * f), rotation)))
        return f >= 1.0

    def _roll(self, bike, now, dt):
        lane = bike.lane
        wait, start = bike.start_point, lane.start.location
        begin = carla.Location(wait.x, wait.y, wait.z + CYCLE_SIDEWALK_DROP)
        if self._glide(bike, dt, begin, start, lane.start.rotation, CYCLE_ROLL_AHEAD):
            # Free the waiting spot of the queue: the next one may leave once this bicycle is gone
            if lane.bikes.get(bike.position) is bike:
                del lane.bikes[bike.position]
            bike.actor.set_target_velocity(carla.Vector3D(0, 0, 0))
            bike.actor.set_simulate_physics(True)
            bike.state, bike.since, bike.last, bike.ridden, bike.held = "riding", now, start, 0.0, 0.0

    def _ride(self, bike, now, dt):
        lane = bike.lane
        here = self.ctx.location_of(bike.actor)
        bike.ridden += math.hypot(here.x - bike.last.x, here.y - bike.last.y)
        bike.last = here
        fx, fy = math.cos(lane.yaw), math.sin(lane.yaw)
        if self._in_the_way(bike, here, fx, fy, CYCLE_RIDE_AHEAD):
            # Do not push it into whatever is on its line: let it brake, and carry on when the line is clear
            bike.held += dt
            bike.since += dt  # the wait does not count against the ride's time
            self.held_up += 1
            bike.actor.apply_control(carla.VehicleControl(brake=CYCLE_BRAKE))
            if bike.held > CYCLE_HOLD_MAX:
                bike.state, bike.since = "stopping", now
            return
        bike.actor.set_target_velocity(carla.Vector3D(fx * lane.speed, fy * lane.speed, 0))
        bike.actor.apply_control(carla.VehicleControl(throttle=1.0))
        if bike.ridden > lane.push or now - bike.since > lane.push / lane.speed + CYCLE_RIDE_SLACK:
            bike.state, bike.since = "stopping", now

    def _stop(self, bike, now):
        bike.actor.apply_control(carla.VehicleControl(brake=CYCLE_BRAKE))
        if CarlaDataProvider.get_velocity(bike.actor) <= CYCLE_STOPPED_SPEED or now - bike.since > CYCLE_STOP_MAX:
            if bike.lane.exit is not None:
                self._start_leaving(bike, now)
            else:
                bike.state, bike.since = "lingering", now
                bike.actor.apply_control(carla.VehicleControl(brake=CYCLE_BRAKE, hand_brake=True))

    def _start_leaving(self, bike, now):
        """Physics off, and on to the far sidewalk along its own line; the lane is free again at once."""
        lane = bike.lane
        bike.actor.set_target_velocity(carla.Vector3D(0, 0, 0))
        bike.actor.set_simulate_physics(False)
        exit = lane.exit.location
        here = self.ctx.location_of(bike.actor)  # on the mesh: take the surface offset off, _glide adds it back
        bike.leave_from = carla.Location(here.x, here.y, 2 * here.z - self.ctx.surface(here).z)
        bike.leave_to = carla.Location(exit.x, exit.y, exit.z + CYCLE_SIDEWALK_DROP)
        bike.rolled = bike.held = 0.0
        bike.state, bike.since = "leaving", now
        if lane.active is bike:
            lane.active = None

    def _leave(self, bike, now, dt):
        before = bike.rolled
        if self._glide(bike, dt, bike.leave_from, bike.leave_to, bike.lane.exit.rotation, CYCLE_ROLL_AHEAD):
            bike.state, bike.since = "parked", now
        elif bike.rolled == before:
            # Held up by something on its line: after CYCLE_HOLD_MAX seconds it is parked where it is, to be removed once
            # hidden (_step lets it glide on if its line clears first)
            bike.held += dt
            if bike.held > CYCLE_HOLD_MAX:
                bike.state, bike.since = "parked", now

    def _step(self, bike, now, dt, ego):
        if not _alive(bike.actor):
            self._forget(bike)
            self.ctx.remove_actor(bike.actor, group=self)
            return
        if bike.state == "waiting":
            if bike.ready_at is None:
                bike.ready_at = now + self.ctx.rng.uniform(*CYCLE_WAIT)
        elif bike.state == "rolling":
            self._roll(bike, now, dt)
        elif bike.state == "riding":
            self._ride(bike, now, dt)
        elif bike.state == "stopping":
            self._stop(bike, now)
        elif bike.state == "leaving":
            self._leave(bike, now, dt)
        else:  # parked on the far sidewalk, or lingering at the far side of a lane without a sidewalk spot
            hidden = self.ctx.is_hidden(ego, self.ctx.location_of(bike.actor))
            if hidden or (bike.state == "lingering" and now - bike.since >= CYCLE_LINGER_CAP):
                self._remove(bike)
            elif (bike.state == "parked" and bike.leave_from is not None
                  and bike.rolled < dist2d(bike.leave_from, bike.leave_to)):
                # A leave that was held up too long stopped short of the exit spot: it goes on once its line is clear
                self._glide(bike, dt, bike.leave_from, bike.leave_to, bike.lane.exit.rotation, CYCLE_ROLL_AHEAD)

    def _launch(self, now):
        for lane in self.lanes:
            if lane.active is not None or not lane.bikes:
                continue
            front = lane.bikes[min(lane.bikes)]
            if front.state == "waiting" and front.ready_at is not None and now >= front.ready_at:
                if self._ride_blocked(front):
                    self.held_up += 1
                    continue
                front.state, front.since, front.rolled = "rolling", now, 0.0
                lane.active = front

    def _update(self, now):
        dt = self.ctx.time_step() if self.last_time is None else min(max(now - self.last_time, 0.0), 0.25)
        self.last_time = now
        ego = self.ctx.ego_transform()
        for bike in list(self.bikes):
            try:
                self._step(bike, now, dt, ego)
            except RuntimeError:
                bike.errors += 1
                if not _alive(bike.actor) or bike.errors >= FLOW_ACTOR_ERRORS:  # gone from the server, or failing for good
                    self._forget(bike)
                    self.ctx.remove_actor(bike.actor, group=self)
        if self._paused(ego, self.ctx.trigger_location):
            return
        self._launch(now)
        self._spawn(now, ego)


def make_lane(spot, waits, speed, step, exit=None):
    """
    A CycleLane for a CyclistSpot, its waiting spots (road.cyclist_waiting_spots), its riding speed [m/s] and the
    Transform on the far sidewalk to leave to (road.cyclist_exit_spot), or None.
    """
    start = carla.Transform(carla.Location(spot.spawn.location.x, spot.spawn.location.y,
                                           spot.spawn.location.z + 0.2), spot.spawn.rotation)
    push = max(cyclist_forced_ride(spot.distance, speed, step, CYCLE_DECELERATION), 0.1)
    return CycleLane(spot.side, waits, start, speed, push, ride=spot.distance, exit=exit)


# --------------------------------------------------------------------------------------------------------------------
# Single pass (autopilot ego): after the crossing the pedestrians walk on and the bicycles ride on, out of sight, the way
# the endless flows do it. Each of these behaviours ends its chain and never finishes (a node under Main that returned
# SUCCESS would end the scenario).


class WalkLine(KeepVelocity):

    """
    KeepVelocity that, with `line` (start, end Locations), also ends when the walker is more than WALK_OFF_LINE off it: one
    knocked aside by a bicycle then goes on with the walk on from where it stands, instead of marching on at its old heading.
    """

    def __init__(self, walker, speed, line=None, **kwargs):
        super().__init__(walker, speed, **kwargs)
        self._line = line

    def update(self):
        status = super().update()
        if status == py_trees.common.Status.RUNNING and self._line is not None:
            if segment_distance(CarlaDataProvider.get_location(self._actor), *self._line) > WALK_OFF_LINE:
                return py_trees.common.Status.SUCCESS
        return status


def attach_walker_controller(ctx, walker):
    """
    An AI walker controller attached to `walker` (registered with the context, destroyed by remove_controllers()) and not
    started, or None when the server refuses it (logged).
    """
    try:
        controller = ctx.world.spawn_actor(ctx.library.find("controller.ai.walker"), carla.Transform(), attach_to=walker)
    except RuntimeError as error:
        ctx.log("walkers", "WARNING no AI controller for pedestrian {}: {}".format(walker.id, error))
        return None
    ctx.add_controller(controller)
    return controller


class WalkOnPoints:

    """
    Navigation points FLOW_BEYOND metres beyond each end of the crosswalk, drawn once at load, where the pedestrians of the
    single pass walk on to. pick() prefers one the ego cannot see.
    """

    def __init__(self, ctx):
        self.ctx = ctx
        crossing = ctx.crossing
        a = crossing.spawn.location
        self.ends = [carla.Location(a.x, a.y, a.z - 1.0), crossing.end]  # ground height; spawn.z is 1 m up
        length = dist2d(self.ends[0], self.ends[1]) or 1.0
        axis = ((self.ends[1].x - self.ends[0].x) / length, (self.ends[1].y - self.ends[0].y) / length)
        self.outward = [(-axis[0], -axis[1]), axis]
        self.pools = ([], [])
        for _ in range(WALK_ON_TRIES):
            if all(len(pool) >= WALK_ON_POINTS for pool in self.pools):
                break
            point = ctx.navigation_point()
            if point is None:
                continue
            for pool, end, outward in zip(self.pools, self.ends, self.outward):
                if len(pool) < WALK_ON_POINTS and _beyond(point, end, outward):
                    pool.append(point)

    def far(self, location):
        """Index of the end of the crosswalk nearest `location`: the end a pedestrian who walks to it walks on beyond."""
        return 0 if dist2d(location, self.ends[0]) <= dist2d(location, self.ends[1]) else 1

    def pick(self, far, ego):
        """A point beyond end `far` that the ego (a Transform) cannot see, else any of them, else None."""
        pool = self.pools[far]
        hidden = [point for point in pool if self.ctx.is_hidden(ego, point)]
        return self.ctx.rng.choice(hidden or pool) if pool else None


def walk_on_points(ctx):
    """The scenario's WalkOnPoints (made on the first call, at load), or None on a map without a navigation mesh (logged once)."""
    if not hasattr(ctx, "walk_on"):
        ctx.walk_on = WalkOnPoints(ctx) if ctx.navigation_available() else None
        if ctx.walk_on is None:
            ctx.log("walkers", "no navigation mesh for this map: the pedestrians stay on the far kerb after crossing")
    return ctx.walk_on


CLEAR_AFTER = 2.5  # [m] a pedestrian that ended off the sidewalk walks at least this much further (WalkOn)
CLEAR_TIMEOUT = 12.0  # [s] and at most this long


class WalkOn(_Flow):

    """
    The end of a pedestrian's crossing in the single pass: the first update starts its AI walker controller and sends it, at
    `speed`, to a navigation point FLOW_BEYOND metres beyond the far end `far` of the crosswalk (one out of the ego's sight when
    there is one); the pedestrian is removed, controller first, once it is hidden_from_ego. Where no point was found it stays.
    """

    def __init__(self, ctx, walker, controller, speed, points, far, name="WalkOn"):
        super().__init__(ctx, name)
        self.walker, self.controller, self.speed, self.points, self.far = walker, controller, speed, points, far
        self.started = False
        self.gone = False  # removed, or with nowhere to go
        self.clearing = None  # (where it ended, since when) while it walks on off the sidewalk

    def _clear_off_road(self, now):
        """
        A pedestrian that ended its crossing off the sidewalk (a 10 m median on WolfsburgHighway) gets no AI controller:
        started off the navmesh they crashed or froze the 0.9.15 server (OccludedCrossing_Crowd, 2026-11-10). It walks on
        straight (WalkerControl) until it is off every driving lane and CLEAR_AFTER m past where it ended (at most
        CLEAR_TIMEOUT s), then stands: standing on the road it would be a crosser for ever, and the crowd's next
        pedestrian of its column, held up behind it, too. Returns True while it is still walking.
        """
        ctx, here = self.ctx, self.ctx.location_of(self.walker)
        if self.clearing is None:
            self.clearing = (here, now)
            ctx.log("walkers", "pedestrian {} ended off the sidewalk: it walks off the road and stays".format(self.walker.id))
        start, since = self.clearing
        on_road = ctx.map.get_waypoint(here, project_to_road=False, lane_type=carla.LaneType.Driving) is not None
        if (on_road or dist2d(here, start) < CLEAR_AFTER) and now - since < CLEAR_TIMEOUT:
            ahead = self.walker.get_transform().get_forward_vector()
            self.walker.apply_control(carla.WalkerControl(direction=carla.Vector3D(ahead.x, ahead.y, 0.0), speed=self.speed))
            return True
        self.walker.apply_control(carla.WalkerControl(speed=0.0))
        return False

    def _update(self, now):
        if self.gone:
            return
        ctx = self.ctx
        ego = ctx.ego_transform()
        if not _alive(self.walker):
            self.gone = True
            ctx.remove_actor(self.walker, self.controller, group=self)
        elif not self.started:
            if self.clearing is not None or not ctx.on_sidewalk(ctx.location_of(self.walker)):
                self.gone = not self._clear_off_road(now)
                return
            point = self.points.pick(self.far, ego)
            if point is None:
                self.gone = True
                ctx.log("walkers", "no navigation point beyond the crosswalk: pedestrian {} stays".format(self.walker.id))
                return
            self.controller.start()
            self.controller.go_to_location(point)
            self.controller.set_max_speed(self.speed)
            self.started = True
        elif ctx.is_hidden(ego, ctx.location_of(self.walker)):
            self.gone = True
            ctx.remove_actor(self.walker, self.controller, group=self)


def sidewalk_run(ctx, start, heading, length=RUN_LENGTH, step=RUN_STEP):
    """
    Points `step` metres apart along the sidewalk lane `start` (a Location on a sidewalk) is on, following the lane
    (round corners too) both ways, at most `length` metres, until a point is over a driving lane. Where a lane ends (a
    junction corner's sidewalk has no successor in CARLA) it goes on along the next sidewalk lane found up to
    SIDEWALK_BRIDGE metres straight ahead. The way that starts along `heading` (a unit vector (x, y), the road's way on,
    away from the ego) if it gives RUN_MIN metres, else the longer way. The z is the sidewalk's. [] when `start` is not
    on a sidewalk.
    """
    lane = ctx.map.get_waypoint(start, project_to_road=True, lane_type=carla.LaneType.Sidewalk)
    # 6 m: at a junction corner the exit spot can be 4.6 m from the corner's sidewalk lane (Town10HD, live)
    if lane is None or lane.transform.location.distance(start) > 6.0:
        return []
    ways = []
    for forward in (True, False):
        points, wp, last = [], lane, lane.transform.location
        while len(points) * step < length:
            following = wp.next(step) if forward else wp.previous(step)
            if not following:
                bridged = _bridge(ctx, wp, last, step)
                if bridged is None:
                    break
                following, forward = bridged
            last = wp.transform.location
            wp = following[0]
            here = wp.transform.location
            if ctx.map.get_waypoint(here, project_to_road=False, lane_type=carla.LaneType.Driving):
                break
            points.append(carla.Location(here.x, here.y, here.z + CYCLE_SIDEWALK_DROP))
        ways.append(points)
    # the way that starts along the road's way on first, as before
    first = [w for w in ways if w and (w[0].x - start.x) * heading[0] + (w[0].y - start.y) * heading[1] > 0]
    other = [w for w in ways if w is not (first[0] if first else None)]
    on = first[0] if first else (other.pop(0) if other else [])
    back = other[0] if other else []
    return on if len(on) * step >= RUN_MIN or len(on) >= len(back) else back


def _bridge(ctx, wp, before, step):
    """
    Where the sidewalk lane of `wp` ends (`before`: the point before it): ([first waypoint], forward) on the next sidewalk
    lane found straight ahead within SIDEWALK_BRIDGE metres, forward telling whether that lane goes on with next()
    (True) or previous(), or None.
    """
    end = wp.transform.location
    dx, dy = end.x - before.x, end.y - before.y
    norm = math.hypot(dx, dy)
    if norm < 1e-6:
        return None
    dx, dy = dx / norm, dy / norm
    probe_at = 1.5
    while probe_at <= SIDEWALK_BRIDGE:
        probe = carla.Location(end.x + dx * probe_at, end.y + dy * probe_at, end.z)
        other = ctx.map.get_waypoint(probe, project_to_road=True, lane_type=carla.LaneType.Sidewalk)
        if (other is not None and (other.road_id, other.lane_id) != (wp.road_id, wp.lane_id)
                and other.transform.location.distance(probe) < 1.0):
            for forward in (True, False):
                following = other.next(step) if forward else other.previous(step)
                if following:
                    there = following[0].transform.location
                    if (there.x - end.x) * dx + (there.y - end.y) * dy > 0:
                        return [other], forward
        probe_at += 1.5
    return None


class RideOn(_Flow):

    """
    The end of a bicycle's ride in the single pass: it does not stop and does not park. It rolls on at once to the far
    sidewalk (as CyclistFlow does, `flow` being a CyclistFlow that is not in the tree and keeps the _Bike records `bike`
    belongs to, so bicycles of the other lane are not in the way), then rides on along the sidewalk (sidewalk_run, at the
    lane's speed, physics off, waiting while something is on its line), turning back at the end of the run, and is
    removed once it is hidden_from_ego. Without a far sidewalk spot it brakes to a stop at the far side and is removed
    once hidden, or after CYCLE_LINGER_CAP seconds. Where the sidewalk gives no run, or it is held up for CYCLE_HOLD_MAX
    seconds, it ends the same way (lingering).
    """

    def __init__(self, flow, bike, name="RideOn"):
        super().__init__(flow.ctx, name)
        self.flow, self.bike = flow, bike
        self.begun = False
        self.last = None
        self.route = []  # the sidewalk points to ride along, from the far sidewalk spot on
        self.at, self.way = 0, 1  # index of the route point it is at, and the way it rides along the route
        exit = bike.lane.exit
        if exit is not None:
            fwd = self.ctx.crossing.wp.transform.get_forward_vector()
            run = sidewalk_run(self.ctx, exit.location, (fwd.x, fwd.y))
            self.route = [carla.Location(exit.location.x, exit.location.y,
                                         exit.location.z + CYCLE_SIDEWALK_DROP)] + run if run else []
            bike.glide = max(bike.lane.speed, CYCLE_ROLL_SPEED)

    def _arrived(self):
        bike = self.bike
        return bike.state == "parked" and bike.rolled >= dist2d(bike.leave_from, bike.leave_to) - 1e-6

    def _ride(self, now, dt):
        bike = self.bike
        if bike.held > CYCLE_HOLD_MAX:
            bike.state, bike.since = "lingering", now  # held up for good: _step removes it once hidden, or at the cap
            return
        if not 0 <= self.at + self.way < len(self.route):
            self.way = -self.way  # the end of the run in view: back along it
        a, b = self.route[self.at], self.route[self.at + self.way]
        before = bike.rolled
        yaw = math.degrees(math.atan2(b.y - a.y, b.x - a.x))
        if self.flow._glide(bike, dt, a, b, carla.Rotation(yaw=yaw), CYCLE_ROLL_AHEAD):
            self.at += self.way
            bike.rolled = 0.0
        elif bike.rolled == before:
            bike.held += dt

    def _update(self, now):
        bike = self.bike
        if not self.begun:
            self.begun = True
            if bike.lane.exit is None:
                bike.state, bike.since = "stopping", now
            else:
                self.flow._start_leaving(bike, now)  # no stop: on to the far sidewalk at once
        if bike not in self.flow.bikes:
            return  # removed
        dt = self.ctx.time_step() if self.last is None else min(max(now - self.last, 0.0), 0.25)
        self.last = now
        ego = self.ctx.ego_transform()
        if bike.state == "running" and _alive(bike.actor):
            if self.ctx.is_hidden(ego, self.ctx.location_of(bike.actor)):
                self.flow._remove(bike)
            else:
                self._ride(now, dt)
            return
        self.flow._step(bike, now, dt, ego)
        if bike in self.flow.bikes and self._arrived():
            if len(self.route) > 1:
                bike.state, bike.rolled, bike.held = "running", 0.0, 0.0
            else:  # no sidewalk to ride along: never parked in view for long (removed once hidden or at the cap)
                bike.state, bike.since = "lingering", now


class DestroyIfPresent(ActorDestroy):

    """ActorDestroy that skips an actor the run has removed already (it is no longer in CarlaDataProvider's pool)."""

    def update(self):
        if self._actor is not None and self._actor.id not in CarlaDataProvider._carla_actor_pool:
            self._actor = None
        return super().update() if self._actor is not None else py_trees.common.Status.SUCCESS


# --------------------------------------------------------------------------------------------------------------------
# Background traffic that follows the ego


class BackgroundBubble(_Flow):

    """
    Background vehicles and wandering pedestrians that follow the ego: they stay within `radius` of it instead of staying
    at the crossing. Every BUBBLE_PERIOD seconds, a background actor that is farther than 1.3 times `radius` from the
    ego and hidden_from_ego is removed; actors are then spawned to bring each kind back to its configured count, at most
    2 per look: vehicles at map spawn points within `radius` of the ego that are hidden_from_ego and free (no vehicle
    within 8 m), pedestrians at navigation points within `radius` that are hidden_from_ego. Same blueprints as at load,
    bicycles slowed in the Traffic Manager, walkers with an AI controller that is started one tick after the spawn.

    `kinds` is a list of (label, count, models, category) for the vehicles; `tracked` the actors placed at load,
    {label: [actors]}, and `walkers` the (walker, controller) pairs among them.
    """

    def __init__(self, ctx, radius, kinds, walkers_wanted, tracked, walkers, manager=None, name="BackgroundBubble"):
        super().__init__(ctx, name)
        self.radius = radius
        self.kinds = kinds
        self.walkers_wanted = walkers_wanted
        self.tracked = {label: list(tracked.get(label, [])) for label, _, _, _ in kinds}
        self.walkers = [list(pair) for pair in walkers]  # [walker, controller, started]
        for pair in self.walkers:
            pair.append(True)  # started after the scenario's first tick
        self.manager = manager
        self.next_check = 0.0
        self._points = None
        self.spawned = 0
        self.removed = 0

    # --- removal ---

    def _despawn(self, ego):
        limit = BUBBLE_KEEP * self.radius
        for label, actors in self.tracked.items():
            for actor in list(actors):
                if not _alive(actor):
                    actors.remove(actor)
                    self.ctx.remove_actor(actor, group=(self, label))
                    continue
                loc = self.ctx.location_of(actor)
                if dist2d(loc, ego.location) > limit and self.ctx.is_hidden(ego, loc):
                    actors.remove(actor)
                    self.removed += 1
                    self.ctx.remove_actor(actor, group=(self, label))
        for pair in list(self.walkers):
            walker, controller = pair[0], pair[1]
            if not _alive(walker):
                self.walkers.remove(pair)
                self.ctx.remove_actor(walker, controller, group=(self, "walkers"))
                continue
            loc = self.ctx.location_of(walker)
            if dist2d(loc, ego.location) > limit and self.ctx.is_hidden(ego, loc):
                self.walkers.remove(pair)
                self.removed += 1
                self.ctx.remove_actor(walker, controller, group=(self, "walkers"))

    # --- spawning ---

    def _vehicle_locations(self):
        actors = self.ctx.world.get_actors().filter("vehicle.*")
        return [actor.get_location() for actor in actors]

    def _spawn_vehicle(self, label, models, category, ego, occupied):
        ctx = self.ctx
        if self._points is None:
            self._points = list(ctx.map.get_spawn_points())
        candidates = [tf for tf in self._points if dist2d(tf.location, ego.location) <= self.radius
                      and ctx.is_hidden(ego, tf.location)]
        ctx.rng.shuffle(candidates)
        tries = 0
        for tf in candidates:
            if tries >= BUBBLE_VEHICLE_TRIES:
                break
            if any(dist2d(tf.location, other) < BUBBLE_FREE_DISTANCE for other in occupied):
                continue
            tries += 1
            # No autopilot in the request: it is set before the actor is registered, and a failing call would leak it
            vehicle = CarlaDataProvider.request_new_actor(ctx.rng.choice(models), ctx.surface(tf, above=SPAWN_ABOVE),
                                                          actor_category=category, tick=False)
            if vehicle is None:
                continue
            self.tracked[label].append(vehicle)  # tracked first: a failing Traffic Manager call must not lose it
            try:
                vehicle.set_autopilot(True, CarlaDataProvider.get_traffic_manager_port())  # as the request would have
            except RuntimeError:
                self.tracked[label].remove(vehicle)
                ctx.remove_actor(vehicle, group=(self, label))
                raise  # the Traffic Manager is failing: the flow's back-off and throttled log apply, no more tries now
            occupied.append(tf.location)
            self.spawned += 1
            if self.manager is not None:
                self.manager.auto_lane_change(vehicle, False)  # as at load (background.py): no unasked cut-ins
                self.manager.update_vehicle_lights(vehicle, True)
                if category == "bicycle":
                    self.manager.vehicle_percentage_speed_difference(vehicle, BICYCLE_SPEED_DIFFERENCE)
            return True
        return False

    def _spawn_walker(self, ego):
        ctx = self.ctx
        for _ in range(BUBBLE_WALKER_TRIES):
            point = ctx.navigation_point()
            if point is None:
                return False
            if dist2d(point, ego.location) > self.radius or not ctx.is_hidden(ego, point):
                continue
            walker, controller = ctx.spawn_walker(point)
            if walker is None:
                return False
            self.walkers.append([walker, controller, False])
            self.spawned += 1
            return True
        return False

    def _respawn(self, ego):
        """Bring every kind back towards its count: at most MAX_SPAWNS_PER_TICK spawns, the biggest shortfall first."""
        wanted = {label: (count - len(self.tracked[label]) - self.ctx.pending_removals((self, label)), models, category)
                  for label, count, models, category in self.kinds if models}
        wanted = {label: item for label, item in wanted.items() if item[0] > 0}
        deficits = {label: item[0] for label, item in wanted.items()}
        missing = self.walkers_wanted - len(self.walkers) - self.ctx.pending_removals((self, "walkers"))
        if missing > 0:
            deficits["walkers"] = missing
        spawned = 0
        occupied = None
        while spawned < MAX_SPAWNS_PER_TICK and deficits:
            key = max(deficits, key=deficits.get)
            if key == "walkers":
                ok = self._spawn_walker(ego)
            else:
                if occupied is None:
                    occupied = self._vehicle_locations()
                _, models, category = wanted[key]
                ok = self._spawn_vehicle(key, models, category, ego, occupied)
            if ok:
                spawned += 1
                deficits[key] -= 1
                if deficits[key] <= 0:
                    del deficits[key]
            else:
                del deficits[key]  # nothing fits now: try again at the next look
        return spawned

    def _start_pending(self):
        for pair in list(self.walkers):
            if pair[2]:
                continue
            walker, controller = pair[0], pair[1]
            destination = self.ctx.navigation_point()
            if destination is None:
                continue  # tried again next time: started without one it would pick any point (base.navigation_point)
            try:
                controller.start()
                controller.go_to_location(destination)
                controller.set_max_speed(self.ctx.rng.uniform(*WALKER_SPEED))
                pair[2] = True
            except RuntimeError:
                self.walkers.remove(pair)
                self.ctx.remove_actor(walker, controller, group=(self, "walkers"))

    def _update(self, now):
        self._start_pending()
        if now >= self.next_check:
            self.next_check = now + BUBBLE_PERIOD
            ego = self.ctx.ego_transform()
            self._despawn(ego)
            self._respawn(ego)
