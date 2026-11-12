"""<cyclists count="2" min_speed="3.0" max_speed="5.0" max_delay="3"/>  bicycles, m/s, s"""

import carla
import py_trees

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import ActorTransformSetter, Idle, KeepVelocity

from carsim.blocks.base import Block, vehicle_ids
from carsim.blocks.flow import CyclistFlow, RideOn, _Bike, make_lane, spawn_waiting_bike
from carsim.road import (CYCLE_DECELERATION, CYCLE_STEP, cyclist_exit_spot, cyclist_forced_ride, cyclist_spawns,
                         cyclist_speed_limit, cyclist_timing, cyclist_waiting_spots)

START_LIFT = 0.2  # the bicycle is set down this far above the road surface, as CarlaDataProvider lifts a spawn [m]
MAX_QUEUE = 4  # waiting spots per lane of the endless flow: the sidewalk fits this many bicycles in a row


class Cyclists(Block):

    """
    `count` bicycles cross at the crosswalk. Only bicycles do this (blueprints whose `base_type` is `bicycle`);
    motorcycles never cross at a crosswalk. They ride in a cycle-crossing lane just beside the zebra, on the junction
    side, 2.1 and 3.15 m from the main pedestrian's line (clear of the crowd and of each other, for the real bicycle
    sizes), from just inside the kerb on one side to just before the kerb on the other, on the road. A ride too short
    to stop within is not stretched past the kerb: the speed is capped instead.

    A bicycle waits visibly on the SIDEWALK, beyond the kerb on its cycle line (1 to 3 m, see road.cyclist_waiting_spots)
    facing its direction, with its physics off so it cannot fall or roll. Nothing pops up: it is spawned there at
    load. If the spot cannot be spawned, a spot 0.5 and 1 m further back is tried, else the bicycle is skipped (logged).

    Single pass (autopilot ego): half from each side. Bicycles of one lane queue 2.5 m apart on the road and ride at one
    speed; the shared trigger plus a random delay starts the front one, each one behind it starts a little later. At
    its turn a bicycle is set down at its start on the road (ActorTransformSetter, physics on), ridden across
    (KeepVelocity) and then ridden on (RideOn) without stopping: it rolls kinematically to the far sidewalk (road.
    cyclist_exit_spot) at its speed and keeps riding along the sidewalk, away from the ego (flow.sidewalk_run, turning
    back at the end of the run), so it is never parked in view; it is removed once the ego cannot see it. A lane without
    a far sidewalk spot keeps the bicycle at the far side, removed once hidden or after 5 s at most.

    Endless (manual ego): see carsim.blocks.flow.CyclistFlow. `count` bicycles keep cycling: each waits a random 3 to
    12 s, rolls onto the road, crosses, stops, rolls on to the far sidewalk and is removed once the ego cannot see it
    (where the far side has no sidewalk spot it stays at the far side, removed once hidden or after 5 s at most); a new
    one is then spawned on a free sidewalk spot of either side, but only while that spot is out of the ego's sight. A
    lane has one bicycle on the road at a time, and a bicycle does not launch or ride into the ego, a vehicle or a
    pedestrian on its line.
    """

    tag = "cyclists"
    needs_crossing = True

    def __init__(self, ctx, params):
        super().__init__(ctx, params)
        self.count = self.int_param("count", 2)
        self.speed = tuple(sorted((self.float_param("min_speed", 3.0), self.float_param("max_speed", 5.0))))  # m/s
        self.max_delay = self.float_param("max_delay", 3.0)  # s
        self._trigger_distance = self.float_param("trigger_distance", None)
        self.bikes = []  # (bicycle, start delay [s], speed [m/s], ride length [m], start transform)
        self.waits = {}  # bicycle id -> the transform it waits at on the sidewalk
        self.flow = None  # the CyclistFlow of an endless run
        self.step = CYCLE_STEP  # simulation step [s], read from the world in spawn()
        self.leaver = None  # single pass: a CyclistFlow (not in the tree) that holds the bicycles' _Bike records for RideOn
        self.records = {}  # single pass: bicycle id -> its _Bike

    def trigger_distance(self):
        return self._trigger_distance

    def spawn(self):
        if self.count <= 0:
            return
        ctx = self.ctx
        self.step = ctx.time_step(CYCLE_STEP)
        models = vehicle_ids(ctx.library, base_type="bicycle")
        if not models:
            self.log("WARNING no bicycle blueprints (base_type bicycle): cyclists skipped")
            return
        if ctx.endless:
            self._spawn_flow(models)
        else:
            self._spawn_single_pass(models)

    def _spawn_single_pass(self, models):
        ctx = self.ctx
        placed = []
        for spot in cyclist_spawns(ctx.map, ctx.crossing, self.count):
            start = ctx.surface(carla.Transform(carla.Location(spot.spawn.location.x, spot.spawn.location.y,
                                                               spot.spawn.location.z + START_LIFT), spot.spawn.rotation))
            # Queue position on the sidewalk = place in the queue of the lane: the front bicycle waits at the kerb
            waits = cyclist_waiting_spots(ctx.map, spot, spot.row + 1)
            if len(waits) <= spot.row:
                self.log("no sidewalk spot for the bicycle of lane {} row {}: skipped".format(spot.side, spot.row))
                continue
            bike, wait = spawn_waiting_bike(ctx, ctx.map, ctx.rng.choice(models), waits[spot.row])
            if bike is None:
                self.log("no bicycle could be spawned on the sidewalk spot of lane {} row {}: skipped".format(
                    spot.side, spot.row))
                continue
            ctx.add_actor(bike)
            self.waits[bike.id] = wait
            placed.append((bike, spot, start))
        limits = [cyclist_speed_limit(spot.distance, self.step) for _, spot, _ in placed]
        timing = cyclist_timing([spot for _, spot, _ in placed], ctx.rng, self.max_delay, self.speed,
                                speed_limits=limits)
        self.leaver = CyclistFlow(ctx, [], 0, [])
        lanes = {}  # one CycleLane per side, so a bicycle ignores the other lane and not its own
        for (bike, spot, start), (delay, speed) in zip(placed, timing):
            self.bikes.append((bike, delay, speed, spot.distance, start))
            if spot.side not in lanes:
                exit_spot = cyclist_exit_spot(ctx.map, spot)
                if exit_spot is None:
                    self.log("no sidewalk spot beyond the far kerb of lane {}: its bicycles stay at the far side "
                             "until hidden (5 s at most)".format(spot.side))
                lanes[spot.side] = make_lane(spot, [], speed, self.step, exit=exit_spot)
            self.records[bike.id] = _Bike(bike, lanes[spot.side], spot.row)
            self.leaver.bikes.append(self.records[bike.id])
        capped = sum(1 for limit in limits if limit < self.speed[1])
        self.log("spawned {}/{} cyclists, waiting on the sidewalk{}".format(
            len(self.bikes), self.count,
            " ({} on a ride so short that the speed is capped to stop before the far kerb)".format(capped)
            if capped else ""))

    def _spawn_flow(self, models):
        ctx = self.ctx
        spots = cyclist_spawns(ctx.map, ctx.crossing, 2)  # one per direction: the lane's full ride
        limits = [cyclist_speed_limit(spot.distance, self.step) for spot in spots]
        timing = cyclist_timing(spots, ctx.rng, 0.0, self.speed, speed_limits=limits)  # the lane speeds
        lanes = []
        for spot, (_, speed) in zip(spots, timing):
            waits = cyclist_waiting_spots(ctx.map, spot, min(self.count, MAX_QUEUE))
            if waits:
                exit_spot = cyclist_exit_spot(ctx.map, spot)
                if exit_spot is None:
                    self.log("no sidewalk spot beyond the far kerb of lane {}: its bicycles stay at the far side "
                             "until hidden (5 s at most)".format(spot.side))
                lanes.append(make_lane(spot, waits, speed, self.step, exit=exit_spot))
            else:
                self.log("no sidewalk spot for lane {}: no bicycles on that side".format(spot.side))
        if not lanes:
            self.log("WARNING no cycle lane with a sidewalk spot: cyclists skipped")
            return
        self.flow = CyclistFlow(ctx, lanes, self.count, models)
        ctx.flow_active = True
        # The first bicycles are placed at load, before the run: one lane after the other, nearest the kerb first
        index = 0
        while len(self.flow.bikes) < self.count:
            free = self.flow.free_spots(0.0)
            if not free:
                break
            lane, position = free[index % len(free)]
            index += 1
            self.flow.place(lane, position, 0.0)
        for record in self.flow.bikes:
            ctx.add_actor(record.actor)
        self.log("spawned {}/{} cyclists on the sidewalk, they keep cycling for as long as the run lasts".format(
            len(self.flow.bikes), self.count))

    def from_start(self):
        return [self.flow] if self.flow is not None else []

    def on_trigger(self):
        sequences = []
        for i, (bike, delay, speed, distance, start) in enumerate(self.bikes):
            # The bicycle is pushed at a forced speed, so stop pushing early enough for the brake to stop it
            ride = max(cyclist_forced_ride(distance, speed, self.step, CYCLE_DECELERATION), 0.1)
            sequence = py_trees.composites.Sequence(name="Cyclist{}".format(i + 1))
            sequence.add_child(Idle(delay, name="Delay"))
            # From the sidewalk onto the road at its start with physics on; succeeds once the actor is within 1 m of it
            sequence.add_child(ActorTransformSetter(bike, start, physics=True, name="Appear"))
            sequence.add_child(KeepVelocity(bike, speed, force_speed=True, duration=ride / speed + 1.0,
                                            distance=ride, name="RideAcross"))
            sequence.add_child(RideOn(self.leaver, self.records[bike.id], name="RideOn{}".format(i + 1)))
            sequences.append(sequence)
        return sequences
