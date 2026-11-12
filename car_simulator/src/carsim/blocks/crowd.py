"""<crowd count="10" min_speed="1.1" max_speed="1.8" max_delay="4" flow_radius="60"/>  extra pedestrians, m/s, s, m"""

import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import Idle, WaitForever

from carsim.blocks.base import Block, walker_ids
from carsim.blocks.flow import WalkLine, WalkOn, attach_walker_controller, make_walker_flow, walk_on_points
from carsim.road import crowd_spawn_retries, crowd_spawns, crowd_timing


class Crowd(Block):

    """
    Single pass (autopilot ego): `count` extra pedestrians cross on the same crosswalk as the main one, half from each
    side, in three columns kept at least 0.45 m from the main pedestrian's line. Walkers in a column queue and leave
    one behind the other, at one speed per column, after the shared trigger plus a random delay. A spot that cannot be
    spawned is retried 0.25 m and 0.5 m further back on its own line (same target, same column, a longer walk), then
    skipped: the crowd is scenery and must not abort the scenario. After its walk across, each walker walks on like the
    main pedestrian (see `pedestrian`): an AI walker controller takes it to a navigation point 30 to 60 m beyond the
    far end of the crosswalk and it is removed once hidden; one knocked off its line by a bicycle walks on from where
    it is, instead of marching on into the ego. Without a navigation mesh (Town12) it stands on the far kerb.

    Endless (manual ego): the crowd is part of the pedestrian flow (see carsim.blocks.flow.PedestrianFlow): up to
    1 + `count` pedestrians at a time (the main one included) arrive, cross and leave out of view, walking at a random speed
    between `min_speed` and `max_speed`; `max_delay` is not used. Without a navigation mesh (Town12) it is a single pass.
    """

    tag = "crowd"
    needs_crossing = True

    def __init__(self, ctx, params):
        super().__init__(ctx, params)
        self.count = self.int_param("count", 0)
        self.speed = tuple(sorted((self.float_param("min_speed", 1.1), self.float_param("max_speed", 1.8))))  # m/s
        self.max_delay = self.float_param("max_delay", 4.0)  # s
        self._trigger_distance = self.float_param("trigger_distance", None)
        self.flow_radius = self.float_param("flow_radius", None)
        self.flow = False
        self.owns_flow = False  # this block made the scenario's pedestrian flow (the first of pedestrian and crowd)
        self.walkers = []  # (walker, start delay [s], speed [m/s], walk distance [m], its CrowdSpot, its AI controller or None)
        self.points = None  # the navigation points they walk on to (flow.WalkOnPoints), None without a navigation mesh

    def trigger_distance(self):
        return self._trigger_distance

    def spawn(self):
        ctx = self.ctx
        if ctx.endless and self.count > 0 and ctx.navigation_available():
            ctx.crossing  # fail now, not when the flow is built, when there is no crossing
            self.flow = True
            ctx.flow_active = True
            ctx.walker_flow_slots += self.count
            ctx.walker_flow_speed = self.speed
            if self.flow_radius is not None and ctx.walker_flow_radius is None:
                ctx.walker_flow_radius = self.flow_radius
            return
        if self.count <= 0:
            return
        models = walker_ids(ctx.library)
        self.points = walk_on_points(ctx)
        placed = []
        moved = 0
        for spot in crowd_spawns(ctx.map, ctx.crossing, self.count, ctx.rng):
            model = ctx.rng.choice(models)
            walker = CarlaDataProvider.request_new_actor(model, ctx.surface(spot.spawn), actor_category="pedestrian", tick=False)
            if walker is None:
                # Something (often a prop) is in the way: try a little further back on the same line
                for _, retry in crowd_spawn_retries(spot):
                    walker = CarlaDataProvider.request_new_actor(model, ctx.surface(retry.spawn), actor_category="pedestrian",
                                                                 tick=False)
                    if walker is not None:
                        spot = retry
                        moved += 1
                        break
            if walker is None:
                continue
            ctx.add_actor(walker)
            placed.append((walker, spot))
        # One speed per column and a start order that follows the queue, so nobody runs into the walker ahead
        timing = crowd_timing([spot for _, spot in placed], ctx.rng, self.max_delay, self.speed)
        for (walker, spot), (delay, speed) in zip(placed, timing):
            controller = attach_walker_controller(ctx, walker) if self.points is not None else None
            self.walkers.append((walker, delay, speed, spot.distance, spot, controller))
        self.log("spawned {}/{} crowd pedestrians{}".format(len(self.walkers), self.count,
                                                            " ({} moved back)".format(moved) if moved else ""))

    def after_tick(self):
        if self.flow and self.ctx.claim_walker_flow():
            self.owns_flow = True
            self.ctx.walker_flow = make_walker_flow(self.ctx)
            self.ctx.walker_flow.populate()

    def from_start(self):
        return [self.ctx.walker_flow] if self.owns_flow else []

    def on_trigger(self):
        sequences = []
        for i, (walker, delay, speed, distance, spot, controller) in enumerate(self.walkers):
            sequence = py_trees.composites.Sequence(name="Crowd{}".format(i + 1))
            sequence.add_child(Idle(delay, name="Delay"))
            line = (spot.spawn.location, spot.end) if controller is not None else None
            sequence.add_child(WalkLine(walker, speed, line=line, duration=distance / speed, distance=distance,
                                        name="WalkAcross"))
            if controller is not None:
                sequence.add_child(WalkOn(self.ctx, walker, controller, speed, self.points, self.points.far(spot.end),
                                          name="WalkOn{}".format(i + 1)))
            else:
                sequence.add_child(WaitForever())
            sequences.append(sequence)
        return sequences
