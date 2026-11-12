"""<pedestrian model="walker.pedestrian.0001" speed="1.4" trigger_distance="25" walk_in="0" flow_radius="60"/>  m/s, m, m, m"""

import math

import py_trees

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaitForever
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTimeToArrivalToLocation

from carsim.blocks.base import Block
from carsim.blocks.flow import (DestroyIfPresent, WalkLine, WalkOn, attach_walker_controller, make_walker_flow,
                                walk_on_points)
from carsim.road import WALKER_ACCELERATION, impact_point, walk_in_spawns, walker_spawns


EGO_HALF_EXTENT = (0.9, 2.4)  # half width, half length of the ego when it has no bounding box (the offline fakes) [m]


class WalkAlong(WalkLine):

    """WalkLine along a given direction instead of the walker's current heading: the turn between two legs of a walk."""

    def __init__(self, walker, speed, direction, **kwargs):
        super().__init__(walker, speed, **kwargs)
        self._direction = direction

    def initialise(self):
        super().initialise()
        self._control.direction = self._direction
        self._actor.apply_control(self._control)


class Pedestrian(Block):

    """
    The main adversary. Single pass (autopilot ego): a pedestrian waits just beyond the right end of the crosswalk
    the ego lane crosses and walks along it, right to left, once the ego is within `trigger_distance` of the crossing
    point. This block also defines the shared trigger distance. It must spawn (the scenario stops otherwise); the
    start can sit inside a kerb or pole, so a few spots around it are tried, each walking to the same end point.

    Endless (manual ego): no single pedestrian. Together with `crowd` (see carsim.blocks.flow.PedestrianFlow) the
    pedestrians arrive at the crosswalk from out of view, cross it, walk on and are removed out of view, 1 + the crowd's
    `count` of them at a time, spawned within `flow_radius` of the crossing (default 60 m). A map without a navigation
    mesh (Town12) cannot do that and falls back to the single pass.

    After its walk across, the single pass pedestrian walks on: an AI walker controller (started then) takes it to a
    navigation point 30 to 60 m beyond the far end of the crosswalk, out of the ego's sight when there is such a point
    (carsim.blocks.flow.WalkOn), and it is removed once hidden. One knocked off its line on the way (by a bicycle)
    walks on from where it is. A map without a navigation mesh keeps the old behaviour: it stands on the far kerb.

    `age` ("adult" or "child") instead of `model`: the single pedestrian is a random walker blueprint of that age
    (the flow picks its own).

    `walk_in` (default 0): the single pedestrian starts that many metres back along the road, toward the ego, off the
    driving lanes (a truck at the kerb can hide it there), walks along the road to where it would have started, then
    turns and crosses as before.

    `overlap` (percent, 0 to 100, single pass): instead of a trigger distance, the walk starts when the ego's time to
    arrival at the impact point equals the pedestrian's walk time to it, so without braking the ego hits it at that
    share of its width from the pedestrian's side (50: the centre line; NCAP CPNCO, see carsim.road.impact_point),
    whatever its speed. The ego's width and length are those of its bounding box. The crowd and the cyclists start
    with it. Not together with `trigger_distance`.
    """

    tag = "pedestrian"
    needs_crossing = True

    def __init__(self, ctx, params):
        super().__init__(ctx, params)
        self.model = self.params.get("model", "walker.pedestrian.0001")
        self.age = self.params.get("age")  # None: `model`
        self.speed = self.float_param("speed", 1.4)  # m/s
        self._trigger_distance = self.float_param("trigger_distance", None)  # None: not set here
        self.flow_radius = self.float_param("flow_radius", None)
        self.flow = False  # endless: the pedestrian flow replaces the single pass
        self.owns_flow = False  # this block made the scenario's pedestrian flow (the first of pedestrian and crowd)
        self.walk_in = self.float_param("walk_in", 0.0)  # m
        self.overlap = self.float_param("overlap", None)  # percent of the ego width; None: the distance trigger
        self.walker = None
        self.walk_distance = None
        self.cross = None  # the transform the walk across starts at (walk_in only)
        self.controller = None  # its AI walker controller (not started until the walk on), None: it stays at the far end
        self.points = None  # the navigation points it walks on to (flow.WalkOnPoints)

    def trigger_distance(self):
        return self._trigger_distance

    def spawn(self):
        ctx = self.ctx
        if ctx.endless:
            if ctx.navigation_available():
                ctx.crossing  # fail now, not when the flow is built, when there is no crossing
                self.flow = True
                ctx.flow_active = True
                ctx.walker_flow_slots += 1
                if self.flow_radius is not None and ctx.walker_flow_radius is None:
                    ctx.walker_flow_radius = self.flow_radius
                return
            self.log("no navigation mesh for this map: no endless pedestrian flow, a single pass instead")
        crossing = ctx.crossing
        if self.age is not None:
            self.model = self._model_of_age()
        spawns = (walk_in_spawns(ctx.map, crossing, self.walk_in) if self.walk_in > 0
                  else ((offset, tf, tf, distance) for offset, tf, distance in walker_spawns(ctx.map, crossing)))
        for offset, spawn_transform, cross_transform, walk_distance in spawns:
            walker = CarlaDataProvider.request_new_actor(self.model, ctx.surface(spawn_transform), actor_category="pedestrian")
            if walker is not None:
                if offset != (0.0, 0.0):
                    self.log("pedestrian spawned {:g} m back / {:g} m along the road from the computed start".format(
                        *offset))
                self.walker = walker
                self.walk_distance = walk_distance
                self.cross = cross_transform
                break
        if self.walker is None:
            raise ValueError("Failed to spawn the pedestrian at {} or nearby".format(crossing.spawn.location))
        ctx.add_actor(self.walker)
        ctx.target = self.walker
        self.points = walk_on_points(ctx)
        if self.points is not None:
            self.controller = attach_walker_controller(ctx, self.walker)

    def _model_of_age(self):
        ids = [bp.id for bp in self.ctx.library.filter("walker.pedestrian.*")
               if bp.has_attribute("age") and bp.get_attribute("age").as_str() == self.age]
        if not ids:
            raise ValueError("No walker blueprint with age '{}'".format(self.age))
        return self.ctx.rng.choice(ids)

    def start_condition(self, ego):
        """
        The condition that starts the walk, when `overlap` is set (single pass): the ego's time to arrival at the point
        where its centre is when its front reaches the impact point is less than the walk time. None: the shared
        trigger distance.
        """
        if self.overlap is None or self.flow:
            return None
        extent = getattr(getattr(ego, "bounding_box", None), "extent", None)
        half_width, half_length = (extent.y, extent.x) if extent is not None else EGO_HALF_EXTENT
        impact, aim = impact_point(self.ctx.crossing, half_width, self.overlap, half_length, self.cross.location)
        walk_time = (self.walk_in + math.hypot(impact.x - self.cross.location.x,
                                               impact.y - self.cross.location.y)) / self.speed
        walk_time += self.speed / (2.0 * WALKER_ACCELERATION)  # it starts from rest
        self.log("starts when the ego is {:.2f} s from the {:g} % impact point (the walk takes that long)".format(
            walk_time, self.overlap))
        return InTimeToArrivalToLocation(ego, walk_time, aim, name="WaitForEgo")

    def after_tick(self):
        """The flow is made here, when every block has said how many pedestrians it wants, and places its first ones."""
        if self.flow and self.ctx.claim_walker_flow():
            self.owns_flow = True
            self.ctx.walker_flow = make_walker_flow(self.ctx)
            self.ctx.walker_flow.populate()

    def from_start(self):
        return [self.ctx.walker_flow] if self.owns_flow else []

    def on_trigger(self):
        if self.flow:
            return []
        adversary = py_trees.composites.Sequence(name="Adversary")
        walk = dict(duration=self.walk_distance / self.speed, distance=self.walk_distance, name="WalkAcross")
        walking_on = self.controller is not None
        if walking_on:  # ends early when it is knocked off its line, to walk on from there
            walk["line"] = (self.cross.location, self.ctx.crossing.end)
        if self.walk_in > 0:
            adversary.add_child(WalkLine(self.walker, self.speed, duration=self.walk_in / self.speed,
                                         distance=self.walk_in, name="WalkIn"))
            adversary.add_child(WalkAlong(self.walker, self.speed, self.cross.get_forward_vector(), **walk))
        else:
            adversary.add_child(WalkLine(self.walker, self.speed, **walk))
        if walking_on:
            adversary.add_child(WalkOn(self.ctx, self.walker, self.controller, self.speed, self.points,
                                       self.points.far(self.ctx.crossing.end)))
        else:
            adversary.add_child(WaitForever())
        return [adversary]

    def cleanup_behaviors(self):
        if self.walker is None:
            return []
        return [DestroyIfPresent(self.walker, name="RemovePedestrian")]
