#!/usr/bin/env python
"""
Offline check of the endless flows of a manual run (src/carsim/blocks/flow.py), no CARLA server needed.

ComposedScenario is built for pedestrian_crossing.xml with a manual ego (the real ScenarioRunner parser, BasicScenario,
py_trees, an offline Town10HD map) against a fake world that lives: walkers walk to what their AI controller was told,
bicycles move with the velocity they are pushed with and brake, and a fake ego drives along its lane at 10 m/s, waits
25 s at the crossing and drives on. The behaviour tree is ticked at 20 Hz for 90 s of game time (a few thousand ticks;
CarlaDataProvider and GameTime are patched, nothing touches a server). Checked:

  - the visibility rule: every spawn and every removal during the run is hidden_from_ego (120 m away, or 50 m away and
    off to the side, or 15 m away and behind the car unless the ego reverses) at the moment it happens, with the ego
    where it was then; the only exception is a bicycle of a lane without a far sidewalk spot, which goes after 5 s at
    most (safety cap), seen or not. Load-time spawns are exempt.
  - the counts are held: never more than 1 + crowd count flow pedestrians, `count` cyclists, the configured number of
    background cars, motorcycles, bicycles and walkers, at any tick; and they are reached
  - the flows work: pedestrians spawn within 60 m and at least 15 m from the crossing, are started a tick after the
    spawn, cross, are removed once out of sight (also the ones that get stuck), cyclists wait on the sidewalk, ride,
    stop and are replaced, the background bubble removes actors left behind and spawns near the ego on free spawn
    points, bicycles of it are slowed in the Traffic Manager
  - cyclists: one bicycle per lane on the road at a time, the front of the queue leaves first, a waiting bicycle never
    has its physics on, a riding one stays on its line; after its ride a bicycle rolls (physics off) onto the far
    sidewalk, frees its lane at once and is removed only when hidden; no launch and no push into an actor on the line;
    a retried waiting spot is where the bicycle starts from (and is only used when it is hidden, too); without a far
    spot the 5 s cap applies; a bicycle leaving or parked on the way to the far sidewalk holds the next one of its lane,
    and a leave that is held up for CYCLE_HOLD_MAX s ends where it is
  - the initial population: a few pedestrians are placed at load near the crossing, already walking at the first tick
  - robustness: a CARLA error in a flow's update does not end the run (logged once per 10 s, 1 s back-off); the
    controller-failure paths destroy what they drop; an actor the server will not destroy is retried and counts against
    its flow's capacity (no replacement over the count); a background vehicle whose autopilot cannot be set is removed; a pool that cannot fill is not sampled every tick; a finished
    pedestrian in view keeps being sent on; the pedestrian and cyclist flows pause when the ego is far
  - the stop file brakes the ego, and a dead ego ends the run (also in a manual run without a flow, before its start
    trigger has fired); reversing switches the behind rule off, and the 50 m rule then only hides points to the side
  - ComposedScenario.terminate() creates the marker file next to the stop file, and the driving window's watcher
    (apps/run_scenario.py) does not take a window that exits after it for the user closing it
  - the stop file ends the run: the tree is RUNNING without the file and SUCCESS the tick after it exists
  - no navigation mesh (Town12): the pedestrian flow falls back to the single pass
  - the single pass walks and rides on (WalkOn, RideOn, WalkLine, run here against the same fake world): a walker
    that has crossed gets a controller target 30 to 60 m beyond the far end of the crosswalk (one the ego cannot see
    when there is one), is removed only once hidden, and the final cleanup does not destroy it twice; with no point it
    stays; a walker knocked off its line ends its walk across; a bicycle that has ridden across does not stop or park: it
    rolls to the far sidewalk, keeps riding along it (physics off, off the driving lanes, never standing still for a
    second, turning back at the end of its run) and is removed only once hidden; a lane without a far sidewalk spot
    stops and lingers as before
  - cleanup: nothing is left alive that ScenarioRunner's pool does not own, every removed actor is out of the pool, the
    controllers are destroyed, the scenario's actor list does not grow
  - the 7 day timeout: TimeOut stays RUNNING for days of game time, and ScenarioManager.analyze_scenario reports SUCCESS

    python tests/check_flows_offline.py

Exits with 1 if any check fails.
"""

import importlib.util
import io
import math
import random
import sys
import tempfile
import threading
from contextlib import ExitStack, redirect_stdout
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "tests"))

import carla  # noqa: E402
import py_trees  # noqa: E402

import check_blocks_offline as cb  # noqa: E402
from check_blocks_offline import FakeActor, FakeClient, FakeLibrary, FakeWorld, MapProxy, check  # noqa: E402
from carsim.blocks import base as cb_base  # noqa: E402
from carsim.blocks import flow as flow_module  # noqa: E402
from carsim.blocks import spec  # noqa: E402
from carsim.blocks.spec import ending_marker  # noqa: E402
from carsim.road import (cyclist_exit_spot, cyclist_spawns, cyclist_wait_retries, find_sidewalk,  # noqa: E402
                         hidden_from_ego, straight_route)
from carsim.scenario_files import COMPOSED_SCENARIO_FILE, config_file  # noqa: E402
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider  # noqa: E402
from srunner.scenariomanager.scenario_manager import ScenarioManager  # noqa: E402
from srunner.scenariomanager.timer import GameTime  # noqa: E402

DT = 0.05
EGO_SPEED = 10.0
HOLD = 25.0  # the ego waits this long at the crossing [s]
DURATION = 90.0
VARIANT = "PedestrianCrossing_Town10HD_1"
PARAMS = {  # the blocks of the manual run
    "ego_control": {"mode": "manual"},
    "pedestrian": {},
    "crowd": {"count": "6"},
    "cyclists": {"count": "3"},
    "background": {"vehicles": "6", "motorcycles": "2", "bicycles": "2", "walkers": "8", "radius": "120"},
}


class Event:
    def __init__(self, kind, what, location, ego, time, ego_speed=None, **extra):
        self.kind, self.what, self.location, self.ego, self.time = kind, what, location, ego, time
        self.ego_speed = ego_speed
        self.hidden = hidden_from_ego(ego, location, forward_speed=ego_speed)
        self.__dict__.update(extra)


class LiveActor(FakeActor):
    """A fake actor that moves like the real ones would: walkers to their destination, bicycles with their velocity."""

    def __init__(self, sim, type_id, transform):
        super().__init__(type_id, transform)
        self.sim = sim
        self.velocity = (0.0, 0.0)
        self.destination = None
        self.max_speed = 1.4
        self.walking = False
        self.braking = False
        self.blocked_after = None  # walkers that stop for good after this much walking [s]
        self.walked = 0.0
        self.autopilot = False
        self.autopilot_failures = 0  # the next this many set_autopilot calls raise RuntimeError
        self.destroy_failures = 0  # the next this many destroy calls raise RuntimeError
        self.last_control = None

    def set_autopilot(self, enabled=True, port=8000):
        if self.autopilot_failures > 0:
            self.autopilot_failures -= 1
            raise RuntimeError("injected: no autopilot")
        self.autopilot = bool(enabled)
        self.sim.on_autopilot(self)

    def set_target_velocity(self, velocity):
        self.velocity = (velocity.x, velocity.y)

    def get_velocity(self):
        return carla.Vector3D(self.velocity[0], self.velocity[1], 0.0)

    def apply_control(self, control):
        if isinstance(control, carla.VehicleControl):
            self.braking = control.brake > 0.0
            self.last_control = control

    @property
    def speed(self):
        return math.hypot(*self.velocity)

    def destroy(self):
        if self.destroy_failures > 0:
            self.destroy_failures -= 1
            raise RuntimeError("injected: cannot destroy")
        self.sim.on_destroy(self)
        return super().destroy()

    def step(self, dt):
        loc = self.transform.location
        if self.type_id.startswith("walker"):
            if self.walking and self.destination is not None:
                if self.blocked_after is not None and self.walked >= self.blocked_after:
                    return
                dx, dy = self.destination.x - loc.x, self.destination.y - loc.y
                d = math.hypot(dx, dy)
                move = min(d, self.max_speed * dt)
                if d > 1e-6:
                    self.transform = carla.Transform(carla.Location(loc.x + dx / d * move, loc.y + dy / d * move, loc.z),
                                                     self.transform.rotation)
                self.walked += dt
        elif self.type_id.startswith("vehicle") and self.physics and not self.autopilot:
            if self.braking:
                v = max(self.speed - 4.0 * dt, 0.0)
                f = v / self.speed if self.speed > 1e-9 else 0.0
                self.velocity = (self.velocity[0] * f, self.velocity[1] * f)
            self.transform = carla.Transform(carla.Location(loc.x + self.velocity[0] * dt, loc.y + self.velocity[1] * dt,
                                                            loc.z), self.transform.rotation)


class LiveController(FakeActor):
    def __init__(self, sim, blueprint, transform, walker):
        super().__init__(blueprint.id, transform)
        self.sim, self.walker = sim, walker
        self.started_tick = None

    def start(self):
        self.started_tick = self.sim.tick
        self.walker.walking = True

    def go_to_location(self, location):
        self.walker.destination = location

    def set_max_speed(self, speed):
        self.walker.max_speed = speed


class LiveWorld(FakeWorld):
    nav_cache = {}

    def __init__(self, sim, cmap, around, step):
        super().__init__(cmap, around, step)
        self.sim = sim
        if id(cmap) not in LiveWorld.nav_cache:
            points = []
            for wp in cmap.generate_waypoints(5.0):
                sidewalk = find_sidewalk(wp, "right")
                if sidewalk is not None:
                    points.append(sidewalk.transform.location)
            LiveWorld.nav_cache[id(cmap)] = points
        self.nav_points = LiveWorld.nav_cache[id(cmap)]
        self.nav_random = random.Random(11)
        self.fail_controllers = 0  # the next this many controller spawns raise RuntimeError

    def get_random_location_from_navigation(self):
        return self.nav_random.choice(self.nav_points) if self.nav_points else None

    def spawn_actor(self, blueprint, transform, attach_to=None):
        if self.fail_controllers > 0:
            self.fail_controllers -= 1
            raise RuntimeError("injected: no controller")
        controller = LiveController(self.sim, blueprint, transform, attach_to)
        self.controllers.append(controller)
        return controller

    def get_actors(self):
        outer = self

        class Actors(list):
            def filter(self, pattern):
                prefix = pattern.split("*")[0]
                return [a for a in self if a.type_id.startswith(prefix)]
        return Actors(a for a in self.sim.actors if a.is_alive)


class Sim:
    """The scenario, the fake world around it, and the log of everything that was spawned and removed."""

    def __init__(self, module, cmap, params, step=DT, nav=True, stop_file=None, hold=HOLD, fail_controllers=0):
        self.cmap, self.dt, self.hold = cmap, step, hold
        self.tick = 0
        self.t = 0.0
        self.phase = "load"
        self.events = []
        self.actors = []
        self.stack = ExitStack()
        CarlaDataProvider.get_random_seed().seed(2000)
        xml = config_file("pedestrian_crossing")
        from srunner.tools.scenario_parser import ScenarioConfigurationParser
        config = ScenarioConfigurationParser.parse_scenario_configuration(VARIANT, str(xml))[0]
        config.other_parameters.clear()
        config.other_parameters.update({tag: dict(values) for tag, values in params.items()})
        if stop_file is not None:
            config.other_parameters["run_control"] = {"stop_file": str(stop_file)}
        around = config.trigger_points[0].location
        self.world = LiveWorld(self, cmap, around, step)
        if not nav:
            self.world.nav_points = []
        self.world.fail_controllers = fail_controllers
        self.ego = LiveActor(self, "vehicle.lincoln.mkz_2017", config.ego_vehicles[0].transform)
        self.ego.autopilot = True  # the simulation moves the ego itself (move_ego) and gives it its velocity
        proxy = MapProxy(cmap, around, radius=1e9)
        proxy.spawn_points = [wp.transform for wp in cmap.generate_waypoints(12.0)]
        patches = [
            mock.patch.object(CarlaDataProvider, "get_map", staticmethod(lambda: proxy)),
            mock.patch.object(CarlaDataProvider, "get_world", staticmethod(lambda: self.world)),
            mock.patch.object(CarlaDataProvider, "get_client", staticmethod(lambda: self.world.client)),
            mock.patch.object(CarlaDataProvider, "request_new_actor", staticmethod(self.request_new_actor)),
            mock.patch.object(CarlaDataProvider, "is_sync_mode", staticmethod(lambda: True)),
            mock.patch.object(CarlaDataProvider, "is_runtime_init_mode", staticmethod(lambda: False)),
            mock.patch.object(CarlaDataProvider, "get_location", staticmethod(lambda a: a.transform.location)),
            mock.patch.object(CarlaDataProvider, "get_transform", staticmethod(lambda a: a.transform)),
            mock.patch.object(CarlaDataProvider, "get_velocity", staticmethod(lambda a: getattr(a, "speed", 0.0))),
            mock.patch.object(GameTime, "get_time", staticmethod(lambda: self.t)),
        ]
        for patch in patches:
            self.stack.enter_context(patch)
        CarlaDataProvider._carla_actor_pool = {}
        self.scenario = module.ComposedScenario(self.world, [self.ego], config, debug_mode=False)
        self.ctx = self.scenario._ctx
        self.by_tag = {block.tag: block for block in self.scenario._blocks}
        route = [self.ctx.ego_start_wp] + straight_route(self.ctx.ego_start_wp, 700.0, 2.0)
        self.route = route
        self.s = 0.0
        self.hold_until = None
        self.crossing_s = self.ctx.crossing.distance
        self.pool_removed = set()
        self.phase = "run"
        self.max_counts = {}

    # --- the fake CarlaDataProvider.request_new_actor ---

    def request_new_actor(self, model, spawn_point=None, rolename="scenario", autopilot=False, random_location=False,
                          color=None, actor_category="car", attribute_filter=None, tick=True):
        assert tick is False or self.phase == "load", f"{actor_category} spawned with a tick of its own during the run"
        actor = LiveActor(self, model, spawn_point)
        actor.autopilot = autopilot
        actor.category = actor_category
        self.actors.append(actor)
        CarlaDataProvider._carla_actor_pool[actor.id] = actor
        self.events.append(Event("spawn", actor, spawn_point.location, self.ego.transform, self.t,
                                 ego_speed=self.ego_forward_speed(), phase=self.phase,
                                 category=actor_category, autopilot=autopilot, tick_no=self.tick))
        if model.startswith("walker") and actor.id % 5 == 0:
            actor.blocked_after = 4.0  # some pedestrians get stuck on the way
        return actor

    def on_autopilot(self, actor):
        """The autopilot of a vehicle was set after the request (the events say what it ended up with)."""
        for e in self.events:
            if e.kind == "spawn" and e.what is actor:
                e.autopilot = actor.autopilot

    def on_destroy(self, actor):
        if actor.is_alive and not actor.type_id.startswith("controller"):
            self.events.append(Event("remove", actor, actor.transform.location, self.ego.transform, self.t,
                                     ego_speed=self.ego_forward_speed(), phase=self.phase,
                                     category=getattr(actor, "category", None), tick_no=self.tick))

    def ego_forward_speed(self):
        yaw = math.radians(self.ego.transform.rotation.yaw)
        return self.ego.velocity[0] * math.cos(yaw) + self.ego.velocity[1] * math.sin(yaw)

    # --- time ---

    def move_ego(self):
        before = self.ego.transform.location
        self._move_ego()
        after = self.ego.transform.location
        self.ego.velocity = ((after.x - before.x) / self.dt, (after.y - before.y) / self.dt)

    def _move_ego(self):
        if self.hold_until is not None and self.t < self.hold_until:
            return
        self.s += EGO_SPEED * self.dt
        if self.hold_until is None and self.s >= self.crossing_s:
            self.hold_until = self.t + self.hold
            self.s = self.crossing_s
        i = min(int(self.s // 2.0), len(self.route) - 2)
        f = min((self.s - i * 2.0) / 2.0, 1.0)
        a, b = self.route[i].transform, self.route[i + 1].transform
        self.ego.transform = carla.Transform(
            carla.Location(a.location.x + (b.location.x - a.location.x) * f,
                           a.location.y + (b.location.y - a.location.y) * f, a.location.z),
            carla.Rotation(yaw=a.rotation.yaw))

    def step(self):
        self.tick += 1
        self.t += self.dt
        self.move_ego()
        for actor in list(self.actors):
            if actor.is_alive:
                actor.step(self.dt)
        self.scenario.behavior_tree.tick_once()

    def run(self, seconds, observe=None):
        for _ in range(int(seconds / self.dt)):
            self.step()
            if observe is not None:
                observe(self)

    def alive(self, prefix):
        return [a for a in self.actors if a.is_alive and a.type_id.startswith(prefix)]

    def close(self):
        self.stack.close()
        CarlaDataProvider._carla_actor_pool = {}


def blocks_flows(sim):
    ped = next(n for n in sim.scenario.behavior_tree.iterate() if isinstance(n, flow_module.PedestrianFlow))
    bikes = next(n for n in sim.scenario.behavior_tree.iterate() if isinstance(n, flow_module.CyclistFlow))
    bubble = next(n for n in sim.scenario.behavior_tree.iterate() if isinstance(n, flow_module.BackgroundBubble))
    return ped, bikes, bubble


def check_visibility(sim, bike_removals):
    runtime = [e for e in sim.events if e.phase == "run" and e.time > 0.0]
    spawns = [e for e in runtime if e.kind == "spawn"]
    removes = [e for e in runtime if e.kind == "remove"]
    check(flow_module.CYCLE_LINGER_CAP <= 5.0, "the safety cap of a stopped bicycle is above 5 s")
    check(flow_module.FLOW_PAUSE_FACTOR == 2.0 and flow_module.FLOW_MAX_REPICKS > 3, "flow constants changed")
    check(len(spawns) > 30, f"only {len(spawns)} spawns during the run: the flows did not run")
    for e in spawns:
        check(e.hidden, f"{e.category} spawned in view at t={e.time:.1f} s: ego at "
                        f"({e.ego.location.x:.0f}, {e.ego.location.y:.0f}) yaw {e.ego.rotation.yaw:.0f}, "
                        f"spot ({e.location.x:.0f}, {e.location.y:.0f})")
    bike_ids = {bike.actor.id: (lingered, hidden, exit_spot) for bike, lingered, hidden, exit_spot in bike_removals}
    for e in removes:
        if e.what.id in bike_ids:
            lingered, hidden, exit_spot = bike_ids[e.what.id]
            if exit_spot:  # it left to the far sidewalk: only the hide rule may remove it
                check(hidden, f"a bicycle that left to the far sidewalk vanished in view after {lingered:.1f} s")
            else:
                check(hidden or lingered >= 5.0 - sim.dt - 1e-9,
                      f"a bicycle vanished in view after {lingered:.1f} s at the far side (cap 5 s)")
            continue
        check(e.hidden, f"{e.category} {e.what.type_id} removed in view at t={e.time:.1f} s: ego at "
                        f"({e.ego.location.x:.0f}, {e.ego.location.y:.0f}), actor at "
                        f"({e.location.x:.0f}, {e.location.y:.0f})")
    # Nothing but the rule decides: the spawns of every kind happened, and so did removals of every flow kind
    kinds = {e.category for e in spawns}
    for kind in ("pedestrian", "bicycle", "car"):
        check(kind in kinds, f"no {kind} was spawned during the run")
    check(any(e.category == "pedestrian" for e in removes), "no pedestrian was removed during the run")
    return spawns, removes


def check_manual_flows(module, cmap):
    print("flows: a manual run of {:.0f} s with a moving ego".format(DURATION))
    stop_file = Path(tempfile.gettempdir()) / "carsim_check_flows_stop_file"
    stop_file.unlink(missing_ok=True)
    sim = Sim(module, cmap, PARAMS, stop_file=stop_file)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        ctx = sim.ctx
        bike_removals = []
        original_remove = flow_module.CyclistFlow._remove

        def watch_remove(self, bike):
            bike_removals.append((bike, sim.t - bike.since, ctx.is_hidden(sim.ego.transform, ctx.location_of(bike.actor)),
                                  bike.lane.exit is not None))
            return original_remove(self, bike)
        walker_checks = []
        seen_spawns = [ped.spawned]  # the pedestrians placed at load are checked in check_initial_population
        counts = {"ped": 0, "bikes": 0, "cars": 0, "motorcycles": 0, "bicycles": 0, "walkers": 0}
        waiting_physics, lane_actives, off_line = [], [], []
        exit_problems, parked_gaps, exit_states, growth = [], [], set(), []
        initial_walkers = len(ped.walkers)
        check(initial_walkers == ped.initial >= 1, f"{ped.initial} pedestrians were placed at load")

        def observe(sim):
            # Counts never above the configured ones
            n_ped = 1 + 6
            check(len(ped.walkers) <= n_ped, f"{len(ped.walkers)} flow pedestrians, at most {n_ped}")
            check(len(bikes.bikes) <= 3, f"{len(bikes.bikes)} flow bicycles, at most 3")
            for label, want in (("cars", 6), ("motorcycles", 2), ("bicycles", 2)):
                check(len(bubble.tracked[label]) <= want, f"{len(bubble.tracked[label])} background {label}, at most {want}")
            check(len(bubble.walkers) <= 8, f"{len(bubble.walkers)} background walkers, at most 8")
            counts["ped"] = max(counts["ped"], len(ped.walkers))
            counts["bikes"] = max(counts["bikes"], len(bikes.bikes))
            for label in ("cars", "motorcycles", "bicycles"):
                counts[label] = max(counts[label], len(bubble.tracked[label]))
            counts["walkers"] = max(counts["walkers"], len(bubble.walkers))
            # A pedestrian that was just spawned: within 60 m and 15 m of the crossing, started a tick later
            if ped.spawned > seen_spawns[0]:
                seen_spawns[0] = ped.spawned
                walker = ped.walkers[-1]
                d = flow_module.dist2d(walker.actor.transform.location, ped.center)
                walker_checks.append((d, walker, sim.tick))
            for walker, d, tick in [(w, d, t) for d, w, t in walker_checks]:
                started = walker.controller.started_tick
                if started is not None:
                    check(started > tick, "a pedestrian's controller was started in the tick of its spawn")
            # Cyclists: waiting ones have no physics; one per lane on the road; riders stay on their line
            # Nothing grows: live actors and tracked controllers stay within the configured counts
            live_walkers = len(sim.alive("walker"))
            if len(sim.alive("vehicle")) > 3 + 6 + 2 + 2 or live_walkers > 7 + 8 or len(sim.ctx._controllers) > live_walkers:
                growth.append((sim.t, len(sim.alive("vehicle")), live_walkers, len(sim.ctx._controllers)))
            for bike in bikes.bikes:
                lane = bike.lane
                if bike.state in ("leaving", "parked"):
                    exit_states.add(bike.state)
                    if bike.actor.physics:
                        exit_problems.append("a bicycle on its way to the far sidewalk has its physics on")
                    if lane.active is bike:
                        exit_problems.append("a bicycle that left the road still holds its lane")
                    if bike.state == "parked" and bike.actor.id not in parked:
                        parked.add(bike.actor.id)
                        target = lane.exit.location
                        parked_gaps.append(flow_module.dist2d(bike.actor.transform.location, target))
                if bike.state == "waiting" and bike.actor.physics:
                    waiting_physics.append(bike.actor.id)
                if bike.state == "riding":
                    loc = bike.actor.transform.location
                    start = lane.start.location
                    fwd = lane.start.get_forward_vector()
                    off = abs(-(loc.x - start.x) * fwd.y + (loc.y - start.y) * fwd.x)
                    if off > 0.05:
                        off_line.append(off)
            for lane in bikes.lanes:
                on_road = [b for b in bikes.bikes if b.lane is lane and b.state in ("rolling", "riding", "stopping",
                                                                                      "lingering")]
                if len(on_road) > 1:
                    lane_actives.append((lane.side, len(on_road)))
                if lane.active is not None and lane.active.state in ("waiting", "leaving", "parked"):
                    lane_actives.append((lane.side, "active but " + lane.active.state))
        parked = set()
        flow_module.CyclistFlow._remove = watch_remove
        try:
            sim.run(DURATION, observe)
        finally:
            flow_module.CyclistFlow._remove = original_remove

        print("  spawned: pedestrians {}, bicycles {}, background {}; removed: pedestrians {}, bicycles {}, background {}".format(
            ped.spawned, bikes.spawned, bubble.spawned, ped.removed, bikes.removed, bubble.removed))
        spawns, removes = check_visibility(sim, bike_removals)

        # Counts are reached
        check(counts["ped"] == 7, f"the pedestrian flow never reached 1 + crowd count = 7 (max {counts['ped']})")
        check(counts["bikes"] == 3, f"the cyclist flow never reached 3 (max {counts['bikes']})")
        check(counts["cars"] == 6 and counts["motorcycles"] == 2 and counts["bicycles"] == 2 and counts["walkers"] == 8,
              f"the background bubble never reached its counts: {counts}")
        check(ped.removed >= 3 and ped.spawned > 7, f"the pedestrian flow did not turn over: {ped.spawned} / {ped.removed}")
        check(bikes.removed >= 2 and bikes.spawned > 3, f"the cyclist flow did not turn over: {bikes.spawned} / {bikes.removed}")
        check(bubble.removed >= 3 and bubble.spawned >= 3, f"the bubble did not follow the ego: {bubble.spawned} / {bubble.removed}")

        # Pedestrians
        check(walker_checks, "no pedestrian was spawned by the flow")
        for d, walker, tick in walker_checks:
            check(flow_module.FLOW_MIN_CROSSING_DISTANCE - 0.01 <= d <= flow_module.FLOW_RADIUS + 0.01,
                  f"a flow pedestrian spawned {d:.1f} m from the crossing, expected 15 to 60 m")
        ticks = [tick for _, _, tick in walker_checks]
        gaps = [(b - a) * sim.dt for a, b in zip(ticks, ticks[1:])]
        check(gaps and min(gaps) >= flow_module.FLOW_SPAWN_INTERVAL[0] - sim.dt - 1e-9,
              f"flow pedestrians spawned {min(gaps) if gaps else 0:.2f} s apart, at least 1.5 s")
        check(max(gaps or [0]) < 60.0, "a pedestrian was never spawned for a minute")
        # Every walker removed mid-run is out of the pool; the controllers of removed walkers are gone
        for e in sim.events:
            if e.kind == "remove" and e.phase == "run":
                check(e.what.id not in CarlaDataProvider._carla_actor_pool, "a removed actor is still in the pool")
        gone = [e.what for e in sim.events if e.kind == "remove" and e.phase == "run" and e.what.type_id.startswith("walker")]
        check(all(sum(1 for c in sim.world.controllers if c.walker is w and c.is_alive) == 0 for w in gone),
              "the AI controller of a removed pedestrian is still alive")
        stuck = [w for w in gone if w.blocked_after is not None]
        check(stuck, "no stuck pedestrian was removed")

        # Cyclists
        check(not waiting_physics, f"a waiting bicycle had its physics on: {set(waiting_physics)}")
        check(not lane_actives, f"two bicycles of a lane on the road at once, or the wrong one active: {lane_actives[:3]}")
        check(not off_line, f"a riding bicycle left its line by {max(off_line or [0]):.2f} m")
        bike_spawns = [e for e in sim.events if e.kind == "spawn" and e.category == "bicycle" and not e.autopilot]
        check(all(sim.cmap.get_waypoint(e.location, project_to_road=False, lane_type=carla.LaneType.Driving) is None
                  for e in bike_spawns), "a cyclist was spawned on a driving lane")
        check(all(not e.what.physics for e in bike_spawns if e.what.is_alive and e.what.transform.location is not None
                  and e.what.id in {b.actor.id for b in bikes.bikes if b.state == "waiting"}),
              "a waiting bicycle has physics")
        check(len(bike_removals) >= 2 and all(lingered >= 0.0 for _, lingered, _, _ in bike_removals), "no bicycle was removed")
        # After the ride a bicycle rolls onto the far sidewalk with its physics off, its lane is free at once
        check(any(lane.exit is not None for lane in bikes.lanes), "no cycle lane has a far sidewalk spot to leave to")
        check({"leaving", "parked"} <= exit_states, f"no bicycle left to the far sidewalk: states {exit_states}")
        check(not exit_problems, f"cyclists leaving the road: {sorted(set(exit_problems))}")
        check(parked_gaps and max(parked_gaps) < 0.05, f"a bicycle parked {max(parked_gaps or [0]):.2f} m from the exit spot")
        check(not growth, f"the actors or controllers grew past the configured counts: {growth[:3]}")
        check(all(0.0 < b.lane.speed <= 5.0 for b in bikes.bikes) and all(l.push > 0.0 for l in bikes.lanes),
              "a lane has no speed or push")
        rode = [b for b, _, _, _ in bike_removals]
        check(rode, "no bicycle rode across")

        # Background
        slowed = sim.world.client.manager.speed_differences
        check(slowed and all(pct == 50.0 and a.type_id in cb.BICYCLES for a, pct in slowed),
              f"background bicycles are not slowed in the Traffic Manager: {len(slowed)}")
        traffic_bikes = [e for e in sim.events if e.kind == "spawn" and e.category == "bicycle" and e.autopilot]
        check(len(slowed) == len(traffic_bikes), f"{len(slowed)} slowed bicycles for {len(traffic_bikes)} spawned")
        lane_changes = sim.world.client.manager.lane_changes
        check(lane_changes and not any(enabled for _, enabled in lane_changes),
              f"background vehicles must have Traffic Manager lane changes off: {len(lane_changes)} set")
        check(all(e.what.autopilot for e in sim.events if e.kind == "spawn" and e.category in ("car", "motorbike")),
              "a background vehicle without autopilot")
        # Spawns within the radius of the ego, and on free spawn points
        for e in sim.events:
            if e.kind == "spawn" and e.phase == "run" and e.time > 0.0 and e.autopilot:
                d = flow_module.dist2d(e.location, e.ego.location)
                check(d <= 120.0 + 1e-6, f"a background vehicle spawned {d:.0f} m from the ego, radius 120 m")
        check_free_spawn_points(sim)

        # The scenario's list of actors does not grow with the flows
        check(len(sim.scenario.other_actors) <= 6 + 2 + 2 + 8 + 3, f"{len(sim.scenario.other_actors)} actors in the list")
        check_stop_file(sim, stop_file)
        check_cleanup(sim)
    finally:
        sim.close()
        stop_file.unlink(missing_ok=True)
        Path(ending_marker(stop_file)).unlink(missing_ok=True)


def check_free_spawn_points(sim):
    """No vehicle spawned within 8 m of another vehicle that was alive at that moment (replayed from the log)."""
    alive = {}  # id -> [born, died, location]
    for e in sim.events:
        if e.kind == "spawn" and e.what.type_id.startswith("vehicle"):
            alive[e.what.id] = [e.time, float("inf"), e.location]
        elif e.kind == "remove" and e.what.id in alive:
            alive[e.what.id][1] = e.time
    spawns = [e for e in sim.events if e.kind == "spawn" and e.phase == "run" and e.time > 0.0
              and e.what.type_id.startswith("vehicle") and e.autopilot]
    for e in spawns:
        for other_id, (born, died, loc) in alive.items():
            if other_id == e.what.id or not born <= e.time <= died:
                continue
            check(flow_module.dist2d(loc, e.location) >= 8.0 - 1e-6,
                  f"a vehicle was spawned {flow_module.dist2d(loc, e.location):.1f} m from another one (at its spawn point)")


def check_stop_file(sim, stop_file):
    tree = sim.scenario.behavior_tree
    check(tree.status == py_trees.common.Status.RUNNING, f"the tree is {tree.status} without a stop file")
    check(not stop_file.exists(), "stale stop file")
    sim.run(1.0)
    check(tree.status == py_trees.common.Status.RUNNING, "the tree ended without a stop file")
    check(sim.ego.last_control is None, "the ego was braked without a stop file")
    stop_file.write_text("")
    sim.step()
    check(tree.status == py_trees.common.Status.SUCCESS, f"the tree is {tree.status} after the stop file was created")
    control = sim.ego.last_control
    check(control is not None and control.brake == 1.0 and control.hand_brake and control.throttle == 0.0,
          "the stop file did not apply full brake and the hand brake to the ego")


def check_cleanup(sim):
    """What ComposedScenario.terminate() and CarlaDataProvider.cleanup() do at the end."""
    sim.phase = "cleanup"
    flows = blocks_flows(sim)
    sim.scenario.terminate()
    check(all(not c.is_alive for c in sim.world.controllers), "an AI controller is still alive after terminate()")
    check(sim.ctx._controllers == [], "the context still tracks controllers")
    pool = set(CarlaDataProvider._carla_actor_pool)
    leftover = [a for a in sim.actors if a.is_alive]
    check(all(a.id in pool for a in leftover), "a live actor is not in ScenarioRunner's pool (it would never be removed)")
    check(all(a.id not in pool for a in sim.actors if not a.is_alive), "a removed actor is still in the pool")
    sim.scenario.remove_all_actors()
    check(all(a.id not in CarlaDataProvider._carla_actor_pool for a in sim.scenario.other_actors if a is not None),
          "remove_all_actors left the pool inconsistent")
    # CarlaDataProvider.cleanup() destroys what is left in the pool
    for actor in list(CarlaDataProvider._carla_actor_pool.values()):
        if actor.is_alive:
            actor.destroy()
    check(not [a for a in sim.actors if a.is_alive], "actors are left alive after the cleanup")
    del flows


def check_no_navigation(module, cmap):
    print("flows: no navigation mesh (Town12): the pedestrian flow becomes a single pass")
    sim = Sim(module, cmap, PARAMS, nav=False)
    try:
        names = [n.name for n in sim.scenario.behavior_tree.iterate()]
        check("PedestrianFlow" not in names and "Adversary" in names, f"no single pass without a navigation mesh: {names}")
        check("CyclistFlow" in names and "BackgroundBubble" in names, "the other flows need no navigation mesh")
        check(sim.by_tag["pedestrian"].walker is not None and len(sim.by_tag["crowd"].walkers) > 0,
              "the single-pass pedestrian and crowd were not spawned")
        check(not sim.by_tag["background"]._walker_pairs, "background walkers without a navigation mesh")
        sim.run(5.0)  # the flows that remain run without a navigation mesh
        check(sim.by_tag["background"].bubble.walkers_wanted == 0, "the bubble wants walkers without a navigation mesh")
    finally:
        sim.close()


def check_hide_rule_config(module, cmap):
    print("flows: <spawning> changes the rule the flows follow")
    params = dict(PARAMS)
    params["spawning"] = {"hide_distance": "80", "behind_distance": "30", "behind_angle": "100", "far_distance": "150",
                          "side_angle": "45"}
    sim = Sim(module, cmap, params)
    try:
        check(sim.ctx.hide.hide_distance == 80.0 and sim.ctx.hide.behind_distance == 30.0
              and sim.ctx.hide.behind_angle == 100.0 and sim.ctx.hide.far_distance == 150.0
              and sim.ctx.hide.side_angle == 45.0, f"the spawning block did not set the rule: {sim.ctx.hide}")
        sim.run(30.0)
        runtime = [e for e in sim.events if e.kind == "spawn" and e.phase == "run" and e.time > 0.0]
        rule = sim.ctx.hide
        check(runtime and all(rule.hidden(e.ego, e.location, e.ego_speed) for e in runtime),
              "a spawn broke the configured hide rule")
    finally:
        sim.close()


def check_timeout(module, cmap):
    print("manual run: the 7 day timeout")
    sim = Sim(module, cmap, PARAMS)
    try:
        scenario = sim.scenario
        check(scenario.timeout == 7 * 24 * 3600.0, f"timeout {scenario.timeout}")
        node = scenario.timeout_node
        check(node is not None and type(node).__name__ == "TimeOut", f"timeout node {node}")
        node.initialise()
        for days in (0.01, 1.0, 3.0, 6.9):
            sim.t = days * 24 * 3600.0
            check(node.update() == py_trees.common.Status.RUNNING, f"TimeOut ended after {days} days")
        check(not node.timeout, "the TimeOut node says it timed out")
        sim.t = 7.01 * 24 * 3600.0
        check(node.update() == py_trees.common.Status.SUCCESS and node.timeout, "TimeOut never ends")
        node.timeout = False
        # ScenarioManager.analyze_scenario with a long run behind it: a clean result
        manager = ScenarioManager()
        manager.scenario, manager.scenario_tree = scenario, scenario.scenario_tree
        manager.ego_vehicles, manager.other_actors = scenario.ego_vehicles, scenario.other_actors
        manager.start_system_time, manager.end_system_time = 1000.0, 1000.0 + 3 * 3600.0
        manager.scenario_duration_system, manager.scenario_duration_game = 3 * 3600.0, 3 * 3600.0
        for criterion in scenario.get_criteria():
            criterion.test_status = "SUCCESS"  # nothing was hit
        failed = manager.analyze_scenario(False, None, None, None)
        check(failed is False, "analyze_scenario reports a failure for a long clean manual run")
    finally:
        sim.close()


def run_until(sim, condition, seconds):
    """Step until `condition()` holds; True when it did within `seconds` of game time."""
    for _ in range(int(seconds / sim.dt)):
        if condition():
            return True
        sim.step()
    return condition()


def add_obstacle(sim, type_id, location):
    """A still actor (a walker or a vehicle) of the scenario's pool at `location`."""
    actor = LiveActor(sim, type_id, carla.Transform(location, carla.Rotation()))
    sim.actors.append(actor)
    CarlaDataProvider._carla_actor_pool[actor.id] = actor
    return actor


def remove_obstacle(actor):
    CarlaDataProvider._carla_actor_pool.pop(actor.id, None)
    actor.is_alive = False


def check_reversing(module, cmap):
    """The 'behind the car' part of the hide rule is off while the ego reverses (the context reads its velocity)."""
    print("flows: reversing switches the behind rule off")
    sim = Sim(module, cmap, PARAMS)
    try:
        ctx, ego = sim.ctx, sim.ego
        sim.hold_until = 1e12
        sim.step()
        yaw = math.radians(ego.transform.rotation.yaw)
        here = ego.transform.location

        def point(distance, degrees):
            a = yaw + math.radians(degrees)
            return carla.Location(here.x + distance * math.cos(a), here.y + distance * math.sin(a), here.z)

        behind, ahead_side = point(40.0, 180.0), point(80.0, 90.0)
        far_behind, very_far_behind, side_back = point(80.0, 180.0), point(130.0, 180.0), point(80.0, 100.0)
        for speed, expect_speed, hidden_behind in ((0.0, 0.0, True), (3.0, 3.0, True), (-0.3, -0.3, True),
                                                   (-3.0, -3.0, False)):
            ego.velocity = (speed * math.cos(yaw), speed * math.sin(yaw))
            sim.t += sim.dt  # the context reads the velocity once per game time
            check(abs(ctx.ego_forward_speed() - expect_speed) < 1e-6, f"forward speed {ctx.ego_forward_speed()}")
            check(ctx.is_hidden(ego.transform, behind) == hidden_behind and ctx.hidden(behind) == hidden_behind,
                  f"40 m behind with forward speed {speed}: hidden should be {hidden_behind}")
            check(ctx.is_hidden(ego.transform, ahead_side), "80 m off to the side must stay hidden while reversing")
            check(not ctx.is_hidden(ego.transform, point(60.0, 0.0)), "60 m straight ahead is hidden")
            # 50 to 120 m straight behind is in view while the ego reverses, hidden otherwise; 120 m on is always hidden
            check(ctx.is_hidden(ego.transform, far_behind) == hidden_behind,
                  f"80 m behind with forward speed {speed}: hidden should be {hidden_behind}")
            check(ctx.is_hidden(ego.transform, very_far_behind), f"130 m behind is in view with forward speed {speed}")
            check(ctx.is_hidden(ego.transform, side_back), f"80 m at 100 degrees is in view with forward speed {speed}")
        # A flow does not use a spot behind the car while it reverses, and uses it when the ego drives forward
        ped, bikes, bubble = blocks_flows(sim)
        for speed, found in ((-3.0, None), (3.0, behind)):
            ego.velocity = (speed * math.cos(yaw), speed * math.sin(yaw))
            sim.t += sim.dt
            picked = ped._pick_hidden([behind], ego.transform)
            check(picked is found, f"a pedestrian spawn point 40 m behind with forward speed {speed}: {picked}")
        # No readable velocity: the ego is treated as going forward
        real = ctx.ego

        class Unreadable:
            id = real.id

            def get_velocity(self):
                raise RuntimeError("injected: not readable")
        ctx.ego = Unreadable()
        sim.t += sim.dt
        check(ctx.ego_forward_speed() is None, "an unreadable velocity must give None")
        ctx.ego = real
    finally:
        sim.close()


def check_pending_removals(module, cmap):
    """An actor the server will not destroy is retried, counts against its flow's count, and is given up after a while."""
    print("flows: an actor that cannot be destroyed is retried and counts against its flow's capacity")
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        ctx = sim.ctx
        sim.hold_until = 1e12  # the ego stays where it starts
        sim.step()

        def cars():
            return [a for a in sim.actors if a.is_alive and getattr(a, "category", None) == "car"]
        check(len(cars()) == 6 and len(bubble.tracked["cars"]) == 6, f"{len(cars())} background cars at the start")
        group = (bubble, "cars")
        # The server refuses twice: at the removal and at the first retry, then it works
        victim = bubble.tracked["cars"].pop()
        victim.destroy_failures = 2
        ctx.remove_actor(victim, group=group)
        check(victim.is_alive and ctx.pending_removals(group) == 1, "a failed removal is not kept as pending")
        check(victim.id in CarlaDataProvider._carla_actor_pool, "an actor that is still alive left the pool")
        check(ctx.pending_removals((bubble, "bicycles")) == 0 and ctx.pending_removals(ped) == 0,
              "a pending removal counts for another group")
        deadline = sim.t + 2.5
        while sim.t < deadline and victim.is_alive:
            sim.step()
            check(len(cars()) <= 6, f"{len(cars())} background cars while one was waiting to be destroyed (6 wanted)")
            if victim.is_alive:
                check(len(bubble.tracked["cars"]) <= 5, "the bubble spawned a replacement for an actor still alive")
        check(not victim.is_alive and ctx.pending_removals(group) == 0, "the retry did not destroy the actor")
        check(victim.id not in CarlaDataProvider._carla_actor_pool, "a destroyed actor is still in the pool")
        # One that stays alive is given up after REMOVAL_RETRIES tries, and is left to the final cleanup
        stubborn = bubble.tracked["cars"].pop()
        stubborn.destroy_failures = 1000
        out = io.StringIO()
        with redirect_stdout(out):
            ctx.remove_actor(stubborn, group=group)
            check(ctx.pending_removals(group) == 1, "a failed removal is not kept as pending")
            sim.run((cb_base.REMOVAL_RETRIES + 2) * cb_base.REMOVAL_RETRY_PERIOD + 1.0)
        check(ctx.pending_removals(group) == 0 and len(ctx._pending) == 0, "a stubborn actor is retried for ever")
        check(stubborn.is_alive and stubborn.id in CarlaDataProvider._carla_actor_pool,
              "a given-up actor is not left to the final cleanup")
        # A destroy that returns False (no exception) drops the actor from the pool: it is put back, and still retried
        refused = bubble.tracked["cars"].pop()
        refused.destroy = lambda: False
        ctx.remove_actor(refused, group=group)
        check(refused.is_alive and ctx.pending_removals(group) == 1 and refused.id in CarlaDataProvider._carla_actor_pool,
              "an actor whose destroy returned False is not pending and in the pool")
        check(refused.id in {thing[0].id for thing in ctx.solid_things()}, "a pending actor is invisible to solid_things()")
        sim.run((cb_base.REMOVAL_RETRIES + 2) * cb_base.REMOVAL_RETRY_PERIOD + 1.0)
        check(ctx.pending_removals(group) == 0 and refused.id in CarlaDataProvider._carla_actor_pool,
              "a given-up actor that was dropped from the pool by a False destroy was not put back")
        del refused.destroy
        check("could not be destroyed" in out.getvalue(), "giving up on an actor logged nothing")
        check(stubborn.destroy_failures <= 1000 - cb_base.REMOVAL_RETRIES, "the actor was not tried again")
        # An actor that is already gone (a dead one) is no pending removal
        gone = bubble.tracked["cars"].pop()
        gone.is_alive = False
        ctx.remove_actor(gone, group=group)
        check(ctx.pending_removals(group) == 0, "an actor that is gone counts as pending")
        # The pedestrian and cyclist flows count them too
        record = bikes.bikes[0]
        record.actor.destroy_failures = 1
        bikes._remove(record)
        check(ctx.pending_removals(bikes) == 1 and record.actor.is_alive, "a bicycle that cannot be destroyed is not pending")
        sim.run(2.5)
        check(not record.actor.is_alive and ctx.pending_removals(bikes) == 0, "the bicycle was not destroyed on a retry")
        walker = ped.walkers[0]
        walker.actor.destroy_failures = 1
        ped._remove(walker)
        check(ctx.pending_removals(ped) == 1 and walker.actor.is_alive, "a pedestrian that cannot be destroyed is not pending")
        sim.run(2.5)
        check(not walker.actor.is_alive and ctx.pending_removals(ped) == 0, "the pedestrian was not destroyed on a retry")
    finally:
        sim.close()


def check_background_autopilot(module, cmap):
    """A background vehicle is requested without autopilot; if its autopilot cannot be set it is removed, not leaked."""
    print("flows: a background vehicle whose autopilot cannot be set is removed and untracked")
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        ctx = sim.ctx
        sim.hold_until = 1e12
        sim.step()
        victim = bubble.tracked["cars"].pop()
        ctx.remove_actor(victim, group=(bubble, "cars"))
        requested = []

        def request(model, spawn_point=None, **kwargs):
            requested.append(kwargs.get("autopilot"))
            actor = sim.request_new_actor(model, spawn_point, **kwargs)
            if len(requested) == 1:
                actor.autopilot_failures = 1  # the first one cannot get its autopilot
            return actor
        before = list(sim.actors)
        spawned = bubble.spawned
        out = io.StringIO()
        bubble.next_check = 0.0
        with mock.patch.object(CarlaDataProvider, "request_new_actor", staticmethod(request)), redirect_stdout(out):
            bubble.update()  # the flow survives the error: it logs it and backs off
        new = [a for a in sim.actors if a not in before]
        check(len(new) == 1 and requested == [None],
              f"{len(new)} vehicles requested after a failed autopilot, it must stop at the first: {requested}")
        check(all(not autopilot for autopilot in requested), f"the autopilot was requested with the spawn: {requested}")
        failed = new[0]
        check(not failed.is_alive and failed.id not in CarlaDataProvider._carla_actor_pool,
              "a vehicle without an autopilot was left on the server")
        check(failed not in bubble.tracked["cars"] and len(bubble.tracked["cars"]) == 5 and bubble.spawned == spawned,
              "the vehicle that failed is still tracked or was counted")
        check(bubble.errors == 1 and bubble._backoff_until > sim.t and "a CARLA call failed" in out.getvalue(),
              "the failed autopilot did not reach the flow's back-off and throttled log")
        # After the back-off the next look spawns one that works
        sim.run(flow_module.FLOW_ERROR_BACKOFF + 2.0)
        kept = next((a for a in sim.actors if a not in before and a is not failed and a.is_alive and a.autopilot), None)
        check(len(bubble.tracked["cars"]) == 6 and kept in bubble.tracked["cars"],
              "the bubble did not spawn a vehicle after the back-off")
        check(all(e.autopilot for e in sim.events if e.kind == "spawn" and e.what is kept), "the spawn log has no autopilot")
    finally:
        sim.close()


def check_initial_population(module, cmap):
    print("flows: pedestrians are placed at load, already walking at the first tick")
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        capacity = 1 + 6
        check(1 <= ped.initial <= flow_module.FLOW_INITIAL and len(ped.walkers) == ped.initial <= capacity,
              f"{ped.initial} pedestrians placed at load, expected 1 to {flow_module.FLOW_INITIAL}")
        pool = CarlaDataProvider._carla_actor_pool
        load_spawns = {e.what.id: e for e in sim.events if e.kind == "spawn" and e.phase == "load"}
        for w in ped.walkers:
            d = flow_module.dist2d(w.actor.transform.location, ped.center)
            check(w.pending and w.controller.started_tick is None, "an initial pedestrian started before the first tick")
            check(d <= flow_module.FLOW_INITIAL_RADIUS + 1.0, f"an initial pedestrian {d:.0f} m from the crossing")
            check(w.actor.id in pool and w.actor.id in load_spawns and load_spawns[w.actor.id].time == 0.0,
                  "an initial pedestrian was not spawned at load")
            check(w.controller in sim.ctx._controllers, "an initial pedestrian's controller is not tracked")
        check(any(w.leg == 1 for w in ped.walkers) or ped.initial < 3, "no initial pedestrian is on the crosswalk")
        initial = list(ped.walkers)  # the flow may spawn more on the first tick (spots hidden from a far ego start)
        sim.step()
        for w in initial:
            check(not w.pending and w.controller.started_tick == 1, "an initial pedestrian was not started at the first tick")
            check(w.actor.walking and w.actor.destination is w.legs[w.leg],
                  "an initial pedestrian is not walking to its leg")
        sim.run(3.0)
        check(any(w.actor.walked > 1.0 for w in ped.walkers), "the initial pedestrians do not walk")
        # At most the capacity, now and later
        sim.run(20.0, lambda s_: check(len(ped.walkers) <= capacity, f"{len(ped.walkers)} pedestrians, at most {capacity}"))
        # A flow without a pedestrian slot places nobody
        empty = flow_module.PedestrianFlow(sim.ctx, 0)
        check(empty.populate() == 0, "populate() placed pedestrians for a flow of capacity 0")
    finally:
        sim.close()


def check_flow_errors(module, cmap):
    """A CARLA RuntimeError in a flow's update() does not end the run: logged once per 10 s, 1 s back-off."""
    print("flows: a CARLA error in update() is survived")
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        tree = sim.scenario.behavior_tree
        out = io.StringIO()
        calls = {"spawn": 0, "refill": 0, "launch": 0, "despawn": 0}

        def boom(name):
            def raiser(*args, **kwargs):
                calls[name] += 1
                raise RuntimeError("injected " + name)
            return raiser

        ped_spawned = ped.spawned
        with redirect_stdout(out):
            # Phase 1: the server refuses every spawn for 4 s (the pedestrian flow wants more pedestrians)
            with mock.patch.object(CarlaDataProvider, "request_new_actor", staticmethod(boom("spawn"))):
                sim.run(4.0)
            check(calls["spawn"] >= 1, "the pedestrian flow never tried to spawn during the failure")
            check(calls["spawn"] <= 4 / flow_module.FLOW_ERROR_BACKOFF + 1,
                  f"{calls['spawn']} spawn attempts in 4 s: the flow does not back off")
            check(tree.status == py_trees.common.Status.RUNNING, f"the tree is {tree.status} after refused spawns")
            # Phase 2: each flow's work raises, for 5 s
            with ExitStack() as stack:
                stack.enter_context(mock.patch.object(flow_module.PedestrianFlow, "_refill", boom("refill")))
                stack.enter_context(mock.patch.object(flow_module.CyclistFlow, "_launch", boom("launch")))
                stack.enter_context(mock.patch.object(flow_module.BackgroundBubble, "_despawn", boom("despawn")))
                sim.run(5.0)
            check(tree.status == py_trees.common.Status.RUNNING, f"the tree is {tree.status} after flow errors")
            for name in ("refill", "launch"):
                check(1 <= calls[name] <= 5 / flow_module.FLOW_ERROR_BACKOFF + 1, f"{name}: {calls[name]} calls in 5 s")
            check(1 <= calls["despawn"] <= 6, f"despawn: {calls['despawn']} calls in 5 s")
        lines = [line for line in out.getvalue().splitlines() if "a CARLA call failed" in line]
        for flow_name in ("PedestrianFlow", "CyclistFlow", "BackgroundBubble"):
            mine = [line for line in lines if f"[{flow_name}]" in line]
            check(len(mine) == 1, f"{flow_name} logged {len(mine)} lines in 9 s of errors, expected exactly 1")
        check(all(f.errors >= 1 for f in (ped, bikes, bubble)), "a flow did not count its errors")
        # After the failure everything carries on
        sim.run(20.0)
        check(tree.status == py_trees.common.Status.RUNNING, "the tree ended after the errors")
        check(ped.spawned > ped_spawned, "the pedestrian flow never spawned again after the errors")
        check(not [a for a in sim.alive("walker") if a.id not in CarlaDataProvider._carla_actor_pool],
              "a walker is alive but not in the pool after the errors")
        # A flow's own update() with a failing _update: RUNNING, logged once
        out = io.StringIO()
        with redirect_stdout(out), mock.patch.object(flow_module.PedestrianFlow, "_update", boom("spawn")):
            statuses = {ped.update() for _ in range(3)}
        check(statuses == {py_trees.common.Status.RUNNING}, f"a failing update returned {statuses}")
        # An error in one bicycle's step does not take the others down
        with mock.patch.object(flow_module.CyclistFlow, "_step", boom("spawn")):
            before = len(bikes.bikes)
            bikes.update()
            check(len(bikes.bikes) == before, "one failing step removed a bicycle that is alive")
    finally:
        sim.close()


def check_controller_failures(module, cmap):
    """What the error paths drop is destroyed: the walker, its controller, and the pool entry."""
    print("flows: the controller-failure paths destroy what they drop")
    # At load: a wandering pedestrian whose AI controller cannot be created is removed, not left standing there
    sim = Sim(module, cmap, PARAMS, fail_controllers=2)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        background = sim.by_tag["background"]
        check(len(background._walker_pairs) == 8 - 2, f"{len(background._walker_pairs)} background pairs, expected 6")
        lost = [e.what for e in sim.events if e.kind == "remove" and e.phase == "load" and e.what.type_id.startswith("walker")]
        check(len(lost) == 2, f"{len(lost)} walkers destroyed at load for a failed controller, expected 2")
        for walker in lost:
            check(not walker.is_alive and walker.id not in CarlaDataProvider._carla_actor_pool
                  and walker not in sim.scenario.other_actors, "a walker without a controller was left behind")
        tracked = {pair[0].id for pair in background._walker_pairs}
        stray = [w for w in sim.alive("walker") if w.id not in tracked and w.id not in {x.actor.id for x in ped.walkers}]
        check(not stray, f"{len(stray)} walkers alive that nobody tracks")
        # The bubble brings the count back to 8, not above it
        sim.run(20.0, lambda s_: check(len(bubble.walkers) <= 8, f"{len(bubble.walkers)} background walkers, at most 8"))
        check(len(bubble.walkers) == 8, f"the bubble holds {len(bubble.walkers)} walkers, expected 8")
    finally:
        sim.close()

    # In the flow: the controller refuses an order, again and again: the walker goes, with its controller
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        sim.hold_until = 1e12
        sim.run(1.0)
        walker = next((w for w in ped.walkers if not w.pending and w.leg == 0), None)
        check(walker is not None, "no started pedestrian on its first leg to break")
        if walker is not None:
            def refuse(location):
                raise RuntimeError("injected: the controller refuses")
            walker.controller.go_to_location = refuse
            target = walker.legs[0]
            walker.actor.transform = carla.Transform(carla.Location(target.x, target.y, target.z), walker.actor.transform.rotation)
            sim.run(0.5)
            check(walker not in ped.walkers, "a pedestrian whose controller fails stays in the flow")
            check(not walker.actor.is_alive and not walker.controller.is_alive,
                  "a pedestrian whose controller fails was left alive on the server")
            check(walker.actor.id not in CarlaDataProvider._carla_actor_pool and walker.controller not in sim.ctx._controllers,
                  "a dropped pedestrian is still in the pool or the controller list")
        # A pedestrian that died on the server: its controller is destroyed too, nothing is left in the pool
        victim = next(w for w in ped.walkers if not w.pending)
        victim.actor.is_alive = False
        sim.run(0.2)
        check(victim not in ped.walkers and not victim.controller.is_alive, "a dead pedestrian's controller was not destroyed")
        check(victim.controller not in sim.ctx._controllers, "a dead pedestrian's controller is still tracked")
        # Same for a background walker that died
        pair = bubble.walkers[0]
        pair[0].is_alive = False
        sim.run(2.0)
        check(pair not in bubble.walkers and not pair[1].is_alive, "a dead background walker's controller was not destroyed")
        check(pair[1] not in sim.ctx._controllers, "a dead background walker's controller is still tracked")
    finally:
        sim.close()


def check_refill_cooldown(module, cmap):
    """A pool that can never fill (no navigation point in range) is not sampled every tick forever."""
    print("flows: a pool that cannot fill rests instead of sampling every tick")
    sim = Sim(module, cmap, {"ego_control": {"mode": "manual"}, "pedestrian": {}, "crowd": {"count": "6"}})
    try:
        ped = next(n for n in sim.scenario.behavior_tree.iterate() if isinstance(n, flow_module.PedestrianFlow))
        calls = [0]
        far = carla.Location(5000.0, 5000.0, 0.0)

        def nowhere():
            calls[0] += 1
            return far
        sim.world.get_random_location_from_navigation = nowhere
        seconds = 30.0
        sim.run(seconds)
        bound = (seconds / flow_module.FLOW_REFILL_COOLDOWN + 1) * flow_module.FLOW_REFILL_DRY * flow_module.FLOW_SAMPLES_PER_TICK
        check(0 < calls[0] <= bound, f"{calls[0]} navigation calls in {seconds:.0f} s for pools that cannot fill, at most {bound:.0f}")
        check(ped.spawned == ped.initial, "pedestrians were spawned without any point in range")
        # Points in range come back: the pools fill, and the flow spawns again
        sim.world.get_random_location_from_navigation = lambda: sim.world.nav_random.choice(sim.world.nav_points)
        sim.run(30.0)
        check(len(ped.spawn_pool) > 0 and ped.spawned > ped.initial, "the flow did not recover when points were back")
    finally:
        sim.close()


def check_finished_walkers(module, cmap):
    """A finished pedestrian in view keeps being sent to hidden points (no freeze after 3), and goes once hidden."""
    print("flows: a finished pedestrian in view is sent on again and again, and removed once hidden")
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, _, _ = blocks_flows(sim)
        sim.hold_until = 1e12
        sim.run(1.0)
        ego = sim.ego.transform
        yaw = math.radians(ego.rotation.yaw)
        walker = next(w for w in ped.walkers if not w.pending)
        ahead = carla.Location(ego.location.x + 20.0 * math.cos(yaw), ego.location.y + 20.0 * math.sin(yaw), ego.location.z)
        walker.actor.transform = carla.Transform(ahead, ego.rotation)
        far = [carla.Location(ego.location.x + 300.0 + 10.0 * i, ego.location.y, ego.location.z) for i in range(5)]
        ped.spawn_pool, ped.destination_pools = list(far), ([], [])
        walker.done, walker.done_at = True, sim.t
        now = sim.t
        for k in range(6):
            now += flow_module.FLOW_LINGER + 1.0
            ped._step(walker, now, ego)
            check(walker.repicks == k + 1 and not walker.done, f"repick {k + 1}: repicks {walker.repicks}, done {walker.done}")
            check(walker.actor.destination in far or any(walker.actor.destination is p for p in far),
                  "a finished pedestrian was not sent to a hidden point")
            walker.done, walker.done_at = True, now  # it has arrived (in view again)
        check(walker.repicks == 6 > 3, "a finished pedestrian stopped being sent on after three repicks")
        # No hidden point at the moment: it does not count as a repick, and asks again soon
        ped.spawn_pool = []
        now += flow_module.FLOW_LINGER + 1.0
        ped._step(walker, now, ego)
        check(walker.repicks == 6 and walker.done, "a repick without a hidden point was counted or lost the walker")
        ped.spawn_pool = list(far)
        now += flow_module.FLOW_REPICK_RETRY + 0.1
        ped._step(walker, now, ego)
        check(walker.repicks == 7 and not walker.done, "a finished pedestrian did not ask for a new point soon enough")
        # Once hidden it is removed
        walker.done, walker.done_at = True, now
        ego_far = carla.Transform(carla.Location(ego.location.x + 400.0, ego.location.y, ego.location.z), ego.rotation)
        ped._step(walker, now, ego_far)
        check(walker not in ped.walkers and not walker.actor.is_alive and not walker.controller.is_alive,
              "a finished pedestrian that is hidden was not removed and destroyed")
    finally:
        sim.close()


def check_cyclist_blocking(module, cmap):
    """No launch and no push into an actor on the line; a roll waits; all resume when the line is clear."""
    print("flows: a cyclist does not launch or ride into an actor on its line")
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        sim.hold_until = 1e12  # the ego stays where it starts, far from the ride
        sim.step()
        lane = next(l for l in bikes.lanes if l.bikes)
        front = lane.bikes[min(lane.bikes)]
        a, b = front.start_point, lane.end
        mid = carla.Location((a.x + b.x) / 2.0, (a.y + b.y) / 2.0, a.z)
        # 1. a pedestrian in the middle of the ride: the front bicycle waits, though it is ready
        walker = add_obstacle(sim, "walker.pedestrian.0001", mid)
        front.ready_at = sim.t
        held = bikes.held_up
        sim.run(4.0)
        check(front.state == "waiting" and lane.active is None, f"a bicycle launched into a pedestrian: {front.state}")
        check(bikes.held_up > held, "the launch was not held up (nothing counted)")
        # 2. the same for a vehicle that is not on the line but near it (3 m + half its length)
        remove_obstacle(walker)
        fx, fy = math.cos(lane.yaw), math.sin(lane.yaw)
        car = add_obstacle(sim, "vehicle.tesla.model3", carla.Location(mid.x - fy * 3.5, mid.y + fx * 3.5, mid.z))
        sim.run(2.0)
        check(front.state == "waiting", f"a bicycle launched with a vehicle 3.5 m from its ride: {front.state}")
        remove_obstacle(car)
        # 3. the ego next to the ride
        real_transform = sim.ego.transform
        sim.ego.transform = carla.Transform(carla.Location(mid.x, mid.y, mid.z), real_transform.rotation)
        sim.run(1.0)
        check(front.state == "waiting", "a bicycle launched with the ego on its ride")
        sim.ego.transform = real_transform
        # 4. a pedestrian 2 m beside the ride (like one on the zebra) does not stop it
        aside = add_obstacle(sim, "walker.pedestrian.0002", carla.Location(mid.x - fy * 2.0, mid.y + fx * 2.0, mid.z))
        check(run_until(sim, lambda: front.state != "waiting", 10.0), "a pedestrian 2 m beside the ride stopped a launch")
        remove_obstacle(aside)
        # 5. the ride: stop pushing while something is on the line within 4 m ahead, go on when clear
        check(run_until(sim, lambda: front.state == "riding", 10.0), f"the bicycle never rode: {front.state}")
        here = sim.ctx.location_of(front.actor)
        blocker = add_obstacle(sim, "walker.pedestrian.0003", carla.Location(here.x + fx * 3.5, here.y + fy * 3.5, here.z))
        sim.run(1.0)
        check(front.held > 0.5 and front.actor.braking, f"a riding bicycle did not brake for a pedestrian on its line ({front.held:.2f} s held)")
        check(front.actor.speed < lane.speed * 0.8 or front.state == "stopping", "a riding bicycle kept its speed")
        check(front.state == "riding", f"the held ride ended early: {front.state}")
        remove_obstacle(blocker)
        sim.run(0.4)
        check(not front.actor.braking and front.actor.speed > lane.speed * 0.95,
              f"the bicycle did not go on when the line was clear: speed {front.actor.speed:.2f} of {lane.speed:.2f}")
    finally:
        sim.close()

    # 6. the kinematic roll to the road waits for something in its way
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        sim.hold_until = 1e12
        sim.step()
        lane = next(l for l in bikes.lanes if l.bikes)
        front = lane.bikes[min(lane.bikes)]
        a, b = front.start_point, lane.start.location
        length = flow_module.dist2d(a, b)
        ux, uy = (b.x - a.x) / length, (b.y - a.y) / length
        front.state, front.since, front.rolled = "rolling", sim.t, 0.0
        lane.active = front
        blocker = add_obstacle(sim, "walker.pedestrian.0003", carla.Location(a.x + ux * 1.5, a.y + uy * 1.5, a.z))
        sim.run(1.0)
        check(front.rolled == 0.0 and front.state == "rolling", f"a rolling bicycle moved into a pedestrian: {front.rolled}")
        remove_obstacle(blocker)
        sim.run(0.5)
        check(front.rolled > 0.5, "the roll did not go on when the way was clear")
        check(run_until(sim, lambda: front.state != "rolling", 5.0), "the roll never ended")
    finally:
        sim.close()


def check_cyclist_retry(module, cmap):
    """A bicycle spawned on a retry spot starts its roll from there (no jump back to the original spot)."""
    print("flows: a retried waiting spot is kept for the roll")
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        sim.hold_until = 1e12
        chosen = None
        for lane in bikes.lanes:
            for position, wait in enumerate(lane.waits):
                retries = list(cyclist_wait_retries(sim.cmap, wait))
                if retries:
                    chosen = (lane, position, retries)
                    break
            if chosen:
                break
        check(chosen is not None, "no waiting spot has a retry spot")
        lane, position, retries = chosen
        for bike in list(bikes.bikes):  # a clean lane
            bikes._remove(bike)
        calls = []

        def refuse_first(model, spawn_point=None, **kwargs):
            calls.append(spawn_point)
            if len(calls) == 1:
                return None
            return sim.request_new_actor(model, spawn_point, **kwargs)
        with mock.patch.object(CarlaDataProvider, "request_new_actor", staticmethod(refuse_first)):
            check(bikes.place(lane, position, sim.t), "the bicycle was not placed on the retry spot")
        record = lane.bikes[position]
        wait = lane.waits[position]
        extra = retries[0][0]
        check(record.origin is calls[1], "the bicycle does not remember the transform it was spawned at")
        check(abs(flow_module.dist2d(record.origin.location, wait.location) - extra) < 1e-3,
              f"the bicycle stands {flow_module.dist2d(record.origin.location, wait.location):.2f} m from its lane's spot, expected {extra}")
        spot = record.actor.transform.location
        check(flow_module.dist2d(spot, record.origin.location) < 1e-3, "the bicycle is not on its retry spot")
        record.state, record.since, record.rolled = "rolling", sim.t, 0.0
        lane.active = record
        bikes._roll(record, sim.t, sim.dt)
        moved = flow_module.dist2d(record.actor.transform.location, spot)
        check(moved <= flow_module.CYCLE_ROLL_SPEED * sim.dt + 1e-6,
              f"the first step of the roll jumped {moved:.2f} m (retry spot {extra} m back)")
        check(run_until(sim, lambda: record.state == "riding", 10.0), "the roll never reached the road")
        end = record.actor.transform.location
        check(flow_module.dist2d(end, lane.start.location) < 1e-3, "the roll did not end at the start of the ride")
    finally:
        sim.close()


def check_cyclist_retry_hidden(module, cmap):
    """A flow only uses a retry spot (0.5 to 1 m further back) when that one is hidden, not just the lane's own spot."""
    print("flows: a bicycle is spawned at a retry spot only when that spot is hidden")
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        sim.hold_until = 1e12
        chosen = None
        for lane in bikes.lanes:
            for position, wait in enumerate(lane.waits):
                retries = list(cyclist_wait_retries(sim.cmap, wait))
                if retries:
                    chosen = (lane, position, wait, retries)
                    break
            if chosen:
                break
        check(chosen is not None, "no waiting spot has a retry spot")
        lane, position, wait, retries = chosen
        for bike in list(bikes.bikes):
            bikes._remove(bike)
        ego = sim.ego.transform
        calls = []

        def refuse_first(model, spawn_point=None, **kwargs):
            calls.append(spawn_point)
            if len(calls) == 1:
                return None
            return sim.request_new_actor(model, spawn_point, **kwargs)

        def hidden_only_at_the_spot(ego_transform, location):
            return flow_module.dist2d(location, wait.location) < 1e-3
        with mock.patch.object(CarlaDataProvider, "request_new_actor", staticmethod(refuse_first)), \
                mock.patch.object(sim.ctx, "is_hidden", hidden_only_at_the_spot):
            check(not bikes.place(lane, position, sim.t, ego=ego), "a bicycle was spawned at a retry spot in view")
        check(len(calls) == 1, f"{len(calls)} spawns were tried, the retry spots in view must be skipped")
        check(position not in lane.bikes and lane.blocked_until.get(position, 0.0) > sim.t, "the spot was not backed off")
        calls.clear()
        with mock.patch.object(CarlaDataProvider, "request_new_actor", staticmethod(refuse_first)), \
                mock.patch.object(sim.ctx, "is_hidden", lambda ego_transform, location: True):
            check(bikes.place(lane, position, sim.t, ego=ego), "a bicycle was not placed at a hidden retry spot")
        record = lane.bikes[position]
        check(len(calls) == 2 and record.origin is calls[1], "the retry spot was not used when it was hidden")
        # Without the ego (the blocks at load) nothing is checked
        for bike in list(bikes.bikes):
            bikes._remove(bike)
        calls.clear()
        with mock.patch.object(CarlaDataProvider, "request_new_actor", staticmethod(refuse_first)), \
                mock.patch.object(sim.ctx, "is_hidden", lambda ego_transform, location: False):
            check(bikes.place(lane, position, sim.t), "a load-time bicycle needs no hidden spot")
        check(len(calls) == 2, "a load-time spawn skipped its retry spot")
    finally:
        sim.close()


def same_lane_bike(sim, bikes, lane, state):
    """
    A bicycle record of `lane` past its ride (not in the lane's queue): parked at the exit spot, or leaving from the end
    of the ride towards it.
    """
    exit_spot = lane.exit.location
    start = exit_spot if state == "parked" else lane.end
    actor = add_obstacle(sim, "vehicle.bh.crossbike", start)
    actor.set_simulate_physics(False)
    record = flow_module._Bike(actor, lane, 99, state=state, since=sim.t, leave_from=start, leave_to=exit_spot)
    bikes.bikes.append(record)
    return record


def check_cyclist_lane_jam(module, cmap):
    """Nobody launches behind a bicycle still on its way to the far sidewalk; a held-up leave does not freeze for ever."""
    print("flows: a bicycle leaving or parked on the way to the far sidewalk holds the next one of its lane")
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        sim.hold_until = 1e12
        sim.step()
        clear = flow_module.CYCLE_LAUNCH_CLEAR_BIKE
        lane = next((l for l in bikes.lanes if l.bikes and l.exit is not None
                     and flow_module.dist2d(l.end, l.exit.location) > clear + 0.5), None)
        check(lane is not None, "no lane has an exit spot beyond the end of its ride")
        sim.ctx.is_hidden = lambda ego, location: False  # nothing is removed or spawned out of sight
        front = lane.bikes[min(lane.bikes)]
        for state in ("leaving", "parked"):
            other = same_lane_bike(sim, bikes, lane, state)
            front.ready_at = sim.t
            held = bikes.held_up
            sim.run(0.8)  # a bicycle that leaves from the end of the ride is not at the exit spot yet
            check(front.state == "waiting" and lane.active is None,
                  f"a bicycle launched behind one that is {state} on the way to the exit spot: {front.state}")
            check(bikes.held_up > held, f"the launch behind a {state} bicycle was not counted as held up")
            bikes._remove(other)
            check(run_until(sim, lambda: front.state != "waiting", 10.0), f"the lane stayed jammed after the {state} one went")
            # Back to the start for the next round
            front.state, front.ready_at = "waiting", None
            lane.active = None
            start = front.start_point
            front.actor.set_transform(carla.Transform(
                carla.Location(start.x, start.y, start.z + flow_module.CYCLE_SIDEWALK_DROP), front.actor.transform.rotation))
            front.actor.set_simulate_physics(False)
            lane.bikes[front.position] = front
    finally:
        sim.close()

    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        sim.hold_until = 1e12
        sim.step()
        sim.ctx.is_hidden = lambda ego, location: False
        lane = next(l for l in bikes.lanes if l.bikes and l.exit is not None)
        front = lane.bikes[min(lane.bikes)]
        front.ready_at = None
        front.state = "stopping"
        bikes._start_leaving(front, sim.t)  # a bicycle that has just ended its ride, on its way to the exit spot
        check(front.state == "leaving" and front.held == 0.0, "the bicycle did not start leaving")
        a, b = front.leave_from, front.leave_to
        length = flow_module.dist2d(a, b)
        blocker = add_obstacle(sim, "walker.pedestrian.0003",
                               carla.Location(a.x + (b.x - a.x) / length * 1.5, a.y + (b.y - a.y) / length * 1.5, a.z))
        sim.run(flow_module.CYCLE_HOLD_MAX - 1.0)
        check(front.state == "leaving" and front.rolled == 0.0, f"a leaving bicycle moved into a pedestrian: {front.state}")
        sim.run(2.5)
        check(front.state == "parked", f"a bicycle held up for {front.held:.1f} s is still {front.state}")
        spot = front.actor.transform.location
        sim.run(1.0)
        check(flow_module.dist2d(front.actor.transform.location, spot) < 1e-6, "a parked bicycle glided into the pedestrian")
        remove_obstacle(blocker)  # the line is clear again: the bicycle goes on to the exit spot
        check(run_until(sim, lambda: flow_module.dist2d(front.actor.transform.location, b) < 1e-3, length / 2.0 + 5.0),
              "a bicycle that gave up its leave did not go on once its line was clear")
        check(front.state == "parked" and front in bikes.bikes, f"the bicycle is {front.state} at the exit spot")
        sim.ctx.is_hidden = lambda ego, location: True  # out of sight: the parked bicycle is removed
        sim.step()
        check(front not in bikes.bikes and not front.actor.is_alive, "a parked bicycle was not removed once hidden")
    finally:
        sim.close()


class BikeRemovals:
    """Patches CyclistFlow._remove to record (bike, seconds since its state began, hidden at that moment, state)."""

    def __init__(self, sim):
        self.sim, self.records = sim, []

    def __enter__(self):
        sim, records = self.sim, self.records
        self.original = flow_module.CyclistFlow._remove

        def watch(flow, bike):
            records.append((bike, sim.t - bike.since, sim.ctx.is_hidden(sim.ego.transform, sim.ctx.location_of(bike.actor)),
                            bike.state))
            return self.original(flow, bike)
        flow_module.CyclistFlow._remove = watch
        return self

    def __exit__(self, *exc):
        flow_module.CyclistFlow._remove = self.original


def check_cyclists_without_exit(module, cmap):
    """A lane without a far sidewalk spot keeps the old rule: removed once hidden, or after 5 s at the far side."""
    print("flows: without a far sidewalk spot the bicycle goes after 5 s at the far side, at the latest")
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        for lane in bikes.lanes:
            lane.exit = None
        states = set()
        with BikeRemovals(sim) as removals:
            sim.run(80.0, lambda s_: states.update(b.state for b in bikes.bikes))
        check(not states & {"leaving", "parked"}, f"a bicycle left to the sidewalk without an exit spot: {states}")
        check("lingering" in states, "no bicycle stopped at the far side")
        check(len(removals.records) >= 2, f"{len(removals.records)} bicycles removed without exit spots")
        for bike, lingered, hidden, state in removals.records:
            check(state == "lingering" and (hidden or lingered >= flow_module.CYCLE_LINGER_CAP - sim.dt - 1e-9),
                  f"a bicycle without an exit spot was removed in view after {lingered:.1f} s ({state})")
    finally:
        sim.close()


def check_ego_gone(module, cmap):
    print("manual run: a dead ego ends the run, the stop file brakes it")
    stop_file = Path(tempfile.gettempdir()) / "carsim_check_flows_stop_file2"
    stop_file.unlink(missing_ok=True)
    # The ego is gone: the run ends after a few ticks, with a log line
    sim = Sim(module, cmap, PARAMS, stop_file=stop_file)
    try:
        tree = sim.scenario.behavior_tree
        names = [n.name for n in tree.iterate()]
        check("EgoGone" in names and "StopFileExists" in names, f"end conditions of a manual run: {names}")
        out = io.StringIO()
        with redirect_stdout(out):
            sim.run(3.0)
            check(tree.status == py_trees.common.Status.RUNNING, "the run ended with a living ego")
            sim.ego.is_alive = False
            for _ in range(flow_module_ego_ticks() - 1):
                sim.step()
            check(tree.status == py_trees.common.Status.RUNNING, "the run ended after one tick without the ego")
            sim.step()
        check(tree.status == py_trees.common.Status.SUCCESS, f"the tree is {tree.status} after the ego was destroyed")
        check("the ego vehicle is gone" in out.getvalue(), "no log line when the ego is gone")
        check(sim.ego.last_control is None, "a dead ego was braked")
    finally:
        sim.close()
    # An ego that comes back alive in time (one odd answer) does not end the run
    sim = Sim(module, cmap, PARAMS)
    try:
        tree = sim.scenario.behavior_tree
        sim.run(1.0)
        sim.ego.is_alive = False
        sim.step()
        sim.step()
        sim.ego.is_alive = True
        sim.run(2.0)
        check(tree.status == py_trees.common.Status.RUNNING, "two ticks without the ego ended the run")
    finally:
        sim.close()


def check_ego_gone_before_start(module, cmap):
    """A manual run without a flow still has the start trigger: the ego vanishing ends the run before it fires."""
    print("manual run without a flow: a destroyed, stationary ego ends the run before the start trigger fires")
    stop_file = Path(tempfile.gettempdir()) / "carsim_check_flows_stop_file3"
    stop_file.unlink(missing_ok=True)
    params = {"ego_control": {"mode": "manual"}, "pedestrian": {}, "crowd": {"count": "2"}}
    sim = Sim(module, cmap, params, nav=False, stop_file=stop_file)
    try:
        tree = sim.scenario.behavior_tree
        names = [n.name for n in tree.iterate()]
        check("TimeToArrival" in names and "EgoGone" in names, f"the start trigger of a run without a flow: {names}")
        sim.hold_until = 1e12  # the ego never moves: the trigger never fires
        out = io.StringIO()
        with redirect_stdout(out):
            sim.run(3.0)
            check(tree.status == py_trees.common.Status.RUNNING, f"the run ended with a living ego: {tree.status}")
            sim.ego.is_alive = False
            for _ in range(flow_module_ego_ticks() - 1):
                sim.step()
            check(tree.status == py_trees.common.Status.RUNNING, "the run ended after one tick without the ego")
            sim.run(1.0)
        main = next(n for n in tree.iterate() if n.name == "Main")  # the cleanup behind it (RemovePedestrian) takes a while
        check(main.status == py_trees.common.Status.SUCCESS,
              f"the main part is {main.status} after the stationary ego was destroyed")
        check("the ego vehicle is gone" in out.getvalue(), "no log line when the ego is gone")
    finally:
        sim.close()
    # Without a stop file (no driving window) the destroyed ego ends the wait as well
    sim = Sim(module, cmap, params, nav=False)
    try:
        tree = sim.scenario.behavior_tree
        sim.hold_until = 1e12
        sim.run(1.0)
        sim.ego.is_alive = False
        with redirect_stdout(io.StringIO()):
            sim.run(1.0)
        main = next(n for n in tree.iterate() if n.name == "Main")
        check(main.status == py_trees.common.Status.SUCCESS,
              f"no stop file: the main part is {main.status} after the ego was destroyed")
    finally:
        sim.close()


def check_ending_marker(module, cmap):
    """terminate() creates the marker file next to the stop file, and survives one that cannot be written."""
    print("manual run: terminate() leaves the ending marker next to the stop file")
    stop_file = Path(tempfile.gettempdir()) / "carsim_check_flows_stop_file4"
    marker = Path(ending_marker(stop_file))
    stop_file.unlink(missing_ok=True)
    marker.unlink(missing_ok=True)
    check(str(marker) == str(stop_file) + ".ending", f"the marker path is {marker}")
    sim = Sim(module, cmap, PARAMS, stop_file=stop_file)
    try:
        sim.run(1.0)
        check(not marker.exists(), "the marker exists before the run ends")
        out = io.StringIO()
        with redirect_stdout(out):
            sim.scenario.terminate()
        check(marker.exists(), "terminate() did not create the ending marker")
        check("ComposedScenario: scenario ending" in out.getvalue(), "terminate() did not print the ending line")
    finally:
        sim.close()
        marker.unlink(missing_ok=True)
    # A marker that cannot be written (its folder does not exist) must not break the end of the run
    missing = Path(tempfile.gettempdir()) / "carsim_check_no_such_folder" / "stop"
    sim = Sim(module, cmap, PARAMS, stop_file=missing)
    try:
        out = io.StringIO()
        with redirect_stdout(out):
            sim.scenario.terminate()
        check("ComposedScenario: scenario ending" in out.getvalue() and not Path(ending_marker(missing)).exists(),
              "terminate() failed when the marker could not be written")
    finally:
        sim.close()
    # No stop file at all: nothing is written
    sim = Sim(module, cmap, PARAMS)
    try:
        with redirect_stdout(io.StringIO()):
            sim.scenario.terminate()
    finally:
        sim.close()


class FakeProcess:
    """A driving window process: still running for `alive` polls, then exited."""

    def __init__(self, alive=0):
        self.alive = alive

    def poll(self):
        if self.alive > 0:
            self.alive -= 1
            return None
        return 0


def check_driving_window():
    """A window that exits while the scenario's ending marker exists is not the user closing it (apps/run_scenario.py)."""
    print("driving window: the ending marker keeps the watcher from writing the stop file")
    spec_ = importlib.util.spec_from_file_location("run_scenario_app", ROOT / "apps" / "run_scenario.py")
    app = importlib.util.module_from_spec(spec_)
    spec_.loader.exec_module(app)
    folder = Path(tempfile.mkdtemp(prefix="carsim_check_window_"))
    try:
        def window(name):
            stop = folder / name
            win = app.DrivingWindow("127.0.0.1", 2000, stop)
            win.POLL_SECONDS = win.ENDING_POLL = 0.01
            win.ENDING_WAIT = 0.4
            return win, stop, Path(ending_marker(stop))
        # 1. the window is closed by the user: the stop file is written
        win, stop, marker = window("one")
        with redirect_stdout(io.StringIO()):
            win._watch(FakeProcess(alive=3))
        check(stop.exists() and win.stop_file_written and win.window_closed, "a closed window did not end the run")
        # 2. the run ended by itself and the marker is there before the window exits (the output was not read yet)
        win, stop, marker = window("two")
        marker.touch()
        with redirect_stdout(io.StringIO()):
            win._watch(FakeProcess(alive=0))
        check(not stop.exists() and not win.stop_file_written and not win.window_closed,
              "the watcher wrote the stop file for a run that was ending by itself")
        # 2b. the marker appears just after the window exited (the ego went first): the wait for it catches that
        win, stop, marker = window("two_b")
        timer = threading.Timer(0.1, marker.touch)
        timer.start()
        with redirect_stdout(io.StringIO()):
            win._watch(FakeProcess(alive=0))
        timer.join()
        check(not stop.exists() and not win.window_closed, "a marker written just after the window exited was missed")
        # 3. the marker appears while the window is still open: the watcher stops watching
        win, stop, marker = window("three")
        timer = threading.Timer(0.1, marker.touch)
        timer.start()
        with redirect_stdout(io.StringIO()):
            win._watch(FakeProcess(alive=10 ** 9))
        timer.join()
        check(marker.exists() and win._ending.is_set() and not stop.exists(), "the watcher ignored the ending marker")
        # 4. the output line was read (the flag of before) without any marker: still no stop file
        win, stop, marker = window("four")
        win.feed("ComposedScenario: scenario ending")
        with redirect_stdout(io.StringIO()):
            win._watch(FakeProcess(alive=2))
        check(not stop.exists(), "the ending line no longer keeps the watcher from writing the stop file")
        # 5. no stop file configured: there is no marker to look for, and nothing breaks
        win = app.DrivingWindow("127.0.0.1", 2000, None)
        win.POLL_SECONDS = win.ENDING_POLL = 0.01
        win.ENDING_WAIT = 0.1
        with redirect_stdout(io.StringIO()):
            win._watch(FakeProcess(alive=2))
        check(win.window_closed and not win.stop_file_written, "a window without a stop file")
    finally:
        for path in folder.iterdir():
            path.unlink()
        folder.rmdir()


def flow_module_ego_ticks():
    from carsim.blocks.run_control import EGO_GONE_TICKS
    return EGO_GONE_TICKS


def check_pause_when_far(module, cmap):
    """Far from the crossing (twice the far distance) the pedestrian and cyclist flows spawn and launch nobody."""
    print("flows: pedestrians and cyclists pause while the ego is far from the crossing")
    sim = Sim(module, cmap, PARAMS)
    try:
        ped, bikes, bubble = blocks_flows(sim)
        ctx = sim.ctx
        sim.hold_until = 1e12
        sim.run(3.0)
        limit = flow_module.FLOW_PAUSE_FACTOR * ctx.hide.far_distance
        sim.ego.transform = carla.Transform(carla.Location(ped.center.x + limit + 50.0, ped.center.y, ped.center.z),
                                            sim.ego.transform.rotation)
        spawned = (ped.spawned, bikes.spawned)
        moving = [w for w in ped.walkers if not w.pending]
        check(moving, "no pedestrian was on its way when the ego left")
        for bike in bikes.bikes:
            if bike.state == "waiting":
                bike.ready_at = sim.t
        sim.run(100.0)
        check((ped.spawned, bikes.spawned) == spawned, f"a flow spawned while the ego was far away: {spawned} -> "
                                                       f"{(ped.spawned, bikes.spawned)}")
        check(all(b.state == "waiting" for b in bikes.bikes), "a bicycle left the sidewalk while the ego was far away")
        check(bikes.bikes, "no bicycle waits")
        check(all(w not in ped.walkers for w in moving), "the pedestrians already on their way did not finish and leave")
        # The ego comes back: the cyclists go on
        sim.ego.transform = carla.Transform(carla.Location(ped.center.x + 40.0, ped.center.y, ped.center.z),
                                            sim.ego.transform.rotation)
        for bike in bikes.bikes:
            if bike.state == "waiting":
                bike.ready_at = sim.t
        check(run_until(sim, lambda: any(b.state != "waiting" for b in bikes.bikes), 15.0),
              "the cyclists did not go on when the ego came back")
    finally:
        sim.close()


def check_single_pass_ends(module, cmap):
    """WalkOn, WalkLine, RideOn and DestroyIfPresent, driven by hand in the fake world of a manual run (no flow blocks)."""
    print("single pass: walkers walk on, bicycles ride on, nothing vanishes in view")
    sim = Sim(module, cmap, {"ego_control": {"mode": "manual"}})
    try:
        ctx = sim.ctx
        points = flow_module.walk_on_points(ctx)
        check(points is not None and ctx.walk_on is points, "no walk-on points on a map with a navigation mesh")
        check(flow_module.walk_on_points(ctx) is points, "the walk-on points are made again")
        for far in (0, 1):
            pool = points.pools[far]
            check(len(pool) == flow_module.WALK_ON_POINTS, f"{len(pool)} walk-on points beyond end {far}")
            for point in pool:
                d = flow_module.dist2d(point, points.ends[far])
                out = ((point.x - points.ends[far].x) * points.outward[far][0]
                       + (point.y - points.ends[far].y) * points.outward[far][1])
                check(flow_module.FLOW_BEYOND[0] <= d <= flow_module.FLOW_BEYOND[1] and out > 0.0,
                      f"walk-on point {d:.1f} m from end {far}, {out:.1f} m outward")
        check(points.far(ctx.crossing.end) == 1 and points.far(ctx.crossing.spawn.location) == 0, "far end of the crosswalk")

        # --- a walker that has crossed: controller target 30 to 60 m beyond the far end, removed only once hidden
        end = ctx.crossing.end
        walker = sim.request_new_actor("walker.pedestrian.0001", carla.Transform(carla.Location(end.x, end.y, end.z + 1.0)),
                                       actor_category="pedestrian", tick=False)
        controller = flow_module.attach_walker_controller(ctx, walker)
        check(controller is not None and controller in ctx._controllers and not controller.started_tick,
              "the walker's controller is missing, unregistered or already started")
        ego = ctx.ego_transform()
        hidden_exists = any(ctx.is_hidden(ego, p) for p in points.pools[1])
        node = flow_module.WalkOn(ctx, walker, controller, 1.4, points, 1)
        tick(node)
        target = walker.destination
        check(target is not None and controller.started_tick is not None and walker.max_speed == 1.4,
              "the controller was not started, sent anywhere or given the speed")
        d = flow_module.dist2d(target, points.ends[1])
        check(flow_module.FLOW_BEYOND[0] <= d <= flow_module.FLOW_BEYOND[1], f"walk-on target {d:.1f} m beyond the far end")
        check(not hidden_exists or ctx.is_hidden(ego, target), "a point out of the ego's sight exists but the target is in view")
        statuses = set()
        sim.run(DURATION, lambda s_: statuses.add(tick(node)))
        check(statuses == {py_trees.common.Status.RUNNING}, f"WalkOn returned {statuses}: it must never end the scenario")
        removals = [e for e in sim.events if e.kind == "remove" and e.what is walker]
        check(len(removals) == 1 and removals[0].hidden, f"the walker was removed {len(removals)} times, hidden: "
              f"{[e.hidden for e in removals]}")
        check(not walker.is_alive and not controller.is_alive and walker.destroy_count == 1 and walker.id not in CarlaDataProvider._carla_actor_pool,
              "the walker or its controller is left after the removal")
        # The cleanup node of the scenario (ActorDestroy before) then has nothing to destroy
        cleanup = flow_module.DestroyIfPresent(walker, name="RemovePedestrian")
        check(cleanup.update() == py_trees.common.Status.SUCCESS and walker.destroy_count == 1,
              "the cleanup destroyed a removed pedestrian again")
        alive = sim.request_new_actor("walker.pedestrian.0002", carla.Transform(carla.Location(0, 0, 0)),
                                      actor_category="pedestrian", tick=False)
        cleanup = flow_module.DestroyIfPresent(alive, name="RemovePedestrian")
        check(cleanup.update() == py_trees.common.Status.SUCCESS and alive.destroy_count == 1,
              "the cleanup did not remove a pedestrian that is still there")

        # --- no navigation point beyond the far end: the walker stays, the controller is not started
        bare = flow_module.WalkOnPoints.__new__(flow_module.WalkOnPoints)
        bare.ctx, bare.ends, bare.outward, bare.pools = ctx, points.ends, points.outward, ([], [])
        stay = sim.request_new_actor("walker.pedestrian.0003", carla.Transform(carla.Location(end.x, end.y, end.z + 1.0)),
                                     actor_category="pedestrian", tick=False)
        stay_controller = flow_module.attach_walker_controller(ctx, stay)
        node = flow_module.WalkOn(ctx, stay, stay_controller, 1.4, bare, 1)
        tick(node)
        tick(node)
        check(node.gone and stay.is_alive and stay_controller.started_tick is None and stay.destination is None,
              "a pedestrian with nowhere to walk on was started or removed")

        # --- ended off the sidewalk (a median on a deformed map): no AI controller (that crashed the 0.9.15 server); it
        # walks straight on until off the driving lanes and CLEAR_AFTER m further, then stands (not a crosser for ever)
        median = sim.request_new_actor("walker.pedestrian.0005", carla.Transform(carla.Location(end.x, end.y, end.z + 1.0)),
                                       actor_category="pedestrian", tick=False)
        median_controller = flow_module.attach_walker_controller(ctx, median)
        controls = []
        median.apply_control = controls.append
        node = flow_module.WalkOn(ctx, median, median_controller, 1.4, points, 1)
        on_road = [True]
        lanes = lambda loc, project_to_road=True, lane_type=None: object() if on_road[0] else None
        with mock.patch.object(ctx, "on_sidewalk", lambda loc: False), mock.patch.object(ctx.map, "get_waypoint", lanes):
            tick(node)
            check(median_controller.started_tick is None and controls and abs(controls[-1].speed - 1.4) < 1e-3 and not node.gone,
                  f"a pedestrian that ended off the sidewalk must walk on without its AI controller: started "
                  f"{median_controller.started_tick}, controls {[getattr(c, 'speed', c) for c in controls]}, gone {node.gone}, "
                  f"clearing {node.clearing}")
            on_road[0] = False
            t = median.get_transform()
            median.transform = carla.Transform(carla.Location(t.location.x + flow_module.CLEAR_AFTER + 0.5, t.location.y,
                                                              t.location.z), t.rotation)
            tick(node)
            check(node.gone and controls[-1].speed == 0.0 and median_controller.started_tick is None and median.is_alive,
                  "off the road and far enough it must stand, without a controller, and stay")
        ctx.remove_actor(median, median_controller)  # out of the way of the bicycles below

        # --- a walker knocked off its line ends its walk across, one on its line does not
        a, b = carla.Location(0, 0, 0), carla.Location(0, 20, 0)
        runner = sim.request_new_actor("walker.pedestrian.0004", carla.Transform(carla.Location(0, 5, 0)),
                                       actor_category="pedestrian", tick=False)
        for line, status in ((None, py_trees.common.Status.RUNNING), ((a, b), py_trees.common.Status.RUNNING)):
            walk = flow_module.WalkLine(runner, 1.4, line=line, duration=100.0, distance=100.0, name="WalkAcross")
            runner.transform = carla.Transform(carla.Location(0, 5, 0))
            check(tick(walk) == status, "a walker on its line ended its walk")
        runner.transform = carla.Transform(carla.Location(flow_module.WALK_OFF_LINE + 0.3, 6, 0))
        check(tick(walk) == py_trees.common.Status.SUCCESS, "a walker knocked off its line goes on marching")
        walk = flow_module.WalkLine(runner, 1.4, duration=100.0, distance=100.0, name="WalkAcross")
        check(tick(walk) == py_trees.common.Status.RUNNING, "a walk across without a line ended off the line")

        # --- a bicycle that has ridden across (each lane): rolls on to the far sidewalk without a stop, keeps riding along it
        # (physics off, never parked in view), is removed only once hidden
        for spot in cyclist_spawns(cmap, ctx.crossing, 2):
            exit_spot = cyclist_exit_spot(cmap, spot)
            check(exit_spot is not None, "no far sidewalk spot for the test lane")
            lane = flow_module.make_lane(spot, [], 4.0, DT, exit=exit_spot)
            helper = flow_module.CyclistFlow(ctx, [], 0, [])
            bike = sim.request_new_actor(next(iter(cb.BICYCLES)), carla.Transform(carla.Location(
                spot.end.x, spot.end.y, spot.end.z + 0.2), spot.spawn.rotation), actor_category="bicycle", tick=False)
            bike.velocity = (math_cos(spot) * 4.0, math_sin(spot) * 4.0)
            record = flow_module._Bike(bike, lane, 0)
            helper.bikes.append(record)
            ride_on = flow_module.RideOn(helper, record)
            sim.s, sim.hold_until = sim.crossing_s - 20.0, None  # the ego comes back to the crosswalk and waits there with the bicycle in view
            sim.ego.transform = carla.Transform(sim.route[int(sim.s / 2.0)].transform.location, sim.ego.transform.rotation)
            states, track = [], []
            check(len(ride_on.route) >= 2 and record.glide == 4.0, f"RideOn has no sidewalk run: {len(ride_on.route)} points")

            def watch(s_):
                check(tick(ride_on) == py_trees.common.Status.RUNNING, "RideOn ended")
                if not states or states[-1] != record.state:
                    states.append(record.state)
                if bike.is_alive:
                    track.append((s_.t, bike.transform.location))
            sim.run(DURATION, watch)
            check(states == ["leaving", "running"], f"RideOn states {states}: no stop, never parked in view")
            check(not bike.physics, "the bicycle rolled on with its physics on")
            # It keeps moving until it is hidden: no second without a move of at least 1 m, in view; it stays off the driving lanes
            for i in range(len(track) - 20):  # the way travelled in each second (it turns back at the end of its run)
                way = sum(flow_module.dist2d(track[j][1], track[j + 1][1]) for j in range(i, i + 20))
                check(way > 1.0, f"the bicycle stood still from t={track[i][0]:.1f} s")
            walked = sum(flow_module.dist2d(a[1], b[1]) for a, b in zip(track, track[1:]))
            check(walked > 8.0, f"the bicycle rode only {walked:.1f} m after its ride")
            for point in ride_on.route:
                check(cmap.get_waypoint(point, project_to_road=False, lane_type=carla.LaneType.Driving) is None,
                      f"a run point {point} is on a driving lane")
                check(all(flow_module.dist2d(point, wp.transform.location) > 1.5 for wp in sim.route),
                      f"a run point {point} is on the ego's route")
            removals = [e for e in sim.events if e.kind == "remove" and e.what is bike]
            check(len(removals) == 1 and removals[0].hidden and bike.destroy_count == 1,
                  f"the bicycle was removed {len(removals)} times, hidden: {[e.hidden for e in removals]}")
            check(record not in helper.bikes and bike.id not in CarlaDataProvider._carla_actor_pool,
                  "the removed bicycle is still tracked")


        # --- a lane without a far sidewalk spot keeps the old end: stops, lingers, removed once hidden or at the cap
        bare_lane = flow_module.make_lane(spot, [], 4.0, DT, exit=None)
        stop_bike = sim.request_new_actor(next(iter(cb.BICYCLES)), carla.Transform(carla.Location(
            spot.end.x, spot.end.y, spot.end.z + 0.2), spot.spawn.rotation), actor_category="bicycle", tick=False)
        sim.s, sim.hold_until = sim.crossing_s - 20.0, None
        stop_bike.velocity = (math_cos(spot) * 4.0, math_sin(spot) * 4.0)
        stop_record = flow_module._Bike(stop_bike, bare_lane, 0)
        helper.bikes.append(stop_record)
        stopper = flow_module.RideOn(helper, stop_record)
        seen = []
        sim.run(DURATION, lambda s_: (tick(stopper), seen.append(stop_record.state)))
        check(seen[0] == "stopping" and "lingering" in seen and not stop_bike.is_alive,
              f"a lane without a far sidewalk spot: states {sorted(set(seen))}, alive {stop_bike.is_alive}")
    finally:
        sim.close()


def tick(node):
    node.tick_once()
    return node.status


def math_cos(spot):
    return math.cos(math.radians(spot.spawn.rotation.yaw))


def math_sin(spot):
    return math.sin(math.radians(spot.spawn.rotation.yaw))


def check_sidewalk_corner(cmap):
    """
    The occluded_crossing exits at (95.6, 4.0) and (114.2, 4.3) on Town10HD (live): the west one is 4.6 m from a
    junction corner's sidewalk lane that ends after 10 m; the run must bridge onto the next sidewalk, not stay short (a
    bicycle that crossed paced back and forth, or parked in view, there).
    """
    from carsim.blocks.flow import RUN_MIN, RUN_STEP, sidewalk_run
    print("sidewalk run round a junction corner")
    ctx = type("Ctx", (), {"map": cmap})()
    for name, start in (("west", carla.Location(95.6, 4.0, 0.0)), ("east", carla.Location(114.2, 4.3, 0.0))):
        run = sidewalk_run(ctx, start, (0.0, 1.0))
        check(len(run) * RUN_STEP >= RUN_MIN, f"{name} exit: the run is only {len(run) * RUN_STEP:.0f} m")
        on_road = [p for p in run if cmap.get_waypoint(p, project_to_road=False, lane_type=carla.LaneType.Driving)]
        check(not on_road, f"{name} exit: {len(on_road)} run points over a driving lane")


def main():
    module = cb.load_scenario_module(COMPOSED_SCENARIO_FILE)
    cmap = carla.Map("Town10HD", cb.xodr_path("Town10HD").read_text(encoding="utf-8"))
    check_manual_flows(module, cmap)
    check_no_navigation(module, cmap)
    check_single_pass_ends(module, cmap)
    check_sidewalk_corner(cmap)
    check_hide_rule_config(module, cmap)
    check_timeout(module, cmap)
    check_reversing(module, cmap)
    check_pending_removals(module, cmap)
    check_background_autopilot(module, cmap)
    check_initial_population(module, cmap)
    check_flow_errors(module, cmap)
    check_controller_failures(module, cmap)
    check_refill_cooldown(module, cmap)
    check_finished_walkers(module, cmap)
    check_cyclist_blocking(module, cmap)
    check_cyclist_retry(module, cmap)
    check_cyclist_retry_hidden(module, cmap)
    check_cyclist_lane_jam(module, cmap)
    check_cyclists_without_exit(module, cmap)
    check_ego_gone(module, cmap)
    check_ego_gone_before_start(module, cmap)
    check_ending_marker(module, cmap)
    check_driving_window()
    check_pause_when_far(module, cmap)
    print("FAILED" if cb.FAILURES else "PASSED")
    sys.exit(1 if cb.FAILURES else 0)


if __name__ == "__main__":
    main()
