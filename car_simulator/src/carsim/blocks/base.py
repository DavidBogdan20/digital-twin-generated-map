"""
The Block interface and the context every block of a composed scenario shares.

A block is a small class keyed by its XML tag. It reads its parameters (strings) from
`config.other_parameters[tag]` and takes part in the scenario through a few hooks, all optional:

    trigger_distance()   its say in the shared trigger distance (only blocks that cross the road)
    timeout()            its say in the scenario timeout
    spawn()              create actors, during BasicScenario._initialize_actors()
    after_tick()         extra setup that needs the world ticked once after all spawns
    from_start()         py_trees behaviours that run from the first tick (also the endless flows of a manual run)
    on_trigger()         py_trees behaviours that run once the ego is within the trigger distance of the crossing
    end_conditions()     py_trees conditions that end the scenario when one succeeds
    cleanup_behaviors()  py_trees behaviours that run after the scenario has ended
    cleanup()            release what is not an actor of the scenario (AI controllers); safe to call repeatedly

ComposedScenario (scenarios/_composed/composed_scenario.py) runs the hooks in the fixed order of
carsim.blocks.spec.BLOCK_ORDER.
"""

import math
import random

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.timer import GameTime

NAVIGATION_SIDEWALK_TRIES = 20  # navigation points drawn per navigation_point() call on a deformed map
SURFACE_RETRY_LIFTS = (0.0, 0.3, 0.8)  # [m] extra height for a spawn refused on a deformed map (request_on_surface)


def raised(spot, dz):
    """A Transform or Location dz m higher."""
    loc = spot.location if isinstance(spot, carla.Transform) else spot
    moved = carla.Location(loc.x, loc.y, loc.z + dz)
    return carla.Transform(moved, spot.rotation) if isinstance(spot, carla.Transform) else moved


from carsim.road import CrosswalkIndex, HideRule, crosswalks, deformed_map, find_crossing, straight_route, surface_offset

NAVIGATION_PROBES = 5  # navigation calls tried to find out whether the map has a navigation mesh
HALF_LENGTH_DEFAULTS = {"vehicle": 2.0, "walker": 0.3}  # half the length of an actor without a bounding box [m]
REMOVAL_RETRIES = 5  # an actor the server would not destroy is tried again this many times, then left to the final cleanup
REMOVAL_RETRY_PERIOD = 1.0  # ... at most this often [s]
SPAWN_ABOVE = 0.5  # on a deformed map a map spawn point is put this far over the road mesh (ctx.surface) [m]


def vehicle_ids(library, base_type=None, wheels=None, lit=False):
    """
    Ids of the vehicle blueprints with the given `base_type` attribute and/or number of wheels; with `lit`, only those
    whose model has lights (`has_lights`: no motorcycle or bicycle in CARLA 0.9.16 has, nor 9 of its 23 cars).
    """
    ids = []
    for bp in library.filter("vehicle.*"):
        if lit and bp.has_attribute("has_lights") and not bp.get_attribute("has_lights").as_bool():
            continue
        if base_type is not None and (not bp.has_attribute("base_type")
                                      or bp.get_attribute("base_type").as_str() != base_type):
            continue
        if wheels is not None and (not bp.has_attribute("number_of_wheels")
                                   or bp.get_attribute("number_of_wheels").as_int() != wheels):
            continue
        ids.append(bp.id)
    return ids


def walker_ids(library):
    return [bp.id for bp in library.filter("walker.pedestrian.*")]


class BlockContext:
    """
    What the blocks of one scenario share.

    world, map, ego, ego_start_wp   the CARLA world, its map, the ego vehicle and the waypoint it starts at
    rng                  random.Random seeded from CarlaDataProvider's generator on first use, so runs repeat
    crossing             the Crossing ahead of the ego (see carsim.road.find_crossing), computed once on first
                         use; raises ValueError when there is none. Only blocks that need it touch it
    trigger_location     the point where the ego lane meets the crossing's walk line
    trigger_distance     the ego-to-trigger_location distance [m] at which the triggered behaviours start
    uses_crossing        True when some block of the scenario crosses the road (set by the scenario)
    after_crossing       how far past the crossing the ego drives before the scenario ends [m] (set by `end`)
    distance             how far the ego drives when there is no crossing [m] (set by `end`)
    endless              True for a manual run (set by `ego_control`): no distance end and no time limit; the run ends
                         with the stop file of <run_control> or Ctrl-C, and the blocks keep their actors flowing
    hide                 the HideRule (see carsim.road.hidden_from_ego) every spawn and removal during the run follows;
                         the <spawning> block changes it
    stop_file            path of the file whose existence ends the run (set by `run_control`), or None
    flow_active          True when a block made an endless flow (the scenario then needs no start trigger). It must be
                         set in spawn(): BasicScenario decides about the trigger before the behaviours are created
    walker_flow          the one PedestrianFlow of the scenario, made by the block that claims it in after_tick()
    walker_flow_*        what the pedestrian and crowd blocks ask of the one shared pedestrian flow: slots (how many
                         pedestrians), radius (m around the crossing, None: the default) and speed (min, max) [m/s]
    pending removals     actors the server would not destroy (remove_actor): retry_removals() tries them again, and
                         pending_removals(group) tells a flow how many of its own are still alive, to count against
                         its capacity
    """

    DEFAULT_TRIGGER_DISTANCE = 25.0  # [m]

    def __init__(self, world, cmap, ego, ego_start_wp, scenario, debug=False):
        self.world = world
        self.map = cmap
        self.ego = ego
        self.ego_start_wp = ego_start_wp
        self.scenario = scenario  # its other_actors list takes the spawned actors
        self.debug = debug
        self.trigger_distance = self.DEFAULT_TRIGGER_DISTANCE
        self.uses_crossing = False
        self.after_crossing = 30.0
        self.distance = 100.0
        self.endless = False
        self.hide = HideRule()
        self.stop_file = None
        self.flow_active = False
        self.walker_flow = None
        self.walker_flow_slots = 0
        self.walker_flow_radius = None
        self.walker_flow_speed = None
        self._walker_flow_claimed = False
        self._navigation = None
        self._library = None
        self._walker_models = None
        self._speed_at = None
        self._forward_speed = None
        self._rng = None
        self._deformed = None
        self._crossing = None
        self._plan = None
        self._controllers = []
        self._pending = []  # [actor, group, tries] of the removals the server refused
        self._retry_at = 0.0
        self.target = None  # the actor the test is about (child, cut-in car, cyclist), set by its block in spawn()
        self.metrics = {}  # events for the metrics block: the driver's first sighting, brake onset, ...

    # --- shared state -------------------------------------------------------

    @property
    def rng(self):
        if self._rng is None:
            # Seeded from ScenarioRunner's generator (a numpy RandomState) so runs with the same seed repeat
            self._rng = random.Random(int(CarlaDataProvider.get_random_seed().randint(0, 2 ** 31 - 1)))
        return self._rng

    @property
    def library(self):
        """The world's blueprint library, fetched from the server once."""
        if self._library is None:
            self._library = self.world.get_blueprint_library()
        return self._library

    @property
    def crossing(self):
        if self._crossing is None:
            crossing = find_crossing(self.map, self.ego_start_wp, CrosswalkIndex(crosswalks(self.map)))
            if crossing is None:
                raise ValueError("No junction ahead of the ego vehicle. "
                                 "Pick another start with apps/find_crossing_spots.py")
            if crossing.distance <= 0.0:
                raise ValueError("The crossing is {:.1f} m behind the ego vehicle. "
                                 "Pick another start with apps/find_crossing_spots.py".format(-crossing.distance))
            if crossing.distance < self.trigger_distance:
                self.log("crossing", "WARNING the crossing is only {:.1f} m ahead, closer than trigger_distance "
                         "{:g} m, so the crossing starts immediately".format(crossing.distance, self.trigger_distance))
            if crossing.crosswalk is None:
                self.log("crossing", "no painted crosswalk found, crossing at the junction entry")
            self._crossing = crossing
        return self._crossing

    @property
    def trigger_location(self):
        return self.crossing.wp.transform.location

    @property
    def center(self):
        """Middle of the scene: the crossing if the scenario has one, else the ego start."""
        if self.uses_crossing:
            return self.trigger_location
        return self.ego_start_wp.transform.location

    def total_drive_distance(self):
        """How far the ego drives before the scenario ends, from its start [m]."""
        if self.uses_crossing:
            return self.crossing.distance + self.after_crossing
        return self.distance

    def ego_plan(self):
        """The ego's route: straight on through the junction, a bit longer than the end condition."""
        if self._plan is None:
            from agents.navigation.local_planner import RoadOption
            route = straight_route(self.ego_start_wp, self.total_drive_distance() + 20.0)
            self._plan = [(wp, RoadOption.LANEFOLLOW) for wp in route]
        return self._plan

    def route_lanes(self):
        """(road_id, lane_id) of every lane the ego's route or start uses."""
        lanes = {(wp.road_id, wp.lane_id) for wp, _ in self.ego_plan()}
        lanes.add((self.ego_start_wp.road_id, self.ego_start_wp.lane_id))
        return lanes

    # --- the ground ---------------------------------------------------------

    @property
    def deformed(self):
        """True on a map whose road mesh waves around the OpenDRIVE heights (carsim.road.deformed_map)."""
        if self._deformed is None:
            self._deformed = deformed_map(getattr(self.map, "name", "") or "")
        return self._deformed

    def surface(self, transform, above=None):
        """
        `transform` (a Transform or Location) as high over the ground as its z is over the OpenDRIVE height, on a
        deformed map (carsim.road.surface_offset; unchanged elsewhere). Every spawn and set_transform of the blocks goes
        through this, never the stored geometry, so nothing is shifted twice. With `above`, its own z (a map spawn
        point's) is replaced by the driving lane's height under it plus `above` first. Navigation points need none.
        """
        if not self.deformed:
            return transform
        is_transform = isinstance(transform, carla.Transform)
        loc = transform.location if is_transform else transform
        z = loc.z
        if above is not None:
            z = self.map.get_waypoint(loc).transform.location.z + above
        moved = carla.Location(loc.x, loc.y, z + surface_offset(self.map, loc))
        return carla.Transform(moved, transform.rotation) if is_transform else moved

    def request_on_surface(self, model, transform, above=None, **kwargs):
        """
        CarlaDataProvider.request_new_actor at surface(transform), tried SURFACE_RETRY_LIFTS m higher when it is refused.
        On a deformed map the computed surface leaves about 0.2 m (request_new_actor's own lift), and a big vehicle's
        collision box reaches lower than that: the european_hgv was refused at 0.2 m and spawned at 0.5 m over the road
        on WolfsburgHighway. The blocks set their vehicles down onto the surface afterwards; the rest drop the few cm.
        """
        for lift in SURFACE_RETRY_LIFTS if self.deformed else (0.0,):
            actor = CarlaDataProvider.request_new_actor(model, raised(self.surface(transform, above), lift), **kwargs)
            if actor is not None:
                return actor
        return None

    # --- registration -------------------------------------------------------

    def add_actor(self, actor):
        """Register a spawned actor with the scenario, which removes it when it ends."""
        self.scenario.other_actors.append(actor)

    @property
    def night(self):
        """True when the weather's sun is below the horizon (ScenarioRunner sets the weather before the actors)."""
        get_weather = getattr(self.world, "get_weather", None)  # the offline checks' fake world has none: day
        return get_weather is not None and get_weather().sun_altitude_angle < 0.0

    def lights_on(self, vehicle):
        """
        Position lights and low beams on `vehicle` at night: CARLA never switches a vehicle's lights on by itself. For
        vehicles the scenario drives or keeps waiting with the engine on; the Traffic Manager handles its own
        (update_vehicle_lights). A parked vehicle stays dark.
        """
        if not self.night:
            return
        try:
            vehicle.set_light_state(carla.VehicleLightState(
                carla.VehicleLightState.Position | carla.VehicleLightState.LowBeam))
        except (AttributeError, RuntimeError):
            pass  # a bicycle or a fake actor in the offline checks: no lights

    def add_controller(self, controller):
        """Register an AI controller; remove_controllers() destroys it before its walker goes."""
        self._controllers.append(controller)

    def remove_controllers(self):
        """Stop and destroy the AI controllers; ones already gone with their walker are skipped. Safe to repeat."""
        controllers, self._controllers = self._controllers, []
        for controller in controllers:
            try:
                if controller.is_alive:
                    controller.stop()
            except RuntimeError:
                pass
            try:
                if controller.is_alive:
                    controller.destroy()
            except RuntimeError:
                pass

    def forget_actor(self, actor):
        """Drop an actor that was removed during the run from the scenario's list, so the list does not grow."""
        self.scenario.other_actors[:] = [other for other in self.scenario.other_actors
                                         if other is not None and other.id != actor.id]

    def discard_controller(self, controller):
        """
        Stop tracking an AI controller that is being destroyed now. Only remove_actor() should call this: dropping a
        controller (or its walker) without destroying it leaves it on the server, so everything else uses remove_actor().
        """
        self._controllers = [other for other in self._controllers if other is not controller]

    def remove_actor(self, actor, controller=None, group=None):
        """
        Remove an actor mid-run, the way ScenarioRunner's ActorDestroy does: through CarlaDataProvider, so its pool
        stays consistent (the final cleanup then skips it). An AI controller goes first. An actor the server would not
        destroy is still alive and nobody tracks it any more: it goes on a list that retry_removals() works through,
        under `group` (whatever the caller counts its actors by), so the caller can count it, see pending_removals().
        """
        if controller is not None:
            self.discard_controller(controller)
            for action in ("stop", "destroy"):
                try:
                    if controller.is_alive:
                        getattr(controller, action)()
                except RuntimeError:
                    pass
        self.forget_actor(actor)
        if not self._destroy(actor):
            self._pending.append([actor, group, 0])

    def _destroy(self, actor):
        """
        Destroy `actor` through the pool (itself when it already left the pool). True when it is gone. An actor that
        is still alive is back in the pool afterwards: remove_actor_by_id drops it from there even when the server
        refused (destroy() returned False), and the pool is what the final cleanup and solid_things() look at.
        """
        pool = CarlaDataProvider._carla_actor_pool
        try:
            if actor.id in pool:
                CarlaDataProvider.remove_actor_by_id(actor.id)
            elif actor.is_alive:
                actor.destroy()
        except RuntimeError:
            pass
        try:
            gone = not actor.is_alive
        except RuntimeError:
            return True
        if not gone:
            pool[actor.id] = actor
        return gone

    def retry_removals(self):
        """
        Try again, at most every REMOVAL_RETRY_PERIOD, the actors the server would not destroy. One that still stays
        after REMOVAL_RETRIES tries is given up: it stays in the pool, so the final cleanup of ScenarioRunner destroys it.
        """
        now = GameTime.get_time()
        if not self._pending or now < self._retry_at:
            return
        self._retry_at = now + REMOVAL_RETRY_PERIOD
        pending, self._pending = self._pending, []
        for item in pending:
            actor, group, tries = item
            if self._destroy(actor):
                continue
            if tries + 1 >= REMOVAL_RETRIES:
                self.log("removal", "WARNING actor {} could not be destroyed, left to the final cleanup".format(
                    actor.id))
                continue
            item[2] = tries + 1
            self._pending.append(item)

    def pending_removals(self, group):
        """How many actors of `group` are still waiting to be destroyed (they count against the group's capacity)."""
        return sum(1 for _, other, _ in self._pending if other == group)

    # --- the ego and the hide rule -----------------------------------------

    def ego_transform(self):
        """Where the ego is now (ScenarioRunner's cached value, refreshed every tick; the server as a fallback)."""
        transform = CarlaDataProvider.get_transform(self.ego)
        return transform if transform is not None else self.ego.get_transform()

    def ego_forward_speed(self):
        """
        The ego's velocity along its heading [m/s], negative when it reverses; None when it cannot be read. Read from
        the server at most once per game time (the flows ask many times per tick).
        """
        now = GameTime.get_time()
        if self._speed_at != now:
            self._speed_at = now
            try:
                velocity = self.ego.get_velocity()
                yaw = math.radians(self.ego_transform().rotation.yaw)
                self._forward_speed = velocity.x * math.cos(yaw) + velocity.y * math.sin(yaw)
            except (AttributeError, RuntimeError):
                self._forward_speed = None
        return self._forward_speed

    def is_hidden(self, ego, location):
        """True when something may appear or vanish at `location`: the ego (a Transform) cannot see it (see `hide`)."""
        return self.hide.hidden(ego, location, self.ego_forward_speed())

    def hidden(self, location):
        """True when something may appear or vanish at `location` now: the ego cannot see it (see `hide`)."""
        return self.is_hidden(self.ego_transform(), location)

    def location_of(self, actor):
        """Where an actor registered with CarlaDataProvider is now (cached, refreshed every tick)."""
        location = CarlaDataProvider.get_location(actor)
        return location if location is not None else actor.get_location()

    def solid_things(self):
        """
        The vehicles and walkers around, for the flows that must not roll into them: the ego and every actor of
        CarlaDataProvider's pool, as a list of (actor, kind, location, half length) with kind "vehicle" or "walker".
        Read from the cache of the data provider, so it costs no server call.
        """
        things, seen = [], set()
        for actor in [self.ego] + list(CarlaDataProvider._carla_actor_pool.values()):
            if actor is None or actor.id in seen:
                continue
            seen.add(actor.id)
            try:
                if not actor.is_alive:
                    continue
                kind = "vehicle" if actor.type_id.startswith("vehicle.") else (
                    "walker" if actor.type_id.startswith("walker.") else None)
                if kind is None:
                    continue
                location = self.location_of(actor)
                try:
                    half = actor.bounding_box.extent.x
                except (AttributeError, RuntimeError):
                    half = HALF_LENGTH_DEFAULTS[kind]
            except RuntimeError:
                continue
            if location is not None:
                things.append((actor, kind, location, half))
        return things

    # --- walkers ------------------------------------------------------------

    def navigation_point(self):
        """
        world.get_random_location_from_navigation(), on a deformed map only a point on a Sidewalk lane (else None, as
        when there is no point), so wandering walkers keep off the carriageway. Every walker spawn and destination goes
        through here. (The FWalkerActor::SetWalkerState server crashes on WolfsburgHighway, 2026-11-10, were walkers put
        to sleep by a wrong large-map tile lookup at the map's north and east edges, fixed in the map: digital_twin
        GUIDE Stage 9, `vm.sh tileoffset`.)
        """
        for _ in range(NAVIGATION_SIDEWALK_TRIES if self.deformed else 1):
            point = self.world.get_random_location_from_navigation()
            if point is None or not self.deformed:
                return point
            if self.map.get_waypoint(point, project_to_road=False, lane_type=carla.LaneType.Sidewalk) is not None:
                return point
        return None

    def on_sidewalk(self, location):
        """True on a map that is not deformed, else whether `location` is on a Sidewalk lane (see navigation_point)."""
        return not self.deformed or self.map.get_waypoint(
            location, project_to_road=False, lane_type=carla.LaneType.Sidewalk) is not None

    def navigation_available(self):
        """False when the map has no navigation mesh (Town12): the first NAVIGATION_PROBES calls all gave no point."""
        if self._navigation is None:
            self._navigation = any(self.world.get_random_location_from_navigation() is not None
                                   for _ in range(NAVIGATION_PROBES))
        return self._navigation

    def spawn_walker(self, location):
        """
        A random pedestrian at the navigation point `location` with an AI controller attached (registered, destroyed
        by remove_controllers()), spawned without a tick of its own. Returns (walker, controller), or (None, None)
        when either could not be spawned. The controller must be started a tick later.
        """
        if self._walker_models is None:
            self._walker_models = walker_ids(self.library)
        walker = CarlaDataProvider.request_new_actor(
            self.rng.choice(self._walker_models),
            carla.Transform(carla.Location(location.x, location.y, location.z + 1.0)),
            actor_category="pedestrian", tick=False)
        if walker is None:
            return None, None
        try:
            controller = self.world.spawn_actor(self.library.find("controller.ai.walker"), carla.Transform(),
                                                attach_to=walker)
        except RuntimeError as error:
            self.log("walkers", "WARNING no AI controller for pedestrian {}: {}".format(walker.id, error))
            self.remove_actor(walker)
            return None, None
        self.add_controller(controller)
        return walker, controller

    def claim_walker_flow(self):
        """True once, for the first block that asks: that block adds the pedestrian flow all of them share."""
        if self._walker_flow_claimed:
            return False
        self._walker_flow_claimed = True
        return True

    # --- helpers ------------------------------------------------------------

    def time_step(self, default=1.0 / 20.0):
        """The world's fixed simulation step [s]; `default` (20 steps per second) in asynchronous mode."""
        try:
            step = self.world.get_settings().fixed_delta_seconds
        except (AttributeError, RuntimeError):
            step = None
        return step if step else default

    def traffic_manager(self):
        """The Traffic Manager ScenarioRunner uses (same port as the autopilot of the spawned vehicles), or None."""
        client = CarlaDataProvider.get_client()
        if client is None:
            return None
        try:
            return client.get_trafficmanager(CarlaDataProvider.get_traffic_manager_port())
        except RuntimeError:
            return None

    def tick_world(self):
        """Let the server process what was just spawned, the way BasicScenario does after _initialize_actors()."""
        if CarlaDataProvider.is_runtime_init_mode():
            self.world.wait_for_tick()
        elif CarlaDataProvider.is_sync_mode():
            self.world.tick()
        else:
            self.world.wait_for_tick()

    def log(self, who, message):
        print("ComposedScenario[{}]: {}".format(who, message))


class Block:
    """Base class of the blocks; every hook does nothing by default."""

    tag = None  # the XML element this block reads
    needs_crossing = False  # True for a block that crosses the road at the crosswalk

    def __init__(self, ctx, params):
        self.ctx = ctx
        self.params = dict(params)

    # --- parameters ---------------------------------------------------------

    def _number(self, name, default, kind=float):
        value = self.params.get(name)
        if value is None:
            return default
        try:
            return kind(float(value)) if kind is int else kind(value)
        except ValueError:
            raise ValueError("<{}> attribute {}=\"{}\" is not a number".format(self.tag, name, value)) from None

    def float_param(self, name, default):
        return self._number(name, default, float)

    def int_param(self, name, default):
        return self._number(name, default, int)

    def log(self, message):
        self.ctx.log(self.tag, message)

    # --- hooks --------------------------------------------------------------

    def trigger_distance(self):
        """The trigger distance this block asks for [m], or None."""
        return None

    def timeout(self):
        """The scenario timeout this block asks for [s], or None."""
        return None

    def spawn(self):
        """Create actors with CarlaDataProvider (tick=False for scenery) and register them with ctx.add_actor()."""

    def after_tick(self):
        """Runs once the world has ticked after every block has spawned."""

    def from_start(self):
        """Behaviours that run from the start; a list of py_trees nodes."""
        return []

    def on_trigger(self):
        """Behaviours that run once the shared trigger fires; they should end in a WaitForever."""
        return []

    def end_conditions(self):
        """Conditions that end the scenario when one of them succeeds."""
        return []

    def cleanup_behaviors(self):
        """Behaviours that run after the main part has ended."""
        return []

    def cleanup(self):
        """Release anything that is not removed with the scenario's actors. Called more than once."""
