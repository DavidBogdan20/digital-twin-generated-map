"""<background vehicles="10" motorcycles="3" bicycles="3" walkers="20" radius="120"/>"""

import math

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from carsim.blocks.base import SPAWN_ABOVE, Block, vehicle_ids, walker_ids
from carsim.blocks.flow import BICYCLE_SPEED_DIFFERENCE, WALKER_SPEED, BackgroundBubble

MIN_EGO_DISTANCE = 25.0  # background vehicles spawn at least this far from the ego start [m]
NAVIGATION_ATTEMPTS_PER_WALKER = 300  # random navigation points tried per wandering pedestrian before giving up
NAVIGATION_NONE_LIMIT = 5  # the first this many navigation calls without a point: the map has no navigation mesh


class Background(Block):

    """
    Scenery that only gets spawned: the Traffic Manager drives the vehicles, the AI walker controllers the
    pedestrians. Within `radius` m of the crossing (of the ego start when the scenario has no crossing block, or in an
    endless manual run):

    vehicles     4-wheel vehicles on autopilot at map spawn points
    motorcycles  motorcycles (blueprints with base_type motorcycle) on autopilot at the same kind of spawn points
    bicycles     bicycles (base_type bicycle) riding in traffic on autopilot at those spawn points too, at half
                 the speed limit (a Traffic Manager speed difference of 50%; the others drive at the limit)
    walkers      pedestrians at random navigation points that wander to random destinations

    Vehicle spawn points are at least 25 m from the ego start and never in a lane the ego's route or start uses
    (such a vehicle would drive slower than the ego or stop at the crowd, and the autopilot ego does not brake).
    Everything is spawned without a tick of its own, and the AI controllers are started after the scenario's one
    tick. The controllers are destroyed in cleanup(), before the walkers they are attached to.

    Endless (manual ego): the traffic follows the ego, in a bubble (see carsim.blocks.flow.BackgroundBubble). The
    first actors are placed at load around the ego start; then every second the ones farther than 1.3 x `radius` from
    the ego that it cannot see are removed, and replacements are spawned (at most 2 per second) within `radius` of the
    ego where it cannot see them, on free spawn points (no vehicle within 8 m) and at navigation points, to hold the
    configured counts.
    """

    tag = "background"

    def __init__(self, ctx, params):
        super().__init__(ctx, params)
        self.vehicles = self.int_param("vehicles", 0)
        self.motorcycles = self.int_param("motorcycles", 0)
        self.bicycles = self.int_param("bicycles", 0)
        self.walkers = self.int_param("walkers", 0)
        self.radius = self.float_param("radius", 120.0)  # around the crossing, or around the ego when endless [m]
        self._controllers = []
        self._tracked = {}  # label -> background vehicles placed at load (endless run)
        self._walker_pairs = []  # (walker, controller) placed at load (endless run)
        self._kinds = []  # (label, count, models, category), for the bubble
        self._manager = None
        self.bubble = None

    @property
    def follow(self):
        """True when the traffic follows the ego (endless run)."""
        return self.ctx.endless

    def spawn(self):
        # The scenario decides about its start trigger before the behaviours are made (BasicScenario.__init__ calls
        # _setup_scenario_trigger before _create_behavior), so an endless run with a bubble says so here, not in from_start()
        if self.follow and (self.vehicles > 0 or self.motorcycles > 0 or self.bicycles > 0
                            or (self.walkers > 0 and self.ctx.navigation_available())):
            self.ctx.flow_active = True
        if self.vehicles > 0 or self.motorcycles > 0 or self.bicycles > 0:
            self._spawn_traffic()
        self._spawn_walkers()

    def _center(self):
        return self.ctx.ego_start_wp.transform.location if self.follow else self.ctx.center

    def _spawn_points(self):
        """Map spawn points in range and off the ego's lanes, shuffled: (points, how many were on the route)."""
        ctx = self.ctx
        center, ego = self._center(), ctx.ego_start_wp.transform.location
        spots = [tf for tf in ctx.map.get_spawn_points()
                 if tf.location.distance(center) <= self.radius and tf.location.distance(ego) >= MIN_EGO_DISTANCE]
        route = ctx.route_lanes()
        free = []
        for tf in spots:
            wp = ctx.map.get_waypoint(tf.location)
            if (wp.road_id, wp.lane_id) not in route:
                free.append(tf)
        ctx.rng.shuffle(free)
        return free, len(spots) - len(free)

    def _spawn_traffic(self):
        ctx = self.ctx
        spots, on_route = self._spawn_points()
        available = len(spots)
        library = ctx.library
        manager = self.ctx.traffic_manager()
        self._manager = manager
        if manager is None:
            self.log("WARNING no Traffic Manager: background vehicles may change lanes and bicycles ride at the speed "
                     "limit")
        night = ctx.night  # at night only models with lights: no CARLA motorcycle has any, so none ride then
        if night and self.motorcycles > 0:
            self.log("night: no motorcycles (no CARLA motorcycle model has lights); cars only from models with lights")
        kinds = (("cars", self.vehicles, vehicle_ids(library, wheels=4, lit=night), "car"),
                 ("motorcycles", 0 if night else self.motorcycles, vehicle_ids(library, base_type="motorcycle"),
                  "motorbike"),
                 ("bicycles", self.bicycles, vehicle_ids(library, base_type="bicycle"), "bicycle"))
        self._kinds = list(kinds)
        for label, count, models, category in kinds:
            if count <= 0:
                continue
            if not models:
                self.log("WARNING no {} blueprints: none spawned".format(label))
                continue
            spawned = 0
            # Every spawn point is used once, whether the spawn worked or not
            while spawned < count and spots:
                vehicle = ctx.request_on_surface(ctx.rng.choice(models), spots.pop(), above=SPAWN_ABOVE,
                                                              autopilot=True, actor_category=category, tick=False)
                if vehicle is not None:
                    ctx.add_actor(vehicle)
                    self._tracked.setdefault(label, []).append(vehicle)
                    spawned += 1
                    if manager is not None:
                        # never into the ego's lane ahead of it: a lane change there is a cut-in the scenario did not ask for
                        manager.auto_lane_change(vehicle, False)
                        manager.update_vehicle_lights(vehicle, True)  # headlights at night, brake lights, indicators
                        if category == "bicycle":
                            manager.vehicle_percentage_speed_difference(vehicle, BICYCLE_SPEED_DIFFERENCE)
            self.log("spawned {}/{} background {} ({} spawn points within {:g} m, {} more dropped as on the ego's "
                     "route)".format(spawned, count, label, available, self.radius, on_route))

    def _spawn_walkers(self):
        """Pedestrians at random navigation points around the crossing, each walking to another random point."""
        if self.walkers <= 0:
            return
        ctx = self.ctx
        world = ctx.world
        center = self._center()
        models = walker_ids(ctx.library)
        walkers = []
        attempts = 0
        any_point = False  # the navigation mesh returned a point at least once
        max_attempts = NAVIGATION_ATTEMPTS_PER_WALKER * self.walkers
        # Same seed, same navigation points (and, in synchronous mode, about the same walks); drawn from our
        # generator so the scenario's seed decides
        world.set_pedestrians_seed(ctx.rng.randint(0, 2 ** 31 - 1))
        misses = 0
        while len(walkers) < self.walkers and attempts < max_attempts:
            attempts += 1
            loc = ctx.navigation_point()
            if loc is None:
                # A map without a navigation mesh (Town12 has none) returns None every time
                misses += 1
                if misses >= NAVIGATION_NONE_LIMIT and not any_point:
                    self.log("no navigation mesh for this map: wandering pedestrians skipped")
                    return
                continue
            any_point = True
            if math.hypot(loc.x - center.x, loc.y - center.y) > self.radius:
                continue
            walker = CarlaDataProvider.request_new_actor(
                ctx.rng.choice(models), carla.Transform(carla.Location(loc.x, loc.y, loc.z + 1.0)),
                actor_category="pedestrian", tick=False)
            if walker is not None:
                ctx.add_actor(walker)
                walkers.append(walker)
        self.log("spawned {}/{} wandering pedestrians ({} navigation points tried)".format(
            len(walkers), self.walkers, attempts))

        # request_new_actor cannot attach, so the AI controllers are spawned directly and removed in cleanup()
        controller_bp = ctx.library.find("controller.ai.walker")
        for walker in walkers:
            try:
                controller = world.spawn_actor(controller_bp, carla.Transform(), attach_to=walker)
            except RuntimeError as error:
                # A walker without a controller would stand there for good and not count as one of ours: remove it
                self.log("WARNING no AI controller for pedestrian {}: {}; the pedestrian is removed".format(
                    walker.id, error))
                ctx.remove_actor(walker)
                continue
            self._controllers.append(controller)
            self._walker_pairs.append((walker, controller))
            ctx.add_controller(controller)

    def after_tick(self):
        """Start the AI controllers and send each pedestrian to a random destination; after a world tick."""
        for controller in self._controllers:
            # Never started without a destination: a started controller then picks any navigation point itself, also
            # one on the carriageway (base.navigation_point keeps to sidewalks). It stands.
            destination = self.ctx.navigation_point()
            if destination is None:
                continue
            controller.start()
            controller.go_to_location(destination)
            controller.set_max_speed(self.ctx.rng.uniform(*WALKER_SPEED))

    def from_start(self):
        if not self.follow:
            return []
        kinds = [(label, count, models, category) for label, count, models, category in self._kinds if count > 0]
        wants_walkers = self.walkers if self.ctx.navigation_available() else 0
        if not kinds and wants_walkers <= 0:
            return []
        self.ctx.flow_active = True
        self.bubble = BackgroundBubble(self.ctx, self.radius, kinds, wants_walkers, self._tracked, self._walker_pairs,
                                       manager=self._manager)
        return [self.bubble]

    def cleanup(self):
        self._controllers = []
        self.ctx.remove_controllers()
