"""<cut_in kind="parking|cyclist" distance="60" trigger_distance="15" speed="4" blocker="vehicle.mercedes.sprinter"/>  m, m, m/s"""

import carla
import py_trees

from agents.navigation.local_planner import RoadOption
from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import BasicAgentBehavior, LaneChange, WaypointFollower
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToVehicle

from carsim.blocks.base import HALF_LENGTH_DEFAULTS, Block, vehicle_ids
from carsim.blocks.spec import CUT_IN_KINDS
from carsim.road import lane_point, straight_route

CAR = "vehicle.mini.cooper_s_2021"  # the parked car that pulls out (a model with lights, for night runs)
BLOCKERS = {"parking": "vehicle.mercedes.sprinter", "cyclist": "vehicle.carlamotors.firetruck"}  # kind -> default blocker
DEFAULTS = {"parking": (60.0, 15.0, 4.0), "cyclist": (25.0, 12.0, 4.0)}  # kind -> distance, trigger_distance, speed
SPAWN_SPACING = 10.0  # the actors are first spawned this far apart, so no spawn overlaps; then set to their real place [m]
START_LIFT = 0.2  # an actor is set down this far above the road surface, as CarlaDataProvider lifts a spawn [m]
PARKED_GAP = 1.5  # clear space between the blocker and the parked car [m]
AHEAD_GAP = 3.0  # clear space between the cyclist and car A ahead of it [m]
BEHIND_GAP = 1.5  # clear space between the blocker and the cyclist [m]
MERGE_DISTANCE = 10.0  # the parked car reaches the ego lane about this far ahead of its spot [m]
MERGE_ROUTE = 60.0  # the route it is given along the ego lane from there; the car then drives on without a plan [m]


class CutIn(Block):

    """
    An actor pulls into the ego lane ahead of the ego, from where a blocker hides it, and keeps driving at `speed` until
    the scenario ends: it never stops, and no behaviour of this block ends the scenario. `distance` is how far along the
    ego lane from the ego start the actor starts, `trigger_distance` how close the ego must be (centre to centre) for it
    to go. Everything is placed by the lane geometry, so the start must be on a straight, junction free lane.

    kind="parking"  the ego is in the kerb lane with a Shoulder on its right. A vehicle.mini.cooper_s is parked on the
                    Shoulder at `distance`, pushed to the outer edge as ScenarioRunner's ParkingCutIn does, and the
                    blocker (default vehicle.mercedes.sprinter) stands behind it (nearer the ego), 1.5 m clear, both
                    with the hand brake on. Without a Shoulder there (maps with none, such as WolfsburgHighway) the
                    ego may be in the lane left of the kerb lane instead: the car and the blocker then stand
                    double-parked in the kerb Driving lane (see is_kerb_lane), the same way. At the trigger the car is given `speed` along its heading (it would need
                    about a second to get going from standstill), pulls out into the ego lane, merging about 10 m
                    ahead, and drives on.
    kind="cyclist"  the ego is in the inner lane, with a Driving lane of the same direction on its right. In that lane,
                    at `distance`, ride a random bicycle, a random car A 3 m clear ahead of it and the blocker (default
                    vehicle.carlamotors.firetruck, 8.5 x 2.9 x 3.8 m) 1.5 m clear behind it (the blocker hides the
                    cyclist), all at `speed`: the three are forced to `speed` along their heading on every tick (the
                    bicycle under a WaypointFollower alone only holds about 0.72 x speed), so the gaps hold.
                    The cyclist changes into the ego lane (LaneChange to the left) and rides on ahead of the ego.
                    The blocker has no collision avoidance: SR's detection box reaches two body lengths ahead, so it
                    would never leave the cyclist.
    blocker         the blueprint id of the hiding vehicle, for either kind. An id that is not a vehicle blueprint is
                    a ValueError.

    Spawned and placed in spawn(): each actor is spawned apart from the others, then set to its real place once its
    bounding box is known. ctx.target is the cut-in actor (the car or the cyclist). The tree is made in from_start().
    The followers have no plan, so they never end. A WaypointFollower of an actor is ended by the next behaviour of that actor (see AtomicBehavior.initialise), which only
    happens here inside a Parallel that has already stopped it, so nothing under Main returns SUCCESS or FAILURE as long
    as the lane change is possible (LaneChange fails without a plan: a junction or a lane that forbids it).
    """

    tag = "cut_in"

    def __init__(self, ctx, params):
        super().__init__(ctx, params)
        self.kind = self.params.get("kind")
        if self.kind not in CUT_IN_KINDS:
            raise ValueError("<cut_in> kind {} must be one of: {}".format(
                "is missing" if self.kind is None else repr(self.kind), ", ".join(CUT_IN_KINDS)))
        distance, trigger, speed = DEFAULTS[self.kind]
        self.distance = self.float_param("distance", distance)  # [m]
        self.trigger_at = self.float_param("trigger_distance", trigger)  # [m] (not trigger_distance(): no say in the shared trigger)
        self.speed = self.float_param("speed", speed)  # [m/s]
        self.blocker = self.params.get("blocker") or BLOCKERS[self.kind]  # blueprint id of the hiding vehicle
        # CarlaDataProvider swaps an unknown model for a default without a word, so a typo is caught here (the defaults are known)
        if self.blocker != BLOCKERS[self.kind] and self.blocker not in vehicle_ids(ctx.library):
            raise ValueError("<cut_in> blocker {!r} is not a vehicle blueprint".format(self.blocker))
        self.actors = {}  # role -> actor, from spawn(); the cut-in actor is also ctx.target
        self.merge_plan = None  # parking: the route into the ego lane

    def spawn(self):
        ctx = self.ctx
        point = lane_point(ctx.ego_start_wp, self.distance)
        side = point.get_right_lane()
        if self.kind == "parking":
            if side is None or (side.lane_type != carla.LaneType.Shoulder and not is_kerb_lane(point, side)):
                raise ValueError("<cut_in kind=\"parking\"> needs a Shoulder on the right of the ego lane {:g} m ahead "
                                 "(pick a start in a kerb lane), or, on a map without one, the kerb Driving lane of the "
                                 "same direction there (pick a start in the lane left of it)".format(self.distance))
            if side.lane_type != carla.LaneType.Shoulder:
                self.log("parking: no Shoulder right of the ego lane, the car and the blocker stand double-parked in "
                         "the kerb Driving lane")
            self._spawn_parking(point, side)
        else:
            if side is None or side.lane_type != carla.LaneType.Driving or (side.lane_id > 0) != (point.lane_id > 0):
                raise ValueError("<cut_in kind=\"cyclist\"> needs a Driving lane of the same direction on the right of "
                                 "the ego lane {:g} m ahead; pick a start in the lane left of the kerb lane".format(
                                     self.distance))
            self._spawn_cyclist(side)
        self.ctx.target = self.actors["car" if self.kind == "parking" else "cyclist"]

    def _spawn_parking(self, point, side):
        car = self._spawn("car", CAR, side)
        van = self._spawn("blocker", self.blocker, lane_point(side, -SPAWN_SPACING))
        self._place(car, side, parked=True)
        self._place(van, lane_point(side, -(half_length(car) + PARKED_GAP + half_length(van))), parked=True)
        for actor in (car, van):
            actor.apply_control(carla.VehicleControl(hand_brake=True))
        self.ctx.lights_on(car)  # about to pull out; the van is parked and stays dark
        merge = lane_point(point, MERGE_DISTANCE)
        self.merge_plan = [(wp, RoadOption.LANEFOLLOW) for wp in [merge] + straight_route(merge, MERGE_ROUTE)]
        self.log("parking: {} on the {} {:g} m ahead of the ego start, {} {:g} m clear behind it".format(
            CAR, "shoulder" if side.lane_type == carla.LaneType.Shoulder else "kerb lane", self.distance, self.blocker,
            PARKED_GAP))

    def _spawn_cyclist(self, side):
        ctx = self.ctx
        bikes = vehicle_ids(ctx.library, base_type="bicycle")
        cars = vehicle_ids(ctx.library, base_type="car", lit=ctx.night)
        if not bikes or not cars:
            raise ValueError("<cut_in kind=\"cyclist\"> needs bicycle and car blueprints")
        bike = self._spawn("cyclist", ctx.rng.choice(bikes), side, "bicycle")
        car = self._spawn("car_a", ctx.rng.choice(cars), lane_point(side, SPAWN_SPACING))
        van = self._spawn("blocker", self.blocker, lane_point(side, -SPAWN_SPACING))
        self._place(bike, side)
        self._place(car, lane_point(side, half_length(bike) + AHEAD_GAP + half_length(car)))
        self._place(van, lane_point(side, -(half_length(bike) + BEHIND_GAP + half_length(van))))
        for actor in (car, van):
            self.ctx.lights_on(actor)
        self.log("cyclist: {} {:g} m ahead of the ego start in the lane to the right, car A {:g} m clear ahead of it, "
                 "{} {:g} m clear behind it".format(bike.type_id, self.distance, AHEAD_GAP, self.blocker, BEHIND_GAP))

    def _spawn(self, role, model, wp, category="car"):
        actor = self.ctx.request_on_surface(model, wp.transform, actor_category=category, tick=False)
        if actor is None:
            raise ValueError("<cut_in> could not spawn the {} ({}) at ({:.1f}, {:.1f})".format(
                role, model, wp.transform.location.x, wp.transform.location.y))
        self.ctx.add_actor(actor)
        self.actors[role] = actor
        return actor

    def _place(self, actor, wp, parked=False):
        """Set `actor` down on the lane of `wp`; a parked one is pushed to the outer edge like ParkingCutIn does."""
        t = wp.transform
        push = (wp.lane_width - half_width(actor)) / 4.0 if parked else 0.0
        right = t.get_right_vector()
        actor.set_transform(self.ctx.surface(carla.Transform(
            carla.Location(t.location.x + push * right.x, t.location.y + push * right.y, t.location.z + START_LIFT),
            t.rotation)))

    def from_start(self):
        ctx, speed = self.ctx, self.speed
        if self.kind == "parking":
            car = self.actors["car"]
            sequence = py_trees.composites.Sequence(name="ParkingCutIn")
            sequence.add_child(InTriggerDistanceToVehicle(ctx.ego, car, self.trigger_at, name="EgoNearParkedCar"))
            sequence.add_child(Launch([car], speed, name="LaunchCar"))  # no start from standstill: it is moving at the trigger
            # The agent ends with its plan: the follower then keeps the car going in the ego lane
            sequence.add_child(BasicAgentBehavior(car, plan=self.merge_plan, target_speed=speed * 3.6,
                                                  opt_dict={"ignore_traffic_lights": True}, name="PullOut"))
            sequence.add_child(WaypointFollower(car, speed, avoid_collision=True, name="DriveOn"))
            return [sequence]

        bike = self.actors["cyclist"]
        ride = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="RideUntilEgoNear")
        ride.add_child(WaypointFollower(bike, speed, name="RideInKerbLane"))
        ride.add_child(InTriggerDistanceToVehicle(ctx.ego, bike, self.trigger_at, name="EgoNearCyclist"))
        sequence = py_trees.composites.Sequence(name="CyclistCutIn")
        sequence.add_child(ride)
        sequence.add_child(LaneChange(bike, speed, direction="left", distance_same_lane=2, distance_other_lane=10,
                                      distance_lane_change=8, name="CutIn"))
        sequence.add_child(WaypointFollower(bike, speed, name="RideAhead"))
        # Under WaypointFollower alone the bicycle holds only ~0.72 x speed and the blocker would ram it, so all three are
        # forced to `speed` along their heading every tick (as KeepVelocity does for the crossing cyclists); the followers
        # still steer (and LaneChange steers the bicycle), so the gaps hold before the trigger and after the lane change
        return [Launch([bike, self.actors["car_a"], self.actors["blocker"]], speed, hold=True, name="HoldConvoySpeed"),
                WaypointFollower(self.actors["car_a"], speed, name="CarAFollows"),
                # no avoid_collision: its detection box reaches two body lengths ahead, so it would hold the blocker
                # still behind the cyclist 1.5 m in front of it
                WaypointFollower(self.actors["blocker"], speed, name="BlockerFollows"),
                sequence]


class Launch(py_trees.behaviour.Behaviour):

    """
    Gives each actor `speed` along its heading (following the road's slope) once, then succeeds. With `hold` it does so
    on every tick and never ends: a forced speed that the actor's own controller cannot lower.
    """

    def __init__(self, actors, speed, hold=False, name="Launch"):
        super().__init__(name)
        self.actors = list(actors)
        self.speed = speed
        self.hold = hold

    def update(self):
        for actor in self.actors:
            forward = actor.get_transform().get_forward_vector()
            actor.set_target_velocity(carla.Vector3D(forward.x * self.speed, forward.y * self.speed, forward.z * self.speed))
        return py_trees.common.Status.RUNNING if self.hold else py_trees.common.Status.SUCCESS


def is_kerb_lane(wp, side):
    """True when `side` (the lane right of `wp`) is the kerb lane: a Driving lane of the same direction with no Driving
    lane right of it."""
    if side is None or side.lane_type != carla.LaneType.Driving or (side.lane_id > 0) != (wp.lane_id > 0):
        return False
    right = side.get_right_lane()
    return right is None or right.lane_type != carla.LaneType.Driving


def half_length(actor):
    try:
        return actor.bounding_box.extent.x
    except (AttributeError, RuntimeError):
        return HALF_LENGTH_DEFAULTS["vehicle"]


def half_width(actor):
    try:
        return actor.bounding_box.extent.y
    except (AttributeError, RuntimeError):
        return 1.0
