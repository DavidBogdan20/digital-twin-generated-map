"""<occluder model="vehicle.carlamotors.european_hgv" gap="1.5" queue="1" prop="static.prop.busstop"/>  m, count"""

import math

import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider

from carsim.blocks.base import HALF_LENGTH_DEFAULTS, Block, vehicle_ids
from carsim.road import occluder_transforms, prop_transform

TRUCK_HALF_LENGTH = 5.0  # half the length of the truck until it is spawned and its bounding box is known [m]
PROP_HALF = 1.0  # half length and half width of the prop until it is spawned and its bounding box is known [m]


class Occluder(Block):

    """
    A big vehicle that hides the crosswalk from the ego: it stands, handbrake on, in the kerb lane (the lane right of
    the ego lane) with its front `gap` m before the near edge of the crosswalk, and `queue` cars (random 4-wheel
    cars) wait behind it, 1.5 m clear apart. The ego must drive in the lane left of the kerb lane: pick the start
    with `find_crossing_spots.py --lane inner`. The truck must spawn (the scenario stops otherwise); a queue car
    that cannot be spawned is skipped.

    `prop` (default "none"): a static prop (a bus stop, say) on the kerb side, with its long side along the road, just
    before the crosswalk and reaching past the start of the pedestrian (see carsim.road.prop_transform). Together with
    the truck it hides the waiting pedestrian from the ego lane. A prop that cannot be spawned is skipped: the scenario
    runs, less hidden.

    The real length of a vehicle is only known once it is spawned, so each one is spawned where a vehicle of a
    typical length would stand (TRUCK_HALF_LENGTH, HALF_LENGTH_DEFAULTS), then moved to its place for its real length.
    """

    tag = "occluder"
    needs_crossing = True

    def __init__(self, ctx, params):
        super().__init__(ctx, params)
        self.model = self.params.get("model", "vehicle.carlamotors.european_hgv")
        self.gap = self.float_param("gap", 1.5)  # m
        self.queue = self.int_param("queue", 1)
        self.prop = self.params.get("prop", "none")  # blueprint id
        # CarlaDataProvider swaps an unknown blueprint for a default without a word (a tesla as the "truck"), so a typo is caught here
        if self.model not in vehicle_ids(ctx.library):
            raise ValueError("<occluder> model {!r} is not a vehicle blueprint".format(self.model))

    def spawn(self):
        ctx = self.ctx
        crossing = ctx.crossing
        models = [self.model]
        cars = vehicle_ids(ctx.library, base_type="car", lit=ctx.night)
        if self.queue > 0 and not cars:
            self.log("WARNING no car blueprints: no queue behind the truck")
        else:
            models += [ctx.rng.choice(cars) for _ in range(self.queue)]
        halves = []  # real half lengths of the vehicles spawned so far
        for i, model in enumerate(models):
            default = TRUCK_HALF_LENGTH if i == 0 else HALF_LENGTH_DEFAULTS["vehicle"]
            vehicle = ctx.request_on_surface(model, occluder_transforms(crossing, halves + [default], self.gap)[-1], tick=False)
            if vehicle is None:
                if i == 0:
                    raise ValueError("Failed to spawn the occluder {} in the kerb lane before the crosswalk".format(model))
                self.log("WARNING could not spawn queue car {} ({}): skipped".format(i, model))
                continue
            ctx.add_actor(vehicle)
            try:
                halves.append(vehicle.bounding_box.extent.x)
            except (AttributeError, RuntimeError):
                halves.append(default)
            spot = occluder_transforms(crossing, halves, self.gap)[-1]
            # request_new_actor lifts the spawn 0.2 m off the road; the vehicle is set down the same way
            vehicle.set_transform(ctx.surface(carla.Transform(spot.location + carla.Location(z=0.2), spot.rotation)))
            vehicle.apply_control(carla.VehicleControl(hand_brake=True))
            ctx.lights_on(vehicle)  # waiting at the crosswalk with the engine on, not parked
        self.log("{} stands {:g} m before the crosswalk, {} queued cars behind it".format(
            self.model, self.gap, len(halves) - 1))
        if self.prop != "none":
            self._spawn_prop()

    def _spawn_prop(self):
        crossing = self.ctx.crossing
        if not self.ctx.library.filter(self.prop):  # else CarlaDataProvider would spawn a streetbarrier
            self.log("WARNING the prop {} is not a blueprint: skipped".format(self.prop))
            return
        centre = prop_transform(crossing, PROP_HALF, PROP_HALF)
        prop = self.ctx.request_on_surface(self.prop, centre, actor_category="misc", tick=False)
        if prop is None:
            self.log("WARNING could not spawn the prop {}: skipped".format(self.prop))
            return
        self.ctx.add_actor(prop)
        try:
            box = prop.bounding_box
        except (AttributeError, RuntimeError):
            return
        # The actor's origin is not the middle of its box: put the middle where the pedestrian needs the cover
        centre = prop_transform(crossing, box.extent.x, box.extent.y)
        yaw = math.radians(centre.rotation.yaw)
        prop.set_transform(self.ctx.surface(carla.Transform(
            carla.Location(x=centre.location.x - box.location.x * math.cos(yaw) + box.location.y * math.sin(yaw),
                           y=centre.location.y - box.location.x * math.sin(yaw) - box.location.y * math.cos(yaw),
                           z=centre.location.z), centre.rotation)))
        self.log("{} stands before the crosswalk on the kerb side".format(self.prop))
