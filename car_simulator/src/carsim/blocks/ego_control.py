"""<ego_control mode="autopilot|manual" target_speed="30" brake="none|sight"/>  km/h"""

import carla
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaypointFollower

from carsim.blocks.base import Block
from carsim.blocks.spec import BRAKE_MODES, EGO_DEFAULT_MODE, EGO_DEFAULT_TARGET_SPEED, EGO_MODES
from carsim.sim import destroy_sensors_on


class EgoAutopilot(WaypointFollower):

    """
    The WaypointFollower, but the ego is at its test speed along its heading when it starts (as in NCAP, and like the
    CautiousDriver), not ramping up from rest: the ramp overshot the test speed (7 m/s for 6.4, max_brake 0.3), so a
    run could not reproduce the planned impact.
    """

    def initialise(self):
        forward = self._actor.get_transform().get_forward_vector()
        self._actor.set_target_velocity(carla.Vector3D(forward.x * self._target_speed, forward.y * self._target_speed,
                                                       forward.z * self._target_speed))
        return super().initialise()


class EgoControl(Block):

    """
    Who drives the ego. autopilot: the ego keeps the lane at `target_speed` straight through the junction and the
    scenario starts right away, as a single pass: the crossing, one drive through, an end condition, pass or fail.
    `brake` picks the driver. "sight": a CautiousDriver (carsim.blocks.driver): it stops for a crossing pedestrian or
    cyclist, brakes for an actor in its path and follows a slower vehicle ahead, all as far as it can see them (line
    of sight). "none": the same CautiousDriver with hazards ignored, so the same speed control and it never brakes for
    anything (the counterfactual run: where would the ego have hit). No `brake` at all: the plain WaypointFollower
    (EgoAutopilot), as before. manual ignores `brake`.

    manual: the user drives it with manual_control.py (run_scenario.py --drive manual opens it) and the run is
    endless (`ctx.endless`): no time limit and no distance end, the crossing and the background traffic keep flowing
    (see carsim.blocks.flow), and it ends only when the stop file of <run_control> appears (the driving window was
    closed) or with Ctrl-C.
    """

    tag = "ego_control"

    def __init__(self, ctx, params):
        super().__init__(ctx, params)
        self.mode = self.params.get("mode", EGO_DEFAULT_MODE)
        if self.mode not in EGO_MODES:
            raise ValueError("<ego_control> mode \"{}\" must be one of: {}".format(self.mode, ", ".join(EGO_MODES)))
        self.brake = self.params.get("brake")  # None: the plain WaypointFollower
        if self.brake is not None and self.brake not in BRAKE_MODES:
            raise ValueError("<ego_control> brake \"{}\" must be one of: {}".format(self.brake, ", ".join(BRAKE_MODES)))
        self.speed = self.float_param("target_speed", EGO_DEFAULT_TARGET_SPEED) / 3.6  # km/h -> m/s
        ctx.ego_target_speed = self.speed  # the test speed the metrics block reports
        ctx.endless = not self.autopilot

    @property
    def autopilot(self):
        return self.mode == "autopilot"

    def spawn(self):
        if self.autopilot:  # a manual driver switches them in the driving window (L)
            self.ctx.lights_on(self.ctx.ego)

    def from_start(self):
        if not self.autopilot:
            return []
        if self.brake is not None:
            from carsim.blocks.driver import CautiousDriver  # only these modes need the driver
            return [CautiousDriver(self.ctx, self.speed, hazards=self.brake == "sight")]
        return [EgoAutopilot(self.ctx.ego, self.speed, plan=self.ctx.ego_plan(), name="EgoAutopilot")]

    def cleanup(self):
        """
        Destroy the chase view's camera on the ego (apps/chase_view.py runs in its own process and would only notice
        once the ego is gone): ComposedScenario.terminate() runs this before ScenarioRunner destroys the ego, so no
        camera is left without its parent (see carsim.sim.destroy_sensors_on). Safe to call again.
        """
        if destroy_sensors_on(self.ctx.world, self.ctx.ego):
            self.log("removed the chase view's camera before the ego")
