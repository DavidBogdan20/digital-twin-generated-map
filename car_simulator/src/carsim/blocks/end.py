"""<end distance_after_crossing="30" distance="100" timeout="60"/>  m, m, s"""

from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import DriveDistance

from carsim.blocks.base import Block
from carsim.blocks.run_control import EgoGone


class End(Block):

    """
    Ends the scenario once the ego has driven `distance_after_crossing` past the crossing point. A scenario without a
    crossing block has no crossing: the ego then drives `distance` from its start. `timeout` (s, counted from
    loading) overrides the default: with the autopilot max(60, drive distance / ego speed * 1.5 + 20) s.

    A manual run (ego_control mode manual) is endless: there is no distance end and no time limit (the default
    timeout is 7 days), and the run ends with the stop file of <run_control> or Ctrl-C. An explicit `timeout` still
    applies there. It also ends when the ego vehicle is gone (destroyed, e.g. by a driving window that was not told to
    keep it): the flows could do nothing but fail without it.
    """

    tag = "end"

    def __init__(self, ctx, params):
        super().__init__(ctx, params)
        ctx.after_crossing = self.float_param("distance_after_crossing", 30.0)
        ctx.distance = self.float_param("distance", 100.0)
        self._timeout = self.float_param("timeout", None)

    def timeout(self):
        return self._timeout

    def end_conditions(self):
        ctx = self.ctx
        if ctx.endless:
            return [EgoGone(ctx.ego)]
        name = "EgoPassedCrossing" if ctx.uses_crossing else "EgoDroveDistance"
        return [DriveDistance(ctx.ego, ctx.total_drive_distance(), name=name)]
