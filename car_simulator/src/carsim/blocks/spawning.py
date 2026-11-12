"""<spawning hide_distance="50" far_distance="120" side_angle="60" behind_distance="15" behind_angle="110"/>  m, m, degrees, m, degrees"""

from carsim.blocks.base import Block
from carsim.road import (HIDE_BEHIND_ANGLE, HIDE_BEHIND_DISTANCE, HIDE_DISTANCE, HIDE_FAR_DISTANCE, HIDE_SIDE_ANGLE,
                         HideRule)


class Spawning(Block):

    """
    Where actors may appear and vanish while the scenario runs. Nothing is spawned or removed in view of the ego car:
    a spot is hidden (carsim.road.hidden_from_ego) when it is at least `far_distance` metres from the ego (whatever the
    direction), or at least `hide_distance` metres away and more than `side_angle` degrees off the ego's heading (off to
    the side), or at least `behind_distance` metres away and more than `behind_angle` degrees off the heading (behind
    the car; not while the ego is reversing). The flows of a manual run (pedestrians, cyclists, background traffic)
    spawn and remove only at hidden spots. Spawns at load, before the scenario starts, are not bound by it. Without the
    element the defaults 50, 120, 60, 15 and 110 apply.
    """

    tag = "spawning"

    def __init__(self, ctx, params):
        super().__init__(ctx, params)
        self.rule = HideRule(hide_distance=self.float_param("hide_distance", HIDE_DISTANCE),
                             behind_distance=self.float_param("behind_distance", HIDE_BEHIND_DISTANCE),
                             behind_angle=self.float_param("behind_angle", HIDE_BEHIND_ANGLE),
                             far_distance=self.float_param("far_distance", HIDE_FAR_DISTANCE),
                             side_angle=self.float_param("side_angle", HIDE_SIDE_ANGLE))
        ctx.hide = self.rule
