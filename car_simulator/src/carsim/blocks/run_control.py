"""<run_control stop_file="C:\\path\\to\\stop"/>  internal: written by apps/run_scenario.py, not by hand"""

import os

import carla
import py_trees

from srunner.scenariomanager.scenarioatomics.atomic_behaviors import AtomicBehavior

from carsim.blocks.base import Block


EGO_GONE_TICKS = 5  # the ego must be gone this many ticks in a row before the run ends


class StopFileExists(AtomicBehavior):

    """
    Succeeds as soon as the file `path` exists (checked every tick), so ScenarioRunner ends the run normally. With an
    `ego` it first applies full brake and the hand brake to it, so the car does not coast on after the driving window
    that steered it was closed.
    """

    def __init__(self, path, ego=None, name="StopFileExists"):
        super().__init__(name)
        self.path = str(path)
        self.ego = ego

    def update(self):
        if os.path.exists(self.path):
            self._brake()
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING

    def _brake(self):
        if self.ego is None:
            return
        try:
            if self.ego.is_alive:
                self.ego.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
        except RuntimeError:
            pass


class EgoGone(AtomicBehavior):

    """
    Succeeds when the ego vehicle no longer exists on the server (for EGO_GONE_TICKS ticks in a row, so a single odd
    answer cannot end a run; the tree is only ticked while the scenario runs, never at load or during the cleanup that
    destroys the ego). A manual run whose car was destroyed has nothing left to drive: it ends instead of every flow
    failing on a missing ego.
    """

    def __init__(self, ego, name="EgoGone"):
        super().__init__(name)
        self.ego = ego
        self.gone = 0

    def update(self):
        try:
            alive = bool(self.ego.is_alive)
        except RuntimeError:
            alive = False
        self.gone = 0 if alive else self.gone + 1
        if self.gone >= EGO_GONE_TICKS:
            print("ComposedScenario[run_control]: the ego vehicle is gone, the run ends")
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class RunControl(Block):

    """
    How a run is ended from outside. apps/run_scenario.py writes `<run_control stop_file="..."/>` into its copy of
    the XML for a manual run with the driving window and creates that file when the user closes the window; this
    block then ends the scenario cleanly, so ScenarioRunner writes its results and removes the actors as it does at
    any end. Not meant to be written by hand. Without the element a manual run ends with Ctrl-C.
    """

    tag = "run_control"

    def __init__(self, ctx, params):
        super().__init__(ctx, params)
        self.stop_file = self.params.get("stop_file") or None
        ctx.stop_file = self.stop_file

    def end_conditions(self):
        if self.stop_file is None:
            return []
        return [StopFileExists(self.stop_file, ego=self.ctx.ego)]
