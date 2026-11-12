#!/usr/bin/env python
"""
ComposedScenario: one generic ScenarioRunner scenario class that is built from blocks.

A scenario folder needs only its XML. Each child element of a <scenario> whose tag is a block name
(ego_control, spawning, run_control, pedestrian, crowd, cyclists, background, end) switches that block on and
carries its parameters; the blocks live in src/carsim/blocks/ and are described in the README of the repository.
ScenarioRunner loads this file through --additionalScenario (apps/run_scenario.py picks it for every scenario folder
without a .py of its own); the class name must match the `type` attribute of the scenario in the XML.

The behavior tree:

    Sequence
      Parallel (ends when one child succeeds)  "Main"
        behaviours of the blocks that run from the start (the ego autopilot; the endless flows of a manual run)
        Sequence "Triggered"                   only if some block has triggered behaviours
          WaitForEgo                           ego within trigger_distance of the crossing point; with the pedestrian's
                                               `overlap` instead: its time to arrival at the impact point is less than
                                               the pedestrian's walk time to it
          Parallel (ends when all succeed)     the triggered behaviours of the blocks
        end conditions of the blocks           the drive distance (autopilot only), the stop file, the ego gone (manual)
        WaitForever                            manual (endless) runs only: nothing else ends them but Ctrl-C
      cleanup behaviours of the blocks (removing the main pedestrian)

A manual ego (ego_control mode manual) makes the run endless: no distance end, a timeout of 7 days (or an explicit
<end timeout>), and it ends when the stop file of <run_control> exists (the driving window was closed; the ego is
braked then), when the ego vehicle no longer exists, or with Ctrl-C. The start trigger of BasicScenario (wait until the
ego moves) is dropped when a flow runs (carsim.blocks.flow: pedestrians, cyclists, or a background bubble of vehicles or
pedestrians; the blocks say so in spawn()), because the flows run from the first tick. A manual run without any flow
(no navigation mesh and no cyclists or background vehicles) keeps it, and then the stop file and the ego vanishing
end the run before the trigger fires too (the end conditions in the tree are not ticked until it has).

Criteria: CollisionTest(ego). terminate() and __del__ call every block's cleanup(). terminate() first prints the line
"ComposedScenario: scenario ending" and creates the marker file next to the stop file (carsim.blocks.spec.ending_marker):
it runs before CarlaDataProvider.cleanup destroys the ego, so a caller that watches the output or the marker can tell a
normal end from the driving window being closed.
"""

from __future__ import print_function

import sys
from pathlib import Path

import py_trees
import carla

from srunner.scenariomanager.carla_data_provider import CarlaDataProvider
from srunner.scenariomanager.scenarioatomics.atomic_criteria import CollisionTest
from srunner.scenariomanager.scenarioatomics.atomic_behaviors import WaitForever
from srunner.scenariomanager.scenarioatomics.atomic_trigger_conditions import InTriggerDistanceToLocation
from srunner.scenarios.basic_scenario import BasicScenario

# Make the repository's shared helpers importable when ScenarioRunner is started directly. Appended, and only when
# missing, so it never shadows what the interpreter already finds first
_SRC = str(Path(__file__).resolve().parents[2] / "src")
if _SRC not in sys.path:
    sys.path.append(_SRC)
from carsim.blocks import create_blocks  # noqa: E402
from carsim.blocks.base import BlockContext  # noqa: E402
from carsim.blocks.run_control import EgoGone, StopFileExists  # noqa: E402
from carsim.blocks.spec import MANUAL_DEFAULT_TIMEOUT, ending_marker  # noqa: E402


DEBUG_LIFE_TIME = 120.0  # --debug markers stay this long at most [s]
CAUTIOUS_WAIT = 60.0  # [s] added to the default autopilot timeout for a cautious driver, which may wait for crossers


class ComposedScenario(BasicScenario):

    """A scenario assembled from the blocks named by the XML elements of its variant. Single ego vehicle."""

    def __init__(self, world, ego_vehicles, config, randomize=False, debug_mode=False, criteria_enable=True,
                 timeout=60):
        cmap = CarlaDataProvider.get_map()
        ego_start_wp = cmap.get_waypoint(config.trigger_points[0].location)
        self._ctx = BlockContext(world, cmap, ego_vehicles[0], ego_start_wp, self, debug=debug_mode)
        self._blocks = create_blocks(self._ctx, config.other_parameters)
        self._block_by_tag = {block.tag: block for block in self._blocks}
        self._ego_autopilot = self._block_by_tag["ego_control"].autopilot
        self._ctx.trigger_distance = self._pick_trigger_distance()
        # ScenarioRunner starts the timeout when the scenario loads; a manual run has none to speak of
        timeouts = [block.timeout() for block in self._blocks if block.timeout() is not None]
        self.timeout = timeouts[0] if timeouts else self._default_timeout(timeout)

        super().__init__("ComposedScenario",
                         ego_vehicles,
                         config,
                         world,
                         debug_mode,
                         criteria_enable=criteria_enable)

    def _default_timeout(self, minimum):
        """
        The timeout [s] when `<end timeout>` is not set. Manual driving is endless: MANUAL_DEFAULT_TIMEOUT (7 days);
        ScenarioRunner's TimeOut only compares the game time with it. Autopilot: the ego needs
        total_drive_distance / speed seconds, so allow 1.5 times that and 20 s more, but at least `minimum`
        (ScenarioRunner's 60 s). The crossing (computed once and cached, as the blocks need it anyway) gives the
        distance to drive. A cautious driver (brake="sight") gets CAUTIOUS_WAIT more: it may wait at the crosswalk.
        """
        if not self._ego_autopilot:
            return float(MANUAL_DEFAULT_TIMEOUT)
        ego = self._block_by_tag["ego_control"]
        speed = ego.speed  # m/s
        if speed <= 0.0:
            return minimum
        wait = CAUTIOUS_WAIT if ego.brake == "sight" else 0.0
        return max(minimum, self._ctx.total_drive_distance() / speed * 1.5 + 20.0 + wait)

    def _pick_trigger_distance(self):
        """From the pedestrian block, else the first block that sets one, else the default of the context."""
        asked = [(block.tag, block.trigger_distance()) for block in self._blocks
                 if block.trigger_distance() is not None]
        asked.sort(key=lambda item: item[0] != "pedestrian")  # stable: the pedestrian first, then block order
        return asked[0][1] if asked else self._ctx.trigger_distance

    def _initialize_actors(self, config):
        for block in self._blocks:
            block.spawn()
        # One tick for everything spawned above (the scenery was requested without one); the AI controllers must
        # exist on the server before they are started
        self._ctx.tick_world()
        for block in self._blocks:
            block.after_tick()

        if self._ctx.debug and self._ctx.uses_crossing:
            crossing = self._ctx.crossing
            debug = self._ctx.world.debug
            life_time = min(self.timeout, DEBUG_LIFE_TIME)  # a manual run's timeout is 7 days: do not leave markers that long
            debug.draw_string(crossing.wp.transform.location + carla.Location(z=2.0), "crossing",
                              color=carla.Color(255, 0, 0), life_time=life_time)
            debug.draw_line(crossing.spawn.location, crossing.end + carla.Location(z=1.0), thickness=0.1,
                            color=carla.Color(255, 0, 0), life_time=life_time)

    def _cleanup_blocks(self):
        """Every block's cleanup; the AI controllers must go before ScenarioRunner destroys their walkers."""
        for block in getattr(self, "_blocks", []):
            block.cleanup()
        if hasattr(self, "_ctx"):
            self._ctx.remove_controllers()

    def terminate(self):
        """
        ScenarioManager.cleanup() calls this and then CarlaDataProvider.cleanup(), which destroys the actors, the
        ego among them. The first line printed is the one a caller can wait for to know that the scenario ended
        on its own terms, before the ego goes. The marker file next to the stop file says the same to a caller that has
        not read the output yet; without it (it cannot be written) the caller only has the line.
        """
        print("ComposedScenario: scenario ending", flush=True)
        stop_file = getattr(getattr(self, "_ctx", None), "stop_file", None)
        if stop_file:
            try:
                Path(ending_marker(stop_file)).touch()
            except OSError:
                pass
        self._cleanup_blocks()
        super().terminate()

    def _setup_scenario_trigger(self, config):
        """
        The default trigger waits until the ego is moving towards its start point, which never happens when the
        scenario itself drives the ego, so an autopilot run has none. A manual run with a flow (pedestrians, cyclists
        or background traffic that keep arriving) has none either: the flows run from the first tick, so the crossing
        is busy when the user gets there. This is decided here, before _create_behavior, so the blocks set
        ctx.flow_active in spawn() (a background bubble included). A manual run without a flow (no navigation mesh
        and nothing else) keeps it, with the stop file and the ego vanishing as other ways through, so closing the
        driving window before moving, or destroying a stationary ego, still ends the run (the end conditions of the
        tree are behind the trigger).
        """
        if self._ego_autopilot or self._ctx.flow_active:
            return None
        trigger = super()._setup_scenario_trigger(config)
        if trigger is not None:
            either = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE,
                                                  name="EgoMovesOrEnds")
            either.add_child(trigger)
            if self._ctx.stop_file:
                either.add_child(StopFileExists(self._ctx.stop_file, ego=self._ctx.ego))
            either.add_child(EgoGone(self._ctx.ego))
            trigger = either
        return trigger

    def _create_behavior(self):
        """The tree in the module docstring, from the hooks of the blocks in BLOCK_ORDER."""
        root = py_trees.composites.Sequence(name="ComposedScenario")
        main = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ONE, name="Main")

        for block in self._blocks:
            for node in block.from_start():
                main.add_child(node)

        triggered = [node for block in self._blocks for node in block.on_trigger()]
        if triggered:
            sequence = py_trees.composites.Sequence(name="Triggered")
            pedestrian = self._block_by_tag.get("pedestrian")
            wait = pedestrian.start_condition(self.ego_vehicles[0]) if pedestrian is not None else None
            sequence.add_child(wait if wait is not None else InTriggerDistanceToLocation(
                self.ego_vehicles[0], self._ctx.trigger_location, self._ctx.trigger_distance, name="WaitForEgo"))
            parallel = py_trees.composites.Parallel(policy=py_trees.common.ParallelPolicy.SUCCESS_ON_ALL,
                                                    name="TriggeredBehaviours")
            for node in triggered:
                parallel.add_child(node)
            sequence.add_child(parallel)
            main.add_child(sequence)

        for block in self._blocks:
            for node in block.end_conditions():
                main.add_child(node)
        if self._ctx.endless:
            # A parallel with nothing in it would succeed at once; only the stop file (or Ctrl-C) ends a manual run
            main.add_child(WaitForever(name="ManualRun"))
        root.add_child(main)

        for block in self._blocks:
            for node in block.cleanup_behaviors():
                root.add_child(node)
        return root

    def _create_test_criteria(self):
        """
        Pass/fail checks evaluated during the whole run. CollisionTest fails on a collision with anything, crowd,
        cyclists and traffic included, but ignores hits while the ego is slower than 0.1 m/s and counts hits within
        5 m of the last one, or with the same actor within 5 s of it, as one.
        """
        return [CollisionTest(self.ego_vehicles[0])]

    def __del__(self):
        self._cleanup_blocks()
        if hasattr(self, "other_actors"):
            self.remove_all_actors()
