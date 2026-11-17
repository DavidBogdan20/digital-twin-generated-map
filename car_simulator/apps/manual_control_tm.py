#!/usr/bin/env python
"""
ScenarioRunner's manual_control.py (the driving window of a manual run) with every Vehicle.set_autopilot call on the
Traffic Manager port --tm-port (default 8100), not CARLA's default 8000.

    python apps/manual_control_tm.py --tm-port 8100 --keep_ego_vehicle [other manual_control.py options]

In CARLA (0.9.15 and 0.9.16) set_autopilot makes a Traffic Manager on its port even when it switches it off, and
manual_control.py calls set_autopilot(False) at its start without a port: on the VM port 8000 is a web server's (the
bind fails and the window exits, which ends the run), elsewhere the window would build a Traffic Manager of its own
(minutes on a large map). On ScenarioRunner's port the window is only a client of ScenarioRunner's Traffic Manager.
run_scenario.py starts the window through this.
"""

import argparse
import runpy
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

import carla  # noqa: E402

from carsim.paths import SCENARIO_RUNNER_ROOT  # noqa: E402
from carsim.sim import TRAFFIC_MANAGER_PORT  # noqa: E402


def pin_autopilot_port(port):
    """Make carla.Vehicle.set_autopilot use Traffic Manager `port` when the caller gives none."""
    original = carla.Vehicle.set_autopilot

    def set_autopilot(self, enabled=True, tm_port=port):
        return original(self, enabled, tm_port)

    carla.Vehicle.set_autopilot = set_autopilot


def main(argv):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument("--tm-port", type=int, default=TRAFFIC_MANAGER_PORT)
    args, rest = parser.parse_known_args(argv)
    pin_autopilot_port(args.tm_port)
    script = Path(SCENARIO_RUNNER_ROOT) / "manual_control.py"
    sys.argv = [str(script)] + rest
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main(sys.argv[1:])
