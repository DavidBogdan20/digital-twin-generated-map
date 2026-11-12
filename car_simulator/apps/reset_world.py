#!/usr/bin/env python
"""
Put the CARLA server back to a normal state after a scenario run was stopped.

    python apps/reset_world.py

Switches the server back to asynchronous mode and removes the ego vehicle and
scenario actors (role names "hero" and "scenario") that a killed ScenarioRunner
left behind, with the sensors attached to them (manual_control.py's) and the AI walker
controllers left without a walker (--all-controllers removes every AI walker controller
and every sensor without a parent). Use it when the CARLA window's camera or mouse
misbehaves or the world looks frozen after stopping run_scenario.py.
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from carsim.sim import connect, reset_world  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--all-controllers", action="store_true",
                        help="also remove AI walker controllers and parentless sensors that do not belong to a scenario")
    parser.add_argument("--tm-port", type=int, metavar="PORT",
                        help="also switch the Traffic Manager on this port back to asynchronous (one is created when none "
                             "runs there, which builds its road map first: minutes on a large map)")
    args = parser.parse_args()

    try:
        reset_world(connect(args.host, args.port), traffic_manager_port=args.tm_port, all_controllers=args.all_controllers)
    except RuntimeError as e:
        sys.exit(str(e))
    print("Server reset: asynchronous mode, no leftover scenario actors.")


if __name__ == "__main__":
    main()
