#!/usr/bin/env python
"""
List the scenarios you can run: the ones in this repository's scenarios/
folder and ScenarioRunner's built-in examples. Does not need a CARLA server.

    python apps/list_scenarios.py
    python apps/list_scenarios.py --ours
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from carsim.paths import list_scenario_names  # noqa: E402
from carsim.runner import run_scenario_runner  # noqa: E402
from carsim.scenario_files import validate, variants  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--ours", action="store_true", help="only list this repository's scenarios")
    args = parser.parse_args()

    print("This repository (python apps/run_scenario.py <scenario> [--variant NAME]):")
    for name in list_scenario_names():
        print(f"  {name}")
        try:
            for v in variants(name):
                print(f"      variant {v.name:<32} class {v.type:<28} map {v.town}")
            problems = validate(name)
        except (OSError, ValueError, ET.ParseError) as e:
            problems = [str(e)]
        for problem in problems:
            print(f"      PROBLEM: {problem}")

    if not args.ours:
        print("\nScenarioRunner built-in examples (external/scenario_runner/srunner/examples):")
        run_scenario_runner(["--list"])


if __name__ == "__main__":
    main()
