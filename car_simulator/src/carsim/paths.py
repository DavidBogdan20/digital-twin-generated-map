"""
Well-known locations in the repository.

CARLA_ROOT and SCENARIO_RUNNER_ROOT can be overridden with environment
variables; by default they point at the folders inside this repository
(on Linux scripts/setup_env_linux.sh links CARLA_0.9.16 to the CARLA install).
"""

import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

# Made absolute, so a relative environment variable keeps working after a program changes its working directory
CARLA_ROOT = Path(os.path.abspath(os.environ.get("CARLA_ROOT", REPO_ROOT / "CARLA_0.9.16")))
SCENARIO_RUNNER_ROOT = Path(os.path.abspath(
    os.environ.get("SCENARIO_RUNNER_ROOT", REPO_ROOT / "external" / "scenario_runner")))

SCENARIOS_DIR = REPO_ROOT / "scenarios"
OUTPUTS_DIR = REPO_ROOT / "outputs"


def scenario_dir(name):
    """Folder of a named scenario, e.g. scenarios/pedestrian_crossing."""
    return SCENARIOS_DIR / name


def list_scenario_names():
    """Names of all scenario folders that contain an XML config."""
    if not SCENARIOS_DIR.is_dir():
        return []
    return sorted(d.name for d in SCENARIOS_DIR.iterdir() if d.is_dir() and any(d.glob("*.xml")))
