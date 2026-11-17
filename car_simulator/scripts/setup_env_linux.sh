#!/usr/bin/env bash
# One-time setup on Linux for CARLA 0.9.16 (a Docker container with Ubuntu 22.04 and Python 3.10, CARLA at
# /work/stock). No venv and no uv: everything goes into the user's site-packages with pip --user. Safe to re-run.
#
#     bash scripts/setup_env_linux.sh
#
# Steps (each is skipped when already done):
#   1. clone ScenarioRunner v0.9.16 into external/scenario_runner
#   2. install the carla 0.9.16 client from PyPI (its cp310 wheel)
#   3. install ScenarioRunner v0.9.16's requirements.txt and pygame
#   4. link CARLA_0.9.16 -> $CARLA_ROOT in the repo (carsim.paths and the offline checks look there)
#   5. write carsim.pth into the user site-packages: src (carsim), $CARLA_ROOT/PythonAPI/carla (agents), ScenarioRunner
#   6. check the imports and the carla client version
#
# Environment: CARLA_ROOT (default /work/stock), PYTHON (default python3).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${PYTHON:-python3}"
CARLA_ROOT="${CARLA_ROOT:-/work/stock}"
SR_TAG=v0.9.16
SR="$REPO/external/scenario_runner"

echo "== ScenarioRunner $SR_TAG"
if [ ! -f "$SR/scenario_runner.py" ]; then
    git clone --depth 1 --branch "$SR_TAG" https://github.com/carla-simulator/scenario_runner.git "$SR"
fi
tag="$(git -c safe.directory="*" -C "$SR" describe --tags --exact-match 2>/dev/null || true)"
if [ "$tag" != "$SR_TAG" ]; then
    echo "external/scenario_runner is at '${tag:-unknown}', not $SR_TAG: delete it and re-run." >&2
    exit 1
fi

echo "== Python packages"
"$PY" -m pip install --user carla==0.9.16 pygame==2.6.1 -r "$SR/requirements.txt"

echo "== Paths"
if [ ! -e "$REPO/CARLA_0.9.16" ] && [ -d "$CARLA_ROOT" ]; then
    ln -s "$CARLA_ROOT" "$REPO/CARLA_0.9.16"
    echo "   linked $REPO/CARLA_0.9.16 -> $CARLA_ROOT"
fi
SITE="$("$PY" -c 'import site; print(site.getusersitepackages())')"
mkdir -p "$SITE"
printf '%s\n' "$REPO/src" "$CARLA_ROOT/PythonAPI/carla" "$SR" > "$SITE/carsim.pth"
echo "   wrote $SITE/carsim.pth"
[ -d "$CARLA_ROOT/PythonAPI/carla/agents" ] || echo "   WARNING: no agents package under $CARLA_ROOT/PythonAPI/carla; set CARLA_ROOT and re-run."

echo "== Check"
"$PY" -c "
import carla, py_trees, pygame, srunner, carsim
from agents.navigation.local_planner import LocalPlanner
version = carla.Client('127.0.0.1', 2000).get_client_version()
assert version.startswith('0.9.16'), 'carla client is ' + version + ', not 0.9.16'
print('   all imports OK, carla client', version, carla.__file__)
"
