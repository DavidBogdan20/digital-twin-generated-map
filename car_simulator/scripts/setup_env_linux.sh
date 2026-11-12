#!/usr/bin/env bash
# One-time setup on Linux for CARLA 0.9.15 (the VM's "carla" container: Ubuntu 20.04, Python 3.8, CARLA at
# /work/stock). No venv and no uv: everything goes into the user's site-packages with pip --user. Safe to re-run.
#
#     bash scripts/setup_env_linux.sh
#
# Steps (each is skipped when already done):
#   1. clone ScenarioRunner v0.9.15 into external/scenario_runner
#   2. install the carla 0.9.15 client: the wheel $CARLA_WHEEL, else the egg next to it (put on the import path),
#      else carla==0.9.15 from PyPI
#   3. install ScenarioRunner v0.9.15's requirements (pinned to versions that have Python 3.8 wheels) and pygame
#   4. link CARLA_0.9.15 -> $CARLA_ROOT in the repo (carsim.paths and the offline checks look there)
#   5. write carsim.pth into the user site-packages: src (carsim), $CARLA_ROOT/PythonAPI/carla (agents), ScenarioRunner
#   6. check the imports and the carla client version
#
# Environment: CARLA_ROOT (default /work/stock), CARLA_WHEEL (default the VM's wheel), PYTHON (default python3).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${PYTHON:-python3}"
CARLA_ROOT="${CARLA_ROOT:-/work/stock}"
CARLA_WHEEL="${CARLA_WHEEL:-/work/carla/PythonAPI/carla/dist/carla-0.9.15-cp38-cp38-linux_x86_64.whl}"
CARLA_EGG="$(dirname "$CARLA_WHEEL")/carla-0.9.15-py3.8-linux-x86_64.egg"
SR_TAG=v0.9.15
SR="$REPO/external/scenario_runner"
EXTRA_PATH=""

echo "== ScenarioRunner $SR_TAG"
if [ ! -f "$SR/scenario_runner.py" ]; then
    git clone --depth 1 --branch "$SR_TAG" https://github.com/carla-simulator/scenario_runner.git "$SR"
fi
tag="$(git -c safe.directory="*" -C "$SR" describe --tags --exact-match 2>/dev/null || true)"
if [ "$tag" != "$SR_TAG" ]; then
    echo "external/scenario_runner is at '${tag:-unknown}', not $SR_TAG: delete it and re-run." >&2
    exit 1
fi

echo "== carla 0.9.15 client"
if [ -f "$CARLA_WHEEL" ]; then
    "$PY" -m pip install --user "$CARLA_WHEEL"
elif [ -e "$CARLA_EGG" ]; then
    EXTRA_PATH="$CARLA_EGG"
else
    "$PY" -m pip install --user carla==0.9.15
fi

echo "== Python packages"
# ScenarioRunner v0.9.15's requirements.txt, pinned where it is open: the newest matplotlib wants numpy >= 1.20,
# which would break its numpy==1.18.4 (pip 20's resolver does not see the conflict)
"$PY" -m pip install --user \
    py-trees==0.8.3 numpy==1.18.4 networkx==2.2 Shapely==1.7.1 psutil==5.9.8 xmlschema==1.0.18 ephem==4.1.5 \
    tabulate==0.9.0 opencv-python==4.2.0.32 matplotlib==3.3.4 six==1.16.0 simple-watchdog-timer==0.1.1 \
    antlr4-python3-runtime==4.10 graphviz==0.20.1 pygame==2.6.1

echo "== Paths"
if [ ! -e "$REPO/CARLA_0.9.15" ] && [ -d "$CARLA_ROOT" ]; then
    ln -s "$CARLA_ROOT" "$REPO/CARLA_0.9.15"
    echo "   linked $REPO/CARLA_0.9.15 -> $CARLA_ROOT"
fi
SITE="$("$PY" -c 'import site; print(site.getusersitepackages())')"
mkdir -p "$SITE"
printf '%s\n' "$REPO/src" "$CARLA_ROOT/PythonAPI/carla" "$SR" $EXTRA_PATH > "$SITE/carsim.pth"
echo "   wrote $SITE/carsim.pth"
[ -d "$CARLA_ROOT/PythonAPI/carla/agents" ] || echo "   WARNING: no agents package under $CARLA_ROOT/PythonAPI/carla; set CARLA_ROOT and re-run."

echo "== Check"
"$PY" -c "
import carla, py_trees, pygame, srunner, carsim
from agents.navigation.local_planner import LocalPlanner
version = carla.Client('127.0.0.1', 2000).get_client_version()
assert version.startswith('0.9.15'), 'carla client is ' + version + ', not 0.9.15'
print('   all imports OK, carla client', version, carla.__file__)
"
