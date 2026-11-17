#!/usr/bin/env python
"""
One-time setup. Runs with any Python on PATH (it only bootstraps; the
project itself uses the Python 3.11 venv that uv builds from pyproject.toml):

    python scripts/setup_env.py
    python scripts/setup_env.py --recreate        # rebuild .venv from scratch

Run it from a terminal where .venv is NOT activated.

Steps (each is skipped when already done, so it is safe to re-run):
  1. clone ScenarioRunner v0.9.16 into external/scenario_runner
  2. get uv (pip install --user uv if it is not on PATH)
  3. install Python 3.11 with uv
  4. uv python pin + uv sync --locked: create .venv and install the locked dependencies from
     pyproject.toml / uv.lock, plus this project's carsim package (editable)
  5. write car_simulator.pth so CARLA's PythonAPI/carla (the 'agents'
     package) and ScenarioRunner ('srunner') are importable in the venv
  6. check the imports
"""

import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
CARLA_ROOT = Path(os.path.abspath(os.environ.get("CARLA_ROOT", REPO_ROOT / "CARLA_0.9.16")))  # absolute: it goes into a .pth
SR_ROOT = REPO_ROOT / "external" / "scenario_runner"
SR_URL = "https://github.com/carla-simulator/scenario_runner.git"
VENV = REPO_ROOT / ".venv"
VENV_PYTHON = VENV / ("Scripts/python.exe" if os.name == "nt" else "bin/python")


def run(cmd, check=True):
    cmd = [str(c) for c in cmd]
    print("> " + " ".join(cmd), flush=True)
    return subprocess.run(cmd, check=check, cwd=REPO_ROOT)


def find_uv():
    exe = shutil.which("uv")
    if exe:
        return [exe]
    module = [sys.executable, "-m", "uv"]
    if subprocess.run(module + ["--version"], capture_output=True).returncode != 0:
        if run([sys.executable, "-m", "pip", "install", "--user", "uv"], check=False).returncode != 0:
            sys.exit("Could not install uv with pip. Install it directly: https://docs.astral.sh/uv/getting-started/installation/")
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--python", default="3.11", help="Python version for .venv (3.10 or 3.11)")
    parser.add_argument("--sr-tag", default="v0.9.16", help="ScenarioRunner git tag")
    parser.add_argument("--recreate", action="store_true", help="delete and rebuild .venv")
    args = parser.parse_args()

    if Path(sys.prefix).resolve() == VENV.resolve():
        sys.exit("Run this with a Python outside .venv: deactivate the venv first (run 'deactivate').")

    # uv's cache (C:) and the repo may be on different drives, where hardlinks fail
    os.environ.setdefault("UV_LINK_MODE", "copy")

    print(f"== ScenarioRunner {args.sr_tag}")
    if (SR_ROOT / "scenario_runner.py").is_file():
        print(f"   already present: {SR_ROOT}")
    else:
        run(["git", "clone", "--branch", args.sr_tag, "--depth", "1", SR_URL, SR_ROOT])

    uv = find_uv()

    print(f"== Python {args.python} and .venv")
    if args.recreate and VENV.exists():
        try:
            shutil.rmtree(VENV)
        except PermissionError as error:
            sys.exit(f"Cannot delete .venv ({error.filename} is in use). "
                     "Close programs using it (IDE, terminals with it activated, running apps) and retry.")
    # On Windows uv may report a harmless "minor version link" error after a
    # successful install; a real failure shows up in `uv sync` below
    run(uv + ["python", "install", args.python], check=False)
    # .python-version is what `uv run` / `uv sync` use later, so keep it in line with --python
    run(uv + ["python", "pin", args.python])
    run(uv + ["sync", "--locked"])

    print("== Import paths")
    site_packages = subprocess.check_output(
        [str(VENV_PYTHON), "-c", "import sysconfig; print(sysconfig.get_paths()['purelib'])"], text=True).strip()
    pth = Path(site_packages) / "car_simulator.pth"
    pth.write_text("\n".join(str(p) for p in (CARLA_ROOT / "PythonAPI" / "carla", SR_ROOT)) + "\n",
                   encoding="ascii")
    print(f"   wrote {pth}")
    if not (CARLA_ROOT / "PythonAPI" / "carla" / "agents").is_dir():
        print(f"   WARNING: CARLA PythonAPI not found under {CARLA_ROOT}; set CARLA_ROOT and re-run.")

    print("== Check")
    run([VENV_PYTHON, "-c",
         "import carla, py_trees, pygame, srunner, carsim; from agents.navigation.basic_agent import BasicAgent; "
         "print('   all imports OK, carla', carla.__file__)"])

    activate = r".venv\Scripts\Activate.ps1" if os.name == "nt" else "source .venv/bin/activate"
    print(f"\nDone. Activate with:  {activate}   (or prefix commands with: uv run)")


if __name__ == "__main__":
    try:
        main()
    except subprocess.CalledProcessError as error:
        sys.exit(f"\nSetup failed: {error}")
