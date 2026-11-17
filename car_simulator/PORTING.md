# What differs from upstream

Upstream (botapaul/car_simulator, "First commit") targets CARLA 0.9.16, ScenarioRunner (SR) v0.9.16 and Python 3.11
on Windows. This copy targets the same versions and adds Linux, large generated maps and WolfsburgHighway. The version
files (`pyproject.toml`, `uv.lock`, `.python-version`, `scripts/setup_env.py`, `scripts/start_carla.ps1`) are
upstream's. The earlier port to CARLA 0.9.15 (Python 3.8, SR v0.9.15 without `--frameRate`, its live results) is git
tag `carla-0.9.15`, file `PORTING_0.9.15.md`.

## Linux

- `scripts/setup_env_linux.sh`: one-time setup in a Docker container with Ubuntu 22.04 and Python 3.10, CARLA 0.9.16
  at `CARLA_ROOT` (default `/work/stock`). Clones SR v0.9.16 into `external/scenario_runner`, installs `carla==0.9.16`
  (PyPI), SR's requirements and pygame with `pip --user`, links `CARLA_0.9.16 -> $CARLA_ROOT`, writes `carsim.pth`
  (`src`, `$CARLA_ROOT/PythonAPI/carla` for `agents`, SR) and checks the imports. Then `python3 apps/<app>.py`.
- `runner.start_window`: on Linux the chase view and the driving window run in their own session with their output in
  the run folder (`chase_view.log`, `manual_control.log`); a warning when `DISPLAY` is not set.
- `apps/manual_control_tm.py`: runs SR's `manual_control.py` with `set_autopilot` pinned to `--tm-port`; without it
  the window's `set_autopilot(False)` makes a Traffic Manager on 8000 (taken on the VM, or minutes on a large map).
- `run_scenario.py`: `--tm-port` (default 8100), `--stream-distance`, `--active-distance`, `--view-spectator`; the
  resets after a run create no Traffic Manager (`sim.reset_world(..., traffic_manager_port=None)`).
- `scripts/test_image.Dockerfile`: the container, for the offline checks (below).

## Large generated maps (`sim.py`, `road.py`)

Measured on CARLA 0.9.15, where WolfsburgHighway was built; recheck on 0.9.16.

- `ensure_map` sets `tile_stream_distance` 600 m and `actor_active_distance` 450 m after every load (`load_world`
  resets them to 3000/2000, which crashed the server). `focus_spectator` moves in steps of at most 100 m.
- `wait_for_ground` accepts other labels after 15 s and, when the trace hits nothing, waits 10 s more (`NOT verified`).
- Spawn heights: the stock tool's road mesh is OpenDRIVE z + 0.7 x `GetZPosInDeformation(x, y)` (LibCarla
  `road/Deformation.h`, the same in 0.9.15 and 0.9.16), sidewalks 0.30 m higher. On generated maps (`road.DEFORMED_MAPS`,
  `/Game/CustomMaps/`, `CARSIM_DEFORMED`) every block spawn and `set_transform` goes through `ctx.surface(...)`;
  WolfsburgHighway has flat roads (`road.FLAT_MAPS`).

## Line of sight and sensors

- `driver.choose_sight`: a self-test ray at `initialise()`; if `World.cast_ray` does not see the ego, the driver uses
  `BoxRays` (rays against the scenario actors' bounding boxes). `CARSIM_SIGHT=cast_ray|boxes` forces one.
- The chase view's camera is destroyed before the ego (`ego_control` cleanup); `--view-spectator` uses no sensor (the
  server's spectator follows the ego). A sleeping sensor crashed 0.9.15 on large maps.

## WolfsburgHighway

- Variants of `pedestrian_crossing`, `occluded_crossing` and `cut_in` on WolfsburgHighway; `tools/wolfsburg`,
  `tools/navmesh`.
- `road.crosswalks()` handles open 4-corner outlines; `cut_in kind="parking"` double-parks in the kerb Driving lane
  when there is no Shoulder (`cut_in.is_kerb_lane`).

## Offline checks

- `check_crossings_offline.py --xodr PATH` (repeatable) and `CARSIM_XODR_DIRS` check other OpenDRIVE files;
  `check_blocks_offline.py` builds the Wolfsburg variants when it finds `WolfsburgHighway.xodr`.
- New checks for the above in `check_driver`, `check_cut_in`, `check_chase_view`, `check_metrics`.
- `run_config.write_run_config` calls `ET.indent` only when it exists (Python 3.8).

Image with Ubuntu 22.04, Python 3.10, carla 0.9.16 and CARLA 0.9.16's `agents` (from the repo root):

```bash
docker build --platform linux/amd64 -f scripts/test_image.Dockerfile -t carsim-test:0.9.16 .
# the checks need CARLA 0.9.16's Town OpenDRIVE files at <maps>/CarlaUE4/Content/Carla/Maps/OpenDrive/*.xodr
docker run --rm --platform linux/amd64 -v <maps>/CarlaUE4:/work/stock/CarlaUE4:ro carsim-test:0.9.16 bash -c \
  'fail=0; for t in tests/check_*_offline.py; do python3 $t > /tmp/$(basename $t).log 2>&1 && echo "PASS $t" || { echo "FAIL $t"; fail=1; }; done; python3 apps/list_scenarios.py --ours; exit $fail'
```

## Open points

- The large-map facts above and the Wolfsburg variants have not been run live on CARLA 0.9.16; the map needs its
  0.9.16 re-cook first. `tools/navmesh` and `tools/wolfsburg` still use the 0.9.15 Docker images and binaries.
- `BoxRays` sees only the scenario's actors, not buildings or terrain.
