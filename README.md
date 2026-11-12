# Digital twin generated map (CARLA 0.9.15)

The code behind **WolfsburgHighway**, a 2 x 2 km CARLA 0.9.15 map of Wolfsburg generated from OpenStreetMap with
CARLA's Digital Twin Tool, and the scenarios that run on it. Everything here ran on the Google Cloud VM (`carla`,
us-west1-a, an L4 GPU) inside the `carla` Docker container (Ubuntu 20.04, Python 3.8). Snapshot of 2026-11-12.

| Folder | On the VM | What it is |
|---|---|---|
| `digital_twin/` | `/work/dt` | Builds the map and tests it. `GUIDE.md` is the step-by-step guide and the log of every problem found; `vm.sh` runs each step on the VM. |
| `patches/` | `git diff` in `/work/carla` and `/work/UE4` | Our changes to CARLA 0.9.15.2 and to Unreal Engine 4.26 (CarlaUnreal commit 8d08ab7fe). `digital_twin/apply_patches.py` applies them. |
| `car_simulator/` | `/work/car_simulator` | Euro NCAP style AEB scenarios (child behind an obstruction, crowded crosswalk, parking and cyclist cut-ins), ported to 0.9.15 and WolfsburgHighway. Copy of the `wolfsburg-0915` branch of botapaul/car_simulator; `PORTING_0.9.15.md` says what changed. |

## Main scripts

- **Build:** `prepare_dt.py` (OSM to a cleaned road file), `apply_patches.py`, `open_tool.py` and `generate.py` (drive the editor's
  Digital Twin Tool), `facades.py`, `rails_plan.py` / `rails.py` (guard rails), `mesh_scan.py`, `tile_offset.py`.
- **Fix an installed map:** `remove_uturns.py` (`vm.sh uturns`), `move_lights.py` (`vm.sh lights`),
  `fix_speed_signs.py` (`vm.sh speedsigns`).
- **Check a map:** `level_audit.py` / `check_level.py` / `level_prune.py`, `check_heights.py`, `merge_test.py` (ramps
  and bridges), `surface_check.py`, `jiggle_test.py`, `signals_check.py` (traffic lights and signs, with autopilot
  cars), `map_test.py`, `drive_test.py`, `dormant_grid.py`.
- **Drive it:** `drive.py` is CARLA's `manual_control.py` as patched by `vm.sh drive`; `elevation_scenario.py` and
  `elevation_chart.py` are the A 39 elevation scenario on the older Wolfsburg2 map.
- **One-off probes** used while debugging (kept as they were on the VM): `edge_probe.py`, `jiggle_probe.py`,
  `lane_width_scan.py`, `look*.py`, `probe*.py`, `sign_check.py`, `spotsrc.py`, `surface_*.py`, `track_profile.py`,
  `treestats.py`, `truckprobe.py`, `walk*.py`, `walker*.py`, `build4_*.sh`, `scen4.sh`.

## Not included

Map data and build outputs: OSM and OpenDRIVE files, the cooked map, the navmesh `.bin` files, logs, screenshots, and
the CARLA, Unreal and ScenarioRunner source trees (only our diffs against them are here).
