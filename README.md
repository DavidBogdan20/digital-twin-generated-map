# Digital twin generated map (CARLA 0.9.15)

The code and inputs behind **WolfsburgHighway**, a 2 x 2 km CARLA 0.9.15 map of Wolfsburg generated from
OpenStreetMap with CARLA's Digital Twin Tool, and the scenarios that run on it. It all ran on a Google Cloud VM
(`carla`, us-west1-a, an L4 GPU) inside the `carla` Docker container (Ubuntu 20.04, Python 3.8). Snapshot of
2026-11-12.

| Path | On the VM / Mac | What it is |
|---|---|---|
| `digital_twin/` | VM `/work/dt` | Builds the map and tests it. `GUIDE.md` is the step-by-step guide and the log of every problem found; `vm.sh` runs each step on the VM. |
| `digital_twin/WolfsburgHighway.{xodr,osm,keepout}` | VM `/work/dt` | The exact inputs the map was generated from (output of `prepare_dt.py`). |
| `digital_twin/Wolfsburg.osm` | VM `/work/dt` | The OpenStreetMap export they were cut from. |
| `patches/` | `git diff` in `/work/carla`, `/work/UE4` | Our changes to CARLA 0.9.15.2 and Unreal Engine 4.26 (CarlaUnreal commit 8d08ab7fe), for reference. `digital_twin/apply_patches.py` makes them. |
| `crop_osm.py`, `load_osm.py`, `Dockerfile` | Mac `~/Coding/CARLA` | Turn the OSM export into the road file; `Dockerfile` is the `carla-client:0.9.15` image `load_osm.py` and `rails_plan.py` run in. |
| `car_simulator/` | VM `/work/car_simulator` | Euro NCAP style AEB scenarios (child behind an obstruction, crowded crosswalk, parking and cyclist cut-ins), ported to 0.9.15 and WolfsburgHighway. Copy of the `wolfsburg-0915` branch of botapaul/car_simulator; `PORTING_0.9.15.md` says what changed. `tools/navmesh/out/WolfsburgHighway.bin` is the walker navmesh in use. |
| `car_simulator_inputs/` | Mac | What car_simulator's tools read: the installed road file (after `uturns` and `lights`), `.keepout`, and CARLA 0.9.15's `RecastBuilder`. |

The road files form one chain, checked byte for byte: `digital_twin/WolfsburgHighway.xodr` (md5 d722a686…) →
`remove_uturns.py` → adb47ff2… (as packaged in the fourth build) → `move_lights.py` →
`car_simulator_inputs/WolfsburgHighway.xodr` (e9d03dfa…, installed now).

## Rebuild on a new VM

About 6 to 8 hours of VM time, most of it unattended. `digital_twin/GUIDE.md` has the details of each stage.

1. **VM** (GUIDE Stage 1): a GPU quota, Cloud NAT in the VM's region (Stage 7 gives the flags Tailscale needs), and the
   VM. Copy `digital_twin/` to `/work/dt`.
2. **System:** `vm.sh setup` (driver, then again after its reboot; it also builds the `carla-build` image from
   `digital_twin/Dockerfile`), `vm.sh start`, `vm.sh stock` (the packaged CARLA 0.9.15 the map is installed into).
3. **Unreal and CARLA** (Stages 3 and 4): `vm.sh fetch` needs a GitHub account linked to Epic Games for CARLA's Unreal
   fork. `vm.sh engine`, then `vm.sh carla`, which clones CARLA **0.9.15.2** and runs `apply_patches.py` on both trees
   before building. To build CARLA from another source, it must still be exactly 0.9.15.2 with `apply_patches.py`
   run on it (it edits exact lines of that version); `patches/` shows the expected result.
4. **The map** (Stage 9, from `vm.sh prepared WolfsburgHighway` on): the inputs are already in `/work/dt`, so the
   OSM steps (`crop_osm.py`, `load_osm.py`, `prepare_dt.py`) are only needed for a new area. Then generate, audit,
   facades, rails (from `WolfsburgHighway_rails.json`), meshscan, tileoffset, package, and
   `vm.sh uturns WolfsburgHighway; vm.sh lights WolfsburgHighway; vm.sh speedsigns`. Copy
   `car_simulator/tools/navmesh/out/WolfsburgHighway.bin` to
   `/work/stock/CarlaUE4/Content/CustomMaps/WolfsburgHighway/Nav/`.
5. **Viewing** (Stage 7): `vm.sh stream` (Tailscale and Sunshine), then pair Moonlight with `vm.sh pair <PIN>`.
6. **Scenarios:** copy `car_simulator/` to `/work/car_simulator` and follow `car_simulator/PORTING_0.9.15.md`
   (`scripts/setup_env_linux.sh`, ScenarioRunner v0.9.15).
7. **Check it:** `signals_check.py`, `merge_test.py`, `surface_check.py`, then the five scenario variants.

Not proven: nobody has rebuilt the map from scratch with this repo yet. The inputs and patches are the ones used, so
the roads match; meshes may still differ in details nothing here pins down (for example which facade each building
gets).

## Scripts in `digital_twin/`

- **Build:** `prepare_dt.py` (OSM to a cleaned road file), `apply_patches.py`, `open_tool.py` and `generate.py` (drive
  the editor's Digital Twin Tool), `facades.py`, `rails_plan.py` / `rails.py` (guard rails), `mesh_scan.py`,
  `tile_offset.py`.
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

The cooked map itself, logs, screenshots and plots, the CARLA, Unreal and ScenarioRunner source trees (only our diffs
are here), and accounts: the GCP project, the Epic-linked GitHub access, Tailscale, and the VM's Sunshine and VNC
credential files.
