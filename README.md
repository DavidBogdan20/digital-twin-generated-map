# Digital twin generated map (CARLA 0.9.16)

The code and inputs behind **WolfsburgHighway**, a 2 x 2 km CARLA map of Wolfsburg generated from
OpenStreetMap with CARLA's Digital Twin Tool, and the scenarios that run on it. It was built on CARLA 0.9.15 on a Google
Cloud VM (`carla`, europe-west4-b, an L4 GPU); that state is the tag `carla-0.9.15`. Since 2026-11-17 the project
targets **CARLA 0.9.16** on any Linux machine with an NVIDIA GPU, in the `carla` Docker container (Ubuntu 22.04,
Python 3.10). `MIGRATION.md` moves the map from the old VM to a new machine.

| Path | On the VM / Mac | What it is |
|---|---|---|
| `digital_twin/` | VM `/work/dt` | Builds the map and tests it. `GUIDE.md` is the step-by-step guide and the log of every problem found; `vm.sh` runs each step on the VM. |
| `digital_twin/WolfsburgHighway.{xodr,osm,keepout}` | VM `/work/dt` | The exact inputs the map was generated from (output of `prepare_dt.py`). |
| `digital_twin/Wolfsburg.osm` | VM `/work/dt` | The OpenStreetMap export they were cut from. |
| `patches/` | `git diff` in `/work/carla`, `/work/UE4` | Our changes to CARLA 0.9.16 and Unreal Engine 4.26 (CarlaUnreal's `carla` branch), for reference. `digital_twin/apply_patches.py` makes them. |
| `crop_osm.py`, `load_osm.py`, `Dockerfile` | Mac `~/Coding/CARLA` | Turn the OSM export into the road file; `Dockerfile` is the `carla-client` image `load_osm.py` and `rails_plan.py` run in (0.9.16; its header builds the 0.9.15 one for the Mac's CARLA.app). |
| `car_simulator/` | VM `/work/car_simulator` | Euro NCAP style AEB scenarios (child behind an obstruction, crowded crosswalk, parking and cyclist cut-ins), ported to WolfsburgHighway and CARLA 0.9.16. Based on the `wolfsburg-0915` branch of botapaul/car_simulator; `PORTING.md` says what differs from upstream (0.9.15 notes: `PORTING_0.9.15.md` in the tag `carla-0.9.15`). `tools/navmesh/out/WolfsburgHighway.bin` is the walker navmesh in use. |
| `car_simulator_inputs/` | Mac | What car_simulator's tools read: the installed road file (after `uturns` and `lights`), `.keepout`, and the `RecastBuilder` (the same Recast branch in 0.9.15 and 0.9.16). |

The road files form one chain, checked byte for byte: `digital_twin/WolfsburgHighway.xodr` (md5 d722a686…) →
`remove_uturns.py` → adb47ff2… (as packaged in the fourth build) → `move_lights.py` →
`car_simulator_inputs/WolfsburgHighway.xodr` (e9d03dfa…, installed now).

## Move or rebuild

- **Move the existing map to a new machine:** `MIGRATION.md`. The old VM packs a bundle (`vm.sh bundle`), and the new
  machine builds CARLA 0.9.16 and cooks the map again from it (`vm.sh unpack`, `vm.sh recook`). About 4 to 5 hours of
  machine time, most of it unattended.
- **Generate the map from scratch** (about 6 to 8 hours; `digital_twin/GUIDE.md` has each stage): `vm.sh setup`,
  `start`, `stock`, `fetch` (a GitHub account linked to Epic Games), `engine`, `carla` (clones CARLA 0.9.16 and runs
  `apply_patches.py`, which edits exact lines of that version; `patches/` shows the result). Then from
  `vm.sh prepared WolfsburgHighway`: generate, audit, facades, rails (from `WolfsburgHighway_rails.json`), meshscan,
  tileoffset, package, and `vm.sh uturns WolfsburgHighway; vm.sh lights WolfsburgHighway; vm.sh speedsigns`. Copy
  `car_simulator/tools/navmesh/out/WolfsburgHighway.bin` to `/work/stock/CarlaUE4/Content/CustomMaps/WolfsburgHighway/Nav/`.
  The OSM steps (`crop_osm.py`, `load_osm.py`, `prepare_dt.py`) are only needed for a new area.

Not proven: nobody has generated the map on 0.9.16 yet, and on 0.9.15 nobody rebuilt it from scratch with this repo.
The inputs are the ones used, so the roads match; meshes may differ in details nothing here pins down (for example
which facade each building gets).

## Scripts in `digital_twin/`

- **Build:** `prepare_dt.py` (OSM to a cleaned road file), `apply_patches.py`, `open_tool.py` and `generate.py` (drive
  the editor's Digital Twin Tool), `facades.py`, `rails_plan.py` / `rails.py` (guard rails), `mesh_scan.py`,
  `tile_offset.py`.
- **Fix an installed map:** `remove_uturns.py` (`vm.sh uturns`), `move_lights.py` (`vm.sh lights`),
  `fix_speed_signs.py` (`vm.sh speedsigns`).
- **Move:** `vm.sh bundle` (old VM), `vm.sh unpack` and `vm.sh recook` (new machine); see `MIGRATION.md`.
- **Check a map:** `level_audit.py` / `check_level.py` / `level_prune.py`, `check_heights.py`, `merge_test.py` (ramps
  and bridges), `surface_check.py`, `jiggle_test.py`, `signals_check.py` (traffic lights and signs, with autopilot
  cars), `map_test.py`, `drive_test.py`, `dormant_grid.py`.
- **Share it:** `publish.py` (`vm.sh publish`) sends the hero car's state as JSON lines on port 2100 for the Herbie
  laptop through an SSH tunnel; `tunnel_check.py` reads the same state straight from CARLA. See `HERBIE_CONNECTION.md`
  and `HERBIE_DATA_PLAN.md`.
- **Drive it:** `drive.py` is CARLA's `manual_control.py` as patched by `vm.sh drive`; `elevation_scenario.py` and
  `elevation_chart.py` are the A 39 elevation scenario on the older Wolfsburg2 map.
- **One-off probes** used while debugging (kept as they were on the VM): `edge_probe.py`, `jiggle_probe.py`,
  `lane_width_scan.py`, `look*.py`, `probe*.py`, `sign_check.py`, `spotsrc.py`, `surface_*.py`, `track_profile.py`,
  `treestats.py`, `truckprobe.py`, `walk*.py`, `walker*.py`, `build4_*.sh`, `scen4.sh`.

## Not included

The cooked map itself, logs, screenshots and plots, the CARLA, Unreal and ScenarioRunner source trees (only our diffs
are here), and accounts: the GCP project, the Epic-linked GitHub access, Tailscale, and the VM's Sunshine and VNC
credential files.
