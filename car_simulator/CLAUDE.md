# CLAUDE.md

Team rules for Claude Code sessions in this repo. `README.md` is the user manual (setup, blocks, block reference,
"Writing a new block", "Adding a new scenario"); read the section you need and do not copy it here.

## Project and goal

- Custom CARLA 0.9.16 driving scenarios, run with ScenarioRunner v0.9.16 on Windows or Linux (a Docker container with
  Ubuntu 22.04 and Python 3.10, CARLA at `/work/stock`), plus the large generated map WolfsburgHighway. Scenarios are
  composed from XML blocks (`src/carsim/blocks/`, run by `scenarios/_composed/composed_scenario.py`). `PORTING.md`
  lists what differs from upstream; the old CARLA 0.9.15 port is git tag `carla-0.9.15`.
- The scenarios mirror Euro NCAP AEB tests: child behind an obstruction, crowded crosswalk, parking cut-in, cyclist
  cut-in hidden by a truck. The long-term aim is to collect data and develop algorithms that make the car detect and
  react faster.
- So keep scenarios measurable and repeatable: deterministic timing (seeded `ctx.rng`), and logged metrics such as
  first-visible time, TTC and brake onset (see the `ctx.log("ego_control", ...)` lines in `blocks/driver.py`).
- Keep a clean perception -> decision seam in the driver. `cast_ray` line of sight is the baseline; a learned
  detector must be able to replace it without touching the decision code.
- `.agents/plans/ncap-data-review.md` holds the NCAP mapping and data-collection proposals (it may not exist yet).

## Commands

- Linux: `bash scripts/setup_env_linux.sh` once, then `python3 <script>` from the repo root (no venv, pip --user,
  carla 0.9.16 from PyPI). Offline checks locally: the Docker image of `scripts/test_image.Dockerfile` (commands in
  `PORTING.md`). The rest of this section is the Windows workflow.

- Always run from the repo root as `python -m uv run python <script>`. uv is not on PATH, and PowerShell blocks
  `.venv\Scripts\Activate.ps1`, so never activate the venv.
- Python is 3.11 (`.python-version`; 3.10 in the Linux container). 3.12 breaks ScenarioRunner's numpy pin. Add
  dependencies with `uv add`, never pip (on Linux: pinned in `scripts/setup_env_linux.sh`).
- Offline checks (no server). All must exit 0 before a change is done:
  `tests/check_blocks_offline.py`, `check_crossings_offline.py`, `check_flows_offline.py`, `check_driver_offline.py`,
  `check_cut_in_offline.py`, `check_trigger_offline.py`, `check_metrics_offline.py`, `check_chase_view_offline.py`
  (set `SDL_VIDEODRIVER=dummy`).
- `apps/list_scenarios.py --ours` must show no problems.
- Live runs: the user starts the CARLA server (`scripts/start_carla.ps1`). Run `apps/reset_world.py` after a stopped
  run. Leave no actors or sensors behind.

## Conventions that bite

- A block is a class keyed by its XML tag. Register it in `blocks/spec.py` (`BLOCK_ORDER`, `PARAM_TYPES`,
  `CROSSING_BLOCKS` if it uses the crosswalk, and `check_params` for any rule on a `"text"` value, since text is not
  checked otherwise) and in `blocks/__init__.py` (`_BLOCKS`). Extend `tests/check_blocks_offline.py`.
- Each tag may appear once per variant (ScenarioRunner keeps the last one). Variant names must be unique, also against
  ScenarioRunner's example XMLs. XML comments must not contain a double hyphen.
- `ComposedScenario`'s Main is a `Parallel` with SUCCESS_ON_ONE over every `from_start()` node, the triggered
  sequence and the end conditions. Any `from_start()` node that returns SUCCESS or FAILURE ends the scenario.
  `WaypointFollower` with no plan never ends; with a finite plan it succeeds at the end. `LaneChange` terminates the
  actor's other `WaypointFollower`, and fails when it has no plan (a junction in the way).
- The visibility rule: nothing spawns or vanishes in view of the ego during a run (`carsim.road.hidden_from_ego`,
  `ctx.hidden(location)`). Spawns at load are exempt.
- Geometry that needs no ScenarioRunner goes in `src/carsim/road.py`, with a check in
  `tests/check_crossings_offline.py`.
- Bounding boxes are unknown before spawn: spawn, then `set_transform` with the real extent (see `occluder.py`,
  `cut_in.py`). The offline fakes have no bounding box, so fall back to `HALF_LENGTH_DEFAULTS`.
- A block must not override `trigger_distance()` unless it means the shared crossing trigger (`pedestrian`, `crowd`,
  `cyclists` do). `cut_in` keeps its own value under another name.
- `blocks/spec.py` is plain data with no CARLA or ScenarioRunner imports, so validation works without them. Keep it so.

## Large generated maps (WolfsburgHighway)

The facts below were measured on CARLA 0.9.15 (the map was built there); recheck them on 0.9.16.

- `load_world` resets `tile_stream_distance`/`actor_active_distance` to 3000/2000, which crashed the server:
  `sim.ensure_map` sets 600/450 after every load. Tiles stream around the `hero`, else the spectator. Move the spectator
  in short steps (`sim.focus_spectator` does): big jumps crashed Vulkan.
- The large-map manager puts far actors, sensors included, to sleep; a sleeping camera crashed 0.9.15. Sensors only on
  the ego, destroyed before it (`ego_control` cleanup, `metrics` cleanup); `--view-spectator` uses no sensor.
- `world.cast_ray`/`ground_projection` may hit nothing on generated maps: the driver picks its line of sight with a
  self-test (`driver.choose_sight`, `CARSIM_SIGHT` forces it), `sim.wait_for_ground` falls back to a settle wait.
- On maps from the stock tool the road mesh waves about +-1.2 m around the OpenDRIVE z by a known function
  (`road.deformation`, LibCarla Deformation.h); WolfsburgHighway (fourth build, `road.FLAT_MAPS`) has flat roads.
  Sidewalks are 0.30 m higher on every generated map. So on generated maps every block spawn and `set_transform` goes
  through `ctx.surface(...)` (an offline check scans for it) and run_scenario spawns the ego on it. Never on Town maps.
- ScenarioRunner's `manual_control.py` runs through `apps/manual_control_tm.py` (set_autopilot would make a Traffic
  Manager on 8000).
- Traffic Manager on port 8100 (8000 is taken on the VM); its first start on a large map takes minutes. Never create one
  in `reset_world` by default (`traffic_manager_port=None`).

## CARLA 0.9.16 facts

- `world.cast_ray` returns many hits labelled `CityObjectLabel.NONE` (invisible volumes: one about 0.2 m from the
  eye on every ray, a capsule around each walker). Ignore them for line of sight. Real occluders are labelled: trucks
  and vans `Truck`, walkers `Pedestrians`, bicycles `Bicycle`. A hit has no actor id.
- `CityObjectLabel` has no `Vehicles` member (that was before 0.9.14): use `Car`, `Truck`, `Bus`, `Motorcycle`,
  `Bicycle`.
- `Sensor.is_listening` is a method in 0.9.16 (`sensor.is_listening()`), a property in older clients.
- Walker blueprints carry an `age` attribute (adult, child, teenager). Children are `walker.pedestrian.0009` to
  `0014`, `0048`, `0049`.
- `LocalPlanner` and `BasicAgent` `target_speed` is in km/h. `LocalPlanner.__init__` calls `vehicle.get_world()`, so
  create it in `initialise()`, not in `__init__`.
- Vehicle lights: CARLA never switches them on by itself (Traffic Manager does with `update_vehicle_lights`). No
  motorcycle or bicycle model has lights (`has_lights` false), nor 9 of the 23 car models (e.g. `mini.cooper_s`,
  `toyota.prius`): use `vehicle_ids(..., lit=True)` for night runs.
- Town10HD_Opt has no Parking lanes: use Shoulder (2.0 to 2.5 m wide, right of the driving lanes). WolfsburgHighway
  has no Shoulder either: `cut_in kind="parking"` then double-parks in the kerb Driving lane (`cut_in.is_kerb_lane`).
- Never call `client.get_trafficmanager()` in a process that outlives the start of ScenarioRunner (e.g. a pre-run
  reset in `run_scenario.py`): that process becomes the Traffic Manager server and, in synchronous mode, nothing
  steps it, so every background vehicle stands still. `sim.reset_world(client, traffic_manager_port=None)` skips it.
- `vehicle.lincoln.mkz_2017`'s automatic gearbox hunts between 1st and 2nd gear at 5 to 7 m/s, which makes any speed
  controller oscillate; the cautious driver holds 2nd gear (`manual_gear_shift`) between 3.5 and 10 m/s.
- A junction corner's sidewalk lane often has no successor (`next()` / `previous()` end): continue on the next
  sidewalk lane found a few metres straight ahead (`flow._bridge`).
- Run autopilot scenarios synchronous (the default): asynchronous runs with a slow client (the chase view, many
  `cast_ray` calls) see a lagging world and give wrong results.
- `road.Crossing.wp` is often inside the junction: take lane points off it with `road.lane_point(wp, s)`.
- Measured on the server (bounding box, L x W x H m): `vehicle.carlamotors.firetruck` 8.5 x 2.9 x 3.8,
  `european_hgv` 7.9 x 2.9 x 3.5, `mitsubishi.fusorosa` (bus) 10.3 x 3.9 x 4.3 (wider than a 3.5 m lane),
  `mercedes.sprinter` 5.9 x 2.0 x 2.6, `static.prop.busstop` 3.9 x 1.9 x 2.7, `static.prop.kiosk_01` 1.6 x 1.7 x 3.2.

## Working style

- Make the smallest complete change. Match the surrounding style (docstrings in the same voice, comment density,
  naming). Every non-trivial logic gets an offline check.
- Plans live in `.agents/plans/<task>-Plan.md`. Sub-agents read the plan file, not the conversation. Report
  deviations from the plan instead of improvising.
- For a multi-part task, split it into parallel subagents with tight briefs and disjoint file ownership (Sonnet for
  implementation, Haiku for mechanical work). Never let two agents edit the same file. Shared files (`spec.py`,
  `__init__.py`) are changed once, first, by the orchestrator.
- Before reporting done, always review: an Opus reviewer, plus an independent GPT/Codex logic review (codex plugin)
  for non-trivial logic. Fix what they find, then verify live where possible.
- Small one-off edits are done inline, without subagents.
- Docs and suggested commands use the `python -m uv run python apps\<app>.py ...` form.
