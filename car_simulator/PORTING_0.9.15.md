# Port to CARLA 0.9.15 on Linux (branch `wolfsburg-0915`)

The scenario framework was written for CARLA 0.9.16, ScenarioRunner (SR) v0.9.16, Python 3.11 on Windows. This
branch runs it on **CARLA 0.9.15 only** (server, client, maps), **SR v0.9.15**, in the VM's Docker container `carla`
(Ubuntu 20.04, Python 3.8.10, pip 20.0.2, user `carla`, `DISPLAY=:0`), including the large generated map
WolfsburgHighway. No 0.9.16 artifact was installed or used for any of this, offline tooling included.

## 1. Audit: what 0.9.16 code needed for 0.9.15

Checked against the 0.9.15 sources and docs (tag `0.9.15` of carla-simulator/carla: `PythonAPI/carla/agents`,
`PythonAPI/carla/source/libcarla`, `Docs/python_api.md`, `Docs/bp_library.md`, `Docs/catalogue_*.md`, `CHANGELOG.md`)
and SR tag `v0.9.15`, and by running every offline check with the 0.9.15 client and the 0.9.15 Town OpenDRIVE files.

| Area | Result |
|---|---|
| Python API | Everything used exists in 0.9.15: `cast_ray`, `ground_projection`, `get_crosswalks`, `WorldSnapshot.has_actor/find`, `ActorSnapshot.get_transform`, `WorldSettings.tile_stream_distance/actor_active_distance/spectator_as_ego`, `set_replayer_ignore_hero/spectator`, `Sensor.is_listening`, `VehicleLightState`, `Actor.is_alive`, `bounding_box` on props. |
| `CityObjectLabel` | 0.9.15's binding (World.cpp) has `Car`, `Truck`, `Bus`, `Motorcycle`, `Bicycle`, `Rider`, `Train`, `NONE`, `Roads`, `Sidewalks`, `Ground`, `Terrain`, `Static`... as used. (The 0.9.15 *docs* still list `Vehicles`; the API does not have it.) |
| Blueprints | All exist in 0.9.15 (catalogue): `lincoln.mkz_2017`, `carlamotors.european_hgv`, `carlamotors.firetruck`, `carlamotors.carlacola`, `mercedes.sprinter`, `mitsubishi.fusorosa`, `mini.cooper_s(_2021)`, `audi.a2`, `tesla.model3`, bicycles `bh.crossbike`, `diamondback.century`, `gazelle.omafiets`, motorcycles, `static.prop.busstop`, `static.prop.kiosk_01`, walkers `0001`-`0049` (children `0048`, `0049` included). Vehicles carry `base_type`, `has_lights`, `number_of_wheels`; walkers carry `age`. No change needed. |
| Agents package | 0.9.15's `LocalPlanner(vehicle, opt_dict, map_inst)` takes the same `opt_dict` keys (`target_speed` km/h, `lateral_control_dict`, `max_throttle`, `max_brake`), `set_global_plan`, `set_speed`, `run_step`, `reset_vehicle`; `RoadOption`, `BasicAgent` as used. No change. |
| SR v0.9.15 | Same constructor signatures for every atomic used (`ActorDestroy`, `KeepVelocity`, `BasicAgentBehavior`, `LaneChange`, `WaypointFollower`, `ActorTransformSetter`, `Idle`, `WaitForever`, `InTimeToArrivalToLocation`, `InTriggerDistanceToVehicle`, `InTriggerDistanceToLocation`, `DriveDistance`, `CollisionTest`), `generate_target_waypoint_list_multilane`, `CarlaDataProvider` (`_carla_actor_pool`, `request_new_actor(..., tick=)`, `remove_actor_by_id`, `get_traffic_manager_port`, random seed 2000), the output lines the runner watches ("ScenarioManager: Running scenario", "Results of Scenario: ... ---- X", "Destroying ego vehicle"), `scenario_runner.py` flags (`--trafficManagerPort`, `--sync`, `--record`, `--additionalScenario`, `--configFile`, `--outputDir`...), `manual_control.py` flags (`--keep_ego_vehicle`, `--host`, `-p/--port`). **Two differences:** SR v0.9.15 has no `--frameRate` (a synchronous run is fixed at 20 Hz, `frame_rate = 20.0` in scenario_runner.py), and its `manual_control.py` calls `set_autopilot(False)` without a port, which in 0.9.15 makes a Traffic Manager on 8000 (see 2, Linux runtime). SR does not reload the world (no `--reloadWorld`), so the streaming settings below survive into its run. |
| SR requirements | v0.9.15 pins numpy 1.18.4, Shapely 1.7.1, networkx 2.2, opencv 4.2.0.32 (all have cp38 wheels). Its unpinned matplotlib would pull 3.7 (needs numpy >= 1.20; pip 20's resolver does not notice): pinned to 3.3.4. |
| Python 3.8 | One 3.9+ call: `xml.etree.ElementTree.indent` in `run_config.write_run_config` (every run writes the XML copy with it) — now only when available. Nothing else (grep for 3.9+ stdlib, `list[...]`, `removeprefix`, ...). |
| Maps (0.9.15 Town OpenDRIVE) | The Town10HD variants' starts are unchanged and still give the crossings their comments describe (120 m and 90 m ahead). Map-derived expectations of the checks differ: Town10HD has 3 inner-lane candidates (4 on 0.9.16), and 9 candidates of Town03/04/05/10HD (none used by a scenario) miss a geometry check (see 3.6). **No scenario XML was changed by this port.** |

## 2. Changes, and why

### Setup and environment
- `scripts/setup_env_linux.sh` (new): clones SR v0.9.15 into `external/scenario_runner` (refuses another tag),
  installs the carla 0.9.15 wheel `/work/carla/PythonAPI/carla/dist/carla-0.9.15-cp38-cp38-linux_x86_64.whl` with
  `pip --user` (else the egg next to it, put on the path; else `carla==0.9.15` from PyPI), installs SR v0.9.15's
  requirements pinned for Python 3.8 plus `pygame==2.6.1`, links `CARLA_0.9.15 -> $CARLA_ROOT` (default `/work/stock`)
  in the repo, writes `carsim.pth` into the user site-packages (`src`, `$CARLA_ROOT/PythonAPI/carla` for `agents`,
  SR), and checks the imports and that the client is 0.9.15. Safe to re-run. Env: `CARLA_ROOT`, `CARLA_WHEEL`, `PYTHON`.
- `scripts/test_image.Dockerfile` (new): the offline-check image (section 5).
- `src/carsim/paths.py`: default `CARLA_ROOT` is `CARLA_0.9.15` (the setup's link). `.gitignore`: `CARLA_0.9.15`.
- Windows files kept, moved off 0.9.16: `pyproject.toml` pins `carla==0.9.15`, Python `>=3.10,<3.11` (0.9.15 has no
  newer wheel), `uv.lock` re-resolved (`uv lock`, metadata only), `.python-version` 3.10, `scripts/setup_env.py`
  defaults to SR v0.9.15 / Python 3.10, `scripts/start_carla.ps1` to `CARLA_0.9.15`. Not tested on Windows.
- `README.md`, `CLAUDE.md`: say that this branch targets 0.9.15 on Linux, with the new facts.

### Linux runtime (`src/carsim/runner.py`, `apps/run_scenario.py`)
- `runner.start_window`: the chase view and the driving window get their own console on Windows (as before); on Linux
  their own session (`start_new_session`, so a Ctrl-C in the scenario terminal reaches SR but not the window, like the
  separate console) and their output in a log file in the run folder (`chase_view.log`, `manual_control.log`), with a
  warning when `DISPLAY` is not set. Messages no longer point at `setup_env.py` with Windows paths only.
- `run_scenario.py`: `--tm-port` (default **8100**, passed to SR as `--trafficManagerPort`, recorded in `run.json`),
  `--stream-distance`/`--active-distance`, `--view-spectator`; the resets after a run no longer create a Traffic Manager.
  SR's arguments come from `scenario_runner_arguments()`: `--sync` only, no `--frameRate` (SR v0.9.15 would refuse
  the whole command line); `--frame-rate` other than 20 is refused with a message, `frame_rate` stays in `run.json`.
- The driving window (`--drive manual`) runs through `apps/manual_control_tm.py`: it pins `carla.Vehicle.set_autopilot`'s
  default port to `--tm-port` and runs SR's `manual_control.py` unchanged (`runpy`, same arguments). In 0.9.15
  `set_autopilot(False)` still makes a Traffic Manager on its port: on 8000 (taken on the VM) the window would exit at
  once and end the run, elsewhere it would build a road map for minutes. The `--no-driver-window` hint prints it too.

### Large maps (`src/carsim/sim.py`, `apps/run_scenario.py`)
- `ensure_map` (all apps that load a map: run_scenario, find_crossing_spots, list_maps `--load`, replay) sets
  `tile_stream_distance` 600 m and `actor_active_distance` 450 m after every load, and whenever they differ
  (`apply_streaming`; `load_world` resets them to 3000/2000, which crashed the server).
- `focus_spectator` travels to a far target in steps of at most 100 m with 1 s of server time between them (big
  spectator jumps crashed Vulkan). Tiles stream around the actor with role `hero` (SR's ego, `rolename="hero"` in all
  XMLs), else around the spectator, which run_scenario keeps at the ego start.
- `wait_for_ground`: a hit within the tolerance with a label outside Roads/Sidewalks/Ground/Terrain is accepted after
  15 s (generated meshes may be labelled otherwise) to know the tile is there; a surface the trace has not hit at all
  after 15 s gets 10 s more of server time and counts as loaded without proof (None), with a `WARNING ... NOT verified`.
- **Spawn heights on generated maps** (`road.deformation`, `road.deformed_map`, `road.surface_offset`,
  `BlockContext.surface`). The Digital Twin Tool's road mesh is OpenDRIVE z + 0.7 x `GetZPosInDeformation(x, y)`
  (LibCarla `road/Deformation.h`, 0.9.15; CARLA x, y), sidewalks 0.1524 + 0.15 m higher (Lane.cpp, mesh generation):
  the navmesh agent's formula, copied into `src/carsim/road.py`. Checked against
  `digital_twin/elevation_run.csv` (Wolfsburg2, a car driving it; car z minus the formula): route 0-860 m,
  **735 samples: mean +0.006 m, sd 0.019 m, 5-95 % -0.015..+0.041 m, worst -0.036/+0.164 m** (the +0.16 is the spawn
  drop at route 0; without it, on the 706 moving samples, the worst is +0.095 m on the A 39 ramp). The rows after 970 m
  (the car stuck under the bridge after the tree launch) and 860-970 m are left out. The driving lanes' bumps
  (`GetBumpDeformation`, <= 0.1 m) made the fit no better (706 moving samples: sd 0.019 m with them, 0.017 m
  without) and are left out.
  Applied only on generated maps: name under `/Game/CustomMaps/` or one of `road.DEFORMED_MAPS` (WolfsburgHighway,
  Wolfsburg2, Wolfsburg); `CARSIM_DEFORMED=0|1` overrides; never on the Town maps.
  - The ego: on a generated map the run's XML copy spawns it at the formula's surface + 0.3 m (printed:
    `Generated map: the road surface at the ego start is at z=..., the ego spawns at z=...`, `ego_z` in `run.json`),
    and the ground wait expects that height. On other maps the ego is only raised, to a `ground_projection` hit
    labelled Roads/Sidewalks/Ground/Terrain (not a tree or a prop).
  - Every spawn and `set_transform` of the blocks goes through `ctx.surface(...)` (the geometry stays OpenDRIVE-based,
    so nothing is shifted twice): occluder truck and queue (spawn and set-down), bus stop (both), main pedestrian,
    crowd (and retries), single-pass cyclists' start, waiting bicycles (spawn and set-down), the flows' gliding
    bicycles, cut-in car/van/convoy (spawn and placement), background and bubble vehicles (map spawn points: rebased
    on the lane under them + 0.5 m). Pedestrians at navigation points keep the navmesh's height.
- `reset_world` no longer touches a Traffic Manager by default (`traffic_manager_port=None`): SR's ends with SR, and a
  new one builds its road map first (over 7 minutes on a large generated map). `apps/reset_world.py --tm-port N` opts in.

### Line of sight (`src/carsim/blocks/driver.py`)
- The perception stays behind the same seam: `can_see(rays, ...)` with any object that has `cast_ray(start, end)`.
  At `initialise()` the cautious driver runs a self-test (`cast_ray_sees_actors`): one `World.cast_ray` from 3 m in
  front of the ego back to its box centre must have a labelled hit on the ego's box as its first labelled hit. Then it
  uses `World.cast_ray`; otherwise `BoxRays`: the same rays against the oriented bounding boxes of the vehicles,
  walkers and static props in CarlaDataProvider's pool (slab test, `ray_box_entry`), refreshed every tick. The choice is
  logged: `ComposedScenario[ego_control]: line of sight: World.cast_ray (the self-test ray hit the ego)` or
  `... rays against the actors' bounding boxes (World.cast_ray did not see the ego in the self-test)`.
  `CARSIM_SIGHT=cast_ray|boxes` forces one. The decision code is untouched. The self-test runs before the ego is set
  to its test speed (an asynchronous client's pose would lag otherwise).

### Sensors on large maps
- The chase view's camera (role `carsim_chase_view`, attached to the ego only) is destroyed by the scenario before the
  ego: the `ego_control` block's `cleanup()` (`sim.destroy_sensors_on`), which `ComposedScenario.terminate()` runs
  before SR destroys the ego. The metrics collision sensor already was (metrics `cleanup()`).
- `apps/chase_view.py --spectator` (`run_scenario.py --view-spectator`): no sensor at all; the server's spectator is
  moved behind the ego every frame in the same three views (`follow_transform`), so the picture is in the CARLA window
  (Moonlight), and the pygame window shows the HUD and the Restart/Quit buttons on black. The ego's pose comes from the
  world snapshot (`find`): an ego that is gone moves nothing (its handle would give a zero transform, a jump to the
  origin), and a move of more than 50 m from the spectator's place is skipped (noted once).

### WolfsburgHighway changes asked for by the placement agent
- `road.crosswalks()`: the map's 6 crosswalks are open 4-corner outlines that never repeat their first point. A
  point now closes the polygon that starts 4, 8, ... points back (the points before it are open outlines), so open
  and closed outlines mix in any order (a corner shared with an open outline does not close anything); open points
  are split into quadrilaterals (a multiple of 4) or taken as one polygon (3+). On WolfsburgHighway.xodr this gives 6
  crosswalks; Town maps are unchanged.
- `cut_in kind="parking"`: without a Shoulder right of the ego lane, the kerb Driving lane of the same direction
  (`is_kerb_lane`: no Driving lane right of it) is accepted: the car and the van stand double-parked there, logged
  ("no Shoulder right of the ego lane, the car and the blocker stand double-parked in the kerb Driving lane"). The
  error names both options. The 'merge' kind of `tools/wolfsburg/WOLFSBURG_VARIANTS.md` is not implemented.

### Offline checks
- `tests/check_crossings_offline.py`: maps from `carsim.paths.CARLA_ROOT`; `--xodr PATH` (repeatable) checks any
  OpenDRIVE file (named after the file; inner-lane occluder check too, no minimum counts); `xodr_path` also finds
  imported packages (`Content/*/Maps/<town>/OpenDrive/`) and folders in `CARSIM_XODR_DIRS`; 0.9.15 expectations
  (3 inner candidates, `KNOWN_MARGINAL` entries, also applied to inner starts); new self-tests of `road.crosswalks`
  (closed, open, both orders, closed-open-closed, a shared corner) and of `sim` (streaming after load and only when
  changed, spectator steps, the ground fallback and other labels, `destroy_sensors_on`); `check_surface_selftest`:
  `road.deformation` against 10 measured samples of elevation_run.csv (within 0.1 m), `deformed_map` names and the
  override, `surface_offset` on road and sidewalk.
- `check_driver_offline.py`: the old checks force `CARSIM_SIGHT=cast_ray` (their fake world never hits the ego); new
  `check_sight`: slab test, BoxRays order and labels, the self-test (hit on the ego, NONE, elsewhere, behind a road
  hit, none, RuntimeError), `choose_sight`, and a driver that picks the boxes by itself, does not see a child behind a
  parked truck's box and does once the truck is gone, the self-test runs before `set_target_velocity`;
  `ego_control.cleanup` removes the chase view's camera.
- `check_cut_in_offline.py`: `is_kerb_lane` cases, parking double-parked on a kerb-lane start, the new error text.
- `check_blocks_offline.py`: Town10HD-only loops skip the new Wolfsburg variants; Wolfsburg variants are built
  (mocked) on their own map when its OpenDRIVE is found (`check_other_maps`, deformed heights applied), else SKIP;
  `check_surface`: `BlockContext.surface` (Town map unchanged, shift, spawn-point rebase) and a scan of the blocks
  that fails on any `request_new_actor(` / `.set_transform(` not going through `surface(` (navigation points exempt).
- `check_chase_view_offline.py`: `follow_transform`, `SpectatorView` (follows; a gone hero and a > 50 m move do
  nothing), `--spectator` argument, `start_window` on Linux; `check_scenario_runner_flags`: every flag
  `run_scenario` passes exists in SR v0.9.15's `scenario_runner.py` and `manual_control.py` parsers (read from their
  `add_argument` calls with `ast`; `--frameRate` must not be there), the wrapper pins `set_autopilot`'s port (an
  explicit port still wins), and the driving window is started through it with `--tm-port`.
- `check_metrics_offline.py`: `write_run_config` on Python 3.8 with `ego_z`. `check_trigger_offline.py`: 3 inner starts.

## 3. On the VM (repo at `/work/car_simulator`)

### 3.1 Server
Started by the orchestrator (`vm.sh run`), port 2000. For reference: as user `carla` with `DISPLAY=:0`,
`/work/stock/CarlaUE4.sh -quality-level=Low -carla-rpc-port=2000` (Low suits the large map). Leave port 8000 alone
(web server): the Traffic Manager uses 8100.

### 3.2 One-time setup
```bash
docker exec -u carla carla bash -lc 'cd /work/car_simulator && CARLA_ROOT=/work/stock bash scripts/setup_env_linux.sh'
```
It ends with `all imports OK, carla client 0.9.15 ...`. If `external/scenario_runner` was copied over from elsewhere,
it must be tag v0.9.15 (the script says so otherwise). The repo's `CARLA_0.9.15` link (to `/work/stock`) is created
here; a copied one already points there.

### 3.3 Offline checks on the VM
```bash
docker exec -u carla carla bash -lc 'cd /work/car_simulator && export SDL_VIDEODRIVER=dummy; fail=0; for t in tests/check_*_offline.py; do python3 $t > /tmp/$(basename $t).log 2>&1 && echo "PASS $t" || { echo "FAIL $t"; fail=1; }; done; python3 apps/list_scenarios.py --ours; exit $fail'
# WolfsburgHighway geometry (find its OpenDRIVE inside the install first)
docker exec -u carla carla bash -lc 'find /work/stock -name WolfsburgHighway.xodr'
docker exec -u carla carla bash -lc 'cd /work/car_simulator && python3 tests/check_crossings_offline.py --xodr <PATH>/WolfsburgHighway.xodr'
```
The Wolfsburg variants are also built by `check_blocks_offline.py` when `xodr_path` finds the file (an imported
package, or `CARSIM_XODR_DIRS=<folder>`).

### 3.4 Runs
Windows open on `:0` (seen through Moonlight). Long runs are best started detached with a console log:
```bash
docker exec -u carla -e DISPLAY=:0 carla bash -lc 'cd /work/car_simulator && python3 apps/list_maps.py'
docker exec -u carla -e DISPLAY=:0 carla bash -lc 'cd /work/car_simulator && python3 apps/list_maps.py --load WolfsburgHighway'

# an autopilot run with the chase view (Restart / Quit in its window); --timeout 600: the first Traffic Manager start
# on the large map builds its road map for minutes
docker exec -d -u carla -e DISPLAY=:0 carla bash -lc 'cd /work/car_simulator && python3 apps/run_scenario.py pedestrian_crossing --variant PedestrianCrossing_WolfsburgHighway_1 --timeout 600 > outputs/console.log 2>&1'

# the same without a camera sensor (the spectator follows the ego in the CARLA window)
docker exec -d -u carla -e DISPLAY=:0 carla bash -lc 'cd /work/car_simulator && python3 apps/run_scenario.py pedestrian_crossing --variant PedestrianCrossing_WolfsburgHighway_1 --timeout 600 --view-spectator > outputs/console.log 2>&1'

# a single run, no window at all
docker exec -u carla carla bash -lc 'cd /work/car_simulator && python3 apps/run_scenario.py cut_in --variant CutIn_Parking_WolfsburgHighway_1 --no-view --timeout 600'

# the others: occluded_crossing --variant OccludedCrossing_Child_WolfsburgHighway_1 / OccludedCrossing_Crowd_WolfsburgHighway_1,
# cut_in --variant CutIn_Cyclist_WolfsburgHighway_1, and the five *_Town10HD_1 variants (map Town10HD_Opt)

# watch / stop / clean up
docker exec carla tail -f /work/car_simulator/outputs/console.log
# stop a run with the chase view open (also once it has ended and waits for Restart/Quit): the window's Quit, or the
# same request from outside; the scenario ends cleanly through its stop file and run_scenario resets and exits
docker exec -u carla carla bash -lc 'touch /tmp/carsim_control_*/quit.request'
# stop a --no-view run: SIGINT to ScenarioRunner (it cleans up; run_scenario sees it exit, resets and exits)
docker exec -u carla carla pkill -INT -f scenario_runner.py
# stop a manual run: close the driving window, or create the run's stop file
docker exec -u carla carla bash -lc 'touch /work/car_simulator/outputs/<run folder>/stop'
docker exec -u carla carla bash -lc 'cd /work/car_simulator && python3 apps/reset_world.py'
```
With the chase view open, `pkill -INT -f scenario_runner.py` ends the run but leaves `run_scenario.py` waiting for
Restart/Quit (use the quit request after it); a SIGINT to `run_scenario.py` alone does not reach SR (SR gets Ctrl-C
from a terminal only because it shares the terminal's process group).
Options: `--tm-port` (8100), `--stream-distance` (600), `--active-distance` (450), `--view-spectator`, `--no-view`,
`--drive manual` (opens `manual_control.py` on `:0` through `apps/manual_control_tm.py`), `--record`, `--async`
(synchronous runs are fixed at 20 Hz). `CARSIM_SIGHT=cast_ray|boxes` (in the
`bash -lc` environment) forces the driver's line of sight. Each run folder `outputs/<time>_<variant>/` has
`scenario_runner.log`, `run.json`, `metrics.json`, `chase_view.log` / `manual_control.log`.
The lines to look for: in the console, `Large-map streaming: ...`, `Generated map: the road surface at the ego
start ...` (WolfsburgHighway) or `The road surface at the ego start ...` (raised elsewhere), any `WARNING:
World.ground_projection hit nothing ... NOT verified`; in
`scenario_runner.log`, `line of sight: ...`, `removed the chase view's camera before the ego`, `parking: ... double-parked ...`.

## 4. Docker test image (local, macOS)

Recipe: `scripts/test_image.Dockerfile` (Ubuntu 20.04 amd64 with apt's python3 3.8.10 and pip 20.0.2, user `carla`;
the PyPI carla 0.9.15 cp38 wheel renamed to the VM's file name in `/work/carla/PythonAPI/carla/dist/`, because pip 20.0.2
does not know its `manylinux_2_27` tag; CARLA 0.9.15's `agents` package at `/work/stock/PythonAPI/carla/agents`; then
`scripts/setup_env_linux.sh` runs exactly as on the VM). Build from the repo root:
```bash
docker build --platform linux/amd64 -f scripts/test_image.Dockerfile -t carsim-test:0.9.15 .
```
The checks need the Town OpenDRIVE files of the CARLA 0.9.15 release (`CarlaUE4/Content/Carla/Maps/OpenDrive/*.xodr`,
Town10HD is required). They were taken from the 0.9.15 Windows release zip by HTTP range requests, without
downloading its 7.7 GB (`python3 -I remote_zip.py https://carla-releases.s3.us-east-005.backblazeb2.com/Windows/CARLA_0.9.15.zip <outdir>`
gives `<outdir>/WindowsNoEditor/CarlaUE4/...`):
```python
import io, os, sys, urllib.request, zipfile
URL, OUT = sys.argv[1], sys.argv[2]
class Remote(io.RawIOBase):  # a seekable file over HTTP Range requests
    def __init__(self, url):
        self.url, self.pos = url, 0
        self.size = int(urllib.request.urlopen(urllib.request.Request(url, method="HEAD")).headers["Content-Length"])
    def seekable(self): return True
    def readable(self): return True
    def tell(self): return self.pos
    def seek(self, off, whence=0):
        self.pos = off if whence == 0 else (self.pos + off if whence == 1 else self.size + off)
        return self.pos
    def readinto(self, b):
        n = min(len(b), self.size - self.pos)
        if n <= 0: return 0
        req = urllib.request.Request(self.url, headers={"Range": "bytes=%d-%d" % (self.pos, self.pos + n - 1)})
        data = urllib.request.urlopen(req).read()
        b[:len(data)] = data; self.pos += len(data); return len(data)
z = zipfile.ZipFile(io.BufferedReader(Remote(URL), buffer_size=1 << 20))
for info in z.infolist():
    if info.filename.endswith(".xodr") and "/Maps/" in info.filename:
        dest = os.path.join(OUT, info.filename)
        os.makedirs(os.path.dirname(dest), exist_ok=True)
        with z.open(info) as src, open(dest, "wb") as f: f.write(src.read())
```
Run every offline check (repo and maps mounted; WolfsburgHighway.xodr from
`car_simulator_inputs`):
```bash
cd /Users/bogdandavid/Coding/CARLA/car_simulator
docker run --rm --platform linux/amd64 -v "$PWD":/work/car_simulator \
  -v <outdir>/WindowsNoEditor/CarlaUE4:/work/stock/CarlaUE4:ro \
  -v /Users/bogdandavid/Coding/CARLA/car_simulator_inputs:/inputs:ro -e CARSIM_XODR_DIRS=/inputs \
  carsim-test:0.9.15 bash -c 'fail=0; for t in tests/check_*_offline.py; do python3 $t > /tmp/o 2>&1 && echo "PASS $t" || { echo "FAIL $t"; tail -20 /tmp/o; fail=1; }; done; python3 tests/check_crossings_offline.py --xodr /inputs/WolfsburgHighway.xodr | tail -3; python3 apps/list_scenarios.py --ours; exit $fail'
```
(The repo's git-ignored `CARLA_0.9.15` link points to `/work/stock`, valid inside the container and on the VM.)

Result at hand-off (after the review fixes): all 8 checks PASS (`check_blocks`, `check_chase_view`, `check_crossings`, `check_cut_in`,
`check_driver`, `check_flows`, `check_metrics`, `check_trigger`); `check_crossings --xodr WolfsburgHighway.xodr`
PASSED (6 crosswalks, 2 candidates, 0 failures); `list_scenarios.py --ours` lists all 10 variants without problems;
SR v0.9.15 `scenario_runner.py --list`, `manual_control.py --help` and every app's `--help` run.

## 5. Only verifiable live (for the orchestrator)

1. `setup_env_linux.sh` in the real container: the VM wheel installs with pip 20.0.2, the pinned packages install
   (no compiler needed), the import check prints client 0.9.15.
2. After `list_maps.py --load WolfsburgHighway` (and any `run_scenario` load): the `Large-map streaming` line, and the
   server survives the load with 600/450 m (try lower values with `--stream-distance/--active-distance` if not).
3. The spectator's stepped travel to a far start (and `find_crossing_spots.py --tour`) does not crash Vulkan.
4. `wait_for_ground` on WolfsburgHighway: does `ground_projection` hit the road (which label?) or fall back to the
   10 s settle with the warning; is the map recognised as generated (the `Generated map:` line; its server name should
   be under `/Game/CustomMaps/`, else it is matched by `DEFORMED_MAPS`), and does the ego spawn at the printed z and
   drive normally.
5. Spawns on the deformed road, now at the formula's surface: the occluder truck, its queue car and the bus stop, the
   cut-in car, van and cyclist convoy, pedestrian, crowd, cyclists waiting on the sidewalk (physics off, 0.05 m over
   the sidewalk mesh) and gliding onto the road, background traffic: no "could not spawn" / "Failed to spawn", nobody
   stuck in or floating over the road or the kerb.
6. The driver's `line of sight:` line on Town10HD (expected: World.cast_ray) and on WolfsburgHighway; with boxes,
   whether `first visible` / braking happen at sensible distances (buildings do not occlude in that mode); with
   cast_ray on Wolfsburg, whether the waving road mesh blocks the eye's rays (then force `CARSIM_SIGHT=boxes`).
7. The chase-view camera attached to the hero on WolfsburgHighway: no crash during the run, and
   `removed the chase view's camera before the ego` at the end, Restart and Quit included. If the camera crashes the
   server, use `--view-spectator` and check the spectator follows the ego smoothly in all three views (C).
8. The metrics collision sensor on the hero on WolfsburgHighway: no crash, `impact` filled in `metrics.json`.
9. Traffic Manager on 8100: background vehicles move (sync mode), how long the first start takes (raise `--timeout`
   if SR's watchdog or the client timeout fires), and `reset_world.py` afterwards returns quickly.
10. Windows on Linux: chase view and `--drive manual` driving window open on `:0`, their logs are written; the
    driving window stays open (no Traffic Manager bind error on 8000: it uses 8100 through the wrapper, look in
    `manual_control.log`); the stop ways of 3.4 end the run cleanly, `run.json` has the `end_reason`.
11. `manual_control.py`'s own sensors (camera, collision, lane invasion, GNSS, IMU on the hero) are not guarded by the
    pre-ego cleanup: check a manual run on WolfsburgHighway ends without a crash.
12. `--record` writes `<variant>.log` into the run folder on Linux and `apps/replay.py` plays it.
13. Town10HD regression: the five `*_Town10HD_1` variants still pass as on 0.9.16 (cast_ray line of sight, chase view).
14. On the live map, `get_crosswalks()` gives the same open outlines as the xodr (the crossing of
    PedestrianCrossing_WolfsburgHighway_1 should log no "no painted crosswalk found").
15. CutIn_Parking_WolfsburgHighway_1: the double-parked van and car in the kerb lane, the pull-out into the ego lane;
    background traffic in that kerb lane has to stop behind them.
16. Without a pedestrian navmesh (a map before tools/navmesh installs one): the pedestrians stand on the far kerb
    after crossing, no background walkers (logged), and nothing fails.

## 5b. Live results on the VM (2026-11-10, first WolfsburgHighway build, autopilot, `--no-view`)

| Variant | Result | Metrics |
|---|---|---|
| PedestrianCrossing_WolfsburgHighway_1 | passed | no impact, walker first visible 47 m ahead, brake onset TTC 1.8 s, min distance 4.3 m |
| OccludedCrossing_Child_WolfsburgHighway_1 | passed (after fix 1) | no impact, brake onset TTC 1.4 s, min distance 5.1 m |
| OccludedCrossing_Crowd_WolfsburgHighway_1 | server froze (fix 3) | to rerun on the second build |
| CutIn_Parking_WolfsburgHighway_1 | passed | no impact, brake onset TTC 2.9 s, min distance 6.8 m |
| CutIn_Cyclist_WolfsburgHighway_1 | passed without background walkers; with them the server crashed (fix 3) | no impact, first visible TTC 1.5 s, brake onset TTC 1.1 s, min distance 3.8 m |

`line of sight` chose the bounding-box rays on this map: `World.cast_ray` did not see the ego in the self-test.

Fixes made by the orchestrator from these runs:

1. **Spawns refused on the deformed surface.** The european_hgv was refused at the computed surface + 0.2 m and spawned
   at + 0.5 m. `BlockContext.request_on_surface` retries a refused spawn 0.3 m and 0.8 m higher on deformed maps; the
   occluder, its prop, the cut-in actors and the background vehicles use it (the blocks set vehicles down afterwards).
2. **Keep every tile loaded on WolfsburgHighway** (`sim.KEEP_LOADED`, 3000/3000 m): streaming crashed the server in
   Vulkan buffer creation while a car moved around the map. Only an explicit `--stream-distance`/`--active-distance`
   overrides it.
3. **Walkers crashed or froze the server** (`FWalkerActor::SetWalkerState` on a walker without a root component) when
   background walkers were spawned on navmesh polygons of sidewalks that lie across driving lanes. Reproduced with a
   plain script and no ScenarioRunner; with sidewalk-only points 15 walkers ran fine. The second map build turns those
   47 sidewalk lanes into borders and the navmesh is rebuilt from it (tools/navmesh).
4. **A ScenarioRunner left over after a server crash keeps port 8100**: the next run then fails with "trying to create rpc
   server for traffic manager ... bind error". Kill it first:
   `docker exec carla pkill -9 -f "[s]cenario_runner.py"`.

## 5c. Final live results (2026-11-10 evening, second WolfsburgHighway build, after the fixes below)

| Variant | Result | Metrics |
|---|---|---|
| PedestrianCrossing_WolfsburgHighway_1 | passed | no impact, brake onset TTC 1.9 s, min distance 4.3 m |
| OccludedCrossing_Child_WolfsburgHighway_1 | passed | no impact, brake onset TTC 1.4 s, min distance 5.1 m |
| OccludedCrossing_Crowd_WolfsburgHighway_1 | passed (no background traffic, see its XML) | no impact, first visible TTC 2.1 s, min distance 3.1 m |
| CutIn_Parking_WolfsburgHighway_1 | passed | no impact, brake onset TTC 2.9 s, min distance 6.8 m |
| CutIn_Cyclist_WolfsburgHighway_1 | passed (no background walkers, see its XML) | no impact, first visible TTC 1.5 s, brake onset TTC 1.1 s, min distance 3.8 m |

Fixes after 5b, from the live runs and the `/code-review` (high) of this branch:

5. **Walkers off the navmesh.** `BlockContext.navigation_point` draws up to 20 navigation points and returns only one on
   a Sidewalk lane (deformed maps); no AI controller is started without a destination (a started one picks any point
   itself); and a pedestrian that ends its crossing off the sidewalk (this map's 10 m medians) gets no AI controller:
   `WalkOn` walks it straight on until it is off every driving lane and `CLEAR_AFTER` (2.5 m) further, then it stands.
   Before, standing on the road, it stayed a crosser for ever and the cautious driver waited until the timeout.
6. **Leaving bicycles** no longer get the surface offset twice (`flow._start_leaving`).
7. **`wait_for_ground`**'s fallbacks (any label after 15 s, no hit at all) only apply on generated maps; elsewhere no
   hit means an unloaded tile, so it waits for the timeout as before.
8. **`--stream-distance` / `--active-distance`** default to None: an explicit value always wins; with neither,
   `sim.KEEP_LOADED` (3000 m on WolfsburgHighway) or the large-map defaults apply.
9. The driving-window hint prints `python -m uv run python` on Windows and `python3` on Linux.
10. Two variants' background was adapted in their XML here (crowd: no background traffic; cyclist: no background
    walkers). Both are back to the original backgrounds since the third build (5d).

Not done from the review (low): the box line of sight rebuilds every actor's box each tick; a little duplicated
offset logic in `chase_view.py` and `run_scenario.py`; `sim.py`'s import-time self-checks.

## 5d. Third WolfsburgHighway build (2026-11-11 00:40 UTC), original backgrounds in every variant

The third build levels the junction heights (no lane lies over another road's lane any more), adds guard rails on
raised roads, and fixes the map's large-map tile lookup (digital_twin GUIDE Stage 9). That lookup was the cause of the
walker crashes: the tool leaves `Tile0Offset` at 0, so the manager looked every location up half a tile off and the
last 250 m in the north and east counted as a tile that does not exist. Any actor there went dormant at once, and
stock 0.9.15 crashes when the client's walker navigation moves a dormant walker (`FWalkerActor::SetWalkerState`
dereferences the missing actor before its dormancy check, CarlaActor.cpp:1221). The cyclist variant's 120 m radius
reaches into the north strip. With the lookup fixed, both XML adaptations of 5c are removed.

| Variant | Background | Result | Metrics |
|---|---|---|---|
| PedestrianCrossing_WolfsburgHighway_1 | 10 cars, 3 motorcycles, 1 of 3 bicycles (spawn points ran out), 20 walkers | passed | no impact, brake onset TTC 1.9 s, min distance 4.3 m |
| OccludedCrossing_Child_WolfsburgHighway_1 | 8, 2, 2, 15 | passed | no impact, min distance 5.1 m |
| OccludedCrossing_Crowd_WolfsburgHighway_1 | 8, 2, 2, 15 | passed | no impact, first visible TTC 2.3 s, brake onset TTC 2.3 s, min distance 3.0 m |
| CutIn_Parking_WolfsburgHighway_1 | 8, 2, 2, 15 | passed | no impact, brake onset TTC 2.9 s, min distance 6.8 m |
| CutIn_Cyclist_WolfsburgHighway_1 | 8, 2, 2, 15 | passed | no impact, first visible TTC 1.5 s, brake onset TTC 1.1 s, min distance 3.8 m |

The server stayed up through all five. Offline checks on the VM: all 8 PASS, `check_crossings --xodr` on the
installed WolfsburgHighway.xodr PASSED, `list_scenarios.py --ours` without problems. The navmesh in
`tools/navmesh/out/WolfsburgHighway.bin` is built from this build's road file (rebuild it whenever the road file
changes).

## 5e. Fourth WolfsburgHighway build (2026-11-11 11:26 UTC), flat roads

Cars rocked on straight roads and jolted at road edges (digital_twin GUIDE Stage 9, "Cars rocked and jolted"). The
stock Digital Twin Tool bakes 0.7 x `GetZPosInDeformation` waves (up to 1.2 m) into every road mesh, and leaves the
ground 80 cm lower beside every road without a sidewalk. The fourth build patches both out (`apply_patches.py`): the
road mesh now lies at the OpenDRIVE heights and the ground at most 0.12 m under an open lane edge; guard rails start
at 0.15 m road height. The road file is unchanged (md5 adb47ff2…).

What changed here: `road.FLAT_MAPS = ("WolfsburgHighway",)`. `road.deformation(x, y, name)` and so
`surface_offset` give no wave offset on it, and the sidewalk lift (0.3024 m) stays; WolfsburgHighway still counts as
a generated map (`deformed_map`). Wolfsburg2 keeps the waves. `tools/navmesh/build_navmesh.py` builds flat for the
same maps; `out/WolfsburgHighway.bin` is rebuilt (md5 a3d353aa…, installed; `check_navmesh_live.py`: 10 of 10 walkers
moved, 91 % of samples on sidewalks). Do not run this branch against a WolfsburgHighway package of builds 1 to 3:
spawns there would be up to 0.84 m off.

| Variant | Background | Result | Metrics |
|---|---|---|---|
| PedestrianCrossing_WolfsburgHighway_1 | 10 cars, 3 motorcycles, 1 of 3 bicycles, 20 walkers | passed | no impact, brake onset TTC 2.1 s, min distance 4.3 m |
| OccludedCrossing_Child_WolfsburgHighway_1 | 8, 2, 2, 15 | passed | no impact, min distance 5.1 m |
| OccludedCrossing_Crowd_WolfsburgHighway_1 | 8, 2, 2, 15 | passed | no impact, brake onset TTC 2.3 s, min distance 3.6 m |
| CutIn_Parking_WolfsburgHighway_1 | 8, 2, 2, 15 | passed | no impact, brake onset TTC 2.8 s, min distance 6.7 m |
| CutIn_Cyclist_WolfsburgHighway_1 | 8, 2, 2, 15 | passed | no impact, first visible TTC 1.7 s, brake onset TTC 1.7 s, min distance 4.4 m |

Every start reported the road surface at z = 0.00 (the ego spawns 0.30 m above it). The server stayed up. Offline
checks on the VM: all 8 PASS with this change.

## 5f. Traffic lights and speed signs fixed (2026-11-12)

`digital_twin/signals_check.py` lists the lights and signs the server spawns from the road file and drives autopilot
(Traffic Manager) cars past them. It found two defects, both fixed on the VM without repackaging:

- **One light was run.** netconvert split the Herzbergweg / Salzwedeler Strasse junction in two with a 1.5 m stub road
  between, and CARLA puts a light's trigger box on the road before its junction, so cars left the first junction at
  34 km/h, saw red 2.3 m past the line and stopped 4.2 m inside the junction (then stayed stuck after green).
  `digital_twin/move_lights.py` (`vm.sh lights WolfsburgHighway`) moves such stop lines to the end of the feeding road:
  lights 312 (Herzbergweg) and 308 (Suhler Strasse, 4.1 m stub; it passed, but cars stopped up to 1.5 m past its line).
  It also drops the yield sign at the new Herzbergweg stop line, whose trigger could turn a car waiting at red green.
  The light poles stay where they were. Installed road file and `car_simulator_inputs/WolfsburgHighway.xodr`: md5
  e9d03dfa… (was adb47ff2…, kept as `car_simulator_inputs/v4/` and `/work/backup/WolfsburgHighway_before_lights.xodr`).
  Only signals changed: geometry, navmesh and spots are as before.
- **100 km/h signs gave 90.** CARLA 0.9.15's `BP_SpeedLimit100/110/120` blueprints carry their own trigger with 90
  (copied from `BP_SpeedLimit90`), which fired after the road file's 100. `digital_twin/fix_speed_signs.py`
  (`vm.sh speedsigns`) changes that one float in `/work/stock` (`.orig` copies kept). The server still reports those
  signs as `traffic.speed_limit.90`; the limit they set is right.

Result after both fixes: 9 of 9 lights stop the car before the junction on red and let it go on green, at its own
limit and at 50 km/h; 3 of 3 stop signs make a full stop and a 2.2 s wait; on a 1.6 min drive from the A 39 every
limit change (30 -> 100 -> 60 -> 50 -> 80 -> 100) happened 5 to 7 m from a sign of that value, and the car never went
over the limit (Traffic Manager drives at about 96 % of it). Not fixed: a car starts at 30 km/h until it passes its
first sign (the road file has no road speed records), and yield signs are untested (they only act with cross traffic).

## 6. Open points

1. Town10HD occluded crossing on 0.9.15: the offline occluder check of the unchanged Child start (62.49, -60.92)
   finds the bus stop and the truck leave the child's waiting spot in sight 25 m before the crossing (the XML comment
   says it was live-tuned on 0.9.16). Not changed; check live (item 13) before re-tuning.
2. Nine 0.9.15 Town candidates (Town03/04/05 cyclists or crowd, Town10HD inner-lane occluder geometry), none used by a
   scenario, miss a check; they are listed in `KNOWN_MARGINAL` instead of re-tuning `road.py` geometry.
3. `BoxRays` sees only the scenario's actors (CarlaDataProvider's pool), not buildings, terrain or other clients'
   actors: on a map without working `cast_ray`, corner occlusion by buildings is missing.
4. The surface formula is the generated mesh's; collision may differ slightly (bumps <= 0.1 m are left out). Moves
   other than `set_transform` (KeepVelocity rides, AI walkers) are physics on the mesh. Walkers spawned at navigation
   points rely on the navmesh's heights (tools/navmesh).
5. The Windows/uv path (pyproject, uv.lock, setup_env.py, start_carla.ps1) was moved to 0.9.15 / Python 3.10 but not run.
