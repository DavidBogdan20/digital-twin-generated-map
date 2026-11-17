# car_simulator

Custom driving scenarios for CARLA, run with ScenarioRunner. Each scenario lives in its own folder under `scenarios/`; small command-line apps in `apps/` list maps and scenarios, pick start positions and run a scenario.

> **CARLA 0.9.16 on Windows and Linux.** On Linux (a Docker container with Ubuntu 22.04 and Python 3.10, CARLA at `/work/stock`) run `bash scripts/setup_env_linux.sh` once, then `python3 apps/<app>.py` where this manual says `python -m uv run python apps\<app>.py`. This copy adds the large generated map WolfsburgHighway; [`PORTING.md`](PORTING.md) lists what differs from upstream. The old CARLA 0.9.15 port is git tag `carla-0.9.15`.

## Folder structure

```
car_simulator/
├── CARLA_0.9.16/              Unzipped CARLA Windows release (git-ignored, several GB)
├── external/
│   └── scenario_runner/       ScenarioRunner, tag v0.9.16 (cloned by the setup script, git-ignored)
├── scripts/
│   ├── setup_env.py           One-time setup: clone ScenarioRunner, uv sync into .venv, import paths
│   ├── setup_env.ps1          PowerShell wrapper for setup_env.py
│   ├── setup_env_linux.sh     The same on Linux (Ubuntu 22.04 container, Python 3.10): pip --user, no venv
│   ├── test_image.Dockerfile  Docker image that mirrors the Linux container, for the offline checks
│   └── start_carla.ps1        Starts the CARLA server
├── apps/                      Command-line tools
│   ├── list_scenarios.py      List our scenarios and ScenarioRunner's built-in ones (no server needed)
│   ├── list_maps.py           List the maps the server can load, or load one
│   ├── reset_world.py         Put the server back to normal after a stopped run (async mode, no leftover actors)
│   ├── find_crossing_spots.py Find valid ego start positions (kerb lane, or `--lane inner`) and write one into the scenario XML
│   ├── run_scenario.py        Load the map, run a scenario through ScenarioRunner, save results; opens the driving window for a manual run, the chase view (with Restart and Quit) for an autopilot run
│   ├── manual_control_tm.py   Starts ScenarioRunner's manual_control.py with its Traffic Manager on --tm-port (the driving window)
│   ├── chase_view.py          The chase-view window of an autopilot run: camera on the ego, a HUD, and the Restart / Quit buttons (never controls or ticks)
│   └── replay.py              Replay a recorded run on the running server
├── scenarios/
│   ├── _composed/             The one generic scenario class, shared by all scenarios (has no XML, so it is not listed)
│   │   └── composed_scenario.py    ComposedScenario: builds the scenario from the blocks named in the XML
│   ├── pedestrian_crossing/   One folder per scenario; its XML is all it needs
│   │   ├── pedestrian_crossing.xml    ScenarioRunner config, one <scenario> per variant, blocks as child elements
│   │   └── README.md                  Blocks and behavior tree of this scenario
│   ├── occluded_crossing/     A truck in the kerb lane hides a child or a crowd from the cautious ego (child and crowd variants)
│   └── cut_in/                A parked car or a cyclist pulls into the ego lane (parking and cyclist variants)
├── src/carsim/                Shared helpers used by apps and scenarios
│   ├── paths.py               Locations of CARLA, ScenarioRunner, scenarios/, outputs/
│   ├── sim.py                 Connect to the server, load maps, reset sync mode, move the spectator, wait for ground
│   ├── road.py                Crosswalk, junction and sidewalk lookups on the map; pedestrian, crowd and cyclist geometry
│   ├── scenario_files.py      Read variants from a scenario's XML, write the ego pose into it, validate names and blocks
│   ├── runner.py              Start scenario_runner.py (and the manual driving window) as a subprocess with the right paths
│   ├── run_config.py          Per-run files: weather and driver overrides, the scenario XML copy, the stop file, run.json
│   └── blocks/                The scenario blocks (see "Scenario blocks")
│       ├── spec.py            Block names, order and attributes (plain data, used by validation)
│       ├── base.py            The Block interface and the BlockContext the blocks share
│       ├── flow.py            The endless flows of a manual run (pedestrians, cyclists, background traffic)
│       ├── driver.py          The cautious driver (`ego_control brake="sight"`)
│       └── ego_control.py, spawning.py, run_control.py, occluder.py, pedestrian.py, crowd.py, cyclists.py, cut_in.py, background.py, end.py    One block each
├── tests/                     Offline checks (no CARLA server needed)
│   ├── check_crossings_offline.py    Crossing, crowd and cyclist geometry on every installed map (`--xodr PATH` for another OpenDRIVE file)
│   ├── check_blocks_offline.py       Block registry, scenario files, validation and a mocked build of every variant
│   ├── check_flows_offline.py        The endless flows of a manual run
│   ├── check_driver_offline.py       The cautious driver: line of sight, crossers, braking decisions
│   ├── check_cut_in_offline.py       The cut_in block: actors, lanes, hiding, the behavior tree
│   └── check_chase_view_offline.py   The chase view's HUD text, nearest actor ahead, time to collision, buttons, banner, request files and the run loop's decision
├── outputs/                   Run results, one folder per run (git-ignored)
├── pyproject.toml             Dependencies (carla from PyPI, ScenarioRunner's requirements) and the carsim package
├── uv.lock                    Locked versions, installed by uv sync
└── .python-version            Python 3.11
```

## One-time setup

1. Have any Python and git on PATH. The project's own Python (3.11) is installed by the setup through [uv](https://docs.astral.sh/uv/), with no admin rights needed. Python 3.12+ does not work for the venv: ScenarioRunner pins `numpy==1.24.4`, which has no 3.12 build, and the wheel bundled with CARLA is cp312 only. That is why `carla==0.9.16` is installed from PyPI instead.

2. Unzip the CARLA 0.9.16 Windows release into `CARLA_0.9.16\`.

3. Run the setup from a terminal where `.venv` is not activated. It clones ScenarioRunner v0.9.16 into `external\scenario_runner`, installs uv if missing (`pip install --user uv`), installs Python 3.11 and runs `uv sync`, which creates `.venv` from `pyproject.toml` and `uv.lock` and installs the `carsim` package (editable) into it. It then writes a `.pth` file that adds CARLA's `PythonAPI\carla` (the `agents` package) and ScenarioRunner to the venv's import path. It is safe to re-run.

```powershell
python scripts\setup_env.py
```

`scripts\setup_env.ps1` is a PowerShell wrapper around the same thing. Options: `--recreate` rebuilds `.venv`, and `--python 3.10` uses 3.10 instead of 3.11. If Windows blocks `.ps1` scripts ("running scripts is disabled on this system"), run the wrapper with the execution policy bypassed for that one call:

```powershell
powershell -ExecutionPolicy Bypass -File D:\GIT\car_simulator\scripts\setup_env.ps1
```

4. Run the project's commands through uv, from the repository folder, as in this README. uv uses `.venv` without activating it (no `Activate.ps1`, so the execution policy doesn't matter):

```powershell
python -m uv run python apps\list_maps.py
```

If you prefer an activated venv, run `.venv\Scripts\Activate.ps1` and drop the `python -m uv run` prefix. If the activation script is blocked, first run `Set-ExecutionPolicy -Scope Process Bypass`, which only affects the current terminal.

To add a Python dependency, use `uv add <package>` (it updates `pyproject.toml` and `uv.lock`), not `pip install`.

## Daily workflow

1. Start the CARLA server. The default is Low quality, which suits large maps such as Town12. Use `-Quality Epic` or `-OffScreen` if needed.

```powershell
.\scripts\start_carla.ps1
```

If Windows blocks `.ps1` scripts, bypass the execution policy for this one call:

```powershell
powershell -ExecutionPolicy Bypass -File D:\GIT\car_simulator\scripts\start_carla.ps1
```

2. List the maps the server knows. The scenario's map must be in the list: `Town10HD_Opt` (always installed):

```powershell
python -m uv run python apps\list_maps.py
```

3. Choose where the ego vehicle starts. This loads the variant's map (`Town10HD_Opt`) and prints numbered candidates with x, y, z, yaw. The numbers are the same in every option below as long as you keep the same `--near`, `--seed`, `--count` and `--approach` (each row prints a command that reproduces its listing). Pick one by looking at it:

   - **Map:** saves `outputs\crossing_candidates.png`, a top-down map of the variant's town with every candidate numbered.
   - **Tour:** flies the camera through every candidate in turn (8 s each, change with `--tour 15`) and labels each one in the world with its number.
   - **Show one:** moves the camera above candidate N and marks it.

```powershell
python -m uv run python apps\find_crossing_spots.py --plot
```

```powershell
python -m uv run python apps\find_crossing_spots.py --tour
```

```powershell
python -m uv run python apps\find_crossing_spots.py --show 3
```

In the world, green is the ego start with an arrow along its drive direction, and red is the pedestrian's walk. The camera looks steeply down with the ego driving towards the top of the screen. The markers disappear by themselves (3 minutes for `--show`). Then write your choice into the XML:

```powershell
python -m uv run python apps\find_crossing_spots.py --apply 3
```

Use `--near X Y` to sort candidates by distance to a point, and `--variant NAME` to pick a variant other than the first. `run_scenario.py` refuses to start while the ego start in the XML is still (0, 0). For a variant with an `occluder`, add `--lane inner` to every `find_crossing_spots.py` command (listing, `--show`, `--apply`): the ego must drive in the lane left of the kerb lane, where the truck stands. `cut_in` has no spot finder: its start is written by hand (see `scenarios\cut_in\README.md`).

4. Run the scenario:

```powershell
python -m uv run python apps\run_scenario.py pedestrian_crossing
```

Before handing over to ScenarioRunner this checks the scenario's naming rules (see "Adding a new scenario"), loads the map, puts the CARLA window's camera 12 m behind and 6 m above the ego start, looking along its way (fly freely from there; it goes back there at each restart), and waits until the ground there has loaded (large-map tiles can take a while). It also refuses to start while another scenario is still running on the same server (the world is synchronous and keeps advancing): end that run first, or run `apps\reset_world.py` if it hangs. Add `--variant NAME` to pick a variant and `--debug` to draw the crossing and print the behavior tree. An autopilot run is synchronous by default: ScenarioRunner's fixed 0.05 s steps (`--frame-rate N` changes the step), repeatable, and every step is computed whatever the PC's speed, so it runs as fast as the PC allows (about 2.8 times real time on Town10HD without the chase view), not at normal speed. `--async` runs it in real time instead (less repeatable; a slow client then sees a lagging world). A manual run is real time (asynchronous) by default; `--sync` forces synchronous mode. `--drive` picks who drives the ego (next step). `pedestrian_crossing` also spawns a crowd of 10 more pedestrians and 2 cyclists crossing from both sides plus background cars, motorcycles, bicycles and pedestrians (the `<crowd>`, `<cyclists>` and `<background>` blocks in the XML; see `scenarios\pedestrian_crossing\README.md`). The scenario has both pedestrians and cyclists, so it is the one to run for either.

5. Who drives the ego: `--drive xml|autopilot|manual` (default `xml`: the variant's `<ego_control mode>`). `autopilot` lets the scenario drive and `manual` lets you drive, whatever the XML says; the choice is applied to the `scenario.xml` copy of the run and the XML itself is not changed. With `--drive manual`, `run_scenario.py` starts ScenarioRunner's `manual_control.py` for you, in its own console and pygame window, as soon as ScenarioRunner reports that the scenario is running (the ego exists by then). Click into the pygame window and drive with W/A/S/D (Q toggles reverse, Space is the handbrake). A manual run has no time limit and no distance end: **it ends when you close the driving window** (Esc or its X) **or press Ctrl-C in the scenario console**. Closing the window makes `run_scenario.py` create a `stop` file in the run folder; the scenario sees it, ends cleanly and saves its results, then the world is reset as after any run. During a manual run pedestrians and cyclists keep crossing in a continuous flow, background traffic follows the car, and nothing spawns or disappears where you could see it: within 120 m straight ahead, within 50 m to the sides, or close behind the car while you reverse (see "Scenario blocks"). The window is started with `--keep_ego_vehicle`, so closing it never removes the car before ScenarioRunner has finished. Do not use the window's Ctrl+R (it starts `manual_control.py`'s own recorder, replacing `--record`), Ctrl+P (it replays a recording mid-run) or P (it switches CARLA's autopilot on). Closing the terminal itself kills `run_scenario.py` before it can end the run: close the window by hand then, and run `python -m uv run python apps\reset_world.py`. `--drive autopilot` stays a single pass that ends by itself, and opens the chase view (see "Chase view": its Restart button runs it again in the same process, its Quit button exits; `--no-view` skips the window and runs once). `--drive autopilot` and `--drive manual` need a scenario built from blocks (a folder without a `.py` of its own); a custom scenario file is run with `--drive xml` only.

```powershell
python -m uv run python apps\run_scenario.py pedestrian_crossing --drive manual
```

A manual run has no timeout. Do not combine `--drive manual` with `--sync` (it prints a warning): the simulation then does not run in real time. With `--no-driver-window`, nothing is started and no stop file is written: `run_scenario.py` prints the command for a second terminal ("Start manual_control.py yourself with: ...; press Ctrl-C here to end the run"), and the run ends with Ctrl-C (`--host` and `--port` are added when they are not the defaults). `apps\manual_control_tm.py` runs ScenarioRunner's `manual_control.py` with its `set_autopilot` calls on the scenario's Traffic Manager port (CARLA would make a Traffic Manager on 8000 otherwise):

```powershell
python -m uv run python apps\manual_control_tm.py --tm-port 8100 --keep_ego_vehicle
```

6. Results (ScenarioRunner output, JSON, JUnit XML and `scenario_runner.log`) are in `outputs\<time>_<variant>\`, for example `outputs\20261027_141500_PedestrianCrossing_Town10HD_1\`.

## How do I see available scenarios?

This lists the scenarios and variants in `scenarios\` plus ScenarioRunner's built-in examples. It does not need a server. Add `--ours` to show only ours.

```powershell
python -m uv run python apps\list_scenarios.py
```

The raw ScenarioRunner equivalent, run from its own folder (uv still finds the project's `.venv`):

```powershell
cd external\scenario_runner
```

```powershell
python -m uv run python scenario_runner.py --list
```

## How do I see the maps?

A CARLA server must be running. List the maps it can load:

```powershell
python -m uv run python apps\list_maps.py
```

Load one (this can take minutes for Town12 on first load):

```powershell
python -m uv run python apps\list_maps.py --load Town12
```

CARLA's own tool does the same:

```powershell
python -m uv run python CARLA_0.9.16\PythonAPI\util\config.py --list
```

## VS Code tasks

Open the Command Palette and choose **Tasks: Run Task** (or Terminal > Run Task). The tasks in `.vscode/tasks.json` are grouped by prefix and run `python -m uv run python ...` from the repository root in PowerShell.

| Task | What it does |
|---|---|
| CARLA: Start server | `scripts\start_carla.ps1` (own panel) |
| CARLA: Reset server | `apps\reset_world.py`: back to async mode, removes leftover actors |
| CARLA: List maps | `apps\list_maps.py` |
| Scenario: List | `apps\list_scenarios.py` |
| Scenario: Run | `apps\run_scenario.py`; asks for the scenario and variant (one list of every variant), who drives the ego (scenario XML, autopilot, or manual: you drive in the driving window, there is no time limit, and closing the window ends the run), the weather, whether to record, and the simulation speed (real time or `--sync`) |
| Scenario: Run (debug drawing) | the same with `--debug` |
| Spots: List / Map image (plot) / Tour | `apps\find_crossing_spots.py` with nothing / `--plot` / `--tour` |
| Spots: Show N / Apply N | `--show N` / `--apply N`; asks for the scenario, the variant and the candidate number N |
| Replay: Last run | `apps\replay.py` |
| Setup: Environment | `python scripts\setup_env.py` |

The driving options in the Scenario: Run prompt are `--drive xml|autopilot|manual`; there is no separate manual-driving task. A manual run ends when you close the driving window. The simulation mode prompt is "default" (synchronous for autopilot, asynchronous for manual driving), synchronous (`--sync`: fixed steps, same result every run) or asynchronous (`--async`: real time). The weather list in the Scenario: Run prompt is `xml` (keep the weather from the scenario XML) plus the CARLA presets. The variants offered by Scenario: Run are the `runVariant` input in `tasks.json` (scenario folder and variant together); add each new variant there. The Spots tasks pick the scenario folder (`scenario` input) and the variant (`variant` input, crosswalk variants only) separately.

## Recording and replay

`--record` records a run with CARLA's recorder into the run folder, and `--weather` overrides the weather with a CARLA preset (`xml`, the default, keeps the XML's `<weather>`):

```powershell
python -m uv run python apps\run_scenario.py pedestrian_crossing --weather HardRainNight --record
```

`--record` also works with `--drive manual`: the recording then holds your own drive. Real time (the default) is what you see in the replay at `--speed 1`; a `--sync` run records fixed 0.05 s steps.

Each run folder `outputs\<time>_<variant>\` holds:

- `scenario.xml`: the copy of the scenario XML that ScenarioRunner ran, with the `--weather` and `--drive` overrides applied;
- `run.json`: scenario, variant, town, weather values, who drove (`drive`: requested and effective), sync mode, frame rate, record path, the stop-file path of a manual run, how the run ended (`end_reason`: `stop_file` = you closed the driving window, `restart` or `quit` = you pressed the chase view's button, `ctrl_c`, or `scenario_end`), the exact ScenarioRunner command, start time and random seeds;
- the results (the results JSON `<variant><timestamp>.json`, JUnit `.xml`, `scenario_runner.log`) and, with `--record`, the recording `<variant>.log`. ScenarioRunner's `--record` also writes `<variant>.json` next to the recording: a dump of the scenario's criteria, not the results JSON (the name differs by the timestamp, so look at it before opening the wrong one).

The server writes the recording, so it must run on the same machine. Replay the newest recorded run on the running server, with the camera following the ego:

```powershell
python -m uv run python apps\replay.py
```

Other options: a run folder (`apps\replay.py outputs\<run folder>`), `--start 5 --duration 20`, `--speed 0.5`, and `--no-follow` for a free camera. `replay.py` loads the recorded map if needed and switches the server to asynchronous mode. It sets the playback speed, the hero (played as recorded) and the spectator (left alone) on every replay, before the replay starts, as CARLA's `start_replaying.py` does: the server keeps these settings between replays, so a `--speed 2` would otherwise stay. The cyclists are in the recording from the first frame, waiting on the sidewalk with physics off, and roll onto the road at their turn. The replayed actors stay on the server when playback ends. `run_scenario.py` removes them (and any other leftover scenario actors) before it starts a run; to clear them sooner, use **CARLA: Reset server** (`apps\reset_world.py`).

## Scenario blocks

Scenarios are composed from blocks. A block is a small class in `src\carsim\blocks\` keyed by an XML tag; every child element of a `<scenario>` whose tag is a block name switches that block on and carries its parameters (as strings, in `config.other_parameters[tag]`). The generic scenario class `ComposedScenario` (`scenarios\_composed\composed_scenario.py`) builds the blocks, spawns their actors and puts their behaviors into one behavior tree. `ego_vehicle` and `weather` are ScenarioRunner's own elements, not blocks.

Rules:

- Blocks always run in this order: `ego_control`, `spawning`, `run_control`, `occluder`, `pedestrian`, `crowd`, `cyclists`, `cut_in`, `background`, `end`, whatever the order in the XML.
- Each tag can appear **once** per variant (ScenarioRunner keeps one entry per tag). `ego_control` and `end` always exist: without an element they run with their defaults. The other blocks exist only when their element does.
- An unknown tag or attribute is a validation problem (`list_scenarios.py` and `run_scenario.py` report it).
- `pedestrian`, `crowd` and `cyclists` cross at the crosswalk of the ego's road at the next junction (found once, shared). In a single pass they share one trigger: they start when the ego is within `trigger_distance` of the point where its lane meets the crosswalk's walk line. `trigger_distance` is taken from `pedestrian`, else from the first of the others that sets it, else 25 m. `occluder` uses the same crosswalk (the truck stands in front of it) but starts nothing at the trigger. `run_scenario.py` runs its crossing pre-check for a variant that has one of these four tags.
- `cut_in` needs no crossing: its actors are placed along the ego's lane from the start (`distance` m ahead) and the cut-in starts when the ego is within the block's own `trigger_distance` (not the shared one).
- At night (the weather's sun below the horizon) the vehicles the scenario drives or keeps waiting with the engine on (the autopilot ego, the occluder truck and its queue, the cut-in car, the convoy's car and truck) have their position lights and low beams on; a parked vehicle (the parking cut-in's van) stays dark, and background traffic gets its lights from the Traffic Manager. A manual driver switches the ego's lights in the driving window. At night random cars (background, the occluder's queue, the convoy's car A) are drawn only from models that have lights, and there are no background motorcycles: in CARLA 0.9.16 no motorcycle or bicycle model has lights, nor 9 of the 23 car models.
- Only bicycles cross at a crosswalk, never motorcycles. The cyclists ride in a cycle-crossing lane just beside the zebra, on the junction side, 2.1 and 3.15 m from the pedestrian's line (wide enough for the widest bicycle, 0.43 m half width, beside the crowd's outer column at 1.25 m and beside a bicycle of the other lane), starting on the road just inside the kerb and stopping on the road 1.45 m before the far kerb.
- Without a crossing block the end condition drives `distance` metres from the start instead.

### Two modes: a single pass and an endless run

`ego_control mode` decides how the scenario runs.

- **Autopilot: a single pass** (a test with pass or fail). The ego follows its lane at `target_speed` (it brakes for crossers only with `brake="sight"`, see "Cautious driver"), the crossing starts at the trigger distance, and the run ends when the ego is `distance_after_crossing` past the crossing (or at the timeout). The pedestrian, crowd and cyclists are placed at load and each crosses once. A collision fails the run. After crossing, the pedestrian and the crowd walk on: an AI walker controller takes each to a navigation point 30 to 60 m beyond the far end of the crosswalk, out of the ego's sight when possible, and it is removed once hidden; a walker knocked off its line (by a bicycle) walks on from where it stands. Crossing cyclists do not stop after crossing: they roll on to the far sidewalk and keep riding along it at their own speed, away from the ego (turning back at the end of the sidewalk), and are removed once hidden; none is parked or destroyed in view. Where the far side has no sidewalk spot (Town12) a cyclist stops at the far side and is removed once hidden or after 5 s at most. On a map without a navigation mesh (Town12) the pedestrians stand on the far kerb instead.
- **Manual: an endless run.** You drive the ego with `manual_control.py`. There is **no time limit and no distance end**: the default timeout is 7 days (an explicit `<end timeout>` still applies), and the run ends only when the driving window is closed (the stop file of `<run_control>`, below; the car is braked then), when the ego vehicle no longer exists, or with Ctrl-C; either way ScenarioRunner writes its results and removes everything it spawned. The crossing stays busy for as long as you drive: **flows** keep pedestrians, cyclists and background traffic around you, arriving and leaving out of your sight (see the visibility rule). The flows start at once, so a manual run has no start trigger (ScenarioRunner's "wait until the car moves"), also when `<background>` is the only flow: the blocks declare their flow while they spawn, before the behaviors are made. Only a manual run without any flow (no navigation mesh, and no cyclists or background actors to place) keeps that trigger, with the stop file and the ego vanishing as other ways through (they end the run before the trigger fires too).

#### The visibility rule

Nothing may appear or vanish in view of the ego. Every spawn and every removal that happens while a scenario runs follows `carsim.road.hidden_from_ego(ego_transform, location, hide_distance=50, behind_distance=15, behind_angle=110, far_distance=120, side_angle=60, forward_speed=None)`. A spot is **hidden** when its 2D distance from the ego and the angle between the ego's forward vector and the direction to the spot are one of:

- (a) at least `far_distance` (120 m) away, in any direction;
- (b) at least `hide_distance` (50 m) away and more than `side_angle` (60 degrees) off the heading: off to the side. A spot 50 to 120 m straight ahead is **not** hidden, because a chase camera shows it;
- (c) at least `behind_distance` (15 m) away and more than `behind_angle` (110 degrees) off the heading: behind the car.

While the ego is reversing (`forward_speed`, its velocity along its heading, below -0.5 m/s) the driver looks backwards: (c) does not apply, and (b) only hides spots up to `behind_angle` off the heading, so it stays a rule for the sides (a spot 50 to 120 m straight behind is in view, as one straight ahead is). (a) holds whatever the ego does.

Anything closer than `behind_distance` is always in view. The optional `<spawning>` element changes the five numbers. While the ego approaches a crossing head-on there is a stretch (roughly 35 to 60 m before it, depending on the side streets) where no point within 60 m of the crossing is hidden; the pedestrian flow then waits and spawns again as soon as the car has come closer or driven past. Spawns at load, before the scenario starts, are not bound by it (except that a cyclist is no longer hidden underground). The exception during a run: the safety cap of a cyclist that has stopped at the far side of a lane that has no sidewalk spot to leave to (below).

#### Cyclists wait on the sidewalk

A cyclist does not appear out of nowhere and is not hidden underground. At load it stands visibly on the **sidewalk**, 1 to 3 m beyond the kerb on its cycle line (`road.cyclist_waiting_spots`: where the map has a Sidewalk lane within reach the nearest point on it, else 1.5 m beyond the kerb; the next waiting bicycle of the line stands 2.2 m further back), facing the way it will ride, with its **physics off** so it cannot fall or roll. If the spot cannot be spawned, a spot 0.5 and 1 m further back is tried, then the bicycle is skipped (logged). At its turn it is set down on the road at its start just inside the kerb (physics on), ridden across with a forced speed, In a manual run it is then braked to a stop, rolls to the far sidewalk and is removed only when hidden (below). In a single pass it does not stop: it rolls on to the far sidewalk and keeps riding along it until hidden.

#### Endless flows (manual ego)

- **Pedestrian flow** (`pedestrian` and `crowd` together): up to 1 + `crowd count` pedestrians are around the crossing at any time. A new one is spawned when there are fewer, at most one every 1.5 to 4 s, at a navigation point within `flow_radius` (60 m) of the crossing and at least 15 m from it that is hidden from the ego, with an AI walker controller (started a tick after the spawn). Its route: the end of the crosswalk on its own side, the far end, then a random navigation point 30 to 60 m beyond the crosswalk (one out of the ego's sight when there is one). A leg ends within 1.5 m of its target or when the pedestrian makes no progress for 10 s (stuck: it moves on). Once at its final point it is removed as soon as it is hidden; if it stays in view for 8 s it is sent on to another point out of sight, again and again (up to 20 times: it does not freeze in view holding a slot). The visibility rule leaves few hidden points while the car approaches head-on, so the flow looks at its whole pool of points for a hidden one and keeps the pool fresh with hidden points while it finds none; a pool that cannot fill at all (no navigation point in range) is sampled in bursts with a pause between them, not every tick. A few pedestrians (up to 4) are placed at load near the crossing, already walking (a third of them on the crosswalk), so the crossing is not empty for the first seconds. Town12 has no navigation mesh: there the pedestrian and crowd blocks keep the single pass, with a log line.
- **Cyclist flow** (`cyclists`): `count` bicycles keep cycling. Each waits 3 to 12 s on its sidewalk spot, rolls onto the road, crosses and stops. It then does not vanish: its physics go off and it rolls along its own line onto the far sidewalk (the waiting spot of the opposite direction, `road.cyclist_exit_spot`; the lane counts as free at once), and it is removed only once the ego cannot see it. Where the far side has no such spot within 10 m of the end of the ride, the bicycle stays at the far side and is removed once hidden or after at most 5 s (safety cap, the one removal that can happen in view). A new bicycle is spawned on a free sidewalk spot of either side, only while the spot it is spawned at (a retry spot included) is hidden. A lane has one bicycle on the road at a time, and the one nearest the kerb leaves first, so bicycles never meet; the next one also waits on the sidewalk while a bicycle of its lane is still leaving or parked on the way to the far sidewalk, and one that something has held up on that way for 8 s stops where it is and waits to be hidden (it glides on if its line clears first); the lane speeds follow the single-pass rules (`cyclist_timing`, `cyclist_speed_limit`). The flow looks at what else is around: a lane does not launch while the ego or another vehicle is within about 3 m (plus half its length) of the ride, or a pedestrian within 0.75 m (the crowd walks beside the cycle lane by design, so pedestrians on the zebra do not count); a riding bicycle stops pushing and brakes while something is on its line within 4 m ahead and goes on when it is clear; the short rolls wait. A bicycle that was spawned on a retry spot (0.5 or 1 m further back) rolls from there. The bicycles of the other lane are clear by design and do not count.
- **Background bubble** (`background`): the traffic follows the ego. The first actors are placed at load around the ego start. Then every second, vehicles and pedestrians farther than 1.3 x `radius` from the ego that it cannot see are removed, and replacements are spawned (at most 2 per second) to hold the configured counts: vehicles at map spawn points within `radius` of the ego that are hidden and free (no vehicle within 8 m), pedestrians at navigation points within `radius` that are hidden. Same blueprints as at load; background bicycles are slowed in the Traffic Manager.

Counts are never exceeded: a flow never spawns above its configured count. Actors removed during the run go through `CarlaDataProvider.remove_actor_by_id` (`BlockContext.remove_actor`, which destroys a pedestrian's AI controller first), so ScenarioRunner's pool stays consistent; an actor the server will not destroy stays on a list that is retried every second (5 tries) and counts against its flow's capacity until it is gone, so no replacement is spawned over the count; a pedestrian whose controller cannot be created or keeps failing is destroyed, not left standing. AI controllers are destroyed in `terminate()` and everything still alive at the end is removed by `CarlaDataProvider.cleanup`.

A CARLA error in one tick (a spawn, a Traffic Manager call or an actor that is gone) does not end a long session: the flow logs it (at most every 10 s), pauses for a second and goes on. The pedestrian and cyclist flows pause (nobody new is spawned, nobody new leaves the sidewalk; what is moving finishes) while the ego is farther than twice `far_distance` from the crossing, which matters on a large map such as Town12 with its dormant tiles. `ComposedScenario.terminate()` prints the line `ComposedScenario: scenario ending` first, before the ego is destroyed, for whoever watches the output, and creates the file `<stop file>.ending`, which tells the driving window's watcher that a window that exits then is not the user closing it (also when the output has not been read yet).

### Cautious driver

`<ego_control mode="autopilot" brake="sight"/>` makes the ego a cautious driver (`blocks\driver.py`) instead of a plain waypoint follower. It still follows its lane at `target_speed`, but it only reacts to what it can see, and it does not know about anything hidden.

- **Start:** the ego starts at its test speed (as in NCAP), not from rest.
- **What it sees:** walkers and vehicles (bicycles included) within 50 m ahead that could reach its route. An actor is visible when a ray from the driver's eye (1.3 m up, just ahead of the bumper; built from the live ego pose) to the actor's centre is clear, else one to its top or bottom: `world.cast_ray` finds nothing nearer than the actor. CARLA's invisible `NONE` hits and hits on the ego itself are ignored. At most 12 rays a tick, nearest actors first. An actor that is hidden again is tracked for 2 s from what was last seen (its last position carried on at its last velocity), never from where it really is.
- **Crossers:** at a crossing, a known walker or bicycle on a driving lane within 4 m of the crosswalk makes it stop 2 m before the crosswalk and wait until the crosswalk is clear, then drive on. A walker waiting on the kerb or already on the far kerb is not a crosser.
- **In its path:** a known actor is a hazard only when its predicted track (constant velocity, up to 6 s) is in the ego's path while the ego gets there; one that has already crossed, or will only arrive after the ego has passed, is not. For a hazard it stops before the conflict, and it brakes at full force when the time to collision is under 1.6 s or it can no longer stop comfortably. A full brake bites 0.3 s after the decision (actuation delay).
- **Lead following:** a visible slower vehicle or cyclist ahead in its lane is followed at its speed with gap feedback (gap 4 m + 1 s), without stop-and-go.
- **Log:** `ego_control` lines in the scenario log: "first visible" (the target, or the first hazard: time, distance bumper to surface, time to collision), "hazard", "stopping for crosser", "emergency brake", "clear, driving on". The same events go into the run metrics (below).

- **Speed control:** `LocalPlanner` only steers; the driver's own controller sets throttle and brake: a feed-forward throttle plus a PI on the speed error (m/s) with anti-windup, braking only when the ego is more than 0.3 m/s over its target. On the `vehicle.lincoln.mkz_2017` it holds 2nd gear between 3.5 and 10 m/s, because the automatic gearbox hunts between 1st and 2nd there; cruise then holds within about 0.1 m/s. The gains are tuned on that car only.
- **Late crosser:** a crosser that appears when the ego can no longer stop before the crosswalk is passed (only the in-path check can still brake); the ego never stops and waits on the crosswalk.

`brake="none"` is the same driver with hazards ignored: same speed control, it never brakes for anything (the counterfactual run: where would the impact be without braking). Without a `brake` attribute the ego is the plain waypoint follower, as before (it starts at its target speed). `brake` is ignored in a manual run.

### Run metrics

Every run writes `outputs\<run>\metrics.json` and copies it into `run.json` under `"metrics"`; the console prints a one-line summary (`Metrics: impact YES at 18.0 km/h | first visible TTC 1.5 s | brake onset TTC 0.9 s | min distance 0.0 m`). The scenario's **target** is the actor the test is about: the single pedestrian, or the `cut_in` car or cyclist. Fields (null when not measured):

| Field | Meaning |
|---|---|
| `test_speed` | m/s, the ego's `target_speed` |
| `first_visible` | the first sighting of the target (not necessarily a hazard yet): time (sim s), actor, kind, distance (m, bumper to surface), time to collision |
| `first_hazard` | when the cautious driver first judged something a hazard: time, actor, kind, distance, time to collision; null without a driver |
| `brake_onset` | the first brake (>= 0.2) the driver commanded in reaction to a hazard (emergency, stop target or braking for a lead): time, speed, time to collision, distance; without a driver, the first such brake of the ego while the target is a hazard (time to collision under 4 s); a start-up cruise brake is not one |
| `emergency`, `emergency_count` | the first emergency brake (time, speed, distance) and how many there were |
| `impact` | `happened`, the ego speed (m/s, one step before the contact), time, the other actor's type and whether it was the target (the metrics' own collision sensor); null (unknown) when the collision sensor could not be spawned |
| `speed_reduction` | `test_speed` minus the impact speed (the whole test speed when there was no impact; null when the impact is unknown) |
| `min_distance`, `min_ttc` | the smallest gap (m, ego box to target box) and time to collision (gap along the ego's heading to the part of the target inside the ego's width band, over the closing speed; null while nothing of it is inside) |
| `duration` | sim s |

### Chase view

An autopilot run opens a window that follows the ego (`apps\chase_view.py`, own console, 1280x720; `--res WxH` when you start it by hand). It is a camera attached to the ego, so the CARLA window stays a free camera. With `run_scenario.py --view-spectator` (`chase_view.py --spectator`) it attaches no sensor: the server's spectator follows the ego in the same views, so the picture is in the CARLA window and this window shows the HUD and the buttons on black (for large maps, where CARLA 0.9.15 crashed when it put a sensor to sleep). The scenario destroys the window's camera before the ego (the `ego_control` block's cleanup). The HUD shows the speed (km/h), throttle, brake and steer, the nearest vehicle, walker or bicycle ahead in the ego's path within 50 m (distance and time to collision), the simulation time and the map. Keys, with the window focused: **C** cycles the view (chase, bonnet, top-down), **H** toggles the HUD, It only watches the world: it applies no control and does not tick it. It waits for the ego to appear (`--wait` seconds). `run_scenario.py --no-view` does not open it; a manual run never does.

It is also the control panel of the run, with two buttons drawn in the window (click them) and two keys: **Restart (R)** and **Quit (Esc, or the X)**. During a run, R ends the run cleanly (the scenario sees a `stop` file in the run folder, as for the driving window, so the results are saved and the actors removed) and starts the same variant and settings again in the same `run_scenario.py` process, without reloading the map; every restart is a normal run with its own `outputs\<time>_<variant>\` folder, `run.json` (`end_reason` is `restart` or `quit` when you ended it) and `metrics.json`. Quit ends the run cleanly and `run_scenario.py` exits. When a run ends by itself the window stays open with the last frame and a banner, in plain words from the run's `metrics.json`: the headline ("PASSED: no collision", "COLLISION: hit the pedestrian at 18 km/h", "Run ended" when unknown), the test speed, when the car first saw the target (how far ahead, and how many seconds before it would have reached it), how soon after that it started braking, whether that was an emergency brake or a normal stop, and the closest distance (or how much it slowed before an impact), with the same two buttons; R restarts, Quit exits. The window re-attaches to the new ego by itself, so it stays open across restarts. `--no-view` runs once and exits as before; Ctrl-C in the console still ends everything. A scenario with its own `.py` (no blocks) cannot be ended from outside: R or Quit then act when the run has ended. The window and `run_scenario.py` talk through small files in a temporary folder that the window gets as `--control DIR` (`restart.request`, `quit.request` written by the window, `result.json` written by `run_scenario.py` after each run); started by hand without `--control` the window only follows the ego and closes when it is gone (Esc closes it).

### Block reference

| Block | Attribute | Default | Meaning |
|---|---|---|---|
| `ego_control` | `mode` | `autopilot` | `autopilot`: the ego starts at `target_speed` and follows its lane straight through the junction (no braking unless `brake="sight"`), a single pass with pass or fail; `manual`: you drive it, with `manual_control.py` (`run_scenario.py --drive manual` overrides this per run and opens it for you), and the run is endless: **no timeout and no distance end**, it ends via the stop file (driving window closed) or Ctrl-C |
| | `target_speed` | `30` | km/h |
| | `brake` | `none` | `none`: no braking, the ego just follows its lane; `sight`: the cautious driver (see "Cautious driver"), and the default autopilot timeout gets 60 s more; ignored in manual mode |
| `spawning` | `hide_distance` | `50` | m from the ego from which a spot off to the side (more than `side_angle` off its heading) is out of sight |
| | `far_distance` | `120` | m from the ego from which a spot is out of sight in any direction, straight ahead included (keep it at or above `hide_distance`) |
| | `side_angle` | `60` | degrees between the ego's heading and the direction to the spot, above which, from `hide_distance` on, the spot is off to the side |
| | `behind_distance` | `15` | m: closer than this a spot is always in sight; from this far a spot behind the car is out of sight |
| | `behind_angle` | `110` | degrees between the ego's heading and the direction to the spot, above which the spot is behind the car (not while the ego reverses) |
| `run_control` | `stop_file` | none | **Internal**: written by `run_scenario.py` into its copy of the XML for a manual run with the driving window and for an autopilot run with the chase view, never by hand. The scenario ends cleanly as soon as this file exists (`run_scenario.py` creates it when you close the driving window, or press Restart or Quit in the chase view) |
| `occluder` | `model` | `vehicle.carlamotors.european_hgv` | A vehicle that stands (handbrake on) in the kerb lane just before the crosswalk and hides it from the ego, which drives in the lane left of it (pick the start with `find_crossing_spots.py --lane inner`); it must spawn or the scenario stops |
| | `gap` | `1.5` | m from the front of the truck to the near edge of the crosswalk |
| | `queue` | `1` | Random cars standing behind the truck, 1.5 m clear apart |
| | `prop` | none | A static prop on the kerb side just before the crosswalk, long side along the road (the Child variant uses `static.prop.busstop`); with the truck it hides the waiting pedestrian from the ego lane. A prop that cannot be spawned is skipped |
| `pedestrian` | `model` | `walker.pedestrian.0001` | The main adversary, crossing right to left; it must spawn or the scenario stops (single pass only) |
| | `age` | none | `adult` or `child`: a random walker of that age instead of `model` (use one of the two, not both) |
| | `walk_in` | `0` | m: the pedestrian starts this far back along the road (towards the ego, off the driving lanes), walks along the road to its usual start, then turns and crosses; a truck at the kerb hides it there once the ego is within about 15 to 20 m. Single pass only |
| | `overlap` | none | % of the ego's width, counted from the side the pedestrian comes from, where the ego's front would hit it without braking (NCAP: 25, 50, 75). Set, the walk starts by time instead of `trigger_distance` (use one of the two): when the ego's time to arrive at that point equals the pedestrian's walk time to it, assuming the ego keeps its speed on a straight approach (single pass) |
| | `speed` | `1.4` | m/s (single pass) |
| | `trigger_distance` | see rules | m between ego and crossing point when the crossing starts (single pass) |
| | `flow_radius` | `60` | m around the crossing within which the pedestrian flow spawns pedestrians (manual run) |
| `crowd` | `count` | `0` | Extra pedestrians on the same crosswalk, about half from each side, in queued columns; in a manual run `count` + 1 pedestrians in the flow |
| | `min_speed`, `max_speed` | `1.1`, `1.8` | m/s, one random speed per column (per pedestrian in the flow) |
| | `max_delay` | `4` | s, random start delay of the front of each column after the trigger (single pass) |
| | `trigger_distance` | see rules | m (single pass) |
| | `flow_radius` | `60` | m, as for `pedestrian` (the pedestrian's value wins) |
| `cyclists` | `count` | `2` | Bicycles (`base_type` bicycle) crossing from both sides, waiting on the sidewalk until their turn; single pass: queued 2.5 m apart, after crossing they ride on along the far sidewalk and are removed once hidden; manual run: `count` bicycles keep cycling and leave to the far sidewalk, removed when hidden |
| | `min_speed`, `max_speed` | `3.0`, `5.0` | m/s, one random speed per direction |
| | `max_delay` | `3` | s, random start delay of the front bicycle of each direction after the trigger (single pass) |
| | `trigger_distance` | see rules | m (single pass) |
| `cut_in` | `kind` | required | `parking`: the ego is in the kerb lane with a shoulder on its right; a parked van hides a parked car that pulls out into the ego lane. `cyclist`: the ego is in the inner lane; in the kerb lane car A (3 m ahead), the cyclist and a truck (1.5 m behind it) ride at the cyclist's speed from the first tick, and the cyclist moves into the ego lane |
| | `distance` | `60` parking, `25` cyclist | m along the ego lane from the ego start to the cut-in actor |
| | `trigger_distance` | `15` parking, `12` cyclist | m between ego and cut-in actor at which the cut-in starts (the block's own; not the shared trigger) |
| | `speed` | `4` | m/s of the cut-in actor (and of car A and the blocker) |
| | `blocker` | `vehicle.carlamotors.firetruck` (cyclist), `vehicle.mercedes.sprinter` (parking) | Blueprint of the vehicle that hides the cut-in actor; an id that is not a vehicle blueprint is an error |
| `background` | `vehicles` | `0` | 4-wheel vehicles on Traffic Manager autopilot, with its lane changes off (no cut-in into the ego's lane); the Traffic Manager also switches their lights (headlights at night, brake lights, indicators) |
| | `motorcycles` | `0` | Motorcycles (`base_type` motorcycle) on autopilot |
| | `bicycles` | `0` | Bicycles riding in traffic on autopilot, at half the speed limit (Traffic Manager speed difference 50 %) |
| | `walkers` | `0` | Pedestrians wandering via `controller.ai.walker` (none on maps without a navigation mesh, such as Town12) |
| | `radius` | `120` | m around the crossing (the ego start without a crossing block); in a manual run around the ego, which the traffic follows |
| `end` | `distance_after_crossing` | `30` | m driven past the crossing point before the scenario ends (autopilot only: a manual run has no distance end) |
| | `distance` | `100` | m driven from the start when there is no crossing block (autopilot only) |
| | `timeout` | autopilot: `max(60, drive distance / ego speed * 1.5 + 20 + W)`, W = 60 s with `brake="sight"` (the cautious driver may wait at the crosswalk), else 0; manual mode: none (7 days) | s, counted from loading. The drive distance is the distance to the crossing plus `distance_after_crossing` (or `distance` with no crossing block), so a long `distance_after_crossing` or a slow `target_speed` lengthens the default by itself. Manual mode has no timeout and no distance end: it ends via the stop file (driving window closed) or Ctrl-C; an explicit value here is still honored |

Values are checked by `list_scenarios.py` and `run_scenario.py`: counts (`count`, `vehicles`, `motorcycles`, `bicycles`, `walkers`) must be whole numbers of 0 or more, speeds, distances, radii and distances (`radius`, `flow_radius`, `hide_distance`, `far_distance`) and `timeout` numbers above 0, `max_delay` and `behind_distance` numbers of 0 or more, `behind_angle` and `side_angle` numbers from 0 to 180, `min_speed` must not exceed `max_speed`, and `far_distance` must not be below `hide_distance`. A tag used twice in one variant is reported (ScenarioRunner keeps only the last one), and so is an `<other_actor>` element, which `ComposedScenario` ignores.

Background vehicles spawn at load at map spawn points at least 25 m from the ego start and never in a lane of the ego's route; the vehicles of a manual run's bubble spawn wherever the visibility rule and a free spawn point allow.

### Writing a new block

A block is a subclass of `carsim.blocks.base.Block` with a `tag`. It reads its parameters from `self.params` and implements any of these hooks; `self.ctx` is the shared `BlockContext` (world, map, ego, the seeded `random.Random`, the ego start waypoint and route plan, the lazily computed `Crossing`, the trigger location and distance, `ctx.endless` for a manual run, the hide rule with `ctx.hidden(location)` for the ego's pose now and `ctx.is_hidden(ego_transform, location)` for a pose you already hold, and registration and mid-run removal of spawned actors and AI controllers):

| Hook | Called | Purpose |
|---|---|---|
| `spawn()` | in `_initialize_actors` | create actors and register them with `ctx.add_actor()` |
| `after_tick()` | after the scenario's one world tick | extra setup that needs the actors on the server (starting AI walker controllers) |
| `from_start()` | when building the tree | behaviors that run from the first tick (the ego autopilot, the endless flows) |
| `on_trigger()` | when building the tree | behaviors that run once the shared trigger fires |
| `end_conditions()` | when building the tree | conditions that end the scenario when one succeeds |
| `cleanup_behaviors()` | when building the tree | behaviors that run after the main part (removing the main pedestrian) |
| `cleanup()` | `terminate()` and `__del__` | release what is not removed with the scenario's actors (controllers); must be safe to call twice |

A behavior that spawns during the run must spawn only where `ctx.hidden(location)` and remove only what `ctx.hidden(...)` allows, remove through `ctx.remove_actor()` and stay at its count (see `src\carsim\blocks\flow.py`). Register the block in `src\carsim\blocks\spec.py` (`BLOCK_ORDER`, `PARAM_TYPES`, `CROSSING_BLOCKS` if it crosses at the crosswalk) and in `src\carsim\blocks\__init__.py`, add a row to the table above and extend `tests\check_blocks_offline.py` (and `tests\check_flows_offline.py` for a flow). Geometry that does not need a ScenarioRunner scenario belongs in `src\carsim\road.py`, where `tests\check_crossings_offline.py` can test it on every map.

## Adding a new scenario

Conventions are enforced by `src/carsim/scenario_files.py`, and the scenario folder name is what you pass to `run_scenario.py`. Most scenarios need only an XML file: compose blocks. Python is only needed for a new block type or a scenario that does not fit the blocks.

- [ ] Create `scenarios\<name>\`.
- [ ] Put exactly one `.xml` file in it: a ScenarioRunner config with one or more `<scenario name="..." type="ComposedScenario" town="...">` variants. Keep a `<ego_vehicle x y z yaw .../>` element if you want `find_crossing_spots.py --apply` to work, add a `<weather .../>` element if you want other weather, and add the blocks you want as child elements (see "Scenario blocks"). XML comments must not contain a double hyphen. Copy `scenarios\pedestrian_crossing\pedestrian_crossing.xml` to start.
- [ ] The variant `name` must be unique, also against the `<scenario name>` entries in `external\scenario_runner\srunner\examples\*.xml` (`validate` checks it).
- [ ] No `.py` is needed: a folder without one uses the shared `scenarios\_composed\composed_scenario.py`. ScenarioRunner imports that file by its basename `composed_scenario`, which is why the module name must not shadow anything importable; the file is checked like any other.
- [ ] Only for a scenario that does not fit the blocks: put exactly one `.py` file in the folder (files starting with `_` are ignored) with your own scenario class and use that class as the XML `type`. ScenarioRunner imports every `srunner\scenarios\*.py` by bare module name and runs the first class whose name equals the XML `type`, including classes those modules merely import, so:
  - The XML `type` must be a class defined in the scenario's `.py`.
  - The `type` must not be a class name that ScenarioRunner's scenario modules define or import (for example `PedestrianCrossing`, `CollisionTest`, `KeepVelocity`, `ActorFlow`; about 176 names). Prefix it, like `JunctionPedestrianCrossing`.
  - The `.py` file name (without `.py`) must not equal a file in `external\scenario_runner\srunner\scenarios\` and must not be an importable module (standard library, `carsim`, `agents`, `srunner`, `manual_control`, ...).
- [ ] `run_scenario.py` checks all of this (including unknown block tags and attributes) and exits with the list of problems; `list_scenarios.py` shows them per scenario.
- [ ] Put code shared with other scenarios or apps in `src\carsim\`, not in the scenario folder; a new kind of actor behavior is a new block.
- [ ] Add a short `README.md` in the folder describing the blocks used and the expected outcome.
- [ ] Check it shows up without problems, then run it:

```powershell
python -m uv run python apps\list_scenarios.py --ours
```

```powershell
python -m uv run python apps\run_scenario.py <name>
```

The offline checks need no server:

```powershell
python -m uv run python tests\check_crossings_offline.py
python -m uv run python tests\check_blocks_offline.py
python -m uv run python tests\check_flows_offline.py
python -m uv run python tests\check_driver_offline.py
python -m uv run python tests\check_cut_in_offline.py
python -m uv run python tests\check_chase_view_offline.py
python -m uv run python tests\check_trigger_offline.py
python -m uv run python tests\check_metrics_offline.py
```

## Troubleshooting

**Wrong map, or "map not found".** `list_maps.py` must show the scenario's map (`Town10HD_Opt` for the shipped scenario). If it is missing, check that the CARLA 0.9.16 release is unzipped into `CARLA_0.9.16\` (setup step 2) and restart the server. The apps load the scenario's map themselves if a different one is active.

**Server left in synchronous mode.** Runs are in real time (asynchronous) unless you pass `--sync`. A `--sync` ScenarioRunner run that crashes can leave the server in synchronous mode, after which the simulation appears frozen. `run_scenario.py` switches the server back to asynchronous before each run, so simply run it again. Restarting CARLA also clears it.

**Large generated maps (WolfsburgHighway).** Every app that loads a map sets the streaming distances afterwards (`tile_stream_distance` 600 m, `actor_active_distance` 450 m; `run_scenario.py --stream-distance/--active-distance`), the spectator travels in steps of at most 100 m, the ground wait accepts that the trace may see nothing there (it then waits 10 s more), and every actor spawns on the generated road mesh, whose height is a known function of x and y (`carsim.road.deformation`). The Traffic Manager runs on port 8100 (`--tm-port`), and its first start on such a map can take minutes. The cautious driver logs which line of sight it uses (`World.cast_ray`, or rays against the actors' bounding boxes when its start-up self-test sees nothing). See `PORTING.md`.

**Large-map tiles load slowly, or the world looks empty near the ego.** Town12 streams the tiles around the hero vehicle (`rolename="hero"`), or around the spectator camera while there is no hero (`spectator_as_ego` defaults to true). The apps move the spectator to the ego start, and `run_scenario.py` then waits up to 60 s for ground to exist there before it starts ScenarioRunner, so the ego and pedestrian do not fall through unloaded ground. If it reports that no ground was found, check that the start is on the map and the server is not paused or minimized, then run it again. Start CARLA with the default Low quality.

**The CARLA window stays black or shows "Not Responding" after starting.** The first start compiles shaders and loads the default map; on this laptop that takes a few minutes, during which clients get `time-out ... waiting for the simulator`. Don't close it; wait until a town appears in the window, then run `python -m uv run python apps\list_maps.py`. `start_carla.ps1` passes `-prefernvidia` so laptops with two GPUs render on the NVIDIA one; if it still uses the Intel GPU, set `CarlaUE4-Win64-Shipping.exe` (in `CARLA_0.9.16\CarlaUE4\Binaries\Win64`) to "High performance" in Windows Settings > System > Display > Graphics.

**The CARLA window's camera or mouse looks wrong, or the world is frozen after you stopped a scenario.** Stopping ScenarioRunner forcibly (closing the terminal, or pressing Ctrl-C three times to abandon it) can leave the server in synchronous mode (after a `--sync` run), where the world only advances when a client ticks it, and the ego vehicle in the world. `run_scenario.py` resets this when it ends normally or after Ctrl-C (the first Ctrl-C waits for ScenarioRunner to clean up and print its results; a second one only prints a reminder; a third abandons ScenarioRunner, and the world is reset anyway). After a hard kill, run:

```powershell
python -m uv run python apps\reset_world.py
```

**First Town12 load takes minutes.** This is normal. The load timeout is 600 s for map loads, but a `timeout of ... seconds` error while loading means the server was still busy; wait until the CARLA window is responsive and run the command again. Loading Town12 beforehand with `python -m uv run python apps\list_maps.py --load Town12` keeps the later commands fast.

**`import carla`, `import srunner` or `import carsim` fails.** Activate the venv (`.venv\Scripts\Activate.ps1`) or use `python -m uv run python apps\...`, and if needed re-run `python scripts\setup_env.py` (not from inside the venv). If `.venv` was built with Python 3.12 or newer, rebuild it with `python scripts\setup_env.py --recreate`. After editing `pyproject.toml` by hand, `uv sync` brings `.venv` up to date.

**`run_scenario.py` stops with "has problems" or "is broken".** The scenario folder breaks a rule from "Adding a new scenario" (a name collision with ScenarioRunner, an unknown block tag or attribute, a `type` that is not a class in the scenario's `.py`, or not exactly one `.xml`). The message says what to rename. `list_scenarios.py --ours` shows the same problems for every scenario and keeps listing the rest.

**Manual driving: the run is still going after I closed the window / the window closed by itself.** Closing the driving window (Esc, its X) ends the run: the scenario finishes, saves its results and the world is reset, which takes a few seconds, and the console prints "Driving window closed: ending the run (results are being saved)". There is no time limit in manual mode, so the run never ends on its own: close the window or press Ctrl-C in the scenario console. With `--no-driver-window` only Ctrl-C ends it. If the window closes by itself right at the start, the scenario had already ended or failed to load (see the scenario console). Keep the run in real time (do not add `--sync`).

**Manual driving: the pygame window says "Not Responding" or stays black.** The driving window (`manual_control.py`) opens when ScenarioRunner reports that the scenario is running. It starts with a black window and prints "Waiting for the ego vehicle..." in its own console until it finds the ego (rolename `hero`), and during that the window can show "Not Responding"; this is normal for a few seconds. If it never finds the ego, the scenario has already ended or failed to load, or the `--host`/`--port` differ; `run_scenario.py` closes the window when the run ends, so a window left open means the run is still going. The window can also be started by hand with `--no-driver-window`.

**The driving window does not take keyboard input.** Click into the pygame window first; keys only reach the window that has the focus.

**CARLA or ScenarioRunner in a different location.** Set the `CARLA_ROOT` environment variable (and `SCENARIO_RUNNER_ROOT` for ScenarioRunner) before running the scripts and apps.
