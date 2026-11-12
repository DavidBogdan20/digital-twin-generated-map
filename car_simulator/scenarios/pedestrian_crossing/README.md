# pedestrian_crossing

The ego vehicle drives towards a junction on the small Town10HD_Opt map (or on WolfsburgHighway, see the last section). It is a
cautious driver (`brake="sight"`) that starts 120 m before the crossing: it stops
before the crosswalk while a pedestrian or cyclist it can see is on it, and drives
on when the crosswalk is clear. A pedestrian waits just
beyond the right end of the painted crosswalk that the ego lane crosses at that
junction. When the ego vehicle gets within `trigger_distance` of the crossing
point, the pedestrian walks right to left along the crosswalk. A crowd of further
pedestrians and two cyclists cross with them from both sides, and cars,
motorcycles, bicycles and pedestrians move around the junction (see "Crowd,
cyclists and background traffic").

There are two ways to run it, chosen by `ego_control mode`. With the **autopilot** it is a
**single pass**, a test: one drive through the crossing, an end condition, pass or fail.
With **manual** driving it is an **endless run**: no time limit, no distance end, and the
crossing stays busy (pedestrians and cyclists keep arriving, crossing and leaving out of
your sight, background traffic follows you) until you close the driving window or press
Ctrl-C (see "Endless manual run" and "Manual driving").

The scenario is built from blocks: the XML names them, `ComposedScenario`
(`scenarios/_composed/composed_scenario.py`, class in the root README's "Scenario
blocks") runs them. There is no Python file in this folder.

## Where the pedestrian crosses

CARLA paints the crosswalk of an approach at the junction, usually a few metres
inside it and sometimes just before it. The scenario takes the crosswalk the ego
lane runs through, from 20 m before to 10 m past the junction entry; the nearest one wins.
Crosswalks parallel to the ego road (they cross the side roads) and crosswalks
more than 3 m above or below it (overpasses) are ignored. Then:

- The **crossing point** is where the ego lane meets the walk line, the line
  along the crosswalk's long axis through its center. It is what
  `trigger_distance` and the end condition refer to, and it can be before or
  inside the junction.
- The **walk** goes from 1 m beyond the right end of the crosswalk (seen from
  the ego) to 1 m beyond the left end, so its length is the crosswalk length
  plus 2 m. An end that would stand on a driving lane is moved outwards.
- Without a painted crosswalk the pedestrian crosses at the junction entry, from
  the right sidewalk to the far sidewalk. A missing sidewalk is replaced by a
  point 1 m beyond the outermost lane on that side, so multi-lane roads work.

## Crowd, cyclists and background traffic

All three are optional; a missing `<crowd>`, `<cyclists>` or `<background>` element switches that feature off.

- **Crowd** (`<crowd>`): `count` extra pedestrians cross on the same crosswalk as the main pedestrian, half
  starting at the right-hand end (walking right to left, like the main one) and half at the left-hand end.
  They queue a little behind the kerb in rows 0.7 m apart, in three columns 0.45, 0.85 and 1.25 m from the main
  pedestrian's line (a painted crosswalk is 2.5 m wide on every map, so the outer column stays on the paint;
  the inner one keeps clear of the main pedestrian, whom a shove would derail). Neighbouring columns are shifted
  by half a row. Each stream keeps to its own right, so the two pass side by side. Every walker walks a straight
  line parallel to the main pedestrian's and the same distance as the others of its side, so the queue is mirrored
  at the far end: the walker at the front ends furthest out and nobody has to pass another. A column uses one
  speed, drawn between `min_speed` and `max_speed`; its front walker starts after the same trigger as the main
  pedestrian plus a random delay of 0 to `max_delay` s, and each walker behind starts at least 0.7 m / `min_speed`
  (plus up to 0.5 s) after the one ahead, so the delays of the back rows exceed `max_delay`. They use random walker
  models and stand still at the far side. A start on a driving lane is moved or skipped. A pedestrian that cannot
  be spawned at its start (a prop is often in the way) is retried 0.25 m and then 0.5 m further back on its own
  line, with the same target and column and a longer walk (the log says `spawned 10/10 crowd pedestrians (1 moved
  back)`), and skipped if that fails too (`spawned 9/10 crowd pedestrians`); the main pedestrian must still spawn
  or the scenario stops.
- **Cyclists** (`<cyclists>`): `count` bicycles cross at the same moment, half from each side. Only bicycles
  (blueprints with `base_type` bicycle: `vehicle.bh.crossbike`, `vehicle.gazelle.omafiets`,
  `vehicle.diamondback.century`) cross at a crosswalk, never motorcycles. They do not use the zebra, which the crowd
  has: they ride in a cycle-crossing lane just beside it, on the junction side (the side the ego drives towards),
  2.1 m (one direction) and 3.15 m (the other) from the main pedestrian's line (the widest bicycle is 0.43 m
  half wide, a walker up to 0.25 m: the first lane is the crowd's outer column at 1.25 m plus a walker, a bicycle
  and 0.15 m, the second the first plus two bicycle half widths and 0.15 m). Each direction keeps to its own
  right, so the outer lane belongs to the stream whose right-hand side faces away from the zebra. A bicycle rides
  ON THE ROAD, from 1.2 m inside the first driving lane point (just inside the kerb, so it does not drop off it) to
  1.45 m inside the last one, on the road just before the far kerb. Bicycles of one direction queue 2.5 m
  apart with the front one starting furthest in, all riding the same distance at one speed (between `min_speed`
  and `max_speed`); the front one leaves after a random 0 to `max_delay` s, each one behind it `gap / speed` s
  (plus up to 0.5 s) after the one ahead, so nobody catches up. A narrow road fits fewer bicycles per lane.
  **A bicycle waits visibly on the sidewalk** and does not pop up: at load it stands 1 to 3 m beyond the kerb on its
  cycle line (`road.cyclist_waiting_spots`: on a Sidewalk lane where the map has one within reach, else 1.5 m beyond
  the kerb; bicycles of the queue stand 2.2 m apart, the front one nearest the kerb), facing its crossing direction,
  with its **physics off** so it cannot fall or roll. A spot that cannot be spawned is retried 0.5 m and 1 m further
  back on the line, then the bicycle is skipped (a log line). After its delay `ActorTransformSetter` sets it
  onto the road at the start with physics on (the short move off the kerb). It is then pushed at its speed with a
  forced velocity (`KeepVelocity`, `force_speed`) for the ride length minus
  the distance it still travels when the push ends one simulation step late (`speed * step`, with the world's fixed
  step, 1/20 s by default) and an assumed braking distance (`speed^2 / (2 * 4 m/s^2)`, a low guess), braked to a
  stop (`StopVehicle`) and **removed** (`ActorDestroy`), so nothing is left parked on the road: in this single
  pass a bicycle is destroyed in view right after it has crossed (the manual run does not do that, see "Endless
  manual run"). A ride too short for this caps the speed instead of being stretched past the kerb.
- **Background** (`<background>`): `vehicles` 4-wheel vehicles, `motorcycles` (base_type motorcycle) and `bicycles`
  (base_type bicycle, riding in traffic, at half the speed limit: Traffic Manager speed difference 50 %) on Traffic Manager autopilot at map spawn points within
  `radius` m of the crossing point (never within 25 m of the ego start, and never in a lane the ego's route or
  start uses: such a vehicle would drive slower than the ego or stop at the crowd, and the autopilot ego does not
  brake; the log says how many spawn points were dropped), and `walkers` pedestrians spawned at random
  navigation points within `radius` that wander to a random destination (`controller.ai.walker`, removed when the
  scenario terminates). The search for navigation points inside the radius is capped, so on a large map fewer may
  be found; the log reports how many were spawned. Two-wheelers under the Traffic Manager are less stable than cars;
  check them in the live run. In a manual run the traffic follows the ego (see "Endless manual run").
- **Town12 has no navigation mesh** (the `Nav` folder of the CARLA package holds none for it), so
  `get_random_location_from_navigation()` returns nothing there. After 5 empty answers the scenario logs
  `no navigation mesh for this map: wandering pedestrians skipped` and goes on without them; the crowd, the
  cyclists and the background vehicles still work. A manual run cannot make its endless pedestrian flow without a
  navigation mesh: the `pedestrian` and `crowd` blocks log `no navigation mesh for this map: no endless pedestrian
  flow, a single pass instead` and keep the single pass; the cyclist flow and the background vehicles still flow.
- All crowd, cyclist, vehicle and pedestrian actors are spawned without a world tick each, followed by one tick for all of them.
- Random choices are drawn from a generator seeded by ScenarioRunner's `CarlaDataProvider` (fixed seed 2000), and
  the pedestrian seed of the world is set from it (`World.set_pedestrians_seed`), so repeated runs place the same
  crowd, cyclists, vehicles and wandering pedestrians. The run itself is repeatable only in synchronous mode (`--sync`; the default is real time), and only
  approximately: physics and the AI of the Traffic Manager and the walker controllers do not replay exactly.

## Endless manual run

With `ego_control mode="manual"` (also `run_scenario.py --drive manual`) the scenario does not drive the ego, has
**no time limit and no distance end**, and the crossing stays busy for as long as you drive. The blocks use flows
(`src/carsim/blocks/flow.py`) instead of the single pass, from the first tick. There is no start trigger to wait for (not
ScenarioRunner's "wait until the car moves", and no trigger distance): the blocks declare their flow while they spawn, so a
variant whose only flow is `<background>` has none either. Only a manual run without any flow (no navigation mesh, and no
cyclists or background actors) keeps the start trigger, with the stop file and the ego vanishing as other ways through:

- **Pedestrian flow** (`pedestrian` and `crowd` together). Up to 1 + `crowd count` pedestrians are around the
  crossing. When there are fewer, a new one is spawned, at most one every 1.5 to 4 s, at a navigation point
  (`get_random_location_from_navigation`) within 60 m (`flow_radius`) of the crossing and at least 15 m from it that is
  hidden from you, with an AI walker controller. The controller is started, given its speed and sent on its way one
  tick after the spawn. A pedestrian walks to the end of the crosswalk on its own side, across to the far end, then to
  a random navigation point 30 to 60 m beyond the crossing (preferably one out of your sight). A leg ends within 1.5 m
  of its target, or when the pedestrian makes no progress for 10 s (stuck: it moves on). At its final point it is removed
  as soon as it is hidden; if it is still in view after 8 s it is sent on to another point out of sight, again and
  again (up to 20 times), so it never freezes in view holding a slot. The visibility rule leaves few hidden points while
  you approach head-on (see below), so the flow looks at its whole pool of points for a hidden one and keeps the pool
  fresh with hidden points while it finds none; a pool that cannot fill at all is sampled in bursts with a pause, not
  every tick. Up to 4 pedestrians are placed at load near the crossing, already walking (a third of them on the
  crosswalk), so the crossing is not empty for the first seconds. The flow pauses (no new pedestrians) while you are
  more than twice `far_distance` (240 m) from the crossing; pedestrians already walking finish and leave.
- **Cyclist flow** (`cyclists`). `count` bicycles keep cycling. A bicycle waits on the sidewalk (physics off) a random
  3 to 12 s, rolls onto the road, crosses and stops. It does not vanish then: its physics go off and it rolls along its
  own line onto the far sidewalk (the waiting spot of the opposite direction, `road.cyclist_exit_spot`), its lane is free
  for the next bicycle at once, and it is removed only when it is hidden. Where the far side has no such spot within 10 m
  of the end of the ride, it stays at the far side and is removed once hidden, or after 5 s at the latest (a safety
  cap: only then can it vanish in view). A new bicycle is spawned on a free sidewalk spot of either side, only while the
  spot it is spawned at (a retry spot included) is hidden (the first ones are placed at load). A lane has one bicycle on
  the road at a time and the one nearest the kerb leaves first, which keeps them apart; the lane speeds are the
  single-pass ones. A bicycle looks at what else is around: it does not leave while you or another vehicle are within
  about 3 m (plus half the vehicle's length) of its ride (up to the far sidewalk spot, where there is one: a bicycle of
  its lane that is still leaving or parked there holds it back) or a pedestrian is within 0.75 m (pedestrians on the
  zebra beside the cycle lane do not count). A bicycle on its way to the far sidewalk that something has held up for 8 s
  stops where it is and waits to be hidden (it glides on if its line clears first). A riding bicycle stops
  pushing and brakes while something is on its line within 4 m ahead and goes on when it is clear, and the short rolls
  wait. A bicycle spawned on a retry spot (0.5 or 1 m further back) rolls from there. The flow pauses (nobody
  new is spawned or leaves the sidewalk) while you are more than 240 m from the crossing.
- **Background bubble** (`background`). The traffic follows you. The first actors are placed at load around the ego start.
  Every second, background vehicles and pedestrians that are farther than 1.3 x `radius` from you and out of your
  sight are removed, and replacements are spawned (at most 2 per second) to hold the configured counts: vehicles at
  map spawn points within `radius` of you that are hidden and free (no vehicle within 8 m), pedestrians at navigation
  points within `radius` that are hidden. Same blueprints as at load, background bicycles are slowed in the Traffic Manager.
- **Visibility rule.** Nothing is spawned or removed in view of the ego. A spot is hidden (`road.hidden_from_ego`) when
  (a) it is at least `far_distance` (120 m) away, in any direction, or (b) it is at least `hide_distance` (50 m) away and
  more than `side_angle` (60 degrees) off the ego's heading (a spot 50 to 120 m straight ahead is in plain view of the
  chase camera, so it is not hidden), or (c) it is at least `behind_distance` (15 m) away and behind the car (more than
  `behind_angle`, 110 degrees, off the heading). While the ego is reversing (its speed along its heading below -0.5 m/s)
  the driver looks backwards: (c) does not apply and (b) only hides spots up to `behind_angle` off the heading, so a spot
  50 to 120 m straight behind is in view; (a) still holds. The optional `<spawning hide_distance="50"
  far_distance="120" side_angle="60" behind_distance="15" behind_angle="110"/>` element changes the numbers. Spawns at
  load are not bound by it. While you approach the crossing head-on, roughly 35 to 60 m before it, no point within 60 m of
  the crossing is hidden, so the pedestrian flow waits there; it spawns again once you are closer (side streets, behind
  you) or past. The single pass destroys its cyclists in view after they have crossed; that is the one place the rule is
  not followed.
- Counts are held (a flow never spawns above its count), actors removed during the run go through
  `CarlaDataProvider.remove_actor_by_id` and everything still alive at the end is removed by ScenarioRunner's cleanup;
  an actor the server will not destroy is retried every second (5 tries) and counts against its flow's count until it is
  gone; AI controllers are destroyed in `terminate()`. A pedestrian whose controller cannot be created, or keeps failing, is
  destroyed rather than left standing.
- A CARLA error in one tick (a spawn, a Traffic Manager call, an actor that is gone) does not end a long session: the
  flow logs it (at most every 10 s), pauses for a second and goes on.

The run ends when the stop file of `<run_control>` appears (`run_scenario.py` creates it when you close the driving
window; the element is written into the run's copy of the XML and is not meant to be written by hand), or with Ctrl-C. In
both cases ScenarioRunner writes its results and removes the actors as at any end. When the stop file appears the ego gets
full brake and the hand brake, so it does not coast on after the window is closed. The run also ends, with a log line, when
the ego vehicle no longer exists (`EgoGone`: it must be gone for 5 ticks in a row), instead of every flow failing on a
missing ego. `ComposedScenario.terminate()` prints `ComposedScenario: scenario ending` first, before the ego is
destroyed, and creates the file `<stop file>.ending`, so the driving window's watcher can tell a run that ends by itself
from a window that was closed. The `CollisionTest` criterion still counts hits, and the result at the end shows them.

## Files

| File | Purpose |
|---|---|
| `pedestrian_crossing.xml` | ScenarioRunner config: map, ego start, blocks and their parameters. Each `<scenario>` element is one variant, with `type="ComposedScenario"`. |

The scenario class is the shared `scenarios/_composed/composed_scenario.py`; the blocks are in `src/carsim/blocks/`
and the crossing, crowd and cyclist geometry in `src/carsim/road.py`.

## Behavior tree

```
ComposedScenario (Sequence)
├── Main (Parallel, ends when one child succeeds)   (autopilot, single pass)
│   ├── EgoAutopilot            WaypointFollower, only when ego_control mode="autopilot"
│   ├── Triggered (Sequence)
│   │   ├── WaitForEgo          InTriggerDistanceToLocation(ego, crossing point, trigger_distance)
│   │   └── TriggeredBehaviours (Parallel, ends when all children succeed, i.e. never)
│   │       ├── Adversary (Sequence)         the main pedestrian
│   │       │   ├── WalkAcross               KeepVelocity(walker, speed, distance = walk length)
│   │       │   └── WaitForever
│   │       ├── Crowd1 ... CrowdN (Sequence) one per <crowd> pedestrian
│   │       │   ├── Delay                    Idle(column's random delay plus 0.7 m / min_speed per row behind the front)
│   │       │   ├── WalkAcross               KeepVelocity(walker, its column's speed, distance = its walk length)
│   │       │   └── WaitForever
│   │       └── Cyclist1 ... CyclistN (Sequence) one per <cyclists> bicycle
│   │           ├── Delay                    Idle(direction's random delay plus gap / speed per place behind the front)
│   │           ├── Appear                   ActorTransformSetter(bicycle, start, physics on); it waited on the sidewalk, physics off
│   │           ├── RideAcross               KeepVelocity(bicycle, speed, force_speed=True, distance = planned push)
│   │           ├── Stop                     StopVehicle(bicycle, brake 1.0)
│   │           ├── Remove                   ActorDestroy(bicycle)
│   │           └── WaitForever
│   └── EgoPassedCrossing       DriveDistance(ego, distance to crossing point + distance_after_crossing)
└── RemovePedestrian            ActorDestroy (main pedestrian; the rest are removed with the scenario's actors)
Criteria: CollisionTest(ego)    a collision fails the run, including with the crowd, the cyclists or background traffic (see below)
```

The tree of a manual run (no start trigger, the flows run from the first tick, nothing ends it but the stop file):

```
ComposedScenario (Sequence)
└── Main (Parallel, ends when one child succeeds)
    ├── PedestrianFlow          pedestrians and crowd; always RUNNING, spawns and removes its walkers each tick
    ├── CyclistFlow             always RUNNING, one state machine per bicycle
    ├── BackgroundBubble        always RUNNING, looks after the traffic every second
    ├── StopFileExists          only with <run_control>: brakes the ego, SUCCESS as soon as the stop file exists
    ├── EgoGone                 SUCCESS when the ego vehicle no longer exists (5 ticks in a row)
    └── ManualRun               WaitForever, so a run without a stop file ends only with Ctrl-C
```

The background vehicles and pedestrians have no behavior of their own: the Traffic Manager and the AI walker
controllers drive them. The controllers are destroyed in `terminate()`, before ScenarioRunner destroys the walkers.

ScenarioRunner adds a timeout around this tree. It starts counting when the scenario is loaded. Without
`<end timeout>` it is `max(60, drive distance / ego speed * 1.5 + 20 + W)` s with the autopilot, where W is 60 s
for a cautious driver (`brake="sight"`, which may wait at the crosswalk) and 0 otherwise. The drive distance is
the distance to the crossing plus `distance_after_crossing`; for the shipped values (about 150 m at 30 km/h) the
timeout is about 107 s. With manual driving there
is no time limit: the timeout is 7 days of game time (ScenarioRunner's `TimeOut` only compares the game time with
it, and its result report is fine with such a number), and the run ends with the stop file or Ctrl-C.

## Parameters (XML)

The full block reference is in the root README ("Scenario blocks"). The shipped values:

| Element | Attribute | Default | Meaning |
|---|---|---|---|
| `ego_vehicle` | `x y z yaw` | – | Ego start; set it with `apps/find_crossing_spots.py --apply N` |
| `ego_control` | `mode` | `autopilot` | `autopilot` follows the lane, a single pass; `manual`: you drive and the run is endless: no timeout, no distance end, it ends when the driving window is closed (stop file) or with Ctrl-C (`run_scenario.py --drive manual` sets this per run) |
| | `target_speed` | `30` | km/h |
| | `brake` | `none` | `sight` (shipped): the cautious driver, which stops for crossers it can see (see "Cautious driver"); `none`: no braking |
| `spawning` | `hide_distance`, `far_distance`, `side_angle`, `behind_distance`, `behind_angle` | `50`, `120`, `60`, `15`, `110` | m, m, degrees, m, degrees: where nothing may appear or vanish (see "Endless manual run"); optional. `far_distance` must not be below `hide_distance` |
| `run_control` | `stop_file` | none | Internal: written by `run_scenario.py` for a manual run with the driving window; the scenario ends as soon as the file exists. Not written by hand |
| `pedestrian` | `model` | `walker.pedestrian.0001` | Walker blueprint |
| | `speed` | `1.4` | m/s |
| | `trigger_distance` | `25` | m between ego and crossing point when the crossing starts (pedestrian, crowd and cyclists together; single pass) |
| | `flow_radius` | `60` | m around the crossing within which a manual run's pedestrian flow spawns pedestrians (also on `crowd`) |
| `crowd` | `count` | `0` (element missing) | Extra pedestrians crossing, about half from each side; `10` in the shipped variants |
| | `min_speed`, `max_speed` | `1.1`, `1.8` | m/s, random speed per column of the crowd |
| | `max_delay` | `4` | s, random start delay 0 to `max_delay` after the trigger for the front of each column; walkers behind start later |
| `cyclists` | `count` | `2` | Bicycles crossing in the cycle lane beside the zebra, about half from each side, waiting on the sidewalk until their turn; `2` in the shipped variants; in a manual run `count` bicycles keep cycling |
| | `min_speed`, `max_speed` | `3.0`, `5.0` | m/s, random speed per direction |
| | `max_delay` | `3` | s, random start delay 0 to `max_delay` after the trigger for the front bicycle of each direction |
| `background` | `vehicles` | `0` (element missing) | 4-wheel vehicles on Traffic Manager autopilot; `10` in the shipped variants |
| | `motorcycles` | `0` | Motorcycles on autopilot; `3` in the shipped variants |
| | `bicycles` | `0` | Bicycles riding in traffic on autopilot; `3` in the shipped variants |
| | `walkers` | `0` (element missing) | Pedestrians wandering via `controller.ai.walker`; `20` in the shipped variants |
| | `radius` | `120` | m around the crossing point in which vehicles and wandering pedestrians are spawned (in a manual run: around the ego, which the traffic follows) |
| `end` | `distance_after_crossing` | `30` | m driven past the crossing point before the scenario ends |
| | `timeout` | autopilot: `max(60, drive distance / ego speed * 1.5 + 20 + W)`, W = 60 s with `brake="sight"`, else 0; manual mode: none (7 days) | s, counted from loading; manual mode has no timeout and no distance end, it ends via the stop file or Ctrl-C |
| `weather` | any `carla.WeatherParameters` field | | e.g. `cloudiness`, `precipitation`, `sun_altitude_angle` |

## Manual driving

With `ego_control mode="manual"` the scenario does not drive the ego; you drive it, and the run is endless (see
"Endless manual run"). Any variant can be driven without editing the XML:

```powershell
python -m uv run python apps\run_scenario.py pedestrian_crossing --drive manual
```

`--drive manual` writes `mode="manual"` into the run's copy of the XML (`outputs\<run>\scenario.xml`) and
starts `manual_control.py` in its own window once the scenario is running; click into it and drive with
W/A/S/D. `--drive autopilot` does the opposite (it keeps the variant's `target_speed`, or uses 30 km/h), and the
default `--drive xml` keeps the variant's own mode. There is no time limit and no distance end. Closing the
driving window ends the run: `run_scenario.py` writes a `<run_control stop_file="..."/>` element into the copy of the
XML and creates that file when the window closes, and the scenario then ends cleanly (results written, actors
removed). Ctrl-C in the terminal ends it too. Without the window (`--no-driver-window`, which prints the
`manual_control.py` command with `--keep_ego_vehicle`) there is no stop file, so the run ends with Ctrl-C. Runs are in
real time by default, which is what you want when driving.

In the window, do not use Ctrl+R (it starts `manual_control.py`'s own recorder, replacing `--record`), Ctrl+P (it
replays a recording mid-run) or P (it switches CARLA's autopilot on). If you close the terminal itself, close the
window by hand and run `python -m uv run python apps\reset_world.py`.

## Expected outcome

The ego starts 120 m before the crossing at 30 km/h. As a cautious driver
(`brake="sight"`) it sees the pedestrian, the crowd and the cyclists cross, stops
before the crosswalk while one of them is on it, and drives on when it is clear.
The expected outcome is **no collision**, so the CollisionTest passes. With
`brake="none"` the ego does not brake: that is the counterfactual run, and the
metrics record its impact, if any. Change
`trigger_distance` or `target_speed` to see the outcome change. (This is the
autopilot single pass; a manual run has no verdict to wait for, it just keeps
going until you stop it.)

With the crowd and the cyclists the test is stricter: `CollisionTest` counts a collision with any crowd pedestrian,
cyclist or background vehicle too, so more things can fail the run. It ignores hits while the ego is slower than
0.1 m/s, and merges hits within 5 m of the previous one, or with the same actor within 5 s, into one. Set `count="0"`
(or remove `<crowd>` and `<cyclists>`) and `<background>` for the single-pedestrian scenario.

To add a variant, copy the `<scenario>` element, give it a new `name` and run
it with `--variant <name>`.

## WolfsburgHighway

`PedestrianCrossing_WolfsburgHighway_1`: the same blocks on the WolfsburgHighway map. The ego drives north on
John-F.-Kennedy-Allee (residential, one lane each way) from (1619.37, -354.76) and reaches the junction at about
(1621, -455) after 90 m (the junction behind the start is too close for 120 m). The map's one usable painted crosswalk
lies on this approach at the junction entry; until `crosswalks()` reads the map's open outlines the crossing is the
junction entry, 0.9 m nearer, and the walk is 8.5 m instead of 11 m. Either way the pedestrian, the crowd and the
cyclists cross both lanes between the two sidewalks; buildings stand 25 m or more away.

The map is a 2 x 2 km Digital Twin of Wolfsburg made with CARLA 0.9.15 (4 x 4 tiles of 500 m). Every road
carries one direction only (a two-way street is two OpenDRIVE roads), there are no shoulders or parking lanes, and the
map's 6 crosswalk outlines are open polygons that `carsim.road.crosswalks()` does not read yet, so crossings fall back
to the junction entry. The ego z is the OpenDRIVE z plus 0.5 m: the road mesh waves about 1.2 m around the OpenDRIVE
surface. The spots were chosen and checked offline with `tools/wolfsburg/wolfsburg_spots.py`; the plots are in
`tools/wolfsburg/plots/` and the reasoning, backups and live checks in `tools/wolfsburg/WOLFSBURG_VARIANTS.md`.
`find_crossing_spots.py` lists painted crosswalks only, so on this map it finds none (pick spots with the tool above).

Walkers use the map's navigation mesh, built by `tools/navmesh` and installed in the map's `Nav` folder: background
walkers wander between sidewalk points, and the pedestrian and the crowd walk on after crossing.

```powershell
python -m uv run python apps\run_scenario.py pedestrian_crossing --variant PedestrianCrossing_WolfsburgHighway_1
```
