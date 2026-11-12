# occluded_crossing

The ego vehicle drives in the inner lane (the lane left of the kerb lane, same direction) towards a junction.
In the kerb lane, just before the crosswalk, a truck is stopped, with one car queued behind it. The truck hides
the crosswalk from the ego.

Two variants on Town10HD_Opt (and the same two on WolfsburgHighway, see the last section):

- `OccludedCrossing_Child_Town10HD_1`: a child waits on the right kerb behind a bus stop (`prop`) just before the
  crosswalk, then walks right to left across it in front of the truck. The bus stop hides the waiting child from the
  ego lane at any distance, and the truck hides it as it walks out, until it is about 15 m from the crossing. The ego
  sees it only for a moment on the way, while the child is between the shadows of the bus stop and the truck.
- `OccludedCrossing_Crowd_Town10HD_1`: the same truck, and a busy crosswalk: a pedestrian and a crowd of 14 from
  both sides, plus 2 cyclists.

The child variant runs at 23 km/h, not 30: the child starts walking when the ego is speed x 6.8 s from it, and on
Town10HD the ego must then be on the last straight before the crossing, where the truck and the bus stop still hide
the child. Both variants start 90 m before the crossing, on the straight before the curve.

Both variants include random background traffic and pedestrians (`<background>`), and each run writes `metrics.json` (see "Run metrics" in the repository README).

The ego runs on autopilot with `brake="sight"`: a cautious driver that uses line-of-sight rays from the simulator.
It stops before the crosswalk while a visible crosser is on it, waits until the way is clear, and brakes for any
visible actor in its path. The scenario is built from blocks: the XML names them, `ComposedScenario`
(`scenarios/_composed/composed_scenario.py`) runs them.

## Blocks used

| Block | Role |
|---|---|
| `ego_control` | Autopilot at 23 km/h (child) or 30 km/h (crowd), `brake="sight"` |
| `occluder` | The truck (`gap="1.5"`), one queued car (`queue="1"`) and, in variant 1, a bus stop on the sidewalk (`prop`) |
| `pedestrian` | The child (`age="child" speed="1.39" overlap="50"`, variant 1) or the main pedestrian (variant 2) |
| `crowd` | 14 pedestrians (variant 2 only) |
| `cyclists` | 2 bicycles (variant 2 only) |
| `background` | Random traffic and pedestrians around the ego: 8 vehicles, 2 motorcycles, 2 bicycles, 15 walkers, radius 120 m (both variants) |
| `end` | Ends 30 m past the crossing, or after 90 s (child) or 120 s (crowd) |

The child's walk starts by time, not distance: when the ego's time to arrival at the impact point equals the child's
walk time to it. Without braking the ego hits the child at `overlap` percent of its width (50: the centre line, as
NCAP CPNCO) at any ego speed.

## Expected outcome

The ego stops for the crosser, or brakes when the child steps out of the truck's shadow. It does not collide, so the
run passes. In the crowd variant the ego stops for the crossing pedestrians and cyclists, waits until they are clear,
and drives on.

## Starting point

The ego starts are set in the XML. To pick a new one, choose a start on the inner lane
that leads into a junction with a crosswalk, with the lane option set to inner:

```powershell
python -m uv run python apps\find_crossing_spots.py --scenario occluded_crossing --variant OccludedCrossing_Child_Town10HD_1 --lane inner --apply N
```

`--apply N` writes the x/y/z/yaw of the chosen spot into the variant.

## Running

```powershell
python -m uv run python apps\run_scenario.py occluded_crossing --variant OccludedCrossing_Child_Town10HD_1
python -m uv run python apps\run_scenario.py occluded_crossing --variant OccludedCrossing_Crowd_Town10HD_1
```

## WolfsburgHighway

`OccludedCrossing_Child_WolfsburgHighway_1` and `OccludedCrossing_Crowd_WolfsburgHighway_1`: Konrad-Adenauer-Allee, a
dual carriageway with two lanes each way and a 10 m median. The ego starts in the inner lane at (1512.79, -1156.03)
and drives 90 m straight south west to the junction with a side street at about (1464, -1232). No crosswalk on this
map lies on a road with two lanes per direction, so the crossing is the junction entry (the scenario's own fallback):
the child walks from the right sidewalk across the kerb lane and the ego lane into the median. The truck stands in
the kerb lane 1.5 m before the junction entry, the bus stop (child variant) on the sidewalk just before it; a 2D check
shows the waiting child hidden by the bus stop over the whole approach and also by the truck from 45 m. The child
starts about 4.9 s before the impact, the ego then about 31 m away. In the crowd variant the far side of the walk is
the median, so the crowd and the cyclists of that side wait there. Beyond the junction the road climbs gently (about
0.4 m in the 30 m the ego drives past the crossing).

The map is a 2 x 2 km Digital Twin of Wolfsburg made with CARLA 0.9.15 (4 x 4 tiles of 500 m). Every road
carries one direction only (a two-way street is two OpenDRIVE roads), there are no shoulders or parking lanes, and the
map's 6 crosswalk outlines are open polygons that `carsim.road.crosswalks()` does not read yet, so crossings fall back
to the junction entry. The ego z is the OpenDRIVE z plus 0.5 m: the road mesh waves about 1.2 m around the OpenDRIVE
surface. The spots were chosen and checked offline with `tools/wolfsburg/wolfsburg_spots.py`; the plots are in
`tools/wolfsburg/plots/` and the reasoning, backups and live checks in `tools/wolfsburg/WOLFSBURG_VARIANTS.md`.
`find_crossing_spots.py` lists painted crosswalks only, so on this map it finds none (pick spots with the tool above).

```powershell
python -m uv run python apps\run_scenario.py occluded_crossing --variant OccludedCrossing_Child_WolfsburgHighway_1
python -m uv run python apps\run_scenario.py occluded_crossing --variant OccludedCrossing_Crowd_WolfsburgHighway_1
```
