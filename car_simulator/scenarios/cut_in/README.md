# cut_in

A vehicle or cyclist on the lane ahead moves into the ego's lane when the ego gets close, and the ego has to brake
or follow it. Two variants run on Town10HD_Opt and two on WolfsburgHighway (see the last section).

- `CutIn_Parking_Town10HD_1`: the ego drives in the kerb lane. On the shoulder ahead a parked van stands, and just in
  front of it, further from the ego, a parked car pulls out into the ego lane when the ego is 15 m away.
- `CutIn_Cyclist_Town10HD_1`: the ego drives in the inner lane. In the kerb lane ahead, car A, a cyclist and a fire
  truck (nearest the ego) ride at the cyclist's speed, and the truck hides the cyclist. When the ego is 14 m away, the cyclist moves
  into the ego lane and rides on ahead of it.

The ego runs on autopilot with `brake="sight"`: a cautious driver that uses line-of-sight rays from the simulator.
It brakes for any visible actor in its path and follows a visible slower vehicle or cyclist in its lane. The scenario
is built from blocks: the XML names them, `ComposedScenario` (`scenarios/_composed/composed_scenario.py`) runs them.

Both variants include random background traffic and pedestrians (`<background>`), and each run writes `metrics.json` (see "Run metrics" in the repository README).

## Blocks used

| Block | Role |
|---|---|
| `ego_control` | Autopilot at 30 km/h, `brake="sight"` |
| `cut_in` | `kind="parking"` (distance 66, trigger 15 m, 4 m/s) or `kind="cyclist"` (distance 40, trigger 14 m, 4 m/s) |
| `background` | Random traffic and pedestrians around the ego: 8 vehicles, 2 motorcycles, 2 bicycles, 15 walkers, radius 120 m |
| `end` | Ends after 100 m (parking) or 90 m (cyclist) of driving, or after 90 s |

## Expected outcome

The ego brakes for the car or the cyclist when it comes into the ego's lane, or follows it at its speed. It does not
collide, so the run passes.

## Starting point

The ego starts are set in the XML. Parking needs a start in the kerb
lane with a shoulder on its right. Cyclist needs a start in the inner lane with a kerb lane on its right, and a
straight road ahead of the cut-in point.

## Running

```powershell
python -m uv run python apps\run_scenario.py cut_in --variant CutIn_Parking_Town10HD_1
python -m uv run python apps\run_scenario.py cut_in --variant CutIn_Cyclist_Town10HD_1
```

## WolfsburgHighway

- `CutIn_Parking_WolfsburgHighway_1`: Konrad-Adenauer-Allee (two lanes each way, 162 m without a junction), ego
  start (1558.63, -1100.43). The map has no shoulder or parking lane, so the van and the car stand double parked in
  the kerb lane and the ego drives in the inner lane, left of them; the car pulls out of the kerb lane into the ego
  lane. This needs `cut_in kind="parking"` to accept a same-direction Driving kerb lane where there is no shoulder
  (the proposed change in `tools/wolfsburg/WOLFSBURG_VARIANTS.md`); until then the variant stops with the shoulder
  error.
- `CutIn_Cyclist_WolfsburgHighway_1`: Dresdener Ring (two lanes each way, 122 m without a junction, a gentle 7 degree
  bend), ego start (888.31, -1691.46) in the inner lane; the convoy rides in the kerb lane, which has a sidewalk on its
  right and a broken line to the ego lane. No change to the blocks.

The map is a 2 x 2 km Digital Twin of Wolfsburg made with CARLA 0.9.15 (4 x 4 tiles of 500 m). Every road
carries one direction only (a two-way street is two OpenDRIVE roads), there are no shoulders or parking lanes, and the
map's 6 crosswalk outlines are open polygons that `carsim.road.crosswalks()` does not read yet, so crossings fall back
to the junction entry. The ego z is the OpenDRIVE z plus 0.5 m: the road mesh waves about 1.2 m around the OpenDRIVE
surface. The spots were chosen and checked offline with `tools/wolfsburg/wolfsburg_spots.py`; the plots are in
`tools/wolfsburg/plots/` and the reasoning, backups and live checks in `tools/wolfsburg/WOLFSBURG_VARIANTS.md`.
`find_crossing_spots.py` lists painted crosswalks only, so on this map it finds none (pick spots with the tool above).

```powershell
python -m uv run python apps\run_scenario.py cut_in --variant CutIn_Parking_WolfsburgHighway_1
python -m uv run python apps\run_scenario.py cut_in --variant CutIn_Cyclist_WolfsburgHighway_1
```
