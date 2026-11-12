# WolfsburgHighway scenario variants

Where each scenario variant happens on the WolfsburgHighway map (CARLA 0.9.15 Digital Twin of Wolfsburg, 2 x 2 km,
4 x 4 tiles of 500 m), why, the backups, the adaptations the map forces, and what must be checked live.

All numbers come from `tools/wolfsburg/wolfsburg_spots.py`, which loads the runtime OpenDRIVE offline with the CARLA
0.9.15 Python API (`carla.Map(name, xodr)`, no server) and runs the repository's own geometry on it
(`carsim.road.find_crossing`, `occluder_transforms`, `prop_transform`, `crowd_spawns`, `cyclist_spawns`,
`cyclist_waiting_spots`, `lane_point`, `straight_route`, and `find_crossing_spots.is_inner_lane`). Its last output is
`tools/wolfsburg/validation_output.txt`; the plots are in `tools/wolfsburg/plots/`.

## Rerun

```sh
docker build -t carla-client-analysis:0.9.15 tools/wolfsburg      # carla-client:0.9.15 + matplotlib, shapely
docker run --rm -v "$PWD":/repo -v /Users/bogdandavid/Coding/CARLA/car_simulator_inputs:/inputs \
    carla-client-analysis:0.9.15 python /repo/tools/wolfsburg/wolfsburg_spots.py --repo /repo \
    --xodr /inputs/WolfsburgHighway.xodr            # add --search for the ranked candidate tables
```

`--repo` is the checkout whose `src/` and `apps/` are imported (the analysis was run against a snapshot taken before
the 0.9.15 port; rerun it against the ported tree). The chosen spots and backups are the `CHOICES` and `BACKUPS`
dicts at the top of the script. Each crossing spot is checked twice: on the map as the simulator reads it (`raw`) and
with the crosswalk outlines closed (`closed`, what the proposed `crosswalks()` fix gives); plots carry the suffix.

## What the map is like (findings that drive the choices)

1. **Every OpenDRIVE road is one-way** (right lanes only). A two-way street is two roads side by side; the urban
   tertiary roads with two lanes per direction (Konrad-Adenauer-Allee, Dresdener Ring, Frankfurter Strasse,
   John-F.-Kennedy-Allee south part) are dual carriageways with a 9 to 10 m median. There are no `primary` roads.
   Lane layouts: residential `driving, sidewalk`; urban tertiary `driving, driving, sidewalk` (some 3 to 5 lanes);
   trunk K 92 (Braunschweiger Strasse) and motorway A 39 `driving x2..4`, no sidewalk.
2. **No Shoulder, Parking, Border or Biking lane anywhere** (lane types are only `driving` and `sidewalk`).
3. **The 6 crosswalks are open 4-corner outlines** (`closed="false"`, no repeated first corner). CARLA returns their
   24 corners as one flat list, and `carsim.road.crosswalks()` splits polygons on a repeated first point, so it finds
   **none**: live, every crossing falls back to the junction entry. Proposed fix below. Even with it, only two
   crosswalks lie on a usable approach (both residential, one lane per direction) and **none lies on a road with two
   lanes per direction**. The crosswalks are 1 m wide (Town10's are 2.5 m), their z is 0.
4. Lane changes between same-direction lanes are allowed everywhere (broken lines: kerb lanes report `Left`, inner
   lanes `Right`), so `LaneChange` with its permission check works.
5. Urban roads are flat (OpenDRIVE z 0). Non-zero elevation only on and near the A 39 / K 92 interchange bridges
   (up to 5 m) and on the Konrad-Adenauer-Allee / Frankfurter Strasse overpass ramps. No chosen drive passes under or
   over another road (checked: no road within 6 m horizontally that is more than 3 m higher or lower).
6. Speed limits: urban 13.89 m/s (50 km/h), K 92 27.78 m/s (100 km/h). The offline map has no spawn points (they
   come from the server), so background vehicle spawning can only be checked live.
7. On-ramps (K 92 junctions 219 and 173, A 39 junctions 252, 259, 260) join as an **added lane**: on the K 92 the
   ramp becomes lane -3 of a 3-lane road of about 94 m whose lane -3 then ends (a real acceleration lane).

**Ego z.** The road mesh waves about +-1.2 m around the OpenDRIVE z and cars sat 0.5 to 0.75 m below it, so every
WolfsburgHighway ego start uses **z = OpenDRIVE z + 0.5 m** (0.50 here: all chosen starts are at OpenDRIVE z 0).
That clears the mesh where it is up to 0.5 m above the OpenDRIVE z and drops the car about 1 to 1.3 m where it sits
0.5 to 0.75 m below. The port agent's ground settling should replace this anyway. The other actors are placed by the
blocks at the lane z (+0.2 for vehicles, +1.0 for walkers, +0.1 / +0.15 for bicycles), which has the same problem; see
the live checks.

**Navigation mesh.** `tools/navmesh` builds it from the map's OpenDRIVE; it is installed in the map's `Nav` folder
(rebuild it whenever the road file changes). With it, background walkers wander, the pedestrian and crowd walk on to
navigation points after crossing, and the manual flows run. Without it the variants still work the way Town12 does
(README "Town12 has no navigation mesh"). Nothing in the XMLs depends on which.

**Tile streaming.** Every actor of every variant is within 150 m of the ego start (background radius 120 m around
the crossing or the ego start), well inside the ~300 m around the hero.

---

## 1. PedestrianCrossing_WolfsburgHighway_1

**Requirements from the code.** `ComposedScenario` takes `map.get_waypoint(ego xy)` as the start lane;
`find_crossing` follows it to the next junction (within 500 m) and takes the crosswalk the lane crosses between 20 m
before and 10 m inside the junction entry (axis across the road, |dz| < 3 m), else the junction entry. The walk runs
from the right sidewalk (or 1 m beyond the outermost lane) to the left side, pushed off the driving lanes. The ego
lane must be the kerb lane (pedestrian steps off on its right), the ego route is `straight_route` through the junction
for crossing + 30 + 20 m, the end is 30 m past the crossing, the shared trigger 25 m. Crowd (10) and cyclists (2)
need room at both kerbs and sidewalk waiting spots; background vehicles spawn 25 m or more from the ego, within
120 m of the crossing, not on the ego's route lanes. Town10 used a 120 m straight approach.

**Chosen.** John-F.-Kennedy-Allee (residential), driving north to the junction at about (1621, -455).

| | |
|---|---|
| ego | x 1619.37, y -354.76, **z 0.50**, yaw -86.35 |
| road / lane | 3177 / -1 (`residential|John-F.-Kennedy-Allee`), the only lane of this direction |
| crossing (raw map) | junction entry (1621.69, -443.81), 89.0 m ahead, walk 8.5 m from (1624.44, -443.82) to (1615.94, -443.80) |
| crossing (crosswalk fix) | painted crosswalk centre (1619.9, -444.7), crossing point (1621.69, -444.71), 89.9 m ahead, walk 11.0 m |
| approach | heading change 4 deg overall, 1 deg in the last 40 m, 2 deg in the 30 m after; flat (dz 0) |
| actors | crowd 14/14 spots (10 used), cyclists 2/2 with 2 sidewalk waiting spots; nearest building 25 m |
| end | 119 to 120 m from the start |
| plots | `plots/PedestrianCrossing_WolfsburgHighway_1_raw.png`, `..._closed.png` |

**Why.** It is the only approach on the map with a painted crosswalk that passes every check: straight, flat,
residential, sidewalks on both sides, straight on through the junction, no bridge. The opposite carriageway is
directly beside the ego lane (no median), so the walk crosses both lanes from sidewalk to sidewalk as on Town10.
The other painted crosswalk with a 120 m approach (Erfurter Ring, ego (1168.9, -1710.9)) ends at a T-junction where
the ego would have to turn 95 deg: rejected.

**Backup.** Goerdelerstrasse (residential), ego (1605.81, -499.86, z 0.50, yaw 179.15), road 2978 lane -1, westbound
to the junction entry (1486.82, -498.13), 119 m, dead straight (0/0/0 deg), no painted crosswalk; crowd and cyclists
fit, buildings 18 m away. Use it to get Town10's 120 m approach at the cost of the painted crosswalk.

**Adaptations.** Approach 90 m instead of 120 m: the previous junction on John-F.-Kennedy-Allee is too close for 120 m
(at 30 km/h and a 25 m trigger, 90 m still gives about 7.8 s of cruise). Blocks and parameters are otherwise the
Town10 ones.

**Expected behaviour.** The cautious ego (30 km/h) sees the pedestrian, crowd and cyclists step off at the 25 m
trigger, stops before the crossing, waits until it is clear and drives on; no collision; run ends about 120 m from the
start.

**Verify live.**
1. The ego spawns, settles on the road mesh and is on road 3177 lane -1 (the log's crossing distance is about 89 m,
   or 90 m with the crosswalk fix).
2. With the `crosswalks()` fix in, the log does not say "no painted crosswalk found"; without it, it does.
3. Pedestrian, crowd and cyclists spawn at their spots (the walkers at lane z + 1.0, the waiting bicycles at lane z
   + 0.15 with physics off): none floating visibly or buried in the mesh; count the "spawned n/10 crowd" line.
4. The crowd and cyclists reach the far sidewalk (it is the sidewalk of the opposite road 2891).
5. Background: how many of 10 vehicles spawn (the live spawn point count near (1621, -444)); without a navmesh the log
   says wandering pedestrians were skipped.
6. The ego stops before the crossing, waits, drives on straight through the junction; no collision; metrics.json.

---

## 2. OccludedCrossing_Child_WolfsburgHighway_1 and 3. OccludedCrossing_Crowd_WolfsburgHighway_1

**Requirements from the code.** The ego in the inner lane: the lane right of it a same-direction Driving lane that is
the rightmost one (`is_inner_lane`), the ego going straight on through the junction (within 30 deg for 40 m). The
truck (european_hgv, 7.9 m) and one queued car stand in that kerb lane, the truck's front `gap` 1.5 m before the near
edge of the crosswalk (the crossing point without one), every vehicle centre outside the junction
(`occluder_transforms`). The bus stop (3.9 x 1.9 m) stands on the kerb side, its crosswalk end 0.3 m before the near
edge, reaching 1.1 m past the child's start (`prop_transform`): the sidewalk must have room and no building. The
child (overlap 50) starts when the ego's time to the impact point equals its walk time; at 23 km/h the ego must then
be on the straight. The crowd variant adds a 35 m trigger, 14 walkers and 2 cyclists from both sides. Town10 started
90 m before the crossing.

**Chosen (both variants).** Konrad-Adenauer-Allee (tertiary), a dual carriageway with a 10 m median, driving south
west to the junction with a side street at about (1464, -1232).

| | |
|---|---|
| ego | x 1512.79, y -1156.03, **z 0.50**, yaw -125.85 |
| road / lane | 2964 / -1 (`tertiary|Konrad-Adenauer-Allee`, lanes -1 inner, -2 kerb, -3 sidewalk) |
| crossing | junction entry (1464.10, -1231.60), 90.0 m ahead (the same on both maps: no crosswalk here) |
| walk | from the right sidewalk (1469.36, -1234.98) across the kerb and inner lane into the median, end (1461.79, -1230.12), 9.0 m |
| truck, queued car | kerb lane, truck front 1.5 m before the junction entry, car behind it 1.5 m clear (see the plot) |
| bus stop | child variant: on the sidewalk 0.3 to 4.2 m before the junction entry; not on a driving lane; building 15.7 m away |
| approach | heading change 3 deg, 0 deg in the last 40 m, 0 deg in the 30 m after; flat |
| actors (crowd) | crowd 14/14, cyclists 2/2 with 2 waiting spots; nearest building 15.7 m |
| end | 120 m from the start, 30 m past the crossing |
| plots | `plots/OccludedCrossing_Child_WolfsburgHighway_1_raw.png`, `plots/OccludedCrossing_Crowd_WolfsburgHighway_1_raw.png` (`_closed` identical) |

**Why.** Of the 25 inner-lane approaches of 90 m on the map it is the only one that is straight before and after the
junction, flat, urban, with a sidewalk on the kerb side, room for truck, queue and bus stop, all crowd and cyclist
spots, and buildings near the corner (a residential block with a side street, the classic child-behind-obstruction
picture). A 2D line-of-sight check (ego eye 2.6 m ahead of its centre, the truck, car and bus stop footprints) shows
the waiting child hidden by the bus stop at every point of the approach, and also by the truck from 45 m to the
crossing. The child (1.39 m/s, 6.25 m to the ego lane centre plus the start-up) starts about 4.9 s before the impact,
with the ego about 31 m away, on the straight.

**Backup.** Frankfurter Strasse (tertiary, 3 lanes + sidewalk), ego (944.40, -1827.24, z 0.50, yaw -110.10), road 3026
lane -2 (the middle lane; the truck stands in lane -3), junction entry (919.02, -1912.90) 90 m ahead, walk 12.5 m into
a 9 m median, all actors fit; but it bends 25 deg after the junction, has no buildings near (open land) and is 87 m
from the map's south edge. Second backup with a shorter approach: Dresdener Ring, road 2997 lane -1, ego
(812.61, -1603.51, yaw -47.6), 75 m to (863.5, -1658.5).

**Adaptations.**
- **No painted crosswalk on any road with two lanes per direction**, so the child and the crowd cross at the junction
  entry: the scenario's built-in fallback, no code change. The truck's gap is then measured to the crossing point
  (near edge = crossing point), the bus stop sits 0.3 m before it, and the child starts on the sidewalk at the
  junction corner. NCAP's CPNCO (child behind an obstruction) does not require a zebra; the intent (child steps out
  from behind a stopped truck into the ego lane, 50 % overlap) is kept.
- The far side of the walk is the **10 m median** (no sidewalk lane), not the far sidewalk: the opposite carriageway
  is a separate road 10 m away. The child crosses the two ego-direction lanes as on Town10. In the crowd variant half
  of the crowd and one cyclist start from the median (a pedestrian refuge); their ground z is the road's.
- After the junction the road climbs gently towards an overpass: about 0.4 m in the 30 m the ego drives past the
  crossing, 1.2 m at 50 m. Accepted (the end condition stops the run before the ramp steepens).

**Expected behaviour.** Child: the ego (23 km/h) does not see the child until it steps out of the truck's shadow,
then brakes (emergency brake likely); pass if no collision, metrics give first-visible TTC and brake onset. Crowd: the
ego (30 km/h) stops for the visible crossers, waits, drives on.

**Verify live.**
1. The occluder log says the european_hgv stands 1.5 m before the crosswalk with 1 queued car (it must spawn, or the
   scenario stops); both are in road 2964 lane -2 and settle on the mesh (not dropped through, not tilted).
2. The bus stop (a static prop placed at the lane z, no physics) sits on the sidewalk surface, not floating or
   buried, and not inside a tree or street light of the 3D map (trees keep 1 m from sidewalks, so it should not).
3. The child spawns behind the bus stop (log: "pedestrian spawned" without offsets, or with one of the retry offsets).
4. The child's walk start line ("starts when the ego is 4.9 s from the 50 % impact point") and that the counterfactual
   (`brake="none"` copy) hits at about the ego's centre line.
5. The ego cannot see the child until it leaves the truck's shadow (metrics `first_visible`), then brakes.
6. Crowd variant: walkers and the cyclist of the median side spawn there and are not floating or buried; the median
   is passable for them (the 3D map may have kerbs, trees or barriers in the median).
7. Background vehicles do not drive into the stopped truck's lane behind it during the run (they queue behind it).

---

## 4. CutIn_Parking_WolfsburgHighway_1

**Requirements from the code.** `point = lane_point(ego_start, distance)`; `point.get_right_lane()` must be a
**Shoulder** (else `ValueError`). The car (mini, 66 m ahead) stands on it pushed to the outer edge, the van (sprinter)
1.5 m clear behind it; at a 15 m trigger (centre to centre) the car is launched at 4 m/s and a `BasicAgentBehavior`
takes it into the ego lane, 10 m ahead, then 60 m on. The ego is in the kerb lane. The end is 100 m of driving; the
lane must be straight and junction-free over roughly distance + 30 m and the end + 10 m.

**Chosen.** Konrad-Adenauer-Allee (tertiary), the same carriageway as the occluder, but starting 10 m after the
previous junction.

| | |
|---|---|
| ego | x 1558.63, y -1100.43, **z 0.50**, yaw -131.08 |
| road / lane | 2964 / -1 (inner lane); the "parking" lane is 2964 / -2 (kerb lane), sidewalk -3 beyond it |
| car, van | car (1519.32, -1153.11) and van (1523.42, -1147.62) in lane -2 (before the 0.6 m push to the kerb) |
| stretch | 162 m without a junction; heading change 9 deg over 110 m (a gentle bend around the parked cars); flat |
| plot | `plots/CutIn_Parking_WolfsburgHighway_1.png` |

**Why.** The longest junction-free, flat, urban stretch with two lanes per direction and a sidewalk on the kerb side.

**Backup.** Dresdener Ring, road 3159 lane -1, ego (888.31, -1691.46, z 0.50, yaw 138.86), 122 m junction-free,
7 deg bend; car (838.41, -1647.68), van (843.21, -1652.67).

**Adaptation (needs code).** The map has **no shoulder or parking lane at all**, so the closest faithful layout is a
**double-parked** van and car (a delivery stop, common in German streets) in the kerb lane of a two-lane carriageway,
with the ego in the inner lane: the car still pulls out from behind a vehicle that hides it into the ego's lane, at
the same distances, speed and trigger. The lateral offset of the hidden car from the ego (lane width 3.5 m plus the
push of about 0.6 m to the kerb) is close to Town10's (kerb lane plus a 2 to 2.5 m shoulder). The block refuses a
non-shoulder lane today; the minimal change (`src/carsim/blocks/cut_in.py`, not applied):

```diff
@@ class CutIn(Block): def spawn(self):
         point = lane_point(ctx.ego_start_wp, self.distance)
         side = point.get_right_lane()
         if self.kind == "parking":
-            if side is None or side.lane_type != carla.LaneType.Shoulder:
-                raise ValueError("<cut_in kind=\"parking\"> needs a Shoulder on the right of the ego lane {:g} m ahead; "
-                                 "pick a start in a kerb lane".format(self.distance))
+            if side is None or not (side.lane_type == carla.LaneType.Shoulder or is_kerb_lane(point, side)):
+                raise ValueError("<cut_in kind=\"parking\"> needs a Shoulder on the right of the ego lane {:g} m ahead, "
+                                 "or on a map without one the kerb lane (ego in the lane left of it)".format(self.distance))
+            if side.lane_type != carla.LaneType.Shoulder:
+                self.log("parking: no shoulder, the van and the car stand double parked in the kerb lane")
             self._spawn_parking(point, side)
@@
+def is_kerb_lane(wp, side):
+    """True when `side`, the lane right of `wp`, is the rightmost Driving lane and goes the same way (no shoulder there)."""
+    if side.lane_type != carla.LaneType.Driving or (side.lane_id > 0) != (wp.lane_id > 0):
+        return False
+    outer = side.get_right_lane()
+    return outer is None or outer.lane_type != carla.LaneType.Driving
```

Everything else already works on a driving lane: `_place(parked=True)` pushes each vehicle (lane width - half width)
/ 4 to the right (about 0.6 m in a 3.5 m lane: both stay inside it), and the merge plan is built on the ego lane.
`tests/check_cut_in_offline.py` should get a case with a fake kerb lane (Driving, same direction) instead of the
Shoulder, plus a README block-reference line ("parking: the ego is in the kerb lane with a shoulder on its right, or
on a map without shoulders in the lane left of the kerb lane"). Until the change lands this variant stops at load with
the shoulder error; the XML says so.

**Expected behaviour.** The ego (30 km/h, inner lane) passes behind the van; the car, launched when the ego is 15 m
away, pulls into the ego lane about 10 m ahead of its spot; the cautious ego sees it late (hidden by the 2.6 m van),
brakes and follows it at 4 m/s; no collision.

**Verify live.**
1. With the change: the log line "parking: no shoulder ... double parked"; car and van stand in lane -2, inside it,
   hand brake on, settled on the mesh.
2. The car pulls out into lane -1 without hitting the van (the agent's plan starts 10 m ahead in the ego lane).
3. The ego's first sighting of the car (metrics) comes late, after it starts to move; it brakes and follows; no
   collision.
4. Background vehicles that spawn in lane -2 behind the van queue there (lane changes are off) and do not ram it.
5. The run ends after 100 m of driving, before the next junction (162 m).

---

## 5. CutIn_Cyclist_WolfsburgHighway_1

**Requirements from the code.** The right lane at `distance` (40 m) must be a same-direction Driving lane; in it the
bicycle, car A 3 m clear ahead and the firetruck (8.5 m) 1.5 m clear behind, all forced to 4 m/s. At 14 m (centre to
centre) the bicycle does `LaneChange(left, distance_same_lane=2, distance_lane_change=8, distance_other_lane=10)`,
which needs a lane-change permission to the left on the kerb lane and fails with a junction in the way. The ego is in
the inner lane; end 90 m of driving. With the ego at 8.33 m/s and the convoy at 4 m/s the trigger comes after about
6 s (the ego about 50 m from its start, the cyclist about 64 m), and the lane change needs about 20 m more: junction-free for
roughly 100 m.

**Chosen.** Dresdener Ring (tertiary), two lanes each way plus a sidewalk, driving south west.

| | |
|---|---|
| ego | x 888.31, y -1691.46, **z 0.50**, yaw 138.86 |
| road / lane | 3159 / -1 (inner); convoy in 3159 / -2 (kerb lane, `lane_change` Left, sidewalk -3 beyond) |
| convoy at load | cyclist (856.52, -1666.44), car A (852.14, -1661.91), firetruck (861.24, -1671.27) |
| stretch | 122 m without a junction (the next junction bends 60 deg, after the end of the run); 7 deg bend; flat |
| plot | `plots/CutIn_Cyclist_WolfsburgHighway_1.png` |

**Why.** A junction-free, flat, urban two-lane carriageway long enough for the convoy, the trigger, the lane change
and the 90 m end, with a sidewalk beside the kerb lane (a realistic place for a cyclist) and buildings around, and on
a different street from the other variants.

**Backup.** Frankfurter Strasse, road 3026 lane -2 (the middle of 3 lanes; convoy in lane -3), ego (956.56, -1796.80,
z 0.50, yaw -112.22), 122 m junction-free, 8 deg bend. A third: Konrad-Adenauer-Allee road 2964 lane -1 from
(1558.63, -1100.43) (the parking start, 162 m).

**Adaptations.** None: the existing blocks express it. (The ego starts 10 m after a junction exit; background vehicles
spawn at least 25 m away from it.)

**Expected behaviour.** The ego approaches the convoy in the next lane; the firetruck hides the cyclist; at 14 m the
cyclist moves into the ego lane; the ego sees it, brakes and follows it at 4 m/s; no collision.

**Verify live.**
1. The convoy spawns in lane -2 at the listed spots, upright, on the mesh, and holds its gaps while riding.
2. The `LaneChange` does not fail at the trigger (the scenario would end at once): the log shows the cyclist in lane -1.
3. The ego's first sighting of the cyclist comes after it leaves the truck's shadow; it brakes and follows; no collision.
4. The run ends after 90 m, before the junction 122 m ahead.

---

## Highway cut-in proposal (not implemented)

The map exists for highway merges. The best place is the **K 92 (trunk Braunschweiger Strasse) on-ramp at junction
219**: main road 3011 (2 lanes, 100 km/h, flat, no bridge) and the trunk_link 3051 become road 3139 (3 lanes); its
lane -3 is a real **acceleration lane of about 94 m that ends** (no successor). Plot:
`plots/HighwayMerge_K92_proposal.png`.

- Ego: road 3011 lane -2 (right main lane), 40 m before the merge: (1138.85, -1320.58, z 0.50, yaw 130.84); it goes
  on in 3139 lane -2, the lane left of the acceleration lane, so the layout is exactly cut_in's "ego in the inner lane,
  actor in the kerb lane", and the acceleration lane permits a change to the left.
- Merging car: 5 m into the acceleration lane (3139 lane -3, which starts at about (1110, -1293)); it changes left
  into the ego lane before the lane ends. The lane is only 94 m long, so the timing is tight: with the ego at 70 km/h
  (19.4 m/s), the car at 12.5 m/s (45 km/h, still accelerating in reality) and a 25 m trigger (centre to centre, the
  cut_in convention), the ego closes the 45 m to 25 m in about 2.9 s, when the car is about 41 m into the lane,
  leaving about 50 m for the change. The ego starts only 40 m before the merge because it starts at its test speed;
  starting it further back would need the car spawned on the ramp, which cut_in cannot do (it places actors along the
  ego's lane at load).
- Second site: junction 173 (road 2909 + link 3064 into 3152, the same 94 m acceleration lane, the opposite direction).
  The A 39 merges are less suitable: at junction 259 the ramp feeds lane -1 (the left lane) of road 3191, a data quirk,
  and junction 252 joins a 4-lane road at a bridge end.

**Can existing blocks express it?** Not faithfully. `cut_in kind="cyclist"` has the right lane geometry (actor in the
kerb lane, `LaneChange` left at a trigger distance, the ego in the lane left of it) but the cut-in actor is always a
random bicycle, car A and a blocker are always spawned, and the `LaneChange` distances (8 m for the change, 10 m in the
other lane) are bicycle-sized: at 20 m/s that is a violent 0.4 s swerve. `kind="parking"` needs a parked car. A small
new kind does it (a sketch: the `from_start` part is pseudocode for the py_trees nodes):

```diff
--- src/carsim/blocks/spec.py
-CUT_IN_KINDS = ("parking", "cyclist")
+CUT_IN_KINDS = ("parking", "cyclist", "merge")
--- src/carsim/blocks/cut_in.py
-BLOCKERS = {"parking": "vehicle.mercedes.sprinter", "cyclist": "vehicle.carlamotors.firetruck"}
-DEFAULTS = {"parking": (60.0, 15.0, 4.0), "cyclist": (25.0, 12.0, 4.0)}
+BLOCKERS = {"parking": "vehicle.mercedes.sprinter", "cyclist": "vehicle.carlamotors.firetruck", "merge": "none"}
+DEFAULTS = {"parking": (60.0, 15.0, 4.0), "cyclist": (25.0, 12.0, 4.0), "merge": (30.0, 20.0, 18.0)}
@@ def spawn(self):
-        else:
+        else:  # cyclist and merge: a same-direction Driving lane on the right (the acceleration lane for merge)
             ...
-            self._spawn_cyclist(side)
-        self.ctx.target = self.actors["car" if self.kind == "parking" else "cyclist"]
+            self._spawn_merge(side) if self.kind == "merge" else self._spawn_cyclist(side)
+        self.ctx.target = self.actors["cyclist" if self.kind == "cyclist" else "car"]
+
+    def _spawn_merge(self, side):
+        """The merging car on the acceleration lane at `distance`; a blocker behind it unless blocker="none"."""
+        car = self._spawn("car", self.ctx.rng.choice(vehicle_ids(self.ctx.library, base_type="car", lit=self.ctx.night)), side)
+        self._place(car, side)
+        if self.blocker != "none":
+            van = self._spawn("blocker", self.blocker, lane_point(side, -SPAWN_SPACING))
+            self._place(van, lane_point(side, -(half_length(car) + BEHIND_GAP + half_length(van))))
+        self.ctx.lights_on(car)
@@ def from_start(self):
+        if self.kind == "merge":
+            car = self.actors["car"]
+            ride = Parallel(SUCCESS_ON_ONE): WaypointFollower(car, speed), InTriggerDistanceToVehicle(ego, car, trigger_at)
+            change = LaneChange(car, speed, direction="left", distance_same_lane=5,
+                                distance_lane_change=max(8.0, 1.5 * speed), distance_other_lane=50)
+            return [Launch([car] + blocker, speed, name="LaunchMerge"),  # once, not hold: the car keeps its own speed
+                    Sequence(ride, change, WaypointFollower(car, speed))]
```

plus `"none"` allowed for `blocker` in `spec.check_params` / the blueprint check, a README block-reference line and a
`check_cut_in_offline.py` case. The XML would then be:

```xml
<scenario name="CutIn_Merge_WolfsburgHighway_1" type="ComposedScenario" town="WolfsburgHighway">
    <ego_vehicle x="1138.85" y="-1320.58" z="0.50" yaw="130.84" model="vehicle.lincoln.mkz_2017" rolename="hero"/>
    <ego_control mode="autopilot" target_speed="70" brake="sight"/>
    <cut_in kind="merge" distance="45" trigger_distance="25" speed="12.5"/>
    <end distance="180" timeout="60"/>
    <background vehicles="10" radius="200"/>
</scenario>
```

Open points for it: the driver's 50 m sight range and its comfort-stop logic were tuned at 30 km/h; at 70 km/h the
stopping distance is about 40 m, so check `driver.py` before trusting results. Check also that `end distance` 180 m
stays before the next junction on road 3139's successor (not measured), and that the Traffic Manager's background
cars on the acceleration lane do not block the car.

---

## Proposed fix: read the map's crosswalks

`src/carsim/road.py` (not applied). The Digital Twin Tool writes crosswalk outlines without repeating the first
corner, so the flat point list never closes a polygon. Every outline it writes has 4 corners:

```diff
 def crosswalks(cmap):
     ...
     polygons, current = [], []
     for point in cmap.get_crosswalks():
         if len(current) > 2 and point.distance(current[0]) < 0.05:
             polygons.append(Crosswalk(current))
             current = []
         else:
             current.append(point)
+    # Maps from CARLA's Digital Twin Tool (WolfsburgHighway) write open 4-corner outlines: nothing ever closes
+    if not polygons and current and len(current) % 4 == 0:
+        polygons = [Crosswalk(current[i:i + 4]) for i in range(0, len(current), 4)]
     return polygons
```

`wolfsburg_spots.py` asserts on the real map that this gives the same 6 polygons as closing the outlines in the
OpenDRIVE (adding the first `cornerLocal` again to each crosswalk object, the alternative data fix). Add a case to
`tests/check_crossings_offline.py` with a fake map returning two open 4-point outlines. Only the pedestrian crossing
variant changes with it (crossing point 0.9 m further, walk 11 m on the paint instead of 8.5 m); the occluder and
cut-in spots are the same either way. Note the crosswalks are 1 m wide: the crowd's outer columns (up to 1.25 m off
the centre line) and the cycle lanes (2.1 and 3.15 m) lie beside the paint, not on it.

## Live checks that apply to every variant

1. The map loads as `WolfsburgHighway` and `run_scenario.py` finds ground at each ego start (large-map wait).
2. Ground settling: the ego (z 0.50) and every placed actor end up on the mesh, not under it. Most at risk are the
   ones the blocks place with physics off at a computed z: the cyclists waiting on the sidewalk (lane z + 0.15) and
   the bus stop prop (lane z). The mesh is up to 1.2 m off the OpenDRIVE z.
3. Background: the live `get_spawn_points()` count around each spot (the log's "spawned n/8 background cars (k spawn
   points within 120 m ...)"); a Digital Twin map may have few or none.
4. Without a navmesh the logs say walkers were skipped and the scenarios still run; with one, the walk-on and manual
   flows work.
5. No variant's drive passes a tile seam with missing ground (watch the chase view).
6. `apps/list_scenarios.py --ours` shows no problems once ScenarioRunner is installed (the variant names are new and
   cannot collide with its examples; validated offline otherwise).
