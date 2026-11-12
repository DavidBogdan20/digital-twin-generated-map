# Pedestrian navmesh for WolfsburgHighway (CARLA 0.9.15)

CARLA's walkers (`controller.ai.walker`, `world.get_random_location_from_navigation()`) need a Recast/Detour
navmesh `<map>.bin`. The Digital Twin Tool makes none, so `build_navmesh.py` builds it from the map's OpenDRIVE
file the same way CARLA builds one for `generate_opendrive_world`, at the heights of the generated 3D surfaces.

Outputs in `out/`: `WolfsburgHighway.bin` (install this), `navmesh.png` (whole map, 0.5 m/px, north up: blue
sidewalk, grey road, yellow crosswalk, red rings mark the 6 crosswalks), `crosswalk_0..5.png` (60 m close-ups,
0.1 m/px, crosswalk outline in red).

## Install (VM)

```
/work/stock/CarlaUE4/Content/CustomMaps/WolfsburgHighway/Nav/WolfsburgHighway.bin
```

The server finds the file by name anywhere under `CarlaUE4/Content` (first match), so the folder is a convention
(the one `Util/BuildTools/Import.py` uses). The name must be the episode's map name, which on a large map is the
`ALargeMapManager::LargeMapName` saved in the level. `check_navmesh_live.py` prints it (`map .../X: the server
looks for X.bin`); if X is not `WolfsburgHighway`, rename the file. No server restart is needed: the server reads
the file on every request, and each client fetches it once per episode, so start a new client process (or
`load_world`) after copying.

## Rebuild

```
python3 tools/navmesh/build_navmesh.py          # about 2.5 min; re-runs itself in Docker
python3 tools/navmesh/build_navmesh.py --skip-recast   # only re-validate out/WolfsburgHighway.bin and redraw
```

Defaults read `../car_simulator_inputs/WolfsburgHighway.xodr`, `.keepout` and `RecastBuilder` (the VM's 0.9.15
build). Needs the `carla-client:0.9.15` image (python:3.8-slim + `carla==0.9.15`, amd64). Nothing from 0.9.16.

## Live check (orchestrator, on the VM with the server running)

```
PYTHONPATH=$(ls /work/carla/PythonAPI/carla/dist/*py3.8*.egg) python3 check_navmesh_live.py [--no-load]
```

Loads the map, sets `tile_stream_distance` 600 / `actor_active_distance` 450 right after `load_world`, centres
on the hero, or with none glides the spectator (150 m legs, 2 s each) to crosswalk 0 (`--crosswalk N`), samples
20,000 navigation points and keeps those within 150 m, then spawns 2 walkers that cross the nearest crosswalk
(sidewalk to sidewalk, both ways) plus 8 going to random nearby sidewalk points, with
`set_pedestrians_cross_factor(0)` so roads are off limits. After 60 simulated seconds it prints per walker the
distance walked, distance left and the share of samples on sidewalk / crosswalk / road / off-lane, and a SUMMARY
line. Exit 0 when every walker moved. No sensors; it destroys what it spawned (unless `--keep`). Crosswalk
centres: 0 (1620,-449), 1 (780,-1545), 2 (862,-1443), 3 (1163,-1831), 4 (1620,-445), 5 (450,-20).

## How CARLA does it (0.9.15 source, tag `0.9.15` = d7b45c1e)

Loading, `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/`:
- `Util/NavigationMesh.cpp:25-36`: file name `MapName + ".bin"`, `FindFilesRecursive` under
  `ProjectContentDir()`, first match.
- `Server/CarlaServer.cpp:476-484`: RPC `get_navigation_mesh` calls it with `Episode->GetMapName()`.
- `Game/CarlaGameModeBase.cpp:91-112`: on a large map `MapName = LMManager->LargeMapName`.
- `Server/CarlaServer.cpp:458-468`: `get_map().name` is `<content dir>/<MapName>`, so its basename is the file name.
- `Util/BuildTools/Package.sh:333-345` packs any `<map>.bin` found under Content; `Import.py:494-565` puts imported
  maps' navmesh in `Content/<package>/Maps/<map>/Nav/` and has tiled (large) maps disabled ("until we have a new
  Recast adapted to work with tiles"), which is why large maps get none.

Building for OpenDRIVE-only worlds:
- `Game/CarlaEpisode.cpp:169-171`: `GenerateMesh(vertex_distance)` + `GetAllCrosswalkMesh()` ->
  `GenerateOBJForRecast()`, written to `Content/Carla/Maps/Nav/OpenDriveMap.obj`; `:216` runs
  `RecastBuilder <obj>` (`:127-144` finds it in `Tools/` or `Util/DockerUtils/dist/`), which writes `<obj>.bin`.
  `Util/DockerUtils/dist/build.sh` does the same for imported FBX maps.
- `LibCarla/source/carla/geom/Mesh.cpp:202-244`: OBJ is `v x z y` (Recast is Y-up, CARLA coordinates with y
  south), `usemtl <name>` per material, faces reversed.
- `LibCarla/source/carla/road/MeshFactory.cpp:105-106`: material `sidewalk` for Sidewalk lanes, `road` for every
  other lane. `road/Map.cpp:1252-1286`: crosswalk objects become material `crosswalk`; `Map.cpp:499-504` widens
  each 1 m past both ends "to contact with the sidewalk". There is no `grass` in this path.
- RecastBuilder = `github.com/carla-simulator/recastnavigation`, branch `carla` (e23d9199, Oct 2023):
  `RecastBuilder/Source/main.cpp:54,88` (`RecastBuilder <file.obj> [agent radius]`, output `<file>.bin`);
  `MeshLoaderObj.cpp:198-228` maps `road|crosswalk|sidewalk|grass` to areas 3|2|1|4, anything else to 0 (block),
  and resets the material on every `v` line; `Sample.cpp:134-150` cell 0.2 m, cell height 0.01 m, agent 1.8 m
  high, radius 0.2 m, max climb 0.30 m, slope 45 deg; `Sample_TileMesh.cpp:74,125` 256-cell (51.2 m) tiles, up to
  2^14 tiles; `:586-615` stamps each triangle's area on the spans from its lowest vertex up 5 m, in file order;
  `:749-775` area -> flags, tiles with only unflagged polys are dropped; `Sample.cpp:320-358` file format (MSET v1).
- `LibCarla/source/carla/nav/Navigation.h:26-44` same areas and flags; `Navigation.cpp:219-227` walker filter 0
  excludes ROAD, filter 1 allows it (chosen per walker by the cross factor, `:493-498`); `:1074`
  `get_random_location_from_navigation` returns SIDEWALK polys only.

## What the build does

- Every Driving and Sidewalk lane of every lane section (the only two types the Digital Twin Tool gives a mesh,
  `CarlaTools/.../OpenDriveToMap.cpp:525-590`) becomes a strip 2 vertices wide, sampled every 1 m with
  `carla.Map.get_waypoint_xodr` (lane centre +- half `lane_width`), material `road` or `sidewalk` as above.
- Junction sidewalks follow `road/Map.cpp:1525-1590` with the project's patch (`digital_twin/apply_patches.py`):
  in a junction of 3+ roads lying within 0.3 m of z = 0, a sidewalk lane whose midpoint is on a driving lane is
  left out (6 here). Junction driving areas use the per-lane strips (the tool's SDF fill is only at z ~ 0 there).
- Crosswalks: `carla.Map.get_crosswalks()` (already widened 1 m). Their outlines are not closed, so CARLA's own
  `GetAllCrosswalkMesh` would find no fan and `generate_opendrive_world` would have no crosswalks; the script splits
  them by 4 corners. Crosswalks 1 and 2 come out slanted along the road (that is CARLA's reading of the xodr
  heading, same polygon the scenarios get), and 1 and 5 have no sidewalk at one end.
- Triangles whose centre is inside a building outline (`.keepout`) are dropped (25). There are no other walkable
  areas (no grass), so buildings only matter where they overlap lanes.
- Heights, `OpenDriveToMap.cpp:532,539,749-800` and `road/Deformation.h`: surface = OpenDRIVE z
  + 0.7 * GetZPosInDeformation(x, y), plus 0.1524 (`road/Lane.cpp:258`) + 0.15 for sidewalks. WolfsburgHighway's
  fourth build (2026-11-11) uses `digital_twin/apply_patches.py`'s flat roads, so for it (`FLAT_MAPS`) the wave term
  is 0: rebuild its navmesh with this version. Two changes:
  roads and crosswalks are raised 0.0324 m so the curb is 0.27 m, because Recast's max climb is exactly 0.30 m and
  a 0.3024 m curb would cut every sidewalk off the road and crosswalks; and all vertices are written 0.02 m lower
  to cancel Recast's measured rasterization bias. Driving-lane bumps (GetBumpDeformation, up to -0.10 m, mostly
  removed by the tool's 15 % decimation) are ignored.
- Triangles are written lowest first and crosswalks last, so a road under a bridge cannot relabel the deck.
- RecastBuilder runs with its defaults (no radius argument), as CARLA runs it.

## Validation (offline, last build)

- 708 tiles of 51.2 m (39 x 39 grid, max 2048 tiles, 2048 polys/tile), 7.2 MB; every tile parsed to its exact size.
- Polys: sidewalk 5,732 (73,240 m2), crosswalk 14 (53 m2), road 6,049 (245,672 m2), unflagged 110 (14 m2).
  Input: 2,189 lanes, 119.8 km, 155,218 road + 86,579 sidewalk + 12 crosswalk triangles.
- Navmesh (detail mesh) height minus 3D surface height: sidewalk lane centres (4,334 points, 4,324 on the
  navmesh): mean -0.002 m, median |d| 0.004 m, 95 % 0.010 m, max 0.158 m. Driving lane centres: +0.034 m mean
  (the 0.0324 lift), 95 % 0.043 m. Crosswalk centres: +0.033 to +0.040 m.
- Crosswalks 0, 3, 4 reach sidewalk on both ends in the close-ups; 7 of 14 crosswalk polys touch a sidewalk poly
  in the same tile (cross-tile links are only checkable live).
