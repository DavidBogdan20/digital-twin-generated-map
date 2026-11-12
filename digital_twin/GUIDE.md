# CARLA Digital Twin on a Google Cloud Linux VM (CARLA 0.9.15)

Goal: generate the Wolfsburg map with the Digital Twin Tool on a rented Linux VM, run it there in stock
CARLA 0.9.15, and watch and control it from the MacBook. No 0.9.16 files are used.

The map stays on the VM. A Linux-built map can't move to the Mac's Windows simulator, and the Mac setup
doesn't change.

This path has not been run end to end. Each stage has a stop point, and Claude runs the long work over SSH
with `vm.sh`.

| Stage | Who | Time | What happens |
|---|---|---|---|
| 1. Create the VM | You sign in, Claude creates it | Minutes, or up to days for GPU quota | The VM |
| 2. First test | Claude, then you look | 1 h | Stock CARLA on your MacBook's screen. **Stop here if it fails** |
| 3. Build Unreal | Claude | 1 h 25 min | Unreal at the commit 0.9.15 was released on |
| 4. Build CARLA | Claude | 25 min, then 50 min for the editor's first start | CARLA 0.9.15 with its assets |
| 5. Tiny test | Claude, you watch | 2 h | 500 m map with 37 buildings. **Stop here if it fails** |
| 6. Wolfsburg rectangle | Claude, you watch | 8-13 h | The 4.5 x 2.4 km area from the screenshot |

The times and costs are estimates, at about $1.79/h in Virginia. Stage 5 is about 7 hours, or $18 with about $5
of download traffic. The full map is about a day, or $50. **Stop** the VM when it's idle: the GPU, CPU and memory
charges stop, but the disk keeps costing about $0.06/h until you **Delete** the VM. L4 machines are scarce, so a
stopped VM may not restart until capacity returns.

Files in this folder:

| File | What it is |
|---|---|
| `vm.sh` | Every step that runs on the VM |
| `Dockerfile` | The Ubuntu 20.04 build environment |
| `apply_patches.py` | Applies the fixes listed at the bottom of this guide |
| `open_tool.py` | Runs inside the editor: opens the tool and takes commands from `cmd.py` |
| `facades.py` | Runs inside the editor: gives the grey generated walls CARLA's facade materials |
| `remove_uturns.py` | Removes U-turns from a road file. `vm.sh uturns` runs it on an installed map |
| `move_lights.py` | Moves traffic light stop lines off stub roads in a road file. `vm.sh lights` runs it on an installed map |
| `fix_speed_signs.py` | Makes stock CARLA's 100/110/120 km/h signs set their own limit, not 90. `vm.sh speedsigns` |
| `signals_check.py` | Lists a map's lights and signs and checks that autopilot cars obey them |
| `Wolfsburg_no_uturns.xodr` | The road file now installed on the VM, after `remove_uturns.py` |
| `drive_test.py` | Drives a hero car with traffic and reports how far they get |
| `shots/` | Camera views from the finished map |
| `Wolfsburg.xodr` | Our road network: bridges, roundabout, 2+2 lanes, signs |
| `Wolfsburg.osm`, `Tiny.osm` | Map data with buildings |
| `Wolfsburg_no_buildings.osm`, `Tiny_no_buildings.osm` | The same data without buildings, the fallback |
| `Wolfsburg.Package.json` | Tells CARLA what to package |

Map area: min lat `52.38568`, min lon `10.71738`, max lat `52.40763`, max lon `10.78425`.


## Stage 1: create the VM

The VM is `g2-standard-32` (32 vCPU, 128 GB RAM, one NVIDIA L4 with 24 GB) with the virtual workstation
licence, Ubuntu 22.04 and a 400 GB disk, in Virginia (`us-east4-c`). Every Netherlands, Belgium, London, Frankfurt
and Iowa zone was sold out on 2026-11-05. The licence is what Google documents for Vulkan on a remote desktop. Don't use Spot VMs: they can be taken back in the middle of a build. It goes in the **CARLA** project
(`carla-513807`), away from the organization's prod, nonprod and development projects.

The organization forbids public IP addresses on VMs and requires Google sign-in for SSH. Those rules stay as
they are: the VM is private, reaches the internet through Cloud NAT, and takes SSH through Google's IAP tunnel.

The new project's limits were too low. Google approved these within minutes on 2026-11-05.

| Quota | Was | Now |
|---|---|---|
| GPUs (all regions) | 0 | 1 |
| CPUs (all regions) | 24 | 32 |
| Persistent Disk SSD (GB) in `europe-west4` | 400 | 500 |

Claude created it from the Mac with these commands. Regions other than `europe-west4` allow 400 GB of disk.

```sh
gcloud services enable iap.googleapis.com
gcloud compute routers create carla-router --network=default --region=us-east4
gcloud compute routers nats create carla-nat --router=carla-router --region=us-east4 \
  --auto-allocate-nat-external-ips --nat-all-subnet-ip-ranges
gcloud compute instances create carla --zone=us-east4-c --machine-type=g2-standard-32 \
  --accelerator=type=nvidia-l4-vws,count=1 --no-address \
  --image-family=ubuntu-2204-lts --image-project=ubuntu-os-cloud \
  --boot-disk-size=400GB --boot-disk-type=pd-balanced \
  --maintenance-policy=TERMINATE --restart-on-failure
```

Steps then run through the tunnel:

```sh
gcloud compute ssh carla --zone=us-west1-a --tunnel-through-iap --command="sudo /work/dt/vm.sh <step>"
```

**Moved to Oregon on 2026-11-09.** The original VM in `us-east4-c` wouldn't restart because no GPU was free there. Claude saved
it as the machine image `carla-image` (155 GB, about $0.25 a day) and started a copy in the first US zone with a GPU free,
`us-west1-a`, with the same disk contents. `us-east4-a` refused it because the old disk already fills that region's 400 GB SSD
quota. To move again, in any zone:

```sh
gcloud compute instances create carla --zone=<zone> --source-machine-image=carla-image
```

Oregon has no Cloud NAT, so the copy can't download anything. CARLA doesn't need to, and SSH through IAP still works.


## Stage 2: first test

Claude copies this folder to `/work/dt` and runs `setup`, `start`, `stock` and `run`.

To see the screen, run this in a Mac terminal and leave it open:

```sh
gcloud compute ssh carla --zone=us-west1-a --tunnel-through-iap -- -N -L 5901:localhost:5900
```

Then run this in a second terminal. The password is `carlavnc`.

```sh
open -n /Applications/TigerVNC.app --args -AutoSelect=0 -PreferredEncoding=Tight -QualityLevel=5 -CompressLevel=6 -ViewOnly=1 localhost::5901
```

The screen is view-only. Mac Screen Sharing can leave a modifier key held on the VM, which stops every click.

**Stop and Destroy the VM if stock CARLA doesn't appear.** That costs about $2.


## Stages 3 and 4: builds

Claude runs `fetch`, `engine` and `carla`. `fetch` needs your GitHub access to download Unreal. Claude asks
before passing it. It's held in memory for that one download and isn't saved.


## Stage 5: tiny test

You watch in TigerVNC, read-only. Claude drives the editor: `vm.sh editor` opens it with the tool already up, and
`open_tool.py` runs inside it. Python placed in `/work/dt/cmd.py` runs in the editor, which is how Claude presses
the tool's buttons.

What the tool needs, found by running it on 2026-11-05:

- **The origin comes from the URL.** The boxes under Navigation Settings don't affect Generate. The tool reads
  `bbox=min lon,min lat,max lon,max lat` from the OSM URL and uses the min corner as the origin, so the local URL
  carries one. The local web server ignores it. `Wolfsburg.xodr` uses the same origin, so its roads match.
- **URL:** `http://localhost:8000/Tiny.osm?bbox=10.7414,52.3977,10.7489,52.4022`
- **File Name:** `Tiny`
- **Generate** is `StartButton`, wired to the event `BndEvt__UWG_OSMDownloader_DownloadButton_...`.

After Generate, the tool runs `GenerateTileCommandlet` once per tile (Tiny: 2 tiles of 500 m, 9 minutes). Claude
then saves, closes the editor, runs `package Tiny` and loads the map in stock CARLA.

Buildings worked on 2026-11-05: 35 of Tiny's 37 were built with no crash. If that changes, the next try uses
`Tiny_no_buildings.osm`, which gives roads and trees without buildings.


## Stage 6: the Wolfsburg rectangle

This is the rectangle from the original screenshot and `map_cropped.osm`, not the whole city. `Wolfsburg.xodr` is
`map_cropped.xodr` moved to the rectangle's corner as its origin, because the tool uses that corner.

Claude runs these. In the URL's bbox, the first two numbers set the origin and the last two set the tile count.

```sh
vm.sh wolfsburg Wolfsburg2
vm.sh editor
vm.sh generate Wolfsburg2 'http://localhost:8000/Wolfsburg.osm?bbox=10.71738,52.38568,10.78425,52.4086'
```

**The max latitude is 52.4086, above the rectangle's real 52.40763, on purpose.** The tool makes one row fewer than
the height needs: 2,442 m gave 4 rows of 500 m and would have dropped the northern 445 m. Check for 51 `.umap` files
straight after Generate: 50 tiles plus the base map.

Claude then saves and runs `facades Wolfsburg2` with the editor open (about 2 buildings a second, no regeneration
needed). It does 2,000 per editor session, because the editor crashed at about 2,500, so Claude restarts the editor and
runs it again until the log says 0 left. Claude then closes the editor and runs `package Wolfsburg2`. Traffic Manager needs port 8100. If buildings crash, use
`Wolfsburg_no_buildings.osm`.

**Nothing may be selected in the editor.** On this display the DPI scale is below 1, and `FEdMode::DrawHUD` divides
the viewport size by it as whole numbers, so selecting any actor crashes the editor. Don't click in the viewport, and
clear the selection from Python in the same frame after spawning an actor: both crashed it on 2026-11-05.

Each editor crash leaves a core dump in `/var/lib/apport/coredump`, up to 79 GB, so check the disk after one.

After packaging, `uturns Wolfsburg2` removes the U-turns from the installed road file. It takes seconds and needs no
repackaging, because CARLA reads the `.xodr` at runtime. Then start CARLA with `vm.sh run` and use the map like this:

```python
world = client.load_world("Wolfsburg2")
s = world.get_settings()
s.tile_stream_distance = 600   # set this after every load: the map loads at 2000 and reset_settings=False doesn't keep it
s.actor_active_distance = 450  # keep it below the tile distance, or cars can lose the ground under them
world.apply_settings(s)
```

Give your car `role_name=hero`, so tiles load around it, and move through the map rather than jumping across it.


## Stage 7: drive it from the Mac

After a start or reboot: `start`, `run`, `check Wolfsburg2` (loads the map with the 600 m radius), then `drive`. `drive`
opens CARLA's `manual_control.py` full screen with a car next to the camera: WASD to drive, P for autopilot (the first P
freezes it for several minutes while Traffic Manager builds its road map), ESC to quit.

VNC is too slow for driving, so the picture goes through **Sunshine** on the VM and **Moonlight** on the Mac. The L4
compresses it like a game stream. They connect over **Tailscale**, since the VM has no public address. One-time setup:

1. Cloud NAT in the VM's region, so it can download: `carla-router-west` and `carla-nat-west` in `us-west1`, created
   like Stage 1's, with `--enable-endpoint-independent-mapping` so Tailscale can connect directly.
2. `vm.sh stream` installs both and prints a Tailscale link. Open it and approve the device `carla`.
3. On the Mac: Tailscale (signed in to the same account) and Moonlight (`brew install --cask moonlight`).
4. In Moonlight, add the VM's Tailscale address (`tailscale ip -4` on the VM). Moonlight shows a PIN: `vm.sh pair <PIN>`.
5. Stream at 1280x720, 30-60 fps, about 8 Mbps.

The stream costs Google's internet traffic, $0.12 per GB, plus $0.045 per GB through Cloud NAT: about $0.60 an hour at
8 Mbps, on top of the VM. Nothing while no stream is open.

Learned on 2026-11-09:

- **Installing packages takes the GPU away from the running container.** apt reloads systemd, which resets the
  container's device rules, so `/dev/nvidiactl` returns EPERM inside it. CARLA keeps its open handles until it needs new
  video memory, then dies with "Out of Local Memory". `start` and `stream` now create NVIDIA's `/dev/char` links, which
  prevent it. If it happens anyway, `docker restart carla`.
- **The container shares IPC with the host** (`--ipc=host`), because pygame draws through the X server's shared memory
  (MIT-SHM). Without it the driving window dies with `BadValue` on `X_ShmPutImage`.
- `drive` waits for the old window to exit before starting a new one; starting at once lost the new one.


## Stage 8: the elevation scenario

`elevation_scenario.py` (`vm.sh scenario`): four cars drive north on the A 39 at 90 km/h, past Stralsunder Ring and a
slip road, then over the motorway's own bridge, which the road data puts at 5 m. A fifth car crosses Stralsunder Ring.
CARLA's window shows it full screen, a strip at the bottom shows the lead car's speed, nose angle and height against the
road data, and every frame goes to `logs/elevation.csv`. `vm.sh chart` draws `logs/elevation.png` from it. About 3 minutes.

Order: `run`, `check Wolfsburg2`, wait until video memory stops changing (about a minute), `scenario`, then `chart`.
`elevation_scenario.py --check` only plans the routes and prints them.

What it showed on 2026-11-09 (`shots/elevation_*`, `elevation_run.csv`):

- **Flat road data:** the car rises and falls about 1.2 m every 110 m. That is CARLA's deliberate road deformation
  (`Deformation.h`), and the physics follows it.
- **The ramps exist**, drawn and solid: the car climbs with the road data to about 4 m. The road is a raised strip with
  no embankment under it.
- **Near the top, at about 865 m, every lead car was thrown up to 13-14 m**, nose up to 19 degrees. It landed at ground
  level about 4 m below the road data and stuck against the down ramp. A smooth crest would lift it centimetres, so
  something sharp sits where the ramp meets the bridge section. Not yet seen in the editor.
- **No passing under:** Stralsunder Ring and the slip road are 5 m up only in the road data. In the 3D map they cross the
  motorway at ground level.

How it avoids crashing, all learned the hard way (7 crashes in the container that day, every one while CARLA created
graphics buffers for many meshes at once, the large-map sensor bug aside):

- **No camera sensor.** The large-map manager put a free camera to sleep 40 m from the lead car and crashed
  (`ASensor::EndPlay`). Actors sleep when they are over the active distance from the hero, or on a tile it counts as
  unloaded, and sensors are not exempt from the second. The spectator never sleeps.
- **350 m of tiles, 300 m active distance**, and the script keeps them at the end. 600 m crashed at a tile boundary;
  raising the active distance crashed at once; restoring 600 m after a run crashed two seconds later.
- **No jumps.** It flies to the start at 30 m/s, and glides back to the bridge. Jumps of 360 m and 900 m crashed.
- **Real time.** Synchronous mode ran about 7 times faster than real when unpaced, so the script sleeps to 20 frames a
  second. Unpaced, the cars also streamed the map at about 175 m/s.
- **The quicker car of each lane in front**, and a brake if anyone is within 25 m ahead: the first run's Tesla ran into
  the slower truck in front of it.


## Stage 9: the 2 x 2 km highway map (`WolfsburgHighway`)

A smaller map for testing highway merges: the 2 x 2 km of the rectangle with every A 39 and K 92 merge and exit in
the data (14 of 15; the 15th is on the south edge). Origin 52.38568 N, 10.731333 E, which is x = 950 m in Wolfsburg2,
so Wolfsburg2's (x, y) is (x - 950, y) here. Generated 2026-11-10 with the fixes below.

**Why the old map had trees on the road**, all from the stock tool (`OpenDriveToMap.cpp`, LibCarla `Map.cpp`):

- Every tree, street light, fire hydrant and light box goes 3 m past a road's last driving lane, which is often a
  sidewalk, a slip road, the road beneath a bridge or a road crossing at a junction. Nothing checks for other roads or
  for buildings. One tile of Wolfsburg2 had 405 of them on or within 0.5 m of a lane.
- Each tile places them along the whole length of every road it touches, so they repeat in every tile a road crosses.
- Tree crowns collide and are up to 8.2 m wide. **That was the A 39 launch**: a maple at ground level 1.2 m beside the
  ramp, whose crown was at deck height, at (1203.5, -1435.3) in Wolfsburg2.
- A junction of three or more roads gets a surface that only exists near z = 0, so the raised ones got none: holes a
  car falls through. That is why cars on the bridges ended up 3-4 m below the road data. Wolfsburg2 has 8.
- Roads were wider than the real ones (4 m lanes, 2.8 m sidewalks), so buildings stood on sidewalks and some lanes.
- Most trees are not from those spots at all: the generator blueprint scatters vegetation over each whole tile
  (instances of `BP_InstancedMesh` actors, about 12,000 on the 2 x 2 km map) and checks nothing either. On the new map
  709 of them stood in a driving lane (125) or close enough for the crown to reach it. That is in a binary blueprint,
  so `vm.sh prune` removes whatever `vm.sh audit` rejects, in the editor, and saves the tiles.

**What changed** (`apply_patches.py`, `prepare_dt.py`):

- `PlaceSpot` (tool patch): keeps a spot only in its own tile, moves it 4.5 m from the nearest driving lane, then drops
  it if it is within 1 m of any other lane or 3 m of a building outline from `<map>.keepout`.
- Raised junctions (any road in them over 0.3 m up) use the per-lane surface, which follows height (LibCarla patch).
- Lanes 3.5 m, sidewalks 2 m. Buildings whose outline still reaches onto a lane are left out (42 of 1,252: mostly
  garages and passages over roads in OSM).

Steps, on the Mac then the VM:

```sh
python3 crop_osm.py digital_twin/Wolfsburg.osm box_roads.osm 52.385725 10.731480 52.403564 10.760499
python3 crop_osm.py digital_twin/Wolfsburg.osm box.osm 52.385725 10.731480 52.403564 10.760499 buildings
# set both files' <bounds> min corner to the origin, 52.38568 10.731333 (the crop is 10 m inside it)
DT_NAME=/out/WolfsburgHighway LANE_W=3.5 python load_osm.py box_roads.osm      # in the carla-client container
python digital_twin/prepare_dt.py WolfsburgHighway.xodr box.osm out/WolfsburgHighway   # narrows sidewalks, drops
                                                    # buildings on lanes, writes .keepout, replays the tree placement
vm.sh prepared WolfsburgHighway; vm.sh editor
vm.sh generate WolfsburgHighway 'http://localhost:8000/WolfsburgHighway.osm?bbox=10.731333,52.38568,10.760646,52.404552'
vm.sh audit WolfsburgHighway       # editor still open: lists every mesh of every tile and checks them (check_level.py)
vm.sh prune WolfsburgHighway       # removes the trees and props the audit rejected, saves the tiles (711 on 2026-11-10)
vm.sh audit WolfsburgHighway       # again: must say 0 and 0
vm.sh facades WolfsburgHighway     # tile by tile; this map's building meshes don't load on their own
python rails_plan.py runtime.xodr WolfsburgHighway_rails.json   # in the carla-client container; copy to /work/dt
vm.sh rails WolfsburgHighway       # guard rails on raised roads, then vm.sh audit again
vm.sh meshscan WolfsburgHighway 1  # switches off the distance field of lane meshes that crash the cook (mesh_scan.py)
vm.sh tileoffset WolfsburgHighway  # half a tile of LocalTileOffset, or the north and east edges put actors to sleep
# quit the editor (cmd.py: unreal.SystemLibrary.quit_editor()), then
vm.sh package WolfsburgHighway; vm.sh uturns WolfsburgHighway; vm.sh lights WolfsburgHighway
vm.sh speedsigns                   # once per stock CARLA install
# and copy car_simulator's tools/navmesh/out/WolfsburgHighway.bin to
# /work/stock/CarlaUE4/Content/CustomMaps/WolfsburgHighway/Nav/ for walkers
```

**Found by driving it (2026-11-10, `merge_test.py`), fixed in the second build of the same day:**

- **Sidewalks across the carriageway.** The tool builds every sidewalk as a raised strip with 1 m deep sides. At
  junctions of two roads (and the raised ones above) it builds every connecting road's sidewalk without checking, so
  where a ramp with a sidewalk merges, its sidewalk lay across the main road: cars stopped dead on 4 of 23 routes.
  `prepare_dt.py` now turns every sidewalk lane that lies on another road's driving lane into a `border` lane of the
  same width (47 lanes here), so nothing else moves. The walker navmesh is built from the same file, and its polygons
  on those sidewalks crashed the server (`FWalkerActor::SetWalkerState` on a walker with no root component): rebuild
  the navmesh whenever the road file changes.
- **Keep every tile loaded.** Streaming tiles in and out as a car moved crashed the server twice in one test
  (`FVulkanResourceMultiBuffer`, VulkanIndexBuffer.cpp:137, a buffer copy into a failed allocation). This map's 16
  tiles take 7.3 GB of video memory, so `map_test.py` sets 3000 m streaming and active distance: nothing streams, and
  no actor or sensor is put to sleep. Do not lower it on this map.
- **No guard rails.** Raised ramps and bridges are bare strips: a car that leaves its lane falls 3 to 5 m. Normal
  lane keeping is fine; the drive test's first driver cut tight ramp curves and fell.
- **The container lost the GPU mid-run** ("Failed to initialize NVML", the server hangs at start with 300 MB of video
  memory): snapd refreshed a snap at 15:45 and the systemd reload took the GPU devices away, as apt did on 2026-11-09.
  `vm.sh start` now holds snap refreshes and stops the apt timers; if it happens anyway, `docker restart carla` and
  `vm.sh start`.
- **Decks over lanes (third build, 2026-11-10 evening).** netconvert gives each connecting road of a junction its own
  height ramp, so at interchanges lanes overlapped in plan at different heights: one deck's edge stood up to 0.9 m
  in the next lane (cars stopped dead at (1185, -1505)). `check_heights.py` finds every such spot (lanes of any type
  over another road's driving lane, 0.15 to 4 m apart at the same point); there were 24. `prepare_dt.py` now
  (1) flattens the A 39 stubs that climbed to a bridge outside the map (`EDGE_STUBS`), (2) blends roads that enter or
  leave a junction side by side to one height over their first 40 m, (3) gives every junction's connecting roads
  their heights from one height field of that junction (anchored on the roads around it), and (4) gives the former
  sidewalks over lanes zero width instead of a border surface. Left: one point 0.20 m apart inside a short junction
  that climbs 2 m. Steepest grade 14 % (15 % before).
- **Guard rails.** `rails_plan.py` traces the outer edges of every road over 1 m up (one-way roads on both sides),
  leaving gaps where another road comes within 2.5 m; `vm.sh rails <map>` places CARLA's `SM_Guardrail_02` along
  those lines in the editor (about 4.9 km here), and `vm.sh audit` fails if a rail stands on a driving lane.
- **The cook crashed on zero-width lanes** (Signal 11 after 824 of 7,747 packages, "Array index out of bounds: 362
  from an array of size 362" in `TkDOPNode::SplitTriangleList`, building the distance field of
  `SM_DrivingLaneMesh84_X_1_Y_3`). Unreal's split reads past the end of its list when more than 4 triangles share
  one centre; a zero-width lane collapses 6 triangles onto one point wherever two of its rows meet (that mesh had
  groups of 7). `vm.sh meshscan <map> 1` finds such meshes and sets their distance field resolution to 0, which
  skips that build; run it before every package of a map from `prepare_dt.py`.
- **Actors went dormant at the north and east edges, and walkers there crashed the server.** The tool lays tile X_Y
  out from its south-west corner and leaves the large map manager's `Tile0Offset` at 0, but
  `ALargeMapManager::GetTileVectorID` takes that offset as tile 0's centre. Every location was looked up half a tile
  off, so the last 250 m in the north (y < -1750) and east (x > 1750) belonged to a tile that does not exist: any actor
  there went dormant at once, with every tile loaded (`dormant_grid.py x0 x1 y0 y1 step z` lists them: a grid of cones, `is_dormant`). Stock
  0.9.15 then crashes when the client's walker navigation moves a dormant walker (`FWalkerActor::SetWalkerState`
  calls `GetActor()->GetRootComponent()` before checking dormancy, CarlaActor.cpp:1221): the cyclist cut-in with
  background walkers crashed this way on builds 2 and 3. `vm.sh tileoffset <map>` sets `LocalTileOffset`, which only
  that lookup uses, to half a tile; repackage after it. Wolfsburg2 has the same offset (not fixed there).
- netconvert left two junction pieces (roads 3684, 3727) whose lanes step 3.5 to 5 m sideways between lane sections;
  waypoint-following clients see a jump there.
- **Cars rocked and jolted (2026-11-11; fixed in the fourth build, below).** `jiggle_test.py async|sync [blueprint] [s]
  [m/s] [x,y]` drives a car
  with the driving window's chase camera and reports frame-to-frame jitter of the car, the camera and the car's place
  in the picture. Parked: none. Centred on the straight A 39 at 76 km/h: under 0.2 degrees and 1.5 px a frame, in
  async and sync alike, so neither the camera nor the frame timing shakes. Two things do move the car:
  (1) **slow rocking everywhere**: the road data there is flat, yet the car pitches over 6 degrees and rolls over
  5 degrees. That is the tool's `GetHeight` (no heightmap) baking 0.7 x `GetZPosInDeformation` (Deformation.h, waves
  up to 1.2 m, slopes up to about 4.5 degrees) into every road mesh; it predicts the car's pitch and roll with r = 0.999.
  (2) **jolts on single-lane ramps**: the paved surface is exactly the 3.5 m lane, with no shoulder, and the ground
  beside it lies 0.2 to 0.6 m lower. A car more than about 0.7 m off centre (cutting or drifting wide on the A 39 loop
  at 52 km/h) drops a wheel off the edge: roll jumps of up to 3 degrees a frame. The road data there is at ground
  level: the drop is the tool's ground, sampled every 2.56 m and put 80 cm down wherever a sample lies under any mesh,
  so a trench up to 80 cm deep runs beside every road without a sidewalk (74 km of lane edge here). The tool's driving
  lane "bumps" (`GetBumpDeformation`) are dead code: `DistanceToLaneBorder` returns metres minus the lane width and is
  compared with 65, so it is never true (measured: no dent where the formula puts one).
  **Fourth build:** `apply_patches.py` makes `GetHeight` return 0 without a heightmap (roads, marks, sidewalks, trees
  and ground keep the road file's heights) and puts the ground under meshes 12 cm down instead of 80; `rails_plan.py`
  rails roads from 0.15 m up instead of 1 m (6.7 km of rail instead of 4.9; the ground beside lies at about 0, so no
  bare step over about 0.15 m stays) and no longer adds the waves. `surface_check.py` checks both live: the road
  mesh against the road data at lane centres, and the ground past every open lane edge (no trench under -0.15 m).
  Left: road 3094 dips to -0.04 m over 5 m, where the ground (-0.01 m) shows about 3 cm over its edge, as before.
  The fourth build's editor crashed once, 36 s after `facades` (an assertion in VulkanMemory.cpp:3721, as `rails`
  loaded its first tile): every step saves tile by tile, so reopen it (`vm.sh editor`) and go on from the next step. On an existing
  build: `python3 apply_patches.py` and `make CarlaUE4Editor` in the container (70 s), then regenerate. car_simulator
  lists the map in `carsim.road.FLAT_MAPS` (no wave offset, the sidewalk lift stays) and its navmesh builder too.
- **A light was run and 100 km/h signs gave 90 (2026-11-12, `signals_check.py`; fixed without repackaging).**
  Where netconvert splits a junction in two with a stub road (Herzbergweg, 1.5 m), CARLA puts the light's trigger box
  on the stub, so cars saw red 2 m past the line and stopped inside the junction: `vm.sh lights` (`move_lights.py`)
  moves such stop lines to the end of the feeding road (2 of 9 lights) and drops a yield sign standing there. CARLA
  0.9.15's `BP_SpeedLimit100/110/120` set 90 from their own trigger box: `vm.sh speedsigns` (`fix_speed_signs.py`)
  changes that float in stock CARLA. After both: 9/9 lights, 3/3 stop signs and every speed change checked out.

The tool saves the map itself when Generate ends (the tiles are written by one commandlet process per tile), so no
Save step is needed. `facades.py` sets a material override on each building in its tile: unlike Wolfsburg2's, these
building meshes fail `load_asset` ("Failed to find object") unless their tile is loaded.

The bbox is 1,995 m wide and 2,100 m tall on purpose: the tool rounds columns up and rows down, so this gives 4 x 4
tiles of 500 m. Then facades, package, uturns as in Stage 6, and `merge_test.py` (below).

`check_level.py` fails if any tree or light is on a lane, within 0.5 m of one or within 4 m of a driving lane, if
a building's middle is on a lane, or if a raised junction has no road mesh at its height. `merge_test.py` drives a car
through all 17 on-ramps and exits (both highways) and fails on a launch (1.5 m over the road data), a fall (2.5 m
under), a stop of over 3 s or a route not finished.


## How this stays on 0.9.15

- **No 0.9.16 files.** The tool, its blueprints and the simulator that runs the map are all 0.9.15's.
- **Tag 0.9.15.2** changes no code in `Unreal/` or `LibCarla/` compared with 0.9.15, only links and docs.
- **The engine is pinned to commit `8d08ab7fe`**, the state of the `carla` branch when 0.9.15 was released.
- **The map runs in stock CARLA 0.9.15**, the official Linux download with no edits. The edits below only
  affect the machine's build tools.

## What the edits do

| Edit | Where it comes from | Why it's needed |
|---|---|---|
| Keep our roads, in `CustomFileDownloader.cpp` | Written for this project | Uses `Wolfsburg.xodr` instead of converting the OSM again |
| Turn off material baking, in `MeshMergeUtilities.cpp` | Written for this project | The buildings crash happens in this optional step. It worked on Tiny, but walls come out plain grey |
| Keep the parent of dynamic materials, in `MeshMergeUtilities.cpp` | Written for this project | Meant to fix the grey walls. It didn't: they still use Unreal's default material |
| Null check in `DistanceFieldAtlas.cpp` | Same fix as Unreal fork commit `a288ae07e` | The packaging crash in #7785 fails on exactly this line |
| Null check in `PrepareAssetsForCookingCommandlet.cpp` | Same fix as later CARLA commit `d7228f746` | Packaging crash on meshes with no material |
| Null check in `Map.cpp` | Same fix as later CARLA PR #7167 | Tree placement crash on lanes with no speed limit |
| `PlaceSpot` in `OpenDriveToMap.cpp` | Written for this project (Stage 9) | Trees and street lights stood on roads and repeated in every tile |
| Per-lane surface for raised junctions, in `Map.cpp` | Written for this project (Stage 9) | Junctions on ramps and bridges had no surface |
| New Boost and libpng links in `Setup.sh` | Official download sites | The old links fail |
| `--exclude=/Engine/Source/ThirdParty/Licenses` on Unreal's `Setup.sh` | Setup's own option | One licence PDF returns 403 |

The two OSM files also had tags with `&`, `|` or `*` removed, because users traced failures to those characters.

## What is still unknown

- Nobody has reported this exact combination working. Stages 2 and 5 test it.
- Google documents Vulkan on the L4 with the virtual workstation licence, but no report of CARLA on it was
  found. Stage 2 checks that `nvidia-smi -q` shows the licence and that Vulkan reaches the GPU in the container.
- The remote screen goes through the tunnel to Virginia, so it will lag more than a nearby direct connection.
- It isn't confirmed that free trial credit pays for GPUs after upgrading. Check **Billing** after the first
  hour. Whatever the credit doesn't cover goes on your card.
- Linux users on plain 0.9.15 report that buildings crash. The material baking edit stopped that on Tiny (35 of
  37 buildings), but the full map has 3,206 and may find another crash.
- Generated walls are grey, so `facades.py` gives each building one of 21 CARLA facades. They're plain textured
  blocks without glass windows, chosen in turn rather than to match the real building.
- **Autopilot isn't reliable on this map yet.** After removing the U-turns, a car drove 697 m from one start and 103 m
  from another (61 m before the fix). It stops at full lock against kerbs on ordinary tight turns, and half speed didn't
  help. Roads are built wider than the real ones (4 m lanes plus 2.8 m pavements), so some buildings beside the street
  stand on a lane, for example at (2153, -904). Fixing that means narrower roads, which needs a new Generate and package.
- Most "vanishing" cars are Traffic Manager clearing stuck ones: it removes a car idle for 90 seconds, one every 10.
  Not all of them, though. In one run the hero itself disappeared while driving at 29 km/h, after 212 m near
  (2953, -977), and two traffic cars went the same way. That didn't happen again and the cause is unknown.
- **The simulator can crash.** The 6 crashes on 2026-11-05 were: 3 on long view jumps or a fast sweep, 2 at map load
  and 1 straight after a jump, and 5 of them were in Vulkan buffer allocation. The sixth, at map load, isn't traced.
  The likely reason is Unreal's limit of 8,192 buffer allocators (`ALLOCATOR_INDEX_BITS`), and it's untested. OpenGL
  isn't available in this build. No run crashed while driving at a 600 m radius: 6 runs, up to 4 minutes each.
- Traffic Manager needs port 8100 here, because the map server uses 8000.
- Roads sit up to about 1 m below the OpenDRIVE height, because the generator deliberately adds waves to the road
  surface. Cars rest on the surface, so `get_location().z` can read around -1 on a road at z 0.
- Generated maps can differ from the real roads. Users reported inaccurate maps, and #9006 reports vehicles
  vanishing in parts of a generated 0.9.15 map.
- A maintainer wrote in April 2026 that the tool isn't under active development and dense city areas are a problem.
