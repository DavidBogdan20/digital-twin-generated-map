# Moving to CARLA 0.9.16 on a new machine

The old machine is the Google Cloud VM `carla` (europe-west4-b), running CARLA 0.9.15. The new one can be any Linux
machine with an NVIDIA GPU: another cloud or your own PC. Nothing below needs Google Cloud except step 1.

The map isn't generated again. The old VM's editor content for WolfsburgHighway, already fixed (prune, facades, rails,
meshscan, tileoffset), goes into a 0.9.16 build and is cooked again. The fixed road file and walker navmesh are copied
unchanged. The repo's state on 0.9.15 is the tag `carla-0.9.15`.

## What the new machine needs

- Ubuntu 22.04 or 24.04, root access and internet. The build runs in an Ubuntu 22.04 container (`digital_twin/Dockerfile`).
- An NVIDIA GPU with Vulkan. The old VM had an L4 with 24 GB; CARLA asks for at least 8 GB.
- At least 32 GB of RAM to build Unreal. The old VM had 128 GB.
- About 500 GB free for `/work`. The old VM used 301 GB.
- A GitHub account linked to Epic Games, for `vm.sh fetch` (Unreal's source).
- A headless machine is assumed. `setup` writes `/etc/X11/xorg.conf` for a virtual 1920x1080 screen, and `start` runs
  X on `:0`. On a PC that already shows a desktop, back up `xorg.conf` first.

## 1. Old VM: pack the bundle (about 1 hour of VM time)

`vm.sh bundle` packs what the new machine can't rebuild into `/work/bundle`. Run the new `vm.sh` from `/tmp`, so the
old VM's own copy stays as it was. From the repo on the Mac:

```sh
gcloud compute instances start carla --zone=europe-west4-b --project=carla-513807  # on STOCKOUT, retry
gcloud compute scp digital_twin/vm.sh carla:/tmp/vm.sh --zone=europe-west4-b --project=carla-513807 --tunnel-through-iap
gcloud compute ssh carla --zone=europe-west4-b --project=carla-513807 --tunnel-through-iap --command='sudo bash /tmp/vm.sh bundle'
```

| File | What it is |
|---|---|
| `maps.tar.zst` | Every map's editor content from `/work/carla/Unreal/CarlaUE4/Content/CustomMaps`, what `recook` cooks |
| `installed.tar.zst` | Each installed map's `OpenDrive` and `Nav` folders from stock CARLA: the road file after `uturns` and `lights`, and the navmesh |
| `<map>_*.tar.gz` | The newest 0.9.15 cooked package of each map, for the fallback below |
| `vm-files.tar.zst` | The VM's `/work/dt`, `/work/logs` and `/work/car_simulator/outputs` |
| `MANIFEST.txt` | Date, maps, sizes, and md5 sums of the installed road files and navmeshes |
| `SHA256SUMS` | Checksums of all of the above |

`vm.sh bundle WolfsburgHighway` packs one map instead of all. Credentials stay on the VM: no Sunshine or VNC
password files, no Tailscale state, nothing from `/root`.

## 2. Copy the bundle off and stop the VM

Over Tailscale (the VM is `carla` in the tailnet), to the Mac:

```sh
rsync -a --partial --progress -e 'ssh -i ~/.ssh/google_compute_engine' <user>@carla:/work/bundle/ ~/carla-bundle/
cd ~/carla-bundle && shasum -a 256 -c SHA256SUMS
```

`<user>` is the OS Login name that `gcloud compute ssh` uses (run `whoami` in that shell). Without Tailscale, the slower
route is `gcloud compute scp --recurse --tunnel-through-iap carla:/work/bundle ~/carla-bundle --zone=europe-west4-b --project=carla-513807`.
Google bills traffic leaving the region, roughly $0.1 per GB.

Then stop the VM: `gcloud compute instances stop carla --zone=europe-west4-b --project=carla-513807`. Keep it, stopped,
until the new machine passes step 4; deleting it is a separate decision. The machine images `carla-image` and
`carla-image-eu` are older copies of the same disk.

## 3. New machine: build and install

As root:

```sh
mkdir -p /work && cd /work
git clone https://github.com/DavidBogdan20/digital-twin-generated-map repo
ln -s repo/digital_twin dt && ln -s repo/car_simulator car_simulator  # relative links, so the container sees them too
mkdir bundle   # then, from the Mac: rsync -a --rsync-path='sudo rsync' ~/carla-bundle/ <user>@<new machine>:/work/bundle/
apt-get install -y zstd
mkdir old && tar -C old -xf bundle/vm-files.tar.*  # the old VM's files, for reference only (.gz if the old VM had no zstd)
```

| Step | Time on the old VM | What it does |
|---|---|---|
| `/work/dt/vm.sh setup` | minutes | NVIDIA driver (Google's installer on Google Cloud, `ubuntu-drivers` elsewhere). Reboot, then run it again for the display, Docker with GPU access and the `carla-build` image |
| `/work/dt/vm.sh start` | seconds | X, VNC, the container. After every boot. It must print an NVIDIA device from `vulkaninfo`, not llvmpipe |
| `/work/dt/vm.sh stock` | 10 min | Stock CARLA 0.9.16 in `/work/stock`. Test it: `vm.sh run`, then `vm.sh check Town10HD_Opt` |
| `echo <token> \| /work/dt/vm.sh fetch` | 10 min | Unreal's source at the head of CarlaUnreal's `carla` branch. The token is used once and not saved; the commit goes to `/work/logs/engine_commit` |
| `/work/dt/vm.sh engine` | 1 h 25 min | Patches and builds Unreal |
| `/work/dt/vm.sh carla` | 25 min | Clones CARLA 0.9.16, runs `apply_patches.py`, builds the Python API and the editor |
| `/work/dt/vm.sh unpack` | minutes | Checks the bundle's sums and puts its maps into the build |
| `/work/dt/vm.sh recook WolfsburgHighway` | 2 to 3 h | Cooks the map, installs it into stock 0.9.16 with the bundle's road file and navmesh, then lists any asset 0.9.16 doesn't have |
| `/work/dt/vm.sh speedsigns` | seconds | The speed sign fix, once per stock install. It prints "already sets" if 0.9.16 doesn't need it |

`recook` writes `/work/logs/package_WolfsburgHighway.log`. The list it prints counts "Failed to load" and similar
lines. A few are normal; a missing material shows as a grey or untextured surface. Fix those in the editor
(`vm.sh editor`) by pointing them at the 0.9.16 asset, save, and run `recook` again. The road file's md5 must match the
bundle's `MANIFEST.txt`; `recook` prints both.

Don't run `uturns` or `lights` again: the bundle's road file already has both.

## 4. Check it

- **Map:** `vm.sh run`, `vm.sh check WolfsburgHighway`, then in the container (`docker exec -it -u carla -e DISPLAY=:0 carla bash`)
  `python3 /work/dt/signals_check.py`, `python3 /work/dt/merge_test.py WolfsburgHighway` and `python3 /work/dt/surface_check.py`.
- **Scenarios:** in the container, `cd /work/car_simulator`, `CARLA_ROOT=/work/stock bash scripts/setup_env_linux.sh`,
  then the five variants in `COMMANDS.md`. `car_simulator/PORTING.md` lists what differs from upstream and what is still open.
- **Remote screen:** `vm.sh stream`, then pair Moonlight with `vm.sh pair <PIN>`. The old VM is still `carla` in the
  tailnet, so the new machine gets `carla-1` until you remove the old one in the Tailscale admin console.
- **Herbie:** `vm.sh publish` as before. The laptop's tunnel becomes plain SSH (`HERBIE_CONNECTION.md`).

## Fallback: stay on 0.9.15

If the re-cook can't be fixed, the new machine can run the old map with no build, after `setup` and `start`:

```sh
mv /work/stock /work/stock-0.9.16 2>/dev/null || true  # a 0.9.16 install would mix with 0.9.15's files
STOCK_URL=https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.15.tar.gz /work/dt/vm.sh stock
cp /work/bundle/WolfsburgHighway_*.tar.gz /work/stock/Import/
docker exec -u carla carla bash -c 'cd /work/stock && ./ImportAssets.sh && pip3 install --user carla==0.9.15'
tar -C /work/stock/CarlaUE4/Content/CustomMaps -xf /work/bundle/installed.tar.* WolfsburgHighway
/work/dt/vm.sh speedsigns
```

The 0.9.15 client goes into the container user's `~/.local` and hides the image's 0.9.16 one; to go back to 0.9.16,
`docker exec -u carla carla pip3 uninstall -y carla`. The scenarios then need the 0.9.15 `car_simulator` from the tag
`carla-0.9.15`.

## Not tested yet

- The whole 0.9.16 path: CARLA 0.9.16 cooking content saved by the 0.9.15 editor, and the Digital Twin patches on
  0.9.16 (`patches/carla-0.9.16.diff`). Both use Unreal 4.26, so the asset format is the same.
- Regenerating a map from OSM on 0.9.16 (README "Rebuild" step 4). The inputs are in the repo, but nobody has run it.
- The engine edits on the head of CarlaUnreal's `carla` branch. They were matched against `8d08ab7fe`, the 0.9.15
  engine; if the branch changed those lines, `vm.sh engine` stops before changing anything and names the file.
- The new machine's driver path (`ubuntu-drivers`). On Google Cloud, `setup` still uses Google's installer.
