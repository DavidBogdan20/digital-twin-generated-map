#!/bin/bash
# Runs on the Linux machine (Ubuntu 22.04/24.04 with an NVIDIA GPU) as root, from /work/dt. Usage: vm.sh <step>.
# GUIDE.md lists the steps in order; MIGRATION.md (repo root) moves an old 0.9.15 VM to a new 0.9.16 machine.
set -euo pipefail

WORK=/work
UE4=$WORK/UE4
CARLA=$WORK/carla
STOCK=$WORK/stock
BUNDLE=$WORK/bundle  # what `bundle` packs on the old VM and `unpack` reads on the new machine
ENGINE_REF=carla  # the head of CARLA's engine branch, as 0.9.16's build_linux.md clones it; fetch logs the commit
CARLA_TAG=0.9.16
STOCK_URL=${STOCK_URL:-https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.16.tar.gz}  # override for another release
SUNSHINE_URL=https://github.com/LizardByte/Sunshine/releases/download/v2026.914.233613/sunshine_2026.914.233613-1%2Bubuntu22.04_amd64.deb
SUNSHINE_CONF=/root/.config/sunshine/sunshine.conf

box() {  # runs in the Ubuntu 22.04 container, as a normal user because Unreal refuses to run as root
  docker exec -i -u carla -e DISPLAY=:0 -e UE4_ROOT=$UE4 -e HOME=/home/carla carla bash -ec "$1"
}

driver() {  # installs the NVIDIA driver; reboot afterwards
  if curl -sf -m 2 -H Metadata-Flavor:Google http://metadata.google.internal >/dev/null; then
    # Google Cloud: its installer picks the licensed virtual workstation driver and may restart the VM
    echo 'options nvidia NVreg_EnableGpuFirmware=0' > /etc/modprobe.d/nvidia-gsp.conf  # Google requires this on L4 workstations
    curl -fsSL -o /root/cuda_installer.pyz https://storage.googleapis.com/compute-gpu-installation-us/installer/latest/cuda_installer.pyz
    python3 /root/cuda_installer.pyz install_driver
  else
    apt-get update
    apt-get install -y ubuntu-drivers-common
    ubuntu-drivers install  # the recommended full driver (with Vulkan and X), not the compute-only one
  fi
  echo "Reboot if the installer didn't, then run setup again"
}

setup() {  # installs the driver, display, remote screen, Docker with GPU access, and the container image
  nvidia-smi || { driver; exit; }
  apt-get update
  apt-get install -y xserver-xorg x11-xserver-utils openbox x11vnc scrot xdotool vulkan-tools zstd
  command -v docker >/dev/null || apt-get install -y docker.io
  command -v nvidia-xconfig >/dev/null || apt-get install -y nvidia-xconfig

  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey |
    gpg --dearmor --yes -o /usr/share/keyrings/nvidia-container.gpg
  echo 'deb [signed-by=/usr/share/keyrings/nvidia-container.gpg] https://nvidia.github.io/libnvidia-container/stable/deb/amd64 /' \
    > /etc/apt/sources.list.d/nvidia-container.list
  apt-get update
  apt-get install -y nvidia-container-toolkit
  nvidia-ctk runtime configure --runtime=docker
  systemctl restart docker

  nvidia-xconfig --preserve-busid -a --virtual=1920x1080 --allow-empty-initial-configuration
  x11vnc -storepasswd carlavnc /root/.vnc_passwd  # only reachable through the SSH tunnel
  mkdir -p $WORK/logs
  chown -R 1000:1000 $WORK
  docker build -t carla-build - < "$(dirname "$0")/Dockerfile"
}

start() {  # starts everything; run it again after any reboot
  pgrep -x Xorg >/dev/null || { nohup X :0 -noreset >/var/log/x0.log 2>&1 & sleep 5; }
  export DISPLAY=:0
  xhost +local:
  xrandr -s 1920x1080  # the display starts at 1024x768, which keeps maximized windows small
  # without these links a systemd reload (any apt install) takes the GPU away from the running container
  nvidia-ctk system create-dev-char-symlinks --create-all >/dev/null
  # a reload still did (2026-11-10 15:45, snapd refreshing core22 mid-run: "Failed to initialize NVML" in the container),
  # so no automatic snap refresh or apt run while the VM is up; start runs after every boot. The VM has no public IP.
  snap refresh --hold >/dev/null 2>&1 || true
  systemctl stop apt-daily.timer apt-daily-upgrade.timer unattended-upgrades 2>/dev/null || true
  pgrep -x x11vnc >/dev/null ||
    x11vnc -display :0 -localhost -forever -shared -noxdamage -viewonly -rfbauth /root/.vnc_passwd -bg -o /var/log/x11vnc.log
  docker start carla 2>/dev/null ||
    docker run -d --name carla --gpus all --net=host --ipc=host -e NVIDIA_DRIVER_CAPABILITIES=all \
      -e VK_ICD_FILENAMES=/etc/vulkan/icd.d/nvidia_icd.json -e XDG_RUNTIME_DIR=/tmp \
      -v /tmp/.X11-unix:/tmp/.X11-unix -v $WORK:$WORK \
      -v /etc/vulkan/icd.d/nvidia_icd.json:/etc/vulkan/icd.d/nvidia_icd.json:ro \
      carla-build sleep infinity  # the GPU option doesn't bring the Vulkan file with this driver; ipc=host lets X apps
                                  # like pygame share memory with the X server (created 2026-11-09, earlier ones lack it)
  command -v sunshine >/dev/null && ! pgrep -x sunshine >/dev/null &&
    (cd /root; DISPLAY=:0 nohup sunshine $SUNSHINE_CONF >/var/log/sunshine.log 2>&1 &)
  box "nohup python3 -m http.server 8000 --bind 127.0.0.1 --directory $WORK/dt >/dev/null 2>&1 &"
  nvidia-smi -q | grep -A2 'vGPU Software Licensed Product' || true  # Google Cloud only: must say NVIDIA RTX Virtual Workstation, Licensed
  box 'vulkaninfo | grep -E "apiVersion|deviceName" | head -4'  # the device must be NVIDIA, not llvmpipe
}

stock() {  # downloads stock CARLA 0.9.16: this tests the GPU, Vulkan and the remote screen before any long build
  box "mkdir -p $STOCK; cd $WORK; aria2c -x8 -s8 -o stock.tar.gz $STOCK_URL; tar -xzf stock.tar.gz -C $STOCK; rm stock.tar.gz"
}

run() {  # opens the simulator on the remote screen
  box "cd $STOCK; nohup ./CarlaUE4.sh -vulkan -windowed -ResX=1920 -ResY=1080 -nosound >$WORK/logs/run.log 2>&1 &"
}

check() {  # loads a generated map in the running simulator. Usage: vm.sh check Tiny
  box "python3 $WORK/dt/map_test.py $1"  # the container has the carla client (pip, Dockerfile)
}

shot() {  # saves the remote screen to /tmp/screen.png
  DISPLAY=:0 scrot -o /tmp/screen.png
}

fetch() {  # fetches Unreal; reads a GitHub token from stdin, uses it once and doesn't save it
  box "
    read -r GH_TOKEN; export GH_TOKEN
    printf '#!/bin/sh\necho \"\$GH_TOKEN\"\n' > ~/.askpass; chmod 700 ~/.askpass; trap 'rm -f ~/.askpass' EXIT
    git init -q $UE4; cd $UE4
    git config core.longpaths true
    git remote add origin https://x-access-token@github.com/CarlaUnreal/UnrealEngine.git
    GIT_ASKPASS=~/.askpass git fetch --depth 1 origin $ENGINE_REF
    git checkout -q FETCH_HEAD
    git remote remove origin
    git rev-parse HEAD | tee $WORK/logs/engine_commit  # the commit this build used; to pin it, set ENGINE_REF to it
  "
}

engine() {  # patches and builds Unreal
  box "
    python3 $WORK/dt/apply_patches.py $UE4 $CARLA
    cd $UE4
    ./Setup.sh --exclude=/Engine/Source/ThirdParty/Licenses
    ./GenerateProjectFiles.sh
    make
  "
}

carla() {  # clones, patches and builds CARLA 0.9.16
  box "
    git clone -b $CARLA_TAG https://github.com/carla-simulator/carla $CARLA
    python3 $WORK/dt/apply_patches.py $UE4 $CARLA
    cd $CARLA
    ./Update.sh
    make PythonAPI
    make CarlaUE4Editor
  "
}

editor() {  # opens the Unreal editor full size with the tool already open
  rm -f $WORK/dt/cmd.py  # one left by a crashed editor would run on the first frames, before the tool opens
  pgrep -x openbox >/dev/null || { DISPLAY=:0 nohup openbox >/dev/null 2>&1 & }
  local scripts=$CARLA/Unreal/CarlaUE4/Content/Python  # the editor runs init_unreal.py at startup; -ExecutePythonScript crashes it
  box "mkdir -p $scripts; cp $WORK/dt/open_tool.py $scripts/init_unreal.py; cd $CARLA; nohup make launch-only >$WORK/logs/editor.log 2>&1 &"
  sleep 30  # the log from the last run stays until the new one starts
  timeout 900 bash -c "until grep -q 'open_tool: tool tab' $CARLA/Unreal/CarlaUE4/Saved/Logs/CarlaUE4.log; do sleep 5; done"
  pkill -x openbox  # with it running, Unreal's menus don't open
  sleep 3
  DISPLAY=:0 xdotool search --name 'CarlaUE4 - Unreal Editor' windowmove 0 0 windowsize 1920 1080
}

generate() {  # fills in the open tool and presses Generate. Usage: vm.sh generate Tiny 'http://...?bbox=...'
  # the tool downloads the .osm from this web server; `drive` stops it (Traffic Manager wants port 8000), and without
  # it Generate fails at once with "Failed to connect to localhost port 8000" and makes empty tiles (2026-11-10)
  curl -sf -o /dev/null http://localhost:8000/ ||
    box "nohup python3 -m http.server 8000 --bind 127.0.0.1 --directory $WORK/dt >/dev/null 2>&1 &"
  sleep 1
  sed -e "s|@NAME@|$1|g" -e "s|@URL@|$2|g" $WORK/dt/generate.py > $WORK/dt/cmd.py
}

wolfsburg() {  # puts our prepared roads where the tool looks for them. Usage: vm.sh wolfsburg Wolfsburg2
  local dir=$CARLA/Unreal/CarlaUE4/Content/CustomMaps/$1/OpenDrive
  box "mkdir -p $dir; cp $WORK/dt/Wolfsburg.xodr $dir/$1.xodr"
}

facades() {  # replaces the grey generated walls with CARLA's facades in the open editor. Usage: vm.sh facades Wolfsburg2
  local log=$CARLA/Unreal/CarlaUE4/Saved/Logs/CarlaUE4.log run=$(date +%s)
  sed -e "s|@NAME@|$1|g" -e "s|@RUN@|$run|g" $WORK/dt/facades.py > $WORK/dt/cmd.py
  timeout 1800 bash -c "until grep -q 'facades: finished $run' $log; do sleep 10; done"
  grep "facades: finished $run" $log
}

prepared() {  # puts a map's prepared inputs (prepare_dt.py) where the tool looks for them. Usage: vm.sh prepared WolfsburgA39
  local dir=$CARLA/Unreal/CarlaUE4/Content/CustomMaps/$1/OpenDrive
  box "mkdir -p $dir; cp $WORK/dt/$1.xodr $WORK/dt/$1.keepout $dir/"
}

audit() {  # lists every mesh of every tile from the open editor, then checks them. Usage: vm.sh audit WolfsburgA39 [x y]
  local name=$1 log=$CARLA/Unreal/CarlaUE4/Saved/Logs/CarlaUE4.log run=$(date +%s); shift
  local tiles=_Tile_  # with x y, only the tile holding that point (tiles are 500 m, numbered east and north)
  [ $# -eq 2 ] && tiles=$(python3 -c "import math; print('_Tile_%d_%d' % (math.floor($1 / 500), math.floor(-$2 / 500)))")
  sed -e "s|@NAME@|$name|g" -e "s|@TILES@|$tiles|g" -e "s|@RUN@|$run|g" $WORK/dt/level_audit.py > $WORK/dt/cmd.py
  timeout 1800 bash -c "until grep -q 'audit: finished $run' $log; do sleep 10; done"  # the editor writes its log late
  box "python3 $WORK/dt/check_level.py $WORK/logs/${name}_meshes.jsonl \
    $CARLA/Unreal/CarlaUE4/Content/CustomMaps/$name/OpenDrive/$name.xodr $*"
}

prune() {  # removes the trees and props that the last `audit` rejected, from the open editor, and saves the tiles
  local name=$1 log=$CARLA/Unreal/CarlaUE4/Saved/Logs/CarlaUE4.log run=$(date +%s)
  box "python3 $WORK/dt/check_level.py $WORK/logs/${name}_meshes.jsonl \
    $CARLA/Unreal/CarlaUE4/Content/CustomMaps/$name/OpenDrive/$name.xodr --write-bad $WORK/logs/${name}_bad.json | tail -1" || true
  sed -e "s|@NAME@|$name|g" -e "s|@RUN@|$run|g" $WORK/dt/level_prune.py > $WORK/dt/cmd.py
  timeout 1800 bash -c "until grep -q 'prune: finished $run' $log; do sleep 10; done"
  grep "prune: finished $run" $log
}

rails() {  # places guard rails from <map>_rails.json (rails_plan.py) in the open editor and saves the tiles
  local name=$1 log=$CARLA/Unreal/CarlaUE4/Saved/Logs/CarlaUE4.log run=$(date +%s)
  sed -e "s|@NAME@|$name|g" -e "s|@RUN@|$run|g" $WORK/dt/rails.py > $WORK/dt/cmd.py
  timeout 1800 bash -c "until grep -q 'rails: finished $run' $log; do sleep 10; done"
  grep "rails: finished $run" $log
}

meshscan() {  # lists road meshes that crash the cook (mesh_scan.py) from the open editor; fix=1 also mends them. Usage: vm.sh meshscan WolfsburgHighway [1]
  local name=$1 fix=${2:-0} log=$CARLA/Unreal/CarlaUE4/Saved/Logs/CarlaUE4.log run=$(date +%s)
  sed -e "s|@NAME@|$name|g" -e "s|@RUN@|$run|g" -e "s|@FIX@|$fix|g" $WORK/dt/mesh_scan.py > $WORK/dt/cmd.py
  timeout 3600 bash -c "until grep -q 'meshscan: finished $run' $log; do sleep 10; done"
  grep "meshscan: finished $run" $log
  cat $WORK/logs/${name}_badmeshes.jsonl
}

tileoffset() {  # makes the large map manager find the right tile (tile_offset.py) in the open editor. Usage: vm.sh tileoffset WolfsburgHighway
  local name=$1 log=$CARLA/Unreal/CarlaUE4/Saved/Logs/CarlaUE4.log run=$(date +%s)
  sed -e "s|@NAME@|$name|g" -e "s|@RUN@|$run|g" $WORK/dt/tile_offset.py > $WORK/dt/cmd.py
  timeout 600 bash -c "until grep -q 'tileoffset: finished $run' $log; do sleep 5; done"
  grep "tileoffset:" $log | tail -2
}

package() {  # cooks a generated map and installs it into stock CARLA. Usage: vm.sh package Tiny
  local name=$1
  box "
    sed s/Wolfsburg/$name/g $WORK/dt/Wolfsburg.Package.json > $CARLA/Unreal/CarlaUE4/Content/Carla/Config/$name.Package.json
    cd $CARLA
    make package ARGS=--packages=$name
    cp Dist/${name}_*.tar.gz $STOCK/Import/
    cd $STOCK
    ./ImportAssets.sh
  "
}

uturns() {  # removes U-turns from an installed map's road file, which needs no repackaging. Usage: vm.sh uturns Wolfsburg2
  local xodr=$STOCK/CarlaUE4/Content/CustomMaps/$1/OpenDrive/$1.xodr
  python3 $WORK/dt/remove_uturns.py $xodr $xodr
}

lights() {  # moves traffic light stop lines off stub roads in an installed map's road file (move_lights.py), after uturns
  local xodr=$STOCK/CarlaUE4/Content/CustomMaps/$1/OpenDrive/$1.xodr
  python3 $WORK/dt/move_lights.py $xodr $xodr
}

speedsigns() {  # makes stock CARLA's 100/110/120 km/h signs set their own limit, not 90 (fix_speed_signs.py); once per install
  python3 $WORK/dt/fix_speed_signs.py $STOCK
}

drive() {  # opens CARLA's manual_control.py (WASD to drive, P for autopilot) on the remote screen, after check has loaded a map
  box "
    pkill -f '^python3 -m http.server 8000' || true  # only the Digital Twin Tool needs it, and autopilot needs port 8000
    pkill -f '^python3 -u /work/dt/drive.py' || true  # anchored: the text 'drive.py' is in this script too
    while pgrep -f '^python3 -u /work/dt/drive.py' >/dev/null; do sleep 1; done  # starting before it has left loses the new one
    python3 -c 'import pygame' 2>/dev/null || python3 -m pip install --user -q numpy==1.24.4 pygame==2.6.1
    python3 -c \"import carla; c = carla.Client('localhost', 2000); c.set_timeout(30); [a.destroy() for a in c.get_world().get_actors().filter('vehicle.*')]\"
    # spawn next to the camera: a random far spot makes the map stream a new area at once, which crashed it before
    # and don't start Traffic Manager at startup: building its road map for this map took over 7 minutes, so it waits for P
    sed -e 's/random.choice(spawn_points)/min(spawn_points, key=lambda p: p.location.distance(self.world.get_spectator().get_location()))/' \
      -e 's/^            world.player.set_autopilot(self._autopilot_enabled)$/            None/' \
      $STOCK/PythonAPI/examples/manual_control.py > $WORK/dt/drive.py
    DISPLAY=:0 nohup python3 -u $WORK/dt/drive.py --res 1920x1080 > $WORK/logs/drive.log 2>&1 &
  "
}

scenario() {  # the A 39 elevation scenario on Wolfsburg2, full screen, after check has loaded the map. Writes logs/elevation.csv
  box "
    pkill -f '^python3 -u /work/dt/(drive|elevation_scenario).py' || true  # one client at a time: the scenario runs synchronous
    while pgrep -f '^python3 -u /work/dt/(drive|elevation_scenario).py' >/dev/null; do sleep 1; done
    DISPLAY=:0 nohup python3 -u $WORK/dt/elevation_scenario.py > $WORK/logs/scenario.log 2>&1 &
  "
}

publish() {  # publishes the hero car's state as JSON lines on 127.0.0.1:2100 for a client through an SSH tunnel
  # (publish.py, HERBIE_DATA_PLAN.md). Usage: vm.sh publish [rate Hz, default 10]; runs until stopped, whatever map or scenario
  box "
    pkill -f '^python3 -u /work/dt/publish.py' || true
    nohup python3 -u $WORK/dt/publish.py ${1:-10} > $WORK/logs/publish.log 2>&1 &
  "
}

chart() {  # draws logs/elevation.png from the last scenario run
  box "
    python3 -c 'import matplotlib' 2>/dev/null || python3 -m pip install --user -q matplotlib==3.7.5
    python3 $WORK/dt/elevation_chart.py $WORK/logs/elevation.csv $WORK/logs/elevation.png
  "
}

stream() {  # one-time: Tailscale and Sunshine, for Moonlight on the Mac. Needs outbound internet (on Google Cloud: Cloud NAT)
  command -v tailscale >/dev/null || curl -fsSL https://tailscale.com/install.sh | sh
  . /etc/os-release  # the Sunshine package must match the host's Ubuntu (22.04 or 24.04)
  command -v sunshine >/dev/null || { curl -fsSL -o /tmp/sunshine.deb ${SUNSHINE_URL/22.04/$VERSION_ID}; apt-get install -y /tmp/sunshine.deb; }
  # installing packages reloads systemd; without these links that takes the GPU away from the running container
  nvidia-ctk system create-dev-char-symlinks --create-all >/dev/null
  mkdir -p $(dirname $SUNSHINE_CONF)
  printf 'sunshine_name = carla\nencoder = nvenc\ncapture = x11\nsystem_tray = disabled\n' > $SUNSHINE_CONF  # no tray: it aborts without one
  [ -f /root/sunshine_creds ] || { (umask 077; echo "admin $(openssl rand -hex 12)" > /root/sunshine_creds)
    sunshine $SUNSHINE_CONF --creds $(cat /root/sunshine_creds) >/dev/null; }  # web login, kept on the VM only
  start
  tailscale status >/dev/null 2>&1 || { nohup tailscale up --hostname=carla --accept-dns=false >/root/ts.log 2>&1 &
    sleep 8; grep -o 'https://login.tailscale.com/[^ ]*' /root/ts.log; }  # open this link to add the VM to your Tailscale
}

pair() {  # enters the PIN Moonlight shows, for the one request waiting. Usage: vm.sh pair 1234
  read -r user pass < /root/sunshine_creds
  local id=$(curl -fsSk -u "$user:$pass" https://localhost:47990/api/pin |
    python3 -c 'import json, sys; p = json.load(sys.stdin)["pairings"]; assert len(p) == 1, p; print(p[0]["id"])')
  curl -fsSk -u "$user:$pass" -X POST https://localhost:47990/api/pin -H 'Content-Type: application/json' \
    -d "{\"pairing_id\":\"$id\",\"pin\":\"$1\",\"name\":\"mac\"}"
}

bundle() {  # on the OLD 0.9.15 VM: packs what a new machine can't rebuild into /work/bundle (MIGRATION.md). Usage: vm.sh bundle [maps]
  local maps=${*:-$(ls $CARLA/Unreal/CarlaUE4/Content/CustomMaps)} z=zst
  command -v zstd >/dev/null || z=gz  # tar -a picks the compression from the name; tar -x detects it
  mkdir -p $BUNDLE; cd $BUNDLE
  # the generated maps' editor content: re-cooking it in 0.9.16 needs only this
  tar -C $CARLA/Unreal/CarlaUE4/Content/CustomMaps -acf maps.tar.$z $maps
  # their road files and walker navmeshes as installed, after uturns and lights: the ones the scenarios ran on
  local inst=$(cd $STOCK/CarlaUE4/Content/CustomMaps; for m in $maps; do ls -d $m/OpenDrive $m/Nav 2>/dev/null || true; done)
  [ -z "$inst" ] || tar -C $STOCK/CarlaUE4/Content/CustomMaps -acf installed.tar.$z $inst
  # the newest 0.9.15 cook of each map: stock 0.9.15 can still import it if the 0.9.16 re-cook stalls
  for m in $maps; do ls -t $CARLA/Dist/${m}_*.tar.gz 2>/dev/null | head -1 || true; done | xargs -r cp -t $BUNDLE
  # the scripts and inputs as they were on the VM, scenario outputs and logs (GUIDE.md's history refers to them)
  tar -C $WORK -acf vm-files.tar.$z --exclude=dt/cmd.py --exclude=__pycache__ dt logs \
    $(cd $WORK; ls -d car_simulator/outputs 2>/dev/null || true)
  { date -u; echo "maps:" $maps; du -sh $CARLA/Unreal/CarlaUE4/Content/CustomMaps/*
    md5sum $STOCK/CarlaUE4/Content/CustomMaps/*/OpenDrive/*.xodr $STOCK/CarlaUE4/Content/CustomMaps/*/Nav/*.bin 2>/dev/null || true; } > MANIFEST.txt
  sha256sum *.tar.* MANIFEST.txt > SHA256SUMS
  chown -R 1000:1000 $BUNDLE
  ls -la; du -sh .
}

unpack() {  # on the NEW machine, after `carla`: checks /work/bundle and puts its maps into the 0.9.16 build
  (cd $BUNDLE; sha256sum -c --quiet SHA256SUMS)
  box "mkdir -p $CARLA/Unreal/CarlaUE4/Content/CustomMaps; tar -C $CARLA/Unreal/CarlaUE4/Content/CustomMaps -xf $(ls -t $BUNDLE/maps.tar.* | head -1)"
  ls $CARLA/Unreal/CarlaUE4/Content/CustomMaps
}

recook() {  # cooks a bundled map in the 0.9.16 build, installs it into stock 0.9.16 with its old road file and navmesh, and
            # lists asset references 0.9.16 lacks. Usage: vm.sh recook WolfsburgHighway (after unpack, with the editor closed)
  local name=$1 log=$WORK/logs/package_$1.log
  package $name > $log 2>&1 || { tail -30 $log; return 1; }
  box "tar -C $STOCK/CarlaUE4/Content/CustomMaps -xf $(ls -t $BUNDLE/installed.tar.* | head -1) $name"
  grep -E "Failed to load|Failed import|Can't find file|Unable to find package|Error:" $log | sort | uniq -c | sort -rn | head -40 || true
  md5sum $STOCK/CarlaUE4/Content/CustomMaps/$name/OpenDrive/$name.xodr  # must match the bundle's MANIFEST.txt:
  grep "/$name/OpenDrive/" $BUNDLE/MANIFEST.txt || echo "not in the bundle"
}

"$@"
echo "DONE $*"
