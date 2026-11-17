# VM commands

`<...>` = fill in. `[...]` = optional.

## Mac terminal

- `gcloud compute instances list --project=carla-513807` -> is the VM running?
- `gcloud compute instances start carla --zone=europe-west4-b --project=carla-513807` -> start the VM (billing starts; on STOCKOUT retry in a few minutes)
- `gcloud compute instances stop carla --zone=europe-west4-b --project=carla-513807` -> stop the VM (billing stops)
- `gcloud compute ssh carla --zone=europe-west4-b --project=carla-513807 --tunnel-through-iap` -> open a shell on the VM
- `gcloud compute scp <file> carla:/tmp/ --zone=europe-west4-b --project=carla-513807 --tunnel-through-iap` -> copy a file to the VM
- `gcloud compute ssh carla --zone=europe-west4-b --project=carla-513807 --tunnel-through-iap -- -N -L 5901:localhost:5900` -> VNC tunnel (keep open)
- `open -n /Applications/TigerVNC.app --args -ViewOnly=1 localhost::5901` -> view the screen over VNC (password `carlavnc`)
- `gcloud compute ssh carla --zone=europe-west4-b --project=carla-513807 --tunnel-through-iap -- -N -L 2100:localhost:2100` -> tunnel for the Herbie bridge (keep open; needs `vm.sh publish`)
- `gcloud compute instances create carla --zone=<zone> --project=carla-513807 --source-machine-image=carla-image-eu` -> new VM from the saved image (copy of 2026-11-13, stored in europe-west4)

## VM shell

- `sudo -i` -> become root (everything below assumes it)
- `/work/dt/vm.sh start` -> start the display, Sunshine and container (after every boot)
- `/work/dt/vm.sh run` -> start CARLA
- `/work/dt/vm.sh check WolfsburgHighway` -> load a map (or `Wolfsburg2`)
- `/work/dt/vm.sh drive` -> drive by hand (WASD, P = autopilot, ESC = quit)
- `/work/dt/vm.sh scenario` -> A 39 elevation scenario (needs Wolfsburg2 loaded)
- `/work/dt/vm.sh chart` -> chart of that run, `/work/logs/elevation.png`
- `/work/dt/vm.sh shot` -> screenshot, `/tmp/screen.png`
- `/work/dt/vm.sh publish` -> send the hero car's state to port 2100 for the tunnel (JSON, 10/s, until stopped)
- `docker exec carla pkill -f publish.py` -> stop publishing
- `tail -f /work/logs/publish.log` -> the publisher's log
- `docker exec carla pkill -f CarlaUE4-Linux-Shipping` -> stop CARLA
- `docker restart carla && /work/dt/vm.sh start` -> fix "Out of Local Memory" / NVML errors
- `nvidia-smi` -> GPU status
- `tail -f /work/logs/run.log` -> CARLA's log
- `/work/dt/vm.sh stream` -> install Tailscale and Sunshine (one time)
- `/work/dt/vm.sh pair <PIN>` -> pair Moonlight
- `tailscale ip -4` -> the VM's address for Moonlight

## Container shell (tests)

- `docker exec -it -u carla -e DISPLAY=:0 carla bash` -> shell in the container (windows show in Moonlight)
- `export PYTHONPATH=$(ls /work/carla/PythonAPI/carla/dist/*py3.8*.egg)` -> lets `/work/dt` scripts import carla
- `python3 /work/dt/signals_check.py [minutes] [map] [lights,stops,speed]` -> autopilot vs traffic lights and signs
- `python3 /work/dt/merge_test.py [map]` -> drive the ramps and bridges
- `python3 /work/dt/surface_check.py [step m]` -> check the road surface for bumps
- `python3 /work/dt/map_test.py <map>` -> load a map and put a car on it
- `python3 /work/dt/tunnel_check.py [host] [seconds]` -> read the hero car straight from CARLA, with rates and round trips

## Container shell (scenarios)

- `cd /work/car_simulator` -> scenario repo (run the rest from here)
- `CARLA_ROOT=/work/stock bash scripts/setup_env_linux.sh` -> one-time setup
- `python3 apps/list_maps.py --load WolfsburgHighway` -> load the map
- `python3 apps/list_scenarios.py --ours` -> list the scenarios
- `python3 apps/run_scenario.py pedestrian_crossing --variant PedestrianCrossing_WolfsburgHighway_1 --timeout 600` -> pedestrians cross at a junction
- `python3 apps/run_scenario.py occluded_crossing --variant OccludedCrossing_Child_WolfsburgHighway_1 --timeout 600` -> child crosses from behind a truck
- `python3 apps/run_scenario.py occluded_crossing --variant OccludedCrossing_Crowd_WolfsburgHighway_1 --timeout 600` -> crowd crosses from behind a truck
- `python3 apps/run_scenario.py cut_in --variant CutIn_Parking_WolfsburgHighway_1 --timeout 600` -> parked car cuts in
- `python3 apps/run_scenario.py cut_in --variant CutIn_Cyclist_WolfsburgHighway_1 --timeout 600` -> cyclist cuts in
- add `--view-spectator` -> CARLA's window follows the car (no chase camera)
- add `--no-view` -> no window
- add `--drive manual` -> you drive
- `touch /tmp/carsim_control_*/quit.request` -> stop a run (same as Quit)
- `pkill -INT -f scenario_runner.py` -> stop a `--no-view` run
- `python3 apps/reset_world.py` -> clean up after a run

## Rebuilding the map (rare, in this order; GUIDE.md has details)

- `/work/dt/vm.sh setup` -> driver, display, Docker (run again after its reboot)
- `/work/dt/vm.sh stock` -> download stock CARLA 0.9.15
- `/work/dt/vm.sh fetch` -> download Unreal (GitHub token on stdin)
- `/work/dt/vm.sh engine` -> build Unreal
- `/work/dt/vm.sh carla` -> build CARLA 0.9.15.2
- `/work/dt/vm.sh prepared <map>` -> put the map's road inputs in place
- `/work/dt/vm.sh editor` -> open the Unreal editor
- `/work/dt/vm.sh generate <map> '<osm url>'` -> generate the map
- `/work/dt/vm.sh audit <map>` -> check trees and props against the roads
- `/work/dt/vm.sh prune <map>` -> remove what the audit rejected
- `/work/dt/vm.sh facades <map>` -> put facades on buildings
- `/work/dt/vm.sh rails <map>` -> place guard rails
- `/work/dt/vm.sh meshscan <map> 1` -> fix road meshes that crash the cook
- `/work/dt/vm.sh tileoffset <map>` -> fix tile loading at the map edges
- `/work/dt/vm.sh package <map>` -> cook the map into stock CARLA
- `/work/dt/vm.sh uturns <map>` -> remove U-turns
- `/work/dt/vm.sh lights <map>` -> move traffic light stop lines off stub roads
- `/work/dt/vm.sh speedsigns` -> fix 100/110/120 km/h signs (once per stock install)
- `/work/dt/vm.sh wolfsburg <map>` -> old Wolfsburg2 road input (instead of `prepared`)
