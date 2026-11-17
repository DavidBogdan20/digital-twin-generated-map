# Plan: Herbie laptop -> CARLA on the VM

The goal is for the Herbie Data Bridge on the Windows laptop to get the ego car's live state from CARLA on the GCP VM. It only reads; nothing in the scenario changes. Only the bridge talks to the VM. The data simulator, the VHAL adapter and the Android apps stay local and need no changes.

## How it works

Tested end to end on 2026-11-13, with the Mac in place of the laptop.

- **On the VM:** `publish.py` reads CARLA locally. It sends the ego car's state as one JSON line about 10 times a second, on `127.0.0.1:2100`.
- **On the laptop:** a single IAP SSH tunnel forwards port 2100, and the bridge reads the lines.
- **Measured:** 10 messages a second, and about 4 KB/s of traffic from the VM.

Reading CARLA directly through the tunnel also works, but it is not used:
- it sent 7.6 MB/s, because CARLA sends every client every frame (about 200 a second when idle);
- every request took about 370 ms;
- it needs the `carla` client of the server's exact version: `carla==0.9.16`, for Python 3.10 to 3.12.

`HERBIE_DATA_PLAN.md` has the measurements.

## What each side needs for the handshake

**VM:** all of this is ready; you only start it.
- CARLA 0.9.16 running with a map loaded: `vm.sh run`, then `vm.sh check WolfsburgHighway`.
- The publisher running: `vm.sh publish`. It listens on localhost only, waits for CARLA, and keeps running across map loads and scenario restarts.
- Network access, already in place:
  - the firewall lets IAP reach port 22 (`default-allow-ssh`);
  - sshd allows port forwarding;
  - the VM has no public IP;
  - OS Login is on.

**Laptop:**
- The gcloud CLI, signed in with the project owner account. No IAM change is needed.
- gcloud's proxy set to 127.0.0.1:9000. IAP only needs outbound HTTPS 443.
- The tunnel, kept open. Its first run registers the laptop's SSH key with OS Login.
  ```
  gcloud compute ssh carla --zone europe-west4-b --project carla-513807 --tunnel-through-iap -- -N -L 2100:localhost:2100
  ```
  On a machine outside Google Cloud (`MIGRATION.md`), the same tunnel is plain SSH, for example over Tailscale:
  `ssh -N -L 2100:localhost:2100 <user>@<machine>`. The publisher and the bridge don't change.
- The bridge reads `localhost:2100`. That's a plain TCP socket, so it needs no `carla` package, works with any Python version and doesn't go through the proxy.

**What happens when the bridge connects:**
1. It opens TCP `localhost:2100`, which goes through the tunnel to the publisher.
2. A JSON line arrives every 0.1 s.
3. If there's no ego, the line is `{"status": ...}` instead of the car's state.

## Steps

1. **Laptop:** install gcloud (`winget install Google.CloudSDK`), set the proxy (`gcloud config set proxy/type http`, `proxy/address 127.0.0.1`, `proxy/port 9000`), then sign in with `gcloud auth login`.
2. **VM** (you, following COMMANDS.md): start the VM, then run `vm.sh start`, `vm.sh run`, `vm.sh check WolfsburgHighway` and `vm.sh publish`, then start a scenario.
3. **Laptop:** open the tunnel and test it:
   ```
   python -c "import socket; print(socket.create_connection(('localhost', 2100)).makefile().readline())"
   ```
4. **Bridge:** read and map the messages (`HERBIE_DATA_PLAN.md`).
5. **Full stack:** the Herbie stack, then speed and gear over adb, then the measurements.

## Risks

- **Not real time.** Synchronous scenarios run their 0.05 s steps as fast as the VM can. Use `sim_time` for timing.
- **Two heroes.** A scenario and `vm.sh drive` running at the same time give "no ego".
- **Panel controls.** The panel's light, door and weather controls don't reach CARLA on this path. It's read-only by design.
- **Dropped tunnel.** The IAP connection reset once during the test, so the bridge must reconnect by itself.
