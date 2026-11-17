# Plan: reading CARLA data in the Herbie bridge

This is the code side of `HERBIE_CONNECTION.md`. The VM publishes the ego car's state, and the bridge reads it through the tunnel. It's read-only.

## What runs on the VM

**Server**
- Stock CARLA **0.9.15**.
- The scenarios use the map **WolfsburgHighway**. Its geo origin is 52.38568 N, 10.731333 E, so positions can be turned into latitude and longitude.

**A scenario run** (`run_scenario.py` with ScenarioRunner 0.9.15)
- **Timing:** synchronous, in fixed 0.05 s steps, run as fast as the VM can, which is **not real time**. `--async` runs in real time, but the scenario results are then not valid.
- **Ego car:** `vehicle.lincoln.mkz_2017` with `role_name = "hero"`.
  - Driven by the scenario's own driver, targeting 30 km/h. It sometimes holds 2nd gear.
  - Its lights stay **off**, because every WolfsburgHighway variant is daytime.
- **Other actors:** the scenario's truck, walkers, cyclist and van; background traffic (8 cars, 2 motorcycles, 2 bicycles, 15 walkers); about 200 traffic signs and 9 lights; and a chase-view camera.
- **Restart and between runs:** a restart spawns a new ego with a new id. Between runs there is no `hero`.

## Why a publisher, not a direct CARLA client

Every step, CARLA pushes a full snapshot of all actors to **every** connected client. There's no way to ask for less. Values beyond that snapshot (lights, wheel angles, weather) each need a request.

Measured on 2026-11-13 with a test `hero` on WolfsburgHighway and the Mac in place of the laptop:

| | CARLA client through the tunnel | `publish.py` on the VM |
|---|---|---|
| traffic from the VM | 7.6 MB/s | 4 KB/s |
| updates | about 110 a second, roughly half the frames dropped | 10 a second, none lost |
| one request | about 370 ms | none |
| time to connect | 12 s (downloads the 4 MB road file) | immediate |
| laptop needs | `carla==0.9.15` and Python 3.7 to 3.10 | nothing |

Capping CARLA's frame rate (`-ExecCmds="t.MaxFPS 20"`) had no effect on the packaged CARLA. It wouldn't cover scenario runs anyway, because their fixed steps run as fast as ScenarioRunner ticks.

## The publisher (VM, done and tested)

- **Files:** `digital_twin/publish.py`, started with `vm.sh publish [rate, default 10]`.
- **What it reads:** it uses `CarlaReader` from `digital_twin/tunnel_check.py`.
  - It subscribes to CARLA's snapshots and keeps only the latest one.
  - It follows the single `hero` and finds the new one after a restart.
  - It connects again after a map load or a CARLA restart.
  - Its only request is the light state, once a second.
  - It never ticks, spawns or changes anything.
- **Where it listens:** `127.0.0.1:2100` only, for any number of clients. A stalled client is dropped after 2 s.
- **Each line** is one JSON object, about 10 times a second:
  ```json
  {"frame": 35367, "sim_time": 183.46, "speed": 1.61, "gear": "D", "steer_deg": 0.0, "throttle": 0.4,
   "brake": 0.0, "lights": 0, "lat": 52.387549, "lon": 10.747195, "alt": 0.0, "heading": 112.0,
   "speed_limit": 30.0, "wall_time": 1794562290.12}
  ```
- **Fields:**
  - `speed`: m/s, negative in reverse.
  - `gear`: `R` or `D`.
  - `steer_deg`: front wheel angle, from steer × the maximum steer angle.
  - `lights`: CARLA's light bits. Position 1, LowBeam 2, HighBeam 4, Brake 8, RightBlinker 16, LeftBlinker 32, Reverse 64, Fog 128.
  - `heading`: 0 is north, clockwise.
  - `sim_time`: seconds of simulation; `wall_time` is the VM's clock.
- **Status lines:** when there's no car, the line is `{"status": "no ego" | "no ticks" | "map changed" | "no CARLA", "wall_time": ...}`.

## Bridge side (laptop, to do)

Replace the CARLA calls in `carla_adapter.py` with a line reader. It needs no `carla` package and no proxy:

```python
import json, socket, time

def ego_states(host="localhost", port=2100):
    """Every message from the VM's publisher: a state dict or {"status": ...}. Reconnects forever."""
    while True:
        try:
            with socket.create_connection((host, port), timeout=5) as s:  # 5 s of silence = link down
                for line in s.makefile():
                    yield json.loads(line)
        except OSError:
            yield {"status": "no link"}
            time.sleep(2)
```

**Map the fields to VHAL:**
- `speed` to `PERF_VEHICLE_SPEED` (m/s, signed);
- `gear` to `GEAR_SELECTION` (R / D);
- `lights` bits to the light properties;
- `lat`, `lon`, `heading` to the Position app.

Show a status line as "no car" in the panel. CARLA can't report doors.

## Steps

1. **VM:** done. `publish.py` and `vm.sh publish` are installed and tested through IAP from the Mac:
   - signed speed and the R/D gear;
   - position inside the map's area;
   - "no ego" when the car is gone;
   - a single connection at startup.
2. **Bridge:** add `ego_states()` as the CARLA source, then map the fields.
   - Offline test with a fake server: a local socket that replays recorded lines, including status lines and a dropped connection.
3. **Live, during a WolfsburgHighway scenario:**
   - speed against the chase-view HUD;
   - latitude and longitude on the right street in OSM;
   - `sim_time` against `wall_time`.
4. **Measure:** the bridge loop rate, the Android update rate, and reconnects.
