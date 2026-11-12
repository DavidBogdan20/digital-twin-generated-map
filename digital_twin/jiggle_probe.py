"""Records the hero car and its chase camera frame by frame, without changing anything, to see where a jiggle comes
from: the car's own motion (physics, road surface) or the camera following it.
usage: jiggle_probe.py seconds [out.csv]   (runs beside manual_control; only reads world snapshots)"""
import csv
import sys
import time

import carla
import numpy as np


def local(m, loc):
    """loc in the frame whose inverse matrix is m"""
    return (np.array(m) @ np.array([loc.x, loc.y, loc.z, 1.0]))[:3]


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def jitter(name, v, unit):
    v = np.asarray(v, float)
    smooth = np.convolve(v, np.ones(5) / 5, mode="same")[2:-2]
    fast = v[2:-2] - smooth  # what changes within a few frames
    print(f"  {name:<22} range {np.ptp(v):9.4f} {unit}   frame-to-frame jitter: std {fast.std():8.4f}  peak {np.abs(fast).max():8.4f} {unit}")


secs = float(sys.argv[1])
client = carla.Client("localhost", 2000)
client.set_timeout(20)
world = client.get_world()
s = world.get_settings()
print(f"map {world.get_map().name}  sync {s.synchronous_mode}  fixed_dt {s.fixed_delta_seconds}  substepping {s.substepping}"
      f"  max_substep_dt {s.max_substep_delta_time}  max_substeps {s.max_substeps}")
actors = world.get_actors()
cars = [a for a in actors.filter("vehicle.*") if a.attributes.get("role_name") == "hero"] or list(actors.filter("vehicle.*"))
car = cars[0]
cams = [a for a in actors.filter("sensor.camera.*") if a.parent and a.parent.id == car.id]
pc = car.get_physics_control()
print(f"car {car.type_id} id {car.id} at {car.get_transform().location}  mass {pc.mass:.0f} kg  wheels "
      + ", ".join(f"r {w.radius:.0f} cm damp {w.damping_rate:.2f}" for w in pc.wheels[:1])
      + f"  cameras {[(c.type_id, c.attributes.get('image_size_x')) for c in cams]}")

rows = []
end = time.time() + secs
while time.time() < end:
    snap = world.wait_for_tick(10.0)
    a = snap.find(car.id)
    if a is None:
        continue
    t, v, av = a.get_transform(), a.get_velocity(), a.get_angular_velocity()
    row = dict(frame=snap.frame, dt=snap.timestamp.delta_seconds, x=t.location.x, y=t.location.y, z=t.location.z,
               pitch=t.rotation.pitch, roll=t.rotation.roll, yaw=t.rotation.yaw, speed=v.length(), vz=v.z,
               wx=av.x, wy=av.y, wz=av.z)
    if cams:
        ca = snap.find(cams[0].id)
        if ca is not None:
            ct = ca.get_transform()
            p = local(t.get_inverse_matrix(), ct.location)
            row.update(cam_x=p[0], cam_y=p[1], cam_z=p[2], cam_dyaw=wrap(ct.rotation.yaw - t.rotation.yaw),
                       cam_dpitch=wrap(ct.rotation.pitch - t.rotation.pitch))
    rows.append(row)

out = sys.argv[2] if len(sys.argv) > 2 else "/work/logs/jiggle.csv"
with open(out, "w", newline="") as f:
    wr = csv.DictWriter(f, fieldnames=list(rows[-1]))
    wr.writeheader()
    wr.writerows(r for r in rows if len(r) == len(rows[-1]))
dt = np.array([r["dt"] for r in rows])
print(f"{len(rows)} frames  dt mean {dt.mean()*1000:.1f} ms  std {dt.std()*1000:.1f}  min {dt.min()*1000:.1f}  max {dt.max()*1000:.1f}"
      f"  speed mean {np.mean([r['speed'] for r in rows]):.2f} m/s  -> {out}")
print("car:")
for k, u in (("z", "m"), ("pitch", "deg"), ("roll", "deg"), ("vz", "m/s"), ("wx", "deg/s"), ("wy", "deg/s")):
    jitter(k, [r[k] for r in rows], u)
if "cam_x" in rows[-1]:
    print("camera in the car's frame:")
    rc = [r for r in rows if "cam_x" in r]
    for k, u in (("cam_x", "m"), ("cam_y", "m"), ("cam_z", "m"), ("cam_dyaw", "deg"), ("cam_dpitch", "deg")):
        jitter(k, [r[k] for r in rc], u)
