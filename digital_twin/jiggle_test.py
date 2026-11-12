"""Where does a car's jiggle come from? Spawns a car with manual_control's chase camera (spring arm, 1920x1080) next to
the spectator, parks it, then drives it along its lane, and reports the frame-to-frame jitter of the car itself (physics,
road surface) and of the camera relative to the car (what the driving window shows).
usage: jiggle_test.py async|sync [blueprint] [seconds per phase] [speed m/s] [spawn x,y (default: by the spectator)]
async runs like manual_control (the server ticks as fast as it can); sync ticks fixed 0.05 s steps. Writes
/work/logs/jiggle_<mode>.csv. Run with no other client: it changes nothing it does not restore."""
import csv
import math
import sys

import carla
import numpy as np

MODE = sys.argv[1]
BP = sys.argv[2] if len(sys.argv) > 2 else "vehicle.tesla.cybertruck"
SECS = float(sys.argv[3]) if len(sys.argv) > 3 else 10.0
SPEED = float(sys.argv[4]) if len(sys.argv) > 4 else 15.0
NEAR = [float(v) for v in sys.argv[5].split(",")] if len(sys.argv) > 5 else None


def wrap(a):
    return (a + 180.0) % 360.0 - 180.0


def jitter(name, v, unit):
    v = np.asarray(v, float)
    if len(v) < 5:
        print(f"    {name:<11} too few samples ({len(v)})")
        return
    fast = v[2:-2] - np.convolve(v, np.ones(5) / 5, mode="same")[2:-2]  # what changes within a few frames
    print(f"    {name:<11} range {np.ptp(v):8.4f}   jitter std {fast.std():7.4f}  peak {np.abs(fast).max():7.4f} {unit}")


def steer_along_lane(car, m, max_steer):
    """pure pursuit on the lane centre with a short look ahead (merge_test.py's), so the car keeps to the middle"""
    t, v = car.get_transform(), car.get_velocity().length()
    ahead = max(5.0, 0.5 * v)
    here = m.get_waypoint(t.location)
    target = (here.next(ahead) or [here])[0].transform.location  # a road that ends: steer to its end
    f, d = t.get_forward_vector(), target - t.location
    alpha = math.atan2(f.x * d.y - f.y * d.x, f.x * d.x + f.y * d.y)
    steer = math.degrees(math.atan2(2 * 2.8 * math.sin(alpha), ahead)) / max_steer
    return carla.VehicleControl(throttle=float(np.clip(0.3 * (SPEED - v) + 0.3, 0, 0.8)), steer=float(np.clip(steer, -1, 1)),
                                brake=0.3 if v > SPEED + 1 else 0.0)


def lane_offset(m, loc):
    """(how far right of its lane's centre loc is, the lane's width) [m]"""
    w = m.get_waypoint(loc)
    r = w.transform.get_right_vector()
    return (loc.x - w.transform.location.x) * r.x + (loc.y - w.transform.location.y) * r.y, w.lane_width


client = carla.Client("localhost", 2000)
client.set_timeout(60)
world = client.get_world()
m = world.get_map()
original = world.get_settings()
if MODE == "sync":
    s = world.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, 0.05
    world.apply_settings(s)
tick = world.tick if MODE == "sync" else (lambda: world.wait_for_tick(10.0))
car = cam = None
rows = []
seen = {}  # frame -> the car's centre x, y and lowest row in the picture [px]
try:
    here = carla.Location(*NEAR) if NEAR else world.get_spectator().get_location()
    spawn = min(m.get_spawn_points(), key=lambda p: p.location.distance(here))
    bp = world.get_blueprint_library().find(BP)
    bp.set_attribute("role_name", "hero")
    car = world.spawn_actor(bp, spawn)
    max_steer = car.get_physics_control().wheels[0].max_steer_angle
    b = car.bounding_box.extent
    # a segmentation camera with the driving window's size and mount: where the car lands in each picture is what the
    # window shows, including any lag between the camera and the car that the poses above cannot show
    cbp = world.get_blueprint_library().find("sensor.camera.semantic_segmentation")
    cbp.set_attribute("image_size_x", "1920")
    cbp.set_attribute("image_size_y", "1080")
    cam = world.spawn_actor(cbp, carla.Transform(carla.Location(x=-2.0 * b.x, z=2.0 * b.z), carla.Rotation(pitch=8.0)),
                            attach_to=car, attachment_type=carla.AttachmentType.SpringArmGhost)
    def on_image(image):
        tags = np.frombuffer(image.raw_data, np.uint8).reshape(image.height, image.width, 4)[:, :, 2]
        ys, xs = np.nonzero((tags >= 14) & (tags <= 16))  # car, truck, bus
        if len(xs):
            seen[image.frame] = (xs.mean(), ys.mean(), ys.max())
    cam.listen(on_image)
    s = world.get_settings()
    print(f"{m.name}  {MODE}  substepping {s.substepping} {s.max_substep_delta_time} x{s.max_substeps}  {BP} at {spawn.location}")
    for phase, frames_settle in (("parked", 60), ("driving", 150)):
        n = 0
        while True:
            car.apply_control(carla.VehicleControl(hand_brake=True) if phase == "parked" else steer_along_lane(car, m, max_steer))
            tick()
            n += 1
            if n <= frames_settle:
                continue
            snap = world.get_snapshot()
            a, c = snap.find(car.id), snap.find(cam.id)
            t, ct, v, av = a.get_transform(), c.get_transform(), a.get_velocity(), a.get_angular_velocity()
            p = np.array(t.get_inverse_matrix()) @ np.array([ct.location.x, ct.location.y, ct.location.z, 1.0])
            lat, width = lane_offset(m, t.location)
            rows.append(dict(phase=phase, lat=lat, width=width, frame=snap.frame, t=snap.timestamp.elapsed_seconds, dt=snap.timestamp.delta_seconds,
                             x=t.location.x, y=t.location.y, z=t.location.z, pitch=t.rotation.pitch, roll=t.rotation.roll,
                             yaw=t.rotation.yaw, speed=v.length(), vz=v.z, wx=av.x, wy=av.y,
                             cam_x=p[0], cam_y=p[1], cam_z=p[2], cam_dyaw=wrap(ct.rotation.yaw - t.rotation.yaw),
                             cam_dpitch=wrap(ct.rotation.pitch - t.rotation.pitch)))
            if rows[-1]["t"] - next(r["t"] for r in rows if r["phase"] == phase) >= SECS:
                break
finally:
    for actor in (cam, car):
        if actor is not None:
            actor.destroy()
    world.apply_settings(original)

for r in rows:
    r["img_x"], r["img_y"], r["img_bottom"] = seen.get(r["frame"], (np.nan,) * 3)
out = f"/work/logs/jiggle_{MODE}.csv"
with open(out, "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)
for phase in ("parked", "driving"):
    rs = [r for r in rows if r["phase"] == phase]
    dt = np.array([r["dt"] for r in rs])
    print(f"  {phase}: {len(rs)} frames, dt {dt.mean() * 1000:.1f} ms (std {dt.std() * 1000:.1f}, max {dt.max() * 1000:.0f}),"
          f" speed {np.mean([r['speed'] for r in rs]):.1f} m/s, {min(r['lat'] for r in rs):+.2f} to {max(r['lat'] for r in rs):+.2f} m"
          f" right of the lane centre (width {min(r['width'] for r in rs):.2f}-{max(r['width'] for r in rs):.2f}), from {rs[0]['x']:.0f},{rs[0]['y']:.0f} to {rs[-1]['x']:.0f},{rs[-1]['y']:.0f}")
    print("   car:")
    for k, u in (("z", "m"), ("pitch", "deg"), ("roll", "deg"), ("vz", "m/s"), ("wx", "deg/s"), ("wy", "deg/s")):
        jitter(k, [r[k] for r in rs], u)
    print("   camera relative to the car:")
    for k, u in (("cam_x", "m"), ("cam_y", "m"), ("cam_z", "m"), ("cam_dyaw", "deg"), ("cam_dpitch", "deg")):
        jitter(k, [r[k] for r in rs], u)
    pics = [r for r in rs if not np.isnan(r["img_x"])]
    print(f"   the car in the picture ({len(pics)} of {len(rs)} frames):")
    for k in ("img_x", "img_y", "img_bottom"):
        jitter(k, [r[k] for r in pics], "px")
print("->", out)
