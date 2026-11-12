# Lists every map object around the A 39 launch point on Wolfsburg2.
import math, time, carla
c = carla.Client("localhost", 2000); c.set_timeout(300)
w = c.get_world()
if not w.get_map().name.endswith("Wolfsburg2"):
    w = c.load_world("Wolfsburg2")
s = w.get_settings(); s.tile_stream_distance = 350; s.actor_active_distance = 300; w.apply_settings(s)
P = carla.Location(1200, -1436, 4)
w.get_spectator().set_transform(carla.Transform(carla.Location(1240, -1400, 40), carla.Rotation(pitch=-35, yaw=-135)))
for _ in range(40): w.wait_for_tick(); time.sleep(0.25)
objs = w.get_environment_objects(carla.CityObjectLabel.Any)
print(len(objs), "objects loaded")
near = []
for o in objs:
    b = o.bounding_box; l = b.location
    dx = max(0, abs(l.x - P.x) - b.extent.x); dy = max(0, abs(l.y - P.y) - b.extent.y)
    if math.hypot(dx, dy) < 6:
        near.append((math.hypot(dx, dy), o.name, str(o.type), round(l.x,1), round(l.y,1), round(l.z - b.extent.z, 2), round(l.z + b.extent.z, 2), round(b.extent.x,1), round(b.extent.y,1)))
for n in sorted(near): print(n)
