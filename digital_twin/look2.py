# Spectator at target + offset, looking at the target. Usage: look2.py tx ty tz ox oy oz
import math, sys, time, carla
tx, ty, tz, ox, oy, oz = map(float, sys.argv[1:7])
c = carla.Client("localhost", 2000); c.set_timeout(30)
w = c.get_world()
eye = carla.Location(tx + ox, ty + oy, tz + oz)
d = carla.Location(tx, ty, tz) - eye
w.get_spectator().set_transform(carla.Transform(eye, carla.Rotation(
    pitch=math.degrees(math.atan2(d.z, math.hypot(d.x, d.y))), yaw=math.degrees(math.atan2(d.y, d.x)))))
time.sleep(4)
