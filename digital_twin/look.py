# Puts the spectator above (x, y) looking down, waits, and leaves it there for a screenshot. Usage: look.py x y [height] [pitch] [yaw]
import sys, time, carla
x, y = float(sys.argv[1]), float(sys.argv[2])
h = float(sys.argv[3]) if len(sys.argv) > 3 else 30
pitch = float(sys.argv[4]) if len(sys.argv) > 4 else -90
yaw = float(sys.argv[5]) if len(sys.argv) > 5 else -90
c = carla.Client("localhost", 2000); c.set_timeout(30)
w = c.get_world()
w.get_spectator().set_transform(carla.Transform(carla.Location(x, y, h), carla.Rotation(pitch=pitch, yaw=yaw)))
time.sleep(4)
