"""Spawns walkers at sidewalk navigation points around a spot (no AI controllers, so no SetWalkerState) and reports
which go dormant and where. usage: walker_probe.py x y [radius] [count]"""
import math, random, sys, time
import carla
x, y = float(sys.argv[1]), float(sys.argv[2])
radius = float(sys.argv[3]) if len(sys.argv) > 3 else 120.0
count = int(sys.argv[4]) if len(sys.argv) > 4 else 40
c = carla.Client("localhost", 2000); c.set_timeout(60)
w = c.get_world(); m = w.get_map()
s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.05
s.tile_stream_distance = 3000; s.actor_active_distance = 3000
w.apply_settings(s)
w.get_spectator().set_transform(carla.Transform(carla.Location(x, y, 40), carla.Rotation(pitch=-90)))
for _ in range(20): w.tick()
print("settings", w.get_settings().tile_stream_distance, w.get_settings().actor_active_distance)
bps = w.get_blueprint_library().filter("walker.pedestrian.*")
random.seed(1); walkers = []; tries = 0
while len(walkers) < count and tries < 20000:
    tries += 1
    p = w.get_random_location_from_navigation()
    if p is None or math.hypot(p.x - x, p.y - y) > radius:
        continue
    if m.get_waypoint(p, project_to_road=False, lane_type=carla.LaneType.Sidewalk) is None:
        continue
    a = w.try_spawn_actor(random.choice(bps), carla.Transform(carla.Location(p.x, p.y, p.z + 1.0)))
    if a:
        walkers.append((a, p))
print("spawned", len(walkers), "tries", tries)
for t in range(100):
    w.tick()
    if t in (1, 10, 99):
        bad = [(a.id, round(p.x, 1), round(p.y, 1), round(p.z, 1)) for a, p in walkers if a.is_dormant or not a.is_active]
        print("tick", t, "dormant/inactive", len(bad), bad[:10])
for a, p in walkers:
    l = a.get_location()
    if a.is_alive and not a.is_dormant and (l.z < p.z - 3 or abs(l.x - p.x) > 3):
        print("moved", a.id, (round(p.x, 1), round(p.y, 1), round(p.z, 1)), "->", (round(l.x, 1), round(l.y, 1), round(l.z, 1)))
c.apply_batch_sync([carla.command.DestroyActor(a) for a, _ in walkers], True)
s.synchronous_mode = False; s.fixed_delta_seconds = None; w.apply_settings(s)
print("done")
