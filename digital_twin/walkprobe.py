# Background walkers near a point, as the scenario's background block does them, logging every step (flushes).
import math, random, sys, carla
cx, cy, n = float(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3])
random.seed(int(sys.argv[4]) if len(sys.argv) > 4 else 1)
c = carla.Client("localhost", 2000); c.set_timeout(30)
w = c.get_world(); m = w.get_map()
lib = w.get_blueprint_library()
pts = []
for _ in range(4000):
    p = w.get_random_location_from_navigation()
    if p and math.hypot(p.x - cx, p.y - cy) < 120 and (len(sys.argv) < 6 or m.get_waypoint(p, project_to_road=True, lane_type=carla.LaneType.Any).lane_type == carla.LaneType.Sidewalk):
        pts.append(p)
print(len(pts), "navigation points within 120 m", flush=True)
s = w.get_settings(); s.synchronous_mode = True; s.fixed_delta_seconds = 0.05; w.apply_settings(s)
actors = []
try:
    walkers = []
    for p in random.sample(pts, min(n, len(pts))):
        wp = m.get_waypoint(p, project_to_road=True, lane_type=carla.LaneType.Any)
        bp = random.choice(lib.filter("walker.pedestrian.*"))
        a = w.try_spawn_actor(bp, carla.Transform(p + carla.Location(z=1.0)))
        print("walker at (%.1f, %.1f, %.2f) lane %s road %s -> %s" % (p.x, p.y, p.z, wp.lane_type if wp else None, wp.road_id if wp else None, a.id if a else "refused"), flush=True)
        if a:
            walkers.append(a); actors.append(a)
    w.tick()
    ctrls = []
    for a in walkers:
        ctrl = w.spawn_actor(lib.find("controller.ai.walker"), carla.Transform(), attach_to=a)
        actors.append(ctrl); ctrls.append(ctrl)
    w.tick()
    for ctrl in ctrls:
        ctrl.start(); ctrl.go_to_location(random.choice(pts)); ctrl.set_max_speed(1.4)
    print("controllers started", flush=True)
    for t in range(400):
        w.tick()
        if t % 50 == 0:
            zs = [a.get_location().z for a in walkers]
            print("tick %d: z %.1f .. %.1f, alive %d" % (t, min(zs), max(zs), sum(a.is_alive for a in walkers)), flush=True)
    print("no crash", flush=True)
finally:
    s.synchronous_mode = False; s.fixed_delta_seconds = None; w.apply_settings(s)
    for a in reversed(actors):
        try:
            if a.type_id.startswith("controller"): a.stop()
            a.destroy()
        except RuntimeError:
            pass
