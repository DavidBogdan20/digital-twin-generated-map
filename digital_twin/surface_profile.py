# The 3D road surface height along a lane, every 0.25 m, from the server's ground trace: steps show up as jumps.
import sys, carla
road, lane, s0, s1 = int(sys.argv[1]), int(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])
c = carla.Client("localhost", 2000); c.set_timeout(30)
w = c.get_world(); m = w.get_map()
prev = None
s = s0
while s <= s1:
    wp = m.get_waypoint_xodr(road, lane, s)
    if wp is None:
        nxt = None
    else:
        p = wp.transform.location
        out = []
        for off in (-1.0, 0.0, 1.0):  # across the lane
            r = wp.transform.get_right_vector()
            q = carla.Location(p.x + r.x * off, p.y + r.y * off, p.z + 3.0)
            hit = w.ground_projection(q, 8.0)
            out.append(hit.location.z if hit else float("nan"))
        jump = max(abs(a - b) for a, b in zip(out, prev)) if prev else 0.0
        print("s %6.2f (%.1f, %.1f) xodr %.2f surface %s%s" % (s, p.x, p.y, p.z, " ".join("%.2f" % z for z in out), "  <-- STEP %.2f" % jump if jump > 0.08 else ""))
        prev = out
    s += 0.25
