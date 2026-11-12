"""Measures how much paved surface lies across a lane: rays every 5 cm from 3 m left to 3 m right of the lane centre,
every few metres along one road. Paved = a hit within 0.12 m of the height at the centre (beyond the edge the ground
lies 0.2-0.6 m lower). usage: lane_width_scan.py road lane [step m]"""
import sys
import numpy as np
import carla
road, lane = int(sys.argv[1]), int(sys.argv[2])
step = float(sys.argv[3]) if len(sys.argv) > 3 else 5.0
m = carla.Client("localhost", 2000).get_world().get_map()
w = carla.Client("localhost", 2000).get_world()
wp = m.get_waypoint_xodr(road, lane, 0.5)
out = []
while wp is not None and wp.road_id == road:
    c, r = wp.transform.location, wp.transform.get_right_vector()
    def top(o):
        p = carla.Location(c.x + r.x * o, c.y + r.y * o, c.z)
        hits = [h.location.z for h in w.cast_ray(p + carla.Location(z=3), p - carla.Location(z=3))]
        return max(hits) if hits else np.nan  # topmost surface
    zc = top(0.0)
    offs = np.arange(-3.0, 3.01, 0.05)
    paved = [abs(top(o) - zc) < 0.12 for o in offs]
    i0 = 60  # index of offset 0
    left = i0
    while left > 0 and paved[left - 1]: left -= 1
    right = i0
    while right < len(offs) - 1 and paved[right + 1]: right += 1
    out.append((wp.s, c.x, c.y, offs[left], offs[right], wp.lane_width))
    nxt = wp.next(step)
    wp = nxt[0] if nxt and nxt[0].road_id == road else None
for s, x, y, a, b, lw in out:
    flag = "  <-- narrower" if (b - a) < lw - 0.25 else ""
    print(f"s {s:6.1f} at {x:6.1f},{y:7.1f}: paved {a:+.2f} .. {b:+.2f} m = {b - a:.2f} m (lane {lw:.2f}, edges +-{lw/2:.2f}){flag}")
