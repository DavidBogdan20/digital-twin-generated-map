"""Lists what lies under the right front wheel of the last jiggle_test run between s0 and s1 m along its path."""
import csv, math, sys
import numpy as np
import carla
s0, s1 = float(sys.argv[1]), float(sys.argv[2])
c = carla.Client("localhost", 2000); w = c.get_world(); m = w.get_map()
rs = [r for r in csv.DictReader(open("/work/logs/jiggle_sync.csv")) if r["phase"] == "driving"]
P = np.array([[float(r[k]) for k in ("x", "y", "z", "yaw")] for r in rs])
s = np.concatenate([[0], np.cumsum(np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1])))])
for S in np.arange(s0, s1, 0.2):
    x, y, z = (np.interp(S, s, P[:, i]) for i in range(3))
    a = np.interp(S, s, np.unwrap(np.radians(P[:, 3])))
    px, py = x + 1.45 * math.cos(a) - 0.8 * math.sin(a), y + 1.45 * math.sin(a) + 0.8 * math.cos(a)
    hits = [(round(p.location.z, 2), str(p.label)) for p in w.cast_ray(carla.Location(px, py, z + 3), carla.Location(px, py, z - 3))]
    loc = carla.Location(px, py, z)
    wp = m.get_waypoint(loc, project_to_road=False, lane_type=carla.LaneType.Any)
    cw = m.get_waypoint(carla.Location(x, y, z))
    r = cw.transform.get_right_vector()
    lat = (x - cw.transform.location.x) * r.x + (y - cw.transform.location.y) * r.y
    where = f"road {wp.road_id} lane {wp.lane_id} {wp.lane_type} w {wp.lane_width:.2f}" if wp else "no lane"
    print(f"s {S:5.1f} wheel {px:6.1f},{py:7.1f} car z {z:5.2f} | car {lat:+.2f} m right of lane centre (w {cw.lane_width:.2f}) | wheel on {where} | hits {hits}")
