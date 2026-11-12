"""Checks a generated map's surfaces against its road data, live, with the map loaded:
(1) the road mesh at lane centres minus the waypoint height: about 0 with apply_patches.py's flat roads, up to 1.2 m
    with the stock tool's waves;
(2) the ground 0.3 m and 1 m past an open lane edge (no sidewalk, road data under 0.15 m: higher ones have rails):
    the drop from the road, and the ground's own height, which the patch keeps between -0.12 and -0.01 m (the stock
    tool dug a trench to 0.8 m under the road surface).
usage: surface_check.py [step m]   in the container, no other actors. Exit code 1 if (1) passes 0.05 m or the ground
lies under -0.15 m anywhere, 2 if the rays hit nothing.
"""
import sys

import carla
import numpy as np

STEP = float(sys.argv[1]) if len(sys.argv) > 1 else 25.0
world = carla.Client("localhost", 2000).get_world()
m = world.get_map()


def hits(x, y, z):
    return [h.location.z for h in world.cast_ray(carla.Location(x, y, z + 2), carla.Location(x, y, z - 2))]


def lane_at(x, y, z):
    return m.get_waypoint(carla.Location(x, y, z), project_to_road=False, lane_type=carla.LaneType.Any)


off, drops = [], {0.3: [], 1.0: []}
worst = []
for w in m.generate_waypoints(STEP):
    if w.lane_type != carla.LaneType.Driving or w.is_junction or w.s < 0.5:
        continue  # s 0 lies on the seam where a road meets its junction: a ray there can pass between the two meshes
    c = w.transform.location
    h = hits(c.x, c.y, c.z)
    if h:
        off.append(min(h, key=lambda q: abs(q - c.z)) - c.z)
    side = w.get_right_lane()
    if c.z >= 0.15 or not (side is None or side.lane_width < 0.3 or side.lane_type in (carla.LaneType.Border, carla.LaneType.NONE)):
        continue
    r = w.transform.get_right_vector()
    edge = w.lane_width / 2
    for d in drops:
        x, y = c.x + r.x * (edge + d), c.y + r.y * (edge + d)
        other = lane_at(x, y, c.z)
        if other is not None and other.lane_width >= 0.3:
            continue  # another road or a sidewalk there
        below = [q for q in hits(x, y, c.z) if q <= c.z + 0.05]
        if below:
            drops[d].append(c.z - max(below))
            if max(below) < -0.15:  # a trench, not just the road's own height over the ground
                worst.append((round(max(below), 2), d, round(x, 1), round(y, 1), w.road_id))
if not off or not all(drops.values()):
    print("no ray hits (cast_ray found nothing): is the map loaded and streamed in?")
    sys.exit(2)
off = np.array(off)
print(f"road mesh minus road data at {len(off)} lane centres: median {np.median(off):+.3f} m, "
      f"95% within {np.percentile(np.abs(off), 95):.3f} m, worst {off[np.argmax(np.abs(off))]:+.3f} m")
for d, v in drops.items():
    v = np.array(v)
    print(f"ground {d} m past an open lane edge, {len(v)} points: drop median {np.median(v):.3f} m, "
          f"95% {np.percentile(v, 95):.3f} m, max {v.max():.3f} m, over 0.15 m: {(v > 0.15).sum()}")
for item in sorted(worst)[:8]:
    print("  ground at %.2f m, %.1f m past the edge, (%.1f, %.1f) road %d" % item)
sys.exit(1 if np.abs(off).max() > 0.05 or worst else 0)
