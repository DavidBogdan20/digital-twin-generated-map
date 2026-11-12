# usage: python check_heights.py map.xodr [rows]   (exit code 1 when anything is found)
# Lanes of any type lying over another road's driving lane at a different height at the same spot: a step a car cannot
# drive over (> 0.15 m) nor under (< 4 m). Heights are compared at the same point (the other lane's z carried along its
# slope), so a ramp's grade does not count.
import math, sys, collections, carla
import xml.etree.ElementTree as ET
xodr = open(sys.argv[1]).read(); m = carla.Map("w", xodr)
LIFT = {carla.LaneType.Sidewalk: 0.30}  # a generated sidewalk stands this much over its lane height
pts = []
for road in ET.fromstring(xodr).iter("road"):
    rid, L = int(road.get("id")), float(road.get("length"))
    for lane in road.iter("lane"):
        lid = int(lane.get("id"))
        if lid == 0 or lane.get("type") in ("none",): continue
        s = 0.25
        while s < L:
            w = m.get_waypoint_xodr(rid, lid, s)
            if w is not None and w.lane_width > 0.3:
                pts.append(w)
            s += 1.0
grid = collections.defaultdict(list)
for w in pts:
    l = w.transform.location; grid[(int(l.x // 5), int(l.y // 5))].append(w)
bad = collections.defaultdict(list)
for a in pts:
    if a.lane_type != carla.LaneType.Driving: continue
    p = a.transform.location
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            for b in grid[(int(p.x // 5) + i, int(p.y // 5) + j)]:
                if b.road_id == a.road_id: continue
                q = b.transform.location
                f = b.transform.get_forward_vector()
                along = (p.x - q.x) * f.x + (p.y - q.y) * f.y
                across = math.hypot(p.x - q.x - along * f.x, p.y - q.y - along * f.y)
                if abs(along) > 0.6 or across > a.lane_width / 2 + b.lane_width / 2 - 0.5:
                    continue  # not the same spot, or only the edges touch
                zb = q.z + math.tan(math.radians(b.transform.rotation.pitch)) * along + LIFT.get(b.lane_type, 0.0)
                dz = abs(p.z - zb)
                if 0.15 < dz < 4.0:
                    bad[(a.road_id, b.road_id, str(b.lane_type))].append((round(p.x), round(p.y), round(dz, 2)))
print(len(bad), "places where a lane lies over another road's driving lane 0.15 to 4 m off its height")
for key, v in sorted(bad.items(), key=lambda kv: -max(x[2] for x in kv[1]))[:int(sys.argv[2]) if len(sys.argv) > 2 else 15]:
    print(key, len(v), "points, dz up to %.2f m, e.g. (%d, %d)" % (max(x[2] for x in v), v[0][0], v[0][1]))

sys.exit(1 if bad else 0)
