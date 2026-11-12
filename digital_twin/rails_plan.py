"""Where guard rails go on a generated map: along the outer edge of every raised road, leaving gaps where another road
comes close (merges, exits, junctions). Writes polylines for rails.py, which places CARLA's guard rail mesh in the
editor. Needs only the carla module.

usage: python rails_plan.py map.xodr out.json
Without rails a car that left its lane on a ramp or bridge fell 3 to 5 m (WolfsburgHighway, 2026-11-10). From 0.15 m up
since 2026-11-11: below 1 m a wheel past the edge still dropped off a bare step (the ground lies at 0) and jolted the car. For maps built with
apply_patches.py's flat roads (the mesh at the road file's heights); a map with the stock waves needs them added back.
"""
import json
import math
import sys
import xml.etree.ElementTree as ET

import carla

RAISED = 0.15      # m: road heights from here up get a rail
OFFSET = 0.3       # m beyond the outermost lane's edge
CLEAR = 2.5        # m: no rail where another road's lane comes this close beyond the edge
STEP = 2.0         # m between polyline points
TILE = 500.0       # m


def other_road_near(m, p, out, road_id):
    """True when a lane of another road lies within CLEAR m outward of p."""
    for k in range(0, int(CLEAR / 0.25) + 1):
        q = carla.Location(p.x + out[0] * k * 0.25, p.y + out[1] * k * 0.25, p.z)
        w = m.get_waypoint(q, project_to_road=False, lane_type=carla.LaneType.Any)
        if w is not None and w.road_id != road_id:
            return True
    return False


def plan(xodr):
    m = carla.Map("rails", xodr)
    lines = []
    for road in ET.fromstring(xodr).iter("road"):
        if road.get("junction") != "-1":
            continue
        rid, length = int(road.get("id")), float(road.get("length"))
        ids = {int(l.get("id")) for l in road.iter("lane")}
        one_way = not any(i > 0 for i in ids)  # netconvert's usual road: right lanes only; its left edge is outside too
        for side, edge in ((-1, "right"), (1, "right")) + (((-1, "left"),) if one_way else ()):
            line = []
            s = 0.5
            while s < length - 0.5:
                outer = None
                order = range(side * 6, 0, -side) if edge == "right" else range(side, side * 7, side)
                for lid in order:  # the outermost (right edge) or innermost (left edge) lane with some width
                    w = m.get_waypoint_xodr(rid, lid, s)
                    if w is not None and w.lane_width > 0.3:
                        outer = w
                        break
                keep = None
                if outer is not None and outer.transform.location.z >= RAISED:
                    r, c = outer.transform.get_right_vector(), outer.transform.location
                    # right-hand traffic: a lane's right is the road's outside; the left edge of a one-way road too
                    out = (r.x, r.y) if edge == "right" else (-r.x, -r.y)
                    d = outer.lane_width / 2 + OFFSET
                    p = carla.Location(c.x + out[0] * d, c.y + out[1] * d, c.z)
                    if not other_road_near(m, p, out, rid):
                        lift = 0.3024 if outer.lane_type == carla.LaneType.Sidewalk else 0.0
                        keep = [round(p.x, 2), round(p.y, 2), round(c.z + lift, 2)]
                if keep:
                    line.append(keep)
                elif len(line) >= 3:
                    lines.append(line)
                    line = []
                else:
                    line = []
                s += STEP
            if len(line) >= 3:
                lines.append(line)
    return lines


def main(xodr_path, out_path):
    lines = plan(open(xodr_path).read())
    total = sum(math.dist(a[:2], b[:2]) for line in lines for a, b in zip(line, line[1:]))
    json.dump({"tile": TILE, "lines": lines}, open(out_path, "w"))
    print(f"{len(lines)} rail lines, {total:.0f} m of guard rail -> {out_path}")


if __name__ == "__main__":
    main(*sys.argv[1:3])
