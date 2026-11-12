"""Checks a generated map from the mesh list level_audit.py wrote: no tree, street light, fire hydrant or light box
on or right next to a lane, no building standing on one, and a road surface at the right height on every raised
junction (the stock tool left those without one). Exit code 1 if anything is wrong.

usage: python check_level.py Name_meshes.jsonl Name.xodr [x y] [--write-bad PATH]
With x y (CARLA metres) it also lists every mesh whose bounding box covers that point. --write-bad writes the trees and
props it rejects to PATH as JSON, for level_prune.py to remove in the editor.
"""
import collections
import json
import math
import re
import sys

import carla

CLEAR = 0.5  # m: a trunk or pole closer than this to a lane's edge counts as on the road
CROWN = 4.0  # m: trees and lights closer than this to a driving lane's edge reach over it (PlaceSpot keeps 4.5)
KINDS = [("road", r"DrivingLane|Sidewalk|LaneMark|/Terrain/|LandscapeMesh"), ("building", r"/Static/Buildings/"),
         ("rail", r"GuardRail|Guardrail"),
         ("prop", r"StreetLight|Fire.?H|LightBox"),  # before trees: "Street" contains "tree"
         ("tree", r"Pine|Oak|Maple|Bush|Birch|Beech|Tree|Veg|Plant")]


def kind(mesh):
    return next((k for k, pattern in KINDS if re.search(pattern, mesh, re.I)), "other")


def lane_gap(m, x, y, kind=carla.LaneType.Any):
    w = m.get_waypoint(carla.Location(x, y, 0), project_to_road=True, lane_type=kind)
    if w is None:
        return 1e9, None
    c = w.transform.location
    return math.hypot(c.x - x, c.y - y) - w.lane_width / 2, w


def raised_junction_holes(m, xodr, roads):
    """Junctions over 0.5 m up with no road mesh whose box covers their middle at their height (within 1.5 m)."""
    import xml.etree.ElementTree as ET
    holes = []
    for j in ET.fromstring(open(xodr).read()).iter("junction"):
        for c in j.findall("connection"):
            rid = int(c.get("connectingRoad"))
            w = m.get_waypoint_xodr(rid, -1, 0.5) or m.get_waypoint_xodr(rid, 1, 0.5)
            if w is None:
                continue
            ahead = w.next(max(0.5, w.lane_width))  # a little into the junction
            p = (ahead[0] if ahead else w).transform.location
            if p.z > 0.5 and not any(abs(b[0] - p.x) <= b[3] and abs(b[1] - p.y) <= b[4] and b[2] - b[5] - 1.5 <= p.z <= b[2] + b[5] + 1.5
                                     for b in (r["box"] for r in roads)):
                holes.append((j.get("id"), p.x, p.y, p.z))
                break
    return holes


def main(meshes, xodr, point=None, write_bad=None):
    m = carla.Map("check", open(xodr).read())
    rows = [json.loads(line) for line in open(meshes)]
    count, bad = collections.Counter(kind(r["mesh"]) for r in rows), []
    for r in rows:
        k = kind(r["mesh"])
        if k in ("tree", "prop"):
            gap, w = lane_gap(m, r["x"], r["y"])
            road, rw = lane_gap(m, r["x"], r["y"], carla.LaneType.Driving)
            if road < CROWN:
                gap, w = min((gap, w), (road - CROWN + CLEAR, rw), key=lambda g: g[0])
        elif k == "rail":  # a rail piece's middle must not be on a driving lane at its own level
            gap, w = lane_gap(m, r["box"][0], r["box"][1], carla.LaneType.Driving)
            if w is not None and abs(w.transform.location.z - (r["box"][2] - r["box"][5])) > 3.0:
                continue  # a bridge's rail over a road below it (or the road on a bridge above it)
            gap += CLEAR  # any overlap at all counts, nothing closer
        elif k == "building":
            gap, w = lane_gap(m, r["box"][0], r["box"][1])
            gap -= CLEAR  # only the middle is tested, so any overlap at all counts
        else:
            continue
        if gap < CLEAR:
            bad.append((gap, k, r, w))
    print("meshes:", dict(count))
    for gap, k, r, w in sorted(bad, key=lambda b: b[0])[:30]:
        print(f"  {k:8} {r['mesh'].rsplit('/', 1)[1]:32} at ({r['x']:7.1f}, {r['y']:7.1f}, {r['z']:5.1f}) {gap:+.1f} m "
              f"from {w.lane_type} lane of road {w.road_id} ({r['tile']})")
    if write_bad:
        with open(write_bad, "w") as f:
            json.dump([r for _, k, r, _ in bad if k in ("tree", "prop", "rail")], f)
    print(f"{len(bad)} on a lane, within {CLEAR} m of one, or within {CROWN} m of a driving lane:",
          dict(collections.Counter(b[1] for b in bad)))
    holes = raised_junction_holes(m, xodr, [r for r in rows if kind(r["mesh"]) == "road"])
    print(f"{len(holes)} raised junctions without a road surface at their height")
    for h in holes:
        print("  junction %s at (%.0f, %.0f) z %.1f" % h)
    if point:
        x, y = point
        print(f"meshes covering ({x}, {y}):")
        for r in rows:
            bx, by, bz, ex, ey, ez = r["box"]
            if abs(bx - x) <= ex and abs(by - y) <= ey:
                print(f"  {kind(r['mesh']):8} {r['mesh'].rsplit('/', 1)[1]:40} z {bz - ez:6.1f} .. {bz + ez:6.1f} ({r['actor']})")
    return not bad and not holes


if __name__ == "__main__":
    args = sys.argv[1:]
    write_bad = None
    if "--write-bad" in args:
        i = args.index("--write-bad")
        write_bad = args[i + 1]
        del args[i:i + 2]
    ok = main(args[0], args[1], tuple(map(float, args[2:4])) if len(args) > 3 else None, write_bad)
    sys.exit(0 if ok else 1)
