#!/usr/bin/env python
"""
Offline search, validation and plots of the WolfsburgHighway scenario spots (no CARLA server needed).

It loads the runtime OpenDRIVE with the CARLA 0.9.15 Python API (carla.Map(name, xodr)) and runs the repository's own
geometry (carsim.road: find_crossing, occluder_transforms, prop_transform, crowd_spawns, cyclist_spawns, ...; and
apps/find_crossing_spots.py: is_inner_lane) on it, so what it prints is what the scenario will compute live.

    docker build -t carla-client-analysis:0.9.15 tools/wolfsburg
    docker run --rm -v "$PWD":/repo -v <inputs>:/inputs carla-client-analysis:0.9.15 \
        python /repo/tools/wolfsburg/wolfsburg_spots.py --repo /repo \
            --xodr /inputs/WolfsburgHighway.xodr --keepout /inputs/WolfsburgHighway.keepout

  --repo DIR   the car_simulator checkout whose src/ and apps/ are used (default: two folders up from this file)
  --search     also print the ranked candidate tables (slower)
  --no-plots   skip the PNGs (tools/wolfsburg/plots/)

Every map is analysed twice: as the simulator reads it ("raw": its 6 crosswalk outlines are open 4-corner polygons, so
carsim.road.crosswalks() finds none and every crossing falls back to the junction entry) and with the outlines closed
("closed": what road.crosswalks() sees once the proposed fix is in, see WOLFSBURG_VARIANTS.md).
"""

import argparse
import importlib.util
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import carla

HERE = Path(__file__).resolve().parent

# The chosen spots. ego: x, y, yaw of the start (z is computed: the lane's xodr z + EGO_LIFT).
EGO_LIFT = 0.5  # m over the OpenDRIVE z (the road mesh waves about +-1.2 m around it; cars sat 0.5 to 0.75 m below)
CHOICES = {
    "PedestrianCrossing_WolfsburgHighway_1": dict(kind="crossing", lane="kerb", ego=(1619.4, -354.7)),
    "OccludedCrossing_Child_WolfsburgHighway_1": dict(kind="occluder", lane="inner", prop=True, ego=(1512.77, -1156.05)),
    "OccludedCrossing_Crowd_WolfsburgHighway_1": dict(kind="occluder", lane="inner", prop=False, ego=(1512.77, -1156.05)),
    "CutIn_Parking_WolfsburgHighway_1": dict(kind="parking", distance=66.0, end=100.0, ego=(1558.64, -1100.42)),
    "CutIn_Cyclist_WolfsburgHighway_1": dict(kind="cyclist", distance=40.0, end=90.0, ego=(888.33, -1691.49)),
}
# The backups, validated and plotted the same way
BACKUPS = {
    "PedestrianCrossing_backup": dict(kind="crossing", lane="kerb", ego=(1605.81, -499.86)),
    "OccludedCrossing_Child_backup": dict(kind="occluder", lane="inner", prop=True, ego=(944.39, -1827.26)),
    "CutIn_Parking_backup": dict(kind="parking", distance=66.0, end=100.0, ego=(888.33, -1691.49)),
    "CutIn_Cyclist_backup": dict(kind="cyclist", distance=40.0, end=90.0, ego=(956.56, -1796.80)),
}

# Measured CARLA vehicle sizes (half length, half width) from the repository's CLAUDE.md
HALF = {"european_hgv": (3.95, 1.45), "firetruck": (4.25, 1.45), "sprinter": (2.95, 1.0), "car": (2.4, 0.95),
        "bike": (0.9, 0.43), "busstop": (1.95, 0.95)}
URBAN = {"residential", "tertiary", "secondary", "primary", "unclassified", "tertiary_link"}


# --- loading -----------------------------------------------------------------------------------------------------------

def close_crosswalks(xodr):
    """The OpenDRIVE text with each crosswalk outline closed (its first corner repeated), as CARLA's Town maps write it."""
    def close(match):
        block = match.group(0)
        corners = re.findall(r"<cornerLocal [^>]*/>", block)
        first = re.sub(r'id="\d+"', 'id="{}"'.format(len(corners)), corners[0])
        return block.replace(corners[-1], corners[-1] + "\n                    " + first)
    return re.sub(r'<object [^>]*type="crosswalk".*?</object>', close, xodr, flags=re.S)


def crosswalks_open_outlines(road, cmap):
    """
    The proposed carsim.road.crosswalks() fix (WOLFSBURG_VARIANTS.md): when the map never repeats a polygon's first
    point (the Digital Twin Tool writes open 4-corner outlines), split the flat list into 4-point polygons.
    """
    polygons = road.crosswalks(cmap)
    points = list(cmap.get_crosswalks())
    if not polygons and points and len(points) % 4 == 0:
        polygons = [road.Crosswalk(points[i:i + 4]) for i in range(0, len(points), 4)]
    return polygons


def road_meta(xodr):
    """road id -> (class, name, has a non-zero elevation) from the OpenDRIVE text."""
    meta = {}
    for road in ET.fromstring(xodr).findall("road"):
        name = road.get("name") or ""
        elevated = any(abs(float(e.get("a"))) > 0.05 or abs(float(e.get("b"))) > 1e-4
                       for e in road.findall("elevationProfile/elevation"))
        meta[int(road.get("id"))] = (name.split("|")[0], name, elevated)
    return meta


def load_buildings(path):
    import shapely.geometry as sg
    from shapely.strtree import STRtree
    polys = []
    if path and Path(path).exists():
        for line in Path(path).read_text().splitlines():
            v = [float(t) for t in line.split()]
            if len(v) >= 6:
                polys.append(sg.Polygon(list(zip(v[0::2], v[1::2]))).buffer(0))
    return polys, STRtree(polys) if polys else None


def import_repo(repo):
    sys.path.insert(0, str(Path(repo) / "src"))
    spec = importlib.util.spec_from_file_location("find_crossing_spots", str(Path(repo) / "apps" / "find_crossing_spots.py"))
    fcs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fcs)
    import carsim.road as road
    return road, fcs


# --- measures ----------------------------------------------------------------------------------------------------------

def yaw_diff(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def along(road, wp, length, step=2.0):
    """Waypoints every `step` m from `wp` for `length` m, straight on at junctions (positive) or back (negative)."""
    out, s = [wp], step
    while s <= abs(length) + 1e-6:
        out.append(road.lane_point(wp, s if length > 0 else -s))
        s += step
    return out


def profile(meta, wps):
    """(max heading change from the first, z range, set of classes, elevated road met, junction met)."""
    yaw0 = wps[0].transform.rotation.yaw
    zs = [w.transform.location.z for w in wps]
    classes = {meta[w.road_id][0] for w in wps if not w.is_junction}
    return (max(yaw_diff(w.transform.rotation.yaw, yaw0) for w in wps), max(zs) - min(zs), classes,
            any(meta[w.road_id][2] for w in wps), any(w.is_junction for w in wps))


_POINTS = {}


def overpasses(cmap, wps, radius=6.0, dz=3.0):
    """Roads within `radius` m (2D) of the route `wps` and more than `dz` m above or below it (bridges)."""
    if id(cmap) not in _POINTS:
        _POINTS[id(cmap)] = [(o.road_id, o.transform.location) for o in cmap.generate_waypoints(4.0)]
    hits = set()
    for w in wps:
        a = w.transform.location
        for rid, b in _POINTS[id(cmap)]:
            if abs(a.x - b.x) < radius and abs(a.y - b.y) < radius and abs(a.z - b.z) > dz:
                hits.add(rid)
    return hits


def building_clearance(tree, polys, points):
    """Smallest distance [m] from the (x, y) points to a building outline (inf without buildings)."""
    if tree is None or not points:
        return float("inf")
    import shapely.geometry as sg
    best = float("inf")
    for x, y in points:
        p = sg.Point(x, y)
        for hit in tree.query(p.buffer(30.0)):
            poly = polys[hit] if isinstance(hit, (int,)) or hasattr(hit, "__index__") else hit
            best = min(best, poly.distance(p))
    return best


def box(cx, cy, yaw, half_l, half_w):
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy) for dx, dy in
            ((half_l, half_w), (half_l, -half_w), (-half_l, -half_w), (-half_l, half_w))]


def on_driving(cmap, x, y, z=0.0):
    return cmap.get_waypoint(carla.Location(x=x, y=y, z=z), project_to_road=False,
                             lane_type=carla.LaneType.Driving) is not None


# --- crossing spots (pedestrian_crossing, occluded_crossing) -----------------------------------------------------------

def crossing_candidates(road, fcs, cmap, meta, lane, approach, tolerance=2.0):
    """
    find_crossing_spots.find_candidates without its "painted crosswalk only" filter (WolfsburgHighway has 6), so the
    junction-entry crossings the scenario falls back to are listed too. Returns [(start_wp, Crossing)].
    """
    index = road.CrosswalkIndex(road.crosswalks(cmap))
    seen, found = set(), []
    for lane_start, junction_wp in cmap.get_topology():
        if lane_start.is_junction or not junction_wp.is_junction:
            continue
        key = (lane_start.road_id, lane_start.lane_id)
        if key in seen:
            continue
        seen.add(key)
        before = junction_wp.previous(2.0)
        if not before or before[0].is_junction:
            continue
        entry, _ = road.junction_entry_ahead(before[0], max_distance=5.0)
        if entry is None:
            continue
        right = entry.get_right_lane()
        if lane == "inner":
            if not fcs.is_inner_lane(entry, right):
                continue
        elif right is not None and right.lane_type == carla.LaneType.Driving:
            continue
        crossing = road.find_crossing(cmap, entry, index, max_distance=5.0, min_ahead=None)
        if crossing is None:
            continue
        for start in entry.previous(max(approach - crossing.distance, 0.5)):
            if start.is_junction:
                continue
            again = road.find_crossing(cmap, start, index, max_distance=approach + 50.0)
            if (again is not None and again.wp.transform.location.distance(crossing.wp.transform.location) < 1.0
                    and abs(again.distance - approach) <= tolerance):
                found.append((start, again))
                break
    return found


def crossing_report(road, cmap, meta, buildings, start, crossing, lane, prop):
    """Measures of one crossing spot; `bad` lists what disqualifies it."""
    polys, tree = buildings
    bad = []
    approach = along(road, start, crossing.distance)
    turn, dz, classes, elevated, junction = profile(meta, approach)
    last = along(road, crossing.wp, -40.0)
    last_turn = profile(meta, last)[0]
    after = road.straight_route(crossing.wp, 30.0)  # the ego drives 30 m past the crossing (end distance_after_crossing)
    after_turn = max([yaw_diff(w.transform.rotation.yaw, crossing.wp.transform.rotation.yaw) for w in after] or [0.0])
    after_zs = [w.transform.location.z for w in after]
    after_elev = bool(after_zs) and max(after_zs) - min(after_zs) > 0.5
    if classes - URBAN:
        bad.append("approach on " + ",".join(sorted(classes - URBAN)))
    if elevated or dz > 0.5 or after_elev:
        bad.append("elevated/ramp (dz {:.1f})".format(dz))
    if last_turn > 15.0:
        bad.append("last 40 m turn {:.0f} deg".format(last_turn))
    if after_turn > 30.0:
        bad.append("not straight after the junction ({:.0f} deg)".format(after_turn))
    sidewalk_r = road.find_sidewalk(crossing.wp, "right") is not None
    if not sidewalk_r:
        bad.append("no right sidewalk")
    crowd = road.crowd_spawns(cmap, crossing, 14)
    cyc = road.cyclist_spawns(cmap, crossing, 2)
    waits = [road.cyclist_waiting_spots(cmap, c, 1) for c in cyc]
    pts = [(crossing.spawn.location.x, crossing.spawn.location.y), (crossing.end.x, crossing.end.y)]
    pts += [(c.spawn.location.x, c.spawn.location.y) for c in crowd] + [(c.end.x, c.end.y) for c in crowd]
    pts += [(w[0].location.x, w[0].location.y) for w in waits if w]
    clear = building_clearance(tree, polys, pts)
    if clear < 0.5:
        bad.append("actor spot {:.1f} m from a building".format(clear))
    occ = None
    if lane == "inner":
        try:
            occ = road.occluder_transforms(crossing, [HALF["european_hgv"][0], HALF["car"][0]], 1.5)
        except ValueError:
            bad.append("no room for the truck and queue")
        if prop:
            ptf = road.prop_transform(crossing, *HALF["busstop"])
            corners = box(ptf.location.x, ptf.location.y, ptf.rotation.yaw, *HALF["busstop"])
            if any(on_driving(cmap, x, y, ptf.location.z) for x, y in corners):
                bad.append("bus stop on a driving lane")
            pc = building_clearance(tree, polys, corners)
            if pc < 0.3:
                bad.append("bus stop {:.1f} m from a building".format(pc))
    edge = min(start.transform.location.x, 2000 - start.transform.location.x,
               -start.transform.location.y, 2000 + start.transform.location.y)
    return dict(bad=bad, turn=turn, last_turn=last_turn, after_turn=after_turn, dz=dz, classes=classes,
                crosswalk=crossing.crosswalk is not None, walk=crossing.walk_distance, crowd=len(crowd),
                cyclists=len(cyc), waits=sum(1 for w in waits if w), clear=clear, edge=edge, occ=occ)


def describe(meta, wp):
    t = wp.transform
    return "({:.2f}, {:.2f}, z {:.2f}, yaw {:.1f}) road {} lane {} '{}'".format(
        t.location.x, t.location.y, t.location.z, t.rotation.yaw, wp.road_id, wp.lane_id, meta[wp.road_id][1])


def rank_crossings(road, fcs, cmap, meta, buildings, lane, approach, prop, top=10):
    rows = []
    for start, crossing in crossing_candidates(road, fcs, cmap, meta, lane, approach):
        rep = crossing_report(road, cmap, meta, buildings, start, crossing, lane, prop)
        score = len(rep["bad"]) * 100 + rep["last_turn"] + 0.3 * rep["turn"] - (20 if rep["crosswalk"] else 0)
        rows.append((score, start, crossing, rep))
    rows.sort(key=lambda r: r[0])
    marked = [r for r in rows[top:] if r[3]["crosswalk"]]
    print("\n== crossing candidates: lane {}, approach {:g} m ({} found) ==".format(lane, approach, len(rows)))
    for i, (score, start, crossing, rep) in enumerate(rows[:top] + marked):
        print("{:2d} {} -> crossing ({:.1f}, {:.1f}) cw={} walk {:.1f} m turn {:.0f}/{:.0f}/{:.0f} dz {:.2f} crowd {} cyc {} "
              "waits {} bldg {:.1f} m {}".format(
                  i, describe(meta, start), crossing.wp.transform.location.x, crossing.wp.transform.location.y,
                  rep["crosswalk"], rep["walk"], rep["turn"], rep["last_turn"], rep["after_turn"], rep["dz"],
                  rep["crowd"], rep["cyclists"], rep["waits"], rep["clear"], "BAD: " + "; ".join(rep["bad"]) if rep["bad"] else "ok"))
    return rows


# --- cut-in spots ------------------------------------------------------------------------------------------------------

def kerb_lane_of(wp):
    """The same-direction Driving lane right of `wp` when it is the rightmost driving lane, else None."""
    side = wp.get_right_lane()
    if side is None or side.lane_type != carla.LaneType.Driving or (side.lane_id > 0) != (wp.lane_id > 0):
        return None
    outer = side.get_right_lane()
    if outer is not None and outer.lane_type == carla.LaneType.Driving:
        return None
    return side


def cut_in_report(road, cmap, meta, buildings, start, kind, distance, end):
    polys, tree = buildings
    bad = []
    need = max(distance + 30.0, end + 10.0)
    entry, free = road.junction_entry_ahead(start, max_distance=400.0)
    if entry is None and free <= 400.0:
        bad.append("lane ends after {:.0f} m".format(free))
    elif free < need:
        bad.append("junction after {:.0f} m (need {:.0f})".format(free, need))
    wps = along(road, start, need)
    turn, dz, classes, elevated, junction = profile(meta, wps)
    if classes - URBAN:
        bad.append("on " + ",".join(sorted(classes - URBAN)))
    if elevated or dz > 0.5:
        bad.append("elevated/ramp (dz {:.1f})".format(dz))
    if turn > 12.0:
        bad.append("turn {:.0f} deg".format(turn))
    point = road.lane_point(start, distance)
    side = kerb_lane_of(point)
    if side is None:
        bad.append("no kerb lane right of the ego lane at the cut-in")
    actors = []
    if side is not None:
        if kind == "cyclist":
            hb, hc, hv = HALF["bike"][0], HALF["car"][0], HALF["firetruck"][0]
            actors = [("cyclist", side, HALF["bike"]), ("car A", road.lane_point(side, hb + 3.0 + hc), HALF["car"]),
                      ("blocker", road.lane_point(side, -(hb + 1.5 + hv)), HALF["firetruck"])]
            if str(side.lane_change) not in ("Left", "Both"):
                bad.append("kerb lane forbids a change to the left")
        else:
            hc, hv = HALF["car"][0], HALF["sprinter"][0]
            actors = [("car", side, HALF["car"]), ("van", road.lane_point(side, -(hc + 1.5 + hv)), HALF["sprinter"])]
        ahead = [road.lane_point(start, distance + d) for d in (-15.0, 0.0, 10.0, 20.0)]
        if any(w.is_junction or kerb_lane_of(w) is None for w in ahead):
            bad.append("kerb lane does not run along the cut-in")
    sidewalk = side is not None and road.find_sidewalk(side, "right") is not None
    edge = min(start.transform.location.x, 2000 - start.transform.location.x,
               -start.transform.location.y, 2000 + start.transform.location.y)
    if edge < 100.0:
        bad.append("{:.0f} m from the map edge".format(edge))
    return dict(bad=bad, free=free, turn=turn, dz=dz, classes=classes,
                actors=actors, sidewalk=sidewalk, edge=edge)


def rank_cut_ins(road, cmap, meta, buildings, kind, distance, end, top=14):
    """Ego starts in the inner lane (the lane left of the kerb lane), 10 m after the junction exit of each such lane."""
    rows, seen = [], set()
    for wp in cmap.generate_waypoints(2.0):
        if wp.is_junction or kerb_lane_of(wp) is None or (wp.road_id, wp.lane_id) in seen:
            continue
        prev = wp.previous(2.0)
        if prev and not prev[0].is_junction and (prev[0].road_id, prev[0].lane_id) == (wp.road_id, wp.lane_id):
            continue  # not the first waypoint of this lane: keep only lane starts
        seen.add((wp.road_id, wp.lane_id))
        start = road.lane_point(wp, 10.0)
        if start.is_junction:
            continue
        rep = cut_in_report(road, cmap, meta, buildings, start, kind, distance, end)
        score = len(rep["bad"]) * 100 + rep["turn"] - min(rep["free"], 300.0) / 10.0 - (5 if rep["sidewalk"] else 0)
        rows.append((score, start, rep))
    rows.sort(key=lambda r: r[0])
    print("\n== cut_in {} candidates (ego in the inner lane; {} lane starts) ==".format(kind, len(rows)))
    for i, (score, start, rep) in enumerate(rows[:top]):
        print("{:2d} {} free {:.0f} m turn {:.0f} dz {:.2f} sidewalk {} edge {:.0f} m {}".format(
            i, describe(meta, start), rep["free"], rep["turn"], rep["dz"], rep["sidewalk"], rep["edge"],
            "BAD: " + "; ".join(rep["bad"]) if rep["bad"] else "ok"))
    return rows


# --- highway merges ----------------------------------------------------------------------------------------------------

def merges(xodr):
    """On-ramp merges: junctions where a *_link and a motorway/trunk road both lead into one road."""
    root = ET.fromstring(xodr)
    roads = {r.get("id"): r for r in root.findall("road")}

    def cls(rid):
        return roads[rid].get("name").split("|")[0]

    def lanes(rid):
        sec = roads[rid].findall("lanes/laneSection")[0]
        return sum(1 for lane in sec.findall("right/lane") if lane.get("type") == "driving")

    found = []
    for j in root.findall("junction"):
        outs = {}
        for c in j.findall("connection"):
            inc, conn = c.get("incomingRoad"), roads[c.get("connectingRoad")]
            for end in (conn.find("link/successor"), conn.find("link/predecessor")):
                if end is not None and end.get("elementType") == "road" and end.get("elementId") != inc:
                    outs.setdefault(end.get("elementId"), set()).add(inc)
        for out, ins in outs.items():
            kinds = [cls(i) for i in ins]
            if (len(ins) >= 2 and any(k in ("motorway", "trunk") for k in kinds)
                    and any(k.endswith("_link") for k in kinds) and cls(out) in ("motorway", "trunk")):
                found.append((j.get("id"), out, cls(out), lanes(out), float(roads[out].get("length")),
                              sorted((i, cls(i), lanes(i)) for i in ins)))
    return found


# --- plots -------------------------------------------------------------------------------------------------------------

def plot_spot(road, cmap, buildings, title, path, ego_wp, route, items, crossing=None, crosswalk_map=None, radius=None):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Circle, Polygon as MPoly

    xs = [w.transform.location.x for w in route] + [x for _, pts, _ in items for x, _ in pts]
    ys = [w.transform.location.y for w in route] + [y for _, pts, _ in items for _, y in pts]
    x0, x1, y0, y1 = min(xs) - 25, max(xs) + 25, min(ys) - 25, max(ys) + 25
    side = max(x1 - x0, y1 - y0)
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    x0, x1, y0, y1 = cx - side / 2, cx + side / 2, cy - side / 2, cy + side / 2

    def inside(loc):
        return x0 - 10 <= loc.x <= x1 + 10 and y0 - 10 <= loc.y <= y1 + 10

    driving, junction, sidewalk = [], [], []
    for wp in cmap.generate_waypoints(1.0):
        if not inside(wp.transform.location):
            continue
        lanes = [wp]
        cur = wp
        for _ in range(4):
            cur = cur.get_right_lane()
            if cur is None:
                break
            if cur.lane_type == carla.LaneType.Sidewalk:
                lanes.append(cur)
        for lane in lanes:
            t = lane.transform
            quad = box(t.location.x, t.location.y, t.rotation.yaw, 0.55, lane.lane_width / 2.0)
            (sidewalk if lane.lane_type == carla.LaneType.Sidewalk else junction if lane.is_junction else driving).append(quad)

    fig, ax = plt.subplots(figsize=(10, 10), dpi=120)
    ax.add_collection(PolyCollection(sidewalk, facecolors="#d9c9a3", edgecolors="none", zorder=1))
    ax.add_collection(PolyCollection(driving, facecolors="#9a9a9a", edgecolors="none", zorder=1))
    ax.add_collection(PolyCollection(junction, facecolors="#6f6f6f", edgecolors="none", zorder=1))
    polys, _ = buildings
    for poly in polys:
        bx0, by0, bx1, by1 = poly.bounds
        if bx1 < x0 or bx0 > x1 or by1 < y0 or by0 > y1:
            continue
        geoms = getattr(poly, "geoms", [poly])
        for g in geoms:
            ax.add_patch(MPoly(list(g.exterior.coords), closed=True, fc="#c9a0a0", ec="#7a4b4b", lw=0.6, zorder=2))
    if crosswalk_map is not None:
        for cw in road.crosswalks(crosswalk_map):
            ax.add_patch(MPoly([(p.x, p.y) for p in cw.points], closed=True, fc="white", ec="black", lw=0.8, zorder=3))
    ax.plot([w.transform.location.x for w in route], [w.transform.location.y for w in route], "-", c="#1f77b4",
            lw=1.5, zorder=4, label="ego route")
    t = ego_wp.transform
    f = t.get_forward_vector()
    ax.annotate("", xy=(t.location.x + 8 * f.x, t.location.y + 8 * f.y), xytext=(t.location.x, t.location.y),
                arrowprops=dict(arrowstyle="-|>", color="#1f77b4", lw=2.5), zorder=6)
    ax.add_patch(MPoly(box(t.location.x, t.location.y, t.rotation.yaw, 2.4, 1.0), closed=True, fc="#1f77b4", ec="k",
                       zorder=6, label="ego start"))
    colors = iter(["#d62728", "#2ca02c", "#ff7f0e", "#9467bd", "#8c564b", "#e377c2", "#17becf", "#bcbd22", "#7f7f7f"])
    for label, pts, style in items:
        c = next(colors)
        if style == "box":
            for i in range(0, len(pts), 4):
                ax.add_patch(MPoly(pts[i:i + 4], closed=True, fc=c, ec="k", lw=0.6, zorder=5,
                                   label=label if i == 0 else None))
        elif style == "line":
            ax.plot([p[0] for p in pts], [p[1] for p in pts], "-", c=c, lw=2, zorder=5, label=label)
        else:
            ax.scatter([p[0] for p in pts], [p[1] for p in pts], s=14, c=c, zorder=5, label=label)
    if crossing is not None and radius:
        loc = crossing
        ax.add_patch(Circle((loc[0], loc[1]), radius, fill=False, ls="--", ec="#555555", zorder=2))
    ax.set_xlim(x0, x1)
    ax.set_ylim(y1, y0)  # CARLA's y axis points south: north up
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x [m] (east)")
    ax.set_ylabel("y [m] (CARLA y points south; north is up)")
    ax.legend(loc="upper right", fontsize=7, framealpha=0.9)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(path), bbox_inches="tight")
    plt.close(fig)


# --- the chosen spots --------------------------------------------------------------------------------------------------

def ego_pose(meta, wp):
    t = wp.transform
    return dict(x=round(t.location.x, 2), y=round(t.location.y, 2), z=round(t.location.z + EGO_LIFT, 2),
                yaw=round(t.rotation.yaw, 2), road=wp.road_id, lane=wp.lane_id, name=meta[wp.road_id][1])


def validate_crossing(road, cmaps, meta, buildings, name, choice, plots):
    out = {}
    for label, cmap in cmaps.items():
        x, y = choice["ego"][:2]
        start = cmap.get_waypoint(carla.Location(x=x, y=y, z=0.0))  # what ComposedScenario does with the ego start
        index = road.CrosswalkIndex(road.crosswalks(cmap))
        crossing = road.find_crossing(cmap, start, index)
        if crossing is None:
            print("  [{}] {}: NO CROSSING".format(label, name))
            continue
        rep = crossing_report(road, cmap, meta, buildings, start, crossing, choice["lane"], choice.get("prop", False))
        cw = crossing.wp.transform.location
        print("  [{}] {}: ego {} | crossing ({:.2f}, {:.2f}) {:.1f} m ahead, crosswalk {}, walk {:.1f} m, "
              "crowd {}/14, cyclists {}/2 (waiting spots {}), building clearance {:.1f} m, turn {:.0f}/{:.0f}/{:.0f} deg, "
              "{}".format(label, name, ego_pose(meta, start), cw.x, cw.y, crossing.distance, rep["crosswalk"],
                          rep["walk"], rep["crowd"], rep["cyclists"], rep["waits"], rep["clear"], rep["turn"],
                          rep["last_turn"], rep["after_turn"], "BAD: " + "; ".join(rep["bad"]) if rep["bad"] else "ok"))
        s, e = crossing.spawn.location, crossing.end
        print("      pedestrian start ({:.2f}, {:.2f}, {:.2f}) end ({:.2f}, {:.2f}); end of run {:.0f} m from the start".format(
            s.x, s.y, s.z, e.x, e.y, crossing.distance + 30.0))
        over = overpasses(cmap, road.straight_route(start, crossing.distance + 30.0))
        print("      roads over/under the drive: {}".format(sorted(over) or "none"))
        out[label] = (start, crossing, rep)
        if not plots:
            continue
        route = road.straight_route(start, crossing.distance + 30.0 + 20.0)
        items = [("pedestrian walk", [(s.x, s.y), (e.x, e.y)], "line")]
        crowd = road.crowd_spawns(cmap, crossing, 14)
        if name.startswith("Pedestrian") or "Crowd" in name:
            items.append(("crowd starts", [(c.spawn.location.x, c.spawn.location.y) for c in crowd], "dots"))
            cyc = road.cyclist_spawns(cmap, crossing, 2)
            items.append(("cyclist rides", [p for c in cyc for p in ((c.spawn.location.x, c.spawn.location.y),
                                                                    (c.end.x, c.end.y))], "dots"))
            waits = [w[0] for w in (road.cyclist_waiting_spots(cmap, c, 1) for c in cyc) if w]
            items.append(("cyclists waiting", [(w.location.x, w.location.y) for w in waits], "dots"))
        if rep["occ"]:
            pts = []
            for tf, half in zip(rep["occ"], (HALF["european_hgv"], HALF["car"])):
                pts += box(tf.location.x, tf.location.y, tf.rotation.yaw, *half)
            items.append(("truck + queued car", pts, "box"))
        if choice.get("prop"):
            ptf = road.prop_transform(crossing, *HALF["busstop"])
            items.append(("bus stop", box(ptf.location.x, ptf.location.y, ptf.rotation.yaw, *HALF["busstop"]), "box"))
        items.append(("crossing point (trigger ref.)", [(cw.x, cw.y)], "dots"))
        end_wp = road.lane_point(start, crossing.distance + 30.0)
        items.append(("end of run", [(end_wp.transform.location.x, end_wp.transform.location.y)], "dots"))
        title = "{} [{} map]\nego {} ; crossing {:.0f} m ahead{}".format(
            name, label, describe(meta, start), crossing.distance,
            " at a painted crosswalk" if rep["crosswalk"] else " at the junction entry (no painted crosswalk)")
        plot_spot(road, cmap, buildings, title, HERE / "plots" / "{}_{}.png".format(name, label), start, route, items,
                  crosswalk_map=cmaps["closed"], crossing=(cw.x, cw.y), radius=None)
    return out


def validate_cut_in(road, cmaps, meta, buildings, name, choice, plots):
    cmap = cmaps["raw"]
    x, y = choice["ego"][:2]
    start = cmap.get_waypoint(carla.Location(x=x, y=y, z=0.0))
    rep = cut_in_report(road, cmap, meta, buildings, start, choice["kind"], choice["distance"], choice["end"])
    print("  {}: ego {} | junction-free {:.0f} m, turn {:.0f} deg, dz {:.2f}, kerb sidewalk {}, {}".format(
        name, ego_pose(meta, start), rep["free"], rep["turn"], rep["dz"], rep["sidewalk"],
        "BAD: " + "; ".join(rep["bad"]) if rep["bad"] else "ok"))
    over = overpasses(cmap, road.straight_route(start, choice["end"]))
    print("      roads over/under the drive: {}".format(sorted(over) or "none"))
    for label, wp, half in rep["actors"]:
        t = wp.transform
        print("      {} at ({:.2f}, {:.2f}) road {} lane {}".format(label, t.location.x, t.location.y, wp.road_id,
                                                                   wp.lane_id))
    if plots:
        route = road.straight_route(start, choice["end"] + 20.0)
        items = []
        for label, wp, half in rep["actors"]:
            t = wp.transform
            push = (wp.lane_width - half[1]) / 4.0 if choice["kind"] == "parking" else 0.0
            r = t.get_right_vector()
            items.append((label, box(t.location.x + push * r.x, t.location.y + push * r.y, t.rotation.yaw, *half), "box"))
        trig = road.lane_point(start, choice["distance"] - (15.0 if choice["kind"] == "parking" else 14.0))
        items.append(("ego position at the trigger", [(trig.transform.location.x, trig.transform.location.y)], "dots"))
        end_wp = road.lane_point(start, choice["end"])
        items.append(("end of run", [(end_wp.transform.location.x, end_wp.transform.location.y)], "dots"))
        title = "{}\nego {} ; cut-in actor {:g} m ahead in the kerb lane".format(name, describe(meta, start),
                                                                                 choice["distance"])
        plot_spot(road, cmap, buildings, title, HERE / "plots" / "{}.png".format(name), start, route, items,
                  crosswalk_map=cmaps["closed"])
    return start, rep


def merge_proposal(road, cmap, meta, buildings, plots):
    """
    The highway cut-in proposal: the K 92 (trunk Braunschweiger Strasse) on-ramp at junction 219, where road 3011 (2
    lanes) and the trunk_link 3051 become road 3139 (3 lanes, 94 m); its lane -3 is the acceleration lane, which ends
    (no successor). Ego in lane -2 (the right main lane), the merging car in lane -3.
    """
    merge_main = cmap.get_waypoint(carla.Location(x=1113.0, y=-1290.0))  # road 3139 lane -2 at its start
    accel = merge_main.get_right_lane()  # road 3139 lane -3
    # ego 40 m before the merge, car 5 m into the acceleration lane (cut_in distance 45): at 70 km/h against 12.5 m/s the
    # 25 m trigger fires after about 2.9 s, with the car about 41 m into the 94 m lane
    ego = [w for w in merge_main.previous(40.0) if w.road_id == 3011][0]
    lane_len = 0.0
    w = accel
    while True:
        nxt = w.next(2.0)
        if not nxt or nxt[0].road_id != accel.road_id:
            break
        w, lane_len = nxt[0], lane_len + 2.0
    print("\n== highway merge proposal (K 92 on-ramp, junction 219) ==")
    print("  ego start {} ; 40 m to the merge".format(ego_pose(meta, ego)))
    print("  acceleration lane road {} lane {}: about {:.0f} m, then it ends; its lane change permission: {}".format(
        accel.road_id, accel.lane_id, lane_len, accel.lane_change))
    ramp = accel.previous(60.0)
    ramp_wps = along(road, ramp[0], 60.0) if ramp else []
    print("  ramp: {}".format(sorted({(w.road_id, meta[w.road_id][1]) for w in ramp_wps})))
    if not plots:
        return
    route = road.straight_route(ego, 200.0)
    items = [("on-ramp and acceleration lane", [(w.transform.location.x, w.transform.location.y) for w in
                                                ramp_wps + along(road, accel, lane_len)], "line"),
             ("merging car, 5 m into the acceleration lane", box(
                 road.lane_point(accel, 5.0).transform.location.x, road.lane_point(accel, 5.0).transform.location.y,
                 accel.transform.rotation.yaw, *HALF["car"]), "box")]
    plot_spot(road, cmap, buildings, "Highway cut-in proposal: K 92 on-ramp merge (junction 219)\nego {}".format(
        describe(meta, ego)), HERE / "plots" / "HighwayMerge_K92_proposal.png", ego, route, items)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--repo", default=str(HERE.parents[1]))
    parser.add_argument("--xodr", default=str(HERE.parents[2] / "car_simulator_inputs" / "WolfsburgHighway.xodr"))
    parser.add_argument("--keepout", default=None)
    parser.add_argument("--search", action="store_true")
    parser.add_argument("--no-plots", action="store_true")
    args = parser.parse_args()
    keepout = args.keepout or str(Path(args.xodr).with_suffix(".keepout"))

    road, fcs = import_repo(args.repo)
    xodr = Path(args.xodr).read_text()
    meta = road_meta(xodr)
    cmaps = {"raw": carla.Map("WolfsburgHighway", xodr), "closed": carla.Map("WolfsburgHighway", close_crosswalks(xodr))}
    buildings = load_buildings(keepout)
    print("crosswalk polygons: raw {}, closed {}; buildings {}".format(
        len(road.crosswalks(cmaps["raw"])), len(road.crosswalks(cmaps["closed"])), len(buildings[0])))

    # Self-check of the proposed fix: on the raw map it must find what the closed outlines give
    fixed = sorted((round(c.center.x, 1), round(c.center.y, 1)) for c in crosswalks_open_outlines(road, cmaps["raw"]))
    closed = sorted((round(c.center.x, 1), round(c.center.y, 1)) for c in road.crosswalks(cmaps["closed"]))
    assert fixed == closed, (fixed, closed)
    print("proposed crosswalks() fix on the raw map: {} polygons, the same as the closed outlines".format(len(fixed)))

    if args.search:
        for label in ("raw", "closed"):
            print("\n######## {} map ########".format(label))
            rank_crossings(road, fcs, cmaps[label], meta, buildings, "kerb", 120.0, False)
            rank_crossings(road, fcs, cmaps[label], meta, buildings, "inner", 90.0, True)
        rank_cut_ins(road, cmaps["raw"], meta, buildings, "cyclist", 40.0, 90.0)
        rank_cut_ins(road, cmaps["raw"], meta, buildings, "parking", 66.0, 100.0)
        print("\n== on-ramp merges (junction, out road, class, lanes, length, incoming) ==")
        for row in merges(xodr):
            print("  ", row)

    print("\n== chosen spots ==")
    for name, choice in list(CHOICES.items()) + list(BACKUPS.items()):
        if choice["ego"] is None:
            print("  {}: not chosen yet".format(name))
        elif choice["kind"] in ("crossing", "occluder"):
            validate_crossing(road, cmaps, meta, buildings, name, choice, not args.no_plots)
        else:
            validate_cut_in(road, cmaps, meta, buildings, name, choice, not args.no_plots)
    merge_proposal(road, cmaps["raw"], meta, buildings, not args.no_plots)


if __name__ == "__main__":
    main()
