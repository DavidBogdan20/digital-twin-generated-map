"""Gets a map's inputs ready for the Digital Twin Tool and checks them before the hours-long Generate.
Needs only the carla module, no simulator.

usage: python prepare_dt.py roads.xodr buildings.osm out/Name
writes out/Name.xodr (sidewalks narrowed), out/Name.osm (without buildings that reach onto a lane) and
out/Name.keepout (the remaining building outlines, read by the patched tool: no tree or street light inside or
near one). Then it replays where the patched tool puts trees (PlaceSpot in apply_patches.py) and counts them.
"""
import math
import re
import sys
import xml.etree.ElementTree as ET

import carla

SIDEWALK_W = 2.0                    # m; netconvert's 2.8 m sidewalks reached into real buildings
EVERY, BEYOND = 50.0, 3.0           # m: the tool's tree spacing and distance past the last driving lane
# m, as in PlaceSpot (apply_patches.py): tree crowns collide and are up to 8.2 m wide, so spots move this far from
# driving lanes; then they must be clear of any other lane (sidewalks) and of building walls
CLEAR_ROAD, CLEAR_LANE, CLEAR_WALL = 4.5, 1.0, 3.0


def tmerc(lat0, lon0):  # same series as load_osm.py
    a, e2 = 6378137.0, 0.00669437999014

    def f(lat, lon):
        p, p0 = math.radians(lat), math.radians(lat0)
        A = math.radians(lon - lon0) * math.cos(p)
        N = a / math.sqrt(1 - e2 * math.sin(p) ** 2)
        M = a * (1 - e2) / (1 - e2 * math.sin((p + p0) / 2) ** 2) ** 1.5
        return N * A, M * (p - p0) + N * math.tan(p) * A * A / 2
    return f


def narrow_sidewalks(xodr):
    lane = re.compile(r'(<lane id="-?\d+" type="sidewalk".*?</lane>)', re.S)
    const = re.compile(r'a="2\.80*"( b="0(?:\.0*)?" c="0(?:\.0*)?" d="0(?:\.0*)?")')  # constant width only
    return lane.sub(lambda m: const.sub(lambda w: f'a="{SIDEWALK_W:.2f}"' + w.group(1), m.group(1)), xodr)


def sidewalks_on_roads(m, xodr):
    """(road id, lane section index, lane id) of each sidewalk lane that lies on another road's driving lane. The tool
    builds every sidewalk as a raised kerb (0.3 m), so where a ramp with a sidewalk merges, its sidewalk ran across the
    main road's lanes and stopped cars."""
    found = set()
    for road in ET.fromstring(xodr).iter("road"):
        secs = road.findall("lanes/laneSection")
        ends = [float(x.get("s")) for x in secs[1:]] + [float(road.get("length"))]
        for k, (sec, end) in enumerate(zip(secs, ends)):
            for lane in sec.iter("lane"):
                if lane.get("type") != "sidewalk":
                    continue
                s = float(sec.get("s")) + 0.1
                while s < end:
                    w = m.get_waypoint_xodr(int(road.get("id")), int(lane.get("id")), s)
                    if w is not None and overlaps_driving(m, w):
                        found.add((road.get("id"), k, lane.get("id")))
                        break
                    s += 1.0
    return found


def overlaps_driving(m, w, depth=0.2):
    """True when any point across the sidewalk lane at w lies over depth m inside another road's driving lane: centre,
    and both sides 0.2 m in from its edges (a sidewalk beside a parallel ramp overlaps it only along one edge)."""
    r, c = w.transform.get_right_vector(), w.transform.location
    for off in (0.0, w.lane_width / 2 - 0.2, -(w.lane_width / 2 - 0.2)):
        p = carla.Location(c.x + r.x * off, c.y + r.y * off, c.z)
        d = m.get_waypoint(p, project_to_road=False, lane_type=carla.LaneType.Driving)
        if d is not None and d.road_id != w.road_id and \
                math.hypot(d.transform.location.x - p.x, d.transform.location.y - p.y) < d.lane_width / 2 - depth:
            return True
    return False


def unpave(xodr, lanes):
    """The same lanes as type border with no width: no kerb, no walkway, no surface over the other road's lane (a
    border lane of full width still got a surface, 0.5 m over the lane it lay on). They are the outermost lanes, so
    nothing else moves."""
    def road(match):
        text = match.group(0)
        rid = re.search(r' id="([^"]+)"', text).group(1)
        parts = re.split(r'(<laneSection\b)', text)  # [head, tag, body, tag, body, ...]
        for k in range(len(parts) // 2):
            for _, sec, lid in (x for x in lanes if x[0] == rid and x[1] == k):
                lane = re.compile(r'<lane id="%s" type="sidewalk".*?</lane>' % lid, re.S)
                parts[2 + 2 * k] = lane.sub(lambda m: re.sub(r'(<width [^>]*?)a="[^"]*" b="[^"]*" c="[^"]*" d="[^"]*"',
                                                             r'\1a="0" b="0" c="0" d="0"', m.group(0))
                                            .replace('type="sidewalk"', 'type="border"', 1), parts[2 + 2 * k], count=1)
        return "".join(parts)
    return re.sub(r'<road\b.*?</road>', road, xodr, flags=re.S)


# Roads that climb to bridges outside the map: at the south edge both A 39 carriageways and a ramp rose to a 5 m bridge
# the crop cut off, and their stubs overlapped at up to 1.3 m apart. Every road around them is at ground level.
EDGE_STUBS = {"3110", "3192", "3191", "3165"}


def flatten(xodr, ids):
    """Those roads at height 0 all along (one flat elevation record)."""
    def swap(match):
        text = match.group(0)
        rid = re.search(r' id="([^"]+)"', text).group(1)
        if rid not in ids:
            return text
        return re.sub(r"<elevationProfile>.*?</elevationProfile>",
                      '<elevationProfile><elevation s="0" a="0" b="0" c="0" d="0" /></elevationProfile>', text, count=1, flags=re.S)
    return re.sub(r"<road\b.*?</road>", swap, xodr, flags=re.S)


def hermite_profile(ss, zs):
    """<elevationProfile> through the heights zs at the s values ss: one cubic per interval, continuous slope."""
    n = len(ss) - 1
    slopes = [(zs[min(i + 1, n)] - zs[max(i - 1, 0)]) / (ss[min(i + 1, n)] - ss[max(i - 1, 0)]) for i in range(n + 1)]
    records = []
    for i in range(n):
        h = ss[i + 1] - ss[i]
        z0, z1, m0, m1 = zs[i], zs[i + 1], slopes[i], slopes[i + 1]
        c = (3 * (z1 - z0) / h - 2 * m0 - m1) / h
        d = (m0 + m1 - 2 * (z1 - z0) / h) / (h * h)
        records.append('<elevation s="%.4f" a="%.4f" b="%.6f" c="%.8f" d="%.10f" />' % (ss[i], z0, m0, c, d))
    return "<elevationProfile>" + "".join(records) + "</elevationProfile>"


def replace_profiles(xodr, profiles):
    def swap(match):
        text = match.group(0)
        rid = re.search(r' id="([^"]+)"', text).group(1)
        if rid not in profiles:
            return text
        return re.sub(r"<elevationProfile>.*?</elevationProfile>", profiles[rid], text, count=1, flags=re.S)
    return re.sub(r"<road\b.*?</road>", swap, xodr, flags=re.S)


def blend_road_ends(m, xodr, near=10.0, blend=40.0, step=2.0):
    """
    Where roads enter or leave the same junction within `near` m of each other at heights more than 0.15 m apart (a
    diverge whose branches start at different heights), bring each of those ends to the group's mean height, blended
    into the road's own profile over `blend` m (at most half the road): the branches part at one height and climb or
    drop after. Their lanes overlapped at up to 0.43 m apart (junction 72).
    """
    root = ET.fromstring(xodr)
    roads = {r.get("id"): r for r in root.iter("road")}

    def centre(rid, s):
        for lane in (-1, 1):
            w = m.get_waypoint_xodr(int(rid), lane, s)
            if w is not None:
                return w.transform.location
        return None

    def heading(rid, s):
        for lane in (-1, 1):
            w = m.get_waypoint_xodr(int(rid), lane, s)
            if w is not None:
                return w.transform.rotation.yaw
        return None
    shifts = {}  # road id -> {"start": dz, "end": dz}
    for junction in root.iter("junction"):
        ends = {}
        for c in junction.findall("connection"):
            link = roads[c.get("connectingRoad")].find("link")
            for side in ("predecessor", "successor"):
                other = link.find(side) if link is not None else None
                if other is None or other.get("elementType") != "road" or other.get("elementId") not in roads:
                    continue
                oid = other.get("elementId")
                at = "start" if other.get("contactPoint") == "start" else "end"
                length = float(roads[oid].get("length"))
                p = centre(oid, 0.01 if at == "start" else max(0.01, length - 0.01))
                if p is not None:  # side: entering the junction (predecessor) or leaving it
                    ends[(oid, at)] = (p, side, heading(oid, 0.01 if at == "start" else max(0.01, length - 0.01)))
        roles = {k: v[1] for k, v in ends.items()}
        yaws = {k: v[2] for k, v in ends.items()}
        ends = {k: v[0] for k, v in ends.items()}

        def parallel(a, b):  # side by side means running the same way, within 30 degrees
            return yaws[a] is not None and yaws[b] is not None and abs((yaws[a] - yaws[b] + 180) % 360 - 180) < 30
        keys = list(ends)
        groups = {k: {k} for k in keys}
        for i, a in enumerate(keys):
            for b in keys[i + 1:]:
                pa, pb = ends[a], ends[b]
                # both entering or both leaving (side by side); one in, one out is a way through, which may climb
                if a[0] != b[0] and roles[a] == roles[b] and parallel(a, b) and math.hypot(pa.x - pb.x, pa.y - pb.y) < near \
                        and abs(pa.z - pb.z) > 0.15:
                    merged = groups[a] | groups[b]
                    for k in merged:
                        groups[k] = merged
        for group in {frozenset(g) for g in groups.values() if len(g) > 1}:
            mean = sum(ends[k].z for k in group) / len(group)
            for rid, at in group:
                shifts.setdefault(rid, {})[at] = mean - ends[(rid, at)].z
    profiles = {}
    for rid, shift in shifts.items():
        length = float(roads[rid].get("length"))
        span = min(blend, length / 2)
        n = max(1, int(math.ceil(length / step)))
        ss = [length * i / n for i in range(n + 1)]
        zs = []
        for si in ss:
            p = centre(rid, min(max(si, 0.01), length - 0.01))
            if p is None:
                break
            z = p.z
            for at, dz in shift.items():
                t = min(1.0, (si if at == "start" else length - si) / span)
                z += dz * (1 - t * t * (3 - 2 * t))  # smoothstep: full shift at the end, none after `span` m
            zs.append(z)
        if len(zs) == len(ss):
            profiles[rid] = hermite_profile(ss, zs)
    return replace_profiles(xodr, profiles), len(profiles)


def level_junctions(m, xodr, step=2.0):
    """
    Give every connecting road of each junction its heights from one height field of that junction: inverse-distance
    weighting of the heights at the ends of all its connecting roads (their original heights, so nothing changes where
    they meet the roads around the junction). netconvert gives each connecting road its own straight ramp, so near
    interchanges two of them overlapped in plan at heights up to 0.7 m apart, and one deck's edge stood in the other's
    lane (cars stopped dead at (1185, -1505), 2026-11-10). Lanes over the same spot now get the same height, give or
    take the slope across a lane. Each road gets one cubic elevation record per `step` m (Hermite, continuous slope).
    """
    root = ET.fromstring(xodr)
    roads = {r.get("id"): r for r in root.iter("road")}

    def centre(rid, s):  # the centre of the road's first lane on either side at s, or None
        for lane in (-1, 1):
            w = m.get_waypoint_xodr(int(rid), lane, s)
            if w is not None:
                return w.transform.location
        return None

    grid = {}  # driving lane points of the roads outside junctions, by 5 m cell
    for w in m.generate_waypoints(1.0):
        if not w.is_junction:
            loc = w.transform.location
            for i in (-1, 0, 1):
                for j in (-1, 0, 1):
                    grid.setdefault((int(loc.x // 5) + i, int(loc.y // 5) + j), []).append(w)
    changed = {}
    for junction in root.iter("junction"):
        ids = sorted({c.get("connectingRoad") for c in junction.findall("connection")})
        ends = []  # (x, y, z) at both ends of every connecting road
        inside = []  # points of the connecting roads, to find other roads lying over them
        for rid in ids:
            length = float(roads[rid].get("length"))
            link = roads[rid].find("link")
            for s, side in ((0.01, "predecessor"), (max(0.01, length - 0.01), "successor")):
                p = centre(rid, s)
                if p is None:
                    continue
                z = p.z
                other = link.find(side) if link is not None else None
                if other is not None and other.get("elementType") == "road" and other.get("elementId") in roads:
                    oid = other.get("elementId")  # the height of the road it joins there (it may have been flattened)
                    olen = float(roads[oid].get("length"))
                    q = centre(oid, 0.01 if other.get("contactPoint") == "start" else max(0.01, olen - 0.01))
                    z = q.z if q is not None else z
                ends.append((p.x, p.y, z))
            s = 1.0
            while s < length - 1.0:
                p = centre(rid, s)
                if p is not None:
                    inside.append(p)
                s += 2.0
        if len(ends) < 2:
            continue
        # Anchors: roads outside the junction whose lanes lie over its connecting roads (where it starts or where a
        # ramp runs alongside): the field takes their heights there, so the overlapping lanes meet at one height
        for p in inside:
            for w in grid.get((int(p.x // 5), int(p.y // 5)), []):
                c = w.transform.location
                if math.hypot(c.x - p.x, c.y - p.y) < w.lane_width / 2:
                    ends.append((c.x, c.y, c.z))

        def height(x, y):
            num = den = 0.0
            for ex, ey, ez in ends:
                d2 = (ex - x) ** 2 + (ey - y) ** 2
                if d2 < 1e-6:
                    return ez
                num, den = num + ez / d2, den + 1.0 / d2
            return num / den
        for rid in ids:
            length = float(roads[rid].get("length"))
            n = max(1, int(math.ceil(length / step)))
            ss = [length * i / n for i in range(n + 1)]
            pts = [centre(rid, min(max(si, 0.01), length - 0.01)) for si in ss]
            if any(p is None for p in pts):
                continue
            zs = [height(p.x, p.y) for p in pts]
            slopes = [(zs[min(i + 1, n)] - zs[max(i - 1, 0)]) / (ss[min(i + 1, n)] - ss[max(i - 1, 0)]) for i in range(n + 1)]
            records = []
            for i in range(n):
                h = ss[i + 1] - ss[i]
                z0, z1, m0, m1 = zs[i], zs[i + 1], slopes[i], slopes[i + 1]
                c = (3 * (z1 - z0) / h - 2 * m0 - m1) / h
                d = (m0 + m1 - 2 * (z1 - z0) / h) / (h * h)
                records.append('<elevation s="%.4f" a="%.4f" b="%.6f" c="%.8f" d="%.10f" />' % (ss[i], z0, m0, c, d))
            changed[rid] = "<elevationProfile>" + "".join(records) + "</elevationProfile>"

    def swap(match):
        text = match.group(0)
        rid = re.search(r' id="([^"]+)"', text).group(1)
        if rid not in changed:
            return text
        return re.sub(r"<elevationProfile>.*?</elevationProfile>", changed[rid], text, count=1, flags=re.S)
    return re.sub(r"<road\b.*?</road>", swap, xodr, flags=re.S), len(changed)


def lane_gap(m, x, y, kind=carla.LaneType.Any, centre=False):
    """m from (x, y) to the nearest edge of a lane of that kind, negative inside one (and the lane's centre)."""
    w = m.get_waypoint(carla.Location(x, y, 0), project_to_road=True, lane_type=kind)
    if w is None:
        return (1e9, None) if centre else 1e9
    c = w.transform.location
    gap = math.hypot(c.x - x, c.y - y) - w.lane_width / 2
    return (gap, c) if centre else gap


def place_spot(m, x, y):
    """PlaceSpot without the tile and building tests: the spot moved CLEAR_ROAD from driving lanes, or None."""
    for _ in range(3):
        gap, c = lane_gap(m, x, y, carla.LaneType.Driving, centre=True)
        if gap >= CLEAR_ROAD:
            break
        d = math.hypot(x - c.x, y - c.y)
        if d < 0.01:
            return None
        k = (d - gap + CLEAR_ROAD + 0.05) / d
        x, y = c.x + (x - c.x) * k, c.y + (y - c.y) * k
    if lane_gap(m, x, y, carla.LaneType.Driving) < CLEAR_ROAD or lane_gap(m, x, y) < CLEAR_LANE:
        return None
    return x, y


def tool_spots(m, xodr):
    """Where Map::GetTreesTransform puts trees: BEYOND m past the outermost right driving lane, every EVERY m."""
    for road in ET.fromstring(xodr).iter("road"):
        if road.get("junction") != "-1":
            continue
        secs = road.findall("lanes/laneSection")
        ends = [float(s.get("s")) for s in secs[1:]] + [float(road.get("length"))]
        for sec, end in zip(secs, ends):
            ids = [int(l.get("id")) for l in sec.iter("lane") if int(l.get("id")) < 0 and l.get("type") == "driving"]
            s = float(sec.get("s"))
            while ids and s < end:
                w = m.get_waypoint_xodr(int(road.get("id")), min(ids), s)
                if w is not None and w.lane_width > 0:
                    r, c, d = w.transform.get_right_vector(), w.transform.location, w.lane_width / 2 + BEYOND
                    yield c.x + r.x * d, c.y + r.y * d
                s += EVERY


def wall_gap(x, y, poly):
    """m from (x, y) to the outline, negative inside it."""
    inside, best = False, 1e9
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
        dx, dy = x2 - x1, y2 - y1
        t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy or 1)))
        best = min(best, math.hypot(x - x1 - t * dx, y - y1 - t * dy))
    return -best if inside else best


def main(xodr_in, osm_in, out):
    xodr = flatten(narrow_sidewalks(open(xodr_in).read()), EDGE_STUBS)
    m = carla.Map("prepare", xodr)
    xodr, blended = blend_road_ends(m, xodr)
    m = carla.Map("prepare", xodr)
    print(f"road ends blended to a shared height where they meet a junction side by side: {blended}")
    xodr, levelled = level_junctions(m, xodr)
    m = carla.Map("prepare", xodr)
    print(f"junction connecting roads given one height field per junction: {levelled}")
    kerbs = sidewalks_on_roads(m, xodr)
    xodr = unpave(xodr, kerbs)
    m = carla.Map("prepare", xodr)
    left = sidewalks_on_roads(m, xodr)
    print(f"sidewalks on other roads' lanes: {len(kerbs)} lanes made border, {len(left)} left")
    assert not left, left
    tree = ET.parse(osm_in)
    root = tree.getroot()
    b = root.find("bounds")
    f = tmerc(float(b.get("minlat")), float(b.get("minlon")))
    xy = {}
    for n in root.iter("node"):
        x, y = f(float(n.get("lat")), float(n.get("lon")))
        xy[n.get("id")] = (x, -y)  # CARLA's y points south
    houses, dropped = {}, 0
    for w in list(root.iter("way")):
        if not any(t.get("k") == "building" for t in w.findall("tag")):
            continue
        poly = [xy[nd.get("ref")] for nd in w.findall("nd")][:-1]
        edge = [(x1 + (x2 - x1) * i / 4, y1 + (y2 - y1) * i / 4)
                for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]) for i in range(4)]
        if any(lane_gap(m, x, y) < 0 for x, y in edge):  # ponytail: 1/4-side samples, misses a corner clipping a lane by < 1 m
            root.remove(w)
            dropped += 1
        else:
            houses[w.get("id")] = poly
    spots = list(tool_spots(m, xodr))
    moved = [p for p in (place_spot(m, x, y) for x, y in spots) if p]
    kept = [(x, y) for x, y in moved
            if all(wall_gap(x, y, p) >= CLEAR_WALL for p in houses.values() if abs(p[0][0] - x) < 400 and abs(p[0][1] - y) < 400)]
    bad_old = sum(lane_gap(m, x, y, carla.LaneType.Driving) < CLEAR_ROAD for x, y in spots)
    print(f"buildings: {len(houses)} kept, {dropped} dropped for reaching onto a lane")
    print(f"tree spots: stock tool {len(spots)}, {bad_old} of them within {CLEAR_ROAD} m of a driving lane, where a large "
          f"crown reaches the lane; patched tool keeps {len(kept)}")
    open(out + ".xodr", "w").write(xodr)
    tree.write(out + ".osm", encoding="utf-8", xml_declaration=True)
    with open(out + ".keepout", "w") as k:
        for poly in houses.values():
            k.write(" ".join(f"{v:.2f}" for p in poly for v in p) + "\n")
    print("wrote", out + ".xodr", out + ".osm", out + ".keepout")
    assert all(lane_gap(m, x, y, carla.LaneType.Driving) >= CLEAR_ROAD and lane_gap(m, x, y) >= CLEAR_LANE for x, y in kept)


if __name__ == "__main__":
    main(*sys.argv[1:4])
