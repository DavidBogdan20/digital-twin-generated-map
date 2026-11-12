"""Convert an .osm to OpenDRIVE and load it into a running CARLA server, with bridges
raised, speed-limit / stop / give-way signs, 2+2-lane main roads and painted lane lines.

usage: python load_osm.py map.osm [host]
"""
import collections
import itertools
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
import carla

LAYER_HEIGHT = 5.0  # m per OSM layer = bridge deck height. Higher = steeper ramps (netconvert builds them)
STEP, NEAR = 25.0, 200.0  # m: node spacing on roads within NEAR of a bridge (split_near_bridges)
WAY_TYPES = ["motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary",
             "secondary_link", "tertiary", "tertiary_link", "unclassified", "residential"]  # imported roads
# Lane lines follow German rules: dashed (6 m, 12 m gap) between lanes of one direction and on
# the centre of 1+1 roads, double solid on the centre of wider two-way roads, solid edges on
# motorways, trunk roads and roundabout rings.
PAINTED = {"motorway", "trunk", "primary", "secondary", "tertiary", "unclassified", "residential"}  # and _link roads
# Edge lines too; town roads only get centre and lane lines. Edging every road would be 4x the
# strips (~30k), and every strip is sent to every client each frame.
EDGED = {"motorway", "trunk", "roundabout"}
WIDEN = {"primary", "secondary", "tertiary"}  # two-way ones get 2 lanes in each direction
DASH, GAP = 6, 12           # m
# Road hierarchy for give-way signs; link roads rank below all (priority_signs).
RANK = {"roundabout": 9, "motorway": 8, "trunk": 7, "primary": 6, "secondary": 5, "tertiary": 4,
        "unclassified": 3, "residential": 2}
# Lines are CARLA's own road paint mesh (a 15.5 m worn white strip from Town03), so they are lit and
# shaded like the road. Debug lines can't look like paint: the shipping build multiplies their
# colour by 1000 (DebugShapeDrawer.cpp), so even Color(1,1,1) renders as glowing white.
LINE_MESH = "/Game/Carla/Static/RoadLine/Town03LaneMarkings/Town03_MarkingNode_37.Town03_MarkingNode_37"
PAINT_W = 0.122  # m: painted width of LINE_MESH at scale 1, measured on camera (its bounding box is wider)
# m: as wide as two side-by-side strips of a 6 m dash, so dashes and solid lines match. German lines
# are 12-15 cm; that would cost a third strip per dash.
LINE_W = 0.095
LIFT = 0.03  # m above the road, so the paint doesn't z-fight with it
RAISED = 1.0  # m: a road higher than this is on a bridge or its ramp
BEAM = "/Game/Carla/Static/Bridge/SM_Beam01.SM_Beam01"  # concrete beam, BEAM_L x BEAM_W/BEAM_S x 2 m
BEAM_L, BEAM_S = 30.0, 0.5  # at 0.5: 15 m long, 1 m tall (half parapet above the deck, half its edge)
BEAM_W = 0.96 * BEAM_S
PILLAR = "/Game/Carla/Static/Bridge/SM_BridgePilar.SM_BridgePilar"  # round column
PILLAR_H, PILLAR_W = 9.0, 4.0  # m at scale 1


def tmerc(lat0, lon0):
    """lat/lon -> OpenDRIVE x/y for the proj string below (+proj=tmerc on WGS84, k=1).
    Series to 2nd order in longitude: well under 1 cm off over a few km."""
    a, e2 = 6378137.0, 0.00669437999014

    def f(lat, lon):
        p, p0 = math.radians(lat), math.radians(lat0)
        A = math.radians(lon - lon0) * math.cos(p)
        N = a / math.sqrt(1 - e2 * math.sin(p) ** 2)
        M = a * (1 - e2) / (1 - e2 * math.sin((p + p0) / 2) ** 2) ** 1.5  # meridian radius, mid-latitude
        return N * A, M * (p - p0) + N * math.tan(p) * A * A / 2
    return f


def prepare_osm(root):
    """Edit the OSM before conversion: carry each way's highway class ("roundabout" for rings) into
    its name (netconvert copies names into the xodr; nothing else of the class survives), make
    two-way main roads 2+2, and drop underpass layers."""
    ring_nodes = {nd.get("ref") for way in root.iter("way")
                  if any(t.get("k") == "junction" and t.get("v") == "roundabout" for t in way.findall("tag"))
                  for nd in way.findall("nd")}
    for way in root.iter("way"):
        tags = {t.get("k"): t for t in way.findall("tag")}
        if "highway" not in tags:
            continue

        def put(k, v):
            if k in tags:
                tags[k].set("v", v)
            else:
                tags[k] = ET.SubElement(way, "tag", k=k, v=v)
        hw = tags["highway"].get("v")
        ring = "junction" in tags and tags["junction"].get("v") == "roundabout"
        put("name", ("roundabout" if ring else hw) + "|" + (tags["name"].get("v") if "name" in tags else ""))
        # ponytail: underpasses (layer<0) would sink below CARLA's flat ground plane, so they stay at
        # ground level. Fine while no imported road crosses over them (true for railway underpasses).
        if "layer" in tags and tags["layer"].get("v").startswith("-"):
            way.remove(tags.pop("layer"))
        elif "layer" in tags and hw not in WAY_TYPES:  # footbridges etc. would lift the roads they touch
            way.remove(tags.pop("layer"))
        oneway = ring or (tags["oneway"].get("v") in ("yes", "true", "1", "-1") if "oneway" in tags else False)
        # Not where a road meets a roundabout: netconvert then joins a 2-lane entry to the 1-lane ring
        # with a straight lane across the island, and cars on it drive through the ring traffic.
        if hw in WIDEN and not oneway and not ring_nodes & {nd.get("ref") for nd in way.findall("nd")}:
            for k in [k for k in tags if k.startswith(("lanes", "turn:lanes", "width"))]:
                way.remove(tags.pop(k))
            put("lanes", "4")
            put("lanes:forward", "2")
            put("lanes:backward", "2")
    split_near_bridges(root)
    return ET.tostring(root, encoding="unicode")


def split_near_bridges(root):
    """netconvert puts bridge nodes at deck height and each node within 100 m of road from one
    (and each node right next to one, however far) at a distance-weighted mix of deck and ground
    height, then ramps straight between nodes. Long roads have few nodes, so an approach whose
    next node is 1 km away, or a dead end at the map edge, stayed at deck height all the way.
    Cut every road near a bridge into ways of about STEP m: then each ramp is ~100 m long."""
    ll = {n.get("id"): (float(n.get("lat")), float(n.get("lon"))) for n in root.iter("node")}
    k = math.cos(math.radians(next(iter(ll.values()))[0]))
    xy = {i: (lon * k * 111320, lat * 110570) for i, (lat, lon) in ll.items()}
    ways = [w for w in root.iter("way") if any(t.get("k") == "highway" and t.get("v") in WAY_TYPES for t in w.findall("tag"))]
    use = collections.Counter(nd.get("ref") for w in ways for nd in w.findall("nd"))
    rings = {w for w in ways if any(t.get("k") == "junction" and t.get("v") == "roundabout" for t in w.findall("tag"))}
    grid = collections.defaultdict(list)  # bridge nodes by NEAR-sized cell
    for w in ways:
        if any(t.get("k") == "layer" and t.get("v") != "0" for t in w.findall("tag")):
            for nd in w.findall("nd"):
                p = xy[nd.get("ref")]
                grid[(p[0] // NEAR, p[1] // NEAR)].append(p)

    def near(p):
        return any(math.dist(p, q) < NEAR for i in (-1, 0, 1) for j in (-1, 0, 1)
                   for q in grid[(p[0] // NEAR + i, p[1] // NEAR + j)])
    ids = itertools.count(max(int(e.get("id")) for e in itertools.chain(root.iter("node"), root.iter("way"))) + 1)
    for w in ways:
        refs = [nd.get("ref") for nd in w.findall("nd")]
        if w in rings or not any(near(xy[r]) for r in refs):  # split rings get kinks
            continue
        dense = refs[:1]
        for a, b in zip(refs, refs[1:]):  # new nodes on long segments, near the bridge only
            n = math.ceil(math.dist(xy[a], xy[b]) / STEP)
            for i in range(1, n):
                f = i / n
                p = (xy[a][0] + f * (xy[b][0] - xy[a][0]), xy[a][1] + f * (xy[b][1] - xy[a][1]))
                if near(p):
                    nid = str(next(ids))
                    xy[nid] = p
                    root.insert(0, ET.Element("node", id=nid, version="1", lat=str(ll[a][0] + f * (ll[b][0] - ll[a][0])),
                                              lon=str(ll[a][1] + f * (ll[b][1] - ll[a][1]))))
                    dense.append(nid)
            dense.append(b)
        cum = [0.0]
        for a, b in zip(dense, dense[1:]):
            cum.append(cum[-1] + math.dist(xy[a], xy[b]))
        hard = [cum[0], cum[-1]] + [c for r, c in zip(dense, cum) if use[r] > 1]  # junctions
        cuts, last = [0], 0.0
        for i in range(1, len(dense) - 1):
            if use[dense[i]] > 1:
                last = cum[i]
            # 15 m from junctions: shorter pieces would get merged into them (junctions.join, 10 m)
            elif cum[i] - last >= STEP and min(abs(cum[i] - h) for h in hard) >= 15 and near(xy[dense[i]]):
                cuts.append(i)
                last = cum[i]
        cuts.append(len(dense) - 1)
        if len(cuts) == 2 and len(dense) == len(refs):
            continue
        tags = w.findall("tag")
        for nd in w.findall("nd"):
            w.remove(nd)
        for j, r in enumerate(dense[:cuts[1] + 1]):  # first piece keeps the way's id
            w.insert(j, ET.Element("nd", ref=r))
        for a, b in zip(cuts[1:], cuts[2:]):
            # ponytail: turn restrictions naming this way now name only its first piece; netconvert
            # warns and drops the rest. None of them are on bridge approaches here.
            piece = ET.SubElement(root, "way", id=str(next(ids)), version="1")
            for r in dense[a:b + 1]:
                ET.SubElement(piece, "nd", ref=r)
            for t in tags:
                ET.SubElement(piece, "tag", k=t.get("k"), v=t.get("v"))


def road_class(road):
    """highway class load_osm put in the road's name ("roundabout" for rings; "" for junction pieces)."""
    return road.get("name", "").split("|")[0] if "|" in road.get("name", "") else ""


def priority_signs(m, tree, roads, kmh):
    """(waypoint, type) of a give-way (205) or stop (206) sign on each minor road into a junction
    without traffic lights. Minor = lower class than the junction's main road; link roads rank
    lowest; roundabout rings rank highest (entries get give-way, as in Germany). Stop where the
    main road is a trunk/primary road or 70+ km/h. Junctions of equal roads stay unsigned
    (right before left), and so do motorway/trunk merges and splits."""
    out = []
    for j in tree.iter("junction"):
        if j.find("controller") is not None:  # traffic lights
            continue
        cls = {}
        for c in j.findall("connection"):
            r = roads[c.get("incomingRoad")]
            cls[r.get("id")] = road_class(r)
        if all(c.split("_")[0] in ("motorway", "trunk") for c in cls.values()):
            continue
        rank = {r: 0 if c.endswith("_link") else RANK.get(c, 0) for r, c in cls.items()}
        top = max(rank.values())
        main = [r for r in rank if rank[r] == top]
        fast = max((kmh.get((int(r), -1)) or kmh.get((int(r), 1)) or 0) for r in main) >= 70
        for rid in rank:
            if rank[rid] == top:
                continue
            r, length = roads[rid], float(roads[rid].get("length"))
            succ = r.find("link/successor")
            # the lanes that drive into the junction: right lanes end at it, left lanes start at it
            ends_here = succ is not None and succ.get("elementType") == "junction" and succ.get("elementId") == j.get("id")
            w = m.get_waypoint_xodr(int(rid), -1 if ends_here else 1, max(length - 3, length / 2) if ends_here else min(3.0, length / 2))
            if w is not None:
                stop = cls[main[0]] != "roundabout" and (fast or cls[main[0]] in ("trunk", "primary"))
                out.append((w, "206" if stop else "205"))
    return out


def add_signs(xodr, osm_root, proj):
    """Speed-limit signs (from the lane speeds netconvert wrote) wherever the limit changes,
    stop / give-way signs from OSM nodes, and give-way / stop signs on the minor roads of the
    other junctions (priority_signs). CARLA spawns them as traffic.* actors and the Traffic
    Manager obeys them. CARLA 0.9.15 has no priority-road (yellow diamond) or roundabout sign."""
    m = carla.Map("osm", xodr)
    tree = ET.fromstring(xodr)
    roads = {r.get("id"): r for r in tree.iter("road")}
    kmh = {(int(rid), int(lane.get("id"))): round(float(sp.get("max")) * 3.6 / 10) * 10
           for rid, r in roads.items() for lane in r.iter("lane") for sp in lane.findall("speed")}
    limit = lambda w: kmh.get((w.road_id, w.lane_id))  # noqa: E731

    signs = []  # (waypoint the sign is for, OpenDRIVE type, value)
    for rid, r in roads.items():
        if r.get("junction") != "-1":
            continue
        length = float(r.get("length"))
        for side in (-1, 1):  # right lanes drive +s, left lanes -s: sign near where each direction starts
            w = m.get_waypoint_xodr(int(rid), side, min(3.0, length / 2) if side < 0 else max(length - 3.0, length / 2))
            if w is None or limit(w) is None:
                continue
            upstream = set()
            for p in w.previous(20.0):
                upstream |= {limit(q) for q in (p.previous(20.0) if p.is_junction else [p]) if not q.is_junction}
            if upstream != {limit(w)}:  # limit changes here, or the road enters the map here
                signs.append((w, "274", limit(w)))

    osm_types = {("highway", "stop"): "206", ("traffic_sign", "DE:206"): "206",
                 ("highway", "give_way"): "205", ("traffic_sign", "DE:205"): "205"}
    for n in osm_root.iter("node"):
        typ = next((osm_types[k] for k in ((t.get("k"), t.get("v")) for t in n.findall("tag")) if k in osm_types), None)
        if typ is None:
            continue
        x, y = proj(float(n.get("lat")), float(n.get("lon")))
        w = m.get_waypoint(carla.Location(x, -y, 0))  # CARLA's y axis is flipped vs OpenDRIVE
        if w is None or w.is_junction:
            continue
        # The sign is for whichever direction of this road reaches a junction first.
        other = m.get_waypoint_xodr(w.road_id, 1 if w.lane_id < 0 else -1, w.s)
        to_junction = lambda c: next((k for k in range(1, 26) if any(q.is_junction for q in c.next(2.0 * k))), 99)  # noqa: E731
        signs.append((min(filter(None, (w, other)), key=to_junction), typ, -1))
    signed = {(w.road_id, w.lane_id < 0) for w, typ, _ in signs if typ != "274"}
    signs += [(w, typ, -1) for w, typ in priority_signs(m, tree, roads, kmh) if (w.road_id, w.lane_id < 0) not in signed]

    for sid, (w, typ, value) in enumerate(signs, 100000):
        side = -1 if w.lane_id < 0 else 1
        width, lid = 0.0, side
        while (q := m.get_waypoint_xodr(w.road_id, lid, w.s)) is not None:  # all lanes on this side
            width += q.lane_width
            lid += side
        r = roads[str(w.road_id)]
        signals = r.find("signals")
        if signals is None:
            signals = ET.SubElement(r, "signals")
        sig = ET.SubElement(signals, "signal", id=str(sid), name=f"sign{sid}", s=f"{w.s:.3f}",
                            t=f"{side * (width + 1.0):.3f}", zOffset="0", hOffset="0", pitch="0", roll="0",
                            orientation="+" if side < 0 else "-", dynamic="no", country="OpenDRIVE",
                            # CARLA picks the speed sign model by subtype ("30".."120"), the limit by value
                            type=typ, subtype=str(value) if typ == "274" else "-1", value=str(value),
                            unit="km/h", height="2", width="0.8", text="")
        ET.SubElement(sig, "validity", fromLane=str(side), toLane=str(lid - side))
    print(f"signs: {collections.Counter((t, v) for _, t, v in signs)}")
    return ET.tostring(tree, encoding="unicode")


def simplify(ps, tol=0.05, max_len=14):
    """Polyline -> (start, end) indices of few straight chords, each within tol m of the points it
    replaces and at most max_len points long."""
    def off(p, a, b):
        ab, ap = b - a, p - a
        c = ab.cross(ap)
        return math.sqrt(c.x ** 2 + c.y ** 2 + c.z ** 2) / max(ab.length(), 1e-6)
    out, a = [], 0
    for b in range(2, len(ps)):
        if b - a > max_len or any(off(ps[k], ps[a], ps[b]) > tol for k in range(a + 1, b)):
            out.append((a, b - 1))
            a = b - 1
    if len(ps) > 1:
        out.append((a, len(ps) - 1))
    if len(out) > 1 and out[-1][1] - out[-1][0] < max_len / 2:  # short last chord: share the last two's length
        a = out[-2][0]
        out[-2:] = [(a, (a + len(ps) - 1) // 2), ((a + len(ps) - 1) // 2, len(ps) - 1)]
    return out


def lane_lines(m, painted, edged):
    """(points, style) for every line to paint on the painted roads; style is solid or broken.
    Only the edged roads get edge lines."""
    lanes = collections.defaultdict(list)
    for w in m.generate_waypoints(1.0):
        if not w.is_junction and w.road_id in painted:
            lanes[(w.road_id, w.section_id, w.lane_id)].append(w)
    n = collections.Counter((road, sec) for road, sec, _ in lanes)  # lanes in this direction

    def edge(ws, side, off=0.0):
        pts = []
        for w in ws:
            t, v = w.transform.location, w.transform.get_right_vector()
            d = side * w.lane_width / 2 + off
            pts.append(carla.Location(t.x + v.x * d, t.y + v.y * d, t.z + LIFT))
        return pts

    def centre(w):
        """Style of the line on the left of innermost lane w. netconvert gives each direction of a
        two-way road its own road, so look for oncoming traffic just past that line."""
        p = edge([w], -1, -0.5)[0]
        o = m.get_waypoint(p, project_to_road=False)
        f, g = w.transform.get_forward_vector(), o.transform.get_forward_vector() if o else None
        if o is None or o.is_junction or f.x * g.x + f.y * g.y > -0.5:
            return "solid" if w.road_id in edged else None  # left edge of a one-way road
        if o.road_id in painted and o.road_id < w.road_id:
            return None  # the oncoming road paints it
        return "double" if max(n[w.road_id, w.section_id], n[o.road_id, o.section_id]) >= 2 else "broken"
    out = []
    for (road, sec, lid), ws in lanes.items():
        ws.sort(key=lambda w: w.s if lid < 0 else -w.s)  # driving order
        # Lanes run outward to the driver's right, so each lane paints its right edge...
        inner = (road, sec, lid + (1 if lid > 0 else -1)) in lanes
        if inner or road in edged:
            out.append((edge(ws, 1), "broken" if inner else "solid"))
        if abs(lid) != 1:
            continue
        # ...and the innermost lane its left one: a one-way road's left edge, or the centre line.
        for style, run in itertools.groupby(ws, centre):
            run = list(run)
            if style == "double":
                out += [(edge(run, -1, o), "solid") for o in (-LINE_W, LINE_W)]
            elif style:
                out.append((edge(run, -1), style))
    return [(r, style) for pts, style in out for r in unkink(pts)]


def unkink(pts, gap=2.5):
    """Split a line (points ~1 m apart) where it jumps: netconvert sometimes ends a road with a
    sub-metre piece at a sharp angle, whose lane edges then swing metres sideways (e.g. into a
    roundabout island)."""
    out = [[pts[0]]]
    for a, b in zip(pts, pts[1:]):
        if a.distance(b) > gap:
            out.append([])
        out[-1].append(b)
    return [r for r in out if len(r) > 1]


def strips(pts, style, max_len):
    """Line points (1 m apart) -> straight chords to lay paint strips on."""
    if style == "broken":
        # ponytail: dashes are straight 6 m chords, so on a 50 m-radius ramp each sags 9 cm. Fine at road scale.
        return list(zip(pts[::DASH + GAP], pts[DASH::DASH + GAP]))
    return [(pts[a], pts[b]) for a, b in simplify(pts, max_len=max_len)]


def strip_transforms(a, b, mesh_len):
    """(centre, rotation, scale) of the mesh strips that paint chord a-b LINE_W wide. The mesh
    only scales uniformly, so a short chord gets thin strips: lay several side by side.
    ponytail: that makes curves and dashes cost 2+ actors each. A white mesh ~6 m long would halve
    dash cost; CARLA 0.9.15 only ships yellow or dotted ones that size."""
    dx, dy, dz = b.x - a.x, b.y - a.y, b.z - a.z
    s = math.sqrt(dx * dx + dy * dy + dz * dz) / mesh_len
    # Capped: kinks in netconvert's road shapes leave chords of a metre or two, which would take 10+
    # strips each (~10% of all strips); the paint is just thinner there.
    k = min(3, max(1, round(LINE_W / (PAINT_W * s))))
    yaw = math.atan2(dy, dx)
    rot = carla.Rotation(pitch=math.degrees(math.atan2(dz, math.hypot(dx, dy))), yaw=math.degrees(yaw))
    step = PAINT_W * s * 0.95  # overlap a little so no seam shows
    out = []
    for j in range(k):
        o = (j - (k - 1) / 2) * step
        out.append((carla.Location((a.x + b.x) / 2 - math.sin(yaw) * o, (a.y + b.y) / 2 + math.cos(yaw) * o,
                                   (a.z + b.z) / 2), rot, s))
    return out


assert [len(r) for r in unkink([carla.Location(x, 0, 0) for x in range(5)] + [carla.Location(4, 9, 0), carla.Location(5, 9, 0)])] == [5, 2]
assert simplify([carla.Location(x, 0, 0) for x in range(10)]) == [(0, 9)]
assert simplify([carla.Location(x, 0, 0) for x in range(5)] + [carla.Location(4, y, 0) for y in range(1, 5)]) == [(0, 4), (4, 8)]
assert simplify([carla.Location(x, 0, 0) for x in range(17)], max_len=12) == [(0, 8), (8, 16)]  # no 4 m tail
_n = 10 * LINE_W / PAINT_W  # chord a 10 m mesh paints LINE_W wide at one strip
_t = strip_transforms(carla.Location(0, 0, 0), carla.Location(_n, 0, 0), 10)
assert len(_t) == 1 and abs(_t[0][0].x - _n / 2) < 1e-6 and abs(_t[0][2] * PAINT_W - LINE_W) < 1e-6
assert len(strip_transforms(carla.Location(0, 0, 0), carla.Location(0, _n / 2, 0), 10)) == 2


def plan_strips(m, xodr, mesh_len):
    """(centre, rotation, scale) of every paint strip on map m, generated from xodr."""
    cls = {int(r.get("id")): r.get("name", "").split("|")[0].replace("_link", "") for r in ET.fromstring(xodr).iter("road")}
    painted = {r for r, c in cls.items() if c in PAINTED | EDGED}
    edged = {r for r, c in cls.items() if c in EDGED}
    max_len = int(mesh_len * LINE_W / PAINT_W)  # m (= points): longest chord one strip paints LINE_W wide
    return [t for pts, style in lane_lines(m, painted, edged)
            for a, b in strips(pts, style, max_len) for t in strip_transforms(a, b, mesh_len)]


def bridge_parts(m):
    """(mesh, centre, rotation, scale) of concrete edge beams along raised roads (bridges and their
    ramps: OSM has no ground elevation, so raised = layer) and of pillars under them every 25 m,
    wherever no road runs below. The road mesh is one-sided, so without these a bridge is
    invisible from below and just a hump from above."""
    raised, ground = collections.defaultdict(list), collections.defaultdict(list)
    for w in m.generate_waypoints(1.0):
        z = w.transform.location.z
        if z <= RAISED:
            ground[(int(w.transform.location.x // 10), int(w.transform.location.y // 10))].append(w)
        elif not w.is_junction:
            raised[(w.road_id, w.section_id, w.lane_id)].append(w)

    def free(x, y, r):  # no ground road within r m of a lane's edge
        return all(math.hypot(x - g.transform.location.x, y - g.transform.location.y) > g.lane_width / 2 + r
                   for i in (-1, 0, 1) for k in (-1, 0, 1) for g in ground[(int(x // 10) + i, int(y // 10) + k)])

    def side(ws, d):
        return [carla.Location(w.transform.location.x + w.transform.get_right_vector().x * d,
                               w.transform.location.y + w.transform.get_right_vector().y * d,
                               w.transform.location.z) for w in ws]
    out = []
    for (road, sec, lid), ws in raised.items():
        if len(ws) < 5:
            continue
        ws.sort(key=lambda w: w.s if lid < 0 else -w.s)  # driving order
        hw = ws[0].lane_width / 2 + BEAM_W / 2
        edges = []
        if (road, sec, lid + (-1 if lid < 0 else 1)) not in raised:  # outermost lane: right edge
            edges.append(side(ws, hw))
        if abs(lid) == 1:  # innermost lane: left edge, unless it is the centre of a two-way road
            mid = ws[len(ws) // 2]
            o = m.get_waypoint(side([mid], -hw - 0.5)[0], project_to_road=False)
            f = mid.transform.get_forward_vector()
            if o is None or o.is_junction or f.x * o.transform.get_forward_vector().x + f.y * o.transform.get_forward_vector().y > -0.5:
                edges.append(side(ws, -hw))
            for w in ws[12::25]:  # pillars: only on the bridge proper, beside any road below
                z, p = w.transform.location.z, w.transform.location
                if z > 3 and free(p.x, p.y, z / PILLAR_H * PILLAR_W / 2 + 1.5):
                    s = (z - 0.05) / PILLAR_H
                    out.append((PILLAR, carla.Location(p.x, p.y, s * PILLAR_H / 2), carla.Rotation(), s))
        for pts in edges:
            run = sum(a.distance(b) for a, b in zip(pts, pts[1:]))
            s = min(BEAM_S, run / BEAM_L)
            half = s * BEAM_L / 2
            for a, b in [(pts[a], pts[b]) for a, b in simplify(pts, tol=0.3, max_len=int(s * BEAM_L))]:
                n = a.distance(b) or 1.0
                u = ((b.x - a.x) / n, (b.y - a.y) / n, (b.z - a.z) / n)
                # A beam longer than its chord overhangs both ends; keep the ends of a run flush.
                o, k = (a, half) if a is pts[0] else (b, -half) if b is pts[-1] else (a, n / 2)
                c = carla.Location(o.x + u[0] * k, o.y + u[1] * k, o.z + u[2] * k)
                rot = carla.Rotation(pitch=math.degrees(math.asin(max(-1.0, min(1.0, u[2])))), yaw=math.degrees(math.atan2(u[1], u[0])))
                out.append((BEAM, c, rot, s))
    return out


def spawn_meshes(client, world, parts):
    """Spawn (mesh path, bounding-box centre, rotation, uniform scale) as static mesh actors.
    They stay until the world is reloaded. Returns how many failed."""
    bp = world.get_blueprint_library().find("static.prop.mesh")
    bbs, cmds = {}, []
    for mesh, c, rot, s in parts:
        bp.set_attribute("mesh_path", mesh)
        if mesh not in bbs:
            # Pivots can be far from the geometry (lane-line meshes were cut out of Town03): measure.
            probe = world.spawn_actor(bp, carla.Transform(carla.Location(0, 0, -500)))
            bbs[mesh] = probe.bounding_box.location
            probe.destroy()
        bl = bbs[mesh]
        off = carla.Transform(carla.Location(), rot).transform(carla.Location(bl.x * s, bl.y * s, bl.z * s))
        bp.set_attribute("scale", f"{s:.4f}")
        cmds.append(carla.command.SpawnActor(bp, carla.Transform(carla.Location(c.x - off.x, c.y - off.y, c.z - off.z), rot)))
    return sum(1 for r in client.apply_batch_sync(cmds, False) if r.error)


def decorate(client, world, xodr):
    """Lane lines and bridge structures, as static mesh actors."""
    m = world.get_map()
    probe_bp = world.get_blueprint_library().find("static.prop.mesh")
    probe_bp.set_attribute("mesh_path", LINE_MESH)
    probe = world.spawn_actor(probe_bp, carla.Transform(carla.Location(0, 0, -500)))
    mesh_len = 2 * probe.bounding_box.extent.x
    probe.destroy()
    lines = [(LINE_MESH, c, rot, s) for c, rot, s in plan_strips(m, xodr, mesh_len)]
    bridge = bridge_parts(m)
    failed = spawn_meshes(client, world, lines + bridge)
    print(f"lane lines: {len(lines)} paint strips; bridges: {sum(p[0] == BEAM for p in bridge)} beams, "
          f"{sum(p[0] == PILLAR for p in bridge)} pillars" + (f"; {failed} failed" if failed else ""))


if __name__ == "__main__":
    osm_path = sys.argv[1]
    host = sys.argv[2] if len(sys.argv) > 2 else "host.docker.internal"

    # Default proj is bare "+proj=tmerc" (central meridian 0°): at 10.75°E that rotates
    # the map ~8° and breaks CARLA's georeference. Anchor it on the bbox instead, at the
    # west edge: the OpenDriveMap level only has ground for x > 0.
    root = ET.parse(osm_path).getroot()
    b = root.find("bounds")
    # DT_NAME=<path>: only write <path>.xodr for CARLA's Digital Twin Tool. Its tiles start at (0,0) and
    # grow towards +x,+y, so (0,0) goes on the bbox's SW corner instead of the west edge's middle.
    dt = os.environ.get("DT_NAME")
    lat0 = float(b.get("minlat")) if dt else (float(b.get("minlat")) + float(b.get("maxlat"))) / 2
    lon0 = float(b.get("minlon"))

    settings = carla.Osm2OdrSettings()
    settings.proj_string = f"+proj=tmerc +lat_0={lat0} +lon_0={lon0} +k=1 +x_0=0 +y_0=0 +ellps=WGS84 +units=m +no_defs"
    settings.set_osm_way_types(WAY_TYPES)
    # LANE_W=3.5 for the Digital Twin maps: 4 m lanes plus their sidewalks reached into the real buildings
    settings.default_lane_width = float(os.environ.get("LANE_W", 4.0))
    settings.generate_traffic_lights = True
    # Default 0 flattens every bridge onto the road below it.
    settings.elevation_layer_height = LAYER_HEIGHT
    # center_map doesn't stop SUMO shifting the net to its corner; explicit offsets do,
    # so (0,0) = (lat0, lon0) and CARLA's geolocation matches the roads.
    settings.use_offsets = True
    settings.offset_x = 0.0
    settings.offset_y = 0.0

    xodr = add_signs(carla.Osm2Odr.convert(prepare_osm(root), settings), root, tmerc(lat0, lon0))
    xodr_path = (dt or osm_path.rsplit(".", 1)[0]) + ".xodr"
    with open(xodr_path, "w", encoding="utf-8") as f:
        f.write(xodr)
    print("wrote", xodr_path)
    # CARLA 0.9.15 kills the whole server (assert "_length > 0.0", Geometry.cpp) on a
    # zero-length road piece, which netconvert sometimes emits (e.g. with wide lanes).
    if re.search(r'<geometry[^>]*length="0(\.0*)?"', xodr):
        sys.exit("xodr has a zero-length geometry; loading it would crash the CARLA server. "
                 "Try a different default_lane_width or crop.")
    if dt:
        sys.exit()

    client = carla.Client(host, 2000)
    client.set_timeout(600.0)  # mesh generation for a few km of road takes a while under Wine
    print("server", client.get_server_version(), "- generating world...")
    # No walls: CARLA walls every OpenDRIVE road's outer edges, and netconvert gives each direction of
    # a two-way road its own road, so they'd also run down every centre line. traffic.py's hybrid mode
    # keeps cars on the lanes without them. ponytail: a physics car (manual driving) that leaves the
    # road falls through the ground; wall_height=1.0 brings back walls, centre ones included.
    world = client.generate_opendrive_world(xodr, carla.OpendriveGenerationParameters(
        vertex_distance=2.0, max_road_length=500.0, wall_height=0.0,
        additional_width=0.6, smooth_junctions=True, enable_mesh_visibility=True))
    print("traffic signs/lights:", collections.Counter(a.type_id for a in world.get_actors().filter("traffic.*")))

    # Top-down view over the map centre, north up.
    sp = [s.location for s in world.get_map().get_spawn_points()]
    cx = (min(p.x for p in sp) + max(p.x for p in sp)) / 2
    cy = (min(p.y for p in sp) + max(p.y for p in sp)) / 2
    world.get_spectator().set_transform(carla.Transform(carla.Location(cx, cy, 2300), carla.Rotation(pitch=-90, yaw=-90)))
    print("loaded:", len(sp), "spawn points")
    decorate(client, world, xodr)
