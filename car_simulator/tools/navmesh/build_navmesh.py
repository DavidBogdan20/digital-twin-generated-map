#!/usr/bin/env python3
"""Builds the CARLA 0.9.15 pedestrian navmesh (<map>.bin) of a Digital Twin map from its OpenDRIVE file.

Run on the Mac:  python3 tools/navmesh/build_navmesh.py
It re-runs itself in the carla-client:0.9.15 Docker image (carla 0.9.15 Python API + an amd64 Linux for
CARLA's RecastBuilder). Writes tools/navmesh/out/<map>.bin and out/navmesh.png. See README.md for the why.
"""
import argparse
import math
import os
import shutil
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET
import zlib

HERE = os.path.dirname(os.path.abspath(__file__))
INPUTS = os.path.normpath(os.path.join(HERE, "..", "..", "..", "car_simulator_inputs"))

STEP = 1.0             # m between samples along a lane (the Digital Twin Tool uses 0.5, then decimates driving lanes)
SIDEWALK_UP = 0.1524 + 0.15  # Lane::GetCornerPositions adds 0.1524 to sidewalks, GenerateRoadMesh another 0.15
ROAD_LIFT = 0.0324     # roads/crosswalks raised in the navmesh so the 0.30 m curb becomes 0.27 m: Recast's
                       # walkableClimb is exactly 0.30 m, so a true curb would cut every sidewalk off the crossings
RASTER_BIAS = 0.02     # m Recast adds when it rasterizes (span tops round up per 0.2 m cell), measured with
                       # this script's height check: subtracted in the OBJ so the navmesh lands on the surfaces
AREA = {1: "sidewalk", 2: "crosswalk", 3: "road", 4: "grass", 63: "none"}  # RecastBuilder Sample.h SamplePolyAreas


FLAT_MAPS = ("WolfsburgHighway",)  # built with digital_twin/apply_patches.py's flat roads (carsim.road.FLAT_MAPS)
WAVES = True  # inside(): False for FLAT_MAPS


def deformation(x, y):
    """0.7 * GetZPosInDeformation (LibCarla road/Deformation.h): the stock UOpenDriveToMap::GetHeight without a
    heightmap; 0 on a map built with the flat-roads patch."""
    if not WAVES:
        return 0.0
    return 0.7 * (0.6 * math.sin(0.035 * x - 0.08 * y + 1000.0) + 1.1 * math.sin(0.02 * x + 0.05 * y - 1500.0))


def surface(x, y, z, sidewalk):
    """Height of the generated 3D surface over an OpenDRIVE point (driving lanes also get bumps of up to -0.10 m)."""
    return z + deformation(x, y) + (SIDEWALK_UP if sidewalk else 0.0)


def elevation(road, s):
    z = 0.0
    for e in road.iter("elevation"):
        if float(e.get("s")) <= s + 1e-9:
            ds = s - float(e.get("s"))
            z = float(e.get("a")) + float(e.get("b")) * ds + float(e.get("c")) * ds ** 2 + float(e.get("d")) * ds ** 3
    return z


def flat_junctions(root):
    """Junctions the tool covers with SDFToMesh: >2 connections and, with the project's Map.cpp patch
    (digital_twin/apply_patches.py), every connecting road within 0.3 m of z = 0 at s = 0, L/2, L."""
    roads = {r.get("id"): r for r in root.iter("road")}
    out = set()
    for j in root.iter("junction"):
        conns = [c.get("connectingRoad") for c in j.iter("connection")]
        top = max((abs(elevation(roads[c], f * float(roads[c].get("length")))) for c in conns for f in (0, .5, 1)),
                  default=0.0)
        if len(conns) > 2 and top < 0.3:
            out.add(j.get("id"))
    return out


def lanes(root):
    for road in root.iter("road"):
        secs = road.find("lanes").findall("laneSection")
        for i, sec in enumerate(secs):
            s0 = float(sec.get("s"))
            s1 = float(secs[i + 1].get("s")) if i + 1 < len(secs) else float(road.get("length"))
            for lane in sec.findall("left/lane") + sec.findall("right/lane"):
                yield road, s0, s1, int(lane.get("id")), lane.get("type")


class Keepout:
    """Building outlines in CARLA metres, one 'x y x y ...' per line, with a 50 m grid index."""

    def __init__(self, path):
        self.polys, self.grid = [], {}
        for line in open(path):
            v = list(map(float, line.split()))
            if len(v) >= 6:
                p = list(zip(v[0::2], v[1::2]))
                self.polys.append(p)
                xs, ys = [a for a, _ in p], [b for _, b in p]
                for gx in range(int(min(xs) // 50), int(max(xs) // 50) + 1):
                    for gy in range(int(min(ys) // 50), int(max(ys) // 50) + 1):
                        self.grid.setdefault((gx, gy), []).append(p)

    def inside(self, x, y):
        for p in self.grid.get((int(x // 50), int(y // 50)), ()):
            hit = False
            for (x1, y1), (x2, y2) in zip(p, p[1:] + p[:1]):
                if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
                    hit = not hit
            if hit:
                return True
        return False


def crosswalks(m, root):
    """carla.Map.get_crosswalks(): corners already widened 1 m each way (Map::GetAllCrosswalkZones). Outlines here
    are not closed, so CARLA's own fan splitter (Map::GetAllCrosswalkMesh needs a repeated vertex) finds none;
    every object has 4 corners, so split by 4."""
    objs = [o for o in root.iter("object") if o.get("type") == "crosswalk"]
    pts = [(p.x, p.y, p.z) for p in m.get_crosswalks()]
    assert all(len(o.findall(".//cornerLocal")) == 4 for o in objs) and len(pts) == 4 * len(objs), "unexpected outlines"
    return [pts[i:i + 4] for i in range(0, len(pts), 4)]


def build_triangles(m, root, keepout):
    """Lane strips like MeshFactory (2 vertices across, STEP along), at the 3D surface heights."""
    import carla
    flat = flat_junctions(root)
    tris, samples, stats = [], [], dict(lanes=0, skipped_junction_sidewalks=0, cut_by_buildings=0, length=0.0)
    for road, s0, s1, lid, kind in lanes(root):
        if kind not in ("driving", "sidewalk") or s1 - s0 < 0.02:
            continue  # the tool spawns meshes only for Driving and Sidewalk lanes
        rid, sidewalk = int(road.get("id")), kind == "sidewalk"
        if sidewalk and road.get("junction") in flat:  # Map::GenerateSingleJunction skips sidewalks lying on a road
            mid = m.get_waypoint_xodr(rid, lid, (s0 + s1) / 2)
            if mid and m.get_waypoint(mid.transform.location, project_to_road=False,
                                      lane_type=carla.LaneType.Driving) is not None:
                stats["skipped_junction_sidewalks"] += 1
                continue
        n = max(1, math.ceil((s1 - s0 - 0.002) / STEP))
        prev = None
        stats["lanes"] += 1
        stats["length"] += s1 - s0
        for k in range(n + 1):
            wp = m.get_waypoint_xodr(rid, lid, s0 + 0.001 + (s1 - s0 - 0.002) * k / n)
            if wp is None or wp.lane_width < 0.01:
                prev = None
                continue
            c, yaw, h = wp.transform.location, math.radians(wp.transform.rotation.yaw), wp.lane_width / 2
            edge = []
            for side in (-h, h):
                x, y = c.x - math.sin(yaw) * side, c.y + math.cos(yaw) * side
                edge.append((x, y, surface(x, y, c.z, sidewalk) + (0 if sidewalk else ROAD_LIFT)))
            if k % 10 == 5:
                samples.append((kind, c.x, c.y, surface(c.x, c.y, c.z, sidewalk)))
            if prev:
                for t in ((prev[0], prev[1], edge[1]), (prev[0], edge[1], edge[0])):
                    if keepout.inside(sum(p[0] for p in t) / 3, sum(p[1] for p in t) / 3):
                        stats["cut_by_buildings"] += 1
                    else:
                        tris.append((kind if sidewalk else "road", t))
            prev = edge
    # Ascending height: RecastBuilder marks each triangle's area on all spans from its lowest vertex up 5 m, in
    # file order, so a lower road written later would relabel the bridge above it. Crosswalks go last (as in
    # CarlaEpisode.cpp: RoadMesh + CrosswalksMesh) to win over the road they lie on.
    tris.sort(key=lambda t: min(p[2] for p in t[1]))
    for poly in crosswalks(m, root):
        pts = [(x, y, surface(x, y, z, False) + ROAD_LIFT) for x, y, z in poly]
        tris += [("crosswalk", (pts[0], pts[i], pts[i + 1])) for i in range(1, len(pts) - 1)]
        cx, cy = sum(p[0] for p in poly) / 4, sum(p[1] for p in poly) / 4
        samples.append(("crosswalk", cx, cy, surface(cx, cy, sum(p[2] for p in poly) / 4, False)))
    return tris, samples, stats


def write_obj(path, tris):
    """Mesh::GenerateOBJForRecast format: 'v x z y' (Recast is Y-up), all vertices first (RecastBuilder's loader
    resets the material on every 'v' row), 'usemtl road|sidewalk|crosswalk' (MeshLoaderObj.cpp), faces turned
    so their normal points up."""
    with open(path, "w") as f:
        for _, t in tris:
            for x, y, z in t:
                f.write("v %.3f %.3f %.3f\n" % (x, z - RASTER_BIAS, y))
        mat = None
        for i, (kind, (a, b, c)) in enumerate(tris):
            if kind != mat:
                f.write("usemtl %s\n" % kind)
                mat = kind
            up = (b[1] - a[1]) * (c[0] - a[0]) - (b[0] - a[0]) * (c[1] - a[1])  # Recast normal y
            f.write("f %d %d %d\n" % ((3 * i + 1, 3 * i + 2, 3 * i + 3) if up > 0 else (3 * i + 1, 3 * i + 3, 3 * i + 2)))


def read_navmesh(path):
    """Parses RecastBuilder's file (Sample::saveAll, 32-bit dtTileRef) and Detour v7 tile data. Returns
    (header, tiles); each tile is a list of polys (area, flags, verts xyz CARLA, detail triangles, neis)."""
    data = open(path, "rb").read()
    magic, version, ntiles, ox, oy, oz, tw, th, max_tiles, max_polys = struct.unpack_from("<3i5f2i", data, 0)
    assert magic == 0x4D534554 and version == 1, "not a navmesh set (MSET v1)"
    pos, tiles = 40, []
    for _ in range(ntiles):
        ref, size = struct.unpack_from("<Ii", data, pos)
        pos += 8
        h = struct.unpack_from("<15i10f", data, pos)
        assert h[0] == 0x444E4156 and h[1] == 7, "bad tile header"
        npoly, nvert, nlink, ndm, ndv, ndt, nbv, noff = h[6], h[7], h[8], h[9], h[10], h[11], h[12], h[13]
        o = pos + 100
        verts = struct.unpack_from("<%df" % (3 * nvert), data, o)
        o += 12 * nvert
        polys_at, o = o, o + 32 * npoly + 12 * nlink
        dm_at, o = o, o + 12 * ndm
        dverts = struct.unpack_from("<%df" % (3 * ndv), data, o)
        o += 12 * ndv
        dt_at = o
        o += 4 * ndt + 16 * nbv + 36 * noff
        assert o - pos == size, "tile size mismatch: %d != %d" % (o - pos, size)
        polys = []
        for i in range(npoly):
            _, *rest = struct.unpack_from("<I6H6HHBB", data, polys_at + 32 * i)
            vi, neis, flags, nv, at = rest[0:6], rest[6:12], rest[12], rest[13], rest[14]
            vbase, tbase, dnv, dnt = struct.unpack_from("<IIBB", data, dm_at + 12 * i)

            def vert(k):
                src, j = (verts, vi[k]) if k < nv else (dverts, vbase + k - nv)
                return src[3 * j], src[3 * j + 2], src[3 * j + 1]  # Recast (x, up, y) -> CARLA (x, y, z)
            dtris = [tuple(vert(data[dt_at + 4 * (tbase + t) + q]) for q in range(3)) for t in range(dnt)]
            polys.append((at & 0x3F, flags, [vert(k) for k in range(nv)], dtris, neis[:nv]))
        tiles.append(polys)
        pos += size
    assert pos == len(data), "trailing bytes"
    return dict(tiles=ntiles, origin=(ox, oz, oy), tile_size=tw, max_tiles=max_tiles, max_polys=max_polys), tiles


def height_at(dtris, x, y):
    for a, b, c in dtris:
        d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
        if abs(d) < 1e-12:
            continue
        u = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / d
        v = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / d
        if u >= -1e-6 and v >= -1e-6 and u + v <= 1 + 1e-6:
            return u * a[2] + v * b[2] + (1 - u - v) * c[2]
    return None


def check_heights(tiles, samples):
    """Navmesh (detail mesh) height minus 3D surface height at lane centres and crosswalk centres."""
    grid = {}
    for polys in tiles:
        for area, _, vs, dtris, _ in polys:
            xs, ys = [v[0] for v in vs], [v[1] for v in vs]
            for gx in range(int(min(xs) // 5), int(max(xs) // 5) + 1):
                for gy in range(int(min(ys) // 5), int(max(ys) // 5) + 1):
                    grid.setdefault((gx, gy), []).append((area, dtris))
    out = {}
    for kind, x, y, z in samples:
        hs = [(h, area) for area, dtris in grid.get((int(x // 5), int(y // 5)), ())
              for h in [height_at(dtris, x, y)] if h is not None]
        r = out.setdefault(kind, dict(n=0, found=0, diffs=[], areas={}))
        r["n"] += 1
        if hs:
            h, area = min(hs, key=lambda t: abs(t[0] - z))  # nearest layer (bridges)
            r["found"] += 1
            r["diffs"].append(h - z)
            r["areas"][AREA.get(area, area)] = r["areas"].get(AREA.get(area, area), 0) + 1
    return out


COLOURS = {1: (110, 160, 215), 2: (255, 200, 0), 3: (95, 95, 95), 4: (90, 170, 90), 63: (215, 120, 120)}


def render(path, tiles, cross, x0, y0, x1, y1, res):
    """Top-down PNG, north up (CARLA y points south = image down), res metres per pixel; crosswalk outlines red."""
    w, h = int((x1 - x0) / res), int((y1 - y0) / res)
    img = bytearray(b"\xff" * (w * h * 3))

    def fill(vs, col):
        px = [((x - x0) / res, (y - y0) / res) for x, y in vs]
        for row in range(max(0, math.ceil(min(p[1] for p in px) - .5)), min(h, int(max(p[1] for p in px) - .5) + 1)):
            yc, xs = row + .5, []
            for (ax, ay), (bx, by) in zip(px, px[1:] + px[:1]):
                if (ay <= yc) != (by <= yc):
                    xs.append(ax + (yc - ay) * (bx - ax) / (by - ay))
            if len(xs) >= 2:
                a, b = max(0, math.ceil(min(xs) - .5)), min(w, int(max(xs) - .5) + 1)
                if b > a:
                    img[3 * (row * w + a):3 * (row * w + b)] = bytes(col) * (b - a)

    def line(p, q, col, width):
        n = int(max(abs(q[0] - p[0]), abs(q[1] - p[1])) / res) + 1
        for i in range(n + 1):
            cx, cy = (p[0] + (q[0] - p[0]) * i / n - x0) / res, (p[1] + (q[1] - p[1]) * i / n - y0) / res
            for dx in range(-width, width + 1):
                for dy in range(-width, width + 1):
                    if 0 <= int(cx) + dx < w and 0 <= int(cy) + dy < h:
                        k = 3 * ((int(cy) + dy) * w + int(cx) + dx)
                        img[k:k + 3] = bytes(col)

    for area in (63, 3, 4, 1, 2):  # sidewalks and crosswalks drawn over roads
        for polys in tiles:
            for a, _, vs, _, _ in polys:
                if a == area:
                    fill([(v[0], v[1]) for v in vs], COLOURS.get(a, (0, 0, 0)))
    for poly in cross:
        pts = [(p[0], p[1]) for p in poly]
        for p, q in zip(pts, pts[1:] + pts[:1]):
            line(p, q, (220, 0, 0), 0 if res < .2 else 1)
        if res >= .2:  # ring, so a 1 m crosswalk is findable on the overview
            cx, cy = sum(p[0] for p in pts) / 4, sum(p[1] for p in pts) / 4
            ring = [(cx + 25 * math.cos(a / 20 * math.pi), cy + 25 * math.sin(a / 20 * math.pi)) for a in range(41)]
            for p, q in zip(ring, ring[1:]):
                line(p, q, (220, 0, 0), 2)
    raw = b"".join(b"\x00" + bytes(img[3 * w * r:3 * w * (r + 1)]) for r in range(h))

    def chunk(t, d):
        return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xFFFFFFFF)
    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def validate(name, out, samples, cross):
    header, tiles = read_navmesh(os.path.join(out, name + ".bin"))
    print("navmesh: %d tiles of %.1f m (max %d tiles, %d polys/tile), origin %s" % (
        header["tiles"], header["tile_size"], header["max_tiles"], header["max_polys"],
        tuple(round(v, 1) for v in header["origin"])))
    count, area_m2, cw_next_to_sidewalk = {}, {}, 0
    for polys in tiles:
        for a, flags, vs, _, neis in polys:
            count[a] = count.get(a, 0) + 1
            area_m2[a] = area_m2.get(a, 0) + abs(sum(p[0] * q[1] - q[0] * p[1] for p, q in zip(vs, vs[1:] + vs[:1]))) / 2
            if a == 2 and any(0 < n < 0x8000 and polys[n - 1][0] == 1 for n in neis):
                cw_next_to_sidewalk += 1
    for a in sorted(count):
        print("  %-9s %6d polys %9.0f m2" % (AREA.get(a, a), count[a], area_m2[a]))
    print("  crosswalk polys with a sidewalk neighbour in the same tile: %d of %d" % (cw_next_to_sidewalk, count.get(2, 0)))
    print("height of navmesh minus 3D surface (lane/crosswalk centres; roads +%.4f by design):" % ROAD_LIFT)
    for kind, r in sorted(check_heights(tiles, samples).items()):
        d = sorted(abs(v) for v in r["diffs"]) or [float("nan")]
        print("  %-9s %5d points, %5d on the navmesh (%s), mean %+.3f m, median |d| %.3f m, 95%% %.3f m, max %.3f m" % (
            kind, r["n"], r["found"], ", ".join("%s %d" % kv for kv in sorted(r["areas"].items())),
            sum(r["diffs"]) / max(1, len(r["diffs"])), d[len(d) // 2], d[int(len(d) * .95)], d[-1]))
    xs = [v[0] for polys in tiles for p in polys for v in p[2]]
    ys = [v[1] for polys in tiles for p in polys for v in p[2]]
    render(os.path.join(out, "navmesh.png"), tiles, cross, min(xs) - 10, min(ys) - 10, max(xs) + 10, max(ys) + 10, 0.5)
    for i, poly in enumerate(cross):  # one close-up per crosswalk, 60 m at 0.1 m/px
        cx, cy = sum(p[0] for p in poly) / 4, sum(p[1] for p in poly) / 4
        render(os.path.join(out, "crosswalk_%d.png" % i), tiles, [poly], cx - 30, cy - 30, cx + 30, cy + 30, 0.1)
    print("wrote navmesh.png and crosswalk_0..%d.png (centres %s)" % (len(cross) - 1, [
        (round(sum(p[0] for p in c) / 4), round(sum(p[1] for p in c) / 4)) for c in cross]))


def inside(args):
    import carla
    global WAVES
    WAVES = args.name not in FLAT_MAPS
    print("road surface: %s" % ("stock tool waves" if WAVES else "flat (the road file's heights)"))
    out = "/nav/out"
    os.makedirs(out, exist_ok=True)
    text = open("/in/map.xodr").read()
    root = ET.fromstring(text)
    m = carla.Map(args.name, text)
    tris, samples, stats = build_triangles(m, root, Keepout("/in/map.keepout"))
    cross = crosswalks(m, root)
    print("lanes %(lanes)d (%(length).0f m), junction sidewalks skipped %(skipped_junction_sidewalks)d, "
          "triangles under buildings dropped %(cut_by_buildings)d" % stats)
    print("triangles: %s" % {k: sum(1 for t in tris if t[0] == k) for k in ("road", "sidewalk", "crosswalk")})
    if not args.skip_recast:
        obj = "/tmp/%s.obj" % args.name
        write_obj(obj, tris)
        log = subprocess.run(["/in/RecastBuilder", obj], cwd="/tmp", stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, universal_newlines=True).stdout
        print("\n".join(l for l in log.splitlines() if not l.startswith("TileRow")))
        shutil.copy("/tmp/%s.bin" % args.name, os.path.join(out, args.name + ".bin"))
    validate(args.name, out, samples, cross)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--xodr", default=os.path.join(INPUTS, "WolfsburgHighway.xodr"))
    ap.add_argument("--keepout", help="building outlines (default: <xodr>.keepout)")
    ap.add_argument("--recast", default=os.path.join(INPUTS, "RecastBuilder"), help="CARLA 0.9.15's RecastBuilder")
    ap.add_argument("--image", default="carla-client:0.9.15")
    ap.add_argument("--skip-recast", action="store_true", help="re-validate out/<map>.bin only")
    ap.add_argument("--inside", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--name", help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.inside:
        return inside(args)
    name = os.path.splitext(os.path.basename(args.xodr))[0]
    keepout = args.keepout or os.path.splitext(args.xodr)[0] + ".keepout"
    cmd = ["docker", "run", "--rm", "-v", HERE + ":/nav", "-v", os.path.abspath(args.xodr) + ":/in/map.xodr:ro",
           "-v", os.path.abspath(keepout) + ":/in/map.keepout:ro",
           "-v", os.path.abspath(args.recast) + ":/in/RecastBuilder:ro",
           args.image, "python", "-u", "/nav/build_navmesh.py", "--inside", "--name", name]
    sys.exit(subprocess.call(cmd + (["--skip-recast"] if args.skip_recast else [])))


if __name__ == "__main__":
    main()
