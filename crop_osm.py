"""Clip an OSM export to a bbox. OSM exports include whole ways that run far
past the bbox; this keeps only the road pieces inside it.

usage: python3 crop_osm.py in.osm out.osm minlat minlon maxlat maxlon [buildings]
With "buildings", building outlines lying wholly inside the bbox are kept too (the Digital Twin Tool reads them).
"""
import sys
import xml.etree.ElementTree as ET


def split_runs(refs, inside):
    """Split a way's node refs into runs of consecutive inside nodes (len >= 2)."""
    runs, cur = [], []
    for r in refs:
        if r in inside:
            cur.append(r)
        else:
            if len(cur) >= 2:
                runs.append(cur)
            cur = []
    if len(cur) >= 2:
        runs.append(cur)
    return runs


assert split_runs("abcdefg", set("abdefg")) == [list("ab"), list("defg")]
assert split_runs("abc", set("b")) == []


def main(src, dst, minlat, minlon, maxlat, maxlon, buildings=False):
    root = ET.parse(src).getroot()
    nodes = {n.get("id"): n for n in root.iter("node")}
    inside = {i for i, n in nodes.items()
              if minlat <= float(n.get("lat")) <= maxlat and minlon <= float(n.get("lon")) <= maxlon}
    next_id = max(int(w.get("id")) for w in root.iter("way")) + 1

    out = ET.Element("osm", version="0.6", generator="crop_osm.py")
    ET.SubElement(out, "bounds", minlat=str(minlat), minlon=str(minlon), maxlat=str(maxlat), maxlon=str(maxlon))
    ways, used = [], set()
    # ponytail: roads only (all CARLA's Osm2Odr reads); relations dropped since clipped ways break turn restrictions
    for w in root.iter("way"):
        tags = w.findall("tag")
        refs = [nd.get("ref") for nd in w.findall("nd")]
        if buildings and any(t.get("k") == "building" for t in tags) and all(r in inside for r in refs):
            ways.append(w)  # whole outline or nothing: a cut one would be a wall across a lawn
            used.update(refs)
            continue
        if not any(t.get("k") == "highway" for t in tags):
            continue
        for k, run in enumerate(split_runs(refs, inside)):
            nw = ET.Element("way", id=w.get("id") if k == 0 else str(next_id), version="1")
            next_id += k > 0
            for r in run:
                ET.SubElement(nw, "nd", ref=r)
            nw.extend(tags)
            ways.append(nw)
            used.update(run)
    out.extend(nodes[i] for i in nodes if i in used)  # keep file order; node tags (traffic signals) survive
    out.extend(ways)
    ET.ElementTree(out).write(dst, encoding="utf-8", xml_declaration=True)
    print(f"{len(used)} nodes, {len(ways)} ways -> {dst}")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2], *map(float, sys.argv[3:7]), buildings=sys.argv[7:] == ["buildings"])
