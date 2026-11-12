# Removes U-turns from an OpenDRIVE file. Usage: remove_uturns.py in.xodr out.xodr
# netconvert adds one at every junction. Autopilot takes them at full speed, and on Digital Twin maps, which have real
# kerbs, the car hits the kerb and stays there. A U-turn that is a lane's only exit stays, so no lane becomes a dead end.
import math
import re
import sys
import xml.etree.ElementTree as ET

UTURN_DEGREES = 150
SAME_POINT_METRES = 3


def first_heading(road):
    return float(road.find("planView").find("geometry").get("hdg"))


def turn_degrees(connector, outgoing):
    difference = first_heading(outgoing) - first_heading(connector)
    return abs(math.degrees(math.atan2(math.sin(difference), math.cos(difference))))


def ends(road):
    geometries = road.find("planView").findall("geometry")
    first, last = geometries[0], geometries[-1]
    heading, length = float(last.get("hdg")), float(last.get("length"))
    end = (float(last.get("x")) + length * math.cos(heading), float(last.get("y")) + length * math.sin(heading))
    return (float(first.get("x")), float(first.get("y"))), end


def is_uturn(connector, incoming, outgoing):
    # Some connectors are under a metre long, so their heading says nothing: the outgoing road running back along
    # the incoming one catches those.
    in_start, in_end = ends(incoming)
    out_start, out_end = ends(outgoing)
    same_street_back = math.dist(in_start, out_end) < SAME_POINT_METRES and math.dist(in_end, out_start) < SAME_POINT_METRES
    return same_street_back or turn_degrees(connector, outgoing) > UTURN_DEGREES


def choose_uturns(root):
    roads = {r.get("id"): r for r in root.findall("road")}
    remove = set()
    for junction in root.findall("junction"):
        exits = {}  # (incoming road, incoming lane) -> [(connector id, is it a U-turn)]
        for connection in junction.findall("connection"):
            connector = roads[connection.get("connectingRoad")]
            successor = connector.find("link").find("successor")
            uturn = is_uturn(connector, roads[connection.get("incomingRoad")], roads[successor.get("elementId")])
            for link in connection.findall("laneLink"):
                key = (connection.get("incomingRoad"), link.get("from"))
                exits.setdefault(key, []).append((connector.get("id"), uturn))
        keep = {cid for routes in exits.values() if all(u for _, u in routes) for cid, _ in routes}
        remove |= {cid for routes in exits.values() for cid, u in routes if u} - keep
    return remove


def strip(text, remove):
    for cid in remove:
        text, roads = re.subn(r'[ \t]*<road\b[^>]*\bid="%s"[^>]*>.*?</road>\n?' % cid, "", text, count=1, flags=re.S)
        text, connections = re.subn(r'[ \t]*<connection\b[^>]*\bconnectingRoad="%s"[^>]*>.*?</connection>\n?' % cid, "", text, flags=re.S)
        assert roads == 1 and connections >= 1, cid
    return text


def check(before, after, remove):
    old = ET.fromstring(before)
    new = ET.fromstring(after)
    assert len(old.findall("road")) - len(new.findall("road")) == len(remove)
    assert len(old.findall("junction")) == len(new.findall("junction"))
    assert not choose_uturns(new), "U-turns left that could have been removed"
    lanes = lambda root: {(j.get("id"), c.get("incomingRoad"), l.get("from"))
                          for j in root.findall("junction") for c in j.findall("connection") for l in c.findall("laneLink")}
    assert lanes(old) == lanes(new), "a lane lost its last exit"


if __name__ == "__main__":
    source, target = sys.argv[1:3]
    text = open(source, encoding="utf-8").read()
    remove = choose_uturns(ET.fromstring(text))
    result = strip(text, remove)
    check(text, result, remove)
    open(target, "w", encoding="utf-8").write(result)
    print("removed", len(remove), "U-turn roads of", len(ET.fromstring(text).findall("road")))
