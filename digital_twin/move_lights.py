# Moves a traffic light's stop line back off a road too short to stop on. Usage: move_lights.py in.xodr out.xodr
# netconvert splits some junctions in two with a stub road between them (WolfsburgHighway: Herzbergweg -> 1.5 m of
# Salzwedeler Strasse -> the lights). CARLA puts a light's trigger box on the road before the junction, so on a stub the
# box lies where cars leave the first junction at speed: they saw red 2 m past the line and stopped 4 m inside the
# junction (signals_check.py, light 74). The reference moves to the end of each road feeding the stub through the first
# junction, for the lanes that lead onto the stub, so cars stop before the first junction like at any other light.
# A stop or yield sign at that new stop line goes: the light governs it now, and the sign's trigger box can turn a car
# waiting at the red light green (Stop/YieldSignComponent::GiveWayIfPossible).
import re
import sys
import xml.etree.ElementTree as ET

STUB_METRES = 10.0  # a light's approach shorter than this is a stub
SIGN_METRES = 10.0  # a stop or yield sign this close to the new stop line goes
TRAFFIC_LIGHT = "1000001"


def choose_moves(root):
    """[(light id, connector the reference is on,
         [(feeding road, s at its junction end, lowest lane, highest lane, [stop or yield sign ids to drop])])]"""
    roads = {r.get("id"): r for r in root.findall("road")}
    junctions = {j.get("id"): j for j in root.findall("junction")}
    lights = {s.get("id") for s in root.iter("signal") if s.get("type") == TRAFFIC_LIGHT}
    moves = []
    for connector in root.findall("road"):
        refs = [r.get("id") for r in connector.findall("signals/signalReference") if r.get("id") in lights]
        link = connector.find("link/predecessor")
        if connector.get("junction") == "-1" or not refs or link is None or link.get("elementType") != "road":
            continue
        stub = roads[link.get("elementId")]
        if float(stub.get("length")) >= STUB_METRES:
            continue
        # the stub's other end, away from the light
        before = stub.find("link/predecessor" if link.get("contactPoint") == "end" else "link/successor")
        if before is None or before.get("elementType") != "junction":
            continue
        feeds = []
        for c in junctions[before.get("elementId")].findall("connection"):
            onto = roads[c.get("connectingRoad")].find("link/successor")
            if onto is None or onto.get("elementId") != stub.get("id"):
                continue
            feeder = roads[c.get("incomingRoad")]
            at_end = feeder.find("link/successor")
            at_end = at_end is not None and at_end.get("elementId") == before.get("elementId")
            section = feeder.findall("lanes/laneSection")[-1 if at_end else 0]
            driving = {l.get("id") for l in section.iter("lane") if l.get("type") == "driving"}
            lanes = [int(l.get("from")) for l in c.findall("laneLink") if l.get("from") in driving]
            if lanes:
                feeds.append((feeder.get("id"), float(feeder.get("length")) if at_end else 0.0, min(lanes), max(lanes)))
        merged = {}
        for road, s, lo, hi in feeds:  # one reference per feeding road, over all its lanes onto the stub
            _, _, a, b = merged.get(road, (road, s, lo, hi))
            merged[road] = (road, s, min(a, lo), max(b, hi))
        feeds = [(road, s, lo, hi, [g.get("id") for g in roads[road].findall("signals/signal")
                                   if g.get("type") in ("205", "206") and abs(float(g.get("s")) - s) <= SIGN_METRES])
                 for road, s, lo, hi in sorted(merged.values())]
        if feeds:
            moves += [(light, connector.get("id"), feeds) for light in refs]
    return moves


def block(text, road_id):
    m = re.search(r'<road\b[^>]*\bid="%s"[^>]*>.*?</road>' % road_id, text, flags=re.S)
    assert m, road_id
    return m


def apply(text, moves):
    for light, connector, feeds in moves:
        m = block(text, connector)
        road, n = re.subn(r'[ \t]*<signalReference\b[^>]*\bid="%s"[^>]*(/>|>.*?</signalReference>)\n?' % light, "",
                          m.group(0), count=1, flags=re.S)
        assert n == 1, (light, connector)
        text = text[:m.start()] + road + text[m.end():]
        for feeder, s, lo, hi, drops in feeds:
            m = block(text, feeder)
            ref = ('<signalReference id="%s" s="%.2f" t="0.00" orientation="%s"><validity fromLane="%d" toLane="%d" />'
                   '</signalReference>') % (light, s, "-" if hi < 0 else "+", lo, hi)
            road = m.group(0)
            if "</signals>" in road:
                road = road.replace("</signals>", ref + "</signals>", 1)
            else:
                road = road.replace("</road>", "<signals>" + ref + "</signals></road>", 1)
            for sign in drops:
                road, n = re.subn(r'<signal\b[^>]*?\bid="%s"[^>]*?(/>|>.*?</signal>)' % sign, "", road, count=1, flags=re.S)
                assert n == 1, (feeder, sign)
            text = text[:m.start()] + road + text[m.end():]
    return text


def check(before, after, moves):
    old, new = ET.fromstring(before), ET.fromstring(after)
    refs = lambda root: sorted((r.get("id"), road.get("id")) for road in root.findall("road")
                               for r in road.findall("signals/signalReference"))
    expected = sorted(set(refs(old)) - {(l, c) for l, c, _ in moves} | {(l, f) for l, _, fs in moves for f, *_ in fs})
    assert refs(new) == expected, "references moved wrongly"
    assert len(old.findall("road")) == len(new.findall("road"))
    signals = lambda root: [(s.attrib, [v.attrib for v in s]) for s in root.iter("signal")]
    dropped = {sign for _, _, fs in moves for *_, drops in fs for sign in drops}
    assert signals(new) == [s for s in signals(old) if s[0]["id"] not in dropped], "a signal changed"
    assert not choose_moves(new), "a light is still on a stub"


if __name__ == "__main__":
    source, target = sys.argv[1:3]
    text = open(source, encoding="utf-8").read()
    moves = choose_moves(ET.fromstring(text))
    result = apply(text, moves)
    check(text, result, moves)
    open(target, "w", encoding="utf-8").write(result)
    for light, connector, feeds in moves:
        print(f"light {light}: stop line moved from connector {connector} to the end of road(s)",
              ", ".join(f"{f} (s {s:.1f}, lanes {lo}..{hi}{', dropped sign ' + ' '.join(d) if d else ''})"
                        for f, s, lo, hi, d in feeds))
    print("moved", len(moves), "of", len({s.get("id") for s in ET.fromstring(text).iter("signal") if s.get("type") == TRAFFIC_LIGHT}), "lights")
