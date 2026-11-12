#!/usr/bin/env python3
"""Live check of the WolfsburgHighway navmesh on a running CARLA 0.9.15 server.

Loads the map (or uses the loaded one with --no-load), sets tile_stream_distance 600 / actor_active_distance 450,
samples get_random_location_from_navigation() near the hero (or near a crosswalk the spectator glides to), spawns
AI walkers with pedestrians_cross_factor 0 (so they may only use sidewalks and crosswalks), sends some across a
crosswalk and the rest to random sidewalk points, and reports where they walked. No sensors; everything stays within
~300 m; the spectator moves at most 150 m at a time. Destroys what it spawned unless --keep.

  python3 check_navmesh_live.py [--no-load] [--crosswalk 0] [--walkers 8] [--seconds 60]
"""
import argparse
import math
import os
import random
import sys
import time

import carla


def crosswalk_polygons(m):
    pts = [(p.x, p.y) for p in m.get_crosswalks()]
    return [pts[i:i + 4] for i in range(0, len(pts) - 3, 4)]  # 4 unclosed corners each on this map


def in_poly(x, y, poly):
    hit = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


def where(m, loc, polys):
    if any(in_poly(loc.x, loc.y, p) for p in polys):
        return "crosswalk"
    ground = carla.Location(loc.x, loc.y, loc.z - 0.9)  # walker locations are the capsule centre
    for name, lane in (("sidewalk", carla.LaneType.Sidewalk), ("road", carla.LaneType.Driving)):
        if m.get_waypoint(ground, project_to_road=False, lane_type=lane) is not None:
            return name
    return "off-lane"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--map", default="WolfsburgHighway")
    ap.add_argument("--no-load", action="store_true", help="use the map that is already loaded")
    ap.add_argument("--crosswalk", type=int, default=0, help="crosswalk to test when there is no hero (0..5)")
    ap.add_argument("--walkers", type=int, default=8, help="random sidewalk walkers, besides 2 crossing ones")
    ap.add_argument("--radius", type=float, default=150.0, help="m around the hero/crosswalk for random points")
    ap.add_argument("--seconds", type=float, default=60.0, help="simulated seconds to watch")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--keep", action="store_true", help="leave the walkers running")
    args = ap.parse_args()
    random.seed(args.seed)

    client = carla.Client(args.host, args.port)
    client.set_timeout(300.0)
    world = client.load_world(args.map) if not args.no_load else client.get_world()
    s = world.get_settings()
    s.tile_stream_distance = 600.0  # right after load_world: loading resets it, and large values crashed the server
    s.actor_active_distance = 450.0
    world.apply_settings(s)
    sync = s.synchronous_mode

    def step(n=1):
        for _ in range(n):
            world.tick() if sync else world.wait_for_tick()

    m = world.get_map()
    print("map %s: the server looks for %s.bin anywhere under CarlaUE4/Content (first match)" % (
        m.name, os.path.basename(m.name)))
    polys = crosswalk_polygons(m)
    heroes = [a for a in world.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "hero"]
    if heroes:
        centre = heroes[0].get_location()
        print("hero %d at (%.0f, %.0f)" % (heroes[0].id, centre.x, centre.y))
    else:
        poly = polys[args.crosswalk]
        centre = carla.Location(sum(p[0] for p in poly) / 4, sum(p[1] for p in poly) / 4, 0.0)
        spectator = world.get_spectator()
        here = spectator.get_location()
        legs = max(1, int(math.ceil(math.hypot(centre.x - here.x, centre.y - here.y) / 150.0)))
        print("no hero: gliding the spectator to crosswalk %d at (%.0f, %.0f) in %d legs" % (
            args.crosswalk, centre.x, centre.y, legs))
        for i in range(1, legs + 1):
            f = float(i) / legs
            spot = carla.Location(here.x + (centre.x - here.x) * f, here.y + (centre.y - here.y) * f, centre.z + 60)
            spectator.set_transform(carla.Transform(spot, carla.Rotation(pitch=-60)))
            t = world.get_snapshot().timestamp.elapsed_seconds
            while world.get_snapshot().timestamp.elapsed_seconds - t < 2.0:
                step()

    points = [world.get_random_location_from_navigation() for _ in range(20000)]
    valid = [p for p in points if p is not None]
    near = [p for p in valid if p.distance(centre) < args.radius]
    if not valid:
        print("FAIL: get_random_location_from_navigation() returned None: the client has no navmesh "
              "(file missing or misnamed; it is read when the client first sees the episode)")
        return 2
    kinds = {}
    for p in near[:300]:
        k = where(m, carla.Location(p.x, p.y, p.z + 0.9), polys)
        kinds[k] = kinds.get(k, 0) + 1
    print("navigation points: %d of %d valid, %d within %.0f m; lane type under 300 of them: %s" % (
        len(valid), len(points), len(near), args.radius, kinds))

    plan = []  # (label, start, target)
    poly = min(polys, key=lambda q: math.hypot(sum(p[0] for p in q) / 4 - centre.x, sum(p[1] for p in q) / 4 - centre.y))
    a = max(((p, q) for p in poly for q in poly), key=lambda pq: math.hypot(pq[0][0] - pq[1][0], pq[0][1] - pq[1][1]))
    ends = []
    for (x, y), (ox, oy) in (a, a[::-1]):  # 3 m past each end of the long axis, snapped to the nearest sidewalk
        d = math.hypot(x - ox, y - oy)
        loc = carla.Location(x + (x - ox) / d * 3, y + (y - oy) / d * 3, 0)
        wp = m.get_waypoint(loc, project_to_road=True, lane_type=carla.LaneType.Sidewalk)
        ends.append(wp.transform.location if wp else None)
    if all(ends) and ends[0].distance(centre) < 300 and ends[0].distance(ends[1]) > 4:
        plan += [("crossing", ends[0], ends[1]), ("crossing", ends[1], ends[0])]
    else:
        print("crosswalk nearest the centre has no sidewalk at both ends within reach: no crossing walkers")
    for _ in range(min(args.walkers, len(near) // 2)):
        plan.append(("random", random.choice(near), random.choice(near)))

    world.set_pedestrians_cross_factor(0.0)  # filter 0: sidewalks and crosswalks only (Navigation::AddWalker)
    bps = world.get_blueprint_library().filter("walker.pedestrian.*")
    batch = []
    for _, start, _ in plan:
        bp = random.choice(bps)
        if bp.has_attribute("is_invincible"):
            bp.set_attribute("is_invincible", "false")
        batch.append(carla.command.SpawnActor(bp, carla.Transform(start + carla.Location(z=1.0))))
    res = client.apply_batch_sync(batch, sync)
    walkers = [(r.actor_id, p) for r, p in zip(res, plan) if not r.error]
    print("spawned %d of %d walkers%s" % (len(walkers), len(plan), "".join(
        "; " + r.error for r in res if r.error)[:300]))
    step(2)
    ctrl_bp = world.get_blueprint_library().find("controller.ai.walker")
    res = client.apply_batch_sync([carla.command.SpawnActor(ctrl_bp, carla.Transform(), wid) for wid, _ in walkers], sync)
    ctrls = [r.actor_id for r in res if not r.error]
    step(2)
    actors = {a.id: a for a in world.get_actors([wid for wid, _ in walkers] + ctrls)}
    try:
        for (wid, (label, start, target)), cid in zip(walkers, ctrls):
            c = actors[cid]
            c.start()
            c.go_to_location(target)
            c.set_max_speed(1.4)
        log = {wid: [] for wid, _ in walkers}
        t0 = world.get_snapshot().timestamp.elapsed_seconds
        last = t0
        while True:
            step()
            now = world.get_snapshot().timestamp.elapsed_seconds
            if now - last >= 0.5:
                last = now
                for wid, _ in walkers:
                    loc = actors[wid].get_location()
                    log[wid].append((loc, where(m, loc, polys)))
            if now - t0 >= args.seconds:
                break
        total, moving, arrived = {}, 0, 0
        print("walker  kind      start->target  walked  left  where (share of samples)")
        for wid, (label, start, target) in walkers:
            locs = [l for l, _ in log[wid]]
            walked = sum(p.distance(q) for p, q in zip(locs, locs[1:]))
            left = math.hypot(locs[-1].x - target.x, locs[-1].y - target.y) if locs else float("nan")
            share = {}
            for _, k in log[wid]:
                share[k] = share.get(k, 0) + 1
                total[k] = total.get(k, 0) + 1
            moving += walked > 3
            arrived += left < 2.5
            print("%6d  %-8s  %8.1f m  %6.1f m %5.1f m  %s" % (
                wid, label, start.distance(target), walked, left,
                ", ".join("%s %.0f%%" % (k, 100.0 * v / len(log[wid])) for k, v in sorted(share.items()))))
        n = sum(total.values()) or 1
        print("SUMMARY: %d of %d walkers moved > 3 m, %d arrived; samples: %s" % (
            moving, len(walkers), arrived, ", ".join("%s %.1f%%" % (k, 100.0 * v / n) for k, v in sorted(total.items()))))
        return 0 if walkers and moving == len(walkers) else 1
    finally:
        if not args.keep:
            for cid in ctrls:
                actors[cid].stop()
            client.apply_batch_sync([carla.command.DestroyActor(x) for x in ctrls + [w for w, _ in walkers]], sync)


if __name__ == "__main__":
    sys.exit(main())
