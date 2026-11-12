"""Drives one car through every highway on-ramp and exit of a map, and over every bridge, and flags launches, falls
and stops, so the map is known to be fit for merge testing. CARLA's window follows the car.

usage: merge_test.py [Name [roads]]  in the container on the VM, after `vm.sh check Name` (default WolfsburgHighway);
                                     roads (e.g. 3205,3208) runs only the routes that start on them
       merge_test.py --plan map.xodr  only plans and prints the routes; needs no simulator
Exit code 1 if any route fails.
"""
import math
import sys
import xml.etree.ElementTree as ET

import carla

STEP, BEFORE, AFTER = 2.0, 150.0, 300.0  # m: route point spacing; highway before an exit; road after the ramp ends
SLOW, FAST = 13.9, 22.2                  # m/s: 50 km/h on ramps, 80 km/h on the motorway and the trunk road
LATERAL = 2.5                            # m/s2: curves are taken no faster than this sideways (a calm driver)
EDGE = 40.0                              # m: no route starts this close to the edge of the road network
HIGH, LOW = 1.5, -2.5                    # m off the map data's road height that counts as a launch or a fall (the
                                         # road surface itself waves about 1.2 m around it: Deformation.h)
HIGHWAY = ("motorway", "trunk")
import os
TRACE = open(os.environ["MERGE_TRACE"], "w") if os.environ.get("MERGE_TRACE") else None  # per-step CSV for debugging
DT, WHEELBASE = 0.05, 2.8


def wrap(d):
    return (d + 180) % 360 - 180


def kinds(xodr):
    """road id -> highway class from load_osm.py's names ("" for junction pieces), road lengths and top heights."""
    roads = list(ET.fromstring(xodr).iter("road"))
    return ({int(r.get("id")): r.get("name").split("|")[0] if "|" in r.get("name", "") else "" for r in roads},
            {int(r.get("id")): float(r.get("length")) for r in roads},
            {int(r.get("id")): max([float(e.get("a")) for e in r.iter("elevation")] or [0]) for r in roads})


def kind_at(cls, wp, ahead=0):
    """Class of the road under wp, looking past junction pieces (they have none)."""
    for _ in range(20):
        if cls.get(wp.road_id):
            return cls[wp.road_id]
        nxt = wp.next(STEP) if ahead >= 0 else wp.previous(STEP)
        if not nxt:
            return ""
        wp = nxt[0]
    return ""


def is_ramp(c):
    return c.endswith("_link")


def follow(cls, wp, metres, take_exit):
    """Route points from wp. At a split it goes straight on (judged 50 m on), or onto the ramp when take_exit."""
    points = [wp]
    while len(points) * STEP < metres:
        options = wp.next(STEP)
        if not options:
            break
        if len(options) > 1:
            yaw = wp.transform.rotation.yaw
            ramps = [o for o in options if is_ramp(kind_at(cls, o))]
            if take_exit and ramps and not is_ramp(kind_at(cls, wp)):
                options, take_exit = ramps, False
            wp = min(options, key=lambda o: abs(wrap((o.next(50) or [o])[0].transform.rotation.yaw - yaw)))
        else:
            wp = options[0]
        points.append(wp)
    return points


def plan(town, xodr):
    """One route per ramp: an on-ramp from its start onto the highway, an exit from BEFORE m up the highway."""
    cls, length, top = kinds(xodr)
    routes = []
    for rid, c in sorted(cls.items()):
        if not is_ramp(c):
            continue
        for lane in (-1, -2, 1, 2):
            wp = town.get_waypoint_xodr(rid, lane, 0.5 if lane < 0 else length[rid] - 0.5)
            if wp is None or wp.lane_type != carla.LaneType.Driving:
                continue
            back = wp.previous(STEP)
            if back and all(is_ramp(kind_at(cls, b, -1)) for b in back):
                continue  # the middle of a ramp: its start is planned already
            came = kind_at(cls, back[0], -1) if back else ""
            if came in HIGHWAY:  # an exit: start on the highway before it
                start = wp
                for _ in range(int(BEFORE / STEP)):
                    prev = [p for p in start.previous(STEP) if kind_at(cls, p, -1) in HIGHWAY or not cls.get(p.road_id)]
                    if not prev:
                        break
                    start = prev[0]
                for _ in range(25):  # never start inside a junction piece: some of them jump sideways
                    prev = start.previous(STEP)
                    if not start.is_junction or not prev:
                        break
                    start = prev[0]
                path = follow(cls, start, BEFORE + 400 + AFTER, True)
                name = "exit from %s" % came
            else:
                path = follow(cls, wp, 400 + AFTER, False)
                name = "on-ramp to %s" % (next((kind_at(cls, p) for p in path if kind_at(cls, p) in HIGHWAY), "no highway"))
            if any(kind_at(cls, p) in HIGHWAY for p in path):
                routes.append((name, rid, lane, path))
    for rid in sorted(r for r, z in top.items() if z >= 4.0 and cls[r] and not is_ramp(cls[r])):  # bridges
        wp = town.get_waypoint_xodr(rid, -1, length[rid] / 2)
        if wp is None or any(p.transform.location.distance(wp.transform.location) < 10 for r in routes for p in r[3]):
            continue  # a route already crosses it
        start = wp
        for _ in range(int(BEFORE / STEP)):
            prev = start.previous(STEP)
            if not prev:
                break
            start = prev[0]
        routes.append(("bridge on %s" % cls[rid], rid, -1, follow(cls, start, 2 * BEFORE, False)))
    return routes, cls


class Driver:
    """Pure pursuit along the route, at SLOW on ramps and FAST on the highway, slowing ahead of curves."""

    def __init__(self, car, path, speeds):
        self.car, self.path, self.speeds, self.i = car, path, speeds, 0
        self.max_steer = car.get_physics_control().wheels[0].max_steer_angle

    def step(self):
        t = self.car.get_transform()
        here = t.location
        while self.i < len(self.path) - 1 and \
                here.distance(self.path[self.i + 1].transform.location) < here.distance(self.path[self.i].transform.location):
            self.i += 1
        v = self.car.get_velocity()
        speed = math.hypot(v.x, v.y)
        ahead = max(5.0, 0.5 * speed)  # short: on tight raised ramps a longer look cut the curve off the edge
        target = self.path[min(self.i + int(ahead / STEP), len(self.path) - 1)].transform.location
        alpha = math.atan2(target.y - here.y, target.x - here.x) - math.radians(t.rotation.yaw)
        steer = math.degrees(math.atan2(2 * WHEELBASE * math.sin(alpha), ahead)) / self.max_steer
        want = min(self.speeds[self.i:self.i + 25]) if self.i < len(self.path) - 10 else 0.0
        error = want - speed
        throttle = min(max(0.35 + 0.25 * error, 0.0), 1.0) if want > 0 else 0.0
        brake = 1.0 if want == 0 else min(max(-0.3 * error - 0.3, 0.0), 1.0)
        self.car.apply_control(carla.VehicleControl(throttle=throttle, steer=min(max(steer, -1.0), 1.0), brake=brake))
        return speed, want


def curve_speed(path, j, span=5):
    """m/s at which the curve around path[j] (heading change over 2 x span points) gives LATERAL m/s2."""
    a, b = path[max(0, j - span)], path[min(len(path) - 1, j + span)]
    turn = abs(math.radians(wrap(b.transform.rotation.yaw - a.transform.rotation.yaw)))
    length = max(a.transform.location.distance(b.transform.location), STEP)
    return math.sqrt(LATERAL * length / turn) if turn > 1e-3 else 1e9


def continuous(path):
    """No jump between route points and no reversal: a route through a junction piece can come out broken."""
    # some junction pieces from netconvert step 3.5 to 5 m sideways between lane sections: a driver copes with that
    return all(a.transform.location.distance(b.transform.location) < 6.0 and
               abs(wrap(a.transform.rotation.yaw - b.transform.rotation.yaw)) < 60 for a, b in zip(path, path[1:]))


def drive(world, blueprint, cls, path):
    """Drives the route once; returns (worst height above, worst below, longest stop in s, metres driven, note)."""
    start = path[0].transform
    car = None
    for lift in (0.5, 1.0, 1.5, 2.0, 2.5):  # the road surface waves about 1.2 m around the map data's height
        car = world.try_spawn_actor(blueprint, carla.Transform(start.location + carla.Location(z=lift), start.rotation))
        if car:
            break
    if car is None:
        return 0, 0, 0, 0, "spawn blocked: something stands on the lane"
    try:
        speeds = [min(FAST if kind_at(cls, p) in HIGHWAY else SLOW, curve_speed(path, j)) for j, p in enumerate(path)]
        d = Driver(car, path, speeds)
        for _ in range(400):  # until its tile has streamed in: until then the car is dormant, with no physics
            world.tick()
            if car.is_active:
                break
        for _ in range(40):
            world.tick()  # settle on the road
        above = below = stopped = longest = 0.0
        stuck = ""
        last_speed = None
        for _ in range(int(1.6 * sum(STEP / s for s in speeds) / DT)):
            speed, want = d.step()
            world.tick()
            here = car.get_transform()
            v = car.get_velocity()
            now = math.hypot(v.x, v.y)
            # over 15 km/h only: a car settling on the road just after spawning loses a few km/h without hitting anything
            if not stuck and last_speed is not None and last_speed > 4.2 and last_speed - now > 0.6 and car.get_control().brake < 0.1:
                w = world.get_map().get_waypoint(here.location)  # over 12 m/s2 without braking: it hit something
                stuck = "hit something at (%.1f, %.1f, %.1f) road %d lane %d%s, %.0f -> %.0f km/h" % (
                    here.location.x, here.location.y, here.location.z, w.road_id, w.lane_id,
                    " (junction)" if w.is_junction else "", last_speed * 3.6, now * 3.6)
            last_speed = now
            road = path[d.i].transform.location
            dz = here.location.z - road.z
            if TRACE:
                TRACE.write("%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d\n" % (
                    path[0].road_id, d.i, here.location.x, here.location.y, here.location.z, road.x, road.y, road.z,
                    speed * 3.6, here.rotation.pitch, path[d.i].road_id))
            if not stuck and dz > HIGH:
                stuck = "left the road at (%.1f, %.1f, %.1f), %.1f m over its height" % (here.location.x, here.location.y, here.location.z, dz)
            above, below = max(above, dz), min(below, dz)
            stopped = stopped + DT if speed < 2.0 and want > 5.0 else 0.0
            longest = max(longest, stopped)
            if stopped > 3.0 and not stuck:
                w = world.get_map().get_waypoint(here.location)
                stuck = "stuck at (%.1f, %.1f, %.1f) road %d lane %d%s, %.1f m off the road height, steer %.2f" % (
                    here.location.x, here.location.y, here.location.z, w.road_id, w.lane_id,
                    " (junction)" if w.is_junction else "", dz, car.get_control().steer)
            world.get_spectator().set_transform(carla.Transform(
                here.location - here.get_forward_vector() * 14 + carla.Location(z=6), carla.Rotation(pitch=-18, yaw=here.rotation.yaw)))
            if d.i >= len(path) - 12 or dz > 6 or dz < -6:
                break
        return above, below, longest, d.i * STEP, stuck
    finally:
        car.destroy()


def main():
    if sys.argv[1:2] == ["--plan"]:
        xodr = open(sys.argv[2]).read()
        routes, cls = plan(carla.Map("plan", xodr), xodr)
        for name, rid, lane, path in routes:
            a, b = path[0].transform.location, path[-1].transform.location
            print("%-24s road %5d lane %2d: %4.0f m, (%5.0f, %6.0f) -> (%5.0f, %6.0f), highest %.1f m" % (
                name, rid, lane, len(path) * STEP, a.x, a.y, b.x, b.y, max(p.transform.location.z for p in path)))
        print(len(routes), "routes")
        return
    name = sys.argv[1] if len(sys.argv) > 1 else "WolfsburgHighway"
    only = {int(r) for r in sys.argv[2].split(",")} if len(sys.argv) > 2 else None  # e.g. 3205,3208: just those routes
    client = carla.Client("localhost", 2000)
    client.set_timeout(120)
    world = client.get_world()
    town = world.get_map()
    assert town.name.endswith(name), "load the map first: vm.sh check " + name
    routes, cls = plan(town, town.to_opendrive())
    routes = [r for r in routes if only is None or r[1] in only]
    xs = [w.transform.location for w in town.generate_waypoints(20.0)]
    lo_x, hi_x, lo_y, hi_y = min(p.x for p in xs), max(p.x for p in xs), min(p.y for p in xs), max(p.y for p in xs)
    keep = []
    for r in routes:
        a = r[3][0].transform.location
        if not continuous(r[3]):
            print("skip %-24s road %5d lane %2d: the planned route jumps" % r[:3])
        elif min(a.x - lo_x, hi_x - a.x, a.y - lo_y, hi_y - a.y) < EDGE:
            print("skip %-24s road %5d lane %2d: starts at the edge of the map" % r[:3])
        else:
            keep.append(r)
    routes = keep
    original = world.get_settings()
    s = world.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, DT
    world.apply_settings(s)
    blueprint = world.get_blueprint_library().find("vehicle.tesla.model3")
    blueprint.set_attribute("role_name", "hero")  # tiles load around it
    failed = 0
    try:
        # nearest next, so the car never jumps far across the map (long jumps crashed CARLA on Wolfsburg2)
        here, todo = carla.Location(), list(routes)
        while todo:
            r = min(todo, key=lambda r: r[3][0].transform.location.distance(here))
            todo.remove(r)
            label, rid, lane, path = r
            here = path[0].transform.location
            above, below, stop, driven, note = drive(world, blueprint, cls, path)
            bad = note or above > HIGH or below < LOW or stop > 3.0 or driven < 0.9 * (len(path) - 12) * STEP
            failed += bool(bad)
            print("%-4s %-24s road %5d lane %2d: %4.0f of %4.0f m, %+.1f / %+.1f m off the road height, stopped %.0f s %s" % (
                "FAIL" if bad else "ok", label, rid, lane, driven, len(path) * STEP, above, below, stop, note), flush=True)
    finally:
        world.apply_settings(original)
    print("%d of %d routes failed" % (failed, len(routes)))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
