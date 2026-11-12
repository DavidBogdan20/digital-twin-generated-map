"""Are there traffic lights and signs on the map, and does an autopilot (Traffic Manager) car obey them?
1. inventory: the traffic.* actors the server spawned from the road file's signals, against the road file's counts,
   and the lanes each traffic light controls;
2. each traffic light held red: an autopilot car starting 40 m before its stop line must stop without entering the
   junction, then drive on when the light turns green;
3. each stop sign: an autopilot car must come to a full stop before the junction, then go on;
4. speed limits: one autopilot car drives for a few minutes from the motorway; each change of the limit the car is
   given (set by the speed signs it passes) must happen at a sign of that value, and its speed must keep to the limit.
usage: signals_check.py [minutes for 4, default 3] [map, default WolfsburgHighway] [parts, default lights,stops,speed]
Each light is tested twice: at the speed the car gets from its limit, and at 50 km/h (Traffic Manager's speed on a 50 road).
In the container with the server running and no other client: loads the map if another one is up, runs synchronous at
20 Hz with the Traffic Manager on 8100, and restores the settings. Part 4 is logged to /work/logs/signals_check.csv.
How the 0.9.15 Traffic Manager obeys (LibCarla trafficmanager/TrafficLightStage.cpp, Carla/Traffic/*SignComponent.cpp):
a red or yellow light stops it once the car is in the light's trigger box; a stop or yield sign's trigger box turns the
car's light state red until the junction is free (stop: after at least 2 s at a standstill, yield: only when another car
is coming); its target speed is the limit the last speed sign set (30 km/h before any; 0.9.15's default is 0 % below).
"""
import collections
import csv
import sys

import carla

MINUTES = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
MAP = sys.argv[2] if len(sys.argv) > 2 else "WolfsburgHighway"
PARTS = sys.argv[3].split(",") if len(sys.argv) > 3 else ["lights", "stops", "speed"]
TM_PORT = 8100
DT = 0.05
BP = "vehicle.tesla.model3"
APPROACH = 40.0  # [m] the car starts this far before a light or stop sign
KIND = {"1000001": "traffic_light", "206": "stop", "205": "yield", "274": "speed_limit"}
RED = carla.TrafficLightState.Red


def front(car):
    t = car.get_transform()
    return t.location + t.get_forward_vector() * car.bounding_box.extent.x


def ahead_of(loc, wp):
    """how far loc lies before wp along wp's lane [m]; negative: past it"""
    f, d = wp.transform.get_forward_vector(), wp.transform.location - loc
    return f.x * d.x + f.y * d.y


def sign_lanes(m, landmark):
    """waypoints at the sign on each driving lane it applies to"""
    out = []
    for a, b in landmark.get_lane_validities():
        for lane in range(min(a, b), max(a, b) + 1):
            wp = m.get_waypoint_xodr(landmark.road_id, lane, landmark.s) if lane else None
            if wp is not None and wp.lane_type == carla.LaneType.Driving:
                out.append(wp)
    return out


def stop_lanes(m, landmark):
    """(waypoint at the sign, first waypoint inside the junction) for each lane the stop sign applies to that reaches a
    junction within 60 m"""
    out = []
    for wp in sign_lanes(m, landmark):
        ahead = wp
        for _ in range(30):
            ahead = (ahead.next(2.0) or [None])[0]
            if ahead is None or ahead.is_junction:
                break
        if ahead is not None and ahead.is_junction:
            out.append((wp, ahead))
    return out


def where(loc):
    return f"({loc.x:.0f}, {loc.y:.0f}, z {loc.z:.1f})"


def spawn(world, wp):
    t = wp.transform
    t.location.z += 0.5
    car = world.try_spawn_actor(world.get_blueprint_library().find(BP), t)
    if car is not None:
        car.set_autopilot(True, TM_PORT)
        world.tick()
    return car


def inventory(world, m):
    marks = {l.id: l for l in m.get_all_landmarks()}.values()  # a signal used on several roads is listed once per road
    want = collections.Counter(KIND.get(l.type, l.type) + (f".{l.value:.0f}" if l.type == "274" else "") for l in marks)
    have = collections.Counter(a.type_id.replace("traffic.", "") for a in world.get_actors().filter("traffic.*"))
    print("1. inventory (road file -> spawned in the server)")
    for k in sorted(set(want) | set(have)):
        print(f"   {k:<20} {want.get(k, 0):4d} -> {have.get(k, 0):4d}")
    print("   (the server picks the nearest speed sign model for a value it has no model for; the limit set is the "
          "road file's value)")
    lights = sorted(world.get_actors().filter("traffic.traffic_light"), key=lambda a: a.id)
    for tl in lights:
        loc = tl.get_location()
        print(f"   light {tl.id} (road file {tl.get_opendrive_id()}) at ({loc.x:.0f}, {loc.y:.0f}): "
              f"{len(tl.get_group_traffic_lights())} in its group, controls {len(tl.get_stop_waypoints())} lane(s)")
    return lights


def light_test(world, m, tm, tl, kmh=None):
    stops = tl.get_stop_waypoints()
    if not stops:
        return "FAIL: controls no lane, so no car ever sees it"
    stop = stops[0]
    tl.set_state(RED)
    car = spawn(world, (stop.previous(APPROACH) or [stop])[0])
    if car is None:
        return "skipped: could not spawn a car"
    tm.set_path(car, [stop.transform.location, (stop.next(20.0) or [stop])[0].transform.location])  # through this light
    if kmh:
        tm.set_desired_speed(car, kmh)
    try:
        seen, moved, stood, entered, gap, box, closest = False, False, 0.0, False, None, None, 1e9
        f = front(car)
        for _ in range(int(25 / DT)):
            world.tick()
            if not car.is_alive:
                return f"FAIL: the car vanished at {where(f)}"
            f, v = front(car), car.get_velocity().length()
            other = car.get_traffic_light()
            seen |= car.is_at_traffic_light() and other is not None and other.id == tl.id
            if box is None and car.is_at_traffic_light():
                box = (ahead_of(f, stop), v * 3.6, other.id if other else None)
            closest = min(closest, ahead_of(f, stop))
            entered |= ahead_of(f, stop) < 0 and m.get_waypoint(f).is_junction
            moved |= v > 2.0
            if moved and v < 0.05:
                stood += DT
                gap = ahead_of(f, stop)
                if stood >= 2.0:
                    break
            else:
                stood = 0.0
        red_ok = stood >= 2.0 and not entered
        tl.set_state(carla.TrafficLightState.Green)
        went, held = False, car.get_location()
        for _ in range(int(15 / DT)):
            world.tick()
            if not car.is_alive:
                return f"FAIL: the car vanished at {where(f)} after the light turned green"
            f = front(car)
            if car.get_location().distance(held) > 10.0:
                went = True
                break
        verdict = "PASS" if red_ok and went else "FAIL"
        stop_where = f"stopped {gap:+.1f} m before the stop line" if red_ok else (
            "entered the junction on red" if entered else "did not stop within 25 s")
        trig = (f"reached light {box[2]}'s trigger box {box[0]:+.1f} m before the line at {box[1]:.0f} km/h" if box
                else "never in a light's trigger box")
        return (f"{verdict}: red: {stop_where}, saw the light {seen}; green: drove on {went}. Details: {trig}, "
                f"closest {closest:+.1f} m, ended at {where(f)}, stop line at {where(stop.transform.location)}")
    finally:
        if car.is_alive:
            car.destroy()


def stop_test(world, m, tm, wp, entry):
    car = spawn(world, (wp.previous(APPROACH) or [wp])[0])
    if car is None:
        return "skipped: could not spawn a car"
    tm.set_path(car, [entry.transform.location, (entry.next(15.0) or [entry])[0].transform.location])  # through it
    try:
        red, moved, stood, longest, through, slowest = False, False, 0.0, 0.0, False, 1e9
        f = front(car)
        for _ in range(int(30 / DT)):
            world.tick()
            if not car.is_alive:
                return f"FAIL: the car vanished at {where(f)}, {ahead_of(f, entry):+.1f} m before the junction"
            f, v = front(car), car.get_velocity().length()
            if ahead_of(f, entry) < 0 and f.distance(entry.transform.location) > 10.0:  # 10 m on, any way it turned
                through = True
                break
            if ahead_of(f, entry) >= 0:  # before the junction
                red |= not car.is_at_traffic_light() and car.get_traffic_light_state() == RED
                moved |= v > 2.0
                stood = stood + DT if moved and v < 0.05 else 0.0
                longest = max(longest, stood)
                if moved:
                    slowest = min(slowest, v * 3.6)
        verdict = "PASS" if longest > 0 and through else "FAIL"
        return (f"{verdict}: stood {longest:.1f} s before the junction, sign turned it red {red}, "
                f"then drove through {through}; slowest {slowest:.1f} km/h before it, ended at {where(f)}")
    finally:
        if car.is_alive:
            car.destroy()


def sign_changes(signs, rows, near=10.0):
    """[(t, old limit, new limit, (sign id, distance m) of the closest sign showing the new limit, or None)]"""
    out = []
    for p, r in zip(rows, rows[1:]):
        if r["limit"] == p["limit"]:
            continue
        here = [(l.id, ((l.transform.location.x - r["x"]) ** 2 + (l.transform.location.y - r["y"]) ** 2) ** 0.5)
                for l in signs if abs(l.value - r["limit"]) < 0.5]
        best = min(here, key=lambda h: h[1], default=None)
        out.append((r["t"], p["limit"], r["limit"], best if best and best[1] <= near else None))
    return out


def speed_test(world, m, minutes):
    signs = m.get_all_landmarks_of_type("274")
    fast = max((l for l in signs if sign_lanes(m, l)), key=lambda l: l.value)  # start on the motorway, 100 m before
    start = sign_lanes(m, fast)[0]
    start = (start.previous(100.0) or [start])[0]
    car = spawn(world, start)
    if car is None:
        print("   skipped: could not spawn a car")
        return
    rows = []
    try:
        for i in range(int(minutes * 60 / DT)):
            world.tick()
            if not car.is_alive:
                print(f"   the car vanished after {i * DT:.0f} s at ({rows[-1]['x']:.0f}, {rows[-1]['y']:.0f})")
                break
            loc, wp = car.get_location(), m.get_waypoint(car.get_location())
            rows.append(dict(t=round(i * DT, 2), x=round(loc.x, 1), y=round(loc.y, 1), z=round(loc.z, 2), road=wp.road_id,
                             lane=wp.lane_id, speed=round(car.get_velocity().length() * 3.6, 1),
                             limit=car.get_speed_limit()))
    finally:
        if car.is_alive:
            car.destroy()
    with open("/work/logs/signals_check.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    sp = start.transform.location
    print(f"   drove {rows[-1]['t'] / 60:.1f} min from ({sp.x:.0f}, {sp.y:.0f}); starting limit {rows[0]['limit']:.0f}")
    changes = sign_changes(signs, rows)
    for t, a, b, sign in changes:
        print(f"   {t:6.1f} s: limit {a:.0f} -> {b:.0f}, " +
              (f"at sign {sign[0]} ({sign[1]:.1f} m away)" if sign else f"FAIL: no {b:.0f} sign within 10 m"))
    print(f"   {sum(1 for *_, sign in changes if sign)} of {len(changes)} limit changes at a sign of that value")
    moving = [r for r in rows if r["speed"] > 5]
    over = sum(DT for r in moving if r["speed"] > r["limit"] + 3)
    ratio = sorted(r["speed"] / r["limit"] for r in moving) or [0]
    print(f"   speed / limit while moving: median {ratio[len(ratio) // 2]:.2f}, max {ratio[-1]:.2f}; "
          f"over the limit by more than 3 km/h for {over:.1f} s")
    print("   -> /work/logs/signals_check.csv")


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(600)  # loading the map and the Traffic Manager's first start take minutes
    world = client.get_world()
    if not world.get_map().name.endswith(MAP):
        world = client.load_world(MAP)
    original = world.get_settings()
    s = world.get_settings()
    s.synchronous_mode, s.fixed_delta_seconds = True, DT
    if MAP == "WolfsburgHighway":
        s.tile_stream_distance = s.actor_active_distance = 3000.0  # all tiles loaded: streaming them crashed the server
    world.apply_settings(s)
    tm = client.get_trafficmanager(TM_PORT)
    tm.set_synchronous_mode(True)
    m = world.get_map()
    print(m.name)
    try:
        lights = inventory(world, m)
        world.freeze_all_traffic_lights(True)
        print("2. traffic lights held red, then green")
        for kmh in (None, 50) if "lights" in PARTS else ():
            for tl in lights:
                print(f"   light {tl.id} at {kmh or 'its limit'} km/h: {light_test(world, m, tm, tl, kmh)}")
        world.freeze_all_traffic_lights(False)
        world.reset_all_traffic_lights()
        print("3. stop signs")
        for l in {l.id: l for l in m.get_all_landmarks_of_type("206")}.values() if "stops" in PARTS else ():
            lanes = stop_lanes(m, l)
            if not lanes:
                print(f"   sign {l.id} (road {l.road_id}): no driving lane leads from it into a junction")
            for wp, entry in lanes:
                print(f"   sign {l.id} (road {l.road_id} lane {wp.lane_id}): {stop_test(world, m, tm, wp, entry)}")
        print("4. speed limits")
        if "speed" in PARTS:
            speed_test(world, m, MINUTES)
    finally:
        world.freeze_all_traffic_lights(False)
        tm.set_synchronous_mode(False)
        s = world.get_settings()
        s.synchronous_mode, s.fixed_delta_seconds = original.synchronous_mode, original.fixed_delta_seconds
        world.apply_settings(s)


if __name__ == "__main__":
    main()
