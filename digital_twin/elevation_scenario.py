# Elevation scenario on Wolfsburg2. Four cars drive north on the A 39, under Stralsunder Ring and a slip road, then over
# the motorway's own 5 m bridge, while a fifth car crosses the Stralsunder Ring bridge above them. Then the camera passes
# slowly along the side of the A 39 bridge. CARLA's own window shows it; a strip at the bottom of the screen plots the
# lead car's height against the road's height from the map data, and every frame goes to /work/logs/elevation.csv.
# Runs in the container on the VM through `vm.sh scenario`, after `vm.sh run` (full screen) and `vm.sh check Wolfsburg2`.
# Usage: elevation_scenario.py [--check]   --check plans the routes, prints them and exits without spawning anything.
import csv
import math
import sys
import time

import carla

A39_ROAD = 5684                   # northbound carriageway of the A 39; it starts south of the interchange
START_S = 30                      # metres into that road where the cars start
ROUTE_M = 1150                    # passes the bridge by about 240 m, so its tiles stay loaded for the last pass beside it
STRALSUND = (4993, -1, 1.0)      # road, lane, s: Stralsunder Ring heading east, 90 m before its bridge over the A 39
A39_BRIDGE = carla.Location(x=1176, y=-1460)  # from the OSM bridge, to tell it apart from the slip road's ramp
STEP = 2.0                        # metres between route points
SPEED, SIDE_SPEED = 25.0, 11.0    # m/s: 90 km/h on the motorway, 40 km/h on Stralsunder Ring
WHEELBASE = 2.8
DT = 0.05                         # 20 simulation steps per second
W, H, STRIP = 1920, 1080, 240      # CARLA's window fills the screen; readouts in a strip at the bottom
CSV_PATH = "/work/logs/elevation.csv"


def wrap(degrees):
    return (degrees + 180) % 360 - 180


def heading_after(wp, metres):
    ahead = wp.next(metres)
    return (ahead[0] if ahead else wp).transform.rotation.yaw


def route(wp, metres):
    """Waypoints along the lane, going straight on at every split. An exit leaves gently at first, so each option is
    judged by where it heads 50 m on."""
    points = [wp]
    while len(points) * STEP < metres:
        options = wp.next(STEP)
        if not options:
            break
        yaw = wp.transform.rotation.yaw
        wp = min(options, key=lambda o: abs(wrap(heading_after(o, 50) - yaw))) if len(options) > 1 else options[0]
        points.append(wp)
    return points


def raised_middle(path, height=4.5):
    """Index of the middle of the longest stretch above `height`, or None."""
    best, start = (0, None), None
    for i, wp in enumerate(path + [None]):
        if wp is not None and wp.transform.location.z > height:
            start = i if start is None else start
        elif start is not None:
            best = max(best, (i - start, (start + i) // 2))
            start = None
    return best[1]


def plan(town):
    lanes = []
    for lane_id in (-1, -2, -3, 1, 2, 3):
        wp = town.get_waypoint_xodr(A39_ROAD, lane_id, START_S)
        if wp is not None and wp.lane_type == carla.LaneType.Driving:
            path = route(wp, ROUTE_M)
            top = raised_middle(path)
            print("lane %d: ends at (%.0f, %.0f), raised middle %s" % (lane_id, path[-1].transform.location.x,
                  path[-1].transform.location.y, None if top is None else path[top].transform.location))
            if top is not None and path[top].transform.location.distance(
                    carla.Location(A39_BRIDGE.x, A39_BRIDGE.y, path[top].transform.location.z)) < 80:
                lanes.append(path)  # stays on the A 39 all the way over its bridge
    assert len(lanes) >= 2, "expected two A 39 lanes over the bridge, found %d" % len(lanes)
    lanes.sort(key=lambda p: p[0].lane_id)  # outermost (right-hand) lane first when ids are negative
    bridge = raised_middle(lanes[0])

    side = route(town.get_waypoint_xodr(*STRALSUND), 350)
    cross, gap = None, 1e9
    for j, p in enumerate(side):
        if p.transform.location.z > 3:  # where it is raised, find the motorway point below
            i = min(range(len(lanes[0])), key=lambda k: lanes[0][k].transform.location.distance(p.transform.location))
            d = math.hypot(lanes[0][i].transform.location.x - p.transform.location.x,
                           lanes[0][i].transform.location.y - p.transform.location.y)
            if d < gap:
                cross, gap = (j, i), d
    assert gap < 15, "Stralsunder Ring doesn't cross the A 39 route (closest %.0f m)" % gap
    return lanes, bridge, side, cross


def describe(lanes, bridge, side, cross):
    a39 = lanes[0]
    print("A 39 lanes %s, %d m each" % ([p[0].lane_id for p in lanes], len(a39) * STEP))
    print("height every 100 m:", [round(a39[i].transform.location.z, 1) for i in range(0, len(a39), 50)])
    b = a39[bridge].transform.location
    print("bridge middle at %.0f m: (%.0f, %.0f) z %.1f" % (bridge * STEP, b.x, b.y, b.z))
    j, i = cross
    print("Stralsunder Ring crosses at %.0f m along the A 39: deck z %.1f over motorway z %.1f, %.0f m from its start" % (
        i * STEP, side[j].transform.location.z, a39[i].transform.location.z, j * STEP))


class Driver:
    """Pure pursuit steering along a route, and a simple speed controller."""

    def __init__(self, car, path, speed):
        self.car, self.path, self.speed, self.i = car, path, speed, 0
        self.max_steer = car.get_physics_control().wheels[0].max_steer_angle

    def step(self, go):
        t = self.car.get_transform()
        here = t.location
        while self.i < len(self.path) - 1 and \
                here.distance(self.path[self.i + 1].transform.location) < here.distance(self.path[self.i].transform.location):
            self.i += 1
        v = self.car.get_velocity()
        speed = math.hypot(v.x, v.y)
        ahead = max(8.0, 0.8 * speed)
        target = self.path[min(self.i + int(ahead / STEP), len(self.path) - 1)].transform.location
        alpha = math.atan2(target.y - here.y, target.x - here.x) - math.radians(t.rotation.yaw)
        steer = math.degrees(math.atan2(2 * WHEELBASE * math.sin(alpha), ahead)) / self.max_steer
        want = self.speed if go and self.i < len(self.path) - 10 else 0.0
        error = want - speed
        throttle = min(max(0.35 + 0.25 * error, 0.0), 1.0) if want > 0 else 0.0
        brake = 1.0 if want == 0 else min(max(-0.3 * error - 0.3, 0.0), 1.0)
        self.car.apply_control(carla.VehicleControl(throttle=throttle, steer=min(max(steer, -1.0), 1.0), brake=brake))
        return speed


def look_at(eye, target):
    d = target - eye
    return carla.Transform(eye, carla.Rotation(pitch=math.degrees(math.atan2(d.z, math.hypot(d.x, d.y))),
                                               yaw=math.degrees(math.atan2(d.y, d.x))))


def main():
    client = carla.Client("localhost", 2000)
    client.set_timeout(60)
    world = client.get_world()
    town = world.get_map()
    assert town.name.endswith("Wolfsburg2"), "load the map first: vm.sh check Wolfsburg2"
    lanes, bridge, side, cross = plan(town)
    describe(lanes, bridge, side, cross)
    if "--check" in sys.argv:
        return

    import os
    import pygame

    a39 = lanes[0]
    mid = a39[bridge].transform
    right = mid.get_right_vector()  # the southbound carriageway is on the left, so look from the right
    original = world.get_settings()
    settings = world.get_settings()
    settings.synchronous_mode, settings.fixed_delta_seconds = True, DT
    # Over 300 m from the lead car, or on a tile CARLA counts as unloaded, actors sleep. 0.9.15 crashed putting a
    # camera sensor to sleep (ALargeMapManager -> ASensor::EndPlay) although it was 40 m from the lead car, so there
    # is no sensor: CARLA's own window shows the views through the spectator, which never sleeps.
    # CARLA also crashed creating mesh buffers (FVulkanResourceMultiBuffer) whenever many meshes arrived at once: at
    # 90 km/h into a new tile with 600 m of tiles loaded, on a 360 m camera jump, and when raising this distance.
    # Fewer tiles at a time is the guess being tested, hence 350 m.
    settings.tile_stream_distance, settings.actor_active_distance = 350, 300
    world.apply_settings(settings)
    for actor in world.get_actors().filter("vehicle.*"):
        actor.destroy()

    os.environ["SDL_VIDEO_WINDOW_POS"] = "0,%d" % (H - STRIP)  # over the bottom of CARLA's full-screen window
    pygame.init()
    screen = pygame.display.set_mode((W, STRIP))
    font = pygame.font.Font(None, 38)
    blueprints = world.get_blueprint_library()
    start = a39[0].transform
    first_eye = start.location + carla.Location(z=30) - start.get_forward_vector() * 30
    first_view = look_at(first_eye, start.location)
    actors, history, shown, clock, fallen = [], [], [None], [time.time()], set()
    out = open(CSV_PATH, "w", newline="")
    log = csv.writer(out)
    log.writerow(["time", "phase", "route_m", "x", "y", "car_z", "road_z", "pitch", "speed_kmh", "lowest_gap"])

    def frame(view, phase, lead=None, speed=0.0, gap=0.0):
        world.get_spectator().set_transform(view)  # with no hero yet, the spectator also decides which tiles load
        if phase != shown[0]:
            print("%.1f s  %s" % (world.get_snapshot().timestamp.elapsed_seconds, phase))
            shown[0] = phase
        world.tick()
        clock[0] += DT  # real time: unpaced, CARLA ran this about 7 times faster than real, too fast to watch
        time.sleep(max(0.0, clock[0] - time.time()))
        clock[0] = max(clock[0], time.time() - DT)
        screen.fill((0, 0, 0))
        lines = [phase]
        if lead is not None:
            t = lead.car.get_transform()
            road_z = lead.path[lead.i].transform.location.z
            history.append((t.location.z, road_z))
            lines += ["Lead car  %3.0f km/h   nose %+.1f deg" % (speed * 3.6, t.rotation.pitch),
                      "Car height %5.2f m   road (map) %5.2f m   difference %+.2f m" % (t.location.z, road_z, t.location.z - road_z)]
            log.writerow([round(world.get_snapshot().timestamp.elapsed_seconds, 2), phase, lead.i * STEP,
                          round(t.location.x, 1), round(t.location.y, 1), round(t.location.z, 3), round(road_z, 3),
                          round(t.rotation.pitch, 2), round(speed * 3.6, 1), round(gap, 2)])
        for n, text in enumerate(lines):
            screen.blit(font.render(text, True, (255, 255, 255)), (20, 25 + 50 * n))
        if len(history) > 1:  # last 15 s: grey = road height from the map, orange = car height, -2 m .. 7 m
            box = pygame.Rect(W - 940, 10, 920, STRIP - 20)
            pygame.draw.rect(screen, (35, 35, 35), box)
            to_y = lambda z: box.bottom - 10 - (z + 2) / 9 * (box.height - 20)
            recent = history[-300:]
            for colour, k in (((170, 170, 170), 1), ((255, 140, 0), 0)):
                pygame.draw.lines(screen, colour, False, [(box.x + 3 * n, to_y(h[k])) for n, h in enumerate(recent)], 3)
            screen.blit(font.render("height, last 15 s: grey road (map), orange car", True, (255, 255, 255)),
                        (box.x + 10, box.y + 6))
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE):
                raise KeyboardInterrupt

    try:
        # Fly there at 30 m/s rather than jump: every jump of a few hundred metres crashed CARLA while it created
        # buffers for the meshes arriving at once (FStagingManager::AcquireBuffer, FVulkanResourceMultiBuffer).
        here = world.get_spectator().get_transform()
        eye0, aim0 = here.location, here.location + here.get_forward_vector() * 50
        steps = max(1, int(eye0.distance(first_eye) / (30 * DT)))
        for s in range(1, steps + 1):
            frame(look_at(eye0 + (first_eye - eye0) * (s / steps), aim0 + (start.location - aim0) * (s / steps)),
                  "Flying to the A 39")
        for _ in range(60):  # let the tiles around the start load before anything stands on them
            frame(first_view, "Loading the A 39 south of the interchange")

        def spawn(name, path, role="autopilot", view=first_view):
            """A car on the first free spot in the next 30 m: buildings and poles stand on some lanes of this map."""
            bp = blueprints.find(name)
            bp.set_attribute("role_name", role)
            for _ in range(40):
                for wp in path[:15:3]:
                    car = world.try_spawn_actor(bp, carla.Transform(wp.transform.location + carla.Location(z=0.5),
                                                                    wp.transform.rotation))
                    if car is not None:
                        actors.append(car)
                        return car
                frame(view, "Placing cars")
            raise RuntimeError("couldn't place %s near %s" % (name, path[0].transform.location))

        # The quicker car of each lane in front, so gaps open while they speed up: the first run put the truck
        # 60 m ahead of the Tesla and the Tesla ran into it. Each driver gets its whole lane and finds its place on it.
        n = int(20 / STEP)  # 20 m
        lead = Driver(spawn("vehicle.tesla.model3", lanes[0][3 * n:], "hero"), lanes[0], SPEED)
        motorway = [lead,
                    Driver(spawn("vehicle.carlamotors.carlacola", lanes[0]), lanes[0], SPEED),
                    Driver(spawn("vehicle.mercedes.coupe_2020", lanes[1][4 * n:]), lanes[1], SPEED),
                    Driver(spawn("vehicle.audi.a2", lanes[1][n:]), lanes[1], SPEED)]
        above, placed = None, False  # the car above is placed when the lead car comes within 300 m of it
        j, i = cross
        # start the car above so it reaches the crossing about when the lead car passes under it
        trigger = i - (j * STEP) * SPEED / SIDE_SPEED / STEP - 15

        def chase():  # eye and the point it looks at
            t = lead.car.get_transform()
            eye = t.location - t.get_forward_vector() * 14 + t.get_right_vector() * 4 + carla.Location(z=5)
            return eye, t.location + t.get_forward_vector() * 12

        def beside(k):  # 30 m right of route point k and 8 m up, looking across the motorway
            a, b = a39[int(k)].transform.location, a39[min(int(k) + 1, len(a39) - 1)].transform.location
            road = a + (b - a) * (k - int(k))
            r = a39[int(k)].transform.get_right_vector()
            return carla.Location(road.x + r.x * 30, road.y + r.y * 30, 8), carla.Location(road.x, road.y, 2)

        def blocked(d):  # another motorway car less than 25 m ahead in the same lane
            t = d.car.get_transform()
            f = t.get_forward_vector()
            for other in motorway:
                v = other.car.get_location() - t.location
                if other is not d and 0 < v.x * f.x + v.y * f.y < 25 and abs(v.x * f.y - v.y * f.x) < 2.5:
                    return True
            return False

        gone_under, ticks = False, 0
        while lead.i < len(lead.path) - 12 and ticks < 2400:
            ticks += 1
            go = {d: ticks > 20 and not blocked(d) for d in motorway}
            speed = lead.step(go[lead])
            for d in motorway[1:]:
                d.step(go[d])
            if not placed and lead.i >= trigger:
                above, placed = Driver(spawn("vehicle.mini.cooper_s_2021", side, view=look_at(*chase())), side, SIDE_SPEED), True
            if above is not None:
                above.step(True)
                if lead.i > i + 150:  # crossed long ago, and the ground under it unloads as the lead car moves on
                    actors.remove(above.car)
                    above.car.destroy()
                    above = None
            cars = motorway + ([above] if above else [])
            gaps = [d.car.get_location().z - d.path[d.i].transform.location.z for d in cars]
            low = cars[gaps.index(min(gaps))]
            if min(gaps) < -3 and low.car.id not in fallen:
                fallen.add(low.car.id)
                print("a car is %.1f m below the road in the map data at %s" % (-min(gaps), low.car.get_location()))
            if abs(lead.i - bridge) < 50 and lead.i < bridge + 30:
                eye = mid.location + right * 45 + carla.Location(z=8 - mid.location.z)  # lower, the road hides below the grass
                view, phase = look_at(eye, lead.car.get_location()), "Over the A 39 bridge (5 m in the map data)"
            else:
                gone_under = gone_under or lead.i > i
                view = look_at(*chase())
                phase = "Under Stralsunder Ring and the slip road" if not gone_under else "A 39 northbound, 90 km/h"
            frame(view, phase, lead, speed, min(gaps))

        for _ in range(40):  # stop everyone
            for d in motorway:
                d.step(False)
            frame(look_at(*chase()), "Stopping", lead)
        # Then back along the motorway past the bridge, without jumps: jumping straight to the bridge crashed CARLA
        # (FStagingManager::AcquireBuffer), as long jumps did last week.
        (eye0, aim0), k = chase(), float(lead.i)
        for s in range(1, 61):  # 3 s glide down to the side of the road
            eye1, aim1 = beside(k)
            frame(look_at(eye0 + (eye1 - eye0) * (s / 60), aim0 + (aim1 - aim0) * (s / 60)), "Back to the bridge", lead)
        while k > bridge - 50:  # 12 m/s, to 100 m before the middle of the bridge, where the ramp starts
            k -= 12 * DT / STEP
            frame(look_at(*beside(k)), "Beside the A 39 bridge: ramps, deck and what is underneath", lead)
    except KeyboardInterrupt:
        print("stopped with Esc")
    finally:
        out.close()
        # back to normal mode, but no wider: restoring 600 m of tiles crashed CARLA two seconds after one run
        original.tile_stream_distance, original.actor_active_distance = 350, 300
        world.apply_settings(original)
        for actor in actors:
            actor.destroy()
        pygame.quit()
    print("wrote", CSV_PATH)


if __name__ == "__main__":
    main()
