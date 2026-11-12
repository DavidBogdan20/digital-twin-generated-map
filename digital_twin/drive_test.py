# A hero car on autopilot with traffic, no teleporting. Usage: drive_test.py radius sim_seconds cars start_x start_y slowdown_percent
import random, sys, time, carla
RADIUS, SECONDS, CARS = float(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
X, Y = float(sys.argv[4]), float(sys.argv[5])
DT = 0.05
client = carla.Client("localhost", 2000); client.set_timeout(120)
world = client.load_world("Wolfsburg2"); town = world.get_map()
tm = client.get_trafficmanager(8100)
tm.global_percentage_speed_difference(float(sys.argv[6]))
s = world.get_settings(); s.tile_stream_distance = RADIUS; s.actor_active_distance = RADIUS * 0.75
s.synchronous_mode = True; s.fixed_delta_seconds = DT; world.apply_settings(s)
tm.set_synchronous_mode(True)
random.seed(1)
lib = world.get_blueprint_library()
spawns = town.get_spawn_points()
spawns.sort(key=lambda p: (p.location.x - X) ** 2 + (p.location.y - Y) ** 2)
hero_bp = lib.find("vehicle.tesla.model3"); hero_bp.set_attribute("role_name", "hero")
hero = None
for p in spawns[:40]:
    hero = world.try_spawn_actor(hero_bp, p)
    if hero:
        break
print("hero at", round(p.location.x), round(p.location.y), flush=True)
for _ in range(60):
    world.tick()
npc_bps = [b for b in lib.filter("vehicle.*") if int(b.get_attribute("number_of_wheels")) == 4]
npcs = {}
for p in spawns[1:]:
    if len(npcs) >= CARS:
        break
    a = world.try_spawn_actor(random.choice(npc_bps), p)
    if a:
        npcs[a.id] = a
for _ in range(40):
    world.tick()
hero.set_autopilot(True, 8100)
for a in npcs.values():
    a.set_autopilot(True, 8100)
print("npcs", len(npcs), flush=True)
last = {}
mark = {}
trail = []
trip = 0.0
prev = hero.get_location()
wall = time.time()
for tick in range(int(SECONDS / DT)):
    world.tick()
    if not hero.is_alive:
        l = world.get_actors().find(hero.id)
        print("HERO GONE sim", round(tick * DT, 2), "last", [round(v, 1) for v in (seen.location.x, seen.location.y, seen.location.z)], "yaw", round(seen.rotation.yaw), "pitch", round(seen.rotation.pitch), "roll", round(seen.rotation.roll),
              "kmh", round(seen_kmh), "road", town.get_waypoint(seen.location).road_id, "find", l, "cars left", len(world.get_actors().filter("vehicle.*")), flush=True)
        for t, loc, kmh in trail[-6:]:
            print("   ", t, loc, kmh, flush=True)
        break
    h = hero.get_transform()
    seen = h
    v0 = hero.get_velocity(); seen_kmh = 3.6 * (v0.x ** 2 + v0.y ** 2 + v0.z ** 2) ** 0.5
    if tick % 10 == 0:
        trail.append((round(tick * DT, 1), (round(h.location.x), round(h.location.y), round(h.location.z, 1)), round(seen_kmh)))
    trip += h.location.distance(prev); prev = h.location
    world.get_spectator().set_transform(carla.Transform(h.location + carla.Location(z=25) - h.get_forward_vector() * 25, carla.Rotation(pitch=-35, yaw=h.rotation.yaw)))
    for i, a in list(npcs.items()):
        if a.is_alive:
            last[i] = a.get_location()
        else:
            l = last.get(i)
            print("  vanished", i, "at", round(l.x), round(l.y), round(l.z, 1), "sim", round(tick * DT), flush=True)
            del npcs[i]
    if tick % int(10 / DT) == 0:
        moved = sum(1 for i, a in npcs.items() if i in mark and a.get_location().distance(mark[i]) > 15)
        mark = {i: a.get_location() for i, a in npcs.items()}
        c = hero.get_control(); v = hero.get_velocity()
        print("sim", round(tick * DT), "hero", round(h.location.x), round(h.location.y), round(h.location.z, 1), "trip", round(trip), "kmh", round(3.6 * (v.x ** 2 + v.y ** 2) ** 0.5), "steer", round(c.steer, 2),
              "npcs", len(npcs), "moved_15m", moved, "fallen", sum(1 for a in npcs.values() if a.get_location().z < -10), "wall_fps", round(tick / max(time.time() - wall, 1)), flush=True)
print("drive done", len(npcs), "npcs left, hero trip", round(trip), "m")
s.synchronous_mode = False; s.fixed_delta_seconds = None; world.apply_settings(s)
