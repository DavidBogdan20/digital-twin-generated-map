# Loads a generated map in the running simulator and checks that a car can sit on it. Usage: map_test.py Tiny
import sys
import carla

name = sys.argv[1]
client = carla.Client("localhost", 2000)
client.set_timeout(180)
print("simulator", client.get_server_version(), "maps", [m for m in client.get_available_maps() if name in m])

world = client.load_world(name)
settings = world.get_settings()
# Wolfsburg2 (4.5 x 2.4 km) crashed at its load default (2000 m), so it streams 600 m around the car. Streaming itself
# crashed it too (Vulkan buffer creation as tiles load), so the 2 x 2 km map keeps all its tiles loaded instead.
small = name == "WolfsburgHighway"
settings.tile_stream_distance = 3000 if small else 600
settings.actor_active_distance = 3000 if small else 450
world.apply_settings(settings)
town = world.get_map()
spawns = town.get_spawn_points()
print("loaded", town.name, "spawn points", len(spawns), "waypoints", len(town.generate_waypoints(2.0)))

blueprint = world.get_blueprint_library().find("vehicle.tesla.model3")
start = spawns[len(spawns) // 2]
car = world.spawn_actor(blueprint, start)
for _ in range(120):
    world.wait_for_tick()
print("car spawned at z", round(start.location.z, 2), "settled at z", round(car.get_location().z, 2))

view = carla.Transform(start.location + carla.Location(z=70), carla.Rotation(pitch=-55, yaw=start.rotation.yaw))
world.get_spectator().set_transform(view)
