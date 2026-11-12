import math, carla
c = carla.Client("localhost", 2000); c.set_timeout(30)
w = c.get_world(); m = w.get_map()
P = carla.Location(1470.575, -1227.975, 0.6416)
wp = m.get_waypoint(P)
print("lane", wp.road_id, wp.lane_id, "width %.2f" % wp.lane_width, "xodr z %.2f" % wp.transform.location.z, "yaw %.1f" % wp.transform.rotation.yaw,
      "junction", wp.is_junction, "offset from lane centre %.2f m" % P.distance(carla.Location(wp.transform.location.x, wp.transform.location.y, P.z)))
bp = w.get_blueprint_library().find("vehicle.carlamotors.european_hgv")
for lift in (0.0, 0.5, 1.0, 2.0, 3.0):
    a = w.try_spawn_actor(bp, carla.Transform(P + carla.Location(z=lift), wp.transform.rotation))
    print("lift %.1f -> %s" % (lift, "spawned" if a else "blocked"))
    if a:
        a.destroy(); break
for o in w.get_environment_objects(carla.CityObjectLabel.Any):
    l = o.bounding_box.location
    if math.hypot(l.x - P.x, l.y - P.y) < 15:
        print("  near:", o.name[:45], str(o.type), "(%.1f, %.1f, %.1f)" % (l.x, l.y, l.z), "extent %.1f %.1f %.1f" % (o.bounding_box.extent.x, o.bounding_box.extent.y, o.bounding_box.extent.z))
for a in w.get_actors():
    if a.get_location().distance(P) < 15:
        print("  actor:", a.type_id, a.get_location())
