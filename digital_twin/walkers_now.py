# Where every walker stands now, and what is under it (another client watching a running scenario).
import math, carla
c = carla.Client("localhost", 2000); c.set_timeout(30)
w = c.get_world(); m = w.get_map()
ego = [a for a in w.get_actors().filter("vehicle.*") if a.attributes.get("role_name") == "hero"]
e = ego[0].get_location() if ego else None
print("ego at", e, "speed %.1f" % (math.hypot(ego[0].get_velocity().x, ego[0].get_velocity().y) if ego else -1))
for a in w.get_actors().filter("walker.pedestrian.*"):
    l = a.get_location(); v = a.get_velocity()
    on = [t for t, k in (("driving", carla.LaneType.Driving), ("sidewalk", carla.LaneType.Sidewalk))
          if m.get_waypoint(l, project_to_road=False, lane_type=k)]
    if e and l.distance(e) < 40:
        print("walker %d at (%.1f, %.1f, %.2f) on %s, %.1f m from ego, speed %.1f" % (a.id, l.x, l.y, l.z, on or "nothing", l.distance(e), math.hypot(v.x, v.y)))
