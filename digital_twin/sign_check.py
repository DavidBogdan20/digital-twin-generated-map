# Runtime signs, poles and traffic lights (spawned by CARLA from the OpenDRIVE signals) that stand on a lane.
import math, collections, carla
c = carla.Client("localhost", 2000); c.set_timeout(60)
w = c.get_world(); m = w.get_map()
objs = w.get_environment_objects(carla.CityObjectLabel.Any)
print(len(objs), "runtime objects:", dict(collections.Counter(str(o.type) for o in objs)))
bad = []
for o in objs:
    l = o.bounding_box.location
    for kind in (carla.LaneType.Driving, carla.LaneType.Sidewalk):
        wp = m.get_waypoint(carla.Location(l.x, l.y, 0), project_to_road=True, lane_type=kind)
        if wp is None: continue
        cl = wp.transform.location
        gap = math.hypot(cl.x - l.x, cl.y - l.y) - wp.lane_width / 2 - max(o.bounding_box.extent.x, o.bounding_box.extent.y)
        if kind == carla.LaneType.Driving and gap < 0.3:
            bad.append((round(gap, 2), o.name[:40], str(o.type), round(l.x, 1), round(l.y, 1), wp.road_id, wp.lane_id))
print(len(bad), "runtime objects on or within 0.3 m of a driving lane")
for b in sorted(bad)[:25]: print(b)
