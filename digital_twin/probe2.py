import math, carla, collections
c = carla.Client("localhost", 2000); c.set_timeout(300)
w = c.get_world()
objs = w.get_environment_objects(carla.CityObjectLabel.Any)
P = carla.Location(1200, -1436, 4)
print(collections.Counter(str(o.type) for o in objs))
xs = [o.bounding_box.location for o in objs]
print("bbox x range", min(l.x for l in xs), max(l.x for l in xs), "y", min(l.y for l in xs), max(l.y for l in xs))
ts = [o.transform.location for o in objs]
print("transform x range", min(l.x for l in ts), max(l.x for l in ts), "y", min(l.y for l in ts), max(l.y for l in ts))
d = sorted(objs, key=lambda o: o.bounding_box.location.distance(P))[:25]
for o in d:
    b = o.bounding_box
    print(round(b.location.distance(P),1), o.name[:60], o.type, [round(v,1) for v in (b.location.x, b.location.y, b.location.z, b.extent.x, b.extent.y, b.extent.z)], b.rotation.yaw)
