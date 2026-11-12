import json, math, collections, sys, carla
sys.path.insert(0, "/work/dt")
import check_level as C
m = carla.Map("w", open("/work/carla/Unreal/CarlaUE4/Content/CustomMaps/WolfsburgHighway/OpenDrive/WolfsburgHighway.xodr").read())
rows = [json.loads(l) for l in open("/work/logs/WolfsburgHighway_meshes.jsonl")]
spots = [(r["x"], r["y"]) for r in rows if r["mesh"] == "spot"]
print("spot actors left:", len(spots), collections.Counter(r["actor"][:9] for r in rows if r["mesh"] == "spot"))
trees = [r for r in rows if C.kind(r["mesh"]) == "tree"]
by_actor = collections.Counter(r["actor"].rstrip("0123456789") for r in trees)
print("tree holders:", by_actor.most_common(6))
bins = collections.Counter()
near_spot = collections.Counter()
for r in trees:
    g, _ = C.lane_gap(m, r["x"], r["y"], carla.LaneType.Driving)
    b = "in lane" if g < 0 else "0-1.5 m" if g < 1.5 else "1.5-4 m" if g < 4 else "ok"
    bins[b] += 1
    if b != "ok":
        d = min((math.hypot(r["x"] - x, r["y"] - y) for x, y in spots), default=1e9)
        near_spot[(b, "at a spot" if d < 0.5 else "not at a spot")] += 1
print("trees by gap to the nearest driving lane:", dict(bins))
print(dict(near_spot))
print("sizes of in-lane ones:", collections.Counter(r["mesh"].split(".")[-1].split("_v")[0] for r in trees if C.lane_gap(m, r["x"], r["y"], carla.LaneType.Driving)[0] < 0).most_common(8))
