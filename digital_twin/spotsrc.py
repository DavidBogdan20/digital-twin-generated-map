import sys, math, carla
sys.path.insert(0, "/work/dt")
import prepare_dt as P
xodr = open("/work/carla/Unreal/CarlaUE4/Content/CustomMaps/Wolfsburg2/OpenDrive/Wolfsburg2.xodr").read()
m = carla.Map("w", xodr)
T = (1203.5, -1435.3)
print("gap at tree:", round(P.lane_gap(m, *T), 2))
w = m.get_waypoint(carla.Location(T[0], T[1], 0), project_to_road=True, lane_type=carla.LaneType.Any)
print("nearest lane:", w.road_id, w.lane_id, w.lane_type, round(w.lane_width, 2), "centre", round(w.transform.location.x, 1), round(w.transform.location.y, 1), round(w.transform.location.z, 1))
best = sorted((math.hypot(x - T[0], y - T[1]), x, y) for x, y in P.tool_spots(m, xodr))[:3]
print("nearest replayed spots:", [(round(d, 2), round(x, 1), round(y, 1)) for d, x, y in best])
