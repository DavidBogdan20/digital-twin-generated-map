"""Measures the road surface under the wheel tracks of the last jiggle_test run (/work/logs/jiggle_sync.csv) and lists
steps and kinks. usage: track_profile.py [step m]"""
import csv, math, sys
import numpy as np
import carla
STEP = float(sys.argv[1]) if len(sys.argv) > 1 else 0.1
w = carla.Client("localhost", 2000).get_world()
rs = [r for r in csv.DictReader(open("/work/logs/jiggle_sync.csv")) if r["phase"] == "driving"]
P = np.array([[float(r[k]) for k in ("x", "y", "z", "yaw", "roll")] for r in rs])
# densify the path
seg = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
s = np.concatenate([[0], np.cumsum(seg)])
S = np.arange(0, s[-1], STEP)
X, Y, Z = (np.interp(S, s, P[:, i]) for i in range(3))
YAW = np.interp(S, s, np.unwrap(np.radians(P[:, 3])))
for name, lon, lat in (("front left", 1.45, -0.8), ("front right", 1.45, 0.8)):
    h = []
    for x, y, z, a in zip(X, Y, Z, YAW):
        px, py = x + lon * math.cos(a) - lat * math.sin(a), y + lon * math.sin(a) + lat * math.cos(a)
        hits = [p.location.z for p in w.cast_ray(carla.Location(px, py, z + 3), carla.Location(px, py, z - 3))]
        h.append(min(hits, key=lambda q: abs(q - z)) if hits else np.nan)
    h = np.array(h)
    step = np.abs(np.diff(h))                                  # height change between neighbouring samples
    k = int(round(0.5 / STEP))
    kink = np.abs(h[2 * k:] - 2 * h[k:-k] + h[:-2 * k])       # slope change over 0.5 m either side
    print(f"{name}: {len(h)} samples over {S[-1]:.0f} m, missing {np.isnan(h).sum()}; "
          f"steps >1 cm per {STEP} m: {(step > 0.01).sum()}, >3 cm: {(step > 0.03).sum()}; "
          f"kinks >2 cm: {(kink > 0.02).sum()}, >5 cm: {(kink > 0.05).sum()}")
    order = np.argsort(-np.nan_to_num(kink))[:8]
    for i in sorted(order):
        j = i + k
        print(f"   s {S[j]:6.1f} m at {X[j]:6.1f},{Y[j]:7.1f}  kink {kink[i]*100:4.1f} cm  step here {np.nanmax(step[max(0,j-2):j+2])*100:4.1f} cm")
