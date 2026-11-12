"""Samples the road surface (collision) by casting rays down on a grid and lists the dents: points lying more than 3 cm
below the median of their 3 m neighbourhood. usage: surface_scan.py x0 x1 y0 y1 step"""
import sys, math
import numpy as np
import carla
x0, x1, y0, y1, st = map(float, sys.argv[1:6])
w = carla.Client("localhost", 2000).get_world()
xs, ys = np.arange(x0, x1 + 1e-6, st), np.arange(y0, y1 + 1e-6, st)
Z = np.full((len(xs), len(ys)), np.nan)
for i, x in enumerate(xs):
    for j, y in enumerate(ys):
        hits = [p for p in w.cast_ray(carla.Location(x, y, 30), carla.Location(x, y, -30)) if p.label == carla.CityObjectLabel.Roads]
        if hits:
            Z[i, j] = hits[0].location.z
k = int(round(1.5 / st))
dent = []
for i in range(len(xs)):
    for j in range(len(ys)):
        if np.isnan(Z[i, j]):
            continue
        nb = Z[max(0, i - k):i + k + 1, max(0, j - k):j + k + 1]
        d = Z[i, j] - np.nanmedian(nb)
        if d < -0.03:
            dent.append((xs[i], ys[j], d))
print(f"{np.count_nonzero(~np.isnan(Z))} road points of {Z.size}, {len(dent)} lie >3 cm under their surroundings")
def bump(px, py):
    bx, by = math.ceil(px / 17) * 17, math.floor(py / 12) * 12
    d = math.hypot(bx - px, by - py)
    return 0.1 * math.sin(d) if d <= 2 else 0.0
for flip in (1, -1):
    hit = sum(1 for x, y, d in dent if bump(x, flip * y) > 0.02)
    pred = sum(1 for i in range(len(xs)) for j in range(len(ys)) if not np.isnan(Z[i, j]) and bump(xs[i], flip * ys[j]) > 0.03)
    print(f"y*{flip:+d}: {hit} of {len(dent)} dents lie where the tool's formula puts a dent; formula predicts >3 cm at {pred} road points")
for x, y, d in sorted(dent, key=lambda t: t[2])[:12]:
    print(f"  dent {x:7.2f} {y:8.2f}  {d*100:5.1f} cm   formula {bump(x, y)*100:4.1f} cm (y flipped {bump(x, -y)*100:4.1f})")
