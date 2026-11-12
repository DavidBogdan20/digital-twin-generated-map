# Chart of an elevation_scenario.py run: the lead car's height against the road's height in the map data, plus its
# speed and nose angle, along the route. Usage: elevation_chart.py elevation.csv elevation.png
import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

rows = [r for r in csv.DictReader(open(sys.argv[1])) if r["phase"].startswith(("Under", "A 39", "Over"))]
metres = [float(r["route_m"]) for r in rows]
fig, (top, bottom) = plt.subplots(2, 1, figsize=(13, 8), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
top.plot(metres, [float(r["road_z"]) for r in rows], color="grey", lw=3, label="road height in the map data (OpenDRIVE)")
top.plot(metres, [float(r["car_z"]) for r in rows], color="darkorange", lw=1.5, label="lead car height (simulated)")
top.axhline(0, color="black", lw=0.5)
top.set_ylabel("height (m)")
top.set_title("Wolfsburg2, A 39 northbound: lead car over the 5 m bridge")
top.legend(loc="upper left")
bottom.plot(metres, [float(r["speed_kmh"]) for r in rows], color="tab:blue", label="speed (km/h)")
bottom.plot(metres, [float(r["pitch"]) * 5 for r in rows], color="tab:green", label="nose angle (degrees x 5)")
bottom.set_xlabel("distance along the route (m)")
bottom.legend(loc="lower left")
for ax in (top, bottom):
    ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(sys.argv[2], dpi=110)
print("wrote", sys.argv[2])
