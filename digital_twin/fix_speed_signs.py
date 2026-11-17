# Makes stock CARLA's 100, 110 and 120 km/h signs set their own limit instead of 90. Usage: fix_speed_signs.py <CARLA>
# Found in 0.9.15; on a release that has it right it prints "already sets", and on an unknown layout it stops.
# A speed sign spawned from the road file has two trigger boxes: CARLA's, with the road file's value, and the sign
# blueprint's own, with a value stored in the blueprint. BP_SpeedLimit100/110/120 were copied from BP_SpeedLimit90 and
# still store 90 (the only 90.0 float in their .uexp; every other BP_SpeedLimitNN stores NN), so a car passing a 100
# sign got 100, then 90 (signals_check.py, 2026-11-12). This changes that one float in place (same file size) and keeps
# a .orig copy. Runs again harmlessly. <CARLA> is the packaged simulator's folder (the one with CarlaUE4.sh).
import os
import shutil
import struct
import sys

SIGNS = os.path.join(sys.argv[1], "CarlaUE4", "Content", "Carla", "Static", "TrafficSign")
WRONG = struct.pack("<f", 90.0)

for value in (100, 110, 120):
    path = os.path.join(SIGNS, f"BP_SpeedLimit{value}.uexp")
    data = open(path, "rb").read()
    right = struct.pack("<f", float(value))
    if data.count(WRONG) == 0 and data.count(right) == 1:
        print(f"BP_SpeedLimit{value}: already sets {value}")
        continue
    assert data.count(WRONG) == 1 and data.count(right) == 0, f"{path}: not the layout this fix knows"
    if not os.path.exists(path + ".orig"):
        shutil.copy2(path, path + ".orig")
    open(path, "wb").write(data.replace(WRONG, right))
    print(f"BP_SpeedLimit{value}: 90 -> {value}")
