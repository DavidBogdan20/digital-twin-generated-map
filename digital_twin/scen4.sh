#!/bin/bash
# runs every WolfsburgHighway scenario variant once, without a window, on the fourth build
cd /work/car_simulator
for sv in "pedestrian_crossing PedestrianCrossing_WolfsburgHighway_1" "occluded_crossing OccludedCrossing_Child_WolfsburgHighway_1" \
          "occluded_crossing OccludedCrossing_Crowd_WolfsburgHighway_1" "cut_in CutIn_Parking_WolfsburgHighway_1" \
          "cut_in CutIn_Cyclist_WolfsburgHighway_1"; do
  set -- $sv
  echo "=== $2 $(date -u +%H:%M)"
  timeout 1200 python3 apps/run_scenario.py $1 --variant $2 --no-view --timeout 600 > /work/logs/scen4_$2.log 2>&1
  echo "exit $? $(date -u +%H:%M)"
  pgrep -f "^/work/stock/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping" >/dev/null || { echo "SERVER GONE"; break; }
done
echo SCEN4_DONE
