#!/bin/bash
# fourth WolfsburgHighway build, after Generate + prune: the rest of GUIDE Stage 9, stopping at the first failure
set -e
cd /work/dt
echo "=== facades $(date -u +%H:%M)"; ./vm.sh facades WolfsburgHighway
echo "=== rails $(date -u +%H:%M)"; ./vm.sh rails WolfsburgHighway
echo "=== audit 3 $(date -u +%H:%M)"; ./vm.sh audit WolfsburgHighway | tail -4
echo "=== meshscan $(date -u +%H:%M)"; ./vm.sh meshscan WolfsburgHighway 1 | tail -3
echo "=== tileoffset $(date -u +%H:%M)"; ./vm.sh tileoffset WolfsburgHighway
echo "=== quit editor $(date -u +%H:%M)"; echo "unreal.SystemLibrary.quit_editor()" > /work/dt/cmd.py
timeout 600 bash -c "while pgrep -x UE4Editor >/dev/null; do sleep 5; done"
echo "=== package $(date -u +%H:%M)"; ./vm.sh package WolfsburgHighway > /work/logs/package_highway4.log 2>&1
tail -3 /work/logs/package_highway4.log
echo "=== uturns $(date -u +%H:%M)"; ./vm.sh uturns WolfsburgHighway | tail -2
echo "BUILD4_DONE $(date -u +%H:%M)"
