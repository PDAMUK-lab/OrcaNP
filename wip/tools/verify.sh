#!/bin/bash
# CLI verification of the S4 / polar integration. Usage: verify.sh [case...]
T=/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad/clitest
C=/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad/check_gcode.py
B=/home/user/OrcaNP/build/src/Release/orca-slicer
run() { # name machine process model
    rm -rf "$T/$1"; mkdir -p "$T/$1"
    local t0=$(date +%s)
    timeout 1800 $B --datadir $T/datadir --load-settings "$T/$2;$T/$3" --slice 0 --outputdir $T/$1 $T/$4 > $T/$1/log 2>&1
    echo "== $1: exit $? in $(( $(date +%s) - t0 )) s"
    grep -iE "S4:|Polar:|error|warning: S4|failed" $T/$1/log | grep -v "^$" | head -8
}
cases=${@:-"cube pi s4 polar s4round theta cone"}
for c in $cases; do case $c in
  cube)  run new_cube machine_cartesian.json process.json cube.obj; python3 $C diff $T/base_cube/plate_1.gcode $T/new_cube/plate_1.gcode ;;
  pi)    run new_pi machine_cartesian.json process.json pi.stl; python3 $C diff $T/base_pi/plate_1.gcode $T/new_pi/plate_1.gcode ;;
  s4)    run s4_pi machine_cartesian.json process_s4.json pi.stl; python3 $C s4 $T/s4_pi/plate_1.gcode ;;
  polar) run polar_pi machine_polar.json process_s4.json pi.stl; python3 $C polar $T/s4_pi/plate_1.gcode $T/polar_pi/plate_1.gcode ;;
  s4round) run s4_round machine_cart_round.json process_s4.json pi.stl; python3 $C s4 $T/s4_round/plate_1.gcode ;;
  theta) run theta_pi machine_thetafirm.json process_s4.json pi.stl; python3 $C polar $T/s4_round/plate_1.gcode $T/theta_pi/plate_1.gcode 0 0 43 -1 ;;
  cone) run cone_pi machine_cart_round.json process_cone.json pi.stl; python3 $C s4 $T/cone_pi/plate_1.gcode ;;
esac; echo "   check exit $?"; done
