#!/bin/bash
# run_slices.sh: full CLI slices of each model per variant, one at a time (alone), timed; then outer wall
# distance to the model for each (wall_dev.py, placed by the planar slice).
S=/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad; T=$S/clitest; B=/home/user/OrcaNP/build/src/Release/orca-slicer
OUT=$S/slices; mkdir -p $OUT; R=$OUT/results.txt
declare -A ENV=( [planar]="" [today]="" \
  [graded2W]="S4_PROTO_FACET_SIZE=2 S4_PROTO_FACET_DISTANCE=0.1 S4_PROTO_WEIGHTED=1 S4_PROTO_WARM=1" \
  [graded1]="S4_PROTO_FACET_SIZE=1 S4_PROTO_FACET_DISTANCE=0.05 S4_PROTO_WARM=1" )
echo "== slices $(date -u +%H:%M:%S) UTC" >> $R
for M in ${MODELS:-3DBenchy idler_x3 benchy170}; do
  for V in planar today graded2W graded1; do
    P=process_q_auto.json; [ $V = planar ] && P=process_q_planar.json
    D=$OUT/${M}_$V; rm -rf $D; mkdir -p $D
    busy=$(pgrep -xc 'cc1plus|ninja|orca-slicer|ld|libslic3r_tests|fff_print_tests' || true)
    t0=$(date +%s.%N)
    env ${ENV[$V]} S4_PROTO_STATS=1 timeout 3600 $B --datadir $T/datadir --load-settings "$T/machine_cartesian.json;$T/$P" --slice 0 --outputdir $D $T/$M.stl > $D/log 2>&1
    code=$?; t1=$(date +%s.%N)
    printf "%-10s %-9s exit %d  slice %6.1f s  (other jobs at start: %s)  %s\n" $M $V $code $(echo "$t1 - $t0" | bc) $busy "$(grep -h 'S4 proto' $D/log | head -1)" >> $R
  done
  python3 $S/render/wall_dev.py $T/$M.stl $OUT/${M}_planar/plate_1.gcode $OUT/${M}_planar/plate_1.gcode $OUT/${M}_today/plate_1.gcode $OUT/${M}_graded2W/plate_1.gcode $OUT/${M}_graded1/plate_1.gcode >> $R 2>&1
done
echo "== slices done $(date -u +%H:%M:%S) UTC" >> $R
