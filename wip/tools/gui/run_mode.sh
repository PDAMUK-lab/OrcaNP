#!/bin/bash
# run_mode.sh MODEL PRESET NAME [ZOOM_STEPS] [SLICE_WAIT_S]: prepare + preview screenshots of one mode.
S=/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad
X="python3 $S/xgui.py"
python3 $S/gui/setproc.py $S/gui/datadir/OrcaSlicer.conf "$2" > /dev/null
$S/gui/launch.sh "$1" "$S/gui/orca_$3.log"
sleep 55
$X shot $S/gui/fb $S/gui/report/$3_prepare.png 200 100 1200 800
$X click :99 1102 152                      # Slice plate
sleep "${5:-90}"
$X shot $S/gui/fb $S/gui/report/$3_sliced_raw.png 200 100 1200 800
$X click :99 930 196; sleep 1              # collapse the legend
$X move :99 1018 540
for i in $(seq 1 "${4:-6}"); do $X click :99 1018 540 4; sleep 0.3; done
sleep 1
$X click :99 1152 880; sleep 3             # end of the top layer
$X move :99 1300 250; sleep 2
$X shot $S/gui/fb $S/gui/report/$3_preview.png 200 100 1200 800
