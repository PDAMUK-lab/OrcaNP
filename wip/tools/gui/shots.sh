#!/bin/bash
# shots.sh MODEL VARIANT ZOOM WAIT: slice MODEL in the GUI with the variant's environment and take an
# external view (all layers) and an internal one (layers up to half height).
S=/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad; X="python3 $S/xgui.py"
M=$1; V=$2; Z=$3; W=$4; O=$S/gui/q/${M}_$V
case $V in
  today)    E="" ;;
  graded2W) E="S4_PROTO_FACET_SIZE=2 S4_PROTO_FACET_DISTANCE=0.1 S4_PROTO_WEIGHTED=1 S4_PROTO_WARM=1" ;;
  graded1)  E="S4_PROTO_FACET_SIZE=1 S4_PROTO_FACET_DISTANCE=0.05 S4_PROTO_WARM=1" ;;
  today_opt)    E="S4_PROTO_WARM=1 S4_PROTO_MT=1" ;;
  graded2W_opt) E="S4_PROTO_FACET_SIZE=2 S4_PROTO_FACET_DISTANCE=0.1 S4_PROTO_WEIGHTED=1 S4_PROTO_WARM=1 S4_PROTO_MT=1" ;;
esac
env $E $S/gui/launch.sh $S/clitest/$M.stl $O.log
sleep 55
$X click :99 760 700; sleep 1
$X click :99 1102 152                       # Slice plate
# Wait for the Export G-code button to turn from grey to teal (sliced), at most W s, then let the preview settle.
t=0
until python3 -c "
import sys; sys.path.insert(0, '$S'); from xgui import shot
from PIL import Image
shot('$S/gui/fb', '/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad/gui/q/_poll.png')
r, g, b = Image.open('$S/gui/q/_poll.png').convert('RGB').getpixel((1300, 152))
sys.exit(0 if g > 170 and r < 130 else 1)" > /dev/null 2>&1 || [ $t -ge $W ]; do sleep 5; t=$((t + 5)); done
echo "$M $V sliced after ~$t s"
sleep 20
$X click :99 760 700; sleep 1
$X click :99 914 196; sleep 2               # collapse the legend
$X click :99 1281 797; sleep 2              # close a warning, if any
$X click :99 646 183; sleep 2               # collapse the sidebar
for i in $(seq 1 $Z); do $X click :99 800 520 4; sleep 0.3; done
sleep 2; $X move :99 1560 960; sleep 2
$X shot $S/gui/fb ${O}_external.png 200 100 1200 800
$X drag :99 1380 288 1380 536; sleep 3      # top layer to half height
$X move :99 1560 960; sleep 2
$X shot $S/gui/fb ${O}_internal.png 200 100 1200 800
