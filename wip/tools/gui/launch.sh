#!/bin/bash
# launch.sh MODEL [LOG]: start OrcaNP on the Xvfb display with the test data dir.
S=/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad
for p in $(pgrep -f "^/home/user/OrcaNP/build/src/Release/orca-slicer"); do kill -9 $p; done; sleep 2
cd $S/gui && (DISPLAY=:99 LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 /home/user/OrcaNP/build/src/Release/orca-slicer --datadir $S/gui/datadir "$1" > "${2:-$S/gui/orca.log}" 2>&1 &)
