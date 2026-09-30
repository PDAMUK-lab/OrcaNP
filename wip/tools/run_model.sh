#!/bin/bash
# run_model.sh MODEL: the three measurement rounds on one model, one after another, each alone.
S=/tmp/claude-0/-home-user/800bed29-fd75-5057-8f10-928593efb56e/scratchpad
M=$1
rm -f $S/ref_$M.bin
# A: multi-threading alone (the first variant is the model's 2 mm reference; CG gives the direct result)
$S/run_quality.sh ${M}_A "2mm mt 4t|2|-|-|0|1|4;auto direct|0|-|-|0|0|1;auto mt 4t|0|-|-|0|1|4;2mm mt 1t|2|-|-|0|1|1;2mm direct|2|-|-|0|0|1" $M
# B: graded meshes alone (direct solver)
$S/run_quality.sh ${M}_B "graded 2mm|0|2|0.1|0|0|1;graded 2mm W|0|2|0.1|1|0|1;graded 1mm|0|1|0.05|0|0|1;graded 1mm W|0|1|0.05|1|0|1" $M
# C: both (graded + multi-threaded, 4 threads)
$S/run_quality.sh ${M}_C "graded 2mm mt 4t|0|2|0.1|0|1|4;graded 2mm W mt 4t|0|2|0.1|1|1|4;graded 1mm mt 4t|0|1|0.05|0|1|4;graded 1mm W mt 4t|0|1|0.05|1|1|4" $M
echo "== model $M done $(date -u +%H:%M:%S)" >> $S/quality_${M}_C.txt
