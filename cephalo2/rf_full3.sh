#!/bin/bash
cd /home/claude/octodrone
for c in cut hit inv drift; do FALL_HW=rev4 FALL_TAG=_rev4 nice python3 fall4.py $c > fin_rev4_$c.log 2>&1; done
for m in 30 33 36 39 42; do for a in 0 45 90; do
  FALL_HITM=$m FALL_HITDIR=$a FALL_TAG=_ts${m}_$a nice python3 fall4.py hit > ts_${m}_$a.log 2>&1; done
  FALL_HW=rev4 FALL_HITM=$m FALL_HITDIR=45 FALL_TAG=_tsr${m}_45 nice python3 fall4.py hit > tsr_${m}_45.log 2>&1
done
echo ALLDONE
