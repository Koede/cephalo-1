#!/bin/bash
cd /home/claude/octodrone
for pair in "cut hit" "inv drift"; do set -- $pair
  FALL_TAG= python3 fall4.py $1 > fin_$1.log 2>&1 & FALL_TAG= python3 fall4.py $2 > fin_$2.log 2>&1; wait; done
for pair in "cut hit" "inv drift"; do set -- $pair
  FALL_HW=rev4 FALL_TAG=_rev4 python3 fall4.py $1 > fin_rev4_$1.log 2>&1 & FALL_HW=rev4 FALL_TAG=_rev4 python3 fall4.py $2 > fin_rev4_$2.log 2>&1; wait; done
for m in 30 33 36 39 42; do
  FALL_HITM=$m FALL_HITDIR=0 FALL_TAG=_ts${m}_0 python3 fall4.py hit > ts_${m}_0.log 2>&1 &
  FALL_HITM=$m FALL_HITDIR=45 FALL_TAG=_ts${m}_45 python3 fall4.py hit > ts_${m}_45.log 2>&1; wait
  FALL_HITM=$m FALL_HITDIR=90 FALL_TAG=_ts${m}_90 python3 fall4.py hit > ts_${m}_90.log 2>&1 &
  FALL_HW=rev4 FALL_HITM=$m FALL_HITDIR=45 FALL_TAG=_tsr${m}_45 python3 fall4.py hit > tsr_${m}_45.log 2>&1; wait
done
echo ALLDONE
