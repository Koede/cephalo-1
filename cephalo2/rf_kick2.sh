#!/bin/bash
cd /home/claude/octodrone
FALL_TAG= python3 fall4.py hit > fin_hit.log 2>&1 & FALL_TAG= python3 fall4.py inv > fin_inv.log 2>&1; wait
for m in 30 33 36; do
  FALL_T=70 FALL_HITM=$m FALL_HITDIR=0 FALL_TAG=_ts${m}_0 python3 fall4.py hit > ts_${m}_0.log 2>&1 &
  FALL_T=70 FALL_HITM=$m FALL_HITDIR=45 FALL_TAG=_ts${m}_45 python3 fall4.py hit > ts_${m}_45.log 2>&1; wait
  FALL_T=70 FALL_HITM=$m FALL_HITDIR=90 FALL_TAG=_ts${m}_90 python3 fall4.py hit > ts_${m}_90.log 2>&1; done
echo ALLDONE
