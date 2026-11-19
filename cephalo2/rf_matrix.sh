#!/bin/bash
# rf_matrix.sh tag "ENV..."  -> runs cut hit32 hit36 hit40 inv drift impacts
tag=$1; envs=$2
jobs=("cut:" "hit:FALL_HITM=32" "hit:FALL_HITM=36" "hit:FALL_HITM=40" "inv:" "drift:")
out=""
i=0
for j in "${jobs[@]}"; do
  c=${j%%:*}; e=${j#*:}; n=$c$(echo $e | grep -o '[0-9]*$')
  ( env FALL_TAG=_m${tag}_$n FALL_T=4 $envs $e python3 fall4.py $c > rm_${tag}_$n.log 2>&1 ) &
  i=$((i+1)); if [ $((i % 2)) -eq 0 ]; then wait; fi
done; wait
for j in "${jobs[@]}"; do c=${j%%:*}; e=${j#*:}; n=$c$(echo $e | grep -o '[0-9]*$')
  printf "%-8s %-7s %s | %s | %s\n" $tag $n "$(grep -o 'gear [0-9.]* of limit' rm_${tag}_$n.log)" "$(grep -o 'decel (10 ms) [0-9]* g' rm_${tag}_$n.log)" "$(grep -o "per joint.*" rm_${tag}_$n.log)"; done
