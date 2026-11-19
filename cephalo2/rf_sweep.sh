#!/bin/bash
# usage: rf_sweep.sh case tag "ENV=.. ENV=.." ...   (pairs run 2 at a time)
case=$1; shift
run() { tag=$1; shift; env FALL_TAG=_rf$tag FALL_T=${FT:-3.5} "$@" python3 fall4.py $case > rf_${case}_$tag.log 2>&1;
  echo "$tag $* :: $(grep -o 'gear [0-9.]* of limit' rf_${case}_$tag.log) | $(grep -o 'decel (10 ms) [0-9]* g' rf_${case}_$tag.log) | $(grep -o 'slip [0-9]* deg' rf_${case}_$tag.log|head -1) | $(grep -o "worst gear at.*" rf_${case}_$tag.log | sed 's/.t.: [0-9.]*,//')"; }
while [ $# -gt 0 ]; do
  a="$1"; b="$2"; shift 2 2>/dev/null || shift
  ta=${a%%:*}; ea=${a#*:}; run $ta $ea &
  if [ -n "$b" ]; then tb=${b%%:*}; eb=${b#*:}; run $tb $eb & fi
  wait
done
