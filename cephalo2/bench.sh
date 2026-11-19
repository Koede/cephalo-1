#!/bin/bash
# usage: bench.sh tag env...
tag=$1; shift
env FLIP_START=stand FLIP_T=14 "$@" python3 flip4.py > b_$tag.log 2>&1
cp flip4.npz b_$tag.npz
python3 -c "
import numpy as np
z=np.load('b_$tag.npz'); ph=z['phase']; L=z['load']; w=ph=='walk'
s=L[w].max(axis=(1,2)); print('$tag', 'walk frames', w.sum(), '>0.9:', (s>0.9).sum(), 'max', s.max().round(2), 'p95', np.percentile(s,95).round(2), 'mean per joint', L[w].mean(axis=(0,1)).round(2), 'up min', z['up'][w].min().round(1))
"
