---
name: singularity-scan
description: Scan cephalo-1 arm pose space for kinematic singularities (Jacobian rank drops, ill-conditioning) with random sampling or a 2-joint grid map. Use to find poses to avoid or to qualify a gait.
---
# singularity-scan

`python3 tools/singularity_scan.py [--samples 5000] [--range 90] [--cond-max 1e5] [--joint-pair I,J --grid 31]`

Random mode prints the number of poses with rank < 6 or cond > cond-max, the minimum 6th singular value and the three worst poses as `--pose` strings (replay with `/collision-check --pose` or `/pose-render --angles`). Pair mode maps two joints with the rest straight; low rank there is expected because the remaining arm is straight. Main design: 0 rank-deficient poses in 5000, median cond about 2e3.
