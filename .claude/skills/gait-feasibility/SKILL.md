---
name: gait-feasibility
description: Check walking stance and gait feasibility for the 8-armed cephalo-1 using the joint-study scripts (pod_stance.py, pod_gait.py, pod_fold_move.py) and kinematic IK - foot reach, stability margin, required joint ranges. Use when changing arm length, joint count or the fan pod.
---
# gait-feasibility

1. `cd joint-study/sim` and run `python3 pod_stance.py`, `python3 pod_gait.py`, `python3 pod_fold_move.py` (read each docstring first; they write JSON into `joint-study/results/`, which is committed, so check `git status` after).
2. Cross-check with `tools/kin.py`: for each foot target use `Kin.ik_position(..., limit=radians(45))`, report residual and joints at limit.
3. Report: stance footprint, minimum stability margin, max joint angle used vs the +-45 command range, any IK failure.
4. Warning: `fan_pod_loads.py` hard-codes `/mnt/project-files/...`; do not run it before fixing the path.
