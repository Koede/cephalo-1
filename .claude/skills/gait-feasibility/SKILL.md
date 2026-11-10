---
name: gait-feasibility
description: Check walking stance and gait feasibility for the 8-armed cephalo-1 using the joint-study scripts (pod_stance.py, pod_gait.py, pod_fold_move.py) and kinematic IK - foot reach, stability margin, required joint ranges. Use when changing arm length, joint count or the fan pod.
---
# gait-feasibility

`python3 tools/study_report.py stance` reads `pod_stance.json` (servo use by gait on the folded pod: 0.43 of stall at 200 mm hub height, 0.99 at twice the foot load: WARN above 0.9). For IK of a specific foot target use `Kin.ik_position(target, limit=radians(45))` (`/fk-ik-playground`) and verify with `/collision-check --pose`. Recompute with `cd joint-study/sim && python3 pod_stance.py` / `pod_gait.py` / `pod_fold_move.py` (overwrite committed JSON: check `git status`).
