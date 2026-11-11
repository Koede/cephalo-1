---
name: joint-torque-check
description: Compare cephalo-1 joint torques with servo ratings (STS3250 4.9 N m, STS3215) at stretched, horizontal and lifting poses with safety factors, using the joint-study loads scripts. Use when changing length, mass at the tip or the fan pod.
---
# joint-torque-check

`python3 tools/study_report.py joints` judges the committed `joint-study/results/joint_loads.json` (joints 1-4: fork-ear, stub and bore stresses vs PETG 45/25 MPa along/across layers at SF 2).

To recompute the loads: `cd joint-study/sim && python3 joint_loads.py` (needs numba/scipy; overwrites `joint-study/results/joint_loads.json`, so check `git status`). Servo-torque margins by pose are in the study results (`/study-smoke`).
