---
name: joint-torque-check
description: Compare cephalo-1 joint torques with servo ratings (STS3250 4.9 N m, STS3215) at stretched, horizontal and lifting poses with safety factors, using the joint-study loads scripts. Use when changing length, mass at the tip or the fan pod.
---
# joint-torque-check

1. From `joint-study/sim` run `python3 joint_loads.py` (and `bearing_loads.py --a1 --a2 --tag X` for bearings); outputs are JSON in `joint-study/results/` (committed: check `git status`).
2. Library route: `metrics.element_moments` and `var_torques` with `chain.build_chain(cfg)` for custom poses.
3. Table: joint, load N m, servo, rating N m, safety factor (flag < 1.5); highlight joint 0 (geared 2:1 root) and joints 0-4 (STS3250).
4. State assumptions: static loads only unless `dynamics.py` is used; fan-pod thrust 16 N rated.
Needs scipy/numba (see `/setup-env`).
