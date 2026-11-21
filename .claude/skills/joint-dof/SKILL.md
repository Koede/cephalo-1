---
name: joint-dof
description: Degrees-of-freedom and mobility analysis of any cephalo-1 joint or the whole arm - hinge axis direction, parallel/redundant joints, tip Jacobian rank at standard and random poses, singular values, redundancy, optional collision-limited effective joints. Use for questions on DOF, singularities or redundancy.
---
# joint-dof

`python3 tools/joint_dof.py [--joint N|a-b] [--samples 200] [--range 90] [--effective --min-travel 10] [--length L --joints N --twist T]`

- Each joint is a pin hinge: exactly 1 revolute DOF about axis `(cos(j*twist), sin(j*twist), 0)`. With 60 deg twist, joint j is parallel to j+-3, j+-6...
- Arm: 13 revolute joints, 6 x 13 tip Jacobian. Expected: rank 4 straight (singular), 5 curl/zigzag/coil, 6 wave and all generic random poses, so 7 redundant DOF at a generic pose and position-only rank 3.
- Pure numpy via `tools/kin.py` (`Kin.jacobian`, `Kin.dof`, `Kin.ik_position`); takes under 2 s.
- `--effective` combines with joint-range: counts joints with at least `--min-travel` deg of collision-free travel each way.
- Exit 1 if no sampled pose reaches rank 6. Explain in plain words: a hinge cannot be more than 1 DOF; "DOF of a joint" is its 1 DOF plus how many others duplicate its axis.
