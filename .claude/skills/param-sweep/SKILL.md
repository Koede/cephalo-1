---
name: param-sweep
description: Sweep one CAD parameter of cephalo-1 (length, joint count, twist, command range, root servos, hub radius) and show which values pass all gates. Use for design-space exploration.
---
# param-sweep

1. Pick the parameter and values (e.g. `--joints 9..14`, `--length 600..1100 step 100`, `--twist 45,60,75,90`).
2. For each value run a geometry-only build (`tools/common.build_arm(length, joints, twist, geared=False)`), compute `pitch`, `pitch_min`, OD, and the gates: static interference, joint 0/1 `joint_sweep`, `tools/kin.py` tip reach and Jacobian rank.
3. Print a table with PASS/FAIL per gate and plot with matplotlib if asked (save under /tmp).
4. Note that `PinArm` raises `SystemExit` when the pitch is below `pitch_min`: record that as "infeasible", do not crash the sweep.

For hub radius use `python3 arm-pin-hinge/mantle_notch.py --hub-radius R --out /tmp/x` and read `flange_overlap_at_hub_radius_mm3`.
