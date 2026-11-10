---
name: helix-check
description: Verify the cephalo-1 joint-axis helix (60 deg per joint) and that each moving vertebra is the previous one rotated, for any twist. Use after changing twist, joint count or the vertebra design.
---
# helix-check

`python3 -m pytest -q tests/test_kinematics.py tests/test_geometry.py -k "axis or helix or identical"` covers: successive axes differ by `twist`, axis j equals axis j+360/twist, `joint_affine` is orthonormal, and vertebra 2 minus vertebra 1 rotated by `twist` is < 0.5 mm^3. For a different twist run `python3 tools/regen.py --variant twist90` (or `--twist T`) and `python3 tools/check_gates.py DIR`: for twists that do not divide 360 the vertebrae are still identical as long as the helix is regular. Report the first-to-last axis angle `(J-1)*twist` (720 deg for 13 joints at 60).
