---
name: geom-unit-tests
description: Run and extend the cephalo-1 pytest suite (kinematics, gear geometry, identical vertebrae, committed-design gates). Use after editing tools/ or arm-pin-hinge/.
---
# geom-unit-tests

`python3 -m pytest -q tests` (about 2 s). Files: `test_kinematics.py` (axis helix, rot_about_line, Jacobian vs finite difference, DOF ranks, IK), `test_geometry.py` (pitch, involute gear, centre distance, identical vertebra, static clear), `test_design_outputs.py` (gates on committed JSON). When adding a function to `tools/` or a gate, add a test next to these; keep each under 1 s. If a test fails after a deliberate CAD change, say which number moved before updating an expected value.
