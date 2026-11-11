---
name: fk-ik-playground
description: Forward kinematics (tip position/orientation from joint angles) and inverse kinematics (joint angles for a target tip point) for a cephalo-1 arm, with a round-trip check. Use for "where does the tip go" and "what angles reach this point".
---
# fk-ik-playground

```python
import sys; sys.path.insert(0, "tools")
from common import build_arm; from kin import Kin, named_pose
import numpy as np
k = Kin.from_arm(build_arm())            # 1 m, 13 joints, helix60
tip = k.tip(np.radians([...13 values...]))       # mm
q, err = k.ik_position([x, y, z], limit=np.radians(45))   # damped least squares, err in mm
```
Always report the residual `err`; if above 1 mm say the target is unreachable within +-45. IK is position-only (3 of the 7 redundant DOF used), seeds from straight unless `q0` is given, so try several seeds. Validate a result for collisions with `/collision-check --pose`. `tests/test_kinematics.py` holds the FK/Jacobian/IK round-trip tests.
