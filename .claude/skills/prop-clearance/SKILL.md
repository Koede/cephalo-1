---
name: prop-clearance
description: Clearance of the cephalo-1 claw and central props against the arms in all fold states using prop_clearance.py and fan_pod.py outputs. Use when changing the pod, hub radius or arm layout.
---
# prop-clearance

Run `python3 joint-study/sim/prop_clearance.py`; read `fan_pod/fan_pod.json` (`prop_plane_above_tip_plate_mm` 102, `prop_to_finger_top_mm` 23.5, `folded_disc_plane_from_arm_axis_mm` 68, `folded_disc_clear_of_arm_mm` 39.4, `interference_over_travel`). Table by fold state (0, 45, 88 deg): min clearance, pass if > 5 mm margin. `python3 tools/check_gates.py` already fails on interference or a folded disc touching the arm.
