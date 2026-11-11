---
name: regen-subassemblies
description: Run the cephalo-1 sub-assembly generators (horn_bearing.py, fan_pod.py, mantle_notch.py) and summarise their asserts and JSON. Use after changing the root, horn, fan pod or mantle ports.
---
# regen-subassemblies

From `arm-pin-hinge/` (they import `pin_cad` and `root_gear`):

```bash
cd arm-pin-hinge && python3 horn_bearing.py && python3 fan_pod.py && python3 mantle_notch.py --length 1000 --joints 13 --hub-radius 110 --out out_1m_13
```

`horn_bearing.py` and `fan_pod.py` write into `out_1m_13/` (hard-coded), so this **modifies committed files**: check `git status`, tell the user, and run `git diff --stat` afterwards. Prefer running on a branch.

Summarise: horn bearing (static interference, sweeps, `tw_rated_after`, loads), fan pod (interference over travel, folded disc clearance, grip, gyro moment), mantle ports (`min_hub_radius_flanges_clear` vs `hub_radius_used`). `python3 tools/check_gates.py` reads all three.
