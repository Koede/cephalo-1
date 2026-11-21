---
name: prop-clearance
description: Clearance of the cephalo-1 claw and central props against the arms in all fold states using prop_clearance.py and fan_pod.py outputs. Use when changing the pod, hub radius or arm layout.
---
# prop-clearance

`python3 tools/study_report.py prop` reads `prop_clearance.json`: per prop and offset, from which claw height every direction within 30 deg of up is clear in every pose (WARN where none up to 120 mm; superseded by the folding fan pod, whose clearance `python3 tools/check_gates.py` checks from `fan_pod.json`). Recompute: `cd joint-study/sim && python3 prop_clearance.py`.
