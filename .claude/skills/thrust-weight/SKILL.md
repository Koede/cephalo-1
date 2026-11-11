---
name: thrust-weight
description: Thrust-to-weight of cephalo-1 including central prop, claw fans, battery pack and geared-root mass - rated and full-throttle - plus sensitivity to added mass. Use for "can it still fly" questions.
---
# thrust-weight

`python3 tools/study_report.py thrust` gives T/W against added mass from `battery.json` (each +100 g costs 0.0076; headroom to 1.15 is only 13 g on that basis) and the main design's T/W from `root_gear.json` (1.150 at 15.08 kg). `python3 tools/mass_budget.py --add-g N` shows the per-arm effect. Later states (central prop, horn bearing 17.18 kg, T/W 1.045) are in DESIGN.md and `horn_bearing.json`; `/cad-fit-check` flags that as a WARN.
