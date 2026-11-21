---
name: mass-budget
description: Mass breakdown and budget for a cephalo-1 arm and the robot - printed parts, servos, hardware, geared root additions, added mass versus the 46.4 g budget, robot mass and thrust-to-weight. Use for weight questions or when a part changes.
---
# mass-budget

`python3 tools/mass_budget.py [DIR] [--add-g 5]`

Per-arm table (vertebrae, lids, tip, hardware, STS3250 and STS3215 servos) that must add up to `fit.json` arm_out_g, then the geared-root extras (socket, caps, gear, pinion, bearings, idler pin) from `root_gear.json`, added mass vs the 46.4 g budget (margin +0.1 g), robot kg and rated T/W. `--add-g` is a what-if per arm. Exit 1 if over budget or T/W < 1.15. PETG 1.27 g/cm^3 x 0.95 fill; bearing and hardware masses are `[A]`.
