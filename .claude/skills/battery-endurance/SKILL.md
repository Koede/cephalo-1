---
name: battery-endurance
description: Hover endurance and current-limit check for cephalo-1 packs using battery.py - energy, hover draw, climb current vs connector rating, what-if on pack capacity. Use when choosing the battery.
---
# battery-endurance

`python3 tools/study_report.py battery` tabulates `joint-study/results/battery.json`: per pack mass, hover minutes, current drawn vs continuous rating, rated and full-throttle T/W, with WARN for packs that cannot supply the hover current or drop T/W under 1.15 (all of them do). The EDF basis was superseded by the central prop (`central_prop.json`). Recompute: `cd joint-study/sim && python3 battery.py` (overwrites the JSON).
