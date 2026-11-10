---
name: bearing-load-check
description: Check cephalo-1 bearing loads (root gear-side bearing 61803/6800, pinion bearing, horn bearing, joints 1-4 side load) against bearing ratings and idler-pin stress, from DESIGN.md and the JSON reports. Use when thrust, mass or fan pod change.
---
# bearing-load-check

`python3 tools/study_report.py bearings` reads `joint-study/results/bearing_loads.json` ("after the bearing check"): per load case the gear bearing (61803) and pinion bearing (695) static safety factor s0 (WARN below 1.5, FAIL below 1.0) and the 8 mm 7075 idler pin vs 500 N. Full-throttle rows are WARN only (beyond rated). Horn bearing ratings: `python3 tools/check_gates.py`. Recompute: `cd joint-study/sim && python3 bearing_loads.py` (overwrites the JSON).
