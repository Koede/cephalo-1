---
name: thrust-weight
description: Thrust-to-weight of cephalo-1 including central prop, claw fans, battery pack and geared-root mass - rated and full-throttle - plus sensitivity to added mass. Use for "can it still fly" questions.
---
# thrust-weight

`root_gear.json`: `robot_kg` 15.084, `thrust_to_weight` 1.150 (gate 1.15), `budget_g` per arm. DESIGN.md sections "Central prop", "Battery", "Claw fans" give the later totals (horn bearing: robot 17.18 kg, rated T/W 1.061 -> 1.045). Scripts: `joint-study/sim/central_prop.py`, `claw_fans.py`, `battery.py` (write JSON into `joint-study/results/`). Present: robot mass build-up, rated T/W, full-throttle T/W, hover draw, and d(T/W)/d(mass): `T/W = T/(m g)` so +100 g costs about 0.7%. Flag clearly that T/W < 1.15 after the additions is an open item recorded in DESIGN.md.
