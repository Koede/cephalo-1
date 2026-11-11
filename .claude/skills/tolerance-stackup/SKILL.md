---
name: tolerance-stackup
description: Perturb cephalo-1 CAD dimensions (servo clearance, wall, horn size, boss radius, ear gap, servo L/W/H) by a print tolerance and re-check fit, to show which dimensions the design is sensitive to. Use before committing to tolerances for printing.
---
# tolerance-stackup

`python3 tools/tolerance_stackup.py [--param NAME]... [--delta 0.3] [--joint 0]` (about 1 s per case; ~25 s for the default list of 12 parameters, both signs).

Output: per parameter and sign, static interference, travel of the chosen joint, pitch_min, OD and a verdict. `SENSITIVE` = static > 0.5 mm^3 or travel < +-45. `ASSERT` = a built-in assertion in root_gear.py trips, meaning the design is locked to that value (CLEAR is one: the gear stage is tied to it). Nothing is written to disk. Present the sensitive parameters first and suggest which to measure on the real servo (DESIGN.md `[A]` list).
