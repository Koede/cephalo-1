---
name: variant-compare
description: Regenerate several cephalo-1 arm variants (585 mm, 900 mm, 1 m, twist 90, different root-servo counts) and tabulate pitch, OD, mass, travel and collision results side by side. Use when choosing arm length, joint count or twist.
---
# variant-compare

`python3 tools/variant_compare.py [--variant LENGTH:JOINTS:TWIST]...` (joints 0 = as many as fit).

Default set: 585, 700, 900/13, 1000/13, 1000/11, twist 90, twist 45. Columns: joints, pitch / minimum pitch, OD, static interference, joint 0 and 1 travel, verdict (static < 0.5 mm^3, joint 0 covers +-45, pairs clear). Geometry only, no files written. Uses the plain pin_cad arm (the geared root exists only for the main design), so joint 0 shows +-90 here; the geared main design is +-55 (`/joint-range`).
