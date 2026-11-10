---
name: servo-catalog
description: Browse and compare servos from the cephalo-1 catalog (servos.py, joint-study/catalog) by torque, speed, size, mass and recommend swaps against the gates. Use when considering STS3215/STS3250 alternatives.
---
# servo-catalog

Read `joint-study/sim/servos.py` and `joint-study/catalog/`. Table: model, stall torque, speed, case LxWxH, mass, voltage. The CAD assumes one shared case (45.2 x 24.7 x 35.0 mm, spline 11 mm from the end, horn seat 4 mm above the top `[A]`); a servo with another case needs `SERVO` edits and `/tolerance-stackup`. For a swap show the effect on `m_servo`, arm mass, T/W (`/mass-budget`) and joint safety factor (`/joint-torque-check`).
