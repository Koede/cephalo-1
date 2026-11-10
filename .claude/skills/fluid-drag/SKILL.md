---
name: fluid-drag
description: Arm drag scaling with length for cephalo-1 in air or water using fluid_1m.py and fluid_len.py. Use when changing arm length or swimming/flying assumptions.
---
# fluid-drag

Run `python3 joint-study/sim/fluid_1m.py` and `fluid_len.py` (read the docstrings for medium and speed). Report drag force at the tip speed for 585 / 900 / 1000 mm, the scaling exponent, and the added servo torque at the root. State the medium and Reynolds-number assumptions. Outputs are JSON in `joint-study/results/`; check `git status`.
