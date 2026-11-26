---
name: fluid-drag
description: Arm drag scaling with length for cephalo-1 in air or water using fluid_1m.py and fluid_len.py. Use when changing arm length or swimming/flying assumptions.
---
# fluid-drag

`python3 tools/study_report.py fluid` summarises `fluid_len.csv` (best feasible wave frequency, tip rms error, saturation, mass, T/W per arm length and servo fit). Recompute: `cd joint-study/sim && python3 fluid_len.py` (long; overwrites the CSV).
