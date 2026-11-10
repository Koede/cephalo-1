---
name: check-all
description: Run the full cephalo-1 verification chain (unit tests, regenerate main design, gates, collision, STL integrity, golden diff) and print one summary table. Use before committing or opening a PR that touches CAD or simulation code.
---
# check-all

`python3 tools/check_all.py [--quick | --full]`

- default (about 40 s): unit tests, regenerate the main design into a scratch dir, gates, collision (static, sweeps, pairs, Monte Carlo), STL integrity, golden diff vs the committed outputs, joint range, DOF, 8-arm symmetry, mass budget, gear check, fasteners, docs-vs-reports.
- `--quick` (about 8 s): tests, gates, static and pair collision, no regeneration.
- `--full`: adds clearance map, singularity scan, MuJoCo cross-check, study results.

Each step is its own process; the table shows PASS / WARN / FAIL with the step's last line. WARN lines are the known open items (horn-bearing T/W 1.045, mantle flange overlap 408 mm^3 at hub R=110, 2 non-manifold edges in `vertebra_bearing_x4.stl`): report them, and say if a number changed. A FAIL prints the step's last 12 lines.
