---
name: check-all
description: Run the full cephalo-1 verification chain (unit tests, regenerate main design, gates, collision, STL integrity, golden diff) and print one summary table. Use before committing or opening a PR that touches CAD or simulation code.
---
# check-all

Run in order, stop on an infrastructure error, but continue past check failures so the table is complete:

1. `python3 -m pytest -q tests`
2. `python3 tools/regen.py --out /tmp/cephalo_regen/main`
3. `python3 tools/check_gates.py /tmp/cephalo_regen/main`
4. `python3 tools/collision.py --level static,sweeps,pairs,mc --samples 100`
5. `python3 tools/stl_integrity.py /tmp/cephalo_regen/main`
6. `python3 tools/golden_diff.py /tmp/cephalo_regen/main` (WARN for subassembly files absent from the scratch dir is expected)
7. `python3 tools/joint_dof.py --samples 200`

Summarise as a table (step, result, key number). Quote the WARN lines verbatim (horn-bearing T/W 1.045, mantle flange overlap, non-manifold edges): they are known open items, not new regressions, unless the numbers changed. Total time: about 30 s.
