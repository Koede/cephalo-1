---
name: stale-output-clean
description: Find committed cephalo-1 outputs that no script regenerates, orphan or duplicate STLs, and outputs whose generating command is not recorded. Use before tidying or releasing.
---
# stale-output-clean

1. `python3 tools/regen.py` then `python3 tools/golden_diff.py /tmp/cephalo_regen/main`: files only in the committed folder are either subassembly outputs or stale.
2. `python3 tools/stl_integrity.py` reports `no stale duplicates` per `name_xN.stl` family.
3. Known: `arm-pin-hinge/out/render.png` and `out_1m_13/root_gear_check.png` have no generator in the repo; `spec-sheet/cad_alt90` has no recorded command (probably `pin_cad.py --twist 90 --joints 13 --out ../spec-sheet/cad_alt90`); `joint-study/sim/make_options.py` points at a directory `arm` that no longer exists.
4. Report, propose deletions or a recorded command in the README, and **do not delete anything** without confirmation.
