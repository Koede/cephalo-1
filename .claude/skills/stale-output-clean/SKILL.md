---
name: stale-output-clean
description: Find committed cephalo-1 outputs that no script regenerates, orphan or duplicate STLs, and outputs whose generating command is not recorded. Use before tidying or releasing.
---
# stale-output-clean

`python3 tools/stale_outputs.py` (report only, never deletes).

Heuristic: looks for each committed image/svg/dxf/stl name in the python sources, lists `name_xN.stl` families with more than one N, and output folders not mentioned in README/DESIGN. Known: `root_gear_check.png` has no generator in the repo; `spec-sheet/cad_alt90` has no recorded command (probably `pin_cad.py --twist 90 --joints 13 --out ../spec-sheet/cad_alt90`). Propose, do not delete.
