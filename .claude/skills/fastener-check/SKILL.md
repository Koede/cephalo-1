---
name: fastener-check
description: Check cephalo-1 fastener and insert geometry (M3 horn bolt circle, heat-set insert holes, edge distance, screw bosses, lid screws) from the CAD parameters against the DESIGN.md assumptions. Use before printing a prototype.
---
# fastener-check

`python3 tools/fastener_check.py [--min-wall 1.6] [--min-edge 1.5] [--insert-len 5.7] [--bolt-hole-r 1.7]`

Checks from the pin_cad.py constants: horn bolt hole edge to horn rim (1.30 mm, WARN against 1.5), bolt circle inside the ear boss, insert hole 4.0 mm, wall round the idler-stub insert (2.0 mm) and the lid-screw insert (1.5 mm, WARN against 1.6), insert depth, head stack inside the arm OD, horn-ear counterbore. Every line says which inputs are `[A]` assumptions. These are design-intent checks, not FEA.
