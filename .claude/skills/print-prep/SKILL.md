---
name: print-prep
description: Pre-print checklist for cephalo-1 - values to measure on the real parts (the [A] assumptions), STL inventory per arm x8, material and support notes, integrity check. Use before printing a prototype.
---
# print-prep

1. `python3 tools/stl_integrity.py --counts`.
2. Parts per arm (main design): root socket x1, vertebra x12, tip x1, servo lid x12 (joint 0 has none), root end cap, bearing cap, gear 40T and pinion 20T (7075 machined), idler pin; x8 arms. Subassemblies: horn bearing vertebra x4 and lid x4, fan pod x8, palm finger x8.
3. Measure before printing (DESIGN.md `[A]`): spline centre to case end (11.0), horn seat height (4.0), horn thickness (3.0), horn radius (10.0) and bolt circle (14.0), insert hole (4.0 for 4.5 mm M3 insert). Run `/tolerance-stackup` with the measured deltas.
4. Material/process: PETG, supports needed, heat-set inserts, 3-4 perimeters at 0.6 mm; wall 1.8 mm.
5. Known: `vertebra_bearing_x4.stl` has 2 non-manifold edges; the slicer repairs them.
