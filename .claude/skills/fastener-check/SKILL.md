---
name: fastener-check
description: Check cephalo-1 fastener and insert geometry (M3 horn bolt circle, heat-set insert holes, edge distance, screw bosses, lid screws) from the CAD parameters against the DESIGN.md assumptions. Use before printing a prototype.
---
# fastener-check

Read the constants at the top of `arm-pin-hinge/pin_cad.py` (`HORN_PCD`, `HORN_R`, `INSERT_R`, `LID_BOSS_R`, `BOSS_R`, `WALL`, `HEAD_H`) and apply:

- Horn bolt circle 14 mm with 4 x M3: bolt-hole edge to horn rim = `HORN_R - HORN_PCD/2 - 1.5 >= 1.5 mm`.
- Heat-set insert hole r = 2.0 (4.0 mm for a 4.5 mm M3 insert): wall around it >= 1.6 mm (use `LID_BOSS_R - INSERT_R`).
- Insert depth vs part thickness (`IDLER_EAR_T`, `LID_T`): insert length 5.7 mm must not exceed thickness minus 1 mm; flag otherwise.
- Screw head clearance vs arm OD (`HEAD_H` + ear stack within `R`).
Compare with the `[A]` assumptions in DESIGN.md "values to measure before printing" and print PASS/WARN lines. State which values are assumed, not measured.
