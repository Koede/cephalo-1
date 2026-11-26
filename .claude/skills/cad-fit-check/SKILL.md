---
name: cad-fit-check
description: Assert every DESIGN.md gate for a cephalo-1 design from its JSON reports (static interference, joint travel, pair clearance, identical vertebrae, thrust-to-weight, mass budget, bearing, fan pod, mantle ports) with PASS/FAIL/WARN and a non-zero exit on failure. Use after any CAD change.
---
# cad-fit-check

`python3 tools/regen.py && python3 tools/check_gates.py <tempdir>/cephalo/regen/main`
(or `python3 tools/check_gates.py` for the committed `arm-pin-hinge/out_1m_13`).

Gates (DESIGN.md "Fit and collision check"): static interference ~0 mm^3; each swept joint covers +-45 deg (joint 0 is +-55 in the main design); joints 0+1 and 1+2 clear to +-60; vertebra 2 equals vertebra 1 turned 60 deg; thrust-to-weight >= 1.15 and added mass <= budget (46.2 vs 46.4 g); horn bearing and fan pod clear; mantle flanges clear.

WARN lines are known open items (horn-bearing rated T/W 1.045 with the bearing, central prop and pack; mantle flanges overlap 408 mm^3 at hub R=110 mm, clear from R=124.5). Report them, and say if a WARN's number changed. Flags: `--range`, `--tw`, `--static-tol`. A FAIL means stop and explain which gate and by how much.
