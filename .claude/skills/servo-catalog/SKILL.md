---
name: servo-catalog
description: Browse and compare servos from the cephalo-1 catalog (servos.py, joint-study/catalog) by torque, speed, size, mass and recommend swaps against the gates. Use when considering STS3215/STS3250 alternatives.
---
# servo-catalog

`python3 tools/study_report.py servos` lists `joint-study/sim/servos.py` sorted by stall torque: mass, N m per kg, speed, price, case size, and whether it fits the CAD case (45.2 x 24.7 x 35.0 mm). Anything else needs a new cradle: re-run `/tolerance-stackup` with the new `SERVO` dims and `/mass-budget --add-g`.
