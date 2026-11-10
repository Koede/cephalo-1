---
name: gear-check
description: Check the cephalo-1 root gear stage (40T/20T module 1, 7075-T6) - tooth counts, centre distance, tip/root radii, contact ratio, a Lewis bending stress estimate, bearing fit - from root_gear.py parameters. Use when changing the gear stage or thrust.
---
# gear-check

Constants in `arm-pin-hinge/root_gear.py`: `MOD`, `PA`, `Z_GEAR`, `Z_PIN`, `CDIST`, `FACE`, `SECTOR_DEG`, `AL_RHO`. Checks:

- centre distance = (Z1+Z2)/2 * MOD (also in `tests/test_geometry.py`), tip radii 21 / 11 mm, root radius = rp - 1.25 m.
- Contact ratio for standard 20 deg spur gears >= 1.2 (compute from base pitch and the path of contact).
- Lewis bending: sigma = Ft / (face * m * Y); Ft = joint-0 torque / pinion pitch radius; Y about 0.32 for Z=20; compare with 7075-T6 allowable (about 150 MPa fatigue) and report the safety factor.
- Backlash and sector range (`SECTOR_DEG` = +-55 covers +-45 command).
Run `python3 -m pytest -q tests/test_geometry.py` for the geometry asserts. Mark torque and allowable as assumptions.
