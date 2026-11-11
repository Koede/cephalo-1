---
name: gear-check
description: Check the cephalo-1 root gear stage (40T/20T module 1, 7075-T6) - tooth counts, centre distance, tip/root radii, contact ratio, a Lewis bending stress estimate, bearing fit - from root_gear.py parameters. Use when changing the gear stage or thrust.
---
# gear-check

`python3 tools/gear_check.py [--torque 4.9] [--allow 150] [--need 45]`

Reads the constants from `root_gear.py`: centre distance, 2:1 ratio, undercut limit, contact ratio (1.64), root vs base circle, Lewis bending stress at pinion and gear for the given servo torque (stall 4.9 N m gives 153 / 126 MPa against an assumed 150 MPa allowable for 7075-T6, so WARN not FAIL), and that the tooth sector covers +-need deg. Torque and allowable are assumptions: say so. Bearing loads: `/bearing-load-check`.
