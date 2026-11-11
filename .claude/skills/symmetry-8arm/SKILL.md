---
name: symmetry-8arm
description: Check the 8-arm layout of cephalo-1 - even spacing of mantle ports, cutter count, flange clearance at the hub radius, hole positions - from mantle_ports.json and the cutter STL. Use when changing hub radius or root flange.
---
# symmetry-8arm

`python3 tools/symmetry_8arm.py [DIR]`

From `mantle_port_cutters_x8_r110.stl`: 8 openings (largest bodies) of equal volume, equal radius, exactly 45 deg apart; from `mantle_ports.json`: 8 M3 holes, hole-edge distance, and the flange clearance at the hub radius used (WARN: overlap 408 mm^3 at R=110, clear from R=124.5). Test a new radius with `python3 arm-pin-hinge/mantle_notch.py --hub-radius 125 --out SCRATCH_DIR`.
