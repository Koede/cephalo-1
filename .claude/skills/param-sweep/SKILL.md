---
name: param-sweep
description: Sweep one CAD parameter of cephalo-1 (length, joint count, twist, command range, root servos, hub radius) and show which values pass all gates. Use for design-space exploration.
---
# param-sweep

`python3 tools/param_sweep.py {joints|length|twist|range} LO HI [--step S] [--length L --joints N --twist T --range R]`

Sweeps one parameter, others fixed (defaults 1000 mm, 13 joints, 60 deg, 90 deg range). Per value: arm builds (pitch >= minimum), static interference, joint 0/1 travel against range/2, pair clearance, and which gate fails first. Infeasible values are reported, not crashed on. For hub radius use `python3 arm-pin-hinge/mantle_notch.py --hub-radius R --out SCRATCH_DIR`.
