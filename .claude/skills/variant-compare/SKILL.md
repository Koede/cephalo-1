---
name: variant-compare
description: Regenerate several cephalo-1 arm variants (585 mm, 900 mm, 1 m, twist 90, different root-servo counts) and tabulate pitch, OD, mass, travel and collision results side by side. Use when choosing arm length, joint count or twist.
---
# variant-compare

For each variant run `python3 tools/regen.py --variant X --out /tmp/cephalo_regen/X --no-sweep` (or with sweeps for travel), read `fit.json` (+ `root_gear.json` if geared) and tabulate:

joints, pitch, pitch_min, OD, arm_out_g, arm_all_g, joint 0/1/last travel, pair clearance, static interference, thrust-to-weight (geared only).

Variants to include by default: `585` (9 joints), `900` (13 joints), `main` (1000/13), `twist90`, and `--root-servos 0` / `3` / `5`. Then `python3 tools/collision.py --length L --joints N --no-gear --level static,pairs` for each. Highlight which variants break a gate. Everything goes to /tmp; nothing committed is touched.
