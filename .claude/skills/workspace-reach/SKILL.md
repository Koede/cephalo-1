---
name: workspace-reach
description: Reachable workspace of a cephalo-1 arm tip by kinematic Monte Carlo - max reach, tip z range, tips below the base plane, occupied volume, optional 8-arm ring geometry and a scatter PNG. Use for reach, ground-clearance and layout questions.
---
# workspace-reach

`python3 tools/workspace_reach.py [--samples 50000] [--range 90] [--hub-radius 110] [--png /tmp/reach.png] [--voxel 10]`

Kinematics only (no collision); combine with `/collision-check --level mc` for collision-free reach. Main design (1 m): max reach about 980 mm within +-45 deg, ~5.6% of random poses put the tip below the base plane. State the sample count and that it is a Monte Carlo estimate. The PNG is XZ and XY projections.
