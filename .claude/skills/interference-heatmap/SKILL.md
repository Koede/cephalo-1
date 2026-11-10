---
name: interference-heatmap
description: Overlap volume per cephalo-1 link pair versus joint angle, exported as CSV and a plot, to see where and how fast a collision starts. Use to understand a limit found by joint-range or collision-check.
---
# interference-heatmap

1. Choose a joint `j` (or two joints for a 2-D map) and an angle grid (default -90..90, 1 deg).
2. For each angle call `tools.geom.pose_overlaps(arm, bodies, q, pairs)` with pairs (j,j+1), (j-1,j+1), (j,j+2).
3. Write `/tmp/interference_j<j>.csv` (angle, pair, volume) and a matplotlib line plot (log y) with the 2 mm^3 contact line.
4. Quote the angle where each pair crosses 2 mm^3 and how steeply (mm^3 per deg) it grows: a steep rise means a hard stop, a shallow one a graze that tolerances could clear.
Run with `import sys; sys.path.insert(0, 'tools')` from the repo root.
