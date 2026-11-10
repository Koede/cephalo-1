---
name: interference-heatmap
description: Overlap volume per cephalo-1 link pair versus joint angle, exported as CSV and a plot, to see where and how fast a collision starts. Use to understand a limit found by joint-range or collision-check.
---
# interference-heatmap

`python3 tools/interference_heatmap.py --joint N [--lim 90 --step 2] [--with-next DEG --with-prev DEG] [--out BASENAME]`

Writes `BASENAME.csv` (angle, overlap mm^3 per link pair) and `BASENAME.png` (log scale, 2 mm^3 contact line), default folder `<tempdir>/cephalo`. Prints the first contact angle per pair and direction and the growth rate over the next 5 deg: steep = hard stop, shallow = a graze that tolerances could clear. Main design joint 0: first contact +-60 deg (link 0 vs 1), 5.7 mm^3/deg.
