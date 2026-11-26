---
name: joint-range
description: Determine the range of motion of any cephalo-1 joint (one, a list, or all) by mesh interference - exact first-contact angle each way, which link pair limits it, margin against the +-45 deg command range, optionally with neighbouring joints already bent. Use for "how far can joint N bend", "what limits joint 0", or checking a design change.
---
# joint-range

`python3 tools/joint_range.py [--joint all|N|0,2,5|2-6] [--step 2] [--lim 90] [--need 45] [--with-next DEG] [--with-prev DEG] [--length L --joints N --twist T --no-gear] [--out joint_range.json]`

- Turns link j+1 about hinge j from 0 outward each way until overlap of links (j,j+1), (j-1,j+1) or (j,j+2) exceeds 2 mm^3, then bisects to 0.05 deg.
- Result per joint: `lo`/`hi`, limiting link pair, `ok`/`SHORT of --need`. `free to --lim` means no contact up to 90 deg (the servo's own travel then limits).
- Main design check: joint 0 = +-55 deg (limited by link 0 vs 1: the root socket), all others free to +-90. This reproduces `fit.json`.
- `--with-next 30` shows how far joint j closes while joint j+1 is already at 30 deg.
- Exit 1 if any joint is below `--need`. Whole arm at step 5: about 5 s.
