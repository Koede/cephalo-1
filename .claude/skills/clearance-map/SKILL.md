---
name: clearance-map
description: Minimum distance (not just overlap) between cephalo-1 part pairs in the straight pose and common poses, flagging anything under a margin such as 0.5 mm. Use when interference is zero but tolerances are a worry.
---
# clearance-map

`python3 tools/clearance_map.py [--margin 0.5] [--spacing 1.0] [--pose 0:30,2:-30] [--per-joint 45]` (about 50 s for the default 29 poses).

Minimum gap between next-neighbour links (i, i+2) in straight, curl, wave and each joint at +-45, by KD-tree on ~1 mm surface samples (an upper bound on the true gap within the spacing). Neighbours (i, i+1) touch at the hinge by design, so overlap there is `/collision-check`'s job. Exit 1 if any gap is under the margin. Main design: all 29 poses pass at 0.5 mm.
