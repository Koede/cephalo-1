---
name: clearance-map
description: Minimum distance (not just overlap) between cephalo-1 part pairs in the straight pose and common poses, flagging anything under a margin such as 0.5 mm. Use when interference is zero but tolerances are a worry.
---
# clearance-map

1. Build bodies with `tools/common.build_arm()` / `build_bodies()`; place with `arm.pose_frames(q)` (see `tools/geom.py: pose_overlaps`).
2. For each neighbouring or next-neighbour pair convert to trimesh (`pin_cad.to_trimesh`) and compute the minimum distance: `trimesh.proximity.closest_point(mesh_a, mesh_b.vertices)` over a vertex sample, or inflate one body with `Manifold.offset`-style by sweeping `margin` (0.25, 0.5, 1.0 mm) via `body.minkowski`-free alternative: shrink by moving the clearance threshold and test `(A ^ B_dilated).volume()`.
3. Report a table (pair, pose, min distance mm) and flag distances below `--margin` (default 0.5 mm, the printer-tolerance proxy).
4. Poses: straight, `curl`, `wave` (`tools/kin.named_pose`), and each joint at +-45.
Use the mesh distance for the answer and say it is vertex-sampled (lower bound on accuracy ~ mesh edge length).
