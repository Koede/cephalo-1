---
name: stl-integrity
description: Check every STL of a cephalo-1 design for watertightness, winding, volume, degenerate faces, size, and stale duplicate xN files. Use before printing or exporting.
---
# stl-integrity

`python3 tools/stl_integrity.py [DIR] [--max-dim 400] [--counts]` (default DIR: committed `arm-pin-hinge/out_1m_13`).

Known findings on the committed design: `horn_bearing/stl/vertebra_bearing_x4.stl` has 2 non-manifold edges at z = 39.223 (slicers repair it; fix the CAD join), and `mantle_port_cutter_joint0_frame.stl` has degenerate faces (a cutter tool, not a printed part). Assemblies and cutters are only checked for finite, non-empty meshes.
Report per-file failures with the reason; a non-manifold count <= 4 is a WARN, anything else FAIL.
