---
name: symmetry-8arm
description: Check the 8-arm layout of cephalo-1 - even spacing of mantle ports, cutter count, flange clearance at the hub radius, hole positions - from mantle_ports.json and the cutter STL. Use when changing hub radius or root flange.
---
# symmetry-8arm

`python3 tools/collision.py --level mantle` (8 cutter bodies, mutual overlap) and read `arm-pin-hinge/out_1m_13/mantle_ports.json`: `hub_radius_used` 110 vs `min_hub_radius_flanges_clear` 124.5, `flange_overlap_at_hub_radius_mm3` 408 (open item: flanges overlap at R=110). Verify the 8 `holes` are 45 deg apart on the hub radius and `hole_edge_to_opening_mm` positive. Try `python3 arm-pin-hinge/mantle_notch.py --hub-radius 125 --out /tmp/mantle125` to confirm clearance at the minimum radius.
