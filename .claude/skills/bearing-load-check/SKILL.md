---
name: bearing-load-check
description: Check cephalo-1 bearing loads (root gear-side bearing 61803/6800, pinion bearing, horn bearing, joints 1-4 side load) against bearing ratings and idler-pin stress, from DESIGN.md and the JSON reports. Use when thrust, mass or fan pod change.
---
# bearing-load-check

Sources: `out_1m_13/root_gear.json` (`gear_bearing`, `pinion_bearing`), `horn_bearing/horn_bearing.json` (`loads`, `lid_screw_hole_MPa`, bearing dict) and DESIGN.md sections "Root bearing check", "Joints 1-4 side-load check", "Horn bearing". Report each load against the dynamic and static rating (C0) as a ratio; the design records 0.42-0.46 of C0 for the horn bearing at rated thrust within 30-60 deg of up, and 971-1,084 N on the 61803. Regenerate with `/regen-subassemblies` (modifies committed files). Flag any ratio > 0.5 of C0 and any pin stress above the figures in DESIGN.md (24 MPa full throttle on the idler ear).
