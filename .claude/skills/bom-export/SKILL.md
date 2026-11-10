---
name: bom-export
description: Export a cephalo-1 bill of materials (servos, bearings, gears, inserts, fasteners, printed parts counts x8 arms) as CSV and markdown with mass columns. Use when ordering parts.
---
# bom-export

Sources: `fit.json` (servos text, masses), `root_gear.json` (bearings 61803/695, gear, pinion, idler pin, `hw_extra_g`), `pin_cad.py` constants (`HW_G` = 5 g/joint: 4 M3x6 horn screws, M3 idler screw + washer, 6 inserts, PTFE washer), `out_1m_13/stl` names for printed counts. Per arm: STS3250 x5, STS3215 x8, M3 hardware x13 joints. Multiply by 8 arms. Write `/tmp/cephalo_bom.csv` (item, spec, qty/arm, qty total, unit g, total g, source) and show the markdown. Leave price columns empty; mark all quantities derived from the CAD notes, not a purchasing list.
