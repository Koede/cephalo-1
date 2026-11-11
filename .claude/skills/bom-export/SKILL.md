---
name: bom-export
description: Export a cephalo-1 bill of materials (servos, bearings, gears, inserts, fasteners, printed parts counts x8 arms) as CSV and markdown with mass columns. Use when ordering parts.
---
# bom-export

`python3 tools/bom_export.py [DIR] [--arms 8] [--csv FILE]`

Servos (STS3250 x5 and STS3215 x8 per arm), bearings (61803, 695, horn 6705 x4), machined 7075 parts, hardware per joint, printed parts counted from the `_xN` in the STL names; markdown to stdout and a CSV (default `<tempdir>/cephalo/cephalo_bom.csv`, empty `unit_price` column). Quantities come from CAD notes, not a purchasing list: confirm before ordering.
