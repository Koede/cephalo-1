---
name: design-doc-sync
description: Compare numbers quoted in DESIGN.md and README.md with the committed JSON/fit reports of cephalo-1 and flag stale text. Use after regenerating outputs or editing the design.
---
# design-doc-sync

`python3 tools/design_doc_sync.py [--dir DIR]`

Looks for ten quoted figures (pitch 76.9, OD 57.1, arm 1,243 g, added 46.2 g, budget 46.4 g, robot 15.08 kg, T/W 1.150, hub radius 124.5, horn-bearing T/W 1.045, joint 0 +-55) from the reports in DESIGN.md and README.md. MATCH if the true number appears in the docs, WARN if not (with a nearby line). It never edits: propose the replacement lines. DESIGN.md keeps earlier-stage numbers on purpose, so check each WARN by hand.
