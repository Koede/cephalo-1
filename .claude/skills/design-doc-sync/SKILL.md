---
name: design-doc-sync
description: Compare numbers quoted in DESIGN.md and README.md with the committed JSON/fit reports of cephalo-1 and flag stale text. Use after regenerating outputs or editing the design.
---
# design-doc-sync

1. Pull the numbers: `fit.json`, `root_gear.json`, `horn_bearing.json`, `fan_pod.json`, `mantle_ports.json`, `fit_report.md` in `arm-pin-hinge/out_1m_13`.
2. Grep DESIGN.md for each (`grep -n "1\.150\|46\.2\|46\.4\|55\b\|1,242\|15\.08\|124\.5\|408" DESIGN.md`), also pitch 76.9, OD 57.1, arm masses, T/W values.
3. List mismatches as: DESIGN.md line, quoted value, current value, source file. Never edit without asking; propose the replacement lines.
4. Also check the README folder list against `git ls-files | cut -d/ -f1-2 | sort -u`.
