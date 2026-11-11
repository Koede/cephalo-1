---
name: regression-golden
description: Compare a regenerated cephalo-1 design with the committed outputs - STL volume/bbox/byte equality and JSON numbers within tolerance. Use to prove a refactor changed nothing.
---
# regression-golden

`python3 tools/regen.py && python3 tools/golden_diff.py <tempdir>/cephalo/regen/main [--golden DIR] [--rtol 1e-6] [--only substring]`. On the unchanged repo every main-design STL is byte-identical and every JSON equal; WARNs for files only in the committed folder (subassembly outputs, mantle cutters) are expected unless `/regen-subassemblies` ran. Exit 1 on any difference; list changed files and the first differing key.
