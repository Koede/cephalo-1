---
name: cad-review
description: Review a branch or PR that touches cephalo-1 CAD or simulation - run the gates and golden diff, list changed parts, flag missing doc updates and unmarked assumptions. Use when asked to review CAD changes.
---
# cad-review

1. `git diff --stat main...HEAD`; separate generators (`*.py`) from regenerated outputs (`out*/`).
2. Regenerate in scratch (`/cad-regen`) and compare: if committed outputs differ from a fresh regen the PR is stale or hand-edited (`python3 tools/golden_diff.py`).
3. `/check-all` results; call out any new FAIL or changed WARN number.
4. Check: new parameters marked `[A]` until measured; DESIGN.md updated (`/design-doc-sync`); no hard-coded absolute paths; `--out` respected; no scratch files committed; large csv.gz not added.
5. Report findings by severity with file:line; do not post comments unless asked.
