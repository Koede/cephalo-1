---
name: changelog-from-diff
description: Summarise a cephalo-1 CAD or code change in plain English for a PR description - which STLs and numbers changed, gate impact, what to reprint. Use before opening a PR.
---
# changelog-from-diff

`git diff --stat main...HEAD`, `git diff main...HEAD -- '*.json' '*.md'`, and `python3 tools/golden_diff.py <regen dir>` if CAD changed. Output: Summary (2-3 lines), Parts that change geometry (reprint list: file, volume change), Numbers moved (before -> after, from JSON diffs), Gates (all PASS / which WARN changed), Not verified. Use the PR template if present (`.github/pull_request_template.md`).
