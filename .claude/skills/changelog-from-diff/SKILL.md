---
name: changelog-from-diff
description: Summarise a cephalo-1 CAD or code change in plain English for a PR description - which STLs and numbers changed, gate impact, what to reprint. Use before opening a PR.
---
# changelog-from-diff

`python3 tools/changelog_from_diff.py [--base main]` prints markdown: changed files grouped (generators, tools/tests/skills, CAD outputs, docs), every number that moved in changed JSON reports, STL files to reprint, and a verification stub. It diffs committed changes against the base (`origin/main` or `main`); commit first. Paste into the PR description and fill in the check table from `python3 tools/check_all.py`.
