---
name: cad-review
description: Review a branch or PR that touches cephalo-1 CAD or simulation - run the gates and golden diff, list changed parts, flag missing doc updates and unmarked assumptions. Use when asked to review CAD changes.
---
# cad-review

1. `python3 tools/changelog_from_diff.py` for what changed (commit first).
2. `python3 tools/check_all.py` (regenerates in scratch and diffs against committed outputs: a difference means the PR is stale or the outputs were hand-edited).
3. `python3 tools/design_doc_sync.py` for stale docs.
4. By hand: new parameters marked `[A]` until measured; no hard-coded absolute paths; scripts respect `--out`; no scratch files or large `*.csv.gz` committed.
Report findings by severity with file:line. Do not post comments unless asked.
