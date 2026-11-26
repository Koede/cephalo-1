---
name: doctor
description: One-screen health report for the cephalo-1 repo (python, dependencies, committed outputs present, tools run, git state, known stale paths). Use first when something fails unexpectedly or in a new session.
---
# doctor

`python3 tools/doctor.py [--fast]` (about 5 s, 25 s with the unit tests).

Reports PASS/WARN/FAIL for: python version, each dependency (required vs optional), git branch and changed files, committed main-design files, tools/tests/skills present, known `joint-study/sim` path bugs, the unit tests and the design gates. Ends with the next action. Quote any FAIL line verbatim and give the one pip command that fixes it.
