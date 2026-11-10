---
name: doctor
description: One-screen health report for the cephalo-1 repo (python, dependencies, committed outputs present, tools run, git state, known stale paths). Use first when something fails unexpectedly or in a new session.
---
# doctor

Run these and summarise as OK / WARN / FAIL lines:

- `python3 --version`; imports of numpy, manifold3d, trimesh, scipy, matplotlib, pytest, numba, mujoco (list which are missing and what each unlocks).
- `git status -sb` and `git log --oneline -3`; note uncommitted changes and branch vs `origin/main`.
- `python3 tools/check_gates.py` (committed main design gates) and `python3 -m pytest -q tests`.
- Known path bugs: `grep -rn "/mnt/project-files" joint-study/sim`, `grep -n 'makedirs("results")' joint-study/sim/sweep.py`, `grep -n "\\barm\\b" joint-study/sim/make_options.py`.
- Committed outputs with no generator in the repo: `arm-pin-hinge/out/render.png`, `arm-pin-hinge/out_1m_13/root_gear_check.png`.
- Scripts must run from `arm-pin-hinge/` (they import each other); `tools/*.py` run from anywhere.

End with the three most important actions.
