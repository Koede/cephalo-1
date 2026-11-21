---
name: setup-env
description: Install and verify the Python dependencies for cephalo-1 (CAD: manifold3d, trimesh, numpy; study: scipy, numba, mujoco; tests: pytest). Use at the start of a fresh container or when an import error appears.
---
# setup-env

Run from the repo root.

1. `python3 -m pip install -r requirements.txt` (CAD, tools, tests). For the simulation study also `python3 -m pip install -r joint-study/sim/requirements.txt` (numba, mujoco, pandas).
2. `python3 tools/doctor.py` verifies every import, the committed outputs, the unit tests and the gates, and prints the next action.
3. If pip cannot reach PyPI (proxy/403), say so; never disable TLS verification.

On Windows use `python` or `py -3` instead of `python3`. All tools in `tools/` run from any directory.
