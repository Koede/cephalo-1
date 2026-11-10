---
name: setup-env
description: Install and verify the Python dependencies for cephalo-1 (CAD: manifold3d, trimesh, numpy; study: scipy, numba, mujoco; tests: pytest). Use at the start of a fresh container or when an import error appears.
---
# setup-env

1. Install: `pip install -r requirements.txt` from the repo root (CAD + tests). For the simulation study also `pip install -r joint-study/sim/requirements.txt`.
2. Verify: `python3 -c "import numpy, manifold3d, trimesh, scipy, matplotlib, pytest; print('ok')"`; add `numba, mujoco, pandas` when the study will run.
3. Smoke test: `python3 -m pytest -q tests` (about 2 s).
4. If pip cannot reach PyPI (403 / proxy errors), say so and read `/root/.ccr/README.md`; do not disable TLS verification.

The CAD scripts need `manifold3d` and `trimesh`, which `joint-study/sim/requirements.txt` does not list. Report anything still missing.
