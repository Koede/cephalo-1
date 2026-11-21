---
name: study-smoke
description: Quick smoke test of the cephalo-1 joint-study pipeline (run_study.py --limit 20, rank.py, cheap check scripts) after verifying the known path bugs. Use before a full study or after touching joint-study/sim.
---
# study-smoke

`python3 tools/study_run.py smoke [--limit 20] [--workers 2]` (about 30 s).

Checks dependencies and the known path bugs (reported as WARN: `fan_pod_loads.py` hard-codes `/mnt/project-files`), runs `run_study.py --limit N` with `--out` in the scratch dir (never touches committed files), then checks the CSV (rows, columns, key columns). Needs `numba`, `scipy`, `pandas`, `mujoco` (`/setup-env`).
