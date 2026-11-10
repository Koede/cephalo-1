---
name: study-smoke
description: Quick smoke test of the cephalo-1 joint-study pipeline (run_study.py --limit 20, rank.py, cheap check scripts) after verifying the known path bugs. Use before a full study or after touching joint-study/sim.
---
# study-smoke

1. `/setup-env` with `joint-study/sim/requirements.txt` (numpy, scipy, numba, pandas, matplotlib, mujoco).
2. From `joint-study/sim`: `python3 run_study.py --limit 20 --workers 2`, then `python3 rank.py --workers 2 --short 40`.
3. Known bugs to check first, and fix or report: `fan_pod_loads.py` hard-codes `/mnt/project-files/octopus-drone/joint-study/...`; `sweep.py` writes to cwd-relative `results`; `make_options.py` references a non-existent `arm` directory; scripts disagree on `HERE/results` vs `HERE/../results`. `ls joint-study/sim/results joint-study/results` after the run to see where output landed.
4. Large `sweep*.csv.gz` / `pareto_front.csv.gz` are gitignored: confirm `git status` is clean of them.
