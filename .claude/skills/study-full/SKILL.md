---
name: study-full
description: Run the full cephalo-1 joint-study sweep (run_study, rank, tip, eval_main, build_page) with worker count, progress and runtime expectations. Use only when the user asks for the full study; it takes about 30 min on 4 cores.
---
# study-full

`python3 tools/study_run.py full --yes --workers N [--rank]` (about 30 min on 4 cores; it refuses without `--yes`).

Writes `joint-study/sim/results/sweep.csv` (default of `run_study.py`), then with `--rank` runs `rank.py`. Run it in the background and read the log; afterwards `tip.py`, `eval_main.py`, `build_page.py` per `joint-study/sim/README.md`. Set `ARM_LENGTH` in `joint-study/sim/config.py` first for another length.
