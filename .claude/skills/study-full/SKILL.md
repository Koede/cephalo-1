---
name: study-full
description: Run the full cephalo-1 joint-study sweep (run_study, rank, tip, eval_main, build_page) with worker count, progress and runtime expectations. Use only when the user asks for the full study; it takes about 30 min on 4 cores.
---
# study-full

Confirm first (long). From `joint-study/sim`: `python3 run_study.py --workers N` (~30 min on 4 cores) -> `rank.py --workers N` -> `tip.py` -> `eval_main.py [--lengths ...]` -> `build_page.py OUT.html`. Set `ARM_LENGTH` in `config.py` (currently 1.000 m) for another length. Run in the background and poll the log, not with sleep loops. Outputs: `results/sweep.csv` (gz is gitignored), `pareto_front.csv.gz`, JSON and the HTML page. Report runtime, row count, Pareto size.
