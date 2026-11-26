---
name: study-compare
description: Compare two cephalo-1 joint-study runs or scenarios (compare_scenarios.py) - which candidates entered or left the Pareto front and how the scores moved. Use after changing config, servo catalog or arm length.
---
# study-compare

Run `python3 joint-study/sim/compare_scenarios.py` (read its docstring for inputs). Otherwise load two `pareto_front.csv.gz` with pandas, join on the candidate key columns and report entered/left/moved (> 5% in each objective). Present the top 5 changes and say which config differences explain them.
