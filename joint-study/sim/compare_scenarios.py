#!/usr/bin/env python3
"""Compare the baseline sweep with the low-friction tendon scenario, per architecture.
  python3 compare_scenarios.py   (reads results/sweep.csv and results/sweep_tendon_optimistic.csv)"""
import os, json, pandas as pd
HERE = os.path.dirname(os.path.abspath(__file__)); RES = os.path.join(HERE, "results")
ARCH = dict(local="servo in every node", graded="graded servo chain", tendon="base-box tendons",
            pushpull="push-pull rods", section="3-section tendons", hybrid="root servos + base-box tendons")
out = {}
for name, fn in [(n, f) for n, f in [("baseline", "sweep.csv"), ("tendon_optimistic", "sweep_tendon_optimistic.csv"), ("more_thrust", "sweep_more_thrust.csv")] if os.path.exists(os.path.join(RES, f))]:
    d = pd.read_csv(os.path.join(RES, fn), low_memory=False)
    d = d[d.get("error").isna()] if "error" in d else d
    d["arch"] = d.actuation.map(ARCH)
    rows = []
    for a, g in d.groupby("arch"):
        f = g[g.feasible == True]
        best = f.sort_values("score", ascending=False).head(1)
        fails = g["fail"].fillna("").str.split(",").explode()
        rows.append(dict(arch=a, configs=len(g), feasible=len(f),
                         best_score=float(best.score.iloc[0]) if len(best) else None,
                         best=(best.family.iloc[0] + " / " + best.arrangement.iloc[0] + f" / N{int(best.N.iloc[0])}") if len(best) else "",
                         best_dof=int(best.dof.iloc[0]) if len(best) else None,
                         min_walk_util=float(g.util_walk.min()), min_mantle_util=float(g.mantle_util.min()),
                         top_fail=fails[fails != ""].value_counts().head(3).to_dict()))
    out[name] = rows
    pd.DataFrame(rows).to_csv(os.path.join(RES, f"architectures_{name}.csv"), index=False)
json.dump(out, open(os.path.join(RES, "scenario_comparison.json"), "w"), indent=1, default=str)
for k, rows in out.items():
    print("==", k)
    for r in rows: print(r)
