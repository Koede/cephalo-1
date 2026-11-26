#!/usr/bin/env python3
"""Run the joint-study pipeline safely from anywhere, with output redirected away from the repo by default.

  python3 tools/study_run.py smoke                 # run_study.py --limit 20 into the scratch dir, then sanity-check the CSV (~30 s)
  python3 tools/study_run.py smoke --limit 50 --workers 4
  python3 tools/study_run.py full --workers 8      # the full sweep (~30 min on 4 cores): asks for --yes
  python3 tools/study_run.py full --yes --workers 8 --rank
Checks first: dependencies (numpy, scipy, numba, pandas, matplotlib, mujoco), the known path bugs in joint-study/sim
(reported, not fixed here), then runs the scripts with cwd = joint-study/sim. smoke never touches committed files.
full writes joint-study/sim/results/sweep.csv (default of run_study.py); --rank then runs rank.py (reads that file).
"""
import argparse, importlib, os, re, subprocess, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import SIM_DIR, SCRATCH, Checks


def preflight(C):
    for m in ("numpy", "scipy", "numba", "pandas", "matplotlib", "mujoco"):
        try: importlib.import_module(m); C.add(f"import {m}", True)
        except Exception as e: C.add(f"import {m}", False, "pip install -r joint-study/sim/requirements.txt")
    bad = []
    for fn in sorted(os.listdir(SIM_DIR)):
        if fn.endswith(".py"):
            t = open(os.path.join(SIM_DIR, fn), encoding="utf8", errors="ignore").read()
            if "/mnt/project-files" in t: bad.append(f"{fn}: hard-coded /mnt/project-files path")
            if re.search(r'makedirs\("results"\)', t): bad.append(f"{fn}: cwd-relative results dir")
    C.add("known path bugs in joint-study/sim", not bad, "; ".join(bad) if bad else "none", warn=True)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=["smoke", "full"]); ap.add_argument("--limit", type=int, default=20); ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--yes", action="store_true"); ap.add_argument("--rank", action="store_true"); ap.add_argument("--out", default=None)
    a = ap.parse_args()
    C = Checks("joint-study pipeline preflight"); preflight(C); C.report()
    if C.failed: sys.exit(1)
    cmd = [sys.executable, "run_study.py", "--workers", str(a.workers)]
    if a.mode == "smoke":
        out = a.out or os.path.join(SCRATCH, "smoke_sweep.csv"); os.makedirs(os.path.dirname(out), exist_ok=True); cmd += ["--limit", str(a.limit), "--out", out]
    else:
        if not a.yes: sys.exit("full sweep takes ~30 min on 4 cores and writes joint-study/sim/results/sweep.csv; rerun with --yes")
        out = a.out or os.path.join(SIM_DIR, "results", "sweep.csv")
        if a.out: cmd += ["--out", out]
    t = time.time(); r = subprocess.run(cmd, cwd=SIM_DIR)
    if r.returncode: sys.exit(r.returncode)
    import pandas as pd
    df = pd.read_csv(out); D = Checks(f"sweep output {out} ({time.time()-t:.0f} s)")
    D.add("rows written", len(df) > 0, f"{len(df)} rows, {len(df.columns)} columns")
    empty = [c for c in df.columns if df[c].isna().all()]
    D.add("no all-NaN columns", not empty, "none" if not empty else f"{len(empty)} all-NaN columns, e.g. {empty[:3]} (normal for a tiny smoke run)", warn=True)
    for col in ("total_mass_kg", "thrust_to_weight", "feasible"):
        if col in df.columns: D.add(f"column {col}", True, f"min {df[col].min()} max {df[col].max()}" if df[col].dtype != bool else f"{int(df[col].sum())} feasible")
    D.report()
    if a.mode == "full" and a.rank and not D.failed:
        sys.exit(subprocess.run([sys.executable, "rank.py", "--workers", str(a.workers)], cwd=SIM_DIR).returncode)
    sys.exit(1 if D.failed else 0)


if __name__ == "__main__":
    main()
