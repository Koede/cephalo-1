#!/usr/bin/env python3
"""Cephalo-1 tentacle joint study: one command runs the whole screen.

  python3 run_study.py --workers 12            # full sweep (default 12 workers)
  python3 run_study.py --workers 4 --limit 50  # quick smoke run

Needs: numpy scipy numba (mujoco + matplotlib for the later stages: dynamics.py, rank.py).
Writes results/sweep.csv (one row per arm configuration) and results/sweep_errors.csv."""
import os
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMBA_NUM_THREADS"):
    os.environ.setdefault(_v, "1")   # one thread per worker; the pool does the parallelism
import argparse, csv, sys, time, random
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import sweep as S

def _init():
    os.environ.setdefault("NUMBA_NUM_THREADS", "1")

def _work(job):
    return S.run_job(job)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=12)
    ap.add_argument("--out", default=os.path.join(HERE, "results", "sweep.csv"))
    ap.add_argument("--limit", type=int, default=0, help="only the first N geometries (smoke test)")
    a = ap.parse_args()
    jobs = S.kin_jobs()
    random.Random(1).shuffle(jobs)
    jobs.sort(key=lambda j: -(j["cfg"]["n_nodes"] * (2 if S.CH.FAM[j["cfg"]["family"]]["bend"] == 2 else 1)))
    if a.limit: jobs = jobs[:a.limit]
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    S.run_job(jobs[-1])                     # compile numba kernels once before forking
    t0 = time.time(); n = 0; writer = None; errs = []
    with open(a.out, "w", newline="") as fh, Pool(a.workers, initializer=_init) as pool:
        for rows in pool.imap_unordered(_work, jobs, chunksize=2):
            n += 1
            for r in rows:
                if "error" in r: errs.append(r); continue
                if writer is None:
                    writer = csv.DictWriter(fh, fieldnames=list(r.keys()), extrasaction="ignore"); writer.writeheader()
                writer.writerow(r)
            if n % 50 == 0 or n == len(jobs):
                el = time.time() - t0
                print(f"{n}/{len(jobs)} geometries, {el/60:.1f} min, eta {el/n*(len(jobs)-n)/60:.1f} min", flush=True)
                fh.flush()
    if errs:
        with open(a.out.replace(".csv", "_errors.csv"), "w", newline="") as fe:
            w = csv.DictWriter(fe, fieldnames=list(errs[0].keys())); w.writeheader(); w.writerows(errs)
    print(f"done: {n} geometries, {len(errs)} errors, {(time.time()-t0)/60:.1f} min -> {a.out}")

if __name__ == "__main__":
    main()
