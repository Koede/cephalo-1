#!/usr/bin/env python3
"""Overlap volume vs joint angle for neighbouring link pairs (where does a collision start, how fast).

  python3 tools/interference_heatmap.py --joint 0                  # CSV + PNG in the scratch dir
  python3 tools/interference_heatmap.py --joint 0 --lim 70 --step 1 --out joint0
  python3 tools/interference_heatmap.py --joint 3 --with-next 30
Pairs: (j, j+1), (j-1, j+1), (j, j+2). Writes <out>.csv (angle, pair, mm^3) and <out>.png (log y, with the
2 mm^3 contact line) and prints, per pair and direction, the first angle above 2 mm^3 and the growth
rate (mm^3 per deg) over the next 5 deg: steep = hard stop, shallow = a graze tolerances could clear.
"""
import argparse, csv, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import build_arm, build_bodies, SCRATCH
from geom import pose_overlaps, OVERLAP_MM3


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", type=int, default=0); ap.add_argument("--lim", type=float, default=90.0); ap.add_argument("--step", type=float, default=2.0)
    ap.add_argument("--with-next", type=float, default=0.0); ap.add_argument("--with-prev", type=float, default=0.0)
    ap.add_argument("--out", default=None); ap.add_argument("--no-png", action="store_true")
    a = ap.parse_args()
    arm = build_arm(); bodies = build_bodies(arm); J = arm.J; j = a.joint
    if not 0 <= j < J: sys.exit(f"joint must be 0..{J-1}")
    pairs = [(j, j + 1)] + ([(j - 1, j + 1)] if j else []) + ([(j, j + 2)] if j + 2 <= J else [])
    angs = np.arange(-a.lim, a.lim + 1e-9, a.step); data = {p: [] for p in pairs}
    for d in angs:
        q = [0.0] * J; q[j] = math.radians(d)
        if j + 1 < J: q[j + 1] = math.radians(a.with_next)
        if j: q[j - 1] = math.radians(a.with_prev)
        ov = pose_overlaps(arm, bodies, q, pairs)
        for p in pairs: data[p].append(ov[p])
    out = a.out or os.path.join(SCRATCH, f"interference_joint{j}"); os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out + ".csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["angle_deg"] + [f"link{p[0]}_vs_link{p[1]}_mm3" for p in pairs])
        for n, d in enumerate(angs): w.writerow([f"{d:g}"] + [f"{data[p][n]:.4g}" for p in pairs])
    print(f"joint {j}: wrote {out}.csv")
    for p in pairs:
        v = np.array(data[p])
        for sgn, name in ((1, "+"), (-1, "-")):
            idx = [n for n, d in enumerate(angs) if sgn * d > 0 and v[n] > OVERLAP_MM3]
            idx.sort(key=lambda n: abs(angs[n]))
            if not idx: print(f"  link {p[0]} vs {p[1]} {name}: no contact within +-{a.lim:g} deg"); continue
            n0 = idx[0]; d0 = angs[n0]; nxt = [m for m in range(len(angs)) if abs(angs[m] - (d0 + sgn * 5)) < a.step / 2 + 1e-9]
            rate = (v[nxt[0]] - v[n0]) / 5.0 if nxt else float("nan")
            print(f"  link {p[0]} vs {p[1]} {name}: first contact {d0:+.0f} deg ({v[n0]:.1f} mm^3), growth {rate:.1f} mm^3/deg over the next 5 deg")
    if not a.no_png:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 4))
        for p in pairs: ax.semilogy(angs, np.maximum(data[p], 1e-3), label=f"link {p[0]} vs {p[1]}")
        ax.axhline(OVERLAP_MM3, color="k", ls="--", lw=0.8); ax.set_xlabel("joint angle (deg)"); ax.set_ylabel("overlap (mm^3)"); ax.legend(); ax.set_title(f"joint {j}")
        fig.tight_layout(); fig.savefig(out + ".png", dpi=110); print(f"wrote {out}.png")


if __name__ == "__main__":
    main()
