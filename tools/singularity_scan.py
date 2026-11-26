#!/usr/bin/env python3
"""Scan pose space for Jacobian rank drops / ill-conditioning (kinematic only, fast).

  python3 tools/singularity_scan.py --samples 20000
  python3 tools/singularity_scan.py --joint-pair 3,4 --grid 41      # 2-D map of two joints, rest 0
Reports the fraction of poses with rank < 6, cond > --cond-max, the worst poses (joint:deg list)
and the minimum 6th singular value. Exit 1 if any random pose within +-range/2 has rank < 6.
"""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import build_arm
from kin import Kin


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--samples", type=int, default=5000); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--range", type=float, default=90.0); ap.add_argument("--cond-max", type=float, default=1e5)
    ap.add_argument("--joint-pair", default=None); ap.add_argument("--grid", type=int, default=31)
    ap.add_argument("--length", type=float); ap.add_argument("--joints", type=int); ap.add_argument("--twist", type=float)
    a = ap.parse_args()
    k = Kin.from_arm(build_arm(a.length, a.joints, a.twist)); half = a.range / 2
    if a.joint_pair:
        i, j = map(int, a.joint_pair.split(",")); g = np.linspace(-half, half, a.grid); low = []
        for x in g:
            for y in g:
                q = np.zeros(k.J); q[i], q[j] = np.radians(x), np.radians(y)
                r, s, c = k.dof(q)
                if r < 6 or c > a.cond_max: low.append((x, y, r))
        print(f"joints {i},{j} over +-{half:g} deg ({a.grid}^2 grid, others 0): {len(low)} poses rank<6 or cond>{a.cond_max:g} "
              f"(expected: the rest of the arm is straight, so rank stays <6)")
        sys.exit(0)
    rng = np.random.default_rng(a.seed); ranks = np.zeros(a.samples, int); s6 = np.zeros(a.samples); cond = np.zeros(a.samples); Q = []
    for n in range(a.samples):
        q = np.radians(rng.uniform(-half, half, k.J)); r, s, c = k.dof(q)
        ranks[n], s6[n], cond[n] = r, s[min(5, len(s) - 1)], c; Q.append(q)
    print(f"{a.samples} random poses within +-{half:g} deg: rank<6 {int((ranks<6).sum())}, cond>{a.cond_max:g} {int((cond>a.cond_max).sum())}, "
          f"min s_6 {s6.min():.3g}, median cond {np.median(cond):.2e}")
    for n in np.argsort(cond)[::-1][:3]:
        print(f"  worst cond {cond[n]:.2e}: --pose " + ",".join(f"{j}:{np.degrees(v):.0f}" for j, v in enumerate(Q[n])))
    sys.exit(1 if (ranks < 6).any() else 0)


if __name__ == "__main__":
    main()
