#!/usr/bin/env python3
"""Range of motion of any joint of the arm by mesh interference.

  python3 tools/joint_range.py                 # every joint of the main design (1 m, 13 joints)
  python3 tools/joint_range.py --joint 0
  python3 tools/joint_range.py --joint 3,5-8 --step 1 --with-next 30
  python3 tools/joint_range.py --length 585 --joints 9 --no-gear

Method: turn link j+1 about joint j's hinge from 0 outward each way in --step degree steps until
the interference volume of any checked link pair exceeds 2 mm^3 (pin_cad's contact threshold), then
bisect to 0.05 deg. Pairs checked: (j, j+1) and next-neighbours (j-1, j+1), (j, j+2). With
--with-next D the following joint is held at D deg (and --with-prev D likewise for the previous
one), which shows how far a joint closes up when its neighbours are already bent.
Writes joint_range.json (use --out to change). Exit 1 if any joint is short of --need (default 45).
"""
import argparse, json, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import build_arm, build_bodies, parse_joint_arg, table
from geom import pose_overlaps, worst, OVERLAP_MM3


def check_pose(arm, bodies, q, j):
    J = arm.J
    pairs = [(j, j + 1)]
    if j - 1 >= 0: pairs.append((j - 1, j + 1))
    if j + 2 <= J: pairs.append((j, j + 2))
    return worst(pose_overlaps(arm, bodies, q, pairs))


def limit(arm, bodies, j, sign, step, lim, base):
    """first-contact angle (deg, >= 0) turning joint j in direction sign; (angle, pair)"""
    def bad(deg):
        q = list(base); q[j] = sign * math.radians(deg)
        return check_pose(arm, bodies, q, j)
    good, pair = 0.0, None
    if bad(0.0)[0] is not None:
        return 0.0, bad(0.0)[0]            # already touching at the start pose
    d = step
    while d <= lim + 1e-9:
        p, v = bad(d)
        if p is not None:
            lo, hi = good, d
            while hi - lo > 0.05:
                mid = 0.5 * (lo + hi)
                if bad(mid)[0] is None: lo = mid
                else: hi = mid
            return round(lo, 2), p
        good = d; d += step
    return lim, None                       # free all the way to --lim


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", default="all", help="index, list 0,2,5, range 2-6 or all")
    ap.add_argument("--length", type=float); ap.add_argument("--joints", type=int)
    ap.add_argument("--twist", type=float); ap.add_argument("--no-gear", action="store_true", help="plain pin_cad arm instead of the geared-root main design")
    ap.add_argument("--step", type=float, default=2.0); ap.add_argument("--lim", type=float, default=90.0)
    ap.add_argument("--need", type=float, default=45.0, help="required one-sided travel in deg")
    ap.add_argument("--with-next", type=float, default=0.0, help="hold joint j+1 at this angle (deg)")
    ap.add_argument("--with-prev", type=float, default=0.0, help="hold joint j-1 at this angle (deg)")
    ap.add_argument("--out", default="joint_range.json")
    a = ap.parse_args()
    arm = build_arm(a.length, a.joints, a.twist, geared=not a.no_gear)
    bodies = build_bodies(arm)
    rows, res = [], []
    for j in parse_joint_arg(a.joint, arm.J):
        base = [0.0] * arm.J
        if j + 1 < arm.J: base[j + 1] = math.radians(a.with_next)
        if j - 1 >= 0: base[j - 1] = math.radians(a.with_prev)
        hi, ph = limit(arm, bodies, j, +1, a.step, a.lim, base)
        lo, pl = limit(arm, bodies, j, -1, a.step, a.lim, base)
        ok = min(hi, lo) >= a.need
        res.append(dict(joint=j, lo=-lo, hi=hi, limit_lo=pl, limit_hi=ph, ok=ok))
        f = lambda p: "none (free to --lim)" if p is None else f"link {p[0]} vs {p[1]}"
        rows.append([j, f"-{lo:g}", f"+{hi:g}", f(pl), f(ph), "ok" if ok else f"SHORT of {a.need:g}"])
    print(f"arm: {arm.J} joints, pitch {arm.pitch:.1f} mm, twist {arm.twist:g} deg; contact = overlap > {OVERLAP_MM3} mm^3")
    print(table(rows, ["joint", "lo", "hi", "limited by (-)", "limited by (+)", "vs need"]))
    json.dump(dict(step=a.step, with_next=a.with_next, with_prev=a.with_prev, joints=res), open(a.out, "w"), indent=1)
    sys.exit(0 if all(r["ok"] for r in res) else 1)


if __name__ == "__main__":
    main()
