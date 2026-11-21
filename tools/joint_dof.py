#!/usr/bin/env python3
"""Degrees of freedom / mobility of any joint, or of the whole arm.

  python3 tools/joint_dof.py                    # whole arm of the main design, standard poses
  python3 tools/joint_dof.py --joint 4          # axis, parallel/redundant joints, usable range of joint 4
  python3 tools/joint_dof.py --samples 500      # also scan random poses for rank drops

Per joint: revolute (1 DOF) about its hinge axis, direction in the arm frame, angle to the previous
joint's axis, and whether another joint's axis is parallel to it (redundant when both are straight).
Whole arm: rank of the 6 x J tip Jacobian at straight / curl / wave / zigzag / coil and at random
poses within +-range/2, singular values, condition number, and the redundancy J - rank.
--effective also uses joint_range.py's collision limits (a joint with usable travel < --min-travel deg counts as locked).
"""
import argparse, math, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import table, parse_joint_arg
from kin import Kin, axis_dir, named_pose
from common import build_arm


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--joint", default=None); ap.add_argument("--length", type=float)
    ap.add_argument("--joints", type=int); ap.add_argument("--twist", type=float)
    ap.add_argument("--range", type=float, default=90.0, help="total commanded travel (deg), poses use +-range/2")
    ap.add_argument("--samples", type=int, default=200); ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--effective", action="store_true"); ap.add_argument("--min-travel", type=float, default=10.0)
    a = ap.parse_args()
    arm = build_arm(a.length, a.joints, a.twist)
    k = Kin.from_arm(arm)
    half = a.range / 2
    print(f"arm: {k.J} revolute joints, pitch {k.pitch:.2f} mm, length {k.length:.0f} mm, twist {arm.twist:g} deg/joint "
          f"(first-to-last axis {(k.J-1)*arm.twist:g} deg)\n")

    # ---- per joint
    sel = parse_joint_arg(a.joint, k.J)
    rows = []
    for j in sel:
        ax = axis_dir(j, arm.twist)
        prev = axis_dir(j - 1, arm.twist) if j else None
        ang = "-" if prev is None else f"{math.degrees(math.acos(np.clip(prev @ ax, -1, 1))):.0f} deg"
        par = [i for i in range(k.J) if i != j and abs(abs(axis_dir(i, arm.twist) @ ax) - 1) < 1e-9]
        rows.append([j, "revolute (1)", f"({ax[0]:+.3f}, {ax[1]:+.3f}, 0)", f"{(j*arm.twist)%360:g} deg", ang,
                     ",".join(map(str, par[:4])) + ("..." if len(par) > 4 else "") or "-"])
    print(table(rows, ["joint", "type (DOF)", "axis in arm frame", "azimuth", "vs previous axis", "parallel joints"]))
    print("\nA pin hinge has exactly 1 rotational DOF; parallel joints are redundant only while the arm in between is straight.")

    # ---- whole arm
    rng = np.random.default_rng(a.seed)
    print("\nTip Jacobian (6 x J) rank by pose:")
    prow = []
    for nm in ("straight", "curl", "wave", "zigzag", "coil"):
        q = named_pose(k, nm, half * 0.9)
        r, s, c = k.dof(q)
        prow.append([nm, r, k.J - r, f"{s[0]:.0f}", f"{s[min(5, len(s)-1)]:.2f}", f"{c:.1e}"])
    print(table(prow, ["pose", "rank", "redundancy J-rank", "s_max", "s_6", "cond"]))
    ranks, conds = [], []
    for _ in range(a.samples):
        q = np.radians(rng.uniform(-half, half, k.J))
        r, s, c = k.dof(q); ranks.append(r); conds.append(c)
    ranks, conds = np.array(ranks), np.array(conds)
    print(f"\nrandom poses within +-{half:g} deg (n={a.samples}): rank min {ranks.min()} / max {ranks.max()}; "
          f"median cond {np.median(conds):.1e}; rank-deficient: {(ranks < 6).sum()}")
    pos_rank = np.linalg.matrix_rank(k.jacobian(named_pose(k, 'wave', half * 0.9))[:3], tol=1e-6)
    print(f"position-only (3 x J) rank at the wave pose: {pos_rank}")
    if a.effective:
        from joint_range import limit
        from common import build_bodies
        bodies = build_bodies(arm)
        free = []
        for j in range(k.J):
            hi, _ = limit(arm, bodies, j, +1, 5.0, 90.0, [0.0] * k.J); lo, _ = limit(arm, bodies, j, -1, 5.0, 90.0, [0.0] * k.J)
            if min(hi, lo) >= a.min_travel: free.append(j)
        print(f"\neffective joints (>= {a.min_travel:g} deg each way before contact): {len(free)}/{k.J}")
    sys.exit(0 if ranks.max() == 6 else 1)


if __name__ == "__main__":
    main()
