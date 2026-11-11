#!/usr/bin/env python3
"""Sweep one design parameter and show which values pass the geometric gates (geometry only).

  python3 tools/param_sweep.py joints 9 14                 # LENGTH 1000, twist 60, joints 9..14
  python3 tools/param_sweep.py length 500 1100 --step 100 --joints 0
  python3 tools/param_sweep.py twist 30 120 --step 15
  python3 tools/param_sweep.py range 60 120 --step 10      # command range: reports the joint-0 margin vs range/2
Gates per value: arm builds (pitch >= minimum), static interference < 0.5 mm^3, joint 0 and 1 cover +-range/2,
pairs 0+1, 1+2 clear to +-60. Reports pitch, OD, tip reach (straight) and which gate fails first.
"""
import argparse, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import table, add_paths
add_paths()
import pin_cad as C


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("param", choices=["joints", "length", "twist", "range"]); ap.add_argument("lo", type=float); ap.add_argument("hi", type=float)
    ap.add_argument("--step", type=float, default=None)
    ap.add_argument("--length", type=float, default=1000.0); ap.add_argument("--joints", type=int, default=13); ap.add_argument("--twist", type=float, default=60.0); ap.add_argument("--range", type=float, default=90.0)
    a = ap.parse_args(); step = a.step or (1.0 if a.param == "joints" else (a.hi - a.lo) / 6)
    rows, bad = [], 0
    for v in np.arange(a.lo, a.hi + 1e-9, step):
        L, J, tw, rg = a.length, a.joints, a.twist, a.range
        if a.param == "joints": J = int(round(v))
        elif a.param == "length": L = v
        elif a.param == "twist": tw = v
        else: rg = v
        try: arm = C.PinArm(L, tw, J or None, rg)
        except SystemExit as e: rows.append([f"{v:g}", "-", "-", "-", "-", "infeasible: pitch below minimum"]); bad += 1; continue
        b = [arm.body(i) for i in range(arm.J + 1)]
        static = max(float((b[i] ^ b[i + 1]).volume()) for i in range(arm.J))
        s0, s1 = arm.joint_sweep(0, b), arm.joint_sweep(1, b); half = rg / 2
        pairs = [arm.pair_sweep(j, b) for j in (0, 1) if arm.J >= 3]
        fails = [n for n, c in (("static", static < 0.5), ("joint 0 travel", s0[0] <= -half and s0[1] >= half), ("joint 1 travel", s1[0] <= -half and s1[1] >= half), ("pair clearance", all(not p for p in pairs))) if not c]
        rows.append([f"{v:g}", arm.J, f"{arm.pitch:.1f}", f"{2*arm.R:.1f}", f"{s0[0]:g}..{s0[1]:g}", "ok" if not fails else "fails: " + ", ".join(fails)]); bad += bool(fails)
    print(f"sweep {a.param} (others: length {a.length:g}, joints {a.joints}, twist {a.twist:g}, range {a.range:g})")
    print(table(rows, [a.param, "joints", "pitch", "OD", "joint 0 travel", "verdict"]))
    sys.exit(1 if bad == len(rows) else 0)


if __name__ == "__main__":
    main()
