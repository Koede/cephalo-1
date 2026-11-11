#!/usr/bin/env python3
"""Compare arm variants (length / joints / twist / root servos) side by side, geometry only (no files written).

  python3 tools/variant_compare.py                                   # 585, 900, 1000 (13 joints), twist 90, root servos 0/3/5
  python3 tools/variant_compare.py --variant 1000:13:60 --variant 1000:11:60 --variant 900:13:90
Variant syntax LENGTH:JOINTS:TWIST (joints may be 0 = as many as fit). For each: pitch vs minimum, OD, static
interference, joint 0 and 1 travel (5 deg steps), joint pair clearance, and whether it passes. Plain PinArm
(non-geared) is used so every variant builds; the geared root only exists for the main design.
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import table, add_paths
add_paths()
import pin_cad as C

DEFAULT = ["585:0:60", "700:0:60", "900:13:60", "1000:13:60", "1000:11:60", "1000:13:90", "1000:13:45"]


def evaluate(L, J, tw):
    try:
        arm = C.PinArm(L, tw, J or None)
    except SystemExit as e:
        return None, str(e)
    b = [arm.body(i) for i in range(arm.J + 1)]
    static = max(float((b[i] ^ b[i + 1]).volume()) for i in range(arm.J))
    sw = {j: arm.joint_sweep(j, b) for j in ((0, 1) if arm.J > 2 else (0,))}
    pairs = {j: arm.pair_sweep(j, b) for j in (0, 1) if arm.J >= 3}
    return dict(arm=arm, static=static, sw=sw, pairs=pairs), ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", action="append")
    a = ap.parse_args(); rows = []; bad = 0
    for v in a.variant or DEFAULT:
        L, J, tw = v.split(":"); r, why = evaluate(float(L), int(J), float(tw))
        if r is None: rows.append([v, "-", "-", "-", "-", "-", "-", "infeasible: " + why[:60]]); bad += 1; continue
        arm = r["arm"]; s0, s1 = r["sw"][0], r["sw"].get(1, (0, 0)); ok = r["static"] < 0.5 and s0[1] >= 45 and -s0[0] >= 45 and all(not p for p in r["pairs"].values())
        rows.append([v, arm.J, f"{arm.pitch:.1f}/{arm.pitch_min:.1f}", f"{2*arm.R:.1f}", f"{r['static']:.1e}", f"{s0[0]:g}..{s0[1]:g}", f"{s1[0]:g}..{s1[1]:g}", "ok" if ok else "FAILS a gate"])
        bad += 0 if ok else 1
    print(table(rows, ["L:J:twist", "joints", "pitch/min", "OD", "static mm^3", "joint 0", "joint 1", "verdict"]))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
