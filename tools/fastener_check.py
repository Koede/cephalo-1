#!/usr/bin/env python3
"""Fastener / insert geometry check from pin_cad.py parameters (design intent, not FEA).

  python3 tools/fastener_check.py
  python3 tools/fastener_check.py --min-wall 1.6 --min-edge 1.5
Checks M3 horn bolt circle edge distance, heat-set insert hole size and the wall round it (idler stub
and lid bosses), insert depth vs part thickness, screw-head stack inside the arm radius, horn disc.
Parameters marked [A] in pin_cad.py are assumptions until measured; every line says which it used.
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Checks, add_paths, build_arm
add_paths()
import pin_cad as P


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--min-wall", type=float, default=1.6, help="min plastic wall around an insert, mm")
    ap.add_argument("--min-edge", type=float, default=1.5, help="min bolt-hole edge to part edge, mm")
    ap.add_argument("--insert-len", type=float, default=5.7, help="M3 heat-set insert length [A]")
    ap.add_argument("--bolt-hole-r", type=float, default=1.7, help="M3 clearance hole radius")
    a = ap.parse_args()
    arm = build_arm(geared=False)
    C = Checks("fasteners and inserts (pin_cad.py parameters)")
    C.add("horn bolt hole edge to horn rim", P.HORN_R - P.HORN_PCD / 2 - a.bolt_hole_r >= a.min_edge,
          f"{P.HORN_R - P.HORN_PCD/2 - a.bolt_hole_r:.2f} mm (HORN_R {P.HORN_R:g} [A], PCD {P.HORN_PCD:g} [A], M3 clearance r {a.bolt_hole_r:g})", warn=True)
    C.add("horn bolt circle inside the horn ear boss", P.HORN_PCD / 2 + a.bolt_hole_r < P.BOSS_R, f"{P.HORN_PCD/2 + a.bolt_hole_r:.1f} mm < boss r {P.BOSS_R:g}")
    C.add("insert hole diameter 3.9-4.3 mm for a 4.5 mm M3 insert", 3.9 <= 2 * P.INSERT_R <= 4.3, f"{2*P.INSERT_R:g} mm [A]")
    C.add("wall round the idler stub insert", P.STUB_R - P.INSERT_R >= a.min_wall, f"{P.STUB_R - P.INSERT_R:.2f} mm (STUB_R {P.STUB_R:g} - INSERT_R {P.INSERT_R:g})")
    C.add("wall round a lid-screw insert", P.LID_BOSS_R - P.INSERT_R >= a.min_wall, f"{P.LID_BOSS_R - P.INSERT_R:.2f} mm (LID_BOSS_R {P.LID_BOSS_R:g}); lower than the minimum means a thicker boss or smaller insert", warn=True)
    C.add("insert fits the idler stub depth", a.insert_len + 1.0 <= P.FLOOR_T + P.STUB_R * 3, f"insert {a.insert_len:g} mm [A] vs stub/floor depth ~{P.FLOOR_T + P.STUB_R*3:.1f} mm", warn=True)
    C.add("lid thick enough for an insert-free clamp", P.LID_T >= 1.8, f"LID_T {P.LID_T:g} mm (screws go into bosses on the cradle sides, not the lid)")
    C.add("screw head stack inside the arm radius", P.HEAD_H + P.IDLER_EAR_T + P.EAR_GAP + P.FLOOR_T < arm.R * 2, f"head {P.HEAD_H:g} + idler ear {P.IDLER_EAR_T:g} + gap {P.EAR_GAP:g} + floor {P.FLOOR_T:g} within OD {2*arm.R:.1f} (arm.head_lo {arm.head_lo:.1f}..)")
    C.add("horn ear thick enough for 1.8 mm counterbore heads", P.HORN_EAR_T >= 1.8 + 1.5, f"HORN_EAR_T {P.HORN_EAR_T:g} mm (leaves {P.HORN_EAR_T-1.8:.1f} mm under the head)")
    C.exit()


if __name__ == "__main__":
    main()
