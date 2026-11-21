#!/usr/bin/env python3
"""Root gear stage check (module-1 spur pair on joint 0): geometry, contact ratio, Lewis bending.

  python3 tools/gear_check.py                       # torque = STS3250 stall (4.9 N m) at the pinion
  python3 tools/gear_check.py --torque 3.0 --allow 150
Reads the constants from arm-pin-hinge/root_gear.py (MOD, PA, Z_PIN, Z_GEAR, FACE, CDIST, SECTOR_DEG).
Checks: centre distance, tip/root radii, root circle above/below the base circle, contact ratio >= 1.2,
undercut limit (z >= 17 at 20 deg), Lewis bending stress at the pinion and gear vs --allow MPa
(7075-T6 fatigue-ish 150 MPa [A]), sector range covers +-need deg of joint travel. Estimates, not FEA.
"""
import argparse, math, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Checks, add_paths
add_paths()
import root_gear as R


def lewis_y(z):
    """Lewis form factor (20 deg full depth), interpolated from the standard table"""
    tab = {12: .245, 13: .261, 14: .276, 15: .289, 16: .295, 17: .302, 18: .308, 19: .314, 20: .320, 21: .326, 22: .330, 24: .337, 26: .346,
           28: .353, 30: .359, 34: .371, 38: .384, 43: .397, 50: .409, 60: .422, 75: .435, 100: .447, 150: .460}
    ks = sorted(tab)
    for lo, hi in zip(ks, ks[1:]):
        if lo <= z <= hi: return tab[lo] + (tab[hi] - tab[lo]) * (z - lo) / (hi - lo)
    return tab[ks[0]] if z < ks[0] else tab[ks[-1]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--torque", type=float, default=4.9, help="servo torque at the pinion, N m")
    ap.add_argument("--allow", type=float, default=150.0, help="allowable bending stress, MPa [A]")
    ap.add_argument("--need", type=float, default=45.0)
    a = ap.parse_args()
    m, pa, z1, z2, b = R.MOD, R.PA, R.Z_PIN, R.Z_GEAR, R.FACE
    C = Checks(f"root gear: {z1}T pinion / {z2}T gear, module {m:g}, {pa:g} deg, face {b:g} mm")
    rp1, rp2 = m * z1 / 2, m * z2 / 2; rb1, rb2 = rp1 * math.cos(math.radians(pa)), rp2 * math.cos(math.radians(pa))
    ra1, ra2 = rp1 + m, rp2 + m
    C.add("centre distance = (z1+z2)/2 * m", abs(R.CDIST - (rp1 + rp2)) < 1e-6, f"{R.CDIST:g} mm")
    C.add("ratio", abs(z2 / z1 - 2) < 1e-9, f"{z2/z1:g}:1 (joint torque = {z2/z1:g} x servo torque, speed / {z2/z1:g})")
    C.add("no undercut (z >= 17 at 20 deg)", min(z1, z2) >= 17, f"z1 = {z1}")
    path = math.sqrt(ra1**2 - rb1**2) + math.sqrt(ra2**2 - rb2**2) - R.CDIST * math.sin(math.radians(pa))
    cr = path / (math.pi * m * math.cos(math.radians(pa)))
    C.add("contact ratio >= 1.2", cr >= 1.2, f"{cr:.2f}")
    C.add("root circle vs base circle (pinion)", True, f"root {rp1-1.25*m:.2f} mm, base {rb1:.2f} mm (flank continues below base by a fillet)")
    Ft = a.torque / (rp1 / 1000)                      # N at the pinion pitch circle
    s1 = Ft / (b * m * lewis_y(z1)); s2 = Ft / (b * m * lewis_y(z2))
    C.add(f"pinion Lewis stress <= {a.allow:g} MPa", s1 <= a.allow, f"{s1:.0f} MPa, SF {a.allow/s1:.2f}; Ft {Ft:.0f} N (torque {a.torque:g} N m; stall is the worst case, allowable is an assumption)", warn=True)
    C.add(f"gear Lewis stress <= {a.allow:g} MPa", s2 <= a.allow, f"{s2:.0f} MPa, SF {a.allow/s2:.2f}", warn=True)
    sector = getattr(R, "SECTOR_DEG", None)
    if sector:   # teeth over +-SECTOR_DEG of gear angle; joint angle = gear angle (the gear is on the joint)
        C.add(f"sector covers +-{a.need:g} deg of joint travel", sector >= a.need, f"teeth over +-{sector:g} deg")
    print("Bearing: gear side", R.GB["name"], f"C0 {R.GB['C0']} N, pinion side", R.PB["name"], f"C0 {R.PB['C0']} N (loads: tools/study_report.py bearings)")
    C.exit()


if __name__ == "__main__":
    main()
