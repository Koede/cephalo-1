#!/usr/bin/env python3
"""Sensitivity of the fit to print/servo tolerances: perturb one pin_cad parameter and re-check.

  python3 tools/tolerance_stackup.py                           # default list, +-0.3 mm
  python3 tools/tolerance_stackup.py --param CLEAR --delta 0.3
  python3 tools/tolerance_stackup.py --param HORN_R --delta 0.5 --joint 0
For each parameter p in {CLEAR, WALL, HORN_T, HORN_R, BOSS_R, EAR_GAP, ...} sets p -> p +- delta
(servo dims via SERVO['L'|'W'|'H']), rebuilds the arm (geometry only, no files) and reports static
interference, joint 0/1 travel and pitch_min. A parameter whose change drops travel below 45 deg or
makes static interference > 0.5 mm^3 is a tolerance the design is sensitive to.
"""
import argparse, copy, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import add_paths, table
add_paths()
import pin_cad as C, root_gear as R

DEFAULT = ["CLEAR", "WALL", "HORN_T", "HORN_R", "BOSS_R", "EAR_GAP", "HORN_EAR_T", "IDLER_EAR_T", "STUB_R", "SERVO.L", "SERVO.W", "SERVO.H"]


def evaluate(joint):
    arm = R.GearedRootArm(1000, 60, 13)
    b = [arm.body(i) for i in range(arm.J + 1)]
    static = max(float((b[i] ^ b[i + 1]).volume()) for i in range(arm.J))
    lo, hi = arm.joint_sweep(joint, b, step=5.0)
    return static, lo, hi, arm.pitch_min, 2 * arm.R


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--param", action="append"); ap.add_argument("--delta", type=float, default=0.3); ap.add_argument("--joint", type=int, default=0)
    a = ap.parse_args()
    rows = []; base = evaluate(a.joint)
    rows.append(["baseline", "-", f"{base[0]:.1e}", f"{base[1]:g}..{base[2]:g}", f"{base[3]:.1f}", f"{base[4]:.1f}", "ok"])
    for p in (a.param or DEFAULT):
        for sgn in (-1, 1):
            saved = copy.deepcopy((C.SERVO, C.SERVO_ROOT)); name = p
            try:
                if p.startswith("SERVO."):
                    key = p.split(".")[1]; C.SERVO[key] += sgn * a.delta; C.SERVO_ROOT[key] += sgn * a.delta
                else:
                    setattr(C, p, getattr(C, p) + sgn * a.delta); setattr(R.C if hasattr(R, "C") else C, p, getattr(C, p))
                r = evaluate(a.joint); ok = r[0] <= 0.5 and r[1] <= -45 and r[2] >= 45
                rows.append([name, f"{sgn*a.delta:+g}", f"{r[0]:.1e}", f"{r[1]:g}..{r[2]:g}", f"{r[3]:.1f}", f"{r[4]:.1f}", "ok" if ok else "SENSITIVE"])
            except AssertionError as e:
                rows.append([name, f"{sgn*a.delta:+g}", "-", "-", "-", "-", "ASSERT: a built-in fit assertion trips (design is locked to this value)"])
            except Exception as e:
                rows.append([name, f"{sgn*a.delta:+g}", "-", "-", "-", "-", f"error: {str(e)[:40]}"])
            finally:
                C.SERVO.clear(); C.SERVO.update(saved[0]); C.SERVO_ROOT.clear(); C.SERVO_ROOT.update(saved[1])
                if not p.startswith("SERVO."): setattr(C, p, getattr(C, p) - sgn * a.delta)
    print(table(rows, ["parameter", "delta mm", "static mm^3", f"joint {a.joint} travel", "pitch_min", "OD", "verdict"]))
    sys.exit(1 if any(r[-1] == "SENSITIVE" for r in rows) else 0)


if __name__ == "__main__":
    main()
