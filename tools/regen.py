#!/usr/bin/env python3
"""Regenerate a design variant into a scratch directory, in the right order.

  python3 tools/regen.py                              # main design -> <tempdir>/cephalo/regen/main
  python3 tools/regen.py --out DIR --length 900 --joints 13
  python3 tools/regen.py --variant 585                # 585 mm, 9 joints, plain socket (arm-pin-hinge/out)
  python3 tools/regen.py --subassemblies              # + horn_bearing.py, fan_pod.py, mantle_notch.py

Variants: main (1000/13/geared root, 5 root servos), 585, 900, twist90 (spec-sheet cad_alt90 style).
Always runs from arm-pin-hinge/ (the modules import each other), into a fresh directory, so stale
files such as servo_lid_x13.stl (root_gear.py renames the lid) cannot survive. Never writes into
the committed out_* folders unless you pass one explicitly with --force.
"""
import argparse, os, shutil, subprocess, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import PIN_DIR, ROOT, SCRATCH

VARIANTS = {
    "main":    dict(args=["--length", "1000", "--joints", "13", "--root-servos", "5"], geared=True),
    "585":     dict(args=["--length", "585"], geared=False),
    "900":     dict(args=["--length", "900", "--joints", "13", "--root-servos", "5"], geared=True),
    "twist90": dict(args=["--length", "1000", "--joints", "13", "--root-servos", "5", "--twist", "90"], geared=False),
}


def run(cmd, cwd, env=None):
    t = time.time()
    r = subprocess.run(cmd, cwd=cwd, env=env, capture_output=True, text=True)
    print(f"  {' '.join(os.path.basename(c) if i == 1 else c for i, c in enumerate(cmd))}  [{time.time()-t:.1f}s, exit {r.returncode}]")
    if r.returncode:
        print(r.stdout[-1500:], r.stderr[-1500:]); sys.exit(r.returncode)
    return r


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--variant", default="main", choices=VARIANTS)
    ap.add_argument("--out", default=None); ap.add_argument("--force", action="store_true")
    ap.add_argument("--length"); ap.add_argument("--joints"); ap.add_argument("--twist"); ap.add_argument("--root-servos")
    ap.add_argument("--no-gear", action="store_true"); ap.add_argument("--no-sweep", action="store_true")
    ap.add_argument("--subassemblies", action="store_true")
    a = ap.parse_args()
    v = VARIANTS[a.variant]
    args = list(v["args"])
    for flag in ("length", "joints", "twist", "root_servos"):
        val = getattr(a, flag)
        if val is not None:
            k = "--" + flag.replace("_", "-")
            if k in args: args[args.index(k) + 1] = val
            else: args += [k, val]
    out = os.path.abspath(a.out or os.path.join(SCRATCH, "regen", a.variant))
    if os.path.exists(out):
        if os.path.relpath(out, ROOT).replace(os.sep, "/").startswith("arm-pin-hinge/out") and not a.force:
            sys.exit(f"refusing to overwrite committed {out}; use a scratch --out or --force")
        shutil.rmtree(out)
    args += ["--out", out] + (["--no-sweep"] if a.no_sweep else [])
    geared = v["geared"] and not a.no_gear
    print(f"regenerating {a.variant} into {out}")
    run([sys.executable, "root_gear.py" if geared else "pin_cad.py"] + args, PIN_DIR)
    if a.subassemblies:
        if not geared: sys.exit("subassemblies need the geared main design")
        # these scripts write into out_1m_13 next to the script: point them at our dir via cwd copy
        for s in ("horn_bearing.py", "fan_pod.py"):
            print(f"  note: {s} writes to arm-pin-hinge/out_1m_13/ (hard-coded); run it there deliberately")
        run([sys.executable, "mantle_notch.py", "--length", "1000", "--joints", "13", "--out", out], PIN_DIR)
    print(f"done: {out}")


if __name__ == "__main__":
    main()
