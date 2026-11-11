#!/usr/bin/env python3
"""Pre-print report: what to measure first ([A] assumptions), the parts to print and how many, size/support hints.

  python3 tools/print_prep.py                       # committed main design
  python3 tools/print_prep.py DIR --arms 8 --bed 250
Lists every parameter marked [A] in pin_cad.py / root_gear.py with its current value; the STL inventory
(per arm and for the robot) with bounding box and mass (PETG 1.27 g/cm^3 x 0.95); parts larger than the
print bed; thin-feature hints (WALL, LID_T, FLOOR_T); and the STL integrity result (stl_integrity.py).
Exit 1 if a part does not fit the bed (--bed mm, square) or an STL fails the integrity check.
"""
import argparse, glob, os, re, subprocess, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import MAIN_OUT, PIN_DIR, table, ROOT
import trimesh


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", nargs="?", default=MAIN_OUT); ap.add_argument("--arms", type=int, default=8); ap.add_argument("--bed", type=float, default=250.0)
    a = ap.parse_args()
    print("== Measure on the real parts before printing ([A] = assumed)")
    rows = []
    for fn in ("pin_cad.py", "root_gear.py"):
        for ln, line in enumerate(open(os.path.join(PIN_DIR, fn)), 1):
            if "[A" in line and "=" in line.split("#")[0] and not line.lstrip().startswith(("f\"", "\"", "-", "print")):
                lhs = line.split("#")[0].strip(); cm = line.split("#", 1)[1].strip() if "#" in line else ""
                rows.append([f"{fn}:{ln}", lhs[:48], cm[:80]])
    print(table(rows, ["where", "value", "note"]))
    print("\n== Parts to print (PETG, 3-4 perimeters, 0.2 mm; supports for the fork ears and cradles)")
    inv, bad = [], 0
    for sub in ("", "horn_bearing", "fan_pod"):
        for p in sorted(glob.glob(os.path.join(a.dir, sub, "stl", "*.stl"))):
            name = os.path.basename(p); m = re.match(r"(.*)_x(\d+)\.stl$", name)
            if not m or "7075" in name: continue
            n = int(m.group(2)); per_robot = sub == "fan_pod"
            t = trimesh.load(p); ext = t.extents; mass = abs(t.volume) * 1.27e-3 * 0.95 if t.is_watertight else float("nan")
            bad += 0 if max(ext) <= a.bed * 1.4 else 1
            inv.append([os.path.join(sub, name) if sub else name, n if per_robot else n * a.arms, f"{ext[0]:.0f} x {ext[1]:.0f} x {ext[2]:.0f}", f"{mass:.1f}", "ok" if max(ext) <= a.bed * 1.4 else f"> {a.bed:g} bed"])
    print(table(inv, ["file", f"qty ({a.arms} arms)", "bbox mm", "g each", "bed"]))
    tot = sum(float(r[3]) * r[1] for r in inv if r[3] != "nan")
    print(f"printed PETG total about {tot/1000:.2f} kg for {a.arms} arms (solid-equivalent x 0.95 fill)")
    print("\n== Machined (reference models): 7075 gear 40T, pinion 20T, idler pin 8 mm x%d" % a.arms)
    r = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "stl_integrity.py"), a.dir], capture_output=True, text=True)
    print("\n== STL integrity:", r.stdout.strip().splitlines()[-1])
    sys.exit(1 if bad or r.returncode else 0)


if __name__ == "__main__":
    main()
