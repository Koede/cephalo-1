#!/usr/bin/env python3
"""Bill of materials for one arm and for 8 arms, from the CAD reports (design intent, not a purchase order).

  python3 tools/bom_export.py                       # prints markdown, writes cephalo_bom.csv in the scratch dir
  python3 tools/bom_export.py --csv bom.csv --arms 8
Servos and hardware from pin_cad.py, geared-root parts from root_gear.json, printed parts counted from the
STL file names (xN), sub-assemblies (horn bearing x4, fan pod, palm finger) from their own folders. Quantities
come from the CAD notes: confirm against the build before ordering. Unit masses of bearings/hardware are [A].
"""
import argparse, csv, glob, os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import MAIN_OUT, SCRATCH, load_json, table, add_paths
add_paths()
import pin_cad as C


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", nargs="?", default=MAIN_OUT); ap.add_argument("--arms", type=int, default=8); ap.add_argument("--csv", default=None)
    a = ap.parse_args()
    f = load_json(os.path.join(a.dir, "fit.json")); g = load_json(os.path.join(a.dir, "root_gear.json")); J, nr = f["joints"], f["root_servos"]
    N = a.arms; rows = []
    def add(cat, item, spec, per_arm, unit_g, src): rows.append([cat, item, spec, per_arm, per_arm * N, round(unit_g, 1), round(per_arm * unit_g * N), src])
    add("servo", C.SERVO_ROOT["name"], "bus servo, 4.9 N m stall", nr, C.SERVO_ROOT["mass"], "pin_cad.py")
    add("servo", C.SERVO["name"], "bus servo, 2.9 N m", J - nr, C.SERVO["mass"], "pin_cad.py")
    add("bearing", g["gear_bearing"], "root gear side", 1, g["gear_bearing_g"], "root_gear.json")
    add("bearing", g["pinion_bearing"], "pinion side", 1, g["pinion_bearing_g"], "root_gear.json")
    hb = os.path.join(a.dir, "horn_bearing", "horn_bearing.json")
    if os.path.exists(hb):
        h = load_json(hb); add("bearing", h["bearing"]["name"], "horn support, joints 1-4", len(h["joints"]), h["bearing"]["g"], "horn_bearing.json")
    add("machined 7075", "gear 40T m1", f"face 10 mm", 1, g["gear_g"], "root_gear.json"); add("machined 7075", "pinion 20T m1", "", 1, g["pinion_g"], "root_gear.json")
    add("machined 7075", "idler pin 8 mm", "", 1, g["idler_pin_g"], "root_gear.json")
    add("hardware", "M3 screw + insert + PTFE set", "per joint: 4 horn screws, idler screw+washer, 6 inserts, PTFE washer", J, C.HW_G, "pin_cad.py HW_G [A]")
    add("hardware", "extra root hardware", "", 1, g["hw_extra_g"], "root_gear.json [A]")
    for sub in ("", "horn_bearing", "fan_pod"):
        for p in sorted(glob.glob(os.path.join(a.dir, sub, "stl", "*.stl"))):
            m = re.match(r"(.*)_x(\d+)\.stl$", os.path.basename(p))
            if not m: continue
            name, n = m.group(1), int(m.group(2))
            if "7075" in name: continue
            per_robot = n if sub == "fan_pod" else None          # fan pod parts are listed per robot (x8)
            per_arm = n / N if per_robot else n
            rows.append(["printed PETG", name.replace("_", " "), (sub or "main") + " (xN from the file name)", per_arm if per_robot else n, n * (1 if per_robot else N), "-", "-", os.path.relpath(p, a.dir)])
    print(f"BOM for {N} arms ({a.dir})\n"); print(table(rows, ["category", "item", "spec", "qty/arm", f"qty x{N}", "unit g", "total g", "source"]))
    out = a.csv or os.path.join(SCRATCH, "cephalo_bom.csv"); os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["category", "item", "spec", "qty_per_arm", f"qty_x{N}", "unit_g", "total_g", "source", "unit_price"]); [w.writerow(r + [""]) for r in rows]
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
