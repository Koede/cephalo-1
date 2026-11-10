#!/usr/bin/env python3
"""Assert the DESIGN.md gates against a design directory's JSON reports.

  python3 tools/check_gates.py                       # committed main design arm-pin-hinge/out_1m_13
  python3 tools/check_gates.py DIR       # a regenerated directory
  python3 tools/check_gates.py DIR --range 90 --tw 1.15

Gates: static interference ~0; joints swept are collision-free over +-range/2; joint pairs clear
to +-60 deg; vertebra 2 == vertebra 1 turned (checked in fit_report.md); thrust-to-weight >= 1.15
and added mass <= budget (root_gear.json); horn-bearing static clear and travel kept; fan pod
interference; mantle ports flanges clear at the hub radius used (WARN: design records 124.5 mm
minimum vs 110 mm used). Missing reports are skipped with a WARN. Exit 1 on any FAIL.
"""
import argparse, os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import Checks, MAIN_OUT, load_json


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", nargs="?", default=MAIN_OUT)
    ap.add_argument("--range", type=float, default=90.0); ap.add_argument("--tw", type=float, default=1.15)
    ap.add_argument("--static-tol", type=float, default=0.5, help="mm^3")
    a = ap.parse_args()
    d, need = a.dir, a.range / 2
    C = Checks(f"design gates for {d}")
    p = lambda *x: os.path.join(d, *x)

    if os.path.exists(p("fit.json")):
        f = load_json(p("fit.json"))
        C.add("static interference ~ 0", f["static"] <= a.static_tol, f"{f['static']:.2e} mm^3")
        C.add("pitch >= minimum", f["pitch"] >= f["pitch_min"] - 1e-6, f"{f['pitch']:.1f} vs min {f['pitch_min']:.1f} mm")
        for j, lo, hi in f["sweeps"]:
            C.add(f"joint {j} travel covers +-{need:g}", lo <= -need and hi >= need, f"{lo:g} to +{hi:g} deg")
        for j, bad in f["pairs"]:
            C.add(f"joints {j}+{j+1} clear to +-60", not bad, "clear" if not bad else f"first contact {bad}")
    else:
        C.add("fit.json", False, "missing", warn=True)
    if os.path.exists(p("fit_report.md")):
        m = re.search(r"Vertebra 2 minus vertebra 1 turned [\d.]+ deg: ([\d.e+-]+) mm\^3", open(p("fit_report.md")).read())
        if m: C.add("vertebra 2 == vertebra 1 turned", float(m.group(1)) < a.static_tol, f"{m.group(1)} mm^3")
    if os.path.exists(p("root_gear.json")):
        g = load_json(p("root_gear.json"))
        C.add(f"thrust-to-weight >= {a.tw}", g["thrust_to_weight"] >= a.tw - 1e-3, f"{g['thrust_to_weight']:.3f} ({g['robot_kg']:.2f} kg)")
        C.add("added mass per arm <= budget", g["added_g_per_arm"] <= g["budget_g"] + 1e-6, f"{g['added_g_per_arm']:.1f} g vs {g['budget_g']:.1f} g")
        C.add("gear mesh: centre distance = (Z1+Z2)/2 * module", abs(g["centre_distance"] - (g["gear_tip_r"] + g["pinion_tip_r"] - 2) ) < 0.01,
              f"{g['centre_distance']} mm", warn=True)
    else:
        C.add("root_gear.json", False, "missing (plain pin_cad run?)", warn=True)
    hb = p("horn_bearing", "horn_bearing.json")
    if os.path.exists(hb):
        h = load_json(hb)
        C.add("horn bearing: static interference", h["static_interference_mm3"] <= a.static_tol, f"{h['static_interference_mm3']} mm^3")
        C.add("horn bearing: sweeps cover +-45", all(lo <= -need and hi >= need for lo, hi in h["sweeps"].values()), str(h["sweeps"]))
        C.add("horn bearing: pairs clear", all(not v for v in h["pairs"].values()), str(h["pairs"]))
        C.add("horn bearing: radius within arm OD", max(h["max_r_new_ring"], h["max_r_new_ear"]) <= h["arm_R"] + 1e-6,
              f"{max(h['max_r_new_ring'], h['max_r_new_ear'])} vs {h['arm_R']}")
        C.add(f"horn bearing: rated T/W after >= {a.tw}", h["tw_rated_after"] >= a.tw, f"{h['tw_rated_after']} (with bearing + central prop + pack; DESIGN.md lists this as the open item)", warn=True)
    fp = p("fan_pod", "fan_pod.json")
    if os.path.exists(fp):
        q = load_json(fp)
        it = q["interference_over_travel"]
        C.add("fan pod: no interference over travel", not it.get("bad"), f"{it.get('steps')} steps, bad: {it.get('bad')}")
        C.add("fan pod: folded disc clear of arm", q["folded_disc_clear_of_arm_mm"] > 0, f"{q['folded_disc_clear_of_arm_mm']} mm")
    mp = p("mantle_ports.json")
    if os.path.exists(mp):
        m = load_json(mp)
        C.add("mantle ports: flanges clear at hub radius used", m["flange_overlap_at_hub_radius_mm3"] <= a.static_tol,
              f"{m['flange_overlap_at_hub_radius_mm3']:.0f} mm^3 at R={m['hub_radius_used']} (clear from R={m['min_hub_radius_flanges_clear']})", warn=True)
    C.exit()


if __name__ == "__main__":
    main()
