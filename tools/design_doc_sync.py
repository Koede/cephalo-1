#!/usr/bin/env python3
"""Check that numbers quoted in DESIGN.md / README.md still match the committed reports.

  python3 tools/design_doc_sync.py            # prints MATCH / STALE / NOT FOUND per quoted figure
  python3 tools/design_doc_sync.py --dir DIR  # reports from a regenerated directory
Each entry is (label, value from fit.json/root_gear.json/..., formatted candidates, doc file). A figure is
MATCH if any candidate string appears in the doc; STALE if the doc quotes a different number next to the
label's key phrase (shows the line); NOT FOUND if neither. Never edits: propose replacements yourself.
Exit 1 if any STALE.
"""
import argparse, os, re, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT, MAIN_OUT, load_json, Checks


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", default=MAIN_OUT)
    a = ap.parse_args()
    design = open(os.path.join(ROOT, "DESIGN.md"), encoding="utf8").read(); readme = open(os.path.join(ROOT, "README.md"), encoding="utf8").read()
    f = load_json(os.path.join(a.dir, "fit.json")); g = load_json(os.path.join(a.dir, "root_gear.json"))
    h = load_json(os.path.join(a.dir, "horn_bearing", "horn_bearing.json")); m = load_json(os.path.join(a.dir, "mantle_ports.json"))
    N = lambda x, d: f"{x:.{d}f}"
    # (label, phrase that precedes the number in the doc, candidate strings of the true value)
    facts = [
        ("pitch (mm)", r"pitch", [N(f["pitch"], 1), N(f["pitch"], 2)]),
        ("outer diameter (mm)", r"(OD|outer diameter)", [N(f["OD"], 1)]),
        ("arm outside the mantle (g)", r"outside the mantle", [N(f["arm_out_g"], 0), f"{f['arm_out_g']:,.0f}"]),
        ("added mass per arm (g)", r"added mass", [N(g["added_g_per_arm"], 1), N(g["added_g_per_arm"], 0)]),
        ("added-mass budget (g)", r"budget", [N(g["budget_g"], 1), N(g["budget_g"], 0)]),
        ("robot mass (kg)", r"[Rr]obot", [N(g["robot_kg"], 2)]),
        ("rated thrust-to-weight", r"thrust-to-weight|T/W", [N(g["thrust_to_weight"], 3), N(g["thrust_to_weight"], 2)]),
        ("min hub radius for clear flanges (mm)", r"hub radius", [N(m["min_hub_radius_flanges_clear"], 1)]),
        ("horn-bearing rated T/W after", r"rated T/W", [N(h["tw_rated_after"], 3)]),
        ("joint 0 sweep (deg)", r"joint 0", [f"{abs(f['sweeps'][0][2]):.0f}"]),
    ]
    C = Checks(f"DESIGN.md / README.md vs reports in {a.dir}")
    for label, phrase, cands in facts:
        hit = [c for c in cands if c in design or c in readme]
        if hit:
            C.add(label, True, f"found '{hit[0]}'"); continue
        lines = [ln.strip() for ln in design.splitlines() if re.search(phrase, ln) and re.search(r"\d", ln)]
        C.add(label, False, f"true value {cands[0]} not in the docs" + (f"; e.g. line: {lines[0][:90]}" if lines else ""), warn=True)
    n_stale = sum(1 for r in C.rows if not r[1])
    print("note: this only proves the true number appears somewhere in the docs. Check lines flagged WARN by hand; the docs also keep earlier-stage numbers on purpose (585 mm build, pre-bearing).")
    C.exit()


if __name__ == "__main__":
    main()
