#!/usr/bin/env python3
"""Mass breakdown of one arm and the robot from a design directory's reports.

  python3 tools/mass_budget.py                    # committed main design
  python3 tools/mass_budget.py DIR --add-g 5      # what if each arm gets 5 g more?
Components per arm outside the mantle: moving vertebrae, lids, hardware, servos, tip; then the
geared-root extras (socket, caps, gear, pinion, bearings) from root_gear.json. Checks that the parts
add up to fit.json's arm_out_g, shows the added-mass budget (root_gear.json) and the thrust-to-weight
effect: T/W = T/(m g), robot_kg = robot_kg0 + 8 * delta / 1000. Exit 1 if over budget.
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import MAIN_OUT, load_json, table, add_paths
add_paths()
import pin_cad as C


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dir", nargs="?", default=MAIN_OUT); ap.add_argument("--add-g", type=float, default=0.0, help="extra grams per arm")
    a = ap.parse_args()
    f = load_json(os.path.join(a.dir, "fit.json")); J, nroot = f["joints"], f["root_servos"]
    rows = []
    def add(name, n, each): rows.append([name, n, f"{each:.1f}", f"{n*each:.0f}"])
    add("moving vertebra (PETG)", J - 1, f["m_vert"]); add("servo lid (joints 1..)", J - 1, f["m_lid"]); add("tip vertebra", 1, f["m_tip"])
    add("hardware (screws, inserts, PTFE)", J, C.HW_G)
    n3250 = max(0, nroot - 1)                       # joint 0's servo sits inside the mantle
    add(f"{C.SERVO_ROOT['name']} servo (joints 1..{nroot-1})", n3250, C.SERVO_ROOT["mass"])
    add(f"{C.SERVO['name']} servo (joints {nroot}..{J-1})", J - nroot, C.SERVO["mass"])
    out_sum = sum(float(r[3]) for r in rows)
    print(f"arm outside the mantle ({a.dir})"); print(table(rows, ["part", "qty", "g each", "g"]))
    ok = abs(out_sum - f["arm_out_g"]) < 5
    print(f"sum {out_sum:.0f} g vs fit.json arm_out_g {f['arm_out_g']:.0f} g  {'(matches)' if ok else '(MISMATCH)'}")
    gp = os.path.join(a.dir, "root_gear.json")
    code = 0 if ok else 1
    if os.path.exists(gp):
        g = load_json(gp)
        ex = [["geared socket (PETG)", g["socket_g"]], ["end cap", g["end_cap_g"]], ["bearing cap", g["bearing_cap_g"]],
              ["40T gear (7075)", g["gear_g"]], ["20T pinion (7075)", g["pinion_g"]], [f"gear bearing {g['gear_bearing']}", g["gear_bearing_g"]],
              [f"pinion bearing {g['pinion_bearing']}", g["pinion_bearing_g"]], ["idler pin (7075)", g["idler_pin_g"]], ["extra hardware [A]", g["hw_extra_g"]]]
        print("\ngeared root (inside / at the mantle)"); print(table([[n, f"{v:.1f}"] for n, v in ex], ["part", "g"]))
        added = g["added_g_per_arm"] + a.add_g; budget = g["budget_g"]
        print(f"\nadded mass per arm vs the plain socket: {g['added_g_per_arm']:.1f} g" + (f" (+{a.add_g:g} what-if = {added:.1f} g)" if a.add_g else "")
              + f"; budget {budget:.1f} g (keeps T/W at 1.15); margin {budget - added:+.1f} g")
        robot0 = g["robot_kg"] - 8 * g["added_g_per_arm"] / 1000
        robot = robot0 + 8 * added / 1000; tw = (g["thrust_to_weight"] * g["robot_kg"]) / robot
        print(f"robot {robot:.2f} kg, rated thrust-to-weight {tw:.3f} (gate 1.15); one arm all-in {g['arm_all_g']:.0f} g; 8 arms {8*g['arm_all_g']/1000:.1f} kg")
        code = code or (1 if added > budget + 1e-9 or tw < 1.15 - 1e-3 else 0)
    print("masses: PETG 1.27 g/cm^3 x 0.95 fill, 7075 2.81 g/cm^3, servo/bearing masses from datasheets or [A]")
    sys.exit(code)


if __name__ == "__main__":
    main()
