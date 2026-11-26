#!/usr/bin/env python3
"""Read the committed joint-study results (joint-study/results) and judge them. Nothing is re-run.

  python3 tools/study_report.py bearings     # root/pin bearing and idler loads vs static safety factor
  python3 tools/study_report.py joints       # joints 1-4 stresses vs PETG allowables (fork ears, stub, bore)
  python3 tools/study_report.py battery      # packs: hover minutes, current vs pack rating, T/W
  python3 tools/study_report.py thrust       # thrust-to-weight by pack and by added mass
  python3 tools/study_report.py prop         # claw prop clearance by claw height
  python3 tools/study_report.py fluid        # arm drag/length scaling (fluid_len.csv)
  python3 tools/study_report.py stance       # walking stance feasibility on the folded pod
  python3 tools/study_report.py servos       # servo table (servos.py) with the CAD case and a fit flag
  python3 tools/study_report.py all
--results DIR points at another results folder. Exit 1 if any FAIL (WARN = open item recorded in DESIGN.md).
"""
import argparse, csv, json, os, sys, re
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT, Checks, table, load_json

RES = os.path.join(ROOT, "joint-study", "results")


def bearings(R, C):
    d = load_json(os.path.join(R, "bearing_loads.json"))["after the bearing check"]
    for case, r in d["rating"].items():
        C.add(f"bearing: {case}: gear brg s0", r["gear_brg_s0"] >= 1.0, f"{r['gear_brg_N']} N, s0 {r['gear_brg_s0']} ({d['design']['gear_brg'][0]})", warn=r["gear_brg_s0"] < 1.5)
        C.add(f"bearing: {case}: pinion brg s0", r["pin_brg_s0"] >= 1.0, f"{r['pin_brg_N']} N, s0 {r['pin_brg_s0']} ({d['design']['pin_brg'][0]})", warn=r["pin_brg_s0"] < 1.5)
        C.add(f"bearing: {case}: idler pin", r["idler_N"] <= d["design"]["idler_allow"], f"{r['idler_N']} N vs allow {d['design']['idler_allow']:g} N", warn="Full throttle" in case)


def joints(R, C):
    d = load_json(os.path.join(R, "joint_loads.json")); lim = d["limits"]
    along, across, sf = lim["petg_along"] / lim["sf"], lim["petg_across"] / lim["sf"], lim["sf"]
    for jn, cases in d["rated"].items():
        worst = {"case": "", "v": 0.0, "k": ""}
        for case, r in cases.items():
            for k in ("horn_ear_MPa", "idler_ear_MPa", "idler_bore_MPa", "stub_bend_MPa", "stub_shear_MPa"):
                if r[k] > worst["v"]: worst = dict(case=case, v=r[k], k=k)
        C.add(f"{jn}: worst stress {worst['k']}", worst["v"] <= along, f"{worst['v']} MPa in '{worst['case']}'; allowable {along:g} along layers / {across:g} across (PETG {lim['petg_along']:g}/{lim['petg_across']:g} MPa, SF {sf:g})", warn=worst["v"] > across)


def battery(R, C):
    d = load_json(os.path.join(R, "battery.json")); rows = []
    nb = d["no_battery"]; rows.append(["no pack", "-", "-", nb["current_A"], "-", nb["tw_rated"], nb["tw_full"]])
    for n, p in d["packs"].items():
        rows.append([n, p["pack_kg"], p["hover_min"], p["current_A"], f"{p['pack_cont_A']:g}", p["tw_rated"], p["tw_full"]])
        C.add(f"{n}: pack can supply the hover current", p["current_ok"], f"{p['current_A']} A drawn vs {p['pack_cont_A']:g} A continuous", warn=True)
        C.add(f"{n}: rated T/W >= 1.15", p["tw_rated"] >= 1.15, f"{p['tw_rated']} (robot {p['robot_kg']} kg)", warn=True)
    print(table(rows, ["pack", "kg", "hover min", "A drawn", "A cont.", "T/W rated", "T/W full"]))
    a = d["assumptions"]; print(f"assumptions: EDF {a['edf_N']} N @ {a['edf_g_per_W']} g/W, {a['wh_per_kg']} Wh/kg, {a['usable']*100:.0f}% usable, other {a['other_W']} W [A]; EDF replaced later by the central prop (see central_prop.json)\n")


def thrust(R, C):
    nb = load_json(os.path.join(R, "battery.json"))["no_battery"]
    gp = os.path.join(os.path.dirname(R), "..", "arm-pin-hinge", "out_1m_13", "root_gear.json")
    m0, tw0 = nb["robot_kg"], nb["tw_rated"]
    rows = [[f"+{x:g} kg", f"{m0+x:.2f}", f"{tw0*m0/(m0+x):.3f}", "ok" if tw0 * m0 / (m0 + x) >= 1.15 else "below 1.15"] for x in (0, 0.1, 0.25, 0.5, 1.0, 1.5, 2.0)]
    print(f"T/W = T/(m g): rated thrust fixed, base robot {m0} kg at {tw0}"); print(table(rows, ["added", "robot kg", "rated T/W", ""]))
    print(f"each +100 g costs {tw0*m0/(m0+0.1)-tw0:+.4f} T/W; headroom to 1.15 = {(tw0*m0/1.15-m0)*1000:.0f} g (battery.json basis; geared-root budget in root_gear.json)")
    if os.path.exists(gp):
        g = load_json(gp); C.add("main design rated T/W (root_gear.json)", g["thrust_to_weight"] >= 1.15 - 1e-3, f"{g['thrust_to_weight']:.3f} at {g['robot_kg']:.2f} kg")


def prop(R, C):
    rows = load_json(os.path.join(R, "prop_clearance.json"))["rows"]
    by = {}
    for r in rows: by.setdefault(f"{r['prop']} at offset {r['offset_mm']:g} mm", []).append(r)
    for p, rs in sorted(by.items()):
        rs.sort(key=lambda r: r["claw_h_mm"]); ok = [r for r in rs if r["any_dir_within_30deg_of_up_clear_in_every_pose"]]
        C.add(f"prop {p}: some claw height clears within 30 deg of up in every pose", bool(ok), f"clear from claw height {ok[0]['claw_h_mm']:g} mm" if ok else "no tested claw height up to " + f"{rs[-1]['claw_h_mm']:g} mm", warn=True)
        print(f"  {p}: " + ", ".join(f"h={r['claw_h_mm']:g}: {r['frac_dirs_clear_in_every_pose']*100:.0f}% dirs clear" for r in rs))


def fluid(R, C):
    rows = list(csv.DictReader(open(os.path.join(R, "fluid_len.csv"))))
    best = {}
    for r in rows:
        if r["feasible"] == "True":
            k = (r["length_m"], r["fit"], r["joints"])
            if k not in best or float(r["wave_tip_rms_mm"]) < float(best[k]["wave_tip_rms_mm"]): best[k] = r
    out = [[k[0], k[1][:26], k[2], r["wave_hz"], f"{float(r['wave_tip_rms_mm']):.0f}", f"{float(r['wave_sat_frac'])*100:.0f}%", f"{float(r['total_mass_kg']):.1f}", f"{float(r['tw']):.2f}"] for k, r in sorted(best.items())]
    print(table(out, ["length m", "fit", "joints", "best Hz", "tip rms mm", "sat", "kg", "T/W"]))
    C.add("fluid_len.csv has feasible rows", bool(best), f"{len(best)} length/fit combinations")


def stance(R, C):
    d = load_json(os.path.join(R, "pod_stance.json"))
    for n, c in d["cases"].items():
        C.add(f"stance: {n}", c["all_feasible"], f"worst servo use over the gait {c['worst_over_gait']:.2f}, at 2x foot load {c['worst_over_gait_at_2x_foot_load']:.2f} (1.00 = stall)", warn=c["worst_over_gait_at_2x_foot_load"] > 0.9)


def servos(R, C):
    sys.path.insert(0, os.path.join(ROOT, "joint-study", "sim"))
    from servos import SERVOS
    cad = (45.2, 24.7, 35.0); rows = []
    for n, s in sorted(SERVOS.items(), key=lambda kv: -kv[1]["stall"]):
        L, W, H = s["dims"]; fits = L <= cad[0] + 0.5 and W <= cad[1] + 0.5 and H <= cad[2] + 0.5
        rows.append([n, f"{s['mass']*1000:.0f}", f"{s['stall']:.2f}", f"{s['stall']/s['mass']:.0f}", f"{s['speed']:.1f}", f"{s['price']}", f"{L}x{W}x{H}", "fits CAD case" if fits else "needs new cradle"])
    print(table(rows, ["model", "g", "stall N m", "N m/kg", "rad/s", "$", "LxWxH", "vs CAD case 45.2x24.7x35"]))
    C.add("servo table loads", bool(SERVOS), f"{len(SERVOS)} models")


TOPICS = dict(bearings=bearings, joints=joints, battery=battery, thrust=thrust, prop=prop, fluid=fluid, stance=stance, servos=servos)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("topic", choices=list(TOPICS) + ["all"]); ap.add_argument("--results", default=RES)
    a = ap.parse_args()
    C = Checks(f"study results: {a.topic} ({a.results})")
    for t in (TOPICS if a.topic == "all" else [a.topic]):
        try: TOPICS[t](a.results, C)
        except FileNotFoundError as e: C.add(t, False, f"missing {os.path.basename(e.filename)}", warn=True)
    C.exit()


if __name__ == "__main__":
    main()
