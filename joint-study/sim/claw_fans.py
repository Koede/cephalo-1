"""Bigger claw fans for Cephalo-1 (Forest, Nov 9: "maybe we increase the claw/fan size").

For each claw-fan option (motor + prop, from the makers' 6S bench tables, LIGPOWER pages read Nov 9 2026) and each
flight pack, this finds the hover split between the central 17 in prop under the dome and the eight claw fans that
draws the least power, and reports hover time, current, and thrust-to-weight. Then it checks what a bigger fan does
to the arm: tip mass against the joints' holding torque with the fans off, and side load on joints 1 to 4.

Conventions as central_prop.py: 80 % of the pack used, 150 W for the rest, 21 V under load; a fan's "rated" thrust is
75 % of its full thrust (as today: 16 of 21.4 N), and hover may use up to rated. Tables at 23-24 V are taken 5 % low
on full thrust for 6S under load [A]. Writes results/claw_fans.json.   python3 claw_fans.py
"""
import os, sys, json, math
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
RES = os.path.join(HERE, "results") if os.path.isdir(os.path.join(HERE, "results")) else os.path.join(HERE, "..", "results")
import numpy as np
import central_prop as CP

G = 9.81
BASE_KG = 14.93 + 0.259          # central prop under the dome (central_prop.py) + horn bearings on joints 1-4 (horn_bearing.py)
TODAY = dict(motor=52.6, prop=8.0, esc=15.0)   # g; claw fan inside today's 130 g tip budget [prop, ESC A]

# name: motor g, prop g [A], ESC g [A], prop in, table [(thrust g, W)], full-thrust derate, source
FANS = {
    "2807 1250KV + 7.5 in 3-blade (today)": dict(motor=52.6, prop=8, esc=15, d=7.5, law=(13.2, 270.0), full_N=21.4, derate=1.0,
                                                  src="study's T-Motor 2807 table: 13.2 N at 270 W, 21.4 N at 551 W"),
    "MN4006 KV380 + 15x5 CF": dict(motor=68, prop=22, esc=20, d=15, derate=0.95, src="LIGPOWER MN4006 KV380, 24 V",
        tab=[(805, 74), (959, 98), (1093, 115), (1236, 139), (1561, 199), (1823, 257), (2228, 360)]),
    "V3115 640KV + 10.5x5 3-blade": dict(motor=113, prop=18, esc=20, d=10.5, derate=0.95, src="LIGPOWER V3115 KV640, 24 V",
        tab=[(345, 47), (644, 95), (995, 157), (1416, 253), (1835, 386), (2304, 529), (2791, 704), (3288, 908)]),
    "V3115 900KV + 9.5x5 3-blade": dict(motor=113, prop=15, esc=30, d=9.5, derate=0.95, src="LIGPOWER V3115 KV900, 24 V",
        tab=[(547, 93), (933, 177), (1404, 299), (1902, 491), (2480, 681), (3084, 951), (3663, 1258), (4246, 1609)]),
    "MN5008 KV400 + 15x5 CF": dict(motor=132, prop=22, esc=20, d=15, derate=0.95, src="LIGPOWER MN5008 KV400, about 23 V",
        tab=[(672, 66), (845, 89), (1033, 116), (1223, 146), (1409, 178), (1610, 215), (1811, 255), (2048, 302), (2286, 353), (2782, 472), (3027, 535)]),
    "MN5008 KV400 + 16x5.4 CF": dict(motor=132, prop=26, esc=20, d=16, derate=0.95, src="LIGPOWER MN5008 KV400, about 23 V",
        tab=[(803, 76), (1021, 104), (1259, 138), (1492, 176), (1722, 216), (1956, 260), (2185, 304), (2458, 361), (2750, 425), (3323, 566), (3591, 638)]),
    "MN5008 KV400 + 17x5.8 CF": dict(motor=132, prop=30, esc=25, d=17, derate=0.95, src="LIGPOWER MN5008 KV400, about 23 V",
        tab=[(917, 87), (1170, 121), (1447, 163), (1712, 207), (1963, 253), (2230, 305), (2491, 362), (2757, 421), (3090, 495), (3697, 655), (3982, 740)]),
}
PACKS = {   # name: kg, Wh, continuous A (sellers' pages read Nov 9 2026; see central_prop.py)
    "Tattu 16 Ah 30C LiPo": (1.988, 355.2, 480),
    "Tattu 22 Ah 25C LiPo": (2.650, 488.4, 550),
    "2 x DOGCOM 22 Ah 5C semi-solid": (2.780, 976.8, 220),
    "3 x DOGCOM 22 Ah 5C semi-solid": (4.170, 1465.2, 330),
    "4 x DOGCOM 22 Ah 5C semi-solid": (5.560, 1953.6, 440),
}
# arm checks (from joint_loads.py, bearing_loads/tiptorque runs, horn_bearing.py)
HOLD_J3 = (0.89, 0.0016)         # joint 3 need/stall holding the arm straight out, fans off: at 130 g tip, per extra g
HOLD_J6 = (0.80, 0.0018)
SIDE_30 = 191 / 16.0             # N on the horn side of joints 1-4 per N of fan thrust, fan within 30 deg of up
SIDE_ANY = 511 / 21.4            # N per N, fan pushing any direction at full
STUB_30 = 12.3 / 21.4            # MPa in the printed idler stub per N of fan thrust within 30 deg of up (limit 12.5)
BRG_C0 = 834.0

def fan_full(f):
    if "law" in f: return f["full_N"]
    return f["tab"][-1][0] * G / 1000 * f["derate"]

def fan_P(f, T):
    if "law" in f:
        T0, P0 = f["law"]; return P0 * (max(T, 0) / T0) ** 1.5
    t = np.array(f["tab"], float); g = T / G * 1000
    if g <= t[0, 0]: return float(t[0, 1] * (g / t[0, 0]) ** 1.5)
    return float(np.interp(g, t[:, 0], t[:, 1]))

def hover(robot_kg, f, rated):
    W = robot_kg * G; Tmax, fd = CP.P17_MAX_N, 0.036; best = None
    for Tc in np.linspace(0, 0.90 * Tmax, 300):
        fan = (W - Tc * (1 - fd)) / 8
        if fan < 0 or fan > rated: continue
        P = CP.p_central(Tc, 17.0) + 8 * fan_P(f, fan) + CP.OTHER_W
        if best is None or P < best[0]: best = (P, Tc, fan)
    return best

def main():
    out = dict(assumptions=dict(base_kg_no_pack=BASE_KG, rated_fraction=0.75, usable=0.8, other_W=150, v_loaded=21,
                                today_fan_g=TODAY, note="prop and ESC masses assumed; motor masses from the makers' pages"), fans={})
    for nm, f in FANS.items():
        full = fan_full(f); rated = 0.75 * full
        d_g = (f["motor"] - TODAY["motor"]) + (f["prop"] - TODAY["prop"]) + (f["esc"] - TODAY["esc"]) + (5 if f["d"] >= 10 else 0)
        tip_g = 130 + d_g
        row = dict(full_N=round(full, 1), rated_N=round(rated, 1), prop_in=f["d"], extra_tip_g=round(d_g), tip_g=round(tip_g), src=f["src"],
                   hold_straight_fans_off=dict(joint3=round(HOLD_J3[0] + HOLD_J3[1] * d_g, 2), joint6=round(HOLD_J6[0] + HOLD_J6[1] * d_g, 2)),
                   side_N=dict(rated_within_30=round(SIDE_30 * rated), full_any_dir=round(SIDE_ANY * full)),
                   bearing_of_C0=dict(rated_within_30=round(SIDE_30 * rated / BRG_C0, 2), full_any_dir=round(SIDE_ANY * full / BRG_C0, 2)),
                   stub_MPa_full_within_30=round(STUB_30 * full, 1), packs={})
        for pk, (kg, wh, amps) in PACKS.items():
            robot = BASE_KG + 8 * d_g / 1000 + kg; W = robot * G
            tw = (CP.P17_MAX_N * (1 - 0.036) + 8 * rated) / W; twf = (CP.P17_MAX_N * (1 - 0.036) + 8 * full) / W
            b = hover(robot, f, rated)
            if b is None:
                row["packs"][pk] = dict(robot_kg=round(robot, 2), tw_rated=round(tw, 3), hover="no: fans past rated"); continue
            P, Tc, fan = b; A = P / CP.V_LOADED
            row["packs"][pk] = dict(robot_kg=round(robot, 2), tw_rated=round(tw, 3), tw_full=round(twf, 3), central_N=round(Tc, 1),
                                    fan_hover_N=round(fan, 1), fan_hover_W=round(fan_P(f, fan)), total_W=round(P), current_A=round(A),
                                    pack_cont_A=amps, current_ok=bool(A <= amps), hover_min=round(wh * 0.8 / P * 60, 1),
                                    passes_1_15=bool(tw >= 1.15 and A <= amps))
        out["fans"][nm] = row
    json.dump(out, open(os.path.join(RES, "claw_fans.json"), "w"), indent=1)
    for nm, r in out["fans"].items():
        print(f"\n== {nm}: full {r['full_N']} N, rated {r['rated_N']} N, tip {r['tip_g']} g (+{r['extra_tip_g']}), hold j3 {r['hold_straight_fans_off']['joint3']}, "
              f"side30 {r['side_N']['rated_within_30']} N, brg full {r['bearing_of_C0']['full_any_dir']}, stub {r['stub_MPa_full_within_30']} MPa")
        for pk, p in r["packs"].items():
            if "hover_min" in p:
                print(f"   {pk:32s} {p['robot_kg']:6.2f} kg  T/W {p['tw_rated']:.3f}  hover {p['hover_min']:5.1f} min  {p['total_W']} W {p['current_A']} A/{p['pack_cont_A']}  fan {p['fan_hover_N']} N  {'PASS' if p['passes_1_15'] else ''}")
            else:
                print(f"   {pk:32s} {p['robot_kg']:6.2f} kg  T/W {p['tw_rated']:.3f}  {p['hover']}")

if __name__ == "__main__":
    main()
