"""Arm loads, walking and hover with the folding fan pod (arm-pin-hinge/fan_pod.py). Writes results/fan_pod_arm_loads.json."""
import os, sys, json, math
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
sys.path.insert(0, "/mnt/project-files/octopus-drone/joint-study/sim")
import numpy as np
import config as C, bearing_loads as B, joint_loads as JL, chain as CH, claw_fans as CF
CAPS = [9.8] + [4.9] * 4 + [2.94] * 8
FAN = CF.FANS["MN5008 KV400 + 16x5.4 CF"]; FULL = CF.fan_full(FAN); RATED = 0.75 * FULL
PACK = CF.PACKS["2 x DOGCOM 22 Ah 5C semi-solid"]
def need(ch, masses, q, F, r_off=None):
    out = []
    m, eo, ea, eang, R = CH.run_fk(ch, q); es = np.where(ch.et == 0)[0]
    for j in range(13):
        M = B.wrench(ch, q, masses, F, j)[0][0]
        if r_off is not None:
            a = ea[es[j]] / np.linalg.norm(ea[es[j]]); M = M + float(a @ np.cross(r_off, F))
        out.append(abs(M) / CAPS[j])
    return out
def foot_offsets(ch, q, r):
    m = CH.run_fk(ch, q)[0]; z = m[-1] - m[-2]; z /= np.linalg.norm(z)
    u = np.cross(z, [0, 0, 1.0]); u = u / np.linalg.norm(u) if np.linalg.norm(u) > 1e-6 else np.array([1.0, 0, 0]); v = np.cross(z, u)
    return [r * (math.cos(t) * u + math.sin(t) * v) for t in np.linspace(0, 2 * math.pi, 24, endpoint=False)]
out = {}
for tag, tip_g, arm_extra in [("pod, ESC at the tip", 247.6, 0.0), ("pod, ESC moved to joint 10", 227.6, 0.020)]:
    d_g = tip_g + arm_extra * 1000 - 130.0
    robot = CF.BASE_KG + 8 * d_g / 1000 + PACK[0]
    P, Tc, fan = CF.hover(robot, FAN, RATED); W = robot * 9.81
    tw = (CF.CP.P17_MAX_N * (1 - 0.036) + 8 * RATED) / W
    hov = dict(robot_kg=round(robot, 2), tw_rated=round(tw, 2), hover_min=round(PACK[1] * 0.8 / P * 60, 1), current_A=round(P / CF.CP.V_LOADED),
               central_N=round(Tc, 1), fan_hover_N=round(fan, 1), total_W=round(P))
    C.TIP_PAYLOAD_MASS = tip_g / 1000; B.ROBOT_KG = robot; B.FAN = dict(hover=fan, rated=RATED, full=FULL)
    ch, masses = B.build(); masses[10] += arm_extra
    Pz, st = B.poses(ch)
    res = JL.run(); rated = JL.rate_all(res); worst = {}
    for cn in ["Hover, any pose", "Rated thrust within 30 deg of straight up", "Rated thrust within 60 deg of straight up", "Rated thrust, any direction", "Full throttle, any direction", "Walking stance", "Hard landing"]:
        hs = [res[j][cn].get("held", 0) for j in res]; tot = [res[j][cn].get("total", 0) for j in res]
        r = [rated[j][cn] for j in rated if cn in rated[j]]
        worst[cn] = dict(horn_N=max(x["horn_N"] for x in r), idler_N=max(x["idler_N"] for x in r), stub_MPa=max(x["stub_bend_MPa"] for x in r), held=f"{min(hs)}/{tot[0]}")
    fw = robot * C.G / C.FEET_ON_GROUND; F = np.array([0, 0, fw])
    walk_under = np.max([need(ch, masses, q, F) for q in st], axis=0)
    walk_fold_worst = np.max([np.max([need(ch, masses, q, F, r) for r in foot_offsets(ch, q, 0.0425)], axis=0) for q in st], axis=0)
    walk_fold_mean = np.max([np.mean([need(ch, masses, q, F, r) for r in foot_offsets(ch, q, 0.0425)], axis=0) for q in st], axis=0)
    straight = need(ch, masses, Pz["straight out"], np.zeros(3))
    assist = need(ch, masses, Pz["straight out"], np.array([0, 0, 4.0]))
    out[tag] = dict(hover=hov, cases=worst, foot_N=round(fw, 1),
                    walk_on_tip_need_over_stall=[round(float(x), 2) for x in walk_under],
                    walk_on_folded_finger_worst_roll=[round(float(x), 2) for x in walk_fold_worst],
                    walk_on_folded_finger_mean_roll=[round(float(x), 2) for x in walk_fold_mean],
                    straight_out_fans_off=[round(float(x), 2) for x in straight], straight_out_4N_assist=[round(float(x), 2) for x in assist])
    print("==", tag, hov)
    for k, v in worst.items(): print("  ", k, v)
    for k in ("walk_on_tip_need_over_stall", "walk_on_folded_finger_worst_roll", "walk_on_folded_finger_mean_roll", "straight_out_fans_off", "straight_out_4N_assist"):
        print("  ", k, out[tag][k])
json.dump(out, open("/mnt/project-files/octopus-drone/joint-study/results/fan_pod_arm_loads.json", "w"), indent=1)
