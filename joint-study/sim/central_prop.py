"""Central propeller in place of the EDF: above the dome or under it (Forest, Nov 9).

Motor and prop: T-Motor Antigravity MN6007II KV320 on 6S, the maker's bench table (LIGPOWER P22x6.6 CF, 22 in) is
the power curve. Under the dome the walking legs leave room for 17 in only; there the motor is the MN5212 KV420 with a 17x5.8 CF
prop (maker's 24 V table, 5 % less top thrust at 6S [A]), because the MN6007II KV320 cannot spin a 17 in fast enough.

Placement:
  above : disc on a short mast over the dome; must clear the walking arms, which arch up over the body
  below : disc under the dome; must clear the walking legs and stay off the ground (body 200 mm up)
Clearance is checked against the study's stance poses (4 stances, each also with the foot 45 mm up) and the flight
pose (arms straight out), with the 57.1 mm arm tube and 20 mm margin [A].

Download: a body in the prop's flow sees D = 0.5 rho (k v_i)^2 Cd S, with v_i the induced velocity at the disc and
k from the actuator-disc axial velocity, k = 1 + d/sqrt(d^2+R^2) downstream (above case) and 1 - d/sqrt(d^2+R^2)
upstream (below case). As a share of thrust, f = k^2 Cd S / (4 A). Dome [A]: 260 mm across (flanges clear at a
124.5 mm hub radius), Cd 0.4 (hemisphere). Arms in the flight pose run straight out at the hub's height; the part
inside the slipstream counts as a 57.1 mm cylinder, Cd 1.1.

Writes results/central_prop.json.   python3 central_prop.py
"""
import os, sys, math, json
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
RES = os.path.join(HERE, "results") if os.path.isdir(os.path.join(HERE, "results")) else os.path.join(HERE, "..", "results")
import numpy as np

G, RHO = 9.81, 1.20
IN = 0.0254
# MN6007II KV320, LIGPOWER P22x6.6 CF, 6S: thrust g -> power W (maker's table)
P22 = np.array([[1573, 137], [1677, 150], [1795, 165], [1895, 180], [2048, 197], [2156, 214], [2264, 233], [2409, 254],
                [2543, 270], [2666, 291], [2788, 313], [2927, 334], [3061, 357], [3194, 378], [3300, 401], [3421, 423],
                [4112, 560], [4694, 703], [5547, 908]], float)
P22_MAX_N = 5547 * G / 1000
# MN5212 KV420, 17x5.8 CF, 24 V bench (maker's table via LIGPOWER): the motor for the 17 in prop under the dome
P17 = np.array([[1795, 250.8], [2075, 306.0], [2458, 382.3], [2745, 452.6], [3576, 651.8], [4324, 872.4], [5359, 1237.9]], float)
P17_MAX_N = 5359 * G / 1000 * 0.95     # [A] 5 % less at 6S under load (22-23 V) than the 24 V bench
EDF_N = 42.2
ROBOT_KG = 15.08
EDF_SYS_KG = 0.55      # [A] 90 mm 12-blade EDF with motor ~0.42 kg + 120 A ESC ~0.13 kg; Forest's unit is not known
EDF_W = 2967.0         # [A] at 42.2 N, 1.45 g/W (battery.py)
CENTRAL_KG = {22.0: dict(motor=0.180, prop=0.045, esc=0.075),   # MN6007II 180 g with cable (maker); prop and 60 A ESC [A]
              17.0: dict(motor=0.249, prop=0.030, esc=0.085)}   # MN5212 249 g with cable (maker); prop and 70 A ESC [A]
MOUNT_KG = dict(above=0.070, below=0.035)               # [A] printed mast and motor plate / short motor plate
GUARD_KG = 0.120       # [A] printed ring guard round a 22 in disc
FAN_REF_N, FAN_REF_W, FAN_RATED, FAN_FULL = 13.2, 270.0, 16.0, 21.4
OTHER_W, V_LOADED, WH_PER_KG, USABLE = 150.0, 21.0, 192.0, 0.80
ARM_R, MARGIN = 0.02855, 0.020
DOME_D, DOME_CD, ARM_CD = 0.260, 0.40, 1.1
DOME_TOP, DOME_BOTTOM = 0.120, -0.100   # [A] dome extent above / below the hub centre (the arms leave at z 0)
GROUND_Z = -0.200

def p_central(T_N, D_in=22.0):
    g = T_N / G * 1000
    tab = P22 if D_in == 22.0 else P17
    w = np.interp(g, tab[:, 0], tab[:, 1], left=tab[0, 1] * (g / tab[0, 0]) ** 1.5, right=np.nan)
    return float(w)

def p_fan(T):
    return FAN_REF_W * (max(T, 0.0) / FAN_REF_N) ** 1.5

def arm_points():
    import config as C, chain as CH, bearing_loads as B, metrics as M
    ch, masses = B.build(); x = M._x0(ch); sets = {}
    for i, f in enumerate(M.FEET):
        for lift in (0.0, 0.045):
            q = M.ik_foot(ch, f + np.array([0, 0, lift]), x)[0]
            if lift == 0.0: x = q
            sets[f"stance {i+1}{' foot up' if lift else ''}"] = q
    sets["flight, arms straight out"] = M._x0(ch)
    out = {}
    for k, q in sets.items():
        m = CH.run_fk(ch, q)[0]
        s = np.linspace(0, 1, 20)[:, None]
        pts = np.vstack([m[i] + s * (m[i + 1] - m[i]) for i in range(len(m) - 1)])
        out[k] = np.c_[np.hypot(pts[:, 0], pts[:, 1]), pts[:, 2]]
    return out

def disc_gap(rz, R, h):
    r, z = rz[:, 0], rz[:, 1]
    d = np.where(r <= R, np.abs(z - h), np.hypot(r - R, z - h))
    return float(d.min() - ARM_R)

def download(R, h, place):
    A = math.pi * R ** 2
    d = (h - 0.0) if place == "above" else (0.0 - h)          # disc to the hub / arm plane
    dd = (h - DOME_TOP * 0.3) if place == "above" else (DOME_BOTTOM * 0.3 - h)   # disc to the dome's bulk [A]
    if place == "above":
        k_dome = 1 + dd / math.hypot(dd, R); k_arm = 1 + d / math.hypot(d, R)
        r_slip = R * math.sqrt(1 / k_arm)                    # slipstream radius at the arm plane (continuity)
    else:
        k_dome = 1 - dd / math.hypot(dd, R); k_arm = 1 - d / math.hypot(d, R)
        r_slip = R * 1.0                                       # inflow stream tube is wider than the disc upstream
    S_dome = math.pi * min(DOME_D / 2, r_slip) ** 2
    L_arm = max(0.0, r_slip - 0.110)                           # arm length inside the stream, each arm
    S_arm = 8 * 2 * ARM_R * L_arm
    f_dome = k_dome ** 2 * DOME_CD * S_dome / (4 * A); f_arm = k_arm ** 2 * ARM_CD * S_arm / (4 * A)
    return dict(k_dome=round(k_dome, 2), k_arm=round(k_arm, 2), slip_r_mm=round(r_slip * 1e3), f_dome=round(f_dome, 3),
                f_arm_flight=round(f_arm, 3), f_total=round(f_dome + f_arm, 3))

def hover(robot_kg, f, D_in, Tmax):
    W = robot_kg * G; best = None
    for Tc in np.linspace(0, 0.90 * Tmax, 400):
        fan = (W - Tc * (1 - f)) / 8
        if fan < 0 or fan > FAN_RATED: continue
        P = p_central(Tc, D_in) + 8 * p_fan(fan) + OTHER_W
        if best is None or P < best[0]: best = (P, Tc, fan)
    if best is None: return None
    P, Tc, fan = best
    return dict(central_N=round(Tc, 1), fan_N=round(fan, 2), central_W=round(p_central(Tc, D_in)), fans_W=round(8 * p_fan(fan)),
                total_W=round(P), current_A=round(P / V_LOADED),
                tw_rated=round((Tmax * (1 - f) + 8 * FAN_RATED) / W, 3), tw_full=round((Tmax * (1 - f) + 8 * FAN_FULL) / W, 3))

def main():
    pts = arm_points()
    out = dict(assumptions=dict(edf_system_kg=EDF_SYS_KG, central_kg=CENTRAL_KG, mount_kg=MOUNT_KG, guard_kg=GUARD_KG, dome_d_mm=DOME_D * 1e3,
                                dome_cd=DOME_CD, arm_cd=ARM_CD, margin_mm=MARGIN * 1e3, wh_per_kg=WH_PER_KG, usable=USABLE, other_W=OTHER_W),
               edf_baseline=dict(robot_kg=ROBOT_KG, edf_N=EDF_N, edf_W=EDF_W), placements={})
    # above: 22 in, lowest disc height that clears every pose
    R = 22 * IN / 2
    h_above = None
    for h in np.arange(DOME_TOP + 0.03, 0.60, 0.005):
        if min(disc_gap(p, R, h) for p in pts.values()) >= MARGIN: h_above = round(float(h), 3); break
    # below: disc plane 25 mm under the dome bottom for the motor [A]; largest prop that clears the legs
    h_below = DOME_BOTTOM - 0.025
    D_below = None
    for D in np.arange(30, 7, -1.0):
        if min(disc_gap(p, D * IN / 2, h_below) for p in pts.values()) >= MARGIN: D_below = float(D); break
    gaps = {}
    for nm, (D, h) in dict(above=(22.0, h_above), below=(D_below, h_below)).items():
        gaps[nm] = {k: round(disc_gap(p, D * IN / 2, h) * 1e3, 1) for k, p in pts.items()}
    cases = {
        "above, 22 in on a mast": dict(place="above", D=22.0, h=h_above, guard=False),
        "above, 22 in with a ring guard": dict(place="above", D=22.0, h=h_above, guard=True),
        f"below, {D_below:g} in": dict(place="below", D=D_below, h=h_below, guard=False),
    }
    for nm, c in cases.items():
        R = c["D"] * IN / 2
        Tmax = P22_MAX_N if c["D"] == 22.0 else P17_MAX_N
        f = download(R, c["h"], c["place"])
        mass_add = sum(CENTRAL_KG[c["D"]].values()) + MOUNT_KG[c["place"]] + (GUARD_KG if c["guard"] else 0) - EDF_SYS_KG
        base = ROBOT_KG + mass_add
        row = dict(place=c["place"], prop_in=c["D"], disc_z_mm=round(c["h"] * 1e3), ground_clear_mm=round((c["h"] - GROUND_Z) * 1e3) if c["place"] == "below" else None,
                   arm_gaps_mm=gaps[c["place"]], download=f, mass_change_kg=round(mass_add, 3), robot_kg_no_pack=round(base, 2),
                   central_max_N=round(Tmax, 1), central_max_W=round(p_central(Tmax * 0.999, c["D"])), packs={})
        for pk in (0.0, 1.0, 1.6, 2.3):
            h = hover(base + pk, f["f_total"], c["D"], Tmax)
            if h is None: row["packs"][str(pk)] = None; continue
            h["hover_min"] = round(pk * WH_PER_KG * USABLE / h["total_W"] * 60, 1)
            row["packs"][str(pk)] = h
        # walking-stance hover with the dome in the flow but arms down (take-off start): download from the dome only
        row["takeoff_dome_only_f"] = f["f_dome"]
        out["placements"][nm] = row
    # EDF reference with the same packs (battery.py model)
    out["edf_packs"] = {}
    for pk in (0.0, 1.0, 1.6, 2.3):
        W = (ROBOT_KG + pk) * G; fan = (W - EDF_N) / 8; P = 8 * p_fan(fan) + EDF_W + OTHER_W
        out["edf_packs"][str(pk)] = dict(total_W=round(P), hover_min=round(pk * WH_PER_KG * USABLE / P * 60, 1),
                                         tw_rated=round((EDF_N + 8 * FAN_RATED) / W, 3))
    json.dump(out, open(os.path.join(RES, "central_prop.json"), "w"), indent=1)
    print("above disc z", h_above, "below prop", D_below, "in at z", h_below)
    for nm, r in out["placements"].items():
        print(f"\n== {nm}: download {r['download']}, mass change {r['mass_change_kg']} kg, max {r['central_max_N']} N")
        print("   gaps", r["arm_gaps_mm"])
        for pk, h in r["packs"].items(): print(f"   pack {pk} kg:", h)
    print("\nEDF", out["edf_packs"])

if __name__ == "__main__":
    main()

# Real 6S packs near Forest's 2.3 kg pick (Nov 9), with the central prop under the dome.
# name: (kg, Wh nominal, continuous A, burst A) from the sellers' pages read Nov 9 2026
REAL_PACKS = {
    "Tattu 6S 16,000 mAh 30C LiPo": (1.988, 16 * 22.2, 480, None),
    "Tattu 6S 22,000 mAh 25C LiPo": (2.650, 22 * 22.2, 550, 1100),
    "2 x Tattu 6S 10,000 mAh 25C LiPo, parallel": (2.800, 20 * 22.2, 500, 1000),
    "Tattu semi-solid 6S 33,000 mAh (350 Wh/kg)": (2.355, 33 * 22.2, None, 330),
    "2 x DOGCOM semi-solid 6S 22,000 mAh 5C, parallel": (2.780, 44 * 22.2, 220, 440),
}

def real_packs():
    import copy
    global FAN_RATED
    R = 17 * IN / 2; h = DOME_BOTTOM - 0.025
    f = download(R, h, "below")["f_total"]; base = ROBOT_KG + sum(CENTRAL_KG[17.0].values()) + MOUNT_KG["below"] - EDF_SYS_KG
    peak_W = p_central(P17_MAX_N * 0.999, 17.0) + 8 * p_fan(FAN_FULL) + OTHER_W
    out = {}
    for nm, (kg, wh, cont, burst) in REAL_PACKS.items():
        keep = FAN_RATED; FAN_RATED = FAN_FULL          # let the fans pass 16 N if a pack needs it, and say so
        hv = hover(base + kg, f, 17.0, P17_MAX_N); FAN_RATED = keep
        hv["tw_rated"] = round((P17_MAX_N * (1 - f) + 8 * 16.0) / ((base + kg) * G), 3)
        hv.update(pack_kg=kg, pack_Wh=round(wh), hover_min=round(wh * USABLE / hv["total_W"] * 60, 1),
                  cont_A=cont, burst_A=burst, peak_A=round(peak_W / V_LOADED),
                  hover_ok=bool((cont or 0) >= hv["current_A"]), fans_over_16N=bool(hv["fan_N"] > 16.0))
        hv = {k: (float(v) if isinstance(v, np.floating) else v) for k, v in hv.items()}
        out[nm] = hv
    return out

if __name__ == "__main__":
    rp = real_packs()
    d = json.load(open(os.path.join(RES, "central_prop.json"))); d["real_packs_under_dome"] = rp
    json.dump(d, open(os.path.join(RES, "central_prop.json"), "w"), indent=1)
    for k, v in rp.items(): print(k, v)
