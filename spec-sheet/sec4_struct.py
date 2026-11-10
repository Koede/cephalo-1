"""Sections 5-7: stiffness and sag, modes, speed, stresses and safety factors, fatigue, bearing lives. -> results/struct.json"""
import sys, os, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import spec_lib as S

TAG = sys.argv[1] if len(sys.argv) > 1 else "alt90"
A = S.Arm(TAG)
L = json.load(open(os.path.join(S.HERE, "results", f"loads_{TAG}.json")))
P = dict(np.load(os.path.join(S.HERE, "results", f"poses_{TAG}.npz")))
FAN_H, FAN_R, FAN_F = L["fan"]["hover_N"], L["fan"]["rated_N"], L["fan"]["full_N"]
FOOT = L["foot_N"]; SIDES = {"+x": [1, 0, 0], "-x": [-1, 0, 0], "+y": [0, 1, 0], "-y": [0, -1, 0]}
import poses as PS
out = {}
D = math.radians

# ---------------- load states ----------------
def states():
    st = []
    st.append(("straight out, fans off", np.zeros(S.N), np.zeros(3), "thrust", None, 1.0))
    for k, q in P.items():
        if "_flight_" in k or "_tilt_" in k or "_lean" in k:
            T = FAN_F if k.endswith("full") else FAN_H if k.endswith("hover") else FAN_R
            f = A.fk(q); st.append((k.replace(TAG + "_", ""), q, T * f["tipR"][:, 2], "thrust", None, 1.0))
        if "_stance_" in k:
            sd = SIDES[k.split("_")[2]]; f = A.fk(q)
            for mult in (1.0, 2.0):
                st.append((k.replace(TAG + "_", "") + (" x2" if mult > 1 else ""), q, np.zeros(3), "tip",
                           [(PS.foot_point(A, f, sd), np.array([0, 0, mult * FOOT]))], mult))
    return st
ST = states()

# ---------------- per-joint wrench envelope in the joint frame ----------------
A_H, A_I = 0.0245, -0.0245           # horn and idler supports along the hinge axis (m)
def split(Mj, Fj):
    """Mj, Fj in the joint frame (alpha, beta, zeta). Supports at a1, a2 on the alpha axis carry beta/zeta forces."""
    a1, a2 = A_H, A_I
    M = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, -a1, 0, -a2], [a1, 0, a2, 0]], float)
    s = np.linalg.solve(M, [-Fj[1], -Fj[2], -Mj[1], -Mj[2]])
    return s[0:2], s[2:4], float(-Fj[0])
def new_env(): return [dict(tau=0, horn=0, idler=0, axial=0, Mbz=0, horn_state="", idler_state="", Rh=np.zeros(2), Ri=np.zeros(2)) for _ in range(S.N)]
env = new_env(); env_hold = new_env()
per_state = []
for name, q, F, Fpt, extra, mult in ST:
    Ms, Fs, f = A.wrench(q, F, Fpt=Fpt, extra=extra)
    holdable = float(A.need(q, F, Fpt=Fpt, extra=extra)[0].max()) <= 1.0
    row = []
    for j in range(S.N):
        R = A.joint_frame(f, j); Mj = R @ Ms[j]; Fj = R @ Fs[j]
        Rh, Ri, ax = split(Mj, Fj)
        rh, ri = float(np.hypot(*Rh)), float(np.hypot(*Ri))
        for e in ([env[j], env_hold[j]] if holdable else [env[j]]):
            e["tau"] = max(e["tau"], abs(Mj[0]))
            if rh > e["horn"]: e["horn"], e["horn_state"], e["Rh"] = rh, name, Rh
            if ri > e["idler"]: e["idler"], e["idler_state"], e["Ri"] = ri, name, Ri
            e["axial"] = max(e["axial"], abs(ax)); e["Mbz"] = max(e["Mbz"], float(np.hypot(Mj[1], Mj[2])))
        row.append((abs(Mj[0]), rh, ri))
    per_state.append((name, row))
# also the servo at stall in its worst pose: the hinge supports must survive stall torque reacted through the ears
out["envelope"] = [dict(joint=j, servo=S.servo_name(j), max_servo_torque_Nm=round(e["tau"], 2), cap_Nm=round(float(S.CAP[j]), 2),
                        horn_support_N=round(e["horn"], 1), horn_case=e["horn_state"], idler_support_N=round(e["idler"], 1), idler_case=e["idler_state"],
                        axial_N=round(e["axial"], 1), bending_moment_Nm=round(e["Mbz"], 2),
                        horn_support_N_holdable=round(env_hold[j]["horn"], 1), idler_support_N_holdable=round(env_hold[j]["idler"], 1)) for j, e in enumerate(env)]
out["holdable_states"] = [n for n, q, F, Fpt, ex, m in ST if float(A.need(q, F, Fpt=Fpt, extra=ex)[0].max()) <= 1.0]

# ---------------- stresses and safety factors ----------------
PETG = dict(along=45.0, across=25.0, bearing=35.0, E=2000.0)   # MPa, printed, short term [A]
AL7075 = dict(yield_=503.0, uts=572.0, endurance=159.0, shear_y=290.0)
EAR_W, EAR_L, T_H, T_I = 21.0, 20.8, 4.0, 3.2
PIN_D, PIN_LEVER = 8.0, 2.1
Zp = math.pi * PIN_D ** 3 / 32; Ap = math.pi * PIN_D ** 2 / 4
def ear(R, t): return 6 * abs(R[0]) * EAR_L / (t * EAR_W ** 2) + abs(R[1]) / (t * EAR_W)
def stress_rows(env):
  rows = []
  for j, e in enumerate(env):
    pin = "7075 pin" if j <= 4 else "printed PETG stub"
    horn_ear = ear(e["Rh"], T_H); idler_ear = ear(e["Ri"], T_I)
    bore = e["idler"] / (PIN_D * T_I)
    pin_b = e["idler"] * PIN_LEVER / Zp; pin_s = e["idler"] / Ap
    # the ring tube between joints: thin wall 57.1 OD, 2.0 mm wall [A]
    Do, Di = 57.1, 53.1; Zt = math.pi * (Do ** 4 - Di ** 4) / (32 * Do)
    tube = e["Mbz"] * 1000 / Zt + e["tau"] * 1000 / (2 * Zt) * 0   # bending dominates; torsion small
    lim_pin_b = AL7075["yield_"] if j <= 4 else PETG["along"]
    lim_pin_s = AL7075["shear_y"] if j <= 4 else PETG["across"] * 0.6
    sf = dict(horn_ear=PETG["across"] / max(horn_ear, 1e-6), idler_ear=PETG["across"] / max(idler_ear, 1e-6), idler_bore=PETG["bearing"] / max(bore, 1e-6),
              pin_bend=lim_pin_b / max(pin_b, 1e-6), pin_shear=lim_pin_s / max(pin_s, 1e-6), tube=PETG["across"] / max(tube, 1e-6))
    rows.append(dict(joint=j, pin=pin, horn_ear_MPa=round(horn_ear, 1), idler_ear_MPa=round(idler_ear, 1), idler_bore_MPa=round(bore, 1),
                     pin_bend_MPa=round(pin_b, 1), pin_shear_MPa=round(pin_s, 1), tube_MPa=round(tube, 2),
                     sf={k: round(v, 1) for k, v in sf.items()}, min_sf=round(min(sf.values()), 2), weakest=min(sf, key=sf.get)))
  return rows
out["stress"] = stress_rows(env)
hold_rows = stress_rows(env_hold)
for r, h in zip(out["stress"], hold_rows): r["min_sf_holdable"] = h["min_sf"]; r["weakest_holdable"] = h["weakest"]
out["stress_note"] = ("Envelope over every solved pose and load: straight out with fans off, hover, rated and full thrust, rated thrust tilted 30 deg, "
                      "walking stance at 1x and 2x foot load. Ear: in-plane bending over the 20.8 mm ear plus direct pull, across the layers. "
                      "Limits: PETG 25 MPa across layers, 45 along, 35 MPa bearing [A]; 7075-T6 yield 503, shear 290 MPa. Forest's rule is SF >= 2.")
# horn screws and servo output side load (joints 5-12 have no horn bearing)
out["servo_side_load"] = [dict(joint=j, horn_side_N=round(env[j]["horn"], 1), horn_side_N_holdable=round(env_hold[j]["horn"], 1), case_holdable=env_hold[j]["horn_state"], XM430_reference_N=40.0, ratio=round(env[j]["horn"] / 40.0, 2))
                          for j in range(5, S.N)]
# lid screws: 4 x M2 into PETG pilot (pull-out about 150 N each in PETG, 6 mm engagement [A]); the lid carries the servo case axial load
out["lid_screws"] = dict(screws="4 x M2 x 8 self-tapping into PETG [A]", pullout_each_N=150, max_axial_N=round(max(e["axial"] for e in env), 1),
                         sf=round(4 * 150 / max(max(e["axial"] for e in env), 1e-6), 1))
# flange screws, root socket to the mantle: 8 x M3 on a 60 mm circle [A from root_gear.json: 8 x M3]
M0 = env[0]["Mbz"] + env[0]["tau"]; Fb = M0 / (0.030 * 4) + max(np.linalg.norm(x) for x in [np.zeros(1)]) * 0
out["root_flange"] = dict(screws="8 x M3 class 8.8 on a 60 mm bolt circle [A]", max_root_moment_Nm=round(M0, 2),
                          worst_screw_tension_N=round(M0 / (0.030 * 4), 1), M3_proof_N=round(0.6 * 5.03 * 600), sf_screw=round(0.6 * 5.03 * 600 / (M0 / (0.030 * 4)), 1),
                          note="screw sees the root moment over the bolt pattern; PETG under the heads (6 mm washer, 28 mm^2) limits first at about 35 MPa x 28 = 980 N")
# gear: Lewis and Hertz, at stall and at the worst flight and stance demand
def gear_stress(T_joint):
    Tp = T_joint / 2 / 0.98; Ft = Tp / 0.010
    lew_p = Ft / (10 * 1 * 0.322); lew_g = Ft / (10 * 1 * 0.389)
    ZE = math.sqrt(71700 / (2 * math.pi * (1 - 0.33 ** 2))); I = math.cos(D(20)) * math.sin(D(20)) / 2 * (2 / 3)
    hz = ZE * math.sqrt(Ft / (10 * 20 * I))
    return dict(joint_torque_Nm=round(T_joint, 2), tooth_force_N=round(Ft), lewis_pinion_MPa=round(lew_p), lewis_gear_MPa=round(lew_g), hertz_MPa=round(hz))
gtab = {"stall (9.8 N.m)": gear_stress(9.8)}
for name, row in per_state:
    if name.endswith("flight_hover"): gtab["hover"] = gear_stress(row[0][0])
    if name.endswith("flight_full"): gtab["full thrust"] = gear_stress(row[0][0])
gtab["worst solved pose"] = gear_stress(env[0]["tau"])
out["gear"] = dict(cases=gtab, limits=dict(lewis_bending_fatigue_MPa=AL7075["endurance"], yield_MPa=AL7075["yield_"],
                   contact_fatigue_MPa_hard_anodized="about 300 to 400 [A, aluminium gear data is thin]"),
                   note="Lewis Y 0.322 (20T) and 0.389 (40T), 10 mm face, m1. Contact stress at stall is about 540 MPa: fine for short peaks, but "
                        "hard-anodized 7075 pits under long running above roughly 350 MPa, so keep joint 0 under about 40 % of stall for sustained loads, "
                        "or make the pinion steel.")

# ---------------- stiffness and sag ----------------
K_SERVO = {"STS3250": 4.9 / D(2), "STS3215": 2.94 / D(2)}     # N.m/rad: stall torque within 2 deg of error (position loop) [A]
K_ALPHA = np.array([K_SERVO["STS3250"] * 4 * 0.98] + [K_SERVO[S.servo_name(j)] for j in range(1, S.N)])  # gear: ratio^2
# hinge tilt stiffness from the two ears (PETG plates, bending out of plane across a 49 mm span) [A, from ear plate theory]
def ear_tilt_k(t_h, t_i):
    E = PETG["E"] * 1e6; b = EAR_W / 1000; Lr = EAR_L / 1000; span = 0.049
    k_h = 3 * E * b * (t_h / 1000) ** 3 / 12 / Lr ** 3 * 0 + E * b * (t_h / 1000) / Lr      # in-plane: axial-ish stiffness of the ear (N/m)
    k_i = E * b * (t_i / 1000) / Lr
    kk = 1 / (1 / k_h + 1 / k_i)
    return kk * span ** 2 / 2
K_TILT = ear_tilt_k(T_H, T_I)
CLEAR_TILT = {"as drawn (8.4 bore)": 0.4e-3 / 0.049, "reamed (8.1 bore)": 0.1e-3 / 0.049}
def sag(q, F=np.zeros(3), Fpt="thrust", extra=None):
    Ms, Fs, f = A.wrench(q, F, Fpt=Fpt, extra=extra)
    tip = f["tip"]; d = np.zeros(3)
    for j in range(S.N):
        R = A.joint_frame(f, j); Mj = R @ Ms[j]
        lever = tip - f["o"][j]
        d += np.cross(R[0], lever) * Mj[0] / K_ALPHA[j]
        d += np.cross(R[1], lever) * Mj[1] / K_TILT + np.cross(R[2], lever) * Mj[2] / K_TILT
    return d
sg = {}
sg["straight out, fans off"] = sag(np.zeros(S.N))
for nm in ("hover", "full"):
    k = f"{TAG}_flight_{nm}"
    if k in P:
        q = P[k]; f = A.fk(q); T = FAN_H if nm == "hover" else FAN_F
        sg[f"hover pose, {nm} thrust"] = sag(q, T * f["tipR"][:, 2])
if f"{TAG}_lean20_hover" in P:
    q = P[f"{TAG}_lean20_hover"]; f = A.fk(q); sg["leaning hover pose (20 deg), hover thrust"] = sag(q, FAN_H * f["tipR"][:, 2])
for k in P:
    if "_stance_" in k and ("mid" in k or "side" in k):
        q = P[k]; sd = SIDES[k.split("_")[2]]; f = A.fk(q)
        sg[f"stance {k.split('_')[2]} pod, foot load"] = sag(q, np.zeros(3), "tip", [(PS.foot_point(A, f, sd), np.array([0, 0, FOOT]))])
out["sag_mm"] = {k: dict(vec=[round(x * 1000, 1) for x in v], mag=round(float(np.linalg.norm(v)) * 1000, 1)) for k, v in sg.items()}
# tip stiffness straight: unit loads
q0 = np.zeros(S.N); f0 = A.fk(q0); Kt = {}
for nm, dvec in (("up", [0, 0, 1]), ("sideways", [0, 1, 0]), ("along the arm", [1, 0, 0])):
    F = np.array(dvec, float) * 1.0
    d = sag(q0, F, "tip") - sag(q0)
    Kt[nm] = round(1.0 / max(abs(float(d @ np.array(dvec))), 1e-9) / 1000, 3)
out["tip_stiffness_N_per_mm_straight"] = Kt
out["stiffness_note"] = (f"Servo hold stiffness taken as stall torque per 2 deg of position error [A]: STS3250 {K_SERVO['STS3250']:.0f}, STS3215 {K_SERVO['STS3215']:.0f} N.m/rad, "
                         f"joint 0 {K_ALPHA[0]:.0f} through the 2:1 stage. Hinge tilt from ear stretch about {K_TILT:.0f} N.m/rad [A]. The ring tube "
                         "(PETG, 2 mm wall) is about 100 times stiffer than the joints and is left rigid. Clearance tilt is play, not spring: see the stack-up.")
# ---------------- modes ----------------
def modes(q):
    M = A.mass_matrix(q)[0]; K = np.diag(K_ALPHA)
    w2 = np.linalg.eigvals(np.linalg.solve(M, K)).real
    return sorted(round(math.sqrt(max(x, 0)) / (2 * math.pi), 2) for x in w2)[:4]
md = {"straight out": modes(np.zeros(S.N))}
if f"{TAG}_flight_hover" in P: md["hover pose, fan upright"] = modes(P[f"{TAG}_flight_hover"])
if f"{TAG}_lean20_hover" in P: md["hover pose, fan leaning 20 deg"] = modes(P[f"{TAG}_lean20_hover"])
out["modes_Hz"] = md
out["modes_note"] = ("Joint-space modes with the servo hold stiffness above; real servo loops add damping and lag, so treat these as upper bounds. "
                     "Keep the flight controller's attitude loop under about a third of the first mode, and the prop's once-per-rev (about 80 Hz at hover) is far above them.")
# ---------------- speed ----------------
spd = {}
for nm, k in (("fans off, straight", None), ("hover, fan upright", f"{TAG}_flight_hover"), ("hover, fan leaning 20 deg", f"{TAG}_lean20_hover")):
    if k is None: need = A.need(np.zeros(S.N))[0]
    elif k in P: q = P[k]; f = A.fk(q); need = A.need(q, FAN_H * f["tipR"][:, 2])[0]
    else: continue
    w = S.SPEED * np.clip(1 - need, 0, 1)
    spd[nm] = dict(joint_speed_rad_s=[round(float(x), 2) for x in w], at_80pct_rule=[round(float(x) * 0.8, 2) for x in w])
f0 = A.fk(np.zeros(S.N))
lev = np.array([np.linalg.norm(f0["tip"] - f0["o"][j]) for j in range(S.N)])
spd["tip_speed_straight_m_s"] = dict(joint0_alone=round(float(S.SPEED[0] * lev[0]), 2), all_pitch_joints_together=round(float(sum(S.SPEED[j] * lev[j] for j in range(0, S.N, 2))), 2))
spd["no_load_rad_s"] = [round(float(x), 2) for x in S.SPEED]
# bus timing: 1 Mbit/s, 10 bits per byte; sync write of goal position+speed (7 bytes per servo) and a sync read of position+load (each reply 10 bytes + 20 us return delay) [A]
bits = (8 + 13 * 7) * 10 + (8 + 13 * 2) * 10 + 13 * 10 * 10
t = bits / 1e6 + 13 * 20e-6
spd["bus"] = dict(baud=1000000, servos_per_arm_bus=13, cycle_ms=round(t * 1000, 2), max_loop_Hz=round(1 / t), note="one bus per arm; a shared bus for all 104 servos would run near 25 Hz")
out["speed"] = spd

# ---------------- fatigue ----------------
def basquin_N(sa, sf_, b): return 0.5 * (sa / sf_) ** (1 / b) if sa > 0 else float("inf")
walk = {}
st_max = {}; sw = {}
for name, row in per_state:
    if "stance" in name and "x2" not in name:
        for j, (tau, rh, ri) in enumerate(row): st_max[j] = max(st_max.get(j, 0), ri, rh)
# swing: arm unloaded in a mid pose, fans off
_sk = sorted([k for k in P if "_stance_" in k]); _qs = P[_sk[0]] if _sk else np.zeros(S.N)
Ms, Fs, f = A.wrench(_qs, np.zeros(3), Fpt="tip")
for j in range(S.N):
    R = A.joint_frame(f, j); Rh, Ri, _ = split(R @ Ms[j], R @ Fs[j]); sw[j] = max(np.hypot(*Rh), np.hypot(*Ri))
fat = []
for j in range(S.N):
    dF = abs(st_max.get(j, 0) - sw[j]) / 2
    sa_ear = 6 * dF * EAR_L / (T_I * EAR_W ** 2) * 0.7 + dF / (T_I * EAR_W)     # amplitude, mix of components [A]
    N_petg = basquin_N(sa_ear, PETG["across"], -0.085)
    sa_pin = dF * PIN_LEVER / Zp
    N_pin = basquin_N(sa_pin, 1466.0, -0.143) if j <= 4 else basquin_N(sa_pin, PETG["along"], -0.085)
    fat.append(dict(joint=j, support_load_amplitude_N=round(dF, 1), ear_stress_amp_MPa=round(sa_ear, 2), ear_cycles=float(f"{min(N_petg, 1e12):.2e}"),
                    pin_stress_amp_MPa=round(sa_pin, 2), pin_cycles=float(f"{min(N_pin, 1e12):.2e}")))
out["fatigue_walking"] = fat
out["fatigue_note"] = ("One walking step is one load cycle per joint: swing (arm unloaded) to stance (foot load). Basquin: 7075 sigma_f' 1466 MPa, b -0.143; "
                       "printed PETG across layers sigma_f' 25 MPa, b -0.085 (about 30 % of strength at a million cycles) [A]. Mean stress ignored; "
                       "printed parts scatter widely, so divide lives by 10 for design.")
# ---------------- bearing L10 ----------------
def l10_osc(C, Pload, psi_deg):
    if Pload <= 0: return float("inf")
    return (C / Pload) ** 3 * 1e6 * 360 / (4 * psi_deg)
brg = []
for j, (nm, C, C0) in [(0, ("61803 (gear)", 2030, 1270)), *[(j, ("6705 (horn side)", 1079, 834)) for j in range(1, 5)]]:
    Pl = env[j]["horn"] if j else max(env[0]["horn"], env[0]["idler"])
    Pst = st_max.get(j, Pl)
    brg.append(dict(joint=j, bearing=nm, C_N=C, C0_N=C0, worst_static_N=round(Pl, 1), static_sf=round(C0 / max(Pl, 1e-6), 1),
                    stance_load_N=round(Pst, 1), L10_oscillations_15deg=float(f"{l10_osc(C, Pst, 15):.2e}"), L10_oscillations_45deg=float(f"{l10_osc(C, Pst, 45):.2e}")))
out["bearings"] = brg
out["bearing_note"] = ("Oscillating L10 = (C/P)^3 x 1e6 x 360 / (4 psi). Small swings under 10 deg do not roll the balls far enough to spread grease: "
                       "expect false brinelling marks at the hover hold angles; a sweep through the full range now and then (a 'grease cycle' at power-up) helps.")
json.dump(out, open(os.path.join(S.HERE, "results", f"struct_{TAG}.json"), "w"), indent=1, default=float)
print(json.dumps({k: out[k] for k in ("sag_mm", "tip_stiffness_N_per_mm_straight", "modes_Hz", "gear", "bearings")}, indent=0, default=float)[:3500])
for r in out["stress"]: print(r["joint"], r["min_sf"], r["min_sf_holdable"], r["weakest"], r["horn_ear_MPa"], r["idler_ear_MPa"], r["pin_bend_MPa"])
