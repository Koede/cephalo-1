"""Sections 8-10: efficiencies, thermal, resilience, sensitivity. -> results/system.json"""
import sys, os, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, "/mnt/project-files/octopus-drone/joint-study/sim")
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
import numpy as np
from scipy.optimize import linprog
import spec_lib as S, poses as PS
import claw_fans as CF, central_prop as CP

TAG = "alt90"; A = S.Arm(TAG)
L = json.load(open(os.path.join(S.HERE, "results", f"loads_{TAG}.json")))
P = dict(np.load(os.path.join(S.HERE, "results", f"poses_{TAG}.npz")))
ST = json.load(open(os.path.join(S.HERE, "results", f"struct_{TAG}.json")))
FAN = CF.FANS["MN5008 KV400 + 16x5.4 CF"]
FH, FR, FF = L["fan"]["hover_N"], L["fan"]["rated_N"], L["fan"]["full_N"]; FOOT = L["foot_N"]; ROBOT = L["robot_kg"]
qh = P[f"{TAG}_flight_hover"]; fh = A.fk(qh)
need_h, tau_h, *_ = A.need(qh, FH * fh["tipR"][:, 2])
out = {}
V = 12.0; V_PACK = CP.V_LOADED
def servo_par(j):
    s = S.SERVO[S.servo_name(j)]; R = V / s["I_stall"]; kt = s["stall"] / s["I_stall"]
    return R, kt
# ---------------- efficiencies ----------------
eff = {}
Pfan = CF.fan_P(FAN, FH); Tc = L["fan"]["central_N"]
eff["fan_hover"] = dict(thrust_N=FH, electrical_W=round(Pfan, 1), g_per_W=round(FH / S.G * 1000 / Pfan, 2))
# ideal (momentum) power and figure of merit: 16 in disc
Adisc = math.pi * (16 * 0.0254 / 2) ** 2
Pid = FH ** 1.5 / math.sqrt(2 * 1.225 * Adisc)
eff["fan_hover"]["ideal_power_W"] = round(Pid, 1); eff["fan_hover"]["figure_of_merit_incl_motor_esc"] = round(Pid / Pfan, 2)
for nm, T in (("rated", FR), ("full", FF)):
    p = CF.fan_P(FAN, T); eff[f"fan_{nm}"] = dict(thrust_N=round(T, 1), electrical_W=round(p), g_per_W=round(T / S.G * 1000 / p, 2))
# disc loading and download: thrust axis through the arm? the prop sits 102 mm beyond the tip, folded pod blocks part of the disc [A 3 %]
eff["gear_stage"] = dict(mesh=0.99, bearings=0.99, total=0.98, note="lubricated m1 spur pair, two ball bearings [A]")
# servo holding power in the hover pose: I^2 R (PWM drive), plus 0.12 A idle electronics at 12 V [A]
def heat(tau):
    hold = []
    for j in range(S.N):
        R, kt = servo_par(j)
        t_servo = abs(tau[j]) / (2 * 0.98) if j == 0 else abs(tau[j])
        I = t_servo / kt; Pw = I * I * R + 0.12 * V
        hold.append(dict(joint=j, torque_Nm=round(float(t_servo), 2), current_A=round(I, 2), heat_W=round(Pw, 2)))
    return hold
hold = heat(tau_h)
Parm = sum(h["heat_W"] for h in hold)
qL = P.get(f"{TAG}_lean20_hover"); fL = A.fk(qL) if qL is not None else None
holdL = heat(A.need(qL, FH * fL["tipR"][:, 2])[1]) if qL is not None else []
ParmL = sum(h["heat_W"] for h in holdL)
eff["servos_hover"] = dict(per_joint=hold, arm_W=round(Parm, 1), robot_W=round(8 * Parm), share_of_hover_power=round(8 * Parm / L["fan"]["hover_total_W"], 3),
                           note="a position-holding servo turns all its input into heat; current from torque / (stall torque / stall current), copper loss I^2 R")
# wiring: 16 AWG pair, 1.3 m each way, 13.2 mOhm/m
Rw = 2 * 1.3 * 0.0132
for nm, T in (("hover", FH), ("full", FF)):
    I = CF.fan_P(FAN, T) / V_PACK
    eff[f"fan_wiring_{nm}"] = dict(current_A=round(I, 1), loss_W=round(I * I * Rw, 1), drop_V=round(I * Rw, 2), share=round(I * I * Rw / CF.fan_P(FAN, T), 3))
eff["servos_hover_lean20"] = dict(per_joint=holdL, arm_W=round(ParmL, 1), robot_W=round(8 * ParmL), share_of_hover_power=round(8 * ParmL / L["fan"]["hover_total_W"], 3),
                                   extra_fan_power_W_per_arm=round(CF.fan_P(FAN, FH / math.cos(math.radians(20))) - Pfan, 1))
eff["arm_power_chain_hover"] = dict(fan_W=round(Pfan), servo_W=round(Parm, 1), wiring_W=eff["fan_wiring_hover"]["loss_W"],
                                     thrust_per_W_whole_arm=round(FH / (Pfan + Parm + eff["fan_wiring_hover"]["loss_W"]), 4))
out["efficiency"] = eff
# ---------------- thermal ----------------
th = []
for nm in ("STS3250", "STS3215"):
    s = S.SERVO[nm]; R = V / s["I_stall"]; kt = s["stall"] / s["I_stall"]
    Pcont = 3.0; I = math.sqrt((Pcont - 0.12 * V) / R); tc = kt * I
    Cth = s["g"] / 1000 * 1000; Rth = 50 / Pcont      # J/K (case, motor, gears ~ 1 J/gK lumped) [A]; 50 K rise allowed
    th.append(dict(servo=nm, R_ohm=round(R, 2), kt_Nm_per_A=round(kt, 3), continuous_torque_Nm=round(tc, 2), continuous_share_of_stall=round(tc / s["stall"], 2),
                   thermal_time_constant_s=round(Cth * Rth)))
out["thermal_servo_limits"] = th
def therm(hold):
  tt = []
  for h in hold:
    nm = S.servo_name(h["joint"]); s = S.SERVO[nm]; C = s["g"]; Rt = 50 / 3.0
    Pw = h["heat_W"]; dTinf = Pw * Rt
    t50 = float("inf") if dTinf <= 50 else -C * Rt * math.log(1 - 50 / dTinf)
    tt.append(dict(joint=h["joint"], heat_W=Pw, steady_rise_K=round(dTinf), minutes_to_50K=("never" if t50 == float("inf") else round(t50 / 60, 1))))
  return tt
out["thermal_hover"] = therm(hold)
out["thermal_hover_lean20"] = therm(holdL)
out["thermal_note"] = ("About 3 W of steady heat per servo case in still air [A]; in the prop wash it is better. 50 K rise from 25 C ambient keeps the case near 75 C, "
                       "under the 80 C the makers' overheat cut-outs typically sit at [A]. The flight is about 17 minutes.")
Pm_loss = Pfan * (1 - 0.82)
out["thermal_fan_motor"] = dict(hover_heat_W=round(Pm_loss), note="MN5008 about 82 % efficient with its ESC at hover [A]; in its own prop wash the motor runs warm, not hot")
# ---------------- resilience ----------------
res = {}
# 1) drop onto the folded pods: per-leg vertical stiffness in the stance pose, impact force vs height
stk = [k for k in P if "_stance_" in k]
SIDES = {"+x": [1, 0, 0], "-x": [-1, 0, 0], "+y": [0, 1, 0], "-y": [0, -1, 0]}
sk = sorted(stk, key=lambda k: ("mid" not in k, k))[0] if stk else None
qs = P.get(sk) if sk else None; sd = SIDES[sk.split("_")[2]] if sk else [1, 0, 0]
if qs is not None:
    fs = A.fk(qs); fp = PS.foot_point(A, fs, sd)
    def foot_need(Fz): return A.need(qs, np.zeros(3), "tip", [(fp, np.array([0, 0, Fz]))])[0].max()
    Kser = np.array([4.9 / math.radians(2) * 4 * 0.98] + [S.SERVO[S.servo_name(j)]["stall"] / math.radians(2) for j in range(1, S.N)])
    # foot compliance from joint springs: Jacobian of the foot point
    J = np.zeros((3, S.N))
    for j in range(S.N): J[:, j] = np.cross(fs["a"][j], fp - fs["o"][j])
    Cf = J @ np.diag(1 / Kser) @ J.T; kz = 1 / Cf[2, 2]
    rows = []
    for h in (0.05, 0.10, 0.20, 0.30, 0.5, 1.0, 2.0):
        E = ROBOT * S.G * h / 7; F = FOOT + math.sqrt(2 * E * kz)
        rows.append(dict(drop_m=h, impact_s=round(math.sqrt(2 * h / S.G), 2), peak_foot_N=round(F), worst_need=round(foot_need(F), 2)))
    hb = max([r["drop_m"] for r in rows if r["worst_need"] <= 1.5] or [0])
    res["drop"] = dict(stance_pose=sk, foot_vertical_stiffness_N_per_mm=round(kz / 1000, 2), rows=rows,
                       note=f"Seven feet share the landing, servo springs only (no pad). Above 1.5 x stall the servo gear train back-drives or strips [A]; "
                            f"the last listed height under that is {hb} m. Land under power: let thrust carry the weight to touchdown, then hand it over.")
# 2) crash and ground strike: tip force at which each limit is reached, arm straight out
q0 = qL if qL is not None else qh; f0 = A.fk(q0); Fb = FH * f0["tipR"][:, 2]
crash = {}
for nm, dvec in (("down (landing on the fan)", [0, 0, -1]), ("up", [0, 0, 1]), ("sideways", [0, 1, 0]), ("inward, toward the body", [-1, 0, 0]), ("outward", [1, 0, 0])):
    d = np.array(dvec, float)
    def worst(Fm): return A.need(q0, Fb + Fm * d, "tip")[0]
    lim = {}
    for lab, k in (("servo stall", 1.0), ("back-drive at 1.5x stall [A]", 1.5)):
        lo, hi = 0.0, 2000.0
        if worst(hi).max() <= k: lim[lab] = ">2000"; continue
        for _ in range(40):
            m = (lo + hi) / 2
            if worst(m).max() <= k: lo = m
            else: hi = m
        lim[lab] = round(lo, 1); lim[lab + " joint"] = int(np.argmax(worst(hi)))
    # ear failure: scale the support reactions linearly
    Ms, Fs, f = A.wrench(q0, Fb + 100.0 * d, Fpt="tip"); Ms0, Fs0, _ = A.wrench(q0, Fb, Fpt="tip")
    best = (1e9, None)
    for j in range(S.N):
        R = A.joint_frame(f, j)
        for MsX, FsX, sgn in ((Ms, Fs, 1),):
            dM = R @ (Ms[j] - Ms0[j]) / 100.0; dF = R @ (Fs[j] - Fs0[j]) / 100.0
            M = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, -0.0245, 0, 0.0245], [0.0245, 0, -0.0245, 0]], float)
            s = np.linalg.solve(M, [-dF[1], -dF[2], -dM[1], -dM[2]])
            for Rr, t in ((s[0:2], 4.0), (s[2:4], 3.2)):
                sp = 6 * abs(Rr[0]) * 20.8 / (t * 21 ** 2) + abs(Rr[1]) / (t * 21)
                if sp > 0:
                    Fbreak = 25.0 / sp
                    if Fbreak < best[0]: best = (Fbreak, j)
    lim["PETG ear breaks (25 MPa across layers)"] = round(best[0], 1); lim["ear joint"] = best[1]
    crash[nm] = lim
res["tip_force_limits_hover"] = crash
res["tip_force_note"] = ("Extra static force at the tip plate on top of hover thrust, arm in the leaning hover pose, servos holding. A real hit is dynamic: the servos give way first "
                         "when the hit is along a hinge axis, but an end-on hit loads only the ears and pins. Gravity is included.")
# 3) one claw fan out: can the other seven and the central prop hover and trim?
rtip = float(np.hypot(*fh["tip"][:2]) + 0.102 * 0)   # fan radius in hover, metres from the robot centre
ang = np.arange(8) * math.pi / 4
W = ROBOT * S.G
def lp(dead):
    idx = [i for i in range(8) if i != dead]
    # vars: fan thrusts (7), central, t (max fan share)
    n = len(idx); c = np.zeros(n + 2); c[-1] = 1
    Aeq = np.zeros((3, n + 2)); beq = np.array([W, 0, 0])
    for k, i in enumerate(idx):
        Aeq[0, k] = 1; Aeq[1, k] = rtip * math.cos(ang[i]); Aeq[2, k] = rtip * math.sin(ang[i])
    Aeq[0, n] = 1 - 0.036
    Aub = np.zeros((n, n + 2)); bub = np.zeros(n)
    for k in range(n): Aub[k, k] = 1; Aub[k, -1] = -FF
    bounds = [(0, FF)] * n + [(0, 0.9 * CP.P17_MAX_N), (0, 2)]
    r = linprog(c, A_ub=Aub, b_ub=bub, A_eq=Aeq, b_eq=beq, bounds=bounds)
    return r
r = lp(0)
if r.success:
    n = 7; th_ = r.x[:n]
    res["one_fan_out"] = dict(feasible=True, worst_fan_share_of_full=round(float(r.x[-1]), 2), central_N=round(float(r.x[n])), fans_N=[round(float(x), 1) for x in th_],
                              fan_radius_m=round(rtip, 3),
                              note="The opposite fan drops and its two neighbours of the dead fan carry more; yaw from the missing prop's torque (about 0.5 N.m) "
                                   "needs only a fraction of a degree of sideways tilt on the other fans.")
else:
    res["one_fan_out"] = dict(feasible=False)
# central prop out: eight fans alone
need_fan = W / 8
res["central_prop_out"] = dict(fan_N_needed=round(need_fan, 1), fan_full_N=round(FF, 1), share_of_full=round(need_fan / FF, 2), feasible=bool(need_fan <= FF),
                               power_W=round(8 * CF.fan_P(FAN, need_fan)))
# 4) limp servo in flight: can the arm find a pose with zero torque on the dead joint and hover thrust, others within stall?
limp = []
for j in (0, 1, 2, 4, 6, 8, 10, 12):
    def load(q, f): return (FH * f["tipR"][:, 2], "thrust", None)
    def cons(q, f, j=j):
        need, tau, *_ = A.need(q, FH * f["tipR"][:, 2])
        z = f["tipR"][:, 2]
        return [tau[j] / S.CAP[j]], [float(z[2]) - math.cos(math.radians(20)), S.self_clear(f), float(np.hypot(*f["tip"][:2])) - 0.6]
    base = (qL if qL is not None else qh).copy()
    rr = PS.solve(A, load, cons, PS.seeds(5, base=base))
    limp.append(dict(joint=j, hover_holdable=bool(rr["ok"] and rr["worst"] <= 1.0), worst_other=round(rr["worst"], 2), tip_tilt_limit_deg=20))
res["limp_joint_in_hover"] = limp
res["limp_note"] = ("A dead servo leaves its joint free (no brake). The other joints move so that thrust and gravity put no torque on the free joint, "
                    "with the fan still within 20 deg of up and 0.6 m out. If that fails the arm should cut its fan and fold (neutral thrust).")
# 5) brownout: servo stall scales with supply voltage
wn = float(need_h.max()); wnL = float(A.need(qL, FH * fL["tipR"][:, 2])[0].max()) if qL is not None else wn
res["brownout"] = dict(hover_worst_need=round(wn, 3), min_bus_V_to_hold_hover=round(12 * wn, 1), min_bus_V_at_2x_rule=round(24 * wn, 1),
                       lean20_min_bus_V=round(12 * wnL, 1), lean20_min_bus_V_2x=round(24 * wnL, 1),
                       servo_bus_current_hover_A_per_arm=round(sum(h["current_A"] * (h["current_A"] * servo_par(h["joint"])[0]) / 12 for h in hold) + 13 * 0.12, 2),
                       note="Stall torque falls in proportion to voltage. The STS servos cut out near 4 to 5 V by default [A]. Feed each arm's 12 V bus from its own "
                            "buck regulator (about 10 A peak per arm) so one arm's stall cannot pull the others down; the pack sags from 25 V to about 21 V under hover load, "
                            "which a buck converter absorbs.")
# 6) back-drive and overload settings
res["overload"] = dict(note="Feetech STS servos have torque limit, protection current and overload protection time registers. In flight a servo that trips "
                            "goes limp, which is worse than running hot, so for joints 0 to 4 set the overload torque to 100 % and the protection time to its "
                            "maximum, and let the flight controller's own current watch decide. For walking, a 80 % torque limit makes the arm give way "
                            "before the gears strip. Metal gear trains in this size back-drive around 1.5 x stall [A]; above that teeth chip.")
out["resilience"] = res
# ---------------- sensitivity ----------------
sens = {}
def hover_worst(Ab):
    def load(q, f): return (FH * f["tipR"][:, 2], "thrust", None)
    def cons(q, f):
        z = f["tipR"][:, 2]; tip = f["tip"]
        return [], [float(z[2]) - math.cos(math.radians(20)), float(np.hypot(tip[0], tip[1])) - 0.60, S.self_clear(f)]
    rng = np.random.default_rng(2)
    return PS.solve(Ab, load, cons, [qL if qL is not None else qh, qh] + [rng.uniform(-1, 1, S.N) * PS.LIM for _ in range(6)])
rows = []
for tg in (150, 200, 252.5, 300, 350, 400):
    Ab = S.Arm(TAG, tip_g=tg)
    hw = hover_worst(Ab); st = Ab.need(np.zeros(S.N))[0]
    rows.append(dict(tip_g=tg, hover_worst=round(hw["worst"], 3), hover_ok=hw["ok"], straight_out_worst=round(float(st.max()), 3),
                     straight_out_worst_joint=int(np.argmax(st)), arm_mass_g=round(Ab.mass.sum() * 1000 + tg + 131.4, 1)))
sens["tip_mass"] = rows
rows = []
for hr in (0.110, 0.1245, 0.140, 0.160):
    Ab = S.Arm(TAG, hub_r=hr)
    hw = hover_worst(Ab)
    rows.append(dict(hub_radius_m=hr, hover_worst=round(hw["worst"], 3), hover_ok=hw["ok"]))
sens["hub_radius"] = rows
sens["note"] = ("Hover here is the leaning (20 deg) hover pose, re-solved for each case. Claw mass: the tip pod mass stands in for it (the claw sits in the pod). Mantle radius: changes the hub radius where the roots sit, which "
                "moves the 0.6 m fan circle closer to or further from the root, and the walking foot radius in the same way.")
out["sensitivity"] = sens
json.dump(out, open(os.path.join(S.HERE, "results", "system.json"), "w"), indent=1, default=float)
print(json.dumps(out, indent=0, default=float)[:6000])
