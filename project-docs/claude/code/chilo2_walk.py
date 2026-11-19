"""Chilo-1 rev 2 walk: stand -> straight walk -> stop, with loads, estimated currents, reach and joint-range monitors.
python3 chilo2_walk.py [assist]   assist = two free hands spinning as props, 40 N up on the octopus (est.)
Gait: body legs in a head->tail wave (duty 0.75, left/right antiphase); the four arm-feet step one at a time
(low L, side R, low R, side L... 4-beat, duty 0.8), all on one clock.  Open-loop foot targets, stall-limited servos.
-> chilo2_walk[_assist].npz / .json"""
import sys, json
import numpy as np
import mujoco
import chilo2 as C, chilo_arm_legs as A

ASSIST = "assist" in sys.argv[1:] or "assist4" in sys.argv[1:]
LIFT_N = 80.0 if "assist4" in sys.argv[1:] else (40.0 if ASSIST else 0.0)
TAG = {0.0: "", 40.0: "_assist", 80.0: "_assist4"}[LIFT_N]
FPS = 30; CTRL_DT = 0.005
T, STRIDE, LIFT_LEG, LIFT_ARM = 2.4, 0.060, 0.035, 0.040
DUTY_LEG, DUTY_ARM = 0.75, 0.80
T_STAND, T_RAMP, T_STOP, T_END = 1.5, 2.4, 13.0, 17.0
ARM_PHASE = {0: 0.0, 6: 0.25, 7: 0.5, 1: 0.75}          # low L, side R, low R, side L
LAG = -1 / 5
import os
BAL_K = float(os.environ.get("BAL_K", 0.06))                 # m/s per unit of stall-fraction imbalance
PRESS = {0: float(os.environ.get("PRESS_LOW", 0.010)), 1: float(os.environ.get("PRESS_SIDE", 0.012))}   # arm-foot preload: target below the floor (m)


def mj(x):
    x = np.clip(x, 0, 1); return x * x * x * (10 - 15 * x + 6 * x * x)


def amp(t):
    return mj((t - T_STAND) / T_RAMP) * (1 - mj((t - T_STOP) / T_RAMP))


def phase_name(t):
    if t < T_STAND: return "STAND"
    if t < T_STAND + T_RAMP: return "STARTING"
    if t < T_STOP: return "WALK: body-leg wave + 4-beat arm steps"
    if t < T_STOP + T_RAMP: return "SLOWING"
    return "STAND"


class Feet:
    """Stance feet integrated backwards in their own frame; swing = min-jerk from lift-off to nominal touchdown."""
    def __init__(self, home, duty, lift, offset):
        self.home, self.duty, self.lift, self.off = home.copy(), duty, lift, offset
        self.p = home.copy(); self.lo = home.copy(); self.sw = False

    def step(self, clock, a, dadv, along):
        ph = (clock + self.off) % 1.0
        if ph < self.duty or a <= 1e-3:
            self.sw = False; self.p = self.p - along * dadv; self.p[2] = self.home[2]
        else:
            if not self.sw: self.lo = self.p.copy(); self.sw = True
            u = (ph - self.duty) / (1 - self.duty); h = mj(u)
            td = self.home + along * a * STRIDE / 2
            self.p = self.lo + (td - self.lo) * h; self.p[2] = self.home[2] + a * self.lift * 16 * u * u * (1 - u) ** 2
        return self.p, self.sw


def run():
    m = C.build(); d = mujoco.MjData(m)
    names = [m.actuator(a).name[2:] for a in range(m.nu)]
    jadr = np.array([m.joint(n).qposadr[0] for n in names])
    sv = C.servo_of_actuator(m)
    stall = np.array([C.SV[s][1] for s in sv]); rated = np.array([C.SV[s][0] for s in sv]); istall = np.array([C.SV[s][4] for s in sv])
    idx = {n: k for k, n in enumerate(names)}
    # ---------------- nominal poses
    sol = {0: json.load(open("arm_leg_0_24.json")), 1: json.load(open("arm_leg_1_36.json"))}
    q_nom, roots = {}, {}
    for i in C.ARM_LEGS:
        q = np.radians(sol[min(i, 7 - i)]["q_deg"])
        if i >= 4: q = q * np.array([1, -1] * 4)
        q_nom[i] = q; roots[i] = A.arm_frame(i, C.PLATE)
    arm_home = {i: A.fk(q_nom[i], *roots[i])[3] for i in C.ARM_LEGS}
    for i in C.ARM_LEGS: arm_home[i][2] = 0.02
    press = {i: PRESS[min(i, 7 - i)] for i in C.ARM_LEGS}; lf = {i: 0.0 for i in C.ARM_LEGS}; lsp = 0.0
    arm_ix = {i: [idx[f"arm{i}_q{k}"] for k in range(8)] for i in C.ARM_LEGS}; sp_ix = [idx["sp1"], idx["sp2"]]
    q_arm = {i: q_nom[i].copy() for i in C.ARM_LEGS}
    for i in C.ARM_LEGS:
        q_arm[i] = C.arm_ik(arm_home[i], q_arm[i], *roots[i], q_nom[i], iters=30)
    legs = [(seg, s) for seg in range(1, 6) for s in (1, -1)]
    leg_home = np.array([0.0, C.LEG_PTS[-1][0], C.LEG_PTS[-1][1]])
    def leg_q(p, qprev, s, iters=6):
        q = C.leg_ik(p, qprev * np.array([-s, 1, 1, 1, 1]), iters)      # leg_ik works in forward/outward/up coordinates
        q[0] *= -s                                                       # coxa yaw sign: left legs turn forward with -yaw
        return q
    q_leg = {L: leg_q(leg_home, np.zeros(5), L[1], 30) for L in legs}
    feet = {}
    for i in C.ARM_LEGS:
        feet[("arm", i)] = Feet(arm_home[i], DUTY_ARM, LIFT_ARM, ARM_PHASE[i])
    for seg, s in legs:
        feet[("leg", seg, s)] = Feet(leg_home, DUTY_LEG, LIFT_LEG, ((seg - 1) * LAG + (0.0 if s > 0 else 0.5)) % 1.0)

    def ctrl_vector():
        u = np.zeros(m.nu)
        for i in C.ARM_LEGS:
            for k in range(8): u[idx[f"arm{i}_q{k}"]] = q_arm[i][k]
        for seg, s in legs:
            n = f"{'L' if s > 0 else 'R'}{seg}"
            for k in range(5): u[idx[f"{n}_q{k}"]] = q_leg[(seg, s)][k]
        return u

    u = ctrl_vector()
    d.qpos[jadr] = u; d.qpos[2] = C.HC + 0.004
    mujoco.mj_forward(m, d)
    octo = m.body("octo").id
    fgeoms = [m.geom(f"arm{i}_foot").id for i in C.ARM_LEGS] + [m.geom(f"{'L' if s > 0 else 'R'}{seg}_foot").id for seg, s in legs]
    seg_b = [m.body(f"seg{i}").id for i in range(6)]
    n_steps = int(T_END / C.DT); every = int(1 / (FPS * C.DT)); cevery = int(CTRL_DT / C.DT)
    clock = 0.0
    F = np.zeros((n_steps, m.nu), np.float32); Itot = np.zeros(n_steps, np.float32)
    rec = dict(qpos=[], t=[], load=[], down=[], itot=[], com=[], sw=[])
    reach_max = {i: 0.0 for i in C.ARM_LEGS}; jr_max = {i: 0.0 for i in C.ARM_LEGS}; ik_err = {i: 0.0 for i in C.ARM_LEGS}
    c6 = np.zeros(6)
    for k in range(n_steps):
        t = k * C.DT
        if k % cevery == 0:
            a = amp(t); dt = CTRL_DT
            if a > 0: clock += dt / T
            dadv = a * STRIDE / (DUTY_LEG * T) * dt
            sw_all = []
            # load balancer: equalise the worst-joint stall fraction of the planted arms and the head spine joints
            fr = np.abs(d.actuator_force) / stall
            for i in C.ARM_LEGS: lf[i] += 0.05 * (fr[arm_ix[i]].max() - lf[i])
            lsp += 0.05 * (fr[sp_ix].max() - lsp)
            planted = [i for i in C.ARM_LEGS if not feet[("arm", i)].sw]
            if planted:
                mean = np.mean([lf[i] for i in planted])
                pitch = 2 * np.arcsin(np.clip(d.qpos[5], -1, 1))          # head nose-down > 0
                for i in planted:
                    press[i] = float(np.clip(press[i] + dt * (BAL_K * (mean - lf[i]) + 0.25 * pitch), -0.01, 0.04))
            for i in C.ARM_LEGS:
                p, sw = feet[("arm", i)].step(clock, a, a * STRIDE / (DUTY_ARM * T) * dt, np.array([1.0, 0, 0])); sw_all.append(sw)
                tgt = p.copy(); tgt[2] -= press[i] if not sw else 0.0
                q_arm[i] = C.arm_ik(tgt, q_arm[i], *roots[i], q_nom[i])
                if a > 0.05: ik_err[i] = max(ik_err[i], float(np.linalg.norm(A.fk(q_arm[i], *roots[i])[3] - tgt)))
                root = roots[i][0]
                reach_max[i] = max(reach_max[i], float(np.linalg.norm(A.fk(q_arm[i], *roots[i])[3] - root) / A.L_FULL))
                jr_max[i] = max(jr_max[i], float(np.max(np.where(q_arm[i] >= 0, q_arm[i] / A.LIM[:, 1], q_arm[i] / A.LIM[:, 0]))))
            for seg, s in legs:
                p, sw = feet[("leg", seg, s)].step(clock, a, dadv, np.array([1.0, 0, 0])); sw_all.append(sw)
                q_leg[(seg, s)] = leg_q(p, q_leg[(seg, s)], s)
            u = ctrl_vector()
        d.ctrl[:] = u
        d.xfrc_applied[octo, 2] = LIFT_N
        mujoco.mj_step(m, d)
        F[k] = d.actuator_force
        Itot[k] = np.sum(C.I_IDLE + istall * np.abs(d.actuator_force) / stall)
        if k % every == 0:
            down = np.zeros(len(fgeoms), bool)
            for ci in range(d.ncon):
                con = d.contact[ci]
                for g in (con.geom1, con.geom2):
                    if g in fgeoms:
                        mujoco.mj_contactForce(m, d, ci, c6)
                        if c6[0] > 0.5: down[fgeoms.index(g)] = True
            rec["qpos"].append(d.qpos.copy()); rec["t"].append(t); rec["load"].append(np.abs(d.actuator_force) / stall)
            rec["down"].append(down); rec["itot"].append(Itot[k]); rec["com"].append(d.subtree_com[1].copy()); rec["sw"].append(sw_all)
    for k_ in rec: rec[k_] = np.array(rec[k_])
    np.savez_compressed(f"chilo2_walk{TAG}.npz", **rec, names=np.array(names), sv=np.array(sv), assist=ASSIST)

    # ---------------- metrics
    walk = slice(int(T_STAND / C.DT), int((T_STOP + T_RAMP) / C.DT)); stand = slice(int(0.8 / C.DT), int(T_STAND / C.DT))
    groups = {"arm j0-j1 (STS3250)": lambda n: n.startswith("arm") and n[-1] in "01",
              "arm j2-j7 (STS3215)": lambda n: n.startswith("arm") and n[-1] not in "01",
              "leg coxa yaw (STS3215)": lambda n: n[0] in "LR" and n.endswith("_q0"),
              "leg p1-p2 (STS3250)": lambda n: n[0] in "LR" and n[-1] in "12",
              "leg p3-p4 (STS3215)": lambda n: n[0] in "LR" and n[-1] in "34",
              "spine yaw (STS3215)": lambda n: n.startswith("sy"), "spine pitch (STS3250)": lambda n: n.startswith("sp")}
    out = dict(assist=ASSIST, prop_lift_N=LIFT_N, mass_kg=float(m.body_subtreemass[1]),
               gait=dict(T=T, stride_mm=STRIDE * 1e3, speed_cm_s=round(STRIDE / (DUTY_LEG * T) * 100, 2), duty_leg=DUTY_LEG, duty_arm=DUTY_ARM), groups={})
    for g, f in groups.items():
        ix = [k for k, n in enumerate(names) if f(n)]
        L = np.abs(F[walk][:, ix]) / stall[ix]; Ls = np.abs(F[stand][:, ix]) / stall[ix]
        worst = ix[int(np.argmax(L.max(0)))]
        out["groups"][g] = dict(walk_peak_x_stall=round(float(L.max()), 2), walk_p95_x_stall=round(float(np.percentile(L, 95)), 2),
                                walk_rms_x_rated=round(float((np.sqrt((F[walk][:, ix] ** 2).mean(0)) / rated[ix]).max()), 2),
                                stand_x_rated=round(float((np.abs(F[stand][:, ix]).mean(0) / rated[ix]).max()), 2), worst_joint=names[worst])
    out["bus_current_A"] = dict(stand_mean=round(float(Itot[stand].mean()), 1), walk_mean=round(float(Itot[walk].mean()), 1), walk_peak=round(float(Itot[walk].max()), 1),
                                note="12 V servo bus; I = 0.12 A idle + stall current x torque/stall per servo (est.)")
    out["bus_power_W_walk_mean"] = round(float(Itot[walk].mean() * 12), 0)
    com = rec["com"]; i0, i1 = int(T_STAND * FPS), len(com) - 1
    asked = sum(STRIDE / (DUTY_LEG * T) * amp(k * CTRL_DT) * CTRL_DT for k in range(int(T_END / CTRL_DT)))
    out["distance_m"] = round(float(com[i1, 0] - com[i0, 0]), 3); out["distance_asked_m"] = round(float(asked), 3)
    out["side_drift_mm"] = round(float(abs(com[i1, 1] - com[i0, 1]) * 1e3), 1)
    q = rec["qpos"]
    tilt = [np.degrees(np.arccos(np.clip(1 - 2 * (qq[4] ** 2 + qq[5] ** 2), -1, 1))) for qq in q]
    out["head_tilt_max_deg"] = round(float(max(tilt)), 2)
    out["head_height_mm"] = [round(float(q[:, 2].min() * 1e3), 1), round(float(q[:, 2].max() * 1e3), 1)]
    out["feet_down"] = dict(min=int(rec["down"][i0:].sum(1).min()), mean=round(float(rec["down"][i0:].sum(1).mean()), 1),
                            arm_feet_min=int(rec["down"][i0:, :4].sum(1).min()))
    out["arm_reach_max"] = {str(i): round(v, 3) for i, v in reach_max.items()}
    out["arm_joint_range_max"] = {str(i): round(v, 3) for i, v in jr_max.items()}
    out["arm_ik_error_max_mm"] = {str(i): round(v * 1e3, 1) for i, v in ik_err.items()}
    pk = {}
    for g, f in groups.items():
        ix = [k for k, n in enumerate(names) if f(n)]
        L = np.abs(F[walk][:, ix]) / stall[ix]; kk = int(np.argmax(L.max(1)))
        pk[g] = round((walk.start + kk) * C.DT, 3)
    out["peak_time_s"] = pk
    json.dump(out, open(f"chilo2_walk{TAG}.json", "w"), indent=1)
    print(json.dumps(out, indent=1))


if __name__ == "__main__":
    run()
