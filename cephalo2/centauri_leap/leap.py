#!/usr/bin/env python3
"""Centauri-1 extended bounding leap + landing, simulated in MuJoCo on the rev C legs (build_model.py).

One leap = stand -> gather (sit back) -> push (fores lift the nose, hinds drive) -> flight: fores swing to FULL forward extension
and hinds to FULL backward extension at the same moment -> fores reach for the ground -> fore touchdown (fores land first,
compliant) -> hinds come under -> hind touchdown -> back to the stand, at rest.
Control: joint PD on the flex and pivot joints of each pair, torque clipped to the motor envelope at 39.6 V (sim_core.Motors);
phase switches on real foot contact events, not on the clock.  The optimiser (CMA-ES) picks the poses, timings and landing
stiffness; the extension poses are fixed by the joint ranges:
  fore 'full forward'  q0 57, q1 52  (pivot 3 deg inside its 60 stop, flex 2 deg off its 50 stop): 486 mm reach, 31 deg forward
  hind 'full back'     q0 -6, q1 52  (pivot 3 deg inside its -9 stop):                              392 mm reach, 34 deg back
Monitored every step: joint torque and speed against the motor envelope, phase and bus current, tendon (elbow / hock) tension,
gas strut stroke, joint-stop forces, hoof ground forces and slip, every body-to-floor contact, leg-to-leg distance (math).
    python3 leap.py search [n_iter] [seed]   -> best_params.json
    python3 leap.py run                      -> leap_result.json, leap_trajectory.csv, replay frames"""
import os, sys, json, time
import numpy as np
import mujoco
import sim_core as S

HERE = S.HERE; D = S.D; DT_REC = 0.005
STAND = {k: (S.INFO["legs"][k]["stand"]["q0"], S.INFO["legs"][k]["stand"]["q1"]) for k in S.KINDS}
EXT = {"fore": (57.0, 52.0), "hind": (-6.0, 52.0)}
EXT_TOL = 3.0                     # deg: both joints of both pairs within this of the extension pose, at the same instant, in the air
RANGE = {k: S.INFO["legs"][k]["range"] for k in S.KINDS}
KP = {"fore": (600.0, 600.0), "hind": (400.0, 400.0)}     # N m / rad (q0, q1): effectively torque-limited bang to the target
KD = {"fore": (8.0, 6.0), "hind": (5.0, 5.0)}
T_END = 3.2; T_SETTLE0 = 0.25
DYNEEMA_PAIR_N = 2 * 8000.0      # 3 mm Dyneema SK78, ~8 kN break each (est. from typical 3 mm listings), parallel pair

# parameter vector: name, seed, lo, hi
PSPEC = [
    ("Tg", 0.35, 0.15, 0.8),        # gather time
    ("fg0", 22.0, 0.0, 50.0), ("fg1", 100.0, 60.0, 135.0),       # fore gather pose
    ("hg0", 42.0, 10.0, 62.0), ("hg1", 112.0, 70.0, 140.0),      # hind gather pose (crouched under)
    ("fp0", 40.0, 0.0, 60.0), ("fp1", 55.0, 50.0, 110.0),        # fore push target
    ("dh", 0.06, 0.0, 0.25),        # hind push delay after fore push starts
    ("hp0", 5.0, -9.0, 40.0), ("hp1", 52.0, 50.0, 100.0),        # hind push target
    ("tlp", 0.10, 0.0, 0.4),        # after both are airborne: fores leave the extension pose for the landing pose
    ("fl0", 50.0, 20.0, 60.0), ("fl1", 60.0, 50.0, 110.0),       # fore landing pose
    ("ths", 0.08, -0.1, 0.4),       # hinds start to come under, relative to fore touchdown (negative = before)
    ("hl0", 40.0, 0.0, 66.0), ("hl1", 90.0, 50.0, 140.0),        # hind landing pose
    ("kl_f", 0.35, 0.05, 1.0), ("dl_f", 1.5, 0.3, 4.0),          # fore landing stiffness / damping scale
    ("kl_h", 0.35, 0.05, 1.0), ("dl_h", 1.5, 0.3, 4.0),
    ("fa0", 35.0, 10.0, 60.0), ("fa1", 95.0, 60.0, 135.0),       # fore absorb target after touchdown
    ("ha0", 35.0, 0.0, 62.0), ("ha1", 95.0, 60.0, 140.0),        # hind absorb target
    ("kth", 0.6, 0.0, 2.0), ("kthd", 0.08, 0.0, 0.3),            # pitch feedback on the fore pivot while landing (deg/deg, deg/(deg/s))
    ("tst", 0.35, 0.1, 0.8),        # after hind touchdown: time to walk the targets back to the stand
]
PN = [p[0] for p in PSPEC]; P0 = np.array([p[1] for p in PSPEC]); LO = np.array([p[2] for p in PSPEC]); HI = np.array([p[3] for p in PSPEC])


def pdict(x):
    return dict(zip(PN, np.clip(x, LO, HI)))


class Rollout:
    def __init__(s, xml="centauri1_revC.xml", V_bus=39.6):
        s.sim = S.Sim(xml, V_bus)
        s.stop_jid = {s.sim.m.joint("%s_%s" % (t, jn)).id: t[:4] + "_" + jn for t in ("foreL", "hindL") for jn in ("q0", "q1")}

    def contacts(s):
        """per tag: (normal force N, tangential N, sole in contact) ; any non-sole floor contact -> flag"""
        sim = s.sim; m, d = sim.m, sim.d; out = {t: [0.0, 0.0] for t in S.TAGS}; bad = None
        f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]; g1, g2 = c.geom1, c.geom2
            other = g2 if g1 == sim.floor else (g1 if g2 == sim.floor else None)
            if other is None: continue
            hit = None
            for t in S.TAGS:
                if sim.sole[t] == other: hit = t; break
            if hit is None:
                bad = m.geom(other).name; continue
            mujoco.mj_contactForce(m, d, i, f6)
            out[hit][0] += abs(f6[0]); out[hit][1] += float(np.hypot(f6[1], f6[2]))
        return out, bad

    def run(s, x, record=False, T_end=T_END, perturb=None):
        P = pdict(x); sim = s.sim; m, d = sim.m, sim.d; mot = sim.mot
        sim.set_pose(STAND)
        # drop to the ground: find z so the soles just touch
        mujoco.mj_forward(m, d)
        dz = -min(sim.sole_low(t) for t in S.TAGS) + 0.0005; d.qpos[sim.root[1]] += dz; mujoco.mj_forward(m, d)
        dt = m.opt.timestep; n = int(T_end / dt)
        qa, va, act = sim.qa, sim.va, sim.act
        st = dict(phase={"fore": "stand", "hind": "stand"}, t_air=None, td={"fore": None, "hind": None}, lo={"fore": None, "hind": None},
                  mark_hind_lo=None, mark_fore_td=None, in_air_both=False)
        mon = dict(tau_frac=0.0, tau_frac_j={}, speed_frac=0.0, bus_A_max=0.0, bus_A_min=0.0, phase_A_max={}, tendon_N=0.0, gas_min_mm=1e9,
                   gas_bottom=False, stop_Nm=0.0, grf_max={"fore": 0.0, "hind": 0.0}, grf_land={"fore": 0.0, "hind": 0.0}, slip_max=0.0,
                   bad_contact=None, leg_leg_mm=1e9, ext_err_min=1e9, ext_time=None, apex_z=-1e9, z0=None, pitch_minmax=[0.0, 0.0],
                   q_minmax={}, energy_J=0.0, grip_max=0.0, stop_Nm_j={})
        rec = []; root = sim.root; vroot = sim.vroot
        z0 = d.subtree_com[sim.torso][2]; mon["z0"] = float(z0); x0 = float(d.subtree_com[sim.torso][0])
        prev_contact = {"fore": True, "hind": True}; air_count = {"fore": 0, "hind": 0}; gnd_count = {"fore": 0, "hind": 0}
        last_both = None; settle_t = None; hoof_x_prev = {}
        for step in range(n):
            t = step * dt
            pitch = -d.qpos[root[2]] * D; pitch_rate = -d.qvel[vroot[2]] * D
            cf, bad = s.contacts() if step % 2 == 0 or record else (cf, bad)
            if bad and mon["bad_contact"] is None and t > 0.05: mon["bad_contact"] = (bad, round(t, 3))
            con = {k: (cf[k + "L"][0] + cf[k + "R"][0]) > 5.0 for k in S.KINDS}
            # contact debouncing
            for k in S.KINDS:
                if con[k]: gnd_count[k] += 1; air_count[k] = 0
                else: air_count[k] += 1; gnd_count[k] = 0
            # ---------------- phase logic
            ph = st["phase"]
            t_push = T_SETTLE0 + P["Tg"]
            for k in S.KINDS:
                if ph[k] == "stand" and t >= T_SETTLE0: ph[k] = "gather"
                if ph[k] == "gather" and t >= t_push + (P["dh"] if k == "hind" else 0.0): ph[k] = "push"; st.setdefault("t_push_" + k, t)
                if ph[k] == "push" and air_count[k] >= 8:
                    ph[k] = "ext"; st["lo"][k] = t
                    if k == "hind":
                        st["mark_hind_lo"] = float(np.mean([d.geom_xpos[sim.sole[tg]][0] for tg in ("hindL", "hindR")]))
                if ph[k] in ("ext", "prep", "swing") and gnd_count[k] >= 2 and st["t_air"] is not None:
                    ph[k] = "land"; st["td"][k] = t
                    if k == "fore" and st["mark_fore_td"] is None:
                        st["x_td"] = float(d.subtree_com[sim.torso][0])
                        st["mark_fore_td"] = float(np.mean([d.geom_xpos[sim.sole[tg]][0] for tg in ("foreL", "foreR")]))
            if st["t_air"] is None and ph["fore"] == "ext" and ph["hind"] == "ext" and air_count["fore"] > 0 and air_count["hind"] > 0:
                st["t_air"] = t; st["x_air"] = float(d.subtree_com[sim.torso][0])
            if st["t_air"] is not None:
                if ph["fore"] == "ext" and t >= st["t_air"] + P["tlp"]: ph["fore"] = "prep"
                tf = st["td"]["fore"]
                if ph["hind"] == "ext" and ((tf is not None and t >= tf + P["ths"]) or (P["ths"] < 0 and t >= st["t_air"] + P["tlp"] + 0.05 + P["ths"])):
                    ph["hind"] = "swing"
            if st["td"]["fore"] is not None and st["td"]["hind"] is not None and settle_t is None:
                settle_t = max(st["td"]["fore"], st["td"]["hind"])
            # ---------------- targets and gains
            for k in S.KINDS:
                pk = ph[k]; kp = list(KP[k]); kd = list(KD[k])
                if pk == "stand": tg = STAND[k]
                elif pk == "gather":
                    w = min((t - T_SETTLE0) / P["Tg"], 1.0); w = 3 * w * w - 2 * w ** 3
                    g = (P["fg0"], P["fg1"]) if k == "fore" else (P["hg0"], P["hg1"])
                    tg = (STAND[k][0] + w * (g[0] - STAND[k][0]), STAND[k][1] + w * (g[1] - STAND[k][1]))
                elif pk == "push": tg = (P["fp0"], P["fp1"]) if k == "fore" else (P["hp0"], P["hp1"])
                elif pk == "ext": tg = EXT[k]
                elif pk == "prep": tg = (P["fl0"], P["fl1"])
                elif pk == "swing": tg = (P["hl0"], P["hl1"])
                else:   # land
                    sc = (P["kl_f"], P["dl_f"]) if k == "fore" else (P["kl_h"], P["dl_h"])
                    kp = [kp[0] * sc[0], kp[1] * sc[0]]; kd = [kd[0] * sc[1], kd[1] * sc[1]]
                    a = (P["fa0"], P["fa1"]) if k == "fore" else (P["ha0"], P["ha1"])
                    if settle_t is not None and t > settle_t:
                        w = min((t - settle_t) / P["tst"], 1.0); w = 3 * w * w - 2 * w ** 3
                        a = (a[0] + w * (STAND[k][0] - a[0]), a[1] + w * (STAND[k][1] - a[1]))
                        g = 0.5 + 0.5 * w; kp = [kp[0] + w * (KP[k][0] * 0.6 - kp[0]), kp[1] + w * (KP[k][1] * 0.6 - kp[1])]
                    tg = a
                    if k == "fore":                 # pitch feedback: nose dropping -> fores reach / push forward
                        tg = (tg[0] - P["kth"] * pitch - P["kthd"] * pitch_rate, tg[1])
                    else:
                        tg = (tg[0] + 0.5 * P["kth"] * pitch, tg[1])
                for side in ("L", "R"):
                    tag = k + side
                    for j, jn in enumerate(("q0", "q1")):
                        q = d.qpos[qa[(tag, jn)]]; w = d.qvel[va[(tag, jn)]]
                        lo, hi = RANGE[k][jn]
                        ref = min(max(tg[j], lo + 0.5), hi - 0.5) / D
                        tau = kp[j] * (ref - q) - kd[j] * w
                        tau = mot.clip(k, jn, tau, w)
                        d.ctrl[act[(tag, jn)]] = tau
                    # gas strut (passive)
                    q1d = d.qpos[qa[(tag, "q1")]] * D
                    tg_, Ls = S.gas_torque(k, q1d)
                    d.qfrc_applied[va[(tag, "q1")]] = tg_
                    if Ls < mon["gas_min_mm"]: mon["gas_min_mm"] = Ls
            mujoco.mj_step(m, d)
            # ---------------- monitor (left legs; right identical by symmetry)
            if step % 2 == 0 or record:
                mdt = dt * (1 if record else 2)                      # time this monitor sample stands for
                busP = 0.0
                for k in S.KINDS:
                    tag = k + "L"
                    for jn in ("q0", "q1"):
                        tau = d.actuator_force[act[(tag, jn)]]; w = d.qvel[va[(tag, jn)]]; a = mot.j[(k, jn)]
                        fr = abs(tau) / a["peak"]; key = k + "_" + jn
                        if fr > mon["tau_frac_j"].get(key, 0): mon["tau_frac_j"][key] = fr
                        sf = abs(w) / a["w_free"]
                        if sf > mon["speed_frac"]: mon["speed_frac"] = sf
                        I, Pw = mot.elec(k, jn, tau, w); busP += 2 * Pw
                        if I > mon["phase_A_max"].get(key, 0): mon["phase_A_max"][key] = I
                        qd = d.qpos[qa[(tag, jn)]] * D; mm = mon["q_minmax"].setdefault(key, [1e9, -1e9]); mm[0] = min(mm[0], qd); mm[1] = max(mm[1], qd)
                    # tendon: equality constraint force on the slaved joint
                    tq = abs(d.qfrc_constraint[va[(tag, "sl")]]); tn = tq / S.TENDON_R
                    if tn > mon["tendon_N"]: mon["tendon_N"] = tn
                    gN = cf[k + "L"][0] + cf[k + "R"][0]; gT = cf[k + "L"][1] + cf[k + "R"][1]
                    if gN > mon["grf_max"][k]: mon["grf_max"][k] = gN
                    if ph[k] == "land" and gN > mon["grf_land"][k]: mon["grf_land"][k] = gN
                    if gN > 50: mon["grip_max"] = max(mon["grip_max"], gT / gN)
                    if gN > 50:
                        vs = abs(d.cvel[sim.hoof_b[tag]][3]) if False else None
                bA = busP / mot.V
                mon["bus_A_max"] = max(mon["bus_A_max"], bA); mon["bus_A_min"] = min(mon["bus_A_min"], bA)
                mon["energy_J"] += max(busP, 0.0) * mdt
                # joint-stop (limit) forces
                for i_ in range(d.nefc):
                    if d.efc_type[i_] == mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT and d.efc_id[i_] in s.stop_jid:
                        f_ = abs(d.efc_force[i_]); key_ = s.stop_jid[d.efc_id[i_]]
                        if f_ > mon["stop_Nm_j"].get(key_, 0.0): mon["stop_Nm_j"][key_] = f_
                        if f_ > mon["stop_Nm"]: mon["stop_Nm"] = f_
                # slip: sole speed while loaded
                for tg_ in ("foreL", "hindL"):
                    k = tg_[:4]
                    gN = cf[tg_][0]
                    if gN > 50:
                        v6 = np.zeros(6); mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_GEOM, sim.sole[tg_], v6, 0)
                        vs = float(np.hypot(v6[3], v6[4])); mon["slip_max"] = max(mon["slip_max"], vs)
                        mon["slide_mm"] = mon.get("slide_mm", 0.0) + vs * mdt * 1e3      # sliding distance under load (left fore + left hind summed)
                # extension check (in the air: no contact at all)
                if not con["fore"] and not con["hind"] and st["t_air"] is not None:
                    e = 0.0
                    for k in S.KINDS:
                        for j, jn in enumerate(("q0", "q1")):
                            e = max(e, abs(d.qpos[qa[(k + "L", jn)]] * D - EXT[k][j]))
                    if e < mon["ext_err_min"]: mon["ext_err_min"] = e
                    if e <= EXT_TOL and mon["ext_time"] is None: mon["ext_time"] = round(t, 3)
                c = d.subtree_com[sim.torso]
                mon["apex_z"] = max(mon["apex_z"], float(c[2]))
                mon["pitch_minmax"][0] = min(mon["pitch_minmax"][0], pitch); mon["pitch_minmax"][1] = max(mon["pitch_minmax"][1], pitch)
                # leg to leg (same side): fore lower+hoof segment vs hind lower+hoof segment (math, capsules r 30 mm)
                pf = [d.xpos[sim.bodies["foreL"][2]], d.site_xpos[m.site("foreL_ball").id]]
                ph_ = [d.xpos[sim.bodies["hindL"][2]], d.site_xpos[m.site("hindL_ball").id]]
                dd = seg_seg(pf[0], pf[1], ph_[0], ph_[1]) * 1e3 - 60.0
                if dd < mon["leg_leg_mm"]: mon["leg_leg_mm"] = dd
            if record and step % int(round(DT_REC / dt)) == 0:
                rec.append(dict(t=t, qpos=d.qpos.copy(), ph=dict(ph), tau={kk: float(d.actuator_force[act[kk]]) for kk in act},
                                cf={tt: list(cf[tt]) for tt in S.TAGS}, com=d.subtree_com[sim.torso].copy(), pitch=pitch,
                                tendon={tt: abs(d.qfrc_constraint[va[(tt, "sl")]]) / S.TENDON_R for tt in ("foreL", "hindL")},
                                qvel={kk: float(d.qvel[va[kk]]) for kk in va}))
            if not np.isfinite(d.qpos).all():
                mon["bad_contact"] = ("diverged", t); break
        # ---------------- outcome
        c = d.subtree_com[sim.torso]; v = d.qvel[vroot[0]], d.qvel[vroot[1]]
        out = dict(P=P, mon=mon, st={k: v_ for k, v_ in st.items() if k != "phase"}, phase_end=dict(st["phase"]),
                   final=dict(x=float(c[0] - x0), z=float(c[2] - z0), pitch=-float(d.qpos[root[2]] * D), vx=float(v[0]), vz=float(v[1]),
                              w=float(d.qvel[vroot[2]] * D)))
        out["dist"] = (st["mark_fore_td"] - st["mark_hind_lo"]) if (st["mark_fore_td"] is not None and st["mark_hind_lo"] is not None) else 0.0
        out["flight"] = (st["td"]["fore"] - st["t_air"]) if (st["t_air"] is not None and st["td"]["fore"] is not None) else 0.0
        out["rise"] = mon["apex_z"] - z0
        out["air_travel"] = (st["x_td"] - st["x_air"]) if ("x_td" in st and "x_air" in st) else 0.0
        if record: out["rec"] = rec
        return out


def seg_seg(p, q, a, b):
    best = 1e9
    for tt in (0.0, 0.25, 0.5, 0.75, 1.0):
        x = p + (q - p) * tt; ab = b - a
        s_ = min(max((x - a) @ ab / max(ab @ ab, 1e-12), 0.0), 1.0)
        best = min(best, float(np.linalg.norm(x - (a + ab * s_))))
    return best


def cost(o):
    """lower is better.  Goal: leap length (hind take-off mark to fore landing mark) with real forward travel; everything
    else is a constraint (penalty)."""
    mon = o["mon"]; f = o["final"]; pen = 0.0; st = o["st"]
    if mon["bad_contact"] is not None: pen += 5.0
    if o["dist"] <= 0: pen += 3.0
    tf, th = st["td"]["fore"], st["td"]["hind"]
    if tf is None or th is None or th < tf: pen += 2.0 + (0.0 if (tf is None or th is None) else 5.0 * (tf - th))   # fores land first
    pen += 0.15 * max(mon["ext_err_min"] - EXT_TOL, 0.0)                    # reach the full extension in the air
    pen += 2.0 * min(abs(f["pitch"]) / 10.0, 3.0) * (abs(f["pitch"]) > 3) + 1.0 * min(abs(f["z"]) / 0.05, 3.0) * (abs(f["z"]) > 0.03)
    pen += 1.0 * min(abs(f["vx"]) + abs(f["vz"]), 3.0) + 0.02 * min(abs(f["w"]), 100)
    pen += 0.005 * max(mon.get("slide_mm", 0.0) - 40.0, 0.0)
    pen += 0.5 * min(max(mon["stop_Nm"] - 30.0, 0.0) / 30.0, 3.0)               # hard stops should not catch the landing              # hooves may skid at most ~30 mm in all
    pen += 0.2 * max(-mon["leg_leg_mm"] + 20.0, 0.0) / 10
    pen += 0.5 * max(mon["tendon_N"] / DYNEEMA_PAIR_N * 3.0 - 1.0, 0.0)     # keep the tendon at SF >= 3
    pen += 1.0 * max(o["phase_end"]["fore"] != "land", 0) + 1.0 * max(o["phase_end"]["hind"] != "land", 0)
    pen += 5.0 * max(0.12 - o["flight"], 0.0) / 0.12                     # a real flight, long enough to reach full extension
    gain = 1.5 * min(o.get("air_travel", 0.0), 2.0) + 0.7 * min(f["x"], 2.0) + 0.3 * min(o["dist"], 2.0)   # carry distance, not span
    return -gain + pen


# ------------------------------------------------------------------ search
def _eval(args):
    """cost of one parameter set; xml may be a list of models (robust search: mean cost + half the spread to the worst)"""
    x, xml = args
    global _R
    try:
        _R
    except NameError:
        _R = {}
    xs = xml if isinstance(xml, (list, tuple)) else [xml]
    C = []; first = None
    for xm in xs:
        if xm not in _R: _R[xm] = Rollout(xm)
        try:
            o = _R[xm].run(x); C.append(cost(o))
            if first is None: first = (o["dist"], o["mon"]["ext_err_min"], o["final"])
        except Exception as e:
            C.append(50.0)
            if first is None: first = (0.0, 1e9, str(e))
    c = float(np.mean(C) + 0.5 * (max(C) - np.mean(C)))
    return (c,) + first


def search(n_iter=60, seed=1, x0=None, sigma=0.25, xml="centauri1_revC.xml", popsize=16):
    import cma, multiprocessing as mp
    span = HI - LO
    z0 = (np.array(x0 if x0 is not None else P0) - LO) / span
    es = cma.CMAEvolutionStrategy(z0, sigma, dict(bounds=[0, 1], seed=seed, popsize=popsize, verbose=-9))
    best = (1e9, None, None)
    with mp.Pool(4) as pool:
        for it in range(n_iter):
            Z = es.ask(); X = [LO + z * span for z in Z]
            R = pool.map(_eval, [(x, xml) for x in X])
            es.tell(Z, [r[0] for r in R])
            i = int(np.argmin([r[0] for r in R]))
            if R[i][0] < best[0]: best = (R[i][0], X[i].tolist(), R[i])
            print("it %3d  best %.3f  dist %.3f  ext_err %.1f  final %s" % (it, best[0], best[2][1], best[2][2], {k: round(v, 3) for k, v in best[2][3].items()} if isinstance(best[2][3], dict) else best[2][3]), flush=True)
    return best


if __name__ == "__main__":
    cmd = sys.argv[1] if len(sys.argv) > 1 else "run"
    if cmd == "search":
        n_it = int(sys.argv[2]) if len(sys.argv) > 2 else 60; sd = int(sys.argv[3]) if len(sys.argv) > 3 else 1
        start = json.load(open(os.path.join(HERE, "best_params.json")))["x"] if os.path.exists(os.path.join(HERE, "best_params.json")) and "--fresh" not in sys.argv else None
        xmls = os.environ.get("CENT_ROBUST")
        b = search(n_it, sd, start, xml=xmls.split(",") if xmls else "centauri1_revC.xml", sigma=float(os.environ.get("CENT_SIGMA", 0.15)) if start is not None else 0.25)
        prev = json.load(open(os.path.join(HERE, "best_params.json"))) if os.path.exists(os.path.join(HERE, "best_params.json")) else None
        if prev is None or b[0] < prev["cost"]:
            json.dump(dict(cost=b[0], x=b[1], params=pdict(np.array(b[1])), dist=b[2][1]), open(os.path.join(HERE, "best_params.json"), "w"), indent=1, default=float)
            print("saved", b[0])
    elif cmd == "probe":
        R = Rollout(); t0 = time.time(); o = R.run(P0); print("%.2f s" % (time.time() - t0))
        print(json.dumps({k: v for k, v in o.items() if k != "rec"}, indent=1, default=lambda z: round(float(z), 3) if np.isscalar(z) else str(z)))
        print("cost", cost(o))


# ------------------------------------------------------------------ record the best leap: numbers, joint angles, replay frames
def report(x, xml="centauri1_revC.xml", V_bus=39.6, tag=""):
    R = Rollout(xml, V_bus); o = R.run(np.array(x), record=True); sim = R.sim; m = sim.m
    mon = o["mon"]; mot = sim.mot
    lim = {"%s_%s" % (k, j): dict(motor=mot.j[(k, j)]["name"], peak_joint_Nm=round(mot.j[(k, j)]["peak"], 1),
                                    rated_joint_Nm=round(mot.j[(k, j)]["rated"], 1), free_speed_deg_s=round(mot.j[(k, j)]["w_free"] * D))
           for k in S.KINDS for j in ("q0", "q1")}
    peak_tau = {}; peak_w = {}; rms_tau = {}
    for key in lim:
        k, j = key.split("_"); a = sim.act[(k + "L", j)]; v = sim.va[(k + "L", j)]
        tt = np.array([r["tau"][(k + "L", j)] for r in o["rec"]]); ww = np.array([r["qvel"][(k + "L", j)] for r in o["rec"]])
        peak_tau[key] = float(np.abs(tt).max()); peak_w[key] = float(np.abs(ww).max() * D); rms_tau[key] = float(np.sqrt((tt ** 2).mean()))
    rec = o["rec"]
    # flight apex and hoof clearance
    t_air, tdf = o["st"]["t_air"], o["st"]["td"]["fore"]
    res = dict(
        V_bus=V_bus, params={k: round(float(v), 4) for k, v in o["P"].items()},
        leap_length_m=round(o["dist"], 3), air_travel_m=round(o["air_travel"], 3), flight_time_s=round(o["flight"], 3),
        com_rise_m=round(o["rise"], 3), com_apex_m=round(mon["apex_z"], 3), com_stand_m=round(mon["z0"], 3),
        ground_travel_m=round(o["final"]["x"], 3), final=o["final"], pitch_range_deg=[round(v, 1) for v in mon["pitch_minmax"]],
        extension=dict(target=EXT, err_min_deg=round(mon["ext_err_min"], 2), reached_at_s=mon["ext_time"], tol_deg=EXT_TOL),
        events=o["st"],
        joints={key: dict(lim[key], peak_Nm=round(peak_tau[key], 1), peak_frac=round(peak_tau[key] / lim[key]["peak_joint_Nm"], 2),
                          rms_Nm=round(rms_tau[key], 1), peak_speed_deg_s=round(peak_w[key]), speed_frac=round(peak_w[key] / lim[key]["free_speed_deg_s"], 2),
                          phase_A_peak=round(mon["phase_A_max"].get(key, 0), 1), range_used_deg=[round(v, 1) for v in mon["q_minmax"][key]], stop_load_Nm=round(mon["stop_Nm_j"].get(key, 0.0), 1),
                          range_deg=RANGE[key.split("_")[0]][key.split("_")[1]]) for key in lim},
        bus_A_peak=round(mon["bus_A_max"], 1), bus_A_regen_peak=round(mon["bus_A_min"], 1), energy_J=round(mon["energy_J"], 0),
        ground_force_peak_N=dict((k, round(v)) for k, v in mon["grf_max"].items()),
        landing_force_peak_N=dict((k, round(v)) for k, v in mon["grf_land"].items()),
        landing_force_peak_bw=dict((k, round(v / (S.INFO["m_total"] * 9.81), 2)) for k, v in mon["grf_land"].items()),
        grip_max=round(mon["grip_max"], 2), slip_peak_m_s=round(mon["slip_max"], 3), slide_mm=round(mon.get("slide_mm", 0.0), 1),
        tendon_peak_N=round(mon["tendon_N"]), tendon_SF=round(DYNEEMA_PAIR_N / max(mon["tendon_N"], 1), 1),
        gas_min_len_mm=round(mon["gas_min_mm"], 1), gas_bottom_mm=S.INFO["gas"]["fore"]["Lc"],
        leg_leg_min_mm=round(mon["leg_leg_mm"], 0), body_floor_contact=mon["bad_contact"],
        cost=round(cost(o), 4))
    # hoof clearance in flight: lowest sole while airborne
    clr = []
    for r in rec:
        if t_air is not None and tdf is not None and t_air + 0.02 < r["t"] < tdf - 0.01:
            sim.d.qpos[:] = r["qpos"]; mujoco.mj_kinematics(m, sim.d)
            clr.append(min(sim.sole_low(t) for t in S.TAGS))
    res["hoof_clearance_flight_min_mm"] = round(min(clr) * 1e3, 0) if clr else None
    res["hoof_clearance_flight_max_mm"] = round(max(clr) * 1e3, 0) if clr else None
    return res, o, sim


def export(res, o, sim, base="leap"):
    rec = o["rec"]; m = sim.m; d = sim.d
    # joint-angle trajectory csv (deg; both legs of a pair are identical)
    rows = ["t_s,phase_fore,phase_hind,body_x_m,body_z_m,pitch_deg,fore_pivot_q0,fore_flex_q1,fore_elbow,fore_hoof_hp,hind_pivot_q0,hind_flex_q1,hind_hock,hind_hoof_hp,"
            "fore_q0_Nm,fore_q1_Nm,hind_q0_Nm,hind_q1_Nm,fore_ground_N,hind_ground_N,fore_tendon_N,hind_tendon_N"]
    for r in rec:
        q = r["qpos"]; qa = sim.qa
        g = lambda t, j: q[qa[(t, j)]] * D
        rows.append(",".join([("%.3f" % r["t"]), r["ph"]["fore"], r["ph"]["hind"], "%.4f" % q[sim.root[0]], "%.4f" % q[sim.root[1]], "%.2f" % r["pitch"]] +
                             ["%.2f" % g("foreL", j) for j in ("q0", "q1", "sl", "hp")] + ["%.2f" % g("hindL", j) for j in ("q0", "q1", "sl", "hp")] +
                             ["%.1f" % r["tau"][(t, j)] for t in ("foreL", "hindL") for j in ("q0", "q1")] +
                             ["%.0f" % (r["cf"][k + "L"][0] + r["cf"][k + "R"][0]) for k in S.KINDS] + ["%.0f" % r["tendon"][t] for t in ("foreL", "hindL")]))
    open(os.path.join(HERE, base + "_trajectory.csv"), "w").write("\n".join(rows) + "\n")
    # replay frames: world pose of every body (m, quaternion wxyz) at 50 Hz
    names = ["torso"] + ["%s_%s" % (t, g) for t in S.TAGS for g in ("upper", "middle", "lower", "hoof")]
    ids = [m.body(n).id for n in names]; frames = []
    for i, r in enumerate(rec):
        if i % 4: continue
        d.qpos[:] = r["qpos"]; mujoco.mj_kinematics(m, d)
        fr = [round(r["t"], 3)]
        for b in ids: fr += [round(float(v), 4) for v in d.xpos[b]] + [round(float(v), 4) for v in d.xquat[b]]
        fr += [round(r["ph"]["fore"] == "land" and 1 or 0)]
        frames.append(dict(f=fr, ph=[r["ph"]["fore"], r["ph"]["hind"]], gf=round((r["cf"]["foreL"][0] + r["cf"]["foreR"][0])), gh=round(r["cf"]["hindL"][0] + r["cf"]["hindR"][0]),
                           tq=[round(r["tau"][(t, j)], 1) for t in ("foreL", "hindL") for j in ("q0", "q1")], pitch=round(r["pitch"], 1),
                           tn=[round(r["tendon"][t]) for t in ("foreL", "hindL")]))
    json.dump(dict(bodies=names, frames=frames, res=res), open(os.path.join(HERE, base + "_replay.json"), "w"), separators=(",", ":"), default=float)
    json.dump(res, open(os.path.join(HERE, base + "_result.json"), "w"), indent=1, default=float)


if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "run":
    b = json.load(open(os.path.join(HERE, "best_params.json")))
    res, o, sim = report(b["x"]); export(res, o, sim)
    print(json.dumps({k: v for k, v in res.items() if k not in ("params", "events")}, indent=1, default=float))
