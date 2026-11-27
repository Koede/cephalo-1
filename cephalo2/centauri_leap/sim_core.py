#!/usr/bin/env python3
"""Shared simulation core for the Centauri-1 leap: model loading, the motor envelope, the gas struts, the load / current
monitor and the collision checks (all by math).  Units: SI inside, degrees in reports.

Motor envelope (est., linear PMSM model at the bus voltage): full peak torque up to half the no-load speed, falling linearly
to zero at the no-load speed when the motor drives; full peak torque when it brakes.  No-load speed scales with bus voltage
(act_table rpm is at 48 V).  The fore flex joint sees the RobStride 04 through the 46:22 belt (joint 2.09 x faster, torque
x 22/46 x 0.95).
Current: phase current = motor-output torque / kt; bus power per motor = shaft power + copper loss (I^2 r_ll); bus current
= sum / V (regeneration counted, est.).  RobStride 04 kt 2.4 N m/A and r_ll 0.25 ohm are estimates (not in act_table)."""
import os, json
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
INFO = json.load(open(os.path.join(HERE, "model_info.json")))
ACTT = json.load(open(os.path.join(HERE, "legs_revC_snapshot", "act_table.json")))
EST_KT = {"RobStride 04": (2.4, 0.25)}
BELT_EFF = 0.95
KINDS = ("fore", "hind"); TAGS = ("foreL", "foreR", "hindL", "hindR")
D = 180 / np.pi
TENDON_R = 0.030                    # elbow / hock tendon drum radius (m)


def load(xml="centauri1_revC.xml"):
    return mujoco.MjModel.from_xml_path(os.path.join(HERE, xml))


class Motors:
    def __init__(s, V_bus=39.6):
        s.V = V_bus; s.j = {}
        for k in KINDS:
            for jn in ("q0", "q1"):
                a = INFO["act"][k][jn]; t = ACTT[a["name"]]; r = a["ratio"]; eff = BELT_EFF if r != 1 else 1.0
                kt, rl = (t["kt"], t["r_ll"]) if t["kt"] else EST_KT[a["name"]]
                w_m = t["w_free_48"] * 2 * np.pi / 60 * V_bus / 48.0          # motor output no-load speed (rad/s)
                s.j[(k, jn)] = dict(name=a["name"], r=r, eff=eff, peak_m=t["peak"], rated_m=t["rated"], kt=kt, rl=rl,
                                    peak=t["peak"] * r * eff, rated=t["rated"] * r * eff, w_free=w_m / r)

    def clip(s, k, jn, tau, w):
        """Joint torque command -> what the motor can give at joint speed w (rad/s)."""
        a = s.j[(k, jn)]; pk = a["peak"]
        if tau * w > 0:                                   # driving
            f = np.clip((a["w_free"] - abs(w)) / (0.5 * a["w_free"]), 0.0, 1.0); lim = pk * f
        else:
            lim = pk
        return float(np.clip(tau, -lim, lim))

    def elec(s, k, jn, tau, w):
        """-> phase current (A), bus power (W) for joint torque tau at joint speed w."""
        a = s.j[(k, jn)]
        # r = joint torque / motor torque (= motor speed / joint speed); the 46:22 belt speeds the joint up, so r < 1
        tm = tau / a["r"] / a["eff"] if tau * w > 0 else tau / a["r"] * a["eff"]     # motor-output torque
        wm = w * a["r"]
        I = abs(tm) / a["kt"]
        return I, tm * wm + I * I * a["rl"]


def gas_torque(k, q1_deg):
    """Generalised torque on the flex joint from the 400 N gas strut (N m) and the strut length (mm)."""
    g = INFO["gas"][k]; q = np.array(g["q"]); L = np.array(g["L"])
    Ls = float(np.interp(q1_deg, q, L)); dL = float(np.interp(q1_deg, q[:-1] + 0.25, np.diff(L) / np.diff(q * np.pi / 180)))
    F = 400.0 * (1.0 + 0.2 * (g["Le"] - Ls) / (g["Le"] - g["Lc"]))          # 400 N extended, +20 % at full stroke (est.)
    if Ls >= g["Le"]: F = 0.0
    return F * dL * 1e-3, Ls


class Sim:
    """Model + data + index helpers."""
    def __init__(s, xml="centauri1_revC.xml", V_bus=39.6):
        s.m = load(xml); s.d = mujoco.MjData(s.m); s.mot = Motors(V_bus)
        J = lambda n: s.m.joint(n)
        s.qa = {}; s.va = {}; s.act = {}
        for t in TAGS:
            for jn in ("q0", "q1", "sl", "hp"):
                s.qa[(t, jn)] = J("%s_%s" % (t, jn)).qposadr[0]; s.va[(t, jn)] = J("%s_%s" % (t, jn)).dofadr[0]
            for jn in ("q0", "q1"):
                s.act[(t, jn)] = s.m.actuator("%s_%s_m" % (t, jn)).id
        s.root = [J(n).qposadr[0] for n in ("rootx", "rootz", "rooty")]
        s.vroot = [J(n).dofadr[0] for n in ("rootx", "rootz", "rooty")]
        s.sole = {t: s.m.geom("%s_sole" % t).id for t in TAGS}
        s.floor = s.m.geom("floor").id
        s.hoof_b = {t: s.m.body("%s_hoof" % t).id for t in TAGS}
        s.bodies = {t: [s.m.body("%s_%s" % (t, g)).id for g in ("upper", "middle", "lower", "hoof")] for t in TAGS}
        s.torso = s.m.body("torso").id
        s.eq = {t: s.m.equality("%s_tendon" % t).id for t in TAGS}

    def set_pose(s, q, z=0.0, x=0.0, pitch=0.0):
        """q: {kind: (q0, q1)} in degrees; hoof ball joints level with the ground."""
        d = s.d; d.qpos[:] = 0; d.qvel[:] = 0
        d.qpos[s.root[0]] = x; d.qpos[s.root[1]] = z; d.qpos[s.root[2]] = -pitch / D
        for t in TAGS:
            k = t[:4]; q0, q1 = q[k]
            d.qpos[s.qa[(t, "q0")]] = q0 / D; d.qpos[s.qa[(t, "q1")]] = q1 / D; d.qpos[s.qa[(t, "sl")]] = q1 / D
            d.qpos[s.qa[(t, "hp")]] = (q0 - q1 + q1) / D * 0 + (pitch / D) * 0
        mujoco.mj_forward(s.m, d)
        # level hooves: hp = world angle of the lower segment (lower stays at -q0 - delta + pitch); keep the sole flat
        for t in TAGS:
            k = t[:4]
            R = d.xmat[s.m.body("%s_lower" % t).id].reshape(3, 3); ang = np.arctan2(R[0, 2], R[0, 0])
            d.qpos[s.qa[(t, "hp")]] = float(np.clip(-ang - INFO["legs"][k]["hoof_const"] / D, -15 / D, 15 / D))
        mujoco.mj_forward(s.m, d)

    def sole_low(s, t):
        """lowest z of a hoof sole box (m)."""
        g = s.sole[t]; c = s.d.geom_xpos[g]; R = s.d.geom_xmat[g].reshape(3, 3); h = s.m.geom_size[g]
        return float(c[2] - abs(R[2, 0]) * h[0] - abs(R[2, 1]) * h[1] - abs(R[2, 2]) * h[2])

    def com(s):
        return s.d.subtree_com[s.torso].copy()
