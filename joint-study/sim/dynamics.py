"""MuJoCo dynamic check for shortlisted arm configurations.

The arm is mounted horizontally at the hub with gravity on. The wave, step and sag tests are flown:
the claw-prop pushes up with hover thrust (5.5 N). The foot-lift test has no thrust. Each rotation element in chain.py becomes a
hinge; elements that share a control variable (rolling-contact half angles, section coupling) are tied
with joint equality constraints, which is exactly how the kinematic model couples them. Each control
variable gets a position actuator whose stiffness and force limit come from the drive (servo or tendon
with capstan losses) used in the static scoring. Three tests:
  wave  : travelling-wave curl, 1 Hz, 3 rad/m curvature amplitude        -> tracking error, tip jerk, saturation
  step  : straight -> fitted 270 deg curl                   -> settle time, overshoot
  lift  : walking stance -> 45 mm foot lift and back, 1 Hz  -> tip tracking, peak drive utilisation
"""
import math
import numpy as np
import mujoco
import chain as CH
import metrics as M
import config as C

def to_mjcf(ch, masses, kv, cap, fr):
    E = len(ch.et)
    xml = ['<mujoco model="tentacle"><option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast"/>',
           '<default><geom contype="0" conaffinity="0" rgba="0.8 0.5 0.3 1"/></default><worldbody>']
    R = ch.base_R
    # MuJoCo quat from rotation matrix
    q = np.zeros(4); mujoco.mju_mat2Quat(q, R.flatten())
    xml.append(f'<body name="base" pos="{ch.base_pos[0]} {ch.base_pos[1]} {ch.base_pos[2]}" quat="{q[0]} {q[1]} {q[2]} {q[3]}">')
    depth = 1; pend = 0.0; jnames = {}; first_of_var = {}; eqs = []; jcount = 0
    seg_len = np.zeros(ch.N)
    for i in range(E):
        if ch.et[i] == 1: seg_len[ch.enode[i]] += ch.el[i]
    for i in range(E):
        t = ch.et[i]; k = ch.enode[i]
        if t == 0 or t == 2:
            xml.append(f'<body pos="0 0 {pend:.5f}"><geom type="sphere" size="0.004" mass="0.0005"/>'); depth += 1; pend = 0.0
            nm = f"j{jcount}"; jcount += 1
            v = ch.ev[i]
            if t == 0:
                a = ch.ax[i]
                xml.append(f'<joint name="{nm}" type="hinge" axis="{a[0]:.6f} {a[1]:.6f} {a[2]:.6f}" damping="0.02" frictionloss="{fr:.4f}" armature="0.002"/>')
            else:
                xml.append(f'<joint name="{nm}" type="slide" axis="0 0 1" damping="1" armature="0.01"/>')
            jnames[i] = nm
            if v >= 0:
                if v not in first_of_var: first_of_var[v] = (nm, ch.ec[i])
                else:
                    n0, c0 = first_of_var[v]
                    eqs.append(f'<joint joint1="{nm}" joint2="{n0}" polycoef="0 {ch.ec[i]/c0:.6f} 0 0 0" solref="0.002 1"/>')
        elif t == 1:
            L = ch.el[i]; m = max(masses[k] * L / max(seg_len[k], 1e-6), 1e-4)
            xml.append(f'<geom type="capsule" fromto="0 0 {pend:.5f} 0 0 {pend+L:.5f}" size="{ch.seg_r[k]:.4f}" mass="{m:.5f}"/>')
            pend += L
    xml.append(f'<site name="tip" pos="0 0 {pend:.5f}" size="0.005"/>')
    xml.append(f'<geom type="sphere" pos="0 0 {pend:.5f}" size="0.02" mass="{C.TIP_PAYLOAD_MASS}"/>')
    xml.append("</body>" * depth + "</worldbody><equality>" + "".join(eqs) + "</equality><actuator>")
    for v in range(ch.nv):
        nm, c0 = first_of_var[v]
        kp = kv[v] / c0 ** 2; fmax = cap[v] / abs(c0)
        xml.append(f'<position name="a{v}" joint="{nm}" kp="{kp:.4f}" kv="{0.05*kp**0.5:.4f}" forcerange="{-fmax:.4f} {fmax:.4f}" forcelimited="true"/>')
    xml.append("</actuator></mujoco>")
    return "".join(xml), first_of_var

def drive_arrays(kin, cfg, fam, var, comp_name, comp, drive):
    """recompute per-variable stiffness and capacity exactly as metrics.score does (copied logic, small)"""
    a = kin["_arr"]; nv = kin["nv"]; D = a["D"]; vk = a["var_kind"]; vn = a["var_node"]; act = cfg["actuation"]
    kv = np.zeros(nv); cap = np.zeros(nv)
    for v in range(nv):
        if vk[v] == "local":
            nm = cfg.get("root_servo", "STS3250_12V") if act == "hybrid" else fam.get("servo_override", cfg.get("servo"))
            if a.get("node_servo"): nm = a["node_servo"][vn[v]]
            if a["var_axis"][v] == "twist" and not fam.get("diff"): nm = "STS3032"
            sp = CH.drive_spec(nm); mult = 2.0 if fam.get("diff") else 1.0
            cap[v] = sp["stall"] * mult; kv[v] = cap[v] * M.SERVO_K_PER_STALL
        else:
            kind = "pushpull" if act == "pushpull" else vk[v]
            sp = CH.drive_spec(drive)
            rp = min(max(sp["stall"] / M.TENSION_CAP.get(kind, C.TENDON_SWL), 0.004), 0.012)
            rm = 0.38 * D[vn[v]]; eta = math.exp(-M.MU.get(kind, 0.15) * M.ROUTING_WRAP * (vn[v] + 1))   # near-straight flight posture
            cap[v] = min(sp["stall"] / rp, M.TENSION_CAP.get(kind, C.TENDON_SWL)) * rm * eta
            Lt = 0.10 + float(a["P"][:vn[v] + 1].sum())
            kt = 2 * C.TENDON_EA / Lt * rm ** 2; kd = sp["stall"] * M.SERVO_K_PER_STALL * (rm / rp) ** 2
            kv[v] = 1 / (1 / kt + 1 / kd)
    if comp_name in ("tendon_spring", "tpu_coupler"):
        kc = comp["k_rel"] * kv; kv = 1 / (1 / kv + 1 / kc)
    return kv, cap

def _sim(model, data, ch, first, qfun, T, tip_ref_fun=None, thrust=0.0):
    dt = model.opt.timestep; n = int(T / dt)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tip")
    data.xfrc_applied[:] = 0; data.xfrc_applied[model.site_bodyid[sid], 2] = thrust   # hover thrust at the claw-prop
    tips = np.zeros((n, 3)); qerr = np.zeros(n); sat = 0; refs = np.zeros((n, 3)); umax = 0.0
    jid = [model.joint(first[v][0]).qposadr[0] for v in range(ch.nv)]
    c0 = np.array([first[v][1] for v in range(ch.nv)])
    fr = model.actuator_forcerange[:, 1]
    for s in range(n):
        t = s * dt; qd = qfun(t)
        data.ctrl[:] = c0 * qd
        mujoco.mj_step(model, data)
        tips[s] = data.site_xpos[sid]
        qerr[s] = np.sqrt(np.mean((data.qpos[jid] / c0 - qd) ** 2))
        u = np.abs(data.actuator_force) / np.maximum(fr, 1e-9)
        umax = max(umax, float(u.max())); sat += int((u > 0.98).any())
        if tip_ref_fun is not None: refs[s] = tip_ref_fun(qd)
    return tips, qerr, sat / n, refs, umax

def evaluate(kin, cfg, fam, var, comp_name, comp, drive):
    ch = CH.build_chain(cfg); masses = M.node_masses(ch)
    kv, cap = drive_arrays(kin, cfg, fam, var, comp_name, comp, drive)
    xml, first = to_mjcf(ch, masses, kv, cap, var["fr"] * 0.2)
    model = mujoco.MjModel.from_xml_string(xml); data = mujoco.MjData(model)
    tipf = lambda q: CH.run_fk(ch, q)[0][-1]
    out = {}
    # settle under gravity at straight
    q0 = M._x0(ch)
    mujoco.mj_resetData(model, data)
    sid0 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tip")
    data.xfrc_applied[model.site_bodyid[sid0], 2] = C.TIP_THRUST_HOVER
    for _ in range(500): data.ctrl[:] = 0; mujoco.mj_step(model, data)
    sid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tip")
    out["sag_mm"] = float(np.linalg.norm(data.site_xpos[sid] - tipf(q0)) * 1e3)
    # wave
    bend = np.array([1.0 if ch.var_axis_kind[v] == "bend" else 0.0 for v in range(ch.nv)])
    # same physical wave for every arm: curvature amplitude 3 rad/m (about 100 deg over the arm)
    spacing = 2.0 if cfg["arrangement"].startswith("alt90") else (1.5 if cfg["arrangement"] == "helix60" else 1.0)
    per = np.array([3.0 * ch.P[ch.var_node[v]] * spacing * (sum(1 for u in range(ch.nv) if ch.var_node[u] == ch.var_node[v]) if ch.var_kind[v] == "section" else 1)
                    for v in range(ch.nv)])
    if any(k == "section" for k in ch.var_kind):
        per = np.array([3.0 * float(ch.P.sum()) / 3.0 * (1 if ch.fam["bend"] == 2 else 0.5) * spacing for v in range(ch.nv)])
    amp = np.minimum(0.8 * np.minimum(ch.hi, -ch.lo), per) * bend
    ph = np.array([0.5 * ch.var_node[v] for v in range(ch.nv)])
    hz = float(__import__("os").environ.get("WAVE_HZ", "1.0"))   # wave frequency; 1 Hz in the study
    qw = lambda t: amp * np.sin(2 * math.pi * hz * t - ph) * min(1.0, t / 0.5)
    mujoco.mj_resetData(model, data)
    tips, qerr, sat, refs, umax = _sim(model, data, ch, first, qw, 3.0, tipf, thrust=C.TIP_THRUST_HOVER)
    sl = slice(1000, None)
    out["wave_tip_rms_mm"] = float(np.sqrt(np.mean(np.sum((tips[sl] - refs[sl]) ** 2, 1))) * 1e3)
    out["wave_q_rms_deg"] = float(np.degrees(np.mean(qerr[sl])))
    dt = model.opt.timestep
    jerk = np.diff(tips[sl], 3, axis=0) / dt ** 3
    jref = np.diff(refs[sl], 3, axis=0) / dt ** 3
    out["wave_jerk_ratio"] = float(np.sqrt(np.mean(np.sum(jerk ** 2, 1))) / max(np.sqrt(np.mean(np.sum(jref ** 2, 1))), 1e-9))
    out["wave_sat_frac"] = float(sat); out["wave_umax"] = umax
    # step to curl
    tgt = M.target_shapes(ch)["arc_curl"]
    qc, _, _ = M.fit_shape(ch, tgt)
    qs = lambda t: qc if t > 0.2 else q0
    mujoco.mj_resetData(model, data)
    tips, qerr, sat, refs, umax = _sim(model, data, ch, first, qs, 2.0, thrust=C.TIP_THRUST_HOVER)
    goal = tipf(qc); d = np.linalg.norm(tips - goal, axis=1) * 1e3
    k0 = int(0.2 / dt); inside = d[k0:] < 5.0
    settle = None
    for s in range(len(inside)):
        if inside[s:].all(): settle = s * dt; break
    out["step_settle_s"] = float(settle) if settle is not None else 9.9
    out["step_final_mm"] = float(d[-1])
    # stance -> foot lift
    x0 = M._x0(ch)
    qa = M.ik_foot(ch, M.FEET[1], x0)[0]
    up = M.FEET[1] + np.array([0, 0, 0.045]); qb = M.ik_foot(ch, up, qa)[0]
    ql = lambda t: qa + (qb - qa) * 0.5 * (1 - math.cos(2 * math.pi * t)) if t > 0.5 else qa
    mujoco.mj_resetData(model, data)
    data.qpos[:] = 0
    tips, qerr, sat, refs, umax = _sim(model, data, ch, first, ql, 3.0, tipf)
    out["lift_tip_rms_mm"] = float(np.sqrt(np.mean(np.sum((tips[1000:] - refs[1000:]) ** 2, 1))) * 1e3)
    out["lift_umax"] = umax
    return out
