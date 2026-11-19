"""Bench for the 'one pull ring' hub idea: each finger has ONE cord over K1, K2, K3 (levers 6 / 4.5 / 3.5 mm); all four cords end on
one ring pulled by one stationary servo (STS3215 on a 6 mm spool, est. 294 N stall / 98 N rated at the ring, shared by 4 fingers
through a tilting ring = equal tension).  Same contact model and bar test as hand3_mj.py."""
import sys, json, numpy as np, mujoco
import hand3_mj as HM
H = HM.H
F_STALL, F_RATED, K_CORD = 294.0 / 4, 98.0 / 4, 20000.0          # per finger: N, N, N/m (TPU link, est.)
PRE = 0.7; LEV = HM.LEVER; EXT = (0.0, 0.012, 0.024)             # extensor pretension at K1, K2, K3 (N m): closes base first, tip last
def model(r_bar):
    hx = HM.hand_xml(0)
    for j, e in ((1, EXT[1]),):                                   # K2 gets a pretension too (K3 already has HM.EXT_K3)
        for k in range(4):
            o = f'name="h3_0_f{k}_{j}" axis="0 -1 0" pos="{H.KN[j] * 1e-3:.4f} 0 0" range="-2 {H.FOLD_MAX[j]:.0f}" damping="0.002" armature="2e-5" stiffness="0.004"'
            assert o in hx; hx = hx.replace(o, o.replace('stiffness="0.004"', f'stiffness="0.01" springref="{-np.degrees(e / 0.01):.1f}"'))
    ten = "<tendon>" + "".join(f'<fixed name="t{k}">' + "".join(f'<joint joint="h3_0_f{k}_{j}" coef="{LEV[j]}"/>' for j in range(3)) + "</fixed>" for k in range(4)) + "</tendon>"
    act = "".join(f'<position name="c{k}" tendon="t{k}" kp="{K_CORD}" forcerange="0 {F_STALL}" forcelimited="true" ctrlrange="-0.01 0.06"/>' for k in range(4))
    act += "".join(f'<position name="l{k}" joint="h3_0_l{k}" kp="2.0" kv="0.01" forcerange="-0.29 0.29" forcelimited="true" ctrlrange="-0.1 1.0"/>' for k in range(4))
    return mujoco.MjModel.from_xml_string(f"""<mujoco><option timestep="0.0004" gravity="0 0 0" integrator="implicitfast" cone="elliptic" impratio="10"/>
<default><geom solref="0.004 1" solimp="0.9 0.95 0.001" friction="0.8 0.01 0.001"/><joint limited="true"/></default>
<worldbody><body name="pod0"><geom type="sphere" size="0.005" contype="0" conaffinity="0"/>{hx}</body>
<body name="bar" pos="{HM.ZHEEL + r_bar + 0.0005:.5f} 0 0"><freejoint/>
<geom type="cylinder" fromto="0 0 -0.15 0 0 0.15" size="{r_bar}" mass="0.2" contype="0" conaffinity="4" condim="4" friction="0.8 0.004 0.0001"/></body>
</worldbody>{ten}<actuator>{act}</actuator></mujoco>""")
def cap(r_bar, w, rate):
    m = model(r_bar); d = mujoco.MjData(m); bid = m.body("bar").id; uc = [m.actuator(f"c{k}").id for k in range(4)]; ul = [m.actuator(f"l{k}").id for k in range(4)]
    for k in range(4):
        d.qpos[m.joint(f"h3_0_l{k}").qposadr[0]] = np.radians(47); d.qpos[m.joint(f"h3_0_f{k}_0").qposadr[0]] = np.radians(35)
    d.ctrl[ul] = np.radians(47); Lmax = sum(LEV[j] * np.radians(H.FOLD_MAX[j]) for j in range(3)); held = False; p0 = None; out = dict(limit=None, rated=None)
    while d.time < 7.0:
        t = d.time
        if t < 0.5: m.actuator_forcerange[uc, 1] = PRE * F_RATED; d.ctrl[uc] = LEV[0] * np.radians(35) + (t / 0.5) * (Lmax + 0.004)
        elif not held: d.ctrl[uc] = d.ten_length[:4] + PRE * F_RATED / K_CORD; m.actuator_forcerange[uc, 1] = F_STALL; held = True
        d.xfrc_applied[bid] = 0
        if t < 0.45: d.xfrc_applied[bid, 0] = -2.0
        if t >= 1.0:
            if p0 is None: p0 = d.xpos[bid].copy(); R0 = d.xmat[bid].reshape(3, 3).copy()
            mag = rate * (t - 1.0); d.xfrc_applied[bid] = np.asarray(w, float) * mag
            Re = d.xmat[bid].reshape(3, 3) @ R0.T; ang = np.degrees(np.arccos(np.clip((np.trace(Re) - 1) / 2, -1, 1)))
            if out["rated"] is None and d.actuator_force[uc].max() > F_RATED: out["rated"] = float(mag)
            if np.linalg.norm(d.xpos[bid] - p0) > 0.003 or ang > 5: out["limit"] = float(mag); break
        mujoco.mj_step(m, d)
    out["fold"] = [round(float(np.degrees(np.mean([d.qpos[m.joint(f"h3_0_f{k}_{j}").qposadr[0]] for k in range(4)])))) for j in range(3)]
    return out
tests = dict(pull_off_heel=([1, 0, 0, 0, 0, 0], 8.0), across=([0, 1, 0, 0, 0, 0], 8.0), slide_along_bar=([0, 0, 1, 0, 0, 0], 4.0), cantilever=([0, 0, 0, 0, 1, 0], 0.15),
             roll_about_bar=([0, 0, 0, 0, 0, 1], 0.05), twist_about_axis=([0, 0, 0, 1, 0, 0], 0.15))
res = {}
for r in (0.010, 0.014, 0.019):
    res[f"D{round(2000 * r)}"] = {n: cap(r, w, rate) for n, (w, rate) in tests.items()}
    print(f"D{round(2000 * r)}", {n: (round(v["limit"], 2) if v["limit"] else None, "rated from", round(v["rated"], 2) if v["rated"] else None) for n, v in res[f"D{round(2000 * r)}"].items()}, "fold", res[f"D{round(2000 * r)}"]["pull_off_heel"]["fold"], flush=True)
json.dump(res, open("hub_b_bench.json", "w"), indent=1)
