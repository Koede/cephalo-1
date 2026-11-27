#!/usr/bin/env python3
"""Centauri-1 MuJoCo model on the Hippos-1 legs v3 rev C CAD (2026-11-27).

Every leg body's mass, centre of mass and inertia comes from the rev C CAD solids (hippos_legs_cad.py, read-only snapshot in
legs_revC_snapshot/): bought parts at their datasheet mass spread over their envelope, printed parts at 1.20 g/cm3 x 0.55 fill.
Side view model of a bound: the body moves in x, z and pitch; the two legs of a pair move together (same commands), so the
model keeps all four legs with their true y offsets but the body cannot roll or yaw.  Locked for the leap: sling rocker,
sideways swing (abduction), twist (yaw), carpus (toggle lock), fetlock at 0.  Free: hoof ball joint in pitch (+-15 deg, passive).
Elbow / hock are slaved to the flex joint by equality constraints (the crossed Dyneema tendons, drum r 30 mm).
Body (chassis, Cephalo-2 docked, tail, battery) = 41.65 kg total - 17.47 kg legs; its centre of mass is placed so the robot's
centre of mass at the stand is where the Nov 20 study put it (87.5 mm ahead of the body origin, 450 mm up: 60 % on the fores);
its pitch inertia is an estimate (box 0.75 x 0.35 m: 1.4 kg m2), varied in the sensitivity runs.
    python3 build_model.py -> centauri1_revC.xml, model_info.json, leg_meshes.json (viewer)"""
import os, sys, json, io, contextlib
import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
SNAP = os.path.join(HERE, "legs_revC_snapshot")
sys.path.insert(0, SNAP)
with contextlib.redirect_stdout(io.StringIO()):
    import hippos_legs_cad as C
    import legs_cad_verify as V

D2R = np.pi / 180
M_TOTAL = 41.65                       # Centauri-1, rev C (hippos1-legs-v3-revC notes)
COM_ROBOT = np.array([87.5, 0.0, 450.0 + float(os.environ.get("CENT_DZ", "0"))])  # mm, body frame at the stand (study: 60 % weight on the fore pair; height est.)
I_BODY_PITCH = float(os.environ.get("CENT_IBODY", "1.4"))   # kg m2 about the body's own CoM (est.)
EXTRA_LEG = 0.045 + 0.055             # gas spring, toggle / lines / fasteners (est., as in hippos_legs_cad totals)

# which CAD nodes ride on which sim body
GROUP = {"rocker": "torso", "cradle": "torso", "yoke": "torso", "upper": "upper", "drive": "upper", "rodn": "upper",
         "middle": "middle", "lower": "lower", "cannon": "lower", "pastern": "lower", "hoofp": "hoof", "hoof": "hoof"}
JOINT_NODE = {"upper": "upper", "middle": "middle", "lower": "lower", "hoof": "hoofp"}
KINDS = ("fore", "hind")
SIDES = (1, -1)


def part_props(p):
    """mass (kg), CoM (mm, rest frame), inertia about CoM (kg mm2) of one part."""
    v, t = C.mesh_of(p["solid"])
    vol = p["solid"].volume()
    m = p["mass"] if p["mass"] is not None else (vol * C.PRINT_RHO * C.PRINT_FILL if p["printed"] else 0.0)
    if m <= 0 or vol <= 0:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    tm = trimesh.Trimesh(v, t, process=False)
    tm.density = m / vol
    return m, np.array(tm.center_mass), np.array(tm.moment_inertia)


def combine(items):
    m = sum(i[0] for i in items)
    if m <= 0:
        return 0.0, np.zeros(3), np.zeros((3, 3))
    c = sum(i[0] * i[1] for i in items) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in items:
        d = ci - c
        I += Ii + mi * (d @ d * np.eye(3) - np.outer(d, d))
    return m, c, I


def leg_bodies(kind):
    """Per sim body: frame origin (rest coords, mm), mass props in rest coords; plus the joint data."""
    L = C.FORE if kind == "fore" else C.HIND
    N = C.TREE[kind]
    T0 = C.fk(N, {})                                        # rest pose incl. the constant offsets (lower: -delta)
    acc = {g: [] for g in ("torso", "upper", "middle", "lower", "hoof")}
    for p in L.parts:
        mp = part_props(p)
        if mp[0] <= 0: continue
        # props in rest coords as authored; move them to the node's rest-pose placement
        R = T0[p["node"]][:3, :3]; t = T0[p["node"]][:3, 3]
        acc[GROUP[p["node"]]].append((mp[0], R @ mp[1] + t, R @ mp[2] @ R.T))
    acc["upper"].append((EXTRA_LEG, np.array([0.0, 0.0, -100.0]), np.zeros((3, 3))))
    out = {}
    for g, items in acc.items():
        m, c, I = combine(items)
        if g == "torso":
            org = np.zeros(3)
        else:
            nd = JOINT_NODE[g]; o = np.asarray(N[nd][1], float)
            org = (T0[nd] @ np.r_[o, 1.0])[:3]
        out[g] = dict(m=m, c=c, I=I, org=org)
    Rh = T0["hoofp"][:3, :3]; out["hoof"]["const"] = float(np.degrees(np.arctan2(Rh[0, 2], Rh[0, 0])))   # rest tilt of the hoof (deg about y)
    return out, L, T0


def fmt(a):
    return " ".join("%.6g" % x for x in np.ravel(a))


def gas_table(L):
    """Gas strut length (mm) vs flex angle q1 (deg): anchor U on the upper segment, M on the middle (rotates +q1 about zS)."""
    g = L.mech["gas"]; zS = L.joints["zS"]; U = np.array(g["U"]); Mr = np.array(g["M"]) - [0, 0, zS]
    q = np.arange(30.0, 160.1, 0.5); a = q * D2R
    Mx = Mr[0] * np.cos(a) + Mr[2] * np.sin(a); Mz = -Mr[0] * np.sin(a) + Mr[2] * np.cos(a) + zS
    Ls = np.hypot(Mx - U[0], Mz - U[2])
    return dict(q=q.tolist(), L=Ls.tolist(), Le=g["Le"], Lc=g["Lc"])


def build(ibody=I_BODY_PITCH, out_xml="centauri1_revC.xml", info_out="model_info.json"):
    info = dict(legs={}, gas={})
    legs = {}
    for k in KINDS:
        b, L, T0 = leg_bodies(k)
        legs[k] = b
        info["gas"][k] = gas_table(L)
        info["legs"][k] = dict(joints={kk: (vv.tolist() if hasattr(vv, "tolist") else vv) for kk, vv in L.joints.items()},
                               masses={g: round(v["m"], 4) for g, v in b.items()}, pivot=C.LEG[k]["pivot"].tolist(),
                               delta=C.LEG[k]["delta"], range=V.RANGE[k], stand=C.STAND[k], hoof_const=b["hoof"]["const"])
    m_legs = 2 * sum(v["m"] for v in legs["fore"].values()) + 2 * sum(v["m"] for v in legs["hind"].values())
    m_body = M_TOTAL - m_legs
    # leg CoM at the stand pose (q0, q1 from C.STAND) to place the body CoM
    tot = np.zeros(3); mtot = 0.0
    for k in KINDS:
        N = C.TREE[k]; p = dict(C.STAND[k])
        T = C.fk(N, p); T0 = C.fk(N, {})
        piv = C.LEG[k]["pivot"]
        for g, v in legs[k].items():
            if v["m"] <= 0: continue
            nd = {"torso": "yoke", "upper": "upper", "middle": "middle", "lower": "lower", "hoof": "hoofp"}[g]
            Tr = T[nd] @ np.linalg.inv(T0[nd])
            c = (Tr @ np.r_[v["c"], 1])[:3] + [piv[0], 0, piv[2]]
            tot += 2 * v["m"] * c; mtot += 2 * v["m"]
    c_body = (COM_ROBOT * M_TOTAL - tot) / m_body; c_body[1] = 0.0
    info.update(m_legs=m_legs, m_body=m_body, c_body_mm=c_body.tolist(), i_body_pitch=ibody, m_total=M_TOTAL,
                com_robot_mm=COM_ROBOT.tolist())

    A = C.S.ACT
    ACT = dict(fore=dict(q0=("RobStride 03", 1.0), q1=("RobStride 04", 22 / 46)), hind=dict(q0=("RobStride 06", 1.0), q1=("RobStride 06", 1.0)))
    arm = {k: {j: A[n]["j_rotor"] * A[n]["gear"] ** 2 * r ** 2 for j, (n, r) in ACT[k].items()} for k in KINDS}
    info["act"] = {k: {j: dict(name=n, ratio=r, armature=arm[k][j]) for j, (n, r) in ACT[k].items()} for k in KINDS}

    X = []
    X.append('<mujoco model="centauri1_revC">')
    X.append('  <compiler angle="degree" autolimits="true"/>')
    X.append('  <option timestep="0.001" integrator="implicitfast" gravity="0 0 -9.81"><flag warmstart="enable"/></option>')
    X.append('  <default><geom contype="0" conaffinity="0" group="1"/><joint damping="0.05"/></default>')
    X.append('  <worldbody>')
    X.append('    <geom name="floor" type="plane" size="20 2 0.1" contype="1" conaffinity="1" friction="0.7 0.005 0.0001" solref="0.004 1" solimp="0.95 0.99 0.001"/>')
    X.append('    <body name="torso" pos="0 0 0">')
    X.append('      <joint name="rootx" type="slide" axis="1 0 0" pos="%s" damping="0"/>' % fmt(COM_ROBOT * 1e-3))
    X.append('      <joint name="rootz" type="slide" axis="0 0 1" pos="%s" damping="0"/>' % fmt(COM_ROBOT * 1e-3))
    X.append('      <joint name="rooty" type="hinge" axis="0 1 0" pos="%s" damping="0"/>' % fmt(COM_ROBOT * 1e-3))
    # body inertial: body + the four leg torso-group masses (rocker, cradle, yoke)
    items = [(m_body, c_body, np.diag([0.8e6, ibody * 1e6, 1.4e6]))]
    for k in KINDS:
        piv = C.LEG[k]["pivot"]; v = legs[k]["torso"]
        for s in SIDES:
            c = v["c"] * [1, s, 1] + [piv[0], s * piv[1], piv[2]]; I = v["I"].copy()
            if s < 0: I[0, 1] = I[1, 0] = -I[0, 1]; I[1, 2] = I[2, 1] = -I[1, 2]
            items.append((v["m"], c, I))
    m, c, I = combine(items)
    X.append('      <inertial pos="%s" mass="%.5f" fullinertia="%s"/>' % (fmt(c * 1e-3), m, fmt(I[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]] * 1e-6)))
    X.append('      <geom name="bodybox" type="box" size="0.36 0.13 0.09" pos="0.05 0 0.52" contype="1" conaffinity="0" rgba=".8 .8 .8 .3"/>')
    eq = []; acts = []
    for k in KINDS:
        piv = C.LEG[k]["pivot"] * 1e-3; b = legs[k]; rg = V.RANGE[k]
        jl = dict(q0=rg["q0"], q1=rg["q1"])
        for s in SIDES:
            tag = "%s%s" % (k, "L" if s > 0 else "R")
            def P(v):
                return np.array([v[0], s * v[1], v[2]]) * 1e-3
            def inert(v, org):
                I = v["I"].copy()
                if s < 0: I[0, 1] = I[1, 0] = -I[0, 1]; I[1, 2] = I[2, 1] = -I[1, 2]
                return '<inertial pos="%s" mass="%.5f" fullinertia="%s"/>' % (fmt(P(v["c"] - org)), max(v["m"], 1e-4), fmt(I[[0, 1, 2, 0, 0, 1], [0, 1, 2, 1, 2, 2]] * 1e-6))
            ind = "      "
            X.append(ind + '<body name="%s_upper" pos="%s">' % (tag, fmt(piv * [1, s, 1])))
            X.append(ind + '  ' + inert(b["upper"], b["upper"]["org"]))
            X.append(ind + '  <joint name="%s_q0" axis="0 -1 0" range="%g %g" solreflimit="0.005 1" armature="%.5f"/>' % (tag, jl["q0"][0], jl["q0"][1], arm[k]["q0"]))
            X.append(ind + '  <site name="%s_piv" pos="0 0 0"/>' % tag)
            o = b["middle"]["org"]
            X.append(ind + '  <body name="%s_middle" pos="%s">' % (tag, fmt(P(o - b["upper"]["org"]))))
            X.append(ind + '    ' + inert(b["middle"], o))
            X.append(ind + '    <joint name="%s_q1" axis="0 1 0" range="%g %g" solreflimit="0.005 1" armature="%.5f"/>' % (tag, jl["q1"][0], jl["q1"][1], arm[k]["q1"]))
            o2 = b["lower"]["org"]
            X.append(ind + '    <body name="%s_lower" pos="%s">' % (tag, fmt(P(o2 - o))))
            X.append(ind + '      ' + inert(b["lower"], o2))
            X.append(ind + '      <joint name="%s_sl" axis="0 -1 0" range="-200 200" damping="0.02"/>' % tag)
            o3 = b["hoof"]["org"]
            X.append(ind + '      <body name="%s_hoof" pos="%s">' % (tag, fmt(P(o3 - o2))))
            X.append(ind + '        ' + inert(b["hoof"], o3))
            X.append(ind + '        <joint name="%s_hp" axis="0 1 0" range="-15 15" damping="0.3" stiffness="0.5"/>' % tag)
            sole = C.HOOF_H * 1e-3; cst = b["hoof"]["const"] * D2R
            sp = np.array([np.sin(cst) * (-sole + 0.004), 0, np.cos(cst) * (-sole + 0.004)])
            X.append(ind + '        <geom name="%s_sole" type="box" size="0.026 0.022 0.004" pos="%s" euler="0 %.4f 0" contype="0" conaffinity="1" condim="3" friction="0.7 0.005 0.0001" solref="0.006 1" solimp="0.9 0.98 0.002" rgba=".9 .5 .2 1"/>' % (tag, fmt(sp), b["hoof"]["const"]))
            X.append(ind + '        <site name="%s_ball" pos="0 0 0"/>' % tag)
            X.append(ind + '      </body>')
            X.append(ind + '    </body>')
            X.append(ind + '  </body>')
            X.append(ind + '</body>')
            eq.append('    <joint name="%s_tendon" joint1="%s_sl" joint2="%s_q1" polycoef="0 1 0 0 0" solref="0.002 1"/>' % (tag, tag, tag))
            for j in ("q0", "q1"):
                n, r = ACT[k][j]; pk = A[n]["peak"] / r * (C.S.BELT_EFF if r != 1 else 1.0)
                acts.append('    <motor name="%s_%s_m" joint="%s_%s" ctrlrange="%.2f %.2f" gear="1"/>' % (tag, j, tag, j, -pk, pk))
    X.append('    </body>')
    X.append('  </worldbody>')
    X.append('  <equality>'); X += eq; X.append('  </equality>')
    X.append('  <actuator>'); X += acts; X.append('  </actuator>')
    X.append('  <sensor>')
    for k in KINDS:
        for s in SIDES:
            tag = "%s%s" % (k, "L" if s > 0 else "R")
            X.append('    <touch name="%s_touch" site="%s_ball"/>' % (tag, tag)) if False else None
    X.append('  </sensor>')
    X.append('</mujoco>')
    open(os.path.join(HERE, out_xml), "w").write("\n".join(x for x in X if x is not None))
    json.dump(info, open(os.path.join(HERE, info_out), "w"), indent=1, default=float)
    return info


def export_meshes():
    """Per leg kind, per sim body: one merged mesh in the body's own frame (mm, int), for the three.js replay."""
    out = {}
    for k in KINDS:
        L = C.FORE if k == "fore" else C.HIND; N = C.TREE[k]; T0 = C.fk(N, {})
        b, _, _ = leg_bodies(k)
        groups = {}
        for p in L.parts:
            g = GROUP[p["node"]]
            if g == "torso" and p["node"] != "yoke": pass
            v, t = C.mesh_of(p["solid"])
            R = T0[p["node"]][:3, :3]; tr = T0[p["node"]][:3, 3]
            v = v @ R.T + tr - b[g]["org"]
            col = p["col"]
            groups.setdefault(g, {}).setdefault(col, []).append((v, t))
        out[k] = {}
        for g, cols in groups.items():
            out[k][g] = []
            for col, lst in cols.items():
                vs, ts, n0 = [], [], 0
                for v, t in lst:
                    vs.append(v); ts.append(t + n0); n0 += len(v)
                V_ = np.round(np.vstack(vs), 1); T_ = np.vstack(ts)
                out[k][g].append(dict(col=col, v=V_.ravel().tolist(), t=T_.ravel().astype(int).tolist()))
        # gas strut and tendon endpoints for drawing
        out[k]["_org"] = {g: b[g]["org"].tolist() for g in b}
        out[k]["_gas"] = dict(U=L.mech["gas"]["U"], M=L.mech["gas"]["M"])
    json.dump(out, open(os.path.join(HERE, "leg_meshes.json"), "w"), separators=(",", ":"))


if __name__ == "__main__":
    xo = os.environ.get("CENT_XML", "centauri1_revC.xml")
    info = build(out_xml=xo, info_out="model_info.json" if xo == "centauri1_revC.xml" else xo.replace(".xml", "_info.json"))
    print("legs %.2f kg, body %.2f kg, body CoM %s mm" % (info["m_legs"], info["m_body"], np.round(info["c_body_mm"], 1)))
    for k in KINDS:
        print(k, info["legs"][k]["masses"], {j: round(v["armature"], 4) for j, v in info["act"][k].items()})
    if "--mesh" in sys.argv:
        export_meshes()
