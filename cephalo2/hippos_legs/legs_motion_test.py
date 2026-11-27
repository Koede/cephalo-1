#!/usr/bin/env python3
"""Motion test of the rev B legs on the whole robot: every motion replayed through the CAD joint tree.
Per frame (every frame for kinematics, every 2nd for distances):
  1 planted hooves: the sole's contact point from the full 3D tree against the fixed ground point (slip, height)
  2 joint limits, frame-to-frame continuity, loop closure, joint speeds against the motors at 39.6 V, servo line rates
  3 mechanism closure: push-rod length, gas-spring length inside its stroke, toggle reach and lock in stance, tendon wrap left
  4 distances: inside each leg (legs_cad_verify rules), leg to leg, leg to neck receiver, octopus and body shell, ground
    python3 legs_motion_test.py -> legs_motion_test.json"""
import os, sys, json, itertools
import numpy as np
import hippos_legs_cad as C
import legs_cad_verify as V
import legs_motion as M
from manifold3d import Manifold
HERE = M.HERE; D = M.D; S = M.S; KINDS = M.KINDS; SIDES = M.SIDES
STEP = 2


def body4(b):
    c, s = np.cos(b[2] / D), np.sin(b[2] / D)
    Rb = np.array([[c, 0, -s, 0], [0, 1, 0, 0], [s, 0, c, 0], [0, 0, 0, 1.0]])
    A = np.eye(4); A[:3, 3] = [M.REF[0] + b[0], 0, M.REF[1] + b[1]]; B = np.eye(4); B[:3, 3] = [-M.REF[0], 0, -M.REF[1]]
    return A @ Rb @ B


def leg4(kind, side):
    piv = C.LEG[kind]["pivot"]; A = np.eye(4); A[:3, 3] = [piv[0], side * piv[1], piv[2]]; A[1, 1] = side
    return A


def params(l, extra=None):
    p = dict(q0=l["q0"], q1=l["q1"], fold=l["fold"], hp=l["hp"]); p.update(extra or {}); return p


# ------------------------------------------------------------------ body-side solids (mm, body frame at the stand)
def neck_block():
    cc = np.array(M.C3.cc) * 1e3; n = np.array(M.C3.n_up); hw = M.C3.SHEATH_HW * 1e3; Ln = M.C3.SHEATH_L * 1e3
    t = np.array([n[1], -n[0]])                                    # in-plane normal to the collar axis (x, z)
    Mx = np.eye(4); Mx[:3, 0] = [t[0], 0, t[1]]; Mx[:3, 1] = [0, 1, 0]; Mx[:3, 2] = [n[0], 0, n[1]]; Mx[:3, 3] = [cc[0], 0, cc[1]]
    return Manifold.cube([2 * hw, 2 * hw, Ln]).translate([-hw, -hw, -Ln]).transform(Mx[:3, :4])


def shell_solid():
    P = M.C4.shell_points() * 1e3; parts = []
    for i in range(P.shape[0] - 1):
        pts = np.vstack([P[i], P[i + 1]])
        if np.ptp(pts[:, 1]) < 1 or np.ptp(pts[:, 2]) < 1: continue
        parts.append(Manifold.hull_points(pts))
    return C.union(parts)


def octo_points():
    npz = os.path.join(HERE, "deps", "scene2_notail.npz")
    if not os.path.exists(npz): return None
    Dz = np.load(npz); return np.vstack([Dz[k] for k in Dz.files if k.startswith("o") and k.endswith("_v")]) * 1e3


def robot_items(fr, extra=None):
    """World solids of the four legs for one frame: [(leg index, name, node, solid, lo, hi)]."""
    Bm = body4(fr["b"]); out = []
    for i, l in enumerate(fr["l"]):
        items, T = V.posed(KINDS[i], params(l, (extra or {}).get(i)))
        W = (Bm @ leg4(KINDS[i], SIDES[i]))[:3, :4]
        for nm, node, m, _, _ in items:
            w = m.transform(W); bb = w.bounding_box(); out.append((i, nm, node, w, np.array(bb[:3]), np.array(bb[3:])))
    return out, Bm


def robot_check(fr, env, extra=None, search=12.0, body_checks=True):
    """-> dict of minima for one frame (mm)."""
    items, Bm = robot_items(fr, extra); r = dict(leg_leg=(search, ""), neck=(search, ""), shell=(search, ""), ground=(99.0, ""), octo=(99.0, ""), shell_hits=[])
    by = {}
    for it in items: by.setdefault(it[0], []).append(it)
    for a, b in itertools.combinations(range(4), 2):
        for (_, na, _, ma, la, ha) in by[a]:
            for (_, nb, _, mb, lb, hb) in by[b]:
                lim = r["leg_leg"][0]
                if np.any(la - hb > lim) or np.any(lb - ha > lim): continue
                g = ma.min_gap(mb, lim)
                if g < lim: r["leg_leg"] = (float(g), "%s %d: %s | %s %d: %s" % (KINDS[a], a, na[:30], KINDS[b], b, nb[:30]))
    nb_w = env["neck"].transform(Bm[:3, :4]); sh_w = env["shell"].transform(Bm[:3, :4])
    bn = nb_w.bounding_box(); bs = sh_w.bounding_box()
    for (i, nm, node, m, lo, hi) in items:
        if lo[2] < r["ground"][0] and "hoof shoe" not in nm: r["ground"] = (float(lo[2]), "%s %d: %s" % (KINDS[i], i, nm[:40]))
        if "hoof shoe" in nm and lo[2] < -0.05: r["ground"] = (float(lo[2]), "%s %d: %s (below the ground)" % (KINDS[i], i, nm[:30]))
        if node == "rocker": continue                               # brackets and the abduction motor are bolted to the body
        lim = r["neck"][0]
        if not (np.any(lo - np.array(bn[3:]) > lim) or np.any(np.array(bn[:3]) - hi > lim)):
            g = m.min_gap(nb_w, lim)
            if g < lim: r["neck"] = (float(g), "%s %d: %s" % (KINDS[i], i, nm[:40]))
        if body_checks and not (np.any(lo - np.array(bs[3:]) > 3.0) or np.any(np.array(bs[:3]) - hi > 3.0)):
            g = m.min_gap(sh_w, 3.0)
            if g < r["shell"][0]: r["shell"] = (float(g), "%s %d: %s" % (KINDS[i], i, nm[:40]))
            if g <= 0: r["shell_hits"].append((i, nm, m))
    if body_checks and env["octo"] is not None:
        from scipy.spatial import cKDTree
        if "tree" not in env: env["tree"] = cKDTree(env["octo"])
        Bi = np.linalg.inv(Bm)
        for (i, nm, node, m, lo, hi) in items:
            if node in ("rocker",): continue
            Vv = np.asarray(m.to_mesh().vert_properties)[:, :3]; Vb = (np.c_[Vv, np.ones(len(Vv))] @ Bi.T)[:, :3]
            d = env["tree"].query(Vb)[0].min()
            if d < r["octo"][0]: r["octo"] = (float(d), "%s %d: %s" % (KINDS[i], i, nm[:40]))
    return r


def contact_error(fr):
    """Sole contact point from the full tree against the planted point, for the stance legs of one frame."""
    Bm = body4(fr["b"]); out = []
    for i, l in enumerate(fr["l"]):
        if not l["st"]: continue
        kind = KINDS[i]; L = C.FORE if kind == "fore" else C.HIND; ball = L.joints["ball"]; zF = ball[2] - C.HOOF_H
        T = C.fk(C.TREE[kind], params(l)); W = Bm @ leg4(kind, SIDES[i]) @ T["hoof"]
        pt = W @ np.array([ball[0] + l["edge"] * M.R_HOOF, 0, zF, 1.0])
        tgt = np.array([l["F"][0] + l["edge"] * M.R_HOOF, 0.0])
        out.append((abs(pt[0] - tgt[0]), abs(pt[2] - tgt[1])))
        # the whole sole must be on or above the ground
        low = min((W @ np.array([ball[0] + s * M.R_HOOF, 0, zF, 1.0]))[2] for s in (-1, 0, 1))
        out.append((0.0, max(0.0, -low)))
    return out


def mechanism(kind, l):
    """Closure quantities of one leg pose."""
    L = C.FORE if kind == "fore" else C.HIND; T = C.fk(C.TREE[kind], params(l)); r = {}
    g = L.mech["gas"]; a = (T["upper"] @ np.array(g["U"] + [1.0]))[:3]; b = (T["middle"] @ np.array(g["M"] + [1.0]))[:3]; r["gas"] = float(np.linalg.norm(a - b))
    if kind == "hind":
        c0 = L.joints["c0"]; zS = L.joints["zS"]
        a = (T["drive"] @ np.array([c0[0], 0, c0[2], 1.0]))[:3]; b = (T["middle"] @ np.array([c0[0], 0, zS + c0[2], 1.0]))[:3]; r["rod"] = float(np.linalg.norm(a - b))
        r["rod_angle"] = float(abs(((l["q1"] - 20.0 + 90) % 180) - 90))            # angle between crank and rod (0 = dead point)
    for t in L.toggles:
        A_, B_, k, ln = V.knee_of(L, T, t, params(l)); r["toggle_reach"] = float(2 * t["l"] - ln)
        r["locked"] = bool(l["fold"] < t["lock_below"])
    return r


def forces(mot, ev):
    """Loads in the passive mechanisms, from the ground force on each stance leg (est.: weight share / duty, x 1.5 for the peak)."""
    W = ev["total"] * 9.81; share = dict(fore=0.6 / 2, hind=0.4 / 2); out = {}
    for name in ("walk", "trot", "gallop"):
        fr = mot[name]["frames"]; duty = S.GAITS[name]["duty"]; r = dict(grf_peak_N={}, toggle_N=0.0, toggle_tension_N=0.0, tendon_N={}, buffer_N={}, fetlock_deg={})
        for kind in ("fore", "hind"):
            g = C.LEG[kind]; G = 1.5 * W * share[kind] / duty; r["grf_peak_N"][kind] = G; tn = 0.0; bf = 0.0
            for f in fr:
                for i, l in enumerate(f["l"]):
                    if KINDS[i] != kind or not l["st"]: continue
                    a = l["q0"] / D; J2 = g["L1"] * M.u(a) + g["L2"] * M.u(a - l["q1"] / D); col = M.u(a + g["delta"] / D)
                    cx = l["F"][0] + l["edge"] * M.R_HOOF
                    ex = M.to_world(kind, J2, f["b"])[0]; tn = max(tn, G * abs(cx - ex) / C.R_DRUM)
                    L = C.FORE if kind == "fore" else C.HIND
                    T = C.fk(C.TREE[kind], params(l)); Wm = body4(f["b"]) @ leg4(kind, SIDES[i])
                    fetw = (Wm @ T["pastern"] @ np.array([L.joints["fet"][0], 0, L.joints["fet"][2], 1.0]))[0]
                    ballw = (Wm @ T["pastern"] @ np.array([L.joints["ball"][0], 0, L.joints["ball"][2], 1.0]))[0]
                    bf = max(bf, G * max(ballw - fetw, 0) / C.BUF_ARM)
                    if kind == "fore":
                        cw = M.to_world(kind, J2 + g["LC"] * col, f["b"])[0]; m_ = G * (cw - cx)          # > 0: folds the carpus
                        r["toggle_N"] = max(r["toggle_N"], m_ / C.TOG_E); r["toggle_tension_N"] = max(r["toggle_tension_N"], -m_ / C.TOG_E)
            r["tendon_N"][kind] = tn; r["buffer_N"][kind] = bf
            sig = bf / C.BUF_AREA; r.setdefault("buffer_MPa", {})[kind] = sig; r["fetlock_deg"][kind] = float(np.degrees(sig / 26.0 * C.BUF_T / C.BUF_ARM))   # TPU 95A, 26 MPa
        out[name] = r
    rows = {(r[1], r[2], r[3]): r for r in ev["rows"]}
    pk = lambda leg: max(r[5] for r in ev["rows"] if r[1] == leg and r[3] == "joint")
    out["shoulder_belt_peak_N"] = pk("front") / (C.FORE.mech["belt"]["r_joint"] / 1e3)
    out["stifle_rod_peak_N"] = pk("rear") / (0.040 * np.sin(np.radians(30.0)))
    for kind, leg in (("fore", "front"), ("hind", "rear")):
        L = C.FORE if kind == "fore" else C.HIND; g = L.mech["gas"]; q1 = C.STAND[kind]["q1"]
        T = C.fk(C.TREE[kind], dict(q1=q1)); a = (T["upper"] @ np.array(g["U"] + [1.0]))[:3]; b = (T["middle"] @ np.array(g["M"] + [1.0]))[:3]
        j = (T["upper"] @ np.array([0, 45.0, L.joints["zS"], 1.0]))[:3]; ln = np.linalg.norm(b - a); arm = abs(np.cross(b - j, (b - a) / ln)[1])
        F = 400.0 * (1 + 0.30 * (g["Le"] - ln) / (g["Le"] - g["Lc"]))                 # 400 N spring, about 30 % progression over the stroke
        st_full = ev["stand_joint_Nm"][leg]
        out[kind + "_gas_spring"] = dict(stand_len_mm=ln, lever_mm=arm, force_N=F, torque_Nm=F * arm / 1e3, stand_joint_Nm=st_full, offload=F * arm / 1e3 / st_full)
    return out


def run():
    mot, ev, crouch = M.build()
    env = dict(neck=neck_block(), shell=shell_solid(), octo=octo_points())
    w_free = lambda name: S.ACT[name]["w_free_48"] * S.BUS_V / 48.0 * 6.0        # deg/s at 39.6 V
    spd_lim = dict(fore=dict(pivot=w_free(M.ACTS["hip"]["front"]), joint=w_free(M.ACTS["knee"]["front"]) / M.ACTS["ratio"]["front"]),
                   hind=dict(pivot=w_free(M.ACTS["hip"]["rear"]), joint=w_free(M.ACTS["knee"]["rear"])))
    out = dict(mass=ev["total"], jump_mm=ev["jump_mm"], loads=ev["worst"], load_fails=ev["fails"], crouch=crouch, speed_limit_deg_s=spd_lim, motions={})
    hits = {}
    for name, m in mot.items():
        fr = m["frames"]; n = len(fr); dt = m["dt"]; R = dict(frames=n, dt=dt)
        # 1 contact
        ce = [e for f in fr for e in contact_error(f)]
        R["slip_mm"] = float(max((e[0] for e in ce), default=0)); R["height_err_mm"] = float(max((e[1] for e in ce), default=0))
        R["ik_err_mm"] = float(max(l["err"] for f in fr for l in f["l"]))
        # 2 limits, continuity, speeds
        lim_bad = []; R["joint"] = {}
        for i in range(4):
            kind = KINDS[i]; rg = V.RANGE[kind]
            for j in ("q0", "q1", "fold"):
                if j not in rg: continue
                v = np.array([f["l"][i][j] for f in fr]); R["joint"].setdefault(kind + " " + j, [float(v.min()), float(v.max())])
                R["joint"][kind + " " + j] = [min(R["joint"][kind + " " + j][0], float(v.min())), max(R["joint"][kind + " " + j][1], float(v.max()))]
                if v.min() < rg[j][0] - 1e-6 or v.max() > rg[j][1] + 1e-6: lim_bad.append((kind, j, float(v.min()), float(v.max()), rg[j]))
        R["limit_violations"] = lim_bad
        step = {}; spd = {}
        for i in range(4):
            kind = KINDS[i]
            for j, key in (("q0", "pivot"), ("q1", "joint"), ("fold", "fold"), ("hp", "hoof")):
                v = np.array([f["l"][i][j] for f in fr]); dv = np.diff(np.r_[v, v[:1]] if m["cyclic"] else v)
                if len(dv):
                    step[kind + " " + key] = max(step.get(kind + " " + key, 0), float(np.abs(dv).max()))
                    spd[kind + " " + key] = max(spd.get(kind + " " + key, 0), float(np.abs(dv).max() / dt))
        R["max_step_deg"] = step; R["max_speed_deg_s"] = spd
        R["speed_margin"] = {k: float(spd_lim[k.split()[0]][k.split()[1]] / max(v, 1e-9)) for k, v in spd.items() if k.split()[1] in ("pivot", "joint")}
        # 3 mechanisms
        mech = dict(gas=[9e9, 0], rod_dev=0.0, rod_angle=90.0, toggle_reach=9e9, unlocked_in_stance=0)
        for f in fr:
            for i, l in enumerate(f["l"]):
                q = mechanism(KINDS[i], l); L = C.FORE if KINDS[i] == "fore" else C.HIND
                mech["gas"] = [min(mech["gas"][0], q["gas"]), max(mech["gas"][1], q["gas"])]
                if "rod" in q: mech["rod_dev"] = max(mech["rod_dev"], abs(q["rod"] - L.g["L1"])); mech["rod_angle"] = min(mech["rod_angle"], q["rod_angle"])
                if "toggle_reach" in q:
                    mech["toggle_reach"] = min(mech["toggle_reach"], q["toggle_reach"])
                    if l["st"] and not q["locked"]: mech["unlocked_in_stance"] += 1
        R["mechanism"] = mech
        # 4 distances
        dist = dict(in_leg=(99.0, ""), in_leg_bad=[], leg_leg=(99.0, ""), neck=(99.0, ""), shell=(99.0, ""), ground=(99.0, ""), octo=(99.0, ""), roll_deg=[0.0, 0.0])
        for k in ([] if os.environ.get('FAST') else range(0, n, STEP if n > 1 else 1)):
            f = fr[k]
            for i, l in enumerate(f["l"]):
                w, bad, el = V.check(KINDS[i], params(l))
                if w[0] < dist["in_leg"][0]: dist["in_leg"] = (float(w[0]), "%s %d f%d: %s | %s" % (KINDS[i], i, k, w[1][:30], w[2][:30]))
                for b in bad: dist["in_leg_bad"].append((name, k, i, b))
                dist["roll_deg"] = [min(dist["roll_deg"][0], l["roll"]), max(dist["roll_deg"][1], l["roll"])]
            rc = robot_check(f, env, body_checks=(k % 12 == 0))
            for key in ("leg_leg", "neck", "shell", "ground", "octo"):
                if rc[key][0] < dist[key][0]: dist[key] = (rc[key][0], "f%d %s" % (k, rc[key][1]))
            for i, nm, mm in rc["shell_hits"]:
                hits.setdefault((KINDS[i], nm), []).append((name, k))
        R["distance"] = dist
        out["motions"][name] = R
        print("%-7s slip %.1e mm  height %.1e  limits %s  steps %s" % (name, R["slip_mm"], R["height_err_mm"], "ok" if not lim_bad else lim_bad, {k: round(v, 1) for k, v in step.items()}), flush=True)
        print("        speed margin %s" % {k: round(v, 2) for k, v in R["speed_margin"].items()})
        print("        mech %s" % {k: (np.round(v, 3).tolist() if not isinstance(v, int) else v) for k, v in mech.items()})
        print("        in-leg %.2f (%s) bad %d | leg-leg %.1f %s | neck %.1f %s | ground %.2f %s | octo %.1f | shell %.2f %s | roll %s" % (
            dist["in_leg"][0], dist["in_leg"][1], len(dist["in_leg_bad"]), dist["leg_leg"][0], dist["leg_leg"][1][:60], dist["neck"][0], dist["neck"][1][:40],
            dist["ground"][0], dist["ground"][1][:40], dist["octo"][0], dist["shell"][0], dist["shell"][1][:44], np.round(dist["roll_deg"], 1).tolist()), flush=True)
        for b in dist["in_leg_bad"][:6]: print("        UNDER THE RULE", b)
    out["forces"] = forces(mot, ev); print("forces:", json.dumps(out["forces"], default=float)[:1500])
    # gallop with all legs twisted / swung to their limits: leg to leg only
    gt = (99.0, "")
    for k in (() if os.environ.get('FAST') else range(0, len(mot["gallop"]["frames"]), 6)):
        for yaw, abd in ((-25, -10), (-25, 10), (25, -10), (25, 10)):
            rc = robot_check(mot["gallop"]["frames"][k], env, {i: dict(yaw=yaw, abd=abd) for i in range(4)}, body_checks=False)
            if rc["leg_leg"][0] < gt[0]: gt = (rc["leg_leg"][0], "f%d yaw %d abd %d: %s" % (k, yaw, abd, rc["leg_leg"][1]))
    out["gallop_twisted_leg_leg"] = gt; print("gallop, all legs at the twist / swing limits, leg to leg:", gt)
    # twist and sideways swing at the stand, all four legs together (inward and outward)
    st = mot["stand"]["frames"][0]; tw = dict(leg_leg=(99.0, ""), neck=(99.0, ""), octo=(99.0, ""), shell=(99.0, ""))
    for yaw in (() if os.environ.get('FAST') else (-25, 0, 25)):
        for abd in (-10, 0, 10):
            ex = {i: dict(yaw=yaw, abd=abd) for i in range(4)}
            rc = robot_check(st, env, ex)
            for key in tw:
                if rc[key][0] < tw[key][0]: tw[key] = (rc[key][0], "yaw %d abd %d: %s" % (yaw, abd, rc[key][1]))
            for i, nm, mm in rc["shell_hits"]: hits.setdefault((KINDS[i], nm), []).append(("stand twist", yaw, abd))
    out["stand_twist"] = tw; print("stand, all legs twisted / swung together:", {k: (round(v[0], 1), v[1][:70]) for k, v in tw.items()})
    out["shell_overlaps"] = [dict(kind=k[0], part=k[1], cases=len(v)) for k, v in hits.items()]
    print("parts that enter today's body shell:", sorted({k[0] + ": " + k[1][:40] for k in hits}))
    json.dump(out, open(os.path.join(HERE, "legs_motion_test.json"), "w"), indent=1, default=lambda o: float(o) if isinstance(o, (np.floating, np.integer)) else str(o))
    return out


if __name__ == "__main__":
    run()
