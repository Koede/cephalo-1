#!/usr/bin/env python3
"""Animation frames for the legs v3 study (three.js page and the side-view sheet).
For every option and motion: per frame, the body pose (x, z, pitch) and every leg's joint points in world metres.
    python3 legs_v3_frames.py -> frames.json"""
import os, json, importlib.util, io, contextlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("S", os.path.join(HERE, "legs_v3_study.py")); S = importlib.util.module_from_spec(spec)
with contextlib.redirect_stdout(io.StringIO()):
    spec.loader.exec_module(S)
u = S.u
LEGS = [("front", 1), ("front", -1), ("rear", 1), ("rear", -1)]          # (leg, side y sign): FL, FR, RL, RR
NF = 72


def smooth(s):
    s = np.clip(s, 0, 1); return s * s * (3 - 2 * s)


def chain(key, leg, q, fold=0.0):
    """Joint points in the leg frame.  B's foreleg: the carpus (CARPUS along the column) folds by `fold` rad, hoof back."""
    o = S.OPT[key]; m = o["legs"][leg]; P = m.pts(q)
    if o["carpus"] and leg == "front" and len(P) == 4:
        d = P[3] - P[2]; L = np.linalg.norm(d); dn = d / L
        c = P[2] + dn * S.CARPUS * L
        a = np.arctan2(dn[0], -dn[1]) - fold                              # lower column swings back (hoof toward the elbow)
        f = c + (1 - S.CARPUS) * L * u(a)
        return [P[0], P[1], P[2], c, f]
    return P


def to_world(key, leg, side, pts, body):
    """Leg frame -> world: pivot in the body, front legs mirrored when the option says so, body pitch about its CoM point."""
    o = S.OPT[key]; sx = o["sx"][leg]; piv = o["pivot"][leg]
    bx, bz, th = body
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    ref = np.array([S.X_COM, 0.30])
    out = []
    for p in pts:
        b = piv + np.array([sx * p[0], p[1]])                              # body frame (x, z) at the design height
        w = R @ (b - ref) + ref + np.array([bx, bz])
        out.append([round(float(w[0]), 4), round(float(side * S.Y_TRACK), 4), round(float(w[1]), 4)])
    return out


def ik_world(key, leg, foot_w, body):
    """Leg angles for a hoof at a world point (x, z), body pose given."""
    o = S.OPT[key]; sx = o["sx"][leg]; piv = o["pivot"][leg]; m = o["legs"][leg]
    bx, bz, th = body
    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); ref = np.array([S.X_COM, 0.30])
    b = R.T @ (np.asarray(foot_w) - np.array([bx, bz]) - ref) + ref
    p = b - piv
    p = np.array([sx * p[0], p[1]])
    r = np.hypot(*p); rmax = 0.985 * m.lmax; rmin = S.h_min(m)
    if r > rmax: p = p * rmax / r
    if r < rmin: p = p * rmin / r
    return m.ik(p[0], p[1])


def stand_feet(key):
    o = S.OPT[key]
    return {leg: np.array([o["pivot"][leg][0] + o["sx"][leg] * o["x0"][leg], 0.0]) for leg in ("front", "rear")}


def gait(key, gname, phases):
    """One gait cycle.  The body runs at one height for all four legs (the lowest running height any leg needs)."""
    o = S.OPT[key]; out = []
    paths = {leg: S.foot_path(key, leg, gname, n=NF) for leg in ("front", "rear")}
    drop = max(o["pivot"][l][1] - S.h_run(key, l, gname) for l in ("front", "rear"))
    body = (0.0, -drop, 0.0)
    for f in range(NF):
        fr = {"body": [0.0, float(-drop), 0.0], "legs": []}
        for (leg, side), ph in zip(LEGS, phases):
            p = paths[leg]; i = int((f + ph * NF) % NF); m = o["legs"][leg]
            h = o["pivot"][leg][1] - drop
            z = p["z"][i] + S.h_run(key, leg, gname) - h
            q = m.ik(p["x"][i], z)
            sw = i >= p["n_st"]
            lift = z + h
            fold = min(np.radians(110), max(lift, 0) / S.GAITS[gname]["clear"] * np.radians(110)) if sw else 0.0
            fr["legs"].append(to_world(key, leg, side, chain(key, leg, q, fold), body))
        out.append(fr)
    return out


def jump(key):
    """Crouch, push, flight (legs tucked), land, recover.  Body vertical motion from the jump model (hoof clearance)."""
    o = S.OPT[key]; R = RES[key]; hc = R["jump"][0]; v = R["jump"][1]; stroke = R["jump"][3]
    feet = stand_feet(key)
    z_top = {l: 0.97 * o["legs"][l].lmax for l in ("front", "rear")}
    dz_top = min(z_top[l] - o["pivot"][l][1] for l in z_top)                 # body rise at take-off from the stand height
    crouch = dz_top - stroke
    tf = 2 * v / S.G
    keys = [(0, 0.0), (14, crouch), (24, crouch), (32, dz_top)]
    out = []
    for f in range(NF):
        if f <= 32:
            dz = np.interp(f, [k[0] for k in keys], [k[1] for k in keys]); air = 0.0
        elif f <= 52:
            t = (f - 32) / 20 * tf; dz = dz_top + v * t - 0.5 * S.G * t * t; air = np.sin(np.pi * (f - 32) / 20)
        else:
            dz = np.interp(f, [52, 60, NF - 1], [dz_top, crouch * 0.6, 0.0]); air = 0.0
        body = (0.0, dz, 0.0); fr = {"body": [0.0, float(dz), 0.0], "legs": []}
        for leg, side in LEGS:
            m = o["legs"][leg]
            if air > 0:                                                        # tuck: hooves up toward the body
                q = m.ik(o["x0"][leg], -max(S.h_min(m) + 0.02, (o["pivot"][leg][1] + dz_top) - air * 0.09))
                pts = chain(key, leg, q, air * np.radians(100))
            else:
                q = ik_world(key, leg, feet[leg], body); pts = chain(key, leg, q)
            fr["legs"].append(to_world(key, leg, side, pts, body))
        out.append(fr)
    return out


def rear(key):
    """Rear up to REAR_DEG on the hind legs (hooves stay put), hold, come down."""
    o = S.OPT[key]; feet = stand_feet(key); out = []
    th_max = np.radians(S.REAR_DEG)
    for f in range(NF):
        s = smooth(f / 24) if f < 24 else (1.0 if f < 48 else smooth(1 - (f - 48) / 23))
        th = th_max * s
        # pitch about the rear hoof line: keep the hind hooves on the ground; raise the body a little as the legs extend
        ref = np.array([S.X_COM, 0.30]); R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        hp = o["pivot"]["rear"]
        hw = R @ (hp - ref) + ref
        bz = (0.92 * o["legs"]["rear"].lmax * np.cos(0.0) - o["pivot"]["rear"][1]) * s * 0.6
        body = (float(hp[0] - hw[0]), float(hp[1] - hw[1] + bz), float(th))
        fr = {"body": [body[0], body[1], body[2]], "legs": []}
        for leg, side in LEGS:
            m = o["legs"][leg]
            if leg == "rear":
                q = ik_world(key, leg, feet["rear"], body); pts = chain(key, leg, q)
            else:                                                              # forelegs fold up, hooves tucked
                q0 = m.ik(o["x0"][leg], -o["pivot"][leg][1]); q1 = m.ik(o["x0"][leg] + 0.06, -max(S.h_min(m) + 0.03, 0.62 * m.lmax))
                q = q0 + (q1 - q0) * s
                if o["carpus"]:
                    pts = chain(key, leg, q, s * np.radians(115))
                else:
                    pts = chain(key, leg, q)
                if s < 0.05:
                    q = ik_world(key, leg, feet["front"], body); pts = chain(key, leg, q)
            fr["legs"].append(to_world(key, leg, side, pts, body))
        out.append(fr)
    return out


def kneel(key):
    """Lie down: B kneels on its carpi first (as a horse does), the others fold all four legs; then get up."""
    o = S.OPT[key]; feet = stand_feet(key); out = []
    hk = {l: S.kneel_height(key, l) for l in ("front", "rear")}
    PROF = np.asarray(C3.OUT)

    def belly(dzf, dzr):
        th = np.arctan2(dzf - dzr, o["pivot"]["front"][0] - o["pivot"]["rear"][0])
        dzc = dzr + (dzf - dzr) * (S.X_COM - o["pivot"]["rear"][0]) / (o["pivot"]["front"][0] - o["pivot"]["rear"][0])
        Rm = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); ref = np.array([S.X_COM, 0.30])
        return ((Rm @ (PROF - ref).T).T + ref)[:, 1].min() + dzc
    full = (-(o["pivot"]["front"][1] - hk["front"]), -(o["pivot"]["rear"][1] - hk["rear"]))
    k = 1.0
    while belly(full[0] * k, full[1] * k) < 0.040 and k > 0.05:
        k -= 0.01
    hk = {"front": o["pivot"]["front"][1] + full[0] * k, "rear": o["pivot"]["rear"][1] + full[1] * k}
    KNEEL[key] = dict(front=hk["front"], rear=hk["rear"], belly=belly(full[0] * k, full[1] * k))
    for f in range(NF):
        s = smooth(f / 30) if f < 30 else (1.0 if f < 42 else smooth(1 - (f - 42) / 29))
        dzf = -(o["pivot"]["front"][1] - hk["front"]) * s; dzr = -(o["pivot"]["rear"][1] - hk["rear"]) * s
        th = np.arctan2(dzf - dzr, o["pivot"]["front"][0] - o["pivot"]["rear"][0])
        ref_x = S.X_COM; dz_ref = dzr + (dzf - dzr) * (ref_x - o["pivot"]["rear"][0]) / (o["pivot"]["front"][0] - o["pivot"]["rear"][0])
        body = (0.0, float(dz_ref), float(th))
        fr = {"body": [0.0, float(dz_ref), float(th)], "legs": []}
        for leg, side in LEGS:
            m = o["legs"][leg]
            if o["carpus"] and leg == "front":
                mc = S.Panto(m.l1, m.l2, S.CARPUS * m.l3, m.delta)
                # contact moves from the hoof to the carpus pad as the leg folds
                q = ik_world(key, leg, feet[leg], body)
                fold = np.radians(150) * smooth((s - 0.2) / 0.6)
                pts = chain(key, leg, q, fold)
                if s > 0.5:
                    piv = o["pivot"][leg]
                    R = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); ref = np.array([S.X_COM, 0.30])
                    pw = R @ (piv - ref) + ref + np.array([0, dz_ref])
                    tgt = np.array([feet[leg][0] + 0.02, 0.02]) - pw
                    tb = R.T @ tgt
                    qc = mc.ik(tb[0] * o["sx"][leg], tb[1])
                    pts = chain(key, leg, qc, fold)
            else:
                q = ik_world(key, leg, feet[leg] + np.array([0.0, 0.0]), body); pts = chain(key, leg, q)
            fr["legs"].append(to_world(key, leg, side, pts, body))
        out.append(fr)
    return out


GALLOP_PH = {}


def best_gallop(key):
    """Gallop footfall timing: the controller is free to pick it, so take the one with the most fore-hind leg clearance
    (front lead after the hinds by 0.35-0.75 of a stride; transverse and rotary)."""
    best = None
    for lead in np.arange(0.35, 0.80, 0.05):
        for fl, rl in ((0.10, 0.10), (0.10, -0.10), (0.15, 0.08)):
            ph = (lead, lead + fl, 0.0, rl % 1.0)                     # FL, FR, RL, RR
            fr = gait(key, "gallop", ph)
            c = leg_clearance(fr)[0]
            if best is None or c > best[0]:
                best = (c, ph, fr)
    GALLOP_PH[key] = best[1]
    return best[2]


def stand(key):
    o = S.OPT[key]; feet = stand_feet(key); body = (0.0, 0.0, 0.0)
    fr = {"body": [0.0, 0.0, 0.0], "legs": []}
    for leg, side in LEGS:
        q = ik_world(key, leg, feet[leg], body); fr["legs"].append(to_world(key, leg, side, chain(key, leg, q), body))
    return [fr]


with contextlib.redirect_stdout(io.StringIO()):
    RES = S.study()
    import sys
    os.environ.setdefault("EQUO_S", "0.75")
    sys.path[:0] = [os.path.dirname(os.path.abspath(__file__))]
    import chassis_v3 as C3
KNEEL = {}


def seg_dist(p, q, a, b):
    """Shortest distance between segments pq and ab (3D)."""
    best = 1e9
    for t in np.linspace(0, 1, 9):
        x = p + (q - p) * t; ab = b - a
        s_ = np.clip((x - a) @ ab / max(ab @ ab, 1e-12), 0, 1)
        best = min(best, np.linalg.norm(x - (a + ab * s_)))
    return best


def leg_clearance(frames):
    """Fore-hind leg clearance on each side over all frames (segment axes, minus 2 x 15 mm tube radius, est.)."""
    worst = (1e9, None)
    for fi, fr in enumerate(frames):
        for (i, j) in ((0, 2), (1, 3)):
            A = [np.array(p) for p in fr["legs"][i]]; B = [np.array(p) for p in fr["legs"][j]]
            for a0, a1 in zip(A[1:-1], A[2:]):
                for b0, b1 in zip(B[1:-1], B[2:]):
                    d = seg_dist(a0, a1, b0, b1) - 0.030
                    if d < worst[0]:
                        worst = (d, fi)
    return worst

if __name__ == "__main__":
    PH = {"trot": (0.0, 0.5, 0.5, 0.0), "gallop": (0.55, 0.65, 0.0, 0.1), "walk": (0.0, 0.5, 0.75, 0.25)}
    F = {}
    for key in S.OPT:
        F[key] = dict(stand=stand(key), walk=gait(key, "walk", PH["walk"]), trot=gait(key, "trot", PH["trot"]),
                      gallop=best_gallop(key), jump=jump(key), rear=rear(key), kneel=kneel(key))
    json.dump(F, open(os.path.join(HERE, "frames.json"), "w"))
    CL = {k: {m: leg_clearance(v)[0] for m, v in d.items()} for k, d in F.items()}
    json.dump(dict(clear=CL, kneel=KNEEL, gallop_phase=GALLOP_PH), open(os.path.join(HERE, "frames_checks.json"), "w"), indent=1)
    for k in CL:
        print(k, "fore-hind leg clearance (mm):", {m: round(v * 1e3) for m, v in CL[k].items()}, "kneel", {a: round(b * 1e3) for a, b in KNEEL[k].items()})
    print("frames", {k: {m: len(v) for m, v in d.items()} for k, d in F.items()})
