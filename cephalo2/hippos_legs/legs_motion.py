#!/usr/bin/env python3
"""Rev B motions through the CAD joint tree, with the mechanisms closed.
The leg study gives each motion as body pose + hoof targets.  Here every frame is re-solved on the CAD kinematics:
  - the hoof is a flat shoe under a ball joint: in stance the sole stays planted (ball HOOF_H above the planted point);
    when the hoof would need more than the ball joint's 15 deg, it rolls on its heel / toe edge (R_HOOF from the centre)
    about a fixed ground point, so nothing slips;
  - joint limits come from the clearance check (legs_cad_verify.RANGE); nothing is clamped: a motion that needs more is
    re-planned (the lie-down goes only as deep as the joints allow, on the hooves) or reported;
  - evaluate() re-runs the study's load cases at the CAD mass with the built actuators.
    import legs_motion as M;  M.MOTIONS -> {name: dict(frames, dt, cyclic)}"""
import os, sys, io, json, contextlib
import numpy as np
os.environ.setdefault("EQUO_S", "0.75")
sys.path[:0] = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "deps")]
import hippos_legs_cad as C
import legs_cad_verify as V
HERE = os.path.dirname(os.path.abspath(__file__)); D = 180 / np.pi
LIM = {k: {j: r[1] for j, r in V.RANGE[k].items()} for k in V.RANGE}; LO = {k: {j: r[0] for j, r in V.RANGE[k].items()} for k in V.RANGE}
R_HOOF = 26.0; HP_MAX = 15.0
# STS3215 at 12 V: 45 rpm no-load (est., not re-checked against a datasheet here); spool angle per fold from legs_mech (378 deg for 120 deg)
CARPUS_SERVO = dict(servo_deg_s=270.0, servo_deg_per_fold_deg=378.3 / 120.0, fold_rate_deg_s=270.0 / (378.3 / 120.0))
with contextlib.redirect_stdout(io.StringIO()):
    import legs_v3_frames as FR
    import chassis_v3 as C3, chassis_v4 as C4
S = FR.S; O = S.OPT["B"]
KEY = {"fore": "front", "hind": "rear"}; KINDS = ("fore", "fore", "hind", "hind"); SIDES = (1, -1, 1, -1)
REF = np.array([S.X_COM * 1e3, 300.0])


def r_at(m, q1_deg):
    return float(np.hypot(*m.foot(np.array([0.0, q1_deg / D]))))
_old_hmin = S.h_min
def h_min(m):
    lim = LIM["fore"]["q1"] if m.l2 < 0.125 else LIM["hind"]["q1"]
    return max(_old_hmin(m), r_at(m, lim - 1.0))
S.h_min = h_min
ACTS = dict(hip={"front": "RobStride 03", "rear": "RobStride 06"}, knee={"front": "RobStride 04", "rear": "RobStride 06"},
            abd={"front": "RobStride 06", "rear": "RobStride 02"}, ratio={"front": 22 / 46, "rear": 1.0})
CADM = json.load(open(os.path.join(HERE, "legs_cad.json")))
M_OTHER = 33.4 - S.M_LEGS_NOW                                  # everything but the legs (octopus, body, packs, tail), est.


OFFLOAD = dict(front=0.74, rear=0.76)                          # what the 400 N gas springs give at the stand (legs_motion_test: forces)


def evaluate(acts=ACTS, m_legs=None):
    m_legs = CADM["four_legs"] if m_legs is None else m_legs
    S.M_TOTAL["B"] = M_OTHER + m_legs; rows = []; stand = {}
    for leg in ("front", "rear"):
        S.RATIO_NOW[("B", leg)] = acts["ratio"][leg]; S.OFFLOAD = OFFLOAD[leg]
        stand[leg] = float(abs(S.static("B", leg, O["x0"][leg], -O["pivot"][leg][1], (0, S.share(leg) / 2 * S.weight("B")))[0][1]))
        for case, (rms, pk) in S.joint_cases("B", leg, acts).items():
            for j, jn in enumerate(("hip", "knee")):
                a = S.ACT[acts[jn][leg]]; ratio = acts["ratio"][leg] if jn == "knee" else 1.0; eff = S.BELT_EFF if ratio != 1 else 1.0
                m = min(a["rated"] * ratio * eff / max(rms[j], 1e-6) / 1.2 if case in S.CORE else 99, a["peak"] * ratio * eff / max(pk[j], 1e-6) / 1.1)
                rows.append((round(float(m), 2), leg, case, "pivot" if jn == "hip" else "joint", round(float(rms[j]), 1), round(float(pk[j]), 1)))
    rows.sort()
    S.OFFLOAD = 0.8
    return dict(total=S.M_TOTAL["B"], rows=rows, worst=rows[:6], jump_mm=S.max_jump("B", acts)[0] * 1e3, fails=[r for r in rows if r[0] < 1.0], stand_joint_Nm=stand)


# ------------------------------------------------------------------ CAD kinematics in the leg plane (mm, deg)
def u(t):
    return np.array([np.sin(t), -np.cos(t)])


def ball_xz(kind, q0, q1):
    """Ball centre in the leg frame (pivot origin), leg unfolded, fetlock at rest."""
    g = C.LEG[kind]; a = q0 / D
    return g["L1"] * u(a) + g["L2"] * u(a - q1 / D) + (g["L3"] - C.HOOF_H) * u(a + g["delta"] / D)


def end_xz(kind, q0, q1):
    g = C.LEG[kind]; a = q0 / D
    return g["L1"] * u(a) + g["L2"] * u(a - q1 / D) + g["L3"] * u(a + g["delta"] / D)


def ik_ball(kind, target, q_guess):
    q = np.array(q_guess, float)
    for _ in range(40):
        f = ball_xz(kind, *q) - target
        if np.abs(f).max() < 1e-7: break
        J = np.column_stack([(ball_xz(kind, q[0] + 1e-5, q[1]) - ball_xz(kind, *q)) / 1e-5, (ball_xz(kind, q[0], q[1] + 1e-5) - ball_xz(kind, *q)) / 1e-5])
        dq = np.linalg.solve(J, -f); q = q + np.clip(dq, -20, 20)
    return q, float(np.abs(ball_xz(kind, *q) - target).max())


def Rm(th_deg):
    c, s = np.cos(th_deg / D), np.sin(th_deg / D); return np.array([[c, -s], [s, c]])


def to_world(kind, p, body):
    piv = C.LEG[kind]["pivot"][[0, 2]]
    return Rm(body[2]) @ (piv + p - REF) + REF + np.array(body[:2])


def to_leg(kind, w, body):
    piv = C.LEG[kind]["pivot"][[0, 2]]
    return Rm(body[2]).T @ (np.asarray(w) - np.array(body[:2]) - REF) + REF - piv


def roty(deg, v):
    c, s = np.cos(deg / D), np.sin(deg / D); return np.array([v[0] * c + v[1] * s, -v[0] * s + v[1] * c])


def study_frames(frs, st=None, no_fold=False):
    """Study frames -> per frame: body (mm, mm, deg), per leg (q0, q1, fold, contact, hoof target of the unfolded leg in world mm)."""
    out = []
    ang = lambda v: np.degrees(np.arctan2(v[0], -v[1]))
    for kf, fr in enumerate(frs):
        body = (fr["body"][0] * 1e3, fr["body"][1] * 1e3, np.degrees(fr["body"][2])); legs = []
        for i, pts in enumerate(fr["legs"]):
            kind = KINDS[i]; P = np.array([[p[0], p[2]] for p in pts]) * 1e3; s = [ang(P[k + 1] - P[k]) for k in range(len(P) - 1)]
            q0, q1 = s[0] - body[2], s[0] - s[1]; fold = (s[2] - s[3]) if kind == "fore" else 0.0
            F = to_world(kind, end_xz(kind, q0, q1), body); fold = 0.0 if (no_fold or fold < 0.05) else fold
            legs.append(dict(q0=q0, q1=q1, fold=fold, st=bool(st[kf][i]) if st is not None else bool(F[1] < 0.5 and fold == 0.0), F=F))
        out.append(dict(body=body, legs=legs))
    return out


def solve_motion(sf, cyclic):
    """Close every frame on the CAD kinematics.  -> frames [dict(b, l=[dict(q0,q1,fold,hp,st,roll,contact)])]"""
    n = len(sf); res = [dict(b=list(f["body"]), l=[None] * 4) for f in sf]
    for i in range(4):
        kind = KINDS[i]; delta = C.LEG[kind]["delta"]; off = [None] * n; sol = [None] * n
        for k in range(n):                                             # stance frames: planted sole, heel / toe roll past the ball joint's range
            lg = sf[k]["legs"][i]; body = sf[k]["body"]
            if not lg["st"]: continue
            F = np.array([lg["F"][0], 0.0]); B = F + [0, C.HOOF_H]; q = (lg["q0"], lg["q1"]); phi = 0.0; edge = 0
            for _ in range(8):
                q, err = ik_ball(kind, to_leg(kind, B, body), q)
                h = body[2] + q[0] + delta
                phi = 0.0 if abs(h) <= HP_MAX else np.sign(h) * HP_MAX - h           # hoof's rotation in the tree's sense (deg)
                edge = 0 if phi == 0 else (1 if phi > 0 else -1)                     # +1 toe, -1 heel
                E = F + [edge * R_HOOF, 0.0]
                B = E - roty(phi, np.array([edge * R_HOOF, -C.HOOF_H]))
            sol[k] = (q, phi, edge, err); off[k] = B - (F + [0, C.HOOF_H])
        st = [k for k in range(n) if off[k] is not None]
        for k in range(n):                                             # swing frames: study path, offset blended between lift-off and touch-down
            lg = sf[k]["legs"][i]; body = sf[k]["body"]
            if off[k] is None:
                if st:
                    prev = [j for j in st if j < k]; nxt = [j for j in st if j > k]
                    if cyclic:
                        a = prev[-1] if prev else st[-1] - n; b = nxt[0] if nxt else st[0] + n
                    else:
                        a = prev[-1] if prev else None; b = nxt[0] if nxt else None
                    if a is None: dl = off[b % n]
                    elif b is None: dl = off[a % n]
                    else:
                        w = (k - a) / (b - a); w = w * w * (3 - 2 * w); dl = off[a % n] * (1 - w) + off[b % n] * w
                else: dl = np.zeros(2)
                # in the air the ball joint hangs free up to its 15 deg; past that the hoof tilts with the pastern, so the ball
                # is carried high enough for the low edge of the shoe (same expression as the heel / toe roll on the ground)
                q = (lg["q0"], lg["q1"]); ph_ = 0.0
                for _ in range(4):
                    B = np.array([lg["F"][0] + dl[0], lg["F"][1] + C.HOOF_H * np.cos(ph_ / D) + R_HOOF * abs(np.sin(ph_ / D))])
                    q, err = ik_ball(kind, to_leg(kind, B, body), q)
                    h_ = body[2] + q[0] + delta - lg["fold"]; ph_ = 0.0 if abs(h_) <= HP_MAX else np.sign(h_) * HP_MAX - h_
                phi, edge = 0.0, 0
            else:
                q, phi, edge, err = sol[k]
            h = body[2] + q[0] + delta - lg["fold"]
            hp = float(np.clip(h, -HP_MAX, HP_MAX))
            res[k]["l"][i] = dict(q0=float(q[0]), q1=float(q[1]), fold=float(lg["fold"]), hp=hp, st=bool(lg["st"]), roll=float(phi), edge=edge, err=err,
                                  F=[float(lg["F"][0]), 0.0] if lg["st"] else None)
    return res


def crouch_frames():
    """Lie down on the hooves as deep as the joint limits and the belly allow, hold, get up (the carpus kneel needs a pad that
    is not designed, and the cannon cannot fold flat under the forearm)."""
    feet = FR.stand_feet("B"); prof = np.asarray(C3.OUT)
    def pose(dzf, dzr):
        pf, pr = O["pivot"]["front"], O["pivot"]["rear"]
        th = np.arctan2(dzf - dzr, pf[0] - pr[0]); dzc = dzr + (dzf - dzr) * (S.X_COM - pr[0]) / (pf[0] - pr[0])
        return (0.0, float(dzc), float(th))
    def ok(dzf, dzr):
        body = pose(dzf, dzr); b = (0.0, body[1] * 1e3, np.degrees(body[2]))
        for kind in ("fore", "hind"):
            F = feet[KEY[kind]] * 1e3; q, err = ik_ball(kind, to_leg(kind, np.array([F[0], C.HOOF_H]), b), (C.STAND[kind]["q0"], C.STAND[kind]["q1"]))
            if err > 1e-4 or q[1] > LIM[kind]["q1"] - 1.0 or not (LO[kind]["q0"] <= q[0] <= LIM[kind]["q0"]): return False
        Rr = np.array([[np.cos(body[2]), -np.sin(body[2])], [np.sin(body[2]), np.cos(body[2])]]); ref = np.array([S.X_COM, 0.30])
        return ((Rr @ (prof - ref).T).T + ref)[:, 1].min() + body[1] > 0.040
    best = (0.0, 0.0)
    for df in np.arange(0, 0.30, 0.004):
        for dr in np.arange(0, 0.30, 0.004):
            if df + dr > best[0] + best[1] and abs(df - dr) < 0.03 and ok(-df, -dr): best = (df, dr)
    out = []
    for f in range(FR.NF):
        s = FR.smooth(f / 30) if f < 30 else (1.0 if f < 42 else FR.smooth(1 - (f - 42) / 29))
        body = pose(-best[0] * s, -best[1] * s); fr = {"body": list(body), "legs": []}
        for leg, side in FR.LEGS:
            q = FR.ik_world("B", leg, feet[leg], body); fr["legs"].append(FR.to_world("B", leg, side, FR.chain("B", leg, q), body))
        out.append(fr)
    return out, dict(front_pivot_mm=(O["pivot"]["front"][1] - best[0]) * 1e3, rear_pivot_mm=(O["pivot"]["rear"][1] - best[1]) * 1e3)


def build(verbose=False):
    with contextlib.redirect_stdout(io.StringIO()):
        R = S.study(); FR.RES = R
    ev = evaluate()
    S.M_TOTAL["B"] = ev["total"]; FR.RES["B"]["jump"] = S.max_jump("B", ACTS)
    PH = {"trot": (0.0, 0.5, 0.5, 0.0), "walk": (0.0, 0.5, 0.75, 0.25)}
    cr, crouch = crouch_frames()
    raw = dict(stand=FR.stand("B"), walk=FR.gait("B", "walk", PH["walk"]), trot=FR.gait("B", "trot", PH["trot"]), gallop=FR.best_gallop("B"),
               jump=FR.jump("B"), rear=FR.rear("B"), kneel=cr)
    tf = 2 * FR.RES["B"]["jump"][1] / S.G
    dts = dict(stand=0.05, jump=tf / 20, rear=0.05, kneel=0.05)
    for g in ("walk", "trot", "gallop"):
        dts[g] = S.foot_path("B", "front", g, n=FR.NF)["dt"]
    mot = {}
    PH["gallop"] = FR.GALLOP_PH["B"]
    for name, frs in raw.items():
        cyc = name in PH; st = None
        if cyc:                                                    # stance flags from the gait's own phase, not from the hoof height
            n_st = S.foot_path("B", "front", name, n=FR.NF)["n_st"]
            st = [[int((f + ph * FR.NF) % FR.NF) < n_st for ph in PH[name]] for f in range(FR.NF)]
        # the carpus servo turns about 3.2 deg per degree of fold (CARPUS_SERVO): far too slow to fold in stride or in a jump's
        # flight, so gaits and the jump run with the carpus locked; the rear-up keeps its fold and is slowed to the servo
        sf = study_frames(frs, st, no_fold=name in ("walk", "trot", "gallop", "jump"))
        fr = solve_motion(sf, cyc); dt = float(dts[name])
        fs = max((abs(fr[k + 1]["l"][i]["fold"] - fr[k]["l"][i]["fold"]) for k in range(len(fr) - 1) for i in range(2)), default=0.0)
        if fs > 0: dt = max(dt, fs / (CARPUS_SERVO["fold_rate_deg_s"] / 1.5))
        mot[name] = dict(frames=fr, dt=dt, cyclic=cyc)
    return mot, ev, crouch


if __name__ == "__main__":
    mot, ev, crouch = build()
    print("mass", round(ev["total"], 2), "jump", round(ev["jump_mm"]), "worst", ev["worst"][:4], "crouch", crouch)
    for name, m in mot.items():
        fr = m["frames"]
        for i in (0, 2):
            q0 = [f["l"][i]["q0"] for f in fr]; q1 = [f["l"][i]["q1"] for f in fr]; fo = [f["l"][i]["fold"] for f in fr]; ro = [f["l"][i]["roll"] for f in fr]
            print("%-7s %s q0 %6.1f..%5.1f  q1 %6.1f..%5.1f  fold max %5.1f  roll %5.1f..%4.1f  ik err %.1e" % (name, KINDS[i], min(q0), max(q0), min(q1), max(q1), max(fo), min(ro), max(ro), max(f["l"][i]["err"] for f in fr)))
