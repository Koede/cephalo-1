#!/usr/bin/env python3
"""Centauri-1 agility on the rev B legs: how high, how far, how tight, and a horse-show course.
Kinematic plan on the CAD leg tree (hippos_legs_cad) with all four leg joints per leg (sideways swing, twist, pivot, flex):
hooves are planted in the world while in stance; the body follows a path of straights, arcs and spins; a gait clock
(walk -> trot -> fast trot -> gallop by speed) places the footfalls; leaps are runs of four-leg bounds (bounds, leap_plan),
from a square stand or straight out of the gait (sprint_seq, sprint_jump).
Checked per run: joint limits, IK residual, hoof tilt, joint speeds against the motors at 39.6 V, sideways-swing torque (est.),
clearances inside each leg and leg to leg / neck (legs_cad_verify rules).  NOT a dynamics simulation: balance, slip and the
push energy of a moving leap are estimates (marked est.).
    python3 agility.py -> agility.json (numbers), agility_course.csv (joint angles), viewer_agility/agility_data.js"""
import os, sys, json
import numpy as np
import hippos_legs_cad as C
import legs_cad_verify as V
import legs_motion as M
import legs_motion_test as T
HERE = M.HERE; D = M.D; S = M.S; FR = M.FR; KINDS = M.KINDS; SIDES = M.SIDES; G = 9.81
DT = 1 / 60.0; R_SPIN = 0.26                       # spin: pseudo-distance = angle x hoof radius from the centre
TUCK = 60.0                                        # mm the hooves are drawn up in a leap's flight
INSET = 40.0                                       # mm: hooves run this far inboard of the pivots (base-narrow): cuts the sideways-swing motor load
A_LONG, A_LAT = 1.0, 1.6                           # m/s2: speeding up / braking (est.), sideways (0.6 x tip-over g*y/h with the narrow base)
feet = FR.stand_feet("B")
NEUT = [np.array([feet[M.KEY[k]][0] * 1e3, s * (C.LEG[k]["pivot"][1] - INSET)]) for k, s in zip(KINDS, SIDES)]     # hoof stand points, body frame (mm)
ev = M.evaluate(); MASS = ev["total"]
JUMP = S.max_jump("B", M.ACTS)                     # (hoof clearance m, take-off speed m/s, ?, push stroke m)
VZ = float(JUMP[1]); STROKE = float(JUMP[3])
GA = {}                                            # gait table by speed
for name, ph in (("walk", (0.0, 0.5, 0.75, 0.25)), ("trot", (0.0, 0.5, 0.5, 0.0)), ("fast trot", (0.0, 0.5, 0.5, 0.0)), ("gallop", None)):
    g = S.GAITS[name]
    if ph is None:
        FR.best_gallop("B"); ph = FR.GALLOP_PH["B"]
    drop = max(S.OPT["B"]["pivot"][l][1] - S.h_run("B", l, name) for l in ("front", "rear"))
    GA[name] = dict(v=g["v"], L=g["stride"], b=g["duty"], clear=g["clear"], drop=drop, ph=np.array(ph, float))
KN = [GA[n] for n in ("walk", "trot", "fast trot", "gallop")]
RR = {k: min(0.97 * S.OPT["B"]["legs"][M.KEY[k]].lmax * 1e3 - C.HOOF_H, float(np.linalg.norm(M.ball_xz(k, C.STAND[k]["q0"], 53.0)))) for k in ("fore", "hind")}   # swing reach cap


def gait_at(v):
    """Stride, duty, clearance, body drop and leg phase offsets for a speed (blend between the four study gaits)."""
    v = float(np.clip(v, KN[0]["v"], KN[-1]["v"]))
    for a, b in zip(KN[:-1], KN[1:]):
        if v <= b["v"] + 1e-9:
            w = (v - a["v"]) / (b["v"] - a["v"]); d = (b["ph"] - a["ph"] + 0.5) % 1.0 - 0.5
            return dict(L=a["L"] + w * (b["L"] - a["L"]), b=a["b"] + w * (b["b"] - a["b"]), clear=a["clear"] + w * (b["clear"] - a["clear"]),
                        drop=a["drop"] + w * (b["drop"] - a["drop"]), ph=(a["ph"] + w * d) % 1.0)


# ------------------------------------------------------------------ path: straights, arcs, spins
class Path:
    def __init__(s, segs, start=(0.0, 0.0, 0.0)):
        s.segs = []; x, y, h = start; s0 = 0.0
        for sg in segs:
            kind = sg[0]
            if kind == "line": L = sg[1]; k = 0.0
            elif kind == "arc": L = abs(sg[1] * np.radians(sg[2])); k = np.sign(sg[2]) / sg[1]
            else: L = abs(np.radians(sg[1])) * R_SPIN; k = np.sign(sg[1]) / R_SPIN
            s.segs.append(dict(kind=kind, s0=s0, L=L, k=k, x=x, y=y, h=h, vmax=sg[-1] if isinstance(sg[-1], dict) else {}))
            x, y, h = s._adv(s.segs[-1], L); s0 += L
        s.total = s0

    def _adv(s, sg, d):
        x, y, h, k = sg["x"], sg["y"], sg["h"], sg["k"]
        if sg["kind"] == "line": return x + d * np.cos(h), y + d * np.sin(h), h
        if sg["kind"] == "spin": return x, y, h + d * k
        return x + (np.sin(h + d * k) - np.sin(h)) / k, y - (np.cos(h + d * k) - np.cos(h)) / k, h + d * k

    def seg(s, sig):
        for sg in s.segs:
            if sig <= sg["s0"] + sg["L"] + 1e-9: return sg
        return s.segs[-1]

    def pose(s, sig):
        sig = float(np.clip(sig, 0, s.total)); sg = s.seg(sig)
        return s._adv(sg, sig - sg["s0"])

    def turn(s, sig):
        """(curvature 1/m, spinning?)"""
        sg = s.seg(float(np.clip(sig, 0, s.total))); return sg["k"], sg["kind"] == "spin"


def speed_profile(path, vcap=1.6, vspin=0.35, slow=()):
    """Fastest speed along the path: gait top speed, sideways acceleration in arcs, pseudo-speed in spins, slow zones (leaps),
    then accelerate / brake limits forwards and backwards.  -> (grid, v)"""
    ds = 0.005; sg_ = np.arange(0, path.total + ds, ds); v = np.zeros(len(sg_))
    for i, sig in enumerate(sg_):
        k, spin = path.turn(sig); sg = path.seg(sig)
        v[i] = vspin if spin else (min(vcap, 0.8, np.sqrt(A_LAT / abs(k))) if k else vcap)       # arcs at a fast trot at most
        if "v" in sg["vmax"]: v[i] = min(v[i], sg["vmax"]["v"])
        for a, b, vv in slow:
            if a <= sig <= b: v[i] = min(v[i], vv)
    v[0] = v[-1] = 0.05
    for i in range(1, len(v)): v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * A_LONG * ds))
    for i in range(len(v) - 2, -1, -1): v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * A_LONG * ds))
    return sg_, v


def profile_time(sg_, v):
    return float(np.sum(np.diff(sg_) / np.maximum((v[1:] + v[:-1]) / 2, 1e-3)))


# ------------------------------------------------------------------ leg IK in 3D (leg frame: x fwd, y outboard, z up; mm, deg)
def ik3(kind, p, yaw, guess):
    ya, za = C.ABD_AXES[kind]; X, Y, Z = p; A_, B_ = Y - ya, Z - za; cc = X * np.tan(yaw / D) - ya; hy = np.hypot(A_, B_)
    if abs(cc) > hy: return None
    base = np.arctan2(B_, A_); acs = np.arccos(cc / hy)
    al = min(((base + acs + np.pi) % (2 * np.pi) - np.pi, (base - acs + np.pi) % (2 * np.pi) - np.pi), key=abs)        # = abd
    zp = za - A_ * np.sin(al) + B_ * np.cos(al)
    q, err = M.ik_ball(kind, np.array([X / np.cos(yaw / D), zp]), guess)
    return al * D, q[0], q[1], err


def body_T(b):
    """Body frame (mm) -> world (mm): b = (x, y, z, heading deg, pitch deg)."""
    ch, sh = np.cos(b[3] / D), np.sin(b[3] / D); Rz = np.array([[ch, -sh, 0, 0], [sh, ch, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    Tt = np.eye(4); Tt[:3, 3] = b[:3]
    return Tt @ Rz @ T.body4((0.0, 0.0, b[4]))


T_LAND = 0.40
Z_TO, Z_LAND, CROUCH, LAND_DIP, TRAVEL, HOP, LAND_LEAD, V_OUT, PUSH_LEAD = 0.0, 0.0, -50.0, -45.0, 0.18, 0.7, 0.5, 0.3, 0.5   # leap shape (mm, m, m/s)


def t_flight(vz):
    """Take-off at Z_TO, touch-down at Z_LAND.  -> (time, touch-down sink speed m/s)"""
    vd = float(np.sqrt(vz * vz + 2 * G * (Z_TO - Z_LAND) * 1e-3)); return (vz + vd) / G, vd


def stance_t(v0, v1):
    """Time on the ground in one bound: the hooves cover at most 0.24 m under the body."""
    return float(np.clip(TRAVEL / max((v0 + v1) / 2, 1e-3), 0.16, 0.40))


def stance_d(v0, v1):
    return (v0 + v1) / 2 * stance_t(v0, v1)


def herm(w, z0, v0, z1, v1, Tm):
    """Cubic through (z0, v0) -> (z1, v1) over Tm seconds, w in 0..1 (mm, mm/s)."""
    w = min(max(w, 0.0), 1.0); w2, w3 = w * w, w * w * w
    return (2 * w3 - 3 * w2 + 1) * z0 + (w3 - 2 * w2 + w) * Tm * v0 + (-2 * w3 + 3 * w2) * z1 + (w3 - w2) * Tm * v1


A_NET = VZ ** 2 / (2 * STROKE)                     # m/s2: mean net upward acceleration the motors give over the jump stroke (push model)
A_DN = 15.0                                        # m/s2: braking a landing (est.)


TUCK_RUN = 20.0                                    # mm: hoof tuck in the short gather flight of a running leap
T_LIFT, HOP_RUN = 0.08, 0.5                         # s, m/s: the lift-off stride out of a running gait into the gather


def leap_plan(seq, v0=0.03, running=False):
    """Heights and timing of every bound in a leap, from what the motors can do: a landing is braked at A_DN over its own depth,
    the up-stroke runs at A_NET or less over the stroke it needs, and the stride before the main bound lands low enough
    to arrive at the main crouch.  -> [dict per bound] (mm, s, m/s)"""
    m = big(seq); pl = []; zs = 0.0; vd = 0.0; v_in = v0
    for i, (vx, vz) in enumerate(seq):
        d1 = vd * vd / (2 * A_DN) * 1e3; d2 = vz * vz / (2 * A_NET) * 1e3
        crouch = min(zs - d1, Z_TO - d2); z_to = Z_TO if i == m else min(Z_TO, crouch + max(1.25 * d2, 30.0))
        t1 = 0.25 if vd < 0.2 else min(2 * (zs - crouch) / (vd * 1e3), 0.18); t2 = 2 * (z_to - crouch) / (vz * 1e3)
        if running and i == 0: crouch, t1, t2, z_to = 0.0, 0.0, T_LIFT, vz * T_LIFT / 2 * 1e3      # out of the gait: the planted legs lift the body, no crouch
        if i == m: zl = Z_LAND
        elif i + 1 == m:
            d2m = seq[m][1] ** 2 / (2 * A_NET) * 1e3; zl = z_to
            for _ in range(8): zl = min(z_to, Z_TO - d2m + (vz * vz + 2 * G * (z_to - zl) * 1e-3) / (2 * A_DN) * 1e3)
        else: zl = z_to
        vdn = float(np.sqrt(vz * vz + 2 * G * (z_to - zl) * 1e-3)); tf = (vz + vdn) / G
        pl.append(dict(v_in=v_in, vx=vx, vz=vz, zs=zs, vd=vd, crouch=crouch, z_to=z_to, t1=t1, t2=t2, tp=t1 + t2, d=(v_in + vx) / 2 * (t1 + t2), zl=zl, tf=tf, vdn=vdn))
        zs, vd, v_in = zl, vdn, vx
    d1 = vd * vd / (2 * A_DN) * 1e3; dip = zs - max(d1, 20.0); td = 2 * (zs - dip) / (vd * 1e3)
    return pl, dict(zs=zs, vd=vd, dip=dip, td=td, T=td + 0.25, d=(v_in + V_OUT) / 2 * (td + 0.25))


def bounds(vx, vz, hop=None, dv=0.55, out=True):
    """A leap that carries distance: bounding run-up strides (all four legs together, each a small hop) building speed by at most
    dv per stride, then the big bound at (vx, vz).  -> [(vx, vz), ...]"""
    hop = HOP if hop is None else hop; n = max(int(np.ceil((vx - 0.03) / dv)) - 1, 0)
    up = [(0.03 + (vx - 0.03) * (i + 1) / (n + 1), hop) for i in range(n)]
    return up + [(vx, vz)] + (up[::-1] if out else [])          # ... and bounding strides down again after the landing


def big(seq):
    """Index of the main bound in a leap sequence."""
    return int(np.argmax([b[1] for b in seq]))


def run(path, vcap=1.6, vspin=0.35, leaps=(), label=""):
    """Walk the body along the path.  leaps: [(sigma of the gather, [(vx, vz), ...])] (see bounds); a third entry True makes it a
    running leap: no square-up, the first bound is the lift-off stride straight out of the gait.  -> dict(frames, dt, stats)"""
    slow = [(lp[0] - 0.4, lp[0] + 0.05, 0.3) for lp in leaps if len(lp) < 3 or not lp[2]]
    squares = sorted([sg["s0"] for sg in path.segs if sg["kind"] == "spin" and sg["s0"] > 0.01] + [sg["s0"] + sg["L"] for sg in path.segs if sg["kind"] == "spin" and sg["s0"] + sg["L"] < path.total - 0.01])
    slow += [(q_ - 0.25, q_ + 0.02, 0.25) for q_ in squares]; sq_i = 0; after = "gait"; lead_sq = 0.0
    sg_, vp = speed_profile(path, vcap, vspin, slow)
    sig = 0.0; clock = 0.0; t = 0.0; mode = "gait"; tm = 0.0; leap_i = 0; z_extra = 0.0; frames = []
    x0, y0, h0 = path.pose(0)
    def world_neut(i, s_at, lead=0.0):
        x, y, h = path.pose(s_at); n = NEUT[i] / 1e3; kk, sp_ = path.turn(s_at)
        n = np.array([n[0], n[1] + SIDES[i] * 0.030 * (1.0 if sp_ else min(abs(kk) / 1.5, 1.0))])       # 25 mm wider in tight turns: room for the inside legs
        return np.array([x + (n[0] + lead) * np.cos(h) - n[1] * np.sin(h), y + (n[0] + lead) * np.sin(h) + n[1] * np.cos(h)]) * 1e3
    def steer(i, s_at):
        k_, sp_ = path.turn(s_at); n = NEUT[i] / 1e3
        gam = np.arctan2(n[0], -n[1]) if sp_ else np.arctan2(k_ * n[0], 1 - k_ * n[1]); gam = (gam + np.pi / 2) % np.pi - np.pi / 2
        return float(np.clip(SIDES[i] * gam * D * (np.sign(k_) if sp_ else 1.0), -25, 25))
    leg = [dict(st=True, P=world_neut(i, 0), P0=None, P1=None, q=(C.STAND[KINDS[i]]["q0"], C.STAND[KINDS[i]]["q1"]), yaw=0.0) for i in range(4)]
    g = gait_at(0.3); zb = 0.0; pho = g["ph"].copy(); vnow = 0.05; dcl = 0.0; bad = dict(ik=0, reach=0); vx_leap = 0.0; z_to = 0.0; land_t = 0.0
    settle = 0.0; seq = []; bi = 0; plan = []; lnd = {}; vz_leap = 0.0; running = False
    while True:
        done = sig >= path.total - 1e-6
        if done:
            settle += DT
            if settle > 0.7: break
        if mode == "gait" and not done:
            vnow = min(float(np.interp(sig, sg_, vp)), vnow + A_LONG * DT); g = gait_at(vnow); k_now, sp_now = path.turn(sig)
            g["L"] *= 0.45 if sp_now else 1.0 - 0.45 * min(abs(k_now) / 2.2, 1.0)                 # shorter steps in tight turns and spins
            dsig = vnow * DT; sig = min(sig + dsig, path.total); dcl = dsig / (g["L"] / g["b"]); clock += dcl
            pho = (pho + np.clip((g["ph"] - pho + 0.5) % 1.0 - 0.5, -0.08 * dcl, 0.08 * dcl)) % 1.0          # footfall order changes slowly (gallop <-> trot)
            go_leap = leap_i < len(leaps) and sig >= leaps[leap_i][0]; go_sq = sq_i < len(squares) and sig >= squares[sq_i] - 0.004
            if go_leap and len(leaps[leap_i]) > 2 and leaps[leap_i][2]:   # running leap: straight from the gait into the lift-off stride
                seq = leaps[leap_i][1]; bi = 0; plan, lnd = leap_plan(seq, vnow, True); mode = "push"; tm = 0.0; running = True
            elif go_leap or go_sq:                                          # square up: before a leap, and going into / out of a spin
                mode = "square"; after = "push" if go_leap else "gait"; lead_sq = 0.05 if go_leap else 0.0
                if go_leap: running = False; seq = leaps[leap_i][1]; bi = 0; plan, lnd = leap_plan(seq); lead_sq = plan[0]["d"] * PUSH_LEAD
                else: sq_i += 1
                for i_, l in enumerate(leg):
                    if not l["st"]: l["P1"] = world_neut(i_, sig + 0.01, lead=lead_sq); l["y1"] = steer(i_, sig + 0.03)
        elif mode == "square":                                            # gather: nearly stop and bring every hoof up under the body, one leg at a time
            vnow = 0.03; sig = min(sig + vnow * DT, path.total)
            for i_, l in enumerate(leg):
                if not l["st"]:
                    l["u"] = min(l.get("u", 0.0) + DT / 0.2, 1.0)
                    if l["u"] >= 1.0: l["st"] = True; l["P"] = l["P1"].copy(); l["sq"] = True
            if all(l["st"] for l in leg):
                todo = [i_ for i_, l in enumerate(leg) if not l.get("sq")]
                if todo:
                    l = leg[todo[0]]; l["st"] = False; l["u"] = 0.0; l["P0"] = l["P"].copy(); l["P1"] = world_neut(todo[0], sig + 0.01, lead=lead_sq); l["y0"] = l["yaw"]; l["y1"] = steer(todo[0], sig + 0.03)
                else:
                    mode = after; tm = 0.0
                    for l in leg: l["sq"] = False; l["ph"] = None
        elif mode == "push":                                              # all four planted: the body rides over the hooves, sinks, drives up and forward
            b_ = plan[bi]; tm += DT; u = min(tm / b_["tp"], 1.0); z_to = b_["z_to"]
            if tm < b_["t1"]: z_extra = herm(tm / b_["t1"], b_["zs"], -b_["vd"] * 1e3, b_["crouch"], 0.0, b_["t1"])
            else: z_extra = herm((tm - b_["t1"]) / b_["t2"], b_["crouch"], 0.0, z_to, b_["vz"] * 1e3, b_["t2"])
            vnow = b_["v_in"] + (b_["vx"] - b_["v_in"]) * u; sig = min(sig + vnow * DT, path.total)
            if u >= 1.0:
                mode = "flight"; tm = 0.0; tf = b_["tf"]; vx_leap, vz_leap = b_["vx"], b_["vz"]
                d_n = plan[bi + 1]["d"] if bi + 1 < len(seq) else lnd["d"]
                for i, l in enumerate(leg):
                    if not l["st"]: l["sw"] = (l["P0"].copy(), l["P1"].copy(), l["u"])
                    else: l.pop("sw", None)
                    l["st"] = False; l["P0"] = l["P"].copy(); l["P1"] = world_neut(i, sig + vx_leap * tf, lead=d_n * (PUSH_LEAD if bi + 1 < len(seq) else LAND_LEAD)); l["u"] = 0.0
                    xl, yl, hl = path.pose(sig + vx_leap * tf); Til = np.linalg.inv(body_T((xl * 1e3, yl * 1e3, zb + b_["zl"], hl * D, 0.0))); pb_ = (Til @ np.r_[l["P1"], C.HOOF_H, 1.0])[:3]
                    l["pl1"] = np.array([pb_[0] - C.LEG[KINDS[i]]["pivot"][0], SIDES[i] * pb_[1] - C.LEG[KINDS[i]]["pivot"][1], pb_[2] - C.LEG[KINDS[i]]["pivot"][2]]); l.pop("pl0", None)
        elif mode == "flight":
            tm += DT; tf = plan[bi]["tf"]; sig = min(sig + vx_leap * DT, path.total); z_extra = z_to + (vz_leap * tm - 0.5 * G * tm * tm) * 1e3
            if tm >= tf - 1e-9:
                tm = 0.0
                for l in leg: l["st"] = True; l["P"] = l["P1"].copy()
                if bi + 1 < len(seq): mode = "push"; bi += 1          # land straight into the next bound
                else: mode = "land"
        elif mode == "land":                                              # absorb and brake over the planted hooves
            tm += DT; u = min(tm / lnd["T"], 1.0); w_ = min(max((tm - lnd["td"]) / 0.25, 0.0), 1.0)
            z_extra = herm(tm / lnd["td"], lnd["zs"], -lnd["vd"] * 1e3, lnd["dip"], 0.0, lnd["td"]) if tm < lnd["td"] else lnd["dip"] * (1 - (3 * w_ ** 2 - 2 * w_ ** 3))
            vnow = vx_leap + (V_OUT - vx_leap) * u; sig = min(sig + vnow * DT, path.total)
            if u >= 1.0:
                mode = "square"; after = "gait"; lead_sq = 0.0; z_extra = 0.0; leap_i += 1          # square up again, then move off
                for l in leg: l["ph"] = None; l["sq"] = False
        t += DT
        x, y, h = path.pose(sig); k, spin = path.turn(sig)
        zt = -g["drop"] * 1e3; zb += (zt - zb) * min(1.0, DT / 0.25)
        b = (x * 1e3, y * 1e3, zb + z_extra, h * D, 0.0); Tb = body_T(b); Tbi = np.linalg.inv(Tb); row = []; dcl = dcl if (mode == "gait" and not done) else 0.0
        for i, l in enumerate(leg):
            kind, side = KINDS[i], SIDES[i]; n = NEUT[i] / 1e3
            if mode == "gait" and not done:
                if l.get("ph") is None: l["ph"] = (clock + pho[i]) % 1.0 if ((clock + pho[i]) % 1.0) < g["b"] else 0.0
                e_ = ((clock + pho[i]) - l["ph"] + 0.5) % 1.0 - 0.5                      # each leg keeps its own phase, pulled toward the gait's
                l["ph"] = (l["ph"] + dcl + float(np.clip(e_, -0.2 * dcl, 0.2 * dcl))) % 1.0; ph = l["ph"]; st = ph < g["b"]
                if l["st"] and st and (Tbi @ np.r_[l["P"], 0.0, 1.0])[0] - NEUT[i][0] < -(g["L"] * 500 + 20.0):   # planted too far back (speeding up): step now
                    l["ph"] = ph = g["b"]; st = False
                if l["st"] and not st:                                    # lift off: next footfall where the stand point will be at mid-stance
                    lam = g["L"] / g["b"]; l["P0"] = l["P"].copy(); s_n = sig + (1 - g["b"]) * lam + g["L"] / 2; l["P1"] = world_neut(i, s_n)
                    l["y0"] = l["yaw"]; l["y1"] = steer(i, s_n)
                if (not l["st"]) and st: l["P"] = l["P1"].copy()
                l["st"] = st; l["u"] = 0.0 if st else (ph - g["b"]) / (1 - g["b"]); l["du"] = dcl / (1 - g["b"])
            elif done:
                if not l["st"]:
                    l["u"] = min(l.get("u", 0.0) + DT / 0.25, 1.0); l["P1"] = world_neut(i, path.total); l["y1"] = steer(i, path.total)
                    if l["u"] >= 1.0: l["st"] = True; l["P"] = l["P1"].copy()
            elif mode == "push" and not l["st"]:                          # lift-off stride: a leg still in its swing carries on and plants
                l["u"] = min(l.get("u", 0.0) + l.get("du", DT / 0.2), 1.0)             # at the pace it had in the gait
                if l["u"] >= 1.0: l["st"] = True; l["P"] = l["P1"].copy()
            elif mode == "flight": l["u"] = min(tm / plan[bi]["tf"], 1.0)
            if l["st"]: hoof = np.array([l["P"][0], l["P"][1], 0.0])
            else:
                u = l["u"]; w = u * u * u * (10 - 15 * u + 6 * u * u); lift = (90.0 if mode == "flight" else g["clear"] * 1e3) * np.sin(np.pi * u) ** 2
                if mode == "flight": lift = 0.0                           # in flight the body carries the hooves; legs tuck below
                hoof = np.r_[l["P0"] + (l["P1"] - l["P0"]) * w, lift]
            # steering: leg plane along the hoof's travel direction in the body frame, within +-25 deg
            # the twist is set while the leg is in the air (for the turn at its next footfall) and held while it is planted
            if not l["st"] and "y1" in l:
                uu = l.get("u", 0.0); l["yaw"] = l["y0"] + (l["y1"] - l["y0"]) * (3 * uu * uu - 2 * uu ** 3)
            phi = 0.0; q = l["q"]; res = None; hpm = M.HP_MAX
            for _ in range(3):
                def planted(hf):
                    ball = hf + [0, 0, C.HOOF_H * np.cos(phi / D) + M.R_HOOF * abs(np.sin(phi / D))]; pb_ = (Tbi @ np.r_[ball, 1.0])[:3]
                    return np.array([pb_[0] - C.LEG[kind]["pivot"][0], side * pb_[1] - C.LEG[kind]["pivot"][1], pb_[2] - C.LEG[kind]["pivot"][2]])
                if mode == "flight":                                      # legs tuck under the body, blending out of the take-off pose and into the landing one
                    uu = l["u"]
                    if "pl0" not in l:
                        if "sw" in l: p0_, p1_, us_ = l.pop("sw"); ws_ = us_ ** 3 * (10 - 15 * us_ + 6 * us_ * us_); l["pl0"] = planted(np.r_[p0_ + (p1_ - p0_) * ws_, g["clear"] * 1e3 * np.sin(np.pi * us_) ** 2])
                        else: l["pl0"] = planted(np.r_[l["P0"], 0.0])
                    wa = uu * uu * uu * (10 - 15 * uu + 6 * uu * uu)
                    pl = (1 - wa) * l["pl0"] + wa * l["pl1"] + np.array([0.0, 0.0, (TUCK_RUN if running and bi == 0 else TUCK) * np.sin(np.pi * uu) ** 2])       # leg-frame blend: the hooves rise with the body
                else:
                    l.pop("pl0", None)
                    ball = hoof + [0, 0, C.HOOF_H * np.cos(phi / D) + M.R_HOOF * abs(np.sin(phi / D))]
                    pb = (Tbi @ np.r_[ball, 1.0])[:3]; pl = np.array([pb[0] - C.LEG[kind]["pivot"][0], side * pb[1] - C.LEG[kind]["pivot"][1], pb[2] - C.LEG[kind]["pivot"][2]])
                    if not l["st"] and np.linalg.norm(pl) > RR[kind]:           # the swing hoof rides up on the reach circle (eased in from lift-off)
                        wc = min(l.get("u", 1.0) / 0.25, 1.0); pl[2] = (1 - wc) * pl[2] + wc * -np.sqrt(max(RR[kind] ** 2 - pl[0] ** 2 - pl[1] ** 2, 1.0))
                res = ik3(kind, pl, l["yaw"], q)
                if res is not None and not (25.0 < res[2] < 172.0): res = ik3(kind, pl, l["yaw"], (C.STAND[kind]["q0"], C.STAND[kind]["q1"]))   # wrong elbow branch: restart from the stand
                if res is None: break
                hpm = np.sqrt(max(M.HP_MAX ** 2 - res[0] ** 2, 1.0))                 # the ball joint's 15 deg is a cone: sideways swing uses part of it
                q = (res[1], res[2]); hh = res[1] + C.LEG[kind]["delta"]; phi = 0.0 if abs(hh) <= hpm else np.sign(hh) * hpm - hh
            if res is None or res[3] > 1e-3: bad["ik"] += 1; res = res or (0.0, q[0], q[1], 9.0)
            abd, q0, q1, err = res
            if not l["st"]:                                              # in the air the hoof need not be exactly on its path: stay inside the joint ranges
                rg = V.RANGE[kind]; abd = float(np.clip(abd, -9.5, 9.5)); q0 = float(np.clip(q0, rg["q0"][0] + 0.5, rg["q0"][1] - 0.5)); q1 = float(np.clip(q1, rg["q1"][0] + 0.5, rg["q1"][1] - 0.5))
            l["q"] = (q0, q1); hh = q0 + C.LEG[kind]["delta"]
            if mode == "flight":                                          # world hoof position for the record
                pw = Tb @ np.r_[pl[0] + C.LEG[kind]["pivot"][0], side * (pl[1] + C.LEG[kind]["pivot"][1]), pl[2] + C.LEG[kind]["pivot"][2], 1.0]; hoof = pw[:3] - [0, 0, C.HOOF_H]
            row.append(dict(q0=float(q0), q1=float(q1), fold=0.0, st=bool(l["st"]), hp=float(np.clip(hh, -hpm, hpm)), yaw=float(l["yaw"]), abd=float(abd), hr=float(-abd),
                            hoof=[float(v_) for v_ in hoof], roll=float(phi)))
        frames.append(dict(t=t, sig=sig, v=vnow if not done else 0.0, mode=mode, leap=leap_i, bound=bi if mode in ("push", "flight") else -1, b=[float(v_) for v_ in b], l=row))
    return dict(frames=frames, dt=DT, bad=bad, label=label, time=t - 0.7)


# ------------------------------------------------------------------ checks and loads
W_FREE = lambda name: S.ACT[name]["w_free_48"] * S.BUS_V / 48.0 * 6.0
SPD = dict(fore=dict(q0=W_FREE("RobStride 03"), q1=W_FREE(M.ACTS["knee"]["front"]) / M.ACTS["ratio"]["front"], yaw=W_FREE("RobStride 02") / 2, abd=W_FREE("RobStride 06")),
           hind=dict(q0=W_FREE("RobStride 06"), q1=W_FREE("RobStride 06"), yaw=W_FREE("RobStride 02") / 2, abd=W_FREE("RobStride 02")))
ABD_ACT = dict(fore=S.ACT["RobStride 06"], hind=S.ACT["RobStride 02"])
_env = dict(neck=T.neck_block(), shell=T.shell_solid(), octo=None)


def analyze(r, dist=True, nmax=110):
    fr = r["frames"]; n = len(fr); out = dict(time_s=r["time"], frames=n, ik_fail=r["bad"]["ik"])
    lim = {}; spd = {}; tilt = 0.0; roll = [0.0, 0.0]
    for i in range(4):
        kind = KINDS[i]; rg = V.RANGE[kind]
        for j in ("q0", "q1", "yaw", "abd"):
            v = np.array([f["l"][i][j] for f in fr]); key = kind + " " + j
            lim[key] = [min(lim.get(key, [9e9, -9e9])[0], float(v.min())), max(lim.get(key, [9e9, -9e9])[1], float(v.max()))]
            spd[key] = min(spd.get(key, 9e9), SPD[kind][j] / max(float(np.abs(np.diff(v)).max()) / r["dt"], 1e-6))
        tilt = max(tilt, max(float(np.hypot(f["l"][i]["hp"], f["l"][i]["hr"])) for f in fr))
        rr = [f["l"][i]["roll"] for f in fr]; roll = [min(roll[0], min(rr)), max(roll[1], max(rr))]
    out["range"] = lim; out["limit_violations"] = [k for k, (lo, hi) in lim.items() if lo < V.RANGE[k.split()[0]][k.split()[1]][0] - 0.05 or hi > V.RANGE[k.split()[0]][k.split()[1]][1] + 0.05]
    out["speed_margin_min"] = {k: round(v, 2) for k, v in spd.items() if v < 50}; out["hoof_tilt_max_deg"] = tilt; out["toe_heel_roll_deg"] = roll
    # sideways-swing (abduction) motor torque: ground force x its sideways lever about the axis, plus the turn's side force x height (est.)
    W = MASS * G; tq = dict(fore=[], hind=[])
    for f in fr:
        k = 0.0
        for kind, share in (("fore", 0.6), ("hind", 0.4)):
            ids = [i for i in range(4) if KINDS[i] == kind]; nst = sum(f["l"][i]["st"] for i in ids)
            for i in ids:
                l = f["l"][i]
                if not l["st"] or f["mode"] != "gait": tq[kind].append(0.0); continue
                Gz = W * share / nst; al = l["abd"] / D; ya, za = C.ABD_AXES[kind]
                yh = ya + (0 - ya) * np.cos(al) + (C.LEG[kind]["pivot"][2] + f["b"][2] + za) * np.sin(al)        # hoof sideways from the pivot line (leg frame, approx.)
                tq[kind].append(abs(Gz * (yh - ya) / 1e3) + MASS * share / nst * f["v"] ** 2 * abs(k) * 0.47)
    out["abduction"] = {}
    for kind in tq:
        a = np.array(tq[kind]); act = ABD_ACT[kind]
        out["abduction"][kind] = dict(rms_Nm=float(np.sqrt(np.mean(a ** 2))), peak_Nm=float(a.max()), rated=act["rated"], peak=act["peak"],
                                      margin=float(min(act["rated"] / max(np.sqrt(np.mean(a ** 2)), 1e-6) / 1.2, act["peak"] / max(a.max(), 1e-6) / 1.1)))
    if dist:
        step = max(1, n // nmax); d = dict(in_leg=(99.0, ""), under_rule=[], leg_leg=(99.0, ""), neck=(99.0, ""), ground=(99.0, ""))
        for kf in range(0, n, step):
            f = fr[kf]; ex = {i: dict(yaw=f["l"][i]["yaw"], abd=f["l"][i]["abd"], hr=f["l"][i]["hr"]) for i in range(4)}
            for i, l in enumerate(f["l"]):
                p = T.params(l, ex[i])
                try: w, bad, el = V.check(KINDS[i], p)
                except ValueError: continue
                if w[0] < d["in_leg"][0]: d["in_leg"] = (float(w[0]), "f%d %s: %s | %s" % (kf, KINDS[i], w[1][:28], w[2][:28]))
                d["under_rule"] += [(kf, KINDS[i], b_[0], b_[2][:30], b_[3][:30]) for b_ in bad if b_[0] < b_[1] - 1e-3]
            rc = T.robot_check(dict(b=[0.0, f["b"][2], f["b"][4]], l=f["l"]), _env, ex, body_checks=False)
            for key in ("leg_leg", "neck"):
                if rc[key][0] < d[key][0]: d[key] = (rc[key][0], "f%d %s" % (kf, rc[key][1][:70]))
        d["under_rule"] = d["under_rule"][:8]; out["distance"] = d
    return out


def brief(name, a):
    d = a.get("distance", {})
    print("%-14s %5.2f s  ik fail %d  limits %s  speed min %s  tilt %.1f  abd margin fore %.2f hind %.2f  in-leg %s  leg-leg %s  neck %s  under %d" % (
        name, a["time_s"], a["ik_fail"], a["limit_violations"] or "ok", min(a["speed_margin_min"].values()) if a["speed_margin_min"] else "-", a["hoof_tilt_max_deg"],
        a["abduction"]["fore"]["margin"], a["abduction"]["hind"]["margin"], round(d["in_leg"][0], 2) if d else "-", round(d["leg_leg"][0], 1) if d else "-",
        round(d["neck"][0], 1) if d else "-", len(d.get("under_rule", []))), flush=True)


def ok(a):
    return a["ik_fail"] == 0 and not a["limit_violations"] and a["hoof_tilt_max_deg"] <= 15.01 and (not a["speed_margin_min"] or min(a["speed_margin_min"].values()) >= 1.5)


def leap_loads(r, seq, v0=0.03, running=False):
    """Ground covered and time of the whole bounding sequence, and the leg and bus loads the plan asks for (est.):
    vertical force from the planned body acceleration, the peak-torque force the motors have at each crouch depth at
    standstill (push model), mechanical power at take-off and the bus current at 80 % drive efficiency."""
    fr = [f for f in r["frames"] if f["mode"] in ("push", "flight", "land")]; plan, lnd = leap_plan(seq, v0, running); m = big(seq); o = S.OPT["B"]
    def f_static(dz_mm):
        tot = []
        for leg, kind in (("front", "fore"), ("rear", "hind")):
            mm = o["legs"][leg]; z = (np.linalg.norm(M.ball_xz(kind, C.STAND[kind]["q0"], C.STAND[kind]["q1"])) + C.HOOF_H + dz_mm) * 1e-3
            J = S.jac(mm, mm.ik(o["x0"][leg], -z)); F = 1e9
            for j, (name, ratio) in enumerate(((M.ACTS["hip"][leg], 1.0), (M.ACTS["knee"][leg], M.ACTS["ratio"][leg]))):
                if abs(J[1, j]) > 1e-6: F = min(F, S.ACT[name]["peak"] * ratio * (S.BELT_EFF if ratio != 1 else 1.0) / abs(J[1, j]))
            tot.append(2 * F / S.share(leg))
        return float(min(tot))
    acc_dn = max((b["vd"] ** 2 / (2 * max(b["zs"] - b["crouch"], 1e-6) * 1e-3) for b in plan if b["vd"] > 0.2), default=0.0)
    acc_dn = max(acc_dn, lnd["vd"] ** 2 / (2 * (lnd["zs"] - lnd["dip"]) * 1e-3)); acc_up = max(b["vz"] ** 2 / (2 * (b["z_to"] - b["crouch"]) * 1e-3) for b in plan[(1 if running else 0):])
    deep = min(min(b["crouch"] for b in plan), lnd["dip"]); p_to = MASS * (A_NET + G) * plan[m]["vz"]
    return dict(sequence_m=fr[-1]["sig"] - fr[0]["sig"], sequence_s=fr[-1]["t"] - fr[0]["t"], bounds=len(seq), main_stance_s=plan[m]["tp"], main_stance_travel_m=plan[m]["d"],
                main_crouch_mm=plan[m]["crouch"], deepest_crouch_mm=deep, push_force_N=MASS * (acc_up + G), brake_force_N=MASS * (acc_dn + G),
                force_available_N=dict(stand=f_static(0.0), main_crouch=f_static(plan[m]["crouch"]), deepest=f_static(deep)),
                takeoff_power_W=p_to, bus_current_A=p_to / 0.8 / S.BUS_V, land_sink_m_s=max(b["vdn"] for b in plan))


# ------------------------------------------------------------------ capabilities
def capabilities():
    cap = {}
    # high jump: study push model at the CAD mass (body rise after the hooves leave), plus the tuck the joints allow in the air
    rise = VZ ** 2 / (2 * G); tuck = {}
    for kind in ("fore", "hind"):
        st = np.linalg.norm(M.ball_xz(kind, C.STAND[kind]["q0"], C.STAND[kind]["q1"])); mn = np.linalg.norm(M.ball_xz(kind, C.STAND[kind]["q0"], M.LIM[kind]["q1"] - 1))
        top = 0.97 * S.OPT["B"]["legs"][M.KEY[kind]].lmax * 1e3; tuck[kind] = dict(stand=float(st), shortest=float(mn), at_takeoff=float(top - C.HOOF_H))
    cap["jump"] = dict(takeoff_speed=VZ, body_rise_mm=rise * 1e3, hooves_off_ground_mm=float(JUMP[0]) * 1e3, flight_s=2 * VZ / G,
                       hoof_height_tucked_mm={k: rise * 1e3 + (v["at_takeoff"] - v["shortest"]) for k, v in tuck.items()})
    cap["jump"]["bar_cleared_mm"] = min(cap["jump"]["hoof_height_tucked_mm"].values()) - 20.0
    # leap: same take-off speed turned forward (est.); best angle 45 deg; a galloping leap keeps the vertical push
    cap["leap"] = dict(standing_mm=VZ ** 2 / G * 1e3, standing_vx=VZ / np.sqrt(2), standing_flight_s=2 * VZ / np.sqrt(2) / G,
                       from_gallop_mm=1.6 * 2 * VZ / G * 1e3, note="standing leap simulated; the galloping leap is the flight formula only")
    return cap


def tight_turns():
    """Smallest circle at a trot and the fastest spin, by running them."""
    res = {}
    for v in (0.6, 0.9):
        best = None
        for R in (1.2, 1.0, 0.8, 0.7, 0.6, 0.5, 0.45, 0.4, 0.35, 0.3):
            r = run(Path([("line", 0.4), ("arc", R, 200.0), ("line", 0.2)]), vcap=min(v, np.sqrt(A_LAT * R)))
            a = analyze(r, dist=False)
            if not ok(a): break
            best = (R, r, a)
        res[v] = best
    spin = None
    for vs in (0.15, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5):
        r = run(Path([("spin", 180.0)]), vspin=vs); a = analyze(r, dist=False)
        if not ok(a): break
        spin = (vs, r, a)
    return res, spin


# ------------------------------------------------------------------ sprinting long jump
def sprint_seq(vs, vx, vz, dv=0.55):
    """Running leap: lift-off stride out of the gait at vs, main bound speeding up to vx at vz, bounding strides down."""
    n = max(int(np.ceil((vx - 0.03) / dv)) - 1, 0)
    return [(vs, HOP_RUN), (vx, vz)] + [(0.03 + (vx - 0.03) * (i + 1) / (n + 1), HOP) for i in range(n)][::-1]


def sprint_jump(runup=3.0, tries=((1.7, 0.8), (1.7, 0.75), (1.8, 0.75), (1.5, 0.85))):
    """Gallop up to speed, leap without stopping.  Stage 1: where in the stride the lift-off can start (at least two legs
    planted through it, every joint check passed).  Stage 2: take-off speed and push from those points.  The longest
    that also passes the clearance check on every frame is kept.  -> (cap dict, run, analysis, trials)"""
    trials = []; cands = []
    def one(vs, vx, k, off):
        sq = sprint_seq(vs, vx, k * VZ); r = run(Path([("line", runup + off), ("line", 4.0)]), vcap=vs, leaps=[(runup + off, sq, True)]); a = analyze(r, dist=False)
        npl = min(sum(l["st"] for l in f["l"]) for f in r["frames"] if f["mode"] == "push" and f["bound"] == 0)
        L = vx * leap_plan(sq, vs, True)[0][1]["tf"] * 1e3; good = ok(a) and npl >= 2
        trials.append((vs, vx, k, off, round(L), bool(good), npl, min(a["speed_margin_min"].values()), a["limit_violations"], a["ik_fail"]))
        if good: cands.append((L, vs, vx, k, off, sq, r))
        return good
    for vs in (1.3, 1.2):
        offs = [off for off in np.arange(0.06, 0.52, 0.03).round(2).tolist() if one(vs, 1.6, 0.8, off)]
        for off in offs[:2]:
            for vx, k in tries: one(vs, vx, k, off)
    for best in sorted(cands, key=lambda c: (-round(c[0]), -c[1])):
        fr = best[6]["frames"]; a = analyze(dict(frames=fr[next(i for i, f in enumerate(fr) if f["v"] > 0.5):], dt=DT, bad=best[6]["bad"], time=best[6]["time"]), nmax=10 ** 6)
        if a["distance"]["leg_leg"][0] >= 1.5 and not a["distance"]["under_rule"]: break
        trials.append(best[1:5] + ("rejected: " + a["distance"]["leg_leg"][1][:70],))
    L, vs, vx, k, off, sq, r = best
    a["time_s"] = r["time"]; mf = [f for f in fr if f["mode"] == "flight" and f["bound"] == 1]; lift = [f for f in fr if f["mode"] == "push" and f["bound"] == 0]
    i0 = fr.index(mf[0]); i1 = fr.index(mf[-1]) + 1                         # last planted frame, first landed frame
    cap = dict(sprint_speed=vs, takeoff_vx=vx, push_fraction=k, flight_mm=L, flight_s=leap_plan(sq, vs, True)[0][1]["tf"], runup_m=runup + off,
               hoof_to_hoof_mm={n: float(np.hypot(fr[i1]["l"][j]["hoof"][0] - fr[i0 - 1]["l"][j]["hoof"][0], fr[i1]["l"][j]["hoof"][1] - fr[i0 - 1]["l"][j]["hoof"][1])) for n, j in (("fore", 0), ("hind", 2))},
               body_top_mm=max(f["b"][2] for f in mf) - fr[0]["b"][2], hooves_up_mm=min(min(l["hoof"][2] for l in f["l"]) for f in mf[len(mf) // 3: 2 * len(mf) // 3 + 1]),
               bus_V=S.BUS_V, takeoff_vz=k * VZ, lift_legs_planted=min(sum(l["st"] for l in f["l"]) for f in lift), lift_force_N=MASS * (HOP_RUN / T_LIFT + G), loads=leap_loads(r, sq, vs, True))
    cap["lift_force_per_leg_N"] = cap["lift_force_N"] / max(cap["lift_legs_planted"], 1); cap["leg_force_available_N"] = cap["loads"]["force_available_N"]["stand"] / 4
    return cap, r, a, trials


# ------------------------------------------------------------------ the course
def course(Rb, a_w, v_top, leap_vx, leap_vz, bar_vx=None, bar_vz=None):
    """Start, pole weave (4 poles 1.2 m apart), bar, barrel loop, ditch leap, turn-around box, sprint home."""
    Dp = 1.2; Rw = Dp / (2 * np.sin(np.radians(a_w)))
    segs = [("line", 1.0), ("arc", Rw, a_w), ("arc", Rw, -2 * a_w), ("arc", Rw, 2 * a_w), ("arc", Rw, -2 * a_w), ("arc", Rw, a_w), ("line", 0.5),
            ("line", 0.9), ("line", 2.4),                                  # gather + bounding run-up + bar, run-out
            ("arc", Rb, 180.0), ("line", 0.6), ("arc", Rb, 270.0), ("line", 0.8),       # half turn, barrel loop
            ("line", 0.9), ("line", 2.4),                                  # ditch
            ("arc", 0.8, 90.0), ("spin", 180.0), ("line", 3.0)]            # quarter turn off the ditch line, turn-around box, sprint home
    p = Path(segs); s = [sg["s0"] for sg in p.segs]
    leaps = [(s[7] + 0.45, bounds(bar_vx or leap_vx, 0.9 * (bar_vz or leap_vz))), (s[13] + 0.45, bounds(leap_vx, leap_vz))]
    return p, leaps, dict(Rw=Rw, bar_sig=s[7] + 0.45, ditch_sig=s[13] + 0.45, weave_s=(s[1], s[6]), barrel_seg=11, spin_seg=16)


if __name__ == "__main__":
    out = dict(mass=MASS, gaits={n: dict(v=g["v"], stride=g["L"], duty=g["b"]) for n, g in GA.items()}, limits=dict(a_long=A_LONG, a_lat=A_LAT))
    cap = capabilities(); out["capabilities"] = cap; print(json.dumps(cap, default=float)[:900])
    demos = {}
    # 1 high jump: the rev B jump motion (already tested in legs_motion_test)
    # 2 bounding leaps: run-up bounds, then the big one.  Longest (take-off to landing of the last bound) and highest with real carry
    best = None; hb = None; trials = []
    for vx in (1.6, 1.5, 1.4, 1.3, 1.2, 1.0, 0.8):
        for k in (1.0, 0.95, 0.9, 0.85, 0.8, 0.7):
            vz = k * VZ; sq_ = bounds(vx, vz)
            r = run(Path([("line", 6.0)]), vcap=0.4, leaps=[(0.5, sq_)]); a = analyze(r, dist=False)
            fl = [f for f in r["frames"] if f["mode"] == "flight" and f["bound"] == big(sq_)]
            good = ok(a) and len(fl) > 5; dist_mm = vx * leap_plan(sq_)[0][big(sq_)]["tf"] * 1e3
            trials.append((vx, k, round(dist_mm), bool(good), min(a["speed_margin_min"].values()) if a["speed_margin_min"] else 0, a["limit_violations"], a["ik_fail"]))
            if not good: continue
            mid = fl[len(fl) // 3: 2 * len(fl) // 3 + 1]; hz = min(min(l["hoof"][2] for l in f["l"]) for f in mid)
            if best is None or dist_mm > best[0]: best = (dist_mm, vx, k, r, vx, vz, sq_)
            if vx >= 0.8 and (hb is None or hz > hb[0]): hb = (hz, vx, k, r, vx, vz, (mid[-1]["sig"] - mid[0]["sig"]) * 1e3, sq_, dist_mm)
    out["leap_trials"] = trials; print("leap trials (vx, push fraction, length mm, ok, speed margin, limits, ik):", trials)
    cap["leap"]["simulated_mm"] = best[0]; cap["leap"]["takeoff_vx"] = best[1]; cap["leap"]["simulated_push_fraction"] = best[2]; cap["leap"]["run_up_bounds"] = big(best[6])
    a = analyze(best[3]); brief("leap", a); demos["leap"] = (best[3], a)
    cap["leap"]["loads"] = leap_loads(best[3], best[6]); cap["jump"]["moving_bar_loads"] = leap_loads(hb[3], hb[7]); print("leap loads", cap["leap"]["loads"])
    a = analyze(hb[3]); brief("bar leap", a); demos["bar"] = (hb[3], a)
    cap["jump"]["moving_bar_mm"] = hb[0] - 15.0; cap["jump"]["moving_bar_vx"] = hb[1]; cap["jump"]["moving_bar_push_fraction"] = hb[2]; cap["jump"]["moving_bar_span_mm"] = hb[6]; cap["jump"]["moving_bar_leap_mm"] = hb[8]
    cap["sprint_jump"], r, a, out["sprint_trials"] = sprint_jump(); brief("sprint jump", a); demos["sprint"] = (r, a)
    turns, spin = tight_turns()
    for v, bt in turns.items():
        cap.setdefault("turn", {})["trot %.1f m/s" % v] = dict(radius_m=bt[0] if bt else None)
    Rmin = turns[0.6][0]; r = run(Path([("line", 0.4), ("arc", Rmin, 360.0), ("line", 0.3)]), vcap=min(0.6, np.sqrt(A_LAT * Rmin))); a = analyze(r); brief("circle R %.2f" % Rmin, a); demos["circle"] = (r, a)
    cap["turn"]["tightest_radius_m"] = Rmin; cap["turn"]["circle_time_s"] = a["time_s"]
    r = run(Path([("spin", 180.0)]), vspin=spin[0]); a = analyze(r); brief("spin 180", a); demos["spin"] = (r, a)
    cap["turn_around"] = dict(time_s=a["time_s"], spin_rate_deg_s=spin[0] / R_SPIN * D)
    # course: pick the barrel radius, weave angle and top speed that give the shortest time (speed profile only), then run it in full
    lvx, lvz = 0.9 * best[4], 0.9 * best[5]; tab = []
    for Rb in (max(Rmin * 1.15, 0.45), 0.6, 0.75, 0.9, 1.1):
        for a_w in (35.0, 45.0, 55.0):
            for v_top in (0.9, 1.2):
                p, leaps, info = course(Rb, a_w, v_top, lvx, lvz, hb[4], hb[5])
                amp = info["Rw"] * (1 - np.cos(np.radians(a_w)))
                if amp < 0.30 or info["Rw"] < Rmin * 1.15: continue                 # 0.30 m between the body's centre line and a pole
                sg_, vp = speed_profile(p, v_top, spin[0] * 0.85, [(lp[0] - 0.4, lp[0] + 0.05, 0.3) for lp in leaps])
                tab.append((round(profile_time(sg_, vp), 2), round(Rb, 2), a_w, v_top, round(p.total, 2)))
    tab.sort(); out["course_options"] = tab[:12]; print("course options (time, barrel R, weave angle, top speed, length):", tab[:6])
    done = None
    tried = []
    for tt, Rb, a_w, v_top, _ in tab[:8]:
        p, leaps, info = course(Rb, a_w, v_top, lvx, lvz, hb[4], hb[5]); r = run(p, v_top, spin[0] * 0.85, leaps); a = analyze(r, dist=False); brief("course try", a)
        if not ok(a): tried.append((tt, Rb, a_w, v_top, "joint limits or speed")); continue
        a = analyze(r, nmax=160); brief("course", a)
        if a["distance"]["leg_leg"][0] < 1.5 or a["distance"]["under_rule"]: tried.append((tt, Rb, a_w, v_top, "legs touch: " + a["distance"]["leg_leg"][1][:80])); continue
        done = (p, leaps, info, r, Rb, a_w, v_top); break
    out["course_rejected"] = tried; p, leaps, info, r, Rb, a_w, v_top = done; demos["course"] = (r, a)
    out["course"] = dict(barrel_radius=Rb, weave_angle=a_w, weave_radius=info["Rw"], top_speed=v_top, length_m=p.total, leap_vx=lvx, leap_vz=lvz, leap_len_mm=lvx * leap_plan(bounds(lvx, lvz))[0][big(bounds(lvx, lvz))]["tf"] * 1e3, bar_vx=hb[4], bar_vz=0.9 * hb[5], bar_leap_mm=hb[4] * leap_plan(bounds(hb[4], 0.9 * hb[5]))[0][big(bounds(hb[4], 0.9 * hb[5]))]["tf"] * 1e3)
    # obstacles from the chosen path (mm, world)
    obs = []
    def at(sig, off=0.0):
        x, y, h = p.pose(sig); return [(x - off * np.sin(h)) * 1e3, (y + off * np.cos(h)) * 1e3, h * D]
    obs = []
    for j, sgi in enumerate((2, 3, 4)):                                    # poles on the inside of each weave arc, 0.30 m from the path at its apex
        sg = p.segs[sgi]; mid = sg["s0"] + sg["L"] / 2; q = at(mid, np.sign(sg["k"]) * 0.30); obs.append(dict(kind="pole", x=q[0], y=q[1]))
    q0_ = at(p.segs[1]["s0"], -0.30 * np.sign(p.segs[1]["k"])); obs.insert(0, dict(kind="pole", x=q0_[0], y=q0_[1]))
    bigf = lambda li: [f for f in r["frames"] if f["mode"] == "flight" and f["leap"] == li and f["bound"] == big(leaps[li][1])]
    f1, f2 = bigf(0), bigf(1)
    bar_h = 0.8 * (min(min(l["hoof"][2] for l in f["l"]) for f in f1[len(f1) // 3: 2 * len(f1) // 3]))
    m1 = f1[len(f1) // 2]; obs.append(dict(kind="bar", x=m1["b"][0], y=m1["b"][1], h=m1["b"][3], height=float(bar_h), width=900.0))
    if f2:
        m2 = f2[len(f2) // 2]; obs.append(dict(kind="ditch", x=m2["b"][0], y=m2["b"][1], h=m2["b"][3], length=float(0.8 * out["course"]["leap_len_mm"]), width=900.0))
    sg = p.segs[info["barrel_seg"]]; c = at(sg["s0"], np.sign(sg["k"]) * Rb); obs.append(dict(kind="barrel", x=c[0], y=c[1], r=float((Rb - 0.33) * 1e3)))
    e = at(p.total); obs.append(dict(kind="gate", x=e[0], y=e[1], h=e[2])); s_ = at(0.0); obs.append(dict(kind="gate", x=s_[0], y=s_[1], h=s_[2]))
    sp = at(p.segs[info["spin_seg"]]["s0"]); obs.append(dict(kind="box", x=sp[0], y=sp[1], h=sp[2]))
    out["course"]["obstacles"] = obs; out["course"]["bar_height_mm"] = float(bar_h)
    pts = np.array([[f["b"][0], f["b"][1]] for f in r["frames"]])              # the body's centre line against the solid obstacles
    out["course"]["path_to_obstacle_mm"] = {o["kind"] + str(j): float(np.min(np.hypot(pts[:, 0] - o["x"], pts[:, 1] - o["y"])) - o.get("r", 0.0)) for j, o in enumerate(obs) if o["kind"] in ("pole", "barrel")}
    # splits
    marks = [("weave", info["weave_s"][1]), ("bar", info["bar_sig"] + 2.4), ("barrel", p.segs[12]["s0"] + p.segs[12]["L"]), ("ditch", info["ditch_sig"] + 2.4),
             ("turn-around", p.segs[17]["s0"]), ("finish", p.total)]
    out["course"]["splits_s"] = {nm: float(next((f["t"] for f in r["frames"] if f["sig"] >= sg_ - 1e-6), r["time"])) for nm, sg_ in marks}
    out["results"] = {k: v[1] for k, v in demos.items()}
    json.dump(out, open(os.path.join(HERE, "agility.json"), "w"), indent=1, default=lambda o: float(o) if isinstance(o, (np.floating, np.integer)) else str(o))
    # joint angle table of the course (60 Hz)
    cols = ["t_s", "x_mm", "y_mm", "z_mm", "heading_deg", "speed_m_s", "mode"] + ["%s_%s" % (n, j) for n in ("FL", "FR", "HL", "HR") for j in ("swing_deg", "twist_deg", "pivot_deg", "flex_deg", "hoof_pitch_deg", "stance")]
    with open(os.path.join(HERE, "agility_course.csv"), "w") as fcsv:
        fcsv.write(",".join(cols) + "\n")
        for f in r["frames"]:
            row = ["%.4f" % f["t"], "%.1f" % f["b"][0], "%.1f" % f["b"][1], "%.1f" % f["b"][2], "%.2f" % f["b"][3], "%.3f" % f["v"], f["mode"]]
            for l in f["l"]: row += ["%.2f" % l["abd"], "%.2f" % l["yaw"], "%.2f" % l["q0"], "%.2f" % l["q1"], "%.2f" % l["hp"], "%d" % l["st"]]
            fcsv.write(",".join(row) + "\n")
    # viewer data: every 2nd frame
    mot, _, _ = M.build(); jf = mot["jump"]
    vd = dict(motions={}, obstacles=obs, cap=cap, course=out["course"], results={k: dict(time_s=v[1]["time_s"], abd=v[1]["abduction"], dist=v[1].get("distance")) for k, v in demos.items()})
    pk = lambda f: dict(b=[round(v_, 1) for v_ in f["b"][:3]] + [round(f["b"][3], 2), round(f["b"][4], 2)],
                        l=[[round(l["q0"], 2), round(l["q1"], 2), 0, int(l["st"]), round(l["hp"], 2), round(l["yaw"], 2), round(l["abd"], 2), round(l["hr"], 2)] for l in f["l"]])
    for k, (rr, _) in demos.items(): vd["motions"][k] = dict(dt=2 * DT, frames=[pk(f) for f in rr["frames"][::2]])
    vd["motions"]["jump"] = dict(dt=jf["dt"], frames=[dict(b=[f["b"][0], 0.0, f["b"][1], 0.0, f["b"][2]], l=[[round(l["q0"], 2), round(l["q1"], 2), 0, int(l["st"]), round(l["hp"], 2), 0, 0, 0] for l in f["l"]]) for f in jf["frames"]])
    os.makedirs(os.path.join(HERE, "viewer_agility"), exist_ok=True)
    open(os.path.join(HERE, "viewer_agility", "agility_data.js"), "w").write("window.AGI = " + json.dumps(vd, separators=(",", ":"), default=float) + ";")
    print("saved", round(r["time"], 2), "s course,", len(r["frames"]), "rows")
