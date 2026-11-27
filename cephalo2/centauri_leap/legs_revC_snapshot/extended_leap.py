#!/usr/bin/env python3
"""Centauri-1 extended bounding leap (rev C legs): fore pair and hind pair work in turn, the body pitches, the legs reach
fully forward and trail fully back in the long flight, and the landing is fore pair first, then the hinds come under.

Unlike agility.py's level four-leg bounds this plan is dynamically consistent in the side view (est. rigid body, massless
legs): in every single-pair stance the vertical force follows from the planned height, and the horizontal force is the one
that gives the planned pitch acceleration about the centre of mass (I th'' = dx Fz + h Fx), so the forward speed is an
OUTPUT, not an input: fore pairs brake, hind pairs drive, as in a real bound.
Checked per frame: joint ranges, IK, joint speeds (rule x1.5), leg force against peak motor torque through the leg
Jacobian (rule x1.1), grip (Fx / Fz), hoof ground clearance in the air; then agility.analyze for clearances.
NOT simulated: leg mass (the legs are 42 % of the robot), the two legs of a pair sharing unevenly, compliance, slip.
    python3 extended_leap.py [search N] -> extended_leap.json, viewer_agility/agility_data.js (motion "extleap"), agility_extended_leap.csv"""
import os, sys, json
import numpy as np
from scipy.interpolate import CubicSpline
import agility as A
M, C, S, T, V = A.M, A.C, A.S, A.T, A.V
D = A.D; G = A.G; DT = A.DT; MASS = A.MASS
COMB = np.array([87.5, 0.0, 450.0])     # centre of mass in the body frame (mm): 60 % of the weight on the fore pair (study), height est.
I_P = 2.3                               # kg m2 pitch inertia about the centre of mass (est.: 41.65 kg, radius of gyration 0.235 m)
PIV = {k: C.LEG[k]["pivot"] for k in ("fore", "hind")}
YIN = A.NEUT[0][1] - PIV["fore"][1]     # hooves run 40 mm inboard of the pivots (leg-frame y)
TQ = {"fore": (S.ACT[M.ACTS["hip"]["front"]]["peak"], S.ACT[M.ACTS["knee"]["front"]]["peak"] * M.ACTS["ratio"]["front"] * S.BELT_EFF),
      "hind": (S.ACT[M.ACTS["hip"]["rear"]]["peak"], S.ACT[M.ACTS["knee"]["rear"]]["peak"] * M.ACTS["ratio"]["rear"])}
u = lambda a: np.array([np.sin(a), -np.cos(a)])


def R2(th):
    c, s = np.cos(th / D), np.sin(th / D); return np.array([[c, -s], [s, c]])      # body (x, z) -> world, nose-up positive


def hip_w(kind, cx, cz, th):
    """World (x, z) of a pivot for the centre of mass at (cx, COMB z + cz) and pitch th."""
    return np.array([cx, COMB[2] + cz]) + R2(th) @ (PIV[kind][[0, 2]] - COMB[[0, 2]])


def b_row(cx, cz, th):
    """agility frame body entry [x, y, z, heading, pitch] for that pose."""
    o = np.array([cx, COMB[2] + cz]) - R2(th) @ COMB[[0, 2]]; B0 = T.body4((0.0, 0.0, th))[:3, 3]
    return [float(o[0] - B0[0]), 0.0, float(o[1] - B0[2]), 0.0, float(th)]


def herm(w, z0, v0, z1, v1, Tm):
    w = min(max(w, 0.0), 1.0); w2, w3 = w * w, w * w * w
    return (2 * w3 - 3 * w2 + 1) * z0 + (w3 - 2 * w2 + w) * Tm * v0 + (-2 * w3 + 3 * w2) * z1 + (w3 - w2) * Tm * v1


# ------------------------------------------------------------------ pass 1: body dynamics and footfalls
def simulate(P, start=None):
    """Body motion and footfalls of one extended leap.
    Phases: [stand, sit back] or a given start state (all four planted) -> push A (both pairs, weight moving onto the hinds, nose
    coming up) -> push B (hinds alone, to full extension) -> flight -> fore landing (fores alone) -> hinds down -> both pairs
    bring it to rest.  -> (samples at DT, ok)"""
    out = []; t = 0.0; hoof = {"fore": float(A.NEUT[0][0]), "hind": float(A.NEUT[2][0])}; x, z, th = float(COMB[0]), 0.0, 0.0; vx = vz = om = 0.0; flag = []
    def put(mode, st, **kw):
        out.append(dict(t=t, x=x, z=z, th=th, vx=vx, vz=vz, mode=mode, st=dict(st), hoof=dict(hoof), **kw))
    BOTH = dict(fore=True, hind=True)
    if start is None:
        for _ in range(int(0.3 / DT)): t += DT; put("stand", BOTH)
        c = P["crouch"]; x0, n = x, int(round(c["T"] / DT))                # sit back on the hinds, slowly (no dynamics claimed)
        for i in range(1, n + 1):
            w = i / n; s_ = 3 * w * w - 2 * w ** 3; t += DT; x = x0 + c["dx"] * s_; z = c["dz"] * s_; th = c["th"] * s_; put("gather", BOTH)
    else:
        t, x, z, th, vx, vz, om = (start[k] for k in ("t", "x", "z", "th", "vx", "vz", "om")); hoof = dict(start["hoof"])
    def double(q, mode, to_rest=False):
        """Both pairs planted: the motion is planned (x, z, pitch) and the fore share lam of the vertical force follows from the
        moment balance.  lam outside 0..1 means the plan would need a pair to pull on the ground: flagged."""
        nonlocal t, x, z, th, vx, vz, om
        Tt = q["T"]; n = max(int(round(Tt / DT)), 3); x0, z0, th0, vx0, vz0, om0 = x, z, th, vx * 1e3, vz * 1e3, om
        def pos(tt):
            w = tt / Tt
            if w > 1: return x0 + q["dx"] + q["vx"] * 1e3 * (tt - Tt), zend + q["vz"] * 1e3 * (tt - Tt), q["th"] + q["om"] * (tt - Tt)
            zz = herm(w, z0, vz0, zend, q["vz"] * 1e3, Tt) if "down" not in q else (herm(w / q["f"], z0, vz0, z0 - q["down"], 0.0, q["f"] * Tt) if w < q["f"] else herm((w - q["f"]) / (1 - q["f"]), z0 - q["down"], 0.0, zend, q["vz"] * 1e3, (1 - q["f"]) * Tt))
            return herm(w, x0, vx0, x0 + q["dx"], q["vx"] * 1e3, Tt), zz, herm(w, th0, om0, q["th"], q["om"], Tt)
        zend = q["z"] if "z" in q else z0 + q["dz"]
        for i in range(1, n + 1):
            p0, p1, p2 = pos((i - 1) * DT), pos(i * DT), pos((i + 1) * DT); ax = (p2[0] - 2 * p1[0] + p0[0]) / DT ** 2 * 1e-3; az = (p2[1] - 2 * p1[1] + p0[1]) / DT ** 2 * 1e-3; ath = (p2[2] - 2 * p1[2] + p0[2]) / DT ** 2 / D
            Fz = MASS * (az + G); Fx = MASS * ax; h = (COMB[2] + p1[1] - C.HOOF_H) * 1e-3; dxf, dxh = (hoof["fore"] - p1[0]) * 1e-3, (hoof["hind"] - p1[0]) * 1e-3
            lam = (I_P * ath - h * Fx - Fz * dxh) / (Fz * (dxf - dxh)) if Fz > 1 else 0.5
            vx = (p1[0] - x) / DT * 1e-3; vz = (p1[1] - z) / DT * 1e-3; om = (p1[2] - th) / DT; x, z, th = p1; t += DT
            lc = float(np.clip(lam, 0.0, 1.0)); put(mode, BOTH, F=dict(fore=(Fx * lc, Fz * lc), hind=(Fx * (1 - lc), Fz * (1 - lc))), lam=float(lam))
        vx, vz, om = q["vx"], q["vz"], q["om"]
    def single(kind, q, mode):
        """One pair on the ground.  Height and pitch are planned; x follows from the moment balance (I th'' = dx Fz + h Fx).
        With a lift-off pose (world leg angle alo behind the hip, length rlo) the planned time is scaled until the hip gets there."""
        nonlocal t, x, z, th, vx, vz, om
        z0, v0, th0, om0, x0, vx0 = z, vz * 1e3, th, om, x, vx
        z1 = (C.HOOF_H + q["rlo"] * np.cos(q["alo"] / D)) - (R2(q["th"]) @ (PIV[kind][[0, 2]] - COMB[[0, 2]]))[1] - COMB[2] if "rlo" in q else z0 + q["dz"]
        def roll(sc):
            Tt = q["T"] * sc; n = max(int(round(Tt / DT)), 3); zs = []; ths = []; rows = []; f_ = q.get("f", 0.5)
            for i in range(n + 2):
                tt = i * DT; w = tt / Tt
                if w > 1: zs.append(z1 + q["vz"] * 1e3 * (tt - Tt)); ths.append(q["th"] + q["om"] * (tt - Tt)); continue
                zs.append(herm(w, z0, v0, z1, q["vz"] * 1e3, Tt) if "down" not in q else (herm(w / f_, z0, v0, z0 - q["down"], 0.0, f_ * Tt) if w < f_ else herm((w - f_) / (1 - f_), z0 - q["down"], 0.0, z1, q["vz"] * 1e3, (1 - f_) * Tt)))
                ths.append(herm(w, th0, om0, q["th"], q["om"], Tt))
            xx, vv = x0, vx0
            for i in range(1, n + 1):
                az = (zs[i + 1] - 2 * zs[i] + zs[i - 1]) / DT ** 2 * 1e-3; ath = (ths[i + 1] - 2 * ths[i] + ths[i - 1]) / DT ** 2 / D
                Fz = max(MASS * (az + G), 0.0); dx = (hoof[kind] - xx) * 1e-3; h = (COMB[2] + zs[i - 1] - C.HOOF_H) * 1e-3
                Fx = (I_P * ath - dx * Fz) / h; vv += Fx / MASS * DT; xx += vv * DT * 1e3; rows.append((xx, zs[i], ths[i], vv, Fz, Fx))
            hp_ = hip_w(kind, xx, zs[n], ths[n]); return float(np.degrees(np.arctan2(hoof[kind] - hp_[0], hp_[1] - C.HOOF_H))), rows
        sc = 1.0
        if "rlo" in q:                                     # the leg angle falls as the body passes over the hoof: first scale at which it reaches alo
            grid = np.geomspace(0.3, 3.0, 30); ang = [roll(g_)[0] for g_ in grid]; k = next((k for k in range(1, len(grid)) if ang[k] <= q["alo"] < ang[k - 1]), None)
            if k is None: flag.append(kind + " stall")
            else:
                lo, hi = grid[k - 1], grid[k]
                for _ in range(14):
                    mid = (lo + hi) / 2
                    if roll(mid)[0] > q["alo"]: lo = mid
                    else: hi = mid
                sc = (lo + hi) / 2
        for xx, zz, tt_, vv, Fz, Fx in roll(sc)[1]:
            vz = (zz - z) / DT * 1e-3; om = (tt_ - th) / DT; x, z, th, vx = xx, zz, tt_, vv; t += DT
            put(mode, dict(fore=kind == "fore", hind=kind == "hind"), F={kind: (Fx, Fz)})
        vz = q["vz"]; om = q["om"]
    def flight(kind, q, mode):
        """Ballistic until the pair's hooves reach the ground with the leg at its planned touch-down pose (leg-frame angle b, length r)."""
        nonlocal t, x, z, th, vz
        k = 0
        while True:
            t += DT; k += 1; x += vx * DT * 1e3; vz -= G * DT; z += vz * DT * 1e3; th += om * DT
            hp_ = hip_w(kind, x, z, th); a = (q["b"] + th) / D
            if (vz < 0 and hp_[1] - C.HOOF_H <= q["r"] * np.cos(a)) or k > 200:
                hoof[kind] = float(hp_[0] + (hp_[1] - C.HOOF_H) * np.tan(a)); put(mode, dict(fore=False, hind=False))
                if k > 200: flag.append("no landing")
                return
            put(mode, dict(fore=False, hind=False))
    double(P["A"], "push: both pairs"); single("hind", P["B"], "push: hinds")
    flight("fore", P["ftd"], "flight"); single("fore", P["F"], "landing: fores")
    hh_ = hip_w("hind", x, z, th); hoof["hind"] = float(hh_[0] + (hh_[1] - C.HOOF_H) * np.tan((P["hb"] + th) / D))          # the hinds come down under the body
    L = dict(P["L"]); L.update(z=0.0, vx=0.0, vz=0.0, th=0.0, om=0.0)
    L["dx"] = COMB[0] + ((hoof["fore"] - A.NEUT[0][0]) + (hoof["hind"] - A.NEUT[2][0])) / 2 + L.get("dxo", 0.0) - x          # come to rest centred over the four hooves
    double(L, "landing: both pairs")
    vx = vz = om = 0.0
    for _ in range(int(0.5 / DT)): t += DT; put("stand", BOTH)
    return out, not flag


# ------------------------------------------------------------------ pass 2: legs
def legs(sm, P, q_start=None):
    """Joint angles for every sample.  Planted: hoof fixed in the world.  In the air: leg-frame polar path through style keys."""
    n = len(sm); fr = []; bad = dict(ik=0, reach=0); info = dict(force=[], grip=[], clear=[], lam=[])
    air = {"fore": P["air"]["fore"], "hind": P["air"]["hind"]}
    Tb = [A.body_T(b_row(s["x"], s["z"], s["th"])) for s in sm]; Tbi = [np.linalg.inv(m) for m in Tb]
    def planted_pl(i, kind, side, hx, phi=0.0):
        ball = np.array([hx, side * (PIV[kind][1] + YIN), C.HOOF_H * np.cos(phi / D) + M.R_HOOF * abs(np.sin(phi / D)), 1.0]); pb = (Tbi[i] @ ball)[:3]
        return np.array([pb[0] - PIV[kind][0], side * pb[1] - PIV[kind][1], pb[2] - PIV[kind][2]])
    # air intervals per pair and their polar splines (leg-frame angle b deg, length r mm)
    plan = {}
    for kind in ("fore", "hind"):
        st = [s["st"][kind] for s in sm]; i = 0; plan[kind] = [None] * n
        while i < n:
            if st[i]: i += 1; continue
            j = i
            while j < n and not st[j]: j += 1
            p0 = planted_pl(i - 1, kind, 1, sm[i - 1]["hoof"][kind]); p1 = planted_pl(min(j, n - 1), kind, 1, sm[min(j, n - 1)]["hoof"][kind])
            pol = lambda p: (np.degrees(np.arctan2(p[0], -p[2])), float(np.hypot(p[0], p[2])))
            ks = [(0.0,) + pol(p0)] + [(f_, b_, r_ * A.RR[kind]) for f_, b_, r_ in air[kind]] + [(1.0,) + pol(p1)]
            # the first long flight of a hind interval / last of a fore one is where the style keys matter; short intervals keep only the ends
            if j - i < 12: ks = [ks[0], ks[-1]]
            tt = np.array([k_[0] for k_ in ks]); sb = CubicSpline(tt, [k_[1] for k_ in ks], bc_type="clamped"); sr = CubicSpline(tt, [k_[2] for k_ in ks], bc_type="clamped")
            for k in range(i, j):
                w = (k - i + 1) / (j - i + 1); plan[kind][k] = (float(sb(w)), float(min(sr(w), 0.985 * A.RR[kind])))
            i = j
    q = list(q_start) if q_start else [(C.STAND[A.KINDS[i]]["q0"], C.STAND[A.KINDS[i]]["q1"]) for i in range(4)]
    for i, s in enumerate(sm):
        row = []
        for li in range(4):
            kind, side = A.KINDS[li], A.SIDES[li]; phi = 0.0; res = None; hpm = M.HP_MAX
            for _ in range(3):
                if s["st"][kind]: pl = planted_pl(i, kind, side, s["hoof"][kind], phi)
                else:
                    b_, r_ = plan[kind][i]; v = r_ * u(b_ / D); pl = np.array([v[0], YIN, v[1]])
                res = A.ik3(kind, pl, 0.0, q[li])
                if res is not None and not (25.0 < res[2] < 172.0): res = A.ik3(kind, pl, 0.0, (C.STAND[kind]["q0"], C.STAND[kind]["q1"]))
                if res is None: break
                hpm = np.sqrt(max(M.HP_MAX ** 2 - res[0] ** 2, 1.0)); hh = res[1] + C.LEG[kind]["delta"] + s["th"]
                phi = 0.0 if (abs(hh) <= hpm or not s["st"][kind]) else np.sign(hh) * hpm - hh
            if res is None or res[3] > 1e-3: bad["ik"] += 1; res = res or (0.0, q[li][0], q[li][1], 9.0)
            abd, q0, q1, err = res
            if not s["st"][kind]:
                rg = V.RANGE[kind]; q0 = float(np.clip(q0, rg["q0"][0] + 0.5, rg["q0"][1] - 0.5)); q1 = float(np.clip(q1, rg["q1"][0] + 0.5, rg["q1"][1] - 0.5))
            q[li] = (q0, q1); hh = q0 + C.LEG[kind]["delta"] + s["th"]
            if s["st"][kind]: hoof = [s["hoof"][kind], side * (PIV[kind][1] + YIN), 0.0]
            else:
                bl = M.ball_xz(kind, q0, q1); pw = Tb[i] @ np.array([bl[0] + PIV[kind][0], side * (YIN + PIV[kind][1]), bl[1] + PIV[kind][2], 1.0]); hoof = [float(pw[0]), float(pw[1]), float(pw[2] - C.HOOF_H)]
                info["clear"].append(hoof[2])
            row.append(dict(q0=float(q0), q1=float(q1), fold=0.0, st=bool(s["st"][kind]), hp=float(np.clip(hh, -hpm, hpm)), yaw=0.0, abd=float(abd), hr=float(-abd), hoof=hoof, roll=float(phi)))
        # leg force against peak motor torque (single-pair stances): tau = J^T f, f per leg in the leg frame
        for kind, (Fx_, Fz_) in s.get("F", {}).items():
            li = 0 if kind == "fore" else 2; q0, q1 = row[li]["q0"], row[li]["q1"]; e_ = 0.05
            J = np.column_stack([(M.ball_xz(kind, q0 + e_, q1) - M.ball_xz(kind, q0 - e_, q1)) / (2 * e_ / D), (M.ball_xz(kind, q0, q1 + e_) - M.ball_xz(kind, q0, q1 - e_)) / (2 * e_ / D)]) * 1e-3
            f = R2(s["th"]).T @ np.array([Fx_, Fz_]) / 2; tau = J.T @ f
            info["force"].append((s["t"], kind, float(max(abs(tau[0]) / TQ[kind][0], abs(tau[1]) / TQ[kind][1])), float(Fz_), float(Fx_), float(abs(tau[0])), float(abs(tau[1]))))
            if Fz_ > 50: info["grip"].append(abs(Fx_) / Fz_)
        if "lam" in s: info["lam"].append(s["lam"])
        fr.append(dict(t=s["t"], sig=s["x"] * 1e-3, v=float(s["vx"]), mode=s["mode"], leap=0, bound=-1, b=b_row(s["x"], s["z"], s["th"]), l=row))
    return dict(frames=fr, dt=DT, bad=bad, label="extended leap", time=sm[-1]["t"]), info


def quick(r):
    """Joint ranges, speed margins and hoof tilt straight from the frames (what agility.analyze reports, without the clearances)."""
    fr = r["frames"]; lim = []; spd = {}; tilt = 0.0
    for li in (0, 2):
        kind = A.KINDS[li]
        for j in ("q0", "q1", "abd"):
            v = np.array([f["l"][li][j] for f in fr]); lo, hi = V.RANGE[kind][j]
            if v.min() < lo - 0.05 or v.max() > hi + 0.05: lim.append(kind + " " + j)
            spd[kind + " " + j] = float(A.SPD[kind][j] / max(np.abs(np.diff(v)).max() / DT, 1e-6))
        tilt = max(tilt, max(float(np.hypot(f["l"][li]["hp"], f["l"][li]["hr"])) for f in fr))
    return dict(ik_fail=r["bad"]["ik"], limit_violations=lim, speed_margin_min=spd, hoof_tilt_max_deg=tilt)


def evaluate(P, full=False, start=None):
    sm, okf = simulate(P, start); r, info = legs(sm, P, start.get("q") if start else None); a = A.analyze(r, dist=True, nmax=10 ** 6) if full else quick(r)
    fl = {}; cur = None
    for s in sm:                                           # flights: start / end, distance, height
        if s["mode"] == "flight":
            if cur is None: cur = dict(mode=s["mode"], t0=s["t"] - DT, x0=s["x"] - s["vx"] * DT * 1e3, z0=s["z"], zmax=s["z"], vx=s["vx"], hh0=s["hoof"]["hind"])
            cur["t1"], cur["x1"], cur["hf1"] = s["t"], s["x"], s["hoof"]["fore"]; cur["zmax"] = max(cur["zmax"], s["z"])
    ext = [cur] if cur else []; bigf = cur
    sm_ = a["speed_margin_min"]; force = max((f[2] for f in info["force"]), default=0.0)
    m = dict(flights_ok=bool(okf), ik_fail=a["ik_fail"], limits=a["limit_violations"], speed_margin=min(sm_.values()) if sm_ else 99.0, speed_joint=min(sm_, key=sm_.get) if sm_ else "",
             force_ratio=force, force_at=max(info["force"], key=lambda f: f[2])[:2] if info["force"] else None, grip=max(info["grip"], default=0.0), air_clear_mm=min(info["clear"], default=99.0),
             leap_mm=(bigf["x1"] - bigf["x0"]) if bigf else 0.0, leap_s=(bigf["t1"] - bigf["t0"]) if bigf else 0.0, leap_vx=bigf["vx"] if bigf else 0.0, leap_rise_mm=(bigf["zmax"] - bigf["z0"]) if bigf else 0.0, mark_mm=(bigf["hf1"] - bigf["hh0"]) if bigf else 0.0, lam_out=max([max(-l_, l_ - 1.0, 0.0) for l_ in info["lam"]], default=0.0),
             top_speed=max(s["vx"] for s in sm), time_s=sm[-1]["t"], distance_m=(sm[-1]["x"] - sm[0]["x"]) * 1e-3,
             pitch_range=(min(s["th"] for s in sm), max(s["th"] for s in sm)), tilt=a["hoof_tilt_max_deg"])
    return m, r, a, sm, info


def cost(m):
    """0 when every rule is met; the search minimises this, then maximises the leap."""
    c = 50.0 * (not m["flights_ok"]) + 5.0 * m["ik_fail"] + 20.0 * len(m["limits"]) + max(0.0, 1.5 - m["speed_margin"]) * 10 + max(0.0, m["force_ratio"] - 1 / 1.1) * 10
    c += max(0.0, m["grip"] - 0.7) * 10 + max(0.0, 15.0 - m["air_clear_mm"]) * 0.2 + max(0.0, m["tilt"] - 15.01) + m["lam_out"] * 10
    return c


# ------------------------------------------------------------------ learning the leap: random search over the plan's numbers
BOUNDS = {("crouch", "dx"): (-110, -20), ("crouch", "dz"): (-110, -40), ("crouch", "th"): (0, 8),
          ("A", "T"): (0.15, 0.45), ("A", "down"): (5, 90), ("A", "f"): (0.2, 0.6), ("A", "dx"): (40, 420), ("A", "vx"): (0.4, 2.6), ("A", "dz"): (5, 80), ("A", "vz"): (0.1, 1.0), ("A", "th"): (3, 12), ("A", "om"): (-20, 40),
          ("B", "T"): (0.08, 0.35), ("B", "vz"): (0.5, 1.7), ("B", "th"): (3, 14), ("B", "om"): (-90, -5), ("B", "rlo"): (340, 379), ("B", "alo"): (-37, -16),
          ("ftd", "b"): (-2, 24), ("ftd", "r"): (395, 468),
          ("F", "T"): (0.08, 0.32), ("F", "down"): (5, 95), ("F", "dz"): (-80, 15), ("F", "vz"): (-0.4, 0.4), ("F", "th"): (-9, 5), ("F", "om"): (-30, 70), ("F", "f"): (0.3, 0.75),
          ("hb",): (-8, 26), ("L", "T"): (0.35, 0.9), ("L", "down"): (0, 70), ("L", "f"): (0.2, 0.6), ("L", "dxo"): (-70, 70),
          ("air", "fore", 0, 0): (0.2, 0.45), ("air", "fore", 0, 1): (-12, 15), ("air", "fore", 0, 2): (0.55, 0.8),
          ("air", "fore", 1, 0): (0.6, 0.9), ("air", "fore", 1, 1): (28, 36), ("air", "fore", 1, 2): (0.95, 0.985),
          ("air", "hind", 0, 0): (0.3, 0.6), ("air", "hind", 0, 1): (-38, -30), ("air", "hind", 0, 2): (0.95, 0.985)}


def approach(vs=1.3, off=0.24):
    """Run-up from agility.py: gallop up to vs, lift-off stride, short gather flight, all four hooves down together.
    -> (frames up to that touch-down, start state for simulate)"""
    sq = A.sprint_seq(vs, 2.0, 0.8 * A.VZ); r = A.run(A.Path([("line", 3.0 + off), ("line", 4.5)]), vcap=vs, leaps=[(3.0 + off, sq, True)]); fr = r["frames"]
    i1 = next(i for i, f in enumerate(fr) if f["mode"] == "push" and f["bound"] == 1); f0, f1 = fr[i1 - 1], fr[i1]
    for f in fr[:i1]: f["mode"] = {"gait": "run-up", "push": "run-up: lift-off stride", "flight": "run-up: gather flight"}.get(f["mode"], f["mode"])
    st = dict(t=f0["t"], x=f0["b"][0] + COMB[0], z=f0["b"][2], th=0.0, vx=vs, vz=(f0["b"][2] - fr[i1 - 2]["b"][2]) / DT * 1e-3, om=0.0, hoof=dict(fore=f1["l"][0]["hoof"][0], hind=f1["l"][2]["hoof"][0]))
    return fr[:i1], st, r["bad"]["ik"]


def get(P, k):
    o = P
    for a_ in k: o = o[a_]
    return o


def put_(P, k, v):
    o = P
    for a_ in k[:-1]: o = o[a_]
    if isinstance(o, tuple): raise TypeError
    o[k[-1]] = v


def clone(P):
    Q = json.loads(json.dumps(P)); Q["air"] = {k: [list(x) for x in v] for k, v in Q["air"].items()}; return Q


def search(P0, n, seed=0, start=None, objective="leap_mm"):
    """Keep a change when it lowers the rule violations; once every rule is met, when it lengthens the leap."""
    rng = np.random.default_rng(seed); P = clone(P0); P.setdefault("L", {}).setdefault("dxo", 0.0); m = evaluate(P, start=start)[0]; best = (cost(m), -m[objective]); keys = [k for k in BOUNDS if k[0] != "crouch" or start is None]
    keys = [k for k in keys if not (k[0] == "A" and k[1] in ("down", "f") and "down" not in P["A"])]; hist = [(0, best[0], m[objective])]
    for it in range(1, n + 1):
        Q = clone(P); sig = 0.25 if it < n * 0.3 else 0.1 if it < n * 0.7 else 0.04
        for k in [keys[i] for i in rng.choice(len(keys), size=rng.integers(1, 5), replace=False)]:
            lo, hi = BOUNDS[k]; put_(Q, k, float(np.clip(get(Q, k) + rng.normal() * sig * (hi - lo), lo, hi)))
        try: mq = evaluate(Q, start=start)[0]
        except Exception: continue
        c = (cost(mq), -mq[objective])
        if c < best: P, m, best = Q, mq, c; hist.append((it, c[0], mq[objective]))
    return P, m, hist


def build(P, with_runup=True):
    """Full run with every check.  -> (frames, numbers)"""
    pre, st, ik0 = approach() if with_runup else ([], None, 0)
    if st: st["q"] = [(l["q0"], l["q1"]) for l in pre[-1]["l"]]
    m, r, a, sm, info = evaluate(P, full=True, start=st); fr = r["frames"]; pol = {}
    for kind, li in (("fore", 0), ("hind", 2)):
        best = None
        for f, s in zip(fr, sm):
            if s["mode"] != "flight": continue
            p = M.ball_xz(kind, f["l"][li]["q0"], f["l"][li]["q1"]); b_ = float(np.degrees(np.arctan2(p[0], -p[1]))); r_ = float(np.hypot(*p)) / A.RR[kind]; w_ = b_ + s["th"]
            if best is None or (w_ > best[0] if kind == "fore" else w_ < best[0]): best = (w_, b_, r_, s["t"])
        pol[kind] = dict(world_angle_deg=best[0], leg_angle_deg=best[1], length_of_reach=best[2], t=best[3])
    ph = {}
    for s in sm: ph.setdefault(s["mode"], [s["t"], s["t"]])[1] = s["t"]
    pk = {}
    for t_, kind, ratio, Fz, Fx, t0_, t1_ in info["force"]:
        mode = next(s["mode"] for s in sm if abs(s["t"] - t_) < 1e-6); k = mode + " / " + kind
        if k not in pk or ratio > pk[k]["ratio"]: pk[k] = dict(ratio=ratio, Fz_N=Fz, Fx_N=Fx, pivot_Nm=t0_, flex_Nm=t1_)
    fl = [s for s in sm if s["mode"] == "flight"]; ftd = fl[-1]; fo = [s for s in sm if s["mode"] == "landing: fores"]; lb = [s for s in sm if s["mode"] == "landing: both pairs"]
    num = dict(m, speed_margins=a["speed_margin_min"], distance=a.get("distance"), extension=pol, phases_s={k: [round(v[0], 3), round(v[1], 3)] for k, v in ph.items()}, peak_force=pk,
               takeoff=dict(vx=fl[0]["vx"], vz=fl[0]["vz"] + G * DT, pitch_deg=fl[0]["th"], pitch_rate_deg_s=(fl[-1]["th"] - fl[0]["th"]) / max(fl[-1]["t"] - fl[0]["t"], 1e-6)),
               landing=dict(sink_m_s=-ftd["vz"], pitch_deg=ftd["th"], speed_in=ftd["vx"], fores_alone_s=len(fo) * DT, speed_when_hinds_land=fo[-1]["vx"], lowest_mm=min(s["z"] for s in fo + lb),
                            stop_distance_mm=lb[-1]["x"] - ftd["x"], stop_time_s=lb[-1]["t"] - ftd["t"], fore_share_range=(min(info["lam"][-len(lb):]), max(info["lam"][-len(lb):]))),
               model=dict(com_body_mm=COMB.tolist(), pitch_inertia_kg_m2=I_P, mass_kg=MASS, peak_torque_Nm=TQ, grip_limit=0.7), plan=P, runup=bool(with_runup))
    return pre + fr, num, a


if __name__ == "__main__":
    import pickle
    num = lambda o: float(o) if isinstance(o, (np.floating, np.integer)) else (o.tolist() if isinstance(o, np.ndarray) else str(o))
    P = pickle.load(open(os.path.join(A.HERE, "ext_bestP_run.pkl"), "rb")); fr, res, a = build(P); Ps = pickle.load(open(os.path.join(A.HERE, "ext_bestP_stand.pkl"), "rb")); ms = evaluate(Ps)[0]
    res["standing_version"] = dict(leap_mm=ms["leap_mm"], mark_mm=ms["mark_mm"], leap_vx=ms["leap_vx"], cost=cost(ms), plan=Ps)
    json.dump(res, open(os.path.join(A.HERE, "extended_leap.json"), "w"), indent=1, default=num)
    print({k: (round(float(v), 2) if isinstance(v, (float, np.floating)) else v) for k, v in res.items() if k in ("flights_ok", "ik_fail", "limits", "speed_margin", "force_ratio", "grip", "air_clear_mm", "leap_mm", "leap_s", "leap_vx", "leap_rise_mm", "mark_mm", "lam_out", "tilt")}, "cost", cost(res))
    print("distance", res["distance"]); print("extension", res["extension"]); print("takeoff", res["takeoff"]); print("landing", res["landing"]); print("phases", res["phases_s"]); print("forces", json.dumps(res["peak_force"], default=num)[:900]); print("standing", {k: v for k, v in res["standing_version"].items() if k != "plan"})
    p = os.path.join(A.HERE, "viewer_agility", "agility_data.js"); pre = "window.AGI = "; d = json.loads(open(p).read()[len(pre):-1])
    pkf = lambda f: dict(b=[round(v_, 1) for v_ in f["b"][:3]] + [round(f["b"][3], 2), round(f["b"][4], 2)],
                         l=[[round(l["q0"], 2), round(l["q1"], 2), 0, int(l["st"]), round(l["hp"], 2), round(l["yaw"], 2), round(l["abd"], 2), round(l["hr"], 2)] for l in f["l"]])
    d["motions"]["extleap"] = dict(dt=2 * DT, frames=[pkf(f) for f in fr[::2]]); d["cap"]["ext_leap"] = json.loads(json.dumps({k: v for k, v in res.items() if k not in ("plan", "distance", "speed_margins")}, default=num))
    open(p, "w").write(pre + json.dumps(d, separators=(",", ":")) + ";")
    with open(os.path.join(A.HERE, "agility_extended_leap.csv"), "w") as fc:
        fc.write(",".join(["t_s", "x_mm", "y_mm", "z_mm", "heading_deg", "pitch_deg", "speed_m_s", "phase"] + ["%s_%s" % (n, j) for n in ("FL", "FR", "HL", "HR") for j in ("swing_deg", "twist_deg", "pivot_deg", "flex_deg", "hoof_pitch_deg", "stance")]) + "\n")
        for f in fr:
            row = ["%.4f" % f["t"], "%.1f" % f["b"][0], "%.1f" % f["b"][1], "%.1f" % f["b"][2], "%.2f" % f["b"][3], "%.2f" % f["b"][4], "%.3f" % f["v"], f["mode"]]
            for l in f["l"]: row += ["%.2f" % l["abd"], "%.2f" % l["yaw"], "%.2f" % l["q0"], "%.2f" % l["q1"], "%.2f" % l["hp"], "%d" % l["st"]]
            fc.write(",".join(row) + "\n")
    i0 = next(i for i, f in enumerate(fr) if f["mode"] == "flight"); print("flight frames (viewer index)", i0 // 2, "of", len(fr) // 2)
