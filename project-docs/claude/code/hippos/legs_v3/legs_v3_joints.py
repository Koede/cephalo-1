#!/usr/bin/env python3
"""Hippos-1 legs v3, concept B (Forest's pick 2026-11-18): joint-by-joint implementation study.
For every joint: range over all seven motions, loads over every case, speed; then each way of building it, sized.
Joints.  Fore: scapula pivot (body, RS06), shoulder (scapula-humerus, driven), elbow (humerus-column, slaved parallel to
the scapula), carpus (in the column: locked in stance, folded in swing), fetlock (sprung pastern), abduction.
Hind: hip pivot (RS06), stifle (driven), hock (slaved parallel to the femur), fetlock, abduction.
    python3 legs_v3_joints.py -> joints_v3.json + tables"""
import os, json, importlib.util, io, contextlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("S", os.path.join(HERE, "legs_v3_study.py")); S = importlib.util.module_from_spec(spec)
with contextlib.redirect_stdout(io.StringIO()):
    spec.loader.exec_module(S)
    RS = S.study()
D = 180 / np.pi; G = 9.81
KEY = "B"; O = S.OPT[KEY]; RB = RS[KEY]
ACT = S.ACT
W = S.M_TOTAL[KEY] * G
u = S.u


def cross(a, b):
    return a[0] * b[1] - a[1] * b[0]


# ====================================================================== 1. loads at every joint, per case
def poses_and_forces(leg):
    """(case, foot x, foot z, Fx, Fz) samples in the leg frame: stance of every gait (half-sine, aimed at the CoM),
    push, jump stroke, landing, get up, rear-up."""
    out = []
    for gname in S.GAITS:
        p = S.foot_path(KEY, leg, gname); g = S.GAITS[gname]
        for i in range(0, p["n_st"], 3):
            fz = (np.pi / 2) * S.share(leg) / 2 * W / g["duty"] * np.sin(np.pi * p["s"][i])
            fx = -fz * (p["x"][i] - O["x0"][leg]) / S.Z_COM
            out.append((gname, p["x"][i], p["z"][i], fx, fz))
            if gname == "trot":
                out.append(("trot + 0.25 g push", p["x"][i], p["z"][i], fx + S.PUSH_G * fz, fz))
    m = O["legs"][leg]; h = O["pivot"][leg][1]
    jt, ji = S.jump_case(KEY, leg)
    zc = max(0.70 * h, S.h_min(m) + 0.02); z_top = 0.97 * m.lmax
    for z in np.linspace(zc, z_top, 8):
        out.append(("jump 100 mm", O["x0"][leg], -z, 0.0, ji["F"]))
    lt, li = S.landing_case(KEY, leg)
    out.append(("land from 250 mm", O["x0"][leg], -(h - S.STOP / 2), 0.0, li["F"]))
    if leg == "rear":
        _, ri = S.rear_case(KEY)
        th = S.REAR_DEG / D; Rb = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
        for bal in (-0.2, 0.0, 0.2):
            Fb = Rb.T @ (np.array([bal, 1.0]) * W / 2)
            out.append((f"rear up {S.REAR_DEG:.0f} deg", ri["foot_body"][0], ri["foot_body"][1], Fb[0], Fb[1]))
    return out


R_HORN = 0.030                   # horn radius of the slaving rod / drum radius of the slaving cables (est., to pick)


def joint_loads(leg):
    m = O["legs"][leg]; rows = []
    for case, x, z, fx, fz in poses_and_forces(leg):
        q = m.ik(x, z); P = m.pts(q); F = np.array([fx, fz])
        foot = P[-1]
        t_piv = -cross(foot - P[0], F)            # moment the ground load puts about each joint (sign: + = turns distal end forward)
        t_mid = -cross(foot - P[1], F)
        t_low = -cross(foot - P[2], F)
        r = dict(case=case, x=x, z=z, F=F, q=q, P=P, M_pivot=t_piv, M_upper=t_mid, M_lower=t_low, Fmag=np.hypot(fx, fz))
        if leg == "front":
            col = P[3] - P[2]; c = P[2] + S.CARPUS * col
            r["M_carpus"] = -cross(foot - c, F)
            r["carpus_lever"] = cross(col / np.linalg.norm(col), F / np.linalg.norm(F)) * np.linalg.norm(foot - c)
        rows.append(r)
    return rows


# ====================================================================== 2. where the driven-joint motor sits
def motor_torques(leg):
    """Actuated generalised torques (joint side) from the study's cases, then the two motor placements:
    'on the upper segment' (relative drive): pivot motor = tau_q0, knee motor = tau_q1 (as the v3 study);
    'on the body' (absolute drive of the middle segment): pivot motor = tau_q0 + tau_q1, knee motor = -tau_q1."""
    out = {}
    inr = S.inertia(KEY, leg, RB["acts"]["hip"][leg], RB["acts"]["knee"][leg])
    st, _ = S.static(KEY, leg, O["x0"][leg], -O["pivot"][leg][1], (0, S.share(leg) / 2 * W))
    sp = np.array([0.0, -S.OFFLOAD * st[1]])
    for gname in list(S.GAITS) + ["push"]:
        g = S.gait_cycle(KEY, leg, "trot" if gname == "push" else gname, inr, extra_fx=S.PUSH_G if gname == "push" else 0.0, spring=sp)
        t = g["tau"]
        rel = (np.sqrt(np.mean(t ** 2, 0)), np.abs(t).max(0))
        ab = np.c_[t[:, 0] + t[:, 1], -t[:, 1]]
        absd = (np.sqrt(np.mean(ab ** 2, 0)), np.abs(ab).max(0))
        out["trot + 0.25 g push" if gname == "push" else gname] = dict(rel=rel, abs=absd)
    for case, fn in (("jump 100 mm", lambda: S.jump_case(KEY, leg)), ("land from 250 mm", lambda: S.landing_case(KEY, leg))):
        h = O["pivot"][leg][1]; m = O["legs"][leg]
        if case.startswith("jump"):
            _, info = fn(); zs = np.linspace(max(0.70 * h, S.h_min(m) + 0.02), 0.97 * m.lmax, 25); F = info["F"]
        else:
            _, info = fn(); zs = [h - S.STOP / 2]; F = info["F"]
        T = np.array([S.static(KEY, leg, O["x0"][leg], -z, (0, F))[0] for z in zs]) + sp * 0 - np.array([0, sp[1]]) * 0
        T = T + np.array([0.0, sp[1]])        # spring helps the support load
        rel = (np.zeros(2), np.abs(T).max(0)); ab = np.c_[T[:, 0] + T[:, 1], -T[:, 1]]
        out[case] = dict(rel=rel, abs=(np.zeros(2), np.abs(ab).max(0)))
    return out


# ====================================================================== 3. drive from the pivot to the driven joint
BELT_PITCH = 0.005
GT_WORK = 28.0e3                 # 5M GT working tension, N per m width (Gates, < 300 rpm)
TENS_15 = 4786.0                 # 15 mm 5M tensile rating, N
DYNEEMA = {2.0: 3500.0, 3.0: 7000.0}     # SK78 braid break load, N (est. from maker tables)


def pd(z):
    return z * BELT_PITCH / np.pi


def drive_options(leg, tau_peak, tau_rms, ratio, rng_deg, span):
    """tau at the driven joint (motor side x ratio), the range it must sweep, and the span (upper segment length)."""
    o = {}
    # (a) HTD/GT 5M belt, 25 mm, pulleys sized so the driven pulley stays inside a 100 mm housing
    for z2 in (36, 48, 54):
        z1 = int(round(z2 / ratio))
        T = tau_peak / (pd(z2) / 2)
        o[f"belt 5M 25 mm, {z1}T:{z2}T"] = dict(kind="belt", T_peak=T, T_rms=tau_rms / (pd(z2) / 2), work=GT_WORK * 0.025,
                                               break_=TENS_15 / 0.015 * 0.025, pd2=pd(z2), pd1=pd(z1),
                                               ok=T <= 1.6 * GT_WORK * 0.025, range_ok=True, mass=0.045 + 0.02 * (z2 / 48) ** 2)
    # (b) push-rod crank (parallelogram), only for 1:1: rod force over the range, transmission angle at the ends
    if abs(ratio - 1.0) < 1e-6:
        mu_end = 90 - rng_deg / 2
        for r_h in (0.030, 0.040):
            Fr = tau_peak / (r_h * np.sin(np.radians(60)))          # peak torque occurs mid-gait: transmission >= 60 deg there (est.)
            Iy = np.pi * (0.010 ** 4 - 0.008 ** 4) / 64
            o[f"push rod, crank {r_h * 1e3:.0f} mm"] = dict(kind="rod", F_peak=Fr, mu_end=mu_end, buckling=np.pi ** 2 * 100e9 * Iy / span ** 2,
                                                             ok=mu_end >= 25, range_ok=mu_end >= 25, mass=0.035)
    else:
        o["push rod, crank"] = dict(kind="rod", ok=False, range_ok=False, note=f"{ratio:.1f}:1 needs the crank to sweep {rng_deg * ratio:.0f} deg: past the dead points of a four-bar")
    # (c) Dyneema capstan: closed loop on two drums, wraps as needed (no range limit)
    for d_line in (2.0, 3.0):
        r2 = 0.030; T = tau_peak / r2
        o[f"Dyneema {d_line:.0f} mm capstan, drums {r2 / ratio * 1e3:.0f} / {r2 * 1e3:.0f} mm"] = dict(kind="cable", T_peak=T, break_=DYNEEMA[d_line], sf=DYNEEMA[d_line] / T,
                                                                                                    ok=DYNEEMA[d_line] / T >= 4, range_ok=True, mass=0.030)
    return o


# ====================================================================== 4. slaving link for the elbow / hock
def slave_options(rows, rng_deg, span):
    M = max(abs(r["M_lower"]) for r in rows)
    M_land = max(abs(r["M_lower"]) for r in rows if r["case"].startswith("land"))
    o = {}
    mu_end = 90 - rng_deg / 2
    for nrod in (1, 2):
        F = M / (R_HORN * np.sin(np.radians(60)))
        Iy = np.pi * (0.008 ** 4 - 0.006 ** 4) / 64
        o[f"{nrod} rigid push rod{'s' if nrod > 1 else ''} 8 x 6 carbon, horns {R_HORN * 1e3:.0f} mm"] = dict(F_each=F / nrod, buckling=np.pi ** 2 * 100e9 * Iy / span ** 2,
                                                                                                         mu_end=mu_end, ok=mu_end >= 25)
    o["spring rod (preloaded cartridge in the rod)"] = dict(F_each=M / (R_HORN * np.sin(np.radians(60))), preload=1.1 * M / (R_HORN * np.sin(np.radians(60))) * 0.6,
                                                             note="gives 25 mm of hoof travel above 60 % of the gallop peak (est.)", mu_end=mu_end, ok=mu_end >= 25)
    T = M / R_HORN
    o["crossed Dyneema tendons on drums (reciprocal apparatus)"] = dict(T=T, sf2=DYNEEMA[2.0] / T, sf3=DYNEEMA[3.0] / T, mu_end=None, ok=DYNEEMA[3.0] / T >= 4,
                                                                         note="no dead point, constant 1:1, tension only (two lines)")
    return M, M_land, o


# ====================================================================== 5. carpus
def carpus(rows):
    st = [r for r in rows if r["case"] in ("walk", "trot", "fast trot", "gallop", "land from 250 mm", "jump 100 mm", "trot + 0.25 g push")]
    M = np.array([r["M_carpus"] for r in st])
    F = np.array([r["Fmag"] for r in st])
    return dict(M_max=float(M.max()), M_min=float(M.min()), F_max=float(F.max()),
                frac_flexing=float(np.mean(M < 0)))


def run():
    J = {}
    for leg in ("front", "rear"):
        rows = joint_loads(leg)
        mt = motor_torques(leg)
        name_up, name_lo = ("shoulder", "elbow") if leg == "front" else ("stifle", "hock")
        ratio = RB["acts"]["ratio"][leg]
        span = O["legs"][leg].l1; span_mid = O["legs"][leg].l2
        pk = max(v["rel"][1][1] for v in mt.values()); rms = max(mt[g]["rel"][0][1] for g in ("walk", "trot"))
        rng = 122.0 if leg == "rear" else 118.0
        drives = drive_options(leg, pk, rms, ratio, rng, span)
        M_lo, M_lo_land, slaves = slave_options(rows, rng, span_mid)
        Fmax = max(r["Fmag"] for r in rows)
        J[leg] = dict(names=(name_up, name_lo), motor=mt, drives=drives, slave=dict(M=M_lo, M_land=M_lo_land, options=slaves),
                      F_max=Fmax, ratio=ratio, tau_peak=pk, tau_rms=rms,
                      bearings=dict(pivot=Fmax, upper=Fmax + (pk / 0.0286 if ratio > 1 else pk / 0.0318), lower=Fmax + M_lo / R_HORN))
        if leg == "front":
            J[leg]["carpus"] = carpus(rows)
    return J


def report(J):
    p = print
    for leg in ("front", "rear"):
        j = J[leg]; up, lo = j["names"]
        p(f"\n===== {leg} leg ({up} driven, {lo} slaved), ratio {j['ratio']:.1f}:1, ground load up to {j['F_max']:.0f} N")
        p("  motor torques, joint side (rms / peak), RS06 rated 11 / peak 36 at the motor:")
        for c, v in j["motor"].items():
            (r_rel, p_rel), (r_abs, p_abs) = v["rel"], v["abs"]
            p(f"    {c:20s} on the upper segment: pivot {r_rel[0]:5.1f}/{p_rel[0]:5.1f} {up} {r_rel[1]:5.1f}/{p_rel[1]:5.1f}"
              f"   | on the body: pivot {r_abs[0]:5.1f}/{p_abs[0]:5.1f} {up} {r_abs[1]:5.1f}/{p_abs[1]:5.1f}")
        p(f"  drive {up}: peak {j['tau_peak']:.1f} N m at the joint, rms {j['tau_rms']:.1f}")
        for k, v in j["drives"].items():
            p(f"    {k:42s} " + ", ".join(f"{a} {b:.0f}" if isinstance(b, (int, float, np.floating)) and not isinstance(b, bool) else f"{a} {b}" for a, b in v.items() if a not in ("kind",)))
        s = j["slave"]
        p(f"  {lo}: moment up to {s['M']:.1f} N m (landing {s['M_land']:.1f})")
        for k, v in s["options"].items():
            p(f"    {k:58s} " + ", ".join(f"{a} {b:.0f}" if isinstance(b, (int, float, np.floating)) and not isinstance(b, bool) and b is not None else f"{a} {b}" for a, b in v.items()))
        p(f"  bearing loads (N): " + ", ".join(f"{k} {v:.0f}" for k, v in j["bearings"].items()))
        if "carpus" in j:
            c = j["carpus"]; p(f"  carpus moment in stance {c['M_min']:.1f} .. {c['M_max']:.1f} N m (negative = would fold), share of stance samples folding {c['frac_flexing'] * 100:.0f} %, column load up to {c['F_max']:.0f} N")


if __name__ == "__main__":
    J = run(); report(J)
    json.dump(J, open(os.path.join(HERE, "joints_v3.json"), "w"), default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o), indent=1)
