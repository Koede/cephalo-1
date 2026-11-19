"""Three-phalanx hand-prop: thrust and power of the finger blades (2026-11-13).  ESTIMATE: blade-element momentum theory
(hover, Prandtl tip loss) with section polars from NeuralFoil (an XFOIL surrogate) and, for the bluff as-drawn section,
Hoerner's handbook drag.  No CFD, no spin test.  python3 hand3_thrust.py -> hand3_thrust.json, hand3_thrust.png.

Blade plans compared (all four fingers, R 127 mm, lifting span from r 31.2 mm):
  A  the first blueprint finger: rounded slab 4.5 -> 3.2 mm thick on an 18 -> 12 mm chord, 2 mm pad standing proud on the
     thrust face (36-43 % thick, blunt trailing edge)
  B  same chord, shaped: airfoil nose and tapered tail, pad 1 mm and flush (5.5 -> 4.2 mm total, 31-35 % thick)
  C  trailing fairing: a thin printed lip behind each phalanx makes the chord 30 -> 24 mm; pad 2 mm, flush (22 % thick)
  D  thin airfoil finger (hand3_design.py, picked by hand3_blade_opt.py): 22 -> 18 mm chord, 3.4 -> 2.4 mm printed + 1 mm
     flush pad (19-20 % thick), 3 % camber, pitch 12 deg, knuckle hardware faired
Pitch is set by the root grip (as hand-prop.md, 12 deg there); "+twist" = each phalanx printed at its own incidence to its
hinge line (the hinge pins stay parallel, so the curl is unchanged).
"""
import json, sys
import numpy as np
import hand3_design as H

RHO, NU, A_SND = 1.2, 1.5e-5, 340.0
R, R0, NB = H.R_TIP * 1e-3, (H.KN[0] + 1.2) * 1e-3, 4
ALPHAS = np.arange(-8.0, 30.01, 1.0)
R_SEC = np.linspace(R0, R, 7)

# ------------------------------------------------------------------ section shapes (unit chord, Selig order)
def naca4(m, p, t, n=100):
    b = np.linspace(0, np.pi, n); x = 0.5 * (1 - np.cos(b))
    yt = 5 * t * (0.2969 * np.sqrt(x) - 0.1260 * x - 0.3516 * x ** 2 + 0.2843 * x ** 3 - 0.1036 * x ** 4)
    yc = np.where(x < p, m / p ** 2 * (2 * p * x - x ** 2), m / (1 - p) ** 2 * ((1 - 2 * p) + 2 * p * x - x ** 2)) if m > 0 else 0 * x
    up = np.c_[x, yc + yt][::-1]; lo = np.c_[x, yc - yt][1:]
    return np.vstack([up, lo])


def slab_pad(t_c, pad_c, n=120):
    """Plan A: superellipse slab (t_c thick) with a flat pad pad_c proud over the middle 60 % of the upper face."""
    b = np.linspace(0.02, np.pi - 0.02, n); x = 0.5 * (1 + np.cos(b))                # TE -> LE
    y = 0.5 * t_c * np.sin(b) ** 0.6
    ramp = np.clip((0.3 - np.abs(x - 0.5)) / 0.03, 0, 1)
    up = np.c_[x, y + ramp * (pad_c + 0.5 * t_c - y - 0.008)]
    up[:, 1] = np.maximum(up[:, 1], y)
    lo = np.c_[x[::-1], -y[::-1]]
    return np.vstack([[1.0, 0.0], up, [0.0, 0.0], lo, [1.0, 0.0]])


# plans A-C are the first finger and two ways to shape it (fixed numbers); plan D is the finger now in hand3_design.py
def chord_A(r): return float(np.interp(r * 1e3, [H.R_LAG, H.R_TIP], [18.0, 12.0])) * 1e-3
def chord_C(r): return float(np.interp(r * 1e3, [H.R_LAG, H.R_TIP], [30.0, 24.0])) * 1e-3
def chord_D(r): return H.chord(r * 1e3) * 1e-3
def thick_A(r): return float(np.interp(r * 1e3, [H.R_LAG, H.R_TIP], [4.5, 3.2]))            # mm
def _tcA(r, pad): return (thick_A(r) + pad) * 1e-3
def tc_D(r): return (H.thick(r * 1e3) + H.PAD_T) * 1e-3 / chord_D(r)
PLANS = {
    "A first finger": dict(chord=chord_A, tc=lambda r: _tcA(r, 2.0) / chord_A(r), rpm_ref=11000.0, thick=thick_A, pad=2.0, faired=False,
                           shape=lambda r: naca4(0.0, 0.4, _tcA(r, 2.0) / chord_A(r))),          # upper bound only: a faired section this thick
    "B shaped, same chord": dict(chord=chord_A, rpm_ref=11000.0, tc=lambda r: _tcA(r, 1.0) / chord_A(r), thick=thick_A, pad=1.0, faired=False,
                                 shape=lambda r: naca4(0.03, 0.4, _tcA(r, 1.0) / chord_A(r))),
    "C trailing fairing": dict(chord=chord_C, rpm_ref=8000.0, tc=lambda r: _tcA(r, 2.0) / chord_C(r), thick=thick_A, pad=2.0, faired=False,
                               shape=lambda r: naca4(0.04, 0.4, _tcA(r, 2.0) / chord_C(r))),
    "D thin airfoil finger": dict(chord=chord_D, rpm_ref=10000.0, tc=tc_D, thick=lambda r: H.thick(r * 1e3), pad=H.PAD_T, faired=True, pitch=H.PITCH,
                                  shape=lambda r: naca4(H.CAMBER, H.CAMBER_X, tc_D(r))),
}
CD_BRIDGE, CD_BARREL = 0.30, 0.25      # saddle bridge under a printed teardrop fairing; barrel = streamwise cylinder with bullet ends (est.)
def protuberances(plan):
    """Knuckle hardware standing out of the section: (radius, drag area m^2) per blade.  Unfaired: Cd 1.0."""
    P = PLANS[plan]; out = []
    for k, rk in enumerate(H.KN):
        t = P["thick"](rk * 1e-3)
        bridge = max(H.LEVER[k] + 1.4 - (t / 2 + P["pad"]), 0) * 7.0                # saddle bridge + cord above the pad, mm^2
        barrel = max(H.BARREL[k] / 2 - t / 2, 0) * H.BARREL[k] * (2 if P["faired"] else 1)   # thin blade: barrel stands out of both faces
        out.append((rk * 1e-3, (bridge * (CD_BRIDGE if P["faired"] else 1.0) + barrel * (CD_BARREL if P["faired"] else 1.0)) * 1e-6))
    return out
GAPS = [(rk * 1e-3, 2.4e-3) for rk in H.KN]


# ------------------------------------------------------------------ polars
def hoerner_cd0(tc, Re, turbulent):
    if turbulent: return 2 * 0.074 / Re ** 0.2 * (1 + 2 * tc + 60 * tc ** 4)
    return 2 * 1.328 / np.sqrt(Re) * (1 + tc) + 1.1 * tc ** 2                       # subcritical, separated


def polar_tables(name, source):
    """cl, cd [section, alpha].  source: 'nf_turb' NeuralFoil, early transition (n_crit 4: rotor wake, printed surface), clean;
    'nf_trip' NeuralFoil, transition forced at 5 % chord, derated for knuckle gaps, pad seams and layer lines (cl x 0.9,
    cd x 1.3 + 0.004); 'bluff' handbook for a thick rounded section below its critical Reynolds number (Hoerner)."""
    P = PLANS[name]; cl = np.zeros((len(R_SEC), len(ALPHAS))); cd = cl.copy(); conf = []
    if source.startswith("nf"):
        import aerosandbox as asb
        for i, r in enumerate(R_SEC):
            V = P["rpm_ref"] * 2 * np.pi / 60 * r; Re = V * P["chord"](r) / NU
            kw = dict(n_crit=4.0) if source == "nf_turb" else dict(xtr_upper=0.05, xtr_lower=0.05)
            a = asb.Airfoil(name="s", coordinates=P["shape"](r)).get_aero_from_neuralfoil(alpha=ALPHAS, Re=Re, mach=min(V / A_SND, 0.6), model_size="xlarge", **kw)
            cl[i], cd[i] = a["CL"], a["CD"]; conf.append(float(np.mean(a["analysis_confidence"][8:20])))
        if source == "nf_trip": cl *= 0.9; cd = cd * 1.3 + 0.004
    else:
        for i, r in enumerate(R_SEC):
            V = P["rpm_ref"] * 2 * np.pi / 60 * r; Re = V * P["chord"](r) / NU; tc = P["tc"](r)
            k = float(np.clip(1.0 - 1.6 * tc, 0.3, 0.8)); clmax = float(np.clip(1.05 - 1.3 * tc, 0.4, 0.9))
            a = np.radians(ALPHAS); lin = k * 2 * np.pi * a
            cl[i] = np.where(np.abs(lin) < clmax, lin, np.sign(lin) * (clmax * 0.9 + 0.1 * clmax * np.cos(2 * (np.abs(a) - clmax / (k * 2 * np.pi)))))
            cd[i] = hoerner_cd0(tc, Re, False) + 1.2 * np.sin(a) ** 2
    return cl, cd, conf


# ------------------------------------------------------------------ BEMT (hover)
Q_CAL = 1.17      # torque calibration: the method gives 699 W of shaft power where the maker's full-throttle point implies 820 W
def bemt(rpm, chord, pitch_deg, cl_t, cd_t, nb=NB, r0=R0, R_=R, prot=(), gaps=(), n=160, q_cal=Q_CAL):
    """Hover BEMT, all stations solved together (bisection on the induced velocity)."""
    om = rpm * 2 * np.pi / 60
    rr = r0 + (R_ - r0) * (np.arange(n) + 0.5) / n; dr = (R_ - r0) / n
    c = np.array([chord(r) for r in rr]); th = np.radians([pitch_deg(r) for r in rr])
    w = np.clip((rr - R_SEC[0]) / (R_SEC[-1] - R_SEC[0]) * (len(R_SEC) - 1), 0, len(R_SEC) - 1.001); j = w.astype(int); f = w - j
    gapm = np.zeros(n, bool)
    for rk, wd in gaps: gapm |= np.abs(rr - rk) < wd / 2
    def coef(a):
        u = np.clip(np.degrees(a), ALPHAS[0], ALPHAS[-1] - 1e-6) - ALPHAS[0]; k = u.astype(int); g = u - k
        tab = lambda t: (1 - f) * ((1 - g) * t[j, k] + g * t[j, k + 1]) + f * ((1 - g) * t[j + 1, k] + g * t[j + 1, k + 1])
        return np.where(gapm, 0.0, tab(cl_t)), tab(cd_t)
    def resid(vi):
        ph = np.arctan2(vi, om * rr); l, d = coef(th - ph); W2 = (om * rr) ** 2 + vi ** 2
        F = 2 / np.pi * np.arccos(np.exp(-np.minimum(nb / 2 * (R_ - rr) / (rr * np.maximum(np.sin(ph), 1e-4)), 50)))
        be = 0.5 * RHO * W2 * nb * c * (l * np.cos(ph) - d * np.sin(ph))
        return be - 4 * np.pi * RHO * rr * vi * vi * np.maximum(F, 1e-3), be, 0.5 * RHO * W2 * nb * c * (l * np.sin(ph) + d * np.cos(ph)) * rr, th - ph
    lo = np.full(n, 1e-4); hi = np.full(n, 0.6 * om * R_); act = resid(lo)[0] > 0
    for _ in range(45):
        mid = 0.5 * (lo + hi); pos = resid(mid)[0] > 0
        lo = np.where(pos, mid, lo); hi = np.where(pos, hi, mid)
    vi = np.where(act, 0.5 * (lo + hi), 1e-4)
    _, be, q, a = resid(vi)
    Q = float(q.sum() * dr)
    for rk, cda in prot: Q += nb * 0.5 * RHO * (om * rk) ** 2 * cda * rk
    Q *= q_cal
    return dict(T=float(be.sum() * dr), Q=Q, P=Q * om, rr=rr, dT=be * dr, alpha=np.degrees(a))


# ------------------------------------------------------------------ motor (maker data: BrotherHobby SE 2816 810KV on 6S)
KV, I_MAX, I0 = 810.0, 55.2, 1.5
KT = 60 / (2 * np.pi * KV)
V_FULL = 1303.0 / 55.2                                   # 23.6 V at the maker's full-throttle point
R_EFF = (V_FULL - 12368.0 / KV) / 55.2                   # effective drop per amp at full throttle (motor + ESC + Kv droop)
def motor_torque(rpm, i_lim=I_MAX):
    return KT * (min(i_lim, max((V_FULL - rpm / KV) / R_EFF, 0)) - I0)


DRUM_R, DRUM_Z = 0.051, 0.038
def net_thrust(res):
    """Servo drum under the rotor: the slipstream from the inner disc lands on its top face (download, est.)."""
    r_s = DRUM_R / 0.9                                    # wake contraction at 0.3 R below the disc (est.)
    return res["T"] - 0.7 * res["dT"][res["rr"] < r_s].sum()


def full_throttle(run, i_lim=I_MAX):
    lo, hi = 3000.0, 19000.0
    for _ in range(28):
        mid = 0.5 * (lo + hi)
        if run(mid)["Q"] < motor_torque(mid, i_lim): lo = mid
        else: hi = mid
    return 0.5 * (lo + hi)


def rpm_for(run, T_want):
    lo, hi = 3000.0, 22000.0
    for _ in range(28):
        mid = 0.5 * (lo + hi)
        if net_thrust(run(mid)) < T_want: lo = mid
        else: hi = mid
    return 0.5 * (lo + hi)


def twist_fn(base, steps):
    """Pitch per phalanx: base + steps[k] on phalanx k."""
    def f(r):
        k = int(np.searchsorted(np.array(H.KN[1:]) * 1e-3, r))
        return base + steps[k]
    return f


def validate():
    """Sanity check of the method on a moulded 10x5x3 (HQ geometry guessed: +-15 %), against the maker's bench table."""
    import aerosandbox as asb
    ch = lambda r: float(np.interp(r / 0.127, [0.15, 0.35, 0.5, 0.75, 0.9, 1.0], [0.16, 0.21, 0.19, 0.14, 0.10, 0.05])) * 0.127
    pt = lambda r: float(np.degrees(np.arctan(0.127 / (2 * np.pi * r))))
    cl = np.zeros((len(R_SEC), len(ALPHAS))); cd = cl.copy()
    for i, r in enumerate(R_SEC):
        V = 11000.0 * 2 * np.pi / 60 * r
        a = asb.Airfoil(name="s", coordinates=naca4(0.05, 0.4, float(np.interp(r, [R0, R], [0.13, 0.08])))).get_aero_from_neuralfoil(alpha=ALPHAS, Re=V * ch(r) / NU, mach=min(V / A_SND, 0.6), model_size="xlarge")
        cl[i], cd[i] = a["CL"], a["CD"]
    out = []
    for rpm, T_meas, P_el in ((10163, 22.4, 633), (10935, 28.3, 895), (12368, 34.7, 1303)):
        b = bemt(rpm, ch, pt, cl, cd, nb=3, r0=0.019, q_cal=1.0)
        out.append(dict(rpm=rpm, T_calc=round(b["T"], 1), T_maker=T_meas, P_shaft_calc=round(b["P"]), P_electrical_maker=P_el))
    return out


def study():
    out = dict(validation=validate(), motor=dict(Kt=round(KT, 5), V_full=round(V_FULL, 1), R_eff=round(R_EFF, 3)), plans={})
    cases = [("A first finger", "bluff", "low: handbook, separated flow"), ("A first finger", "nf_turb", "high: as a faired section this thick"),
             ("B shaped, same chord", "nf_trip", "low: tripped, derated"), ("B shaped, same chord", "nf_turb", "high: clean"),
             ("C trailing fairing", "nf_trip", "low: tripped, derated"), ("C trailing fairing", "nf_turb", "high: clean"),
             ("D thin airfoil finger", "nf_trip", "low: tripped, derated"), ("D thin airfoil finger", "nf_turb", "high: clean")]
    for name, src, label in cases:
        P = PLANS[name]; cl, cd, conf = polar_tables(name, src); prot = protuberances(name)
        best = None
        for pitch in ([P["pitch"]] if "pitch" in P else np.arange(6.0, 30.01, 2.0)):
            run = lambda rpm, p=pitch: bemt(rpm, P["chord"], lambda r: p, cl, cd, prot=prot, gaps=GAPS)
            rpm_ft = full_throttle(run); Tn = net_thrust(run(rpm_ft))
            if best is None or Tn > best[1]: best = (pitch, Tn, rpm_ft, run)
        pitch, Tmax, rpm_ft, run = best
        row = dict(section=label, nf_confidence=round(float(np.mean(conf)), 2) if conf else None, best_pitch=pitch,
                   T_max=round(Tmax, 1), rpm_max=round(rpm_ft), P_shaft_max=round(run(rpm_ft)["P"]),
                   T_at_11680=round(net_thrust(run(11680.0)), 1), P_shaft_at_11680=round(run(11680.0)["P"]),
                   download_at_max=round(run(rpm_ft)["T"] - Tmax, 1), alpha_75=round(float(np.interp(0.75 * R, run(rpm_ft)["rr"], run(rpm_ft)["alpha"])), 1))
        for T_want, key in ((13.8, "hover"), (26.5, "ceiling")):
            if Tmax >= T_want:
                rp = rpm_for(run, T_want); b = run(rp)
                ideal = b["T"] ** 1.5 / np.sqrt(2 * RHO * np.pi * R * R)
                row[key] = dict(rpm=round(rp), P_shaft=round(b["P"]), P_el=round(b["P"] / 0.80), FM=round(ideal / b["P"], 2), tip_mach=round(rp * 2 * np.pi / 60 * R / A_SND, 2))
            else: row[key] = None
        # stepped twist: each phalanx at its own incidence
        bt = None
        for base in np.arange(pitch - 4, pitch + 4.01, 2.0):
            for st in ((8, 3, 0), (6, 2, 0), (10, 4, 0), (4, 2, 0)):
                runt = lambda rpm, b_=base, s_=st: bemt(rpm, P["chord"], twist_fn(b_, s_), cl, cd, prot=prot, gaps=GAPS)
                rf = full_throttle(runt); Tn = net_thrust(runt(rf))
                if bt is None or Tn > bt[0]: bt = (Tn, base, st, rf)
        row["twist"] = dict(T_max=round(bt[0], 1), tip_pitch=bt[1], steps=bt[2], rpm=round(bt[3]))
        row["curve"] = [(int(rp), round(net_thrust(run(float(rp))), 2)) for rp in range(4000, 17001, 1000)]
        out["plans"].setdefault(name, []).append(row)
        print(name, "|", label, {k: v for k, v in row.items() if k != "curve"}, flush=True)
    return out


def chart(out):
    """Maximum net thrust per hand at full throttle, low to high section data, against what the robot needs."""
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7.2, 5.4), dpi=130)
    col = {"A first finger": "#d1242f", "B shaped, same chord": "#bf8700", "C trailing fairing": "#6e7781", "D thin airfoil finger": "#1a7f37"}
    sub = {"A first finger": "18-12 mm chord, 2 mm pad proud, 36-43 % thick", "B shaped, same chord": "18-12 mm chord, airfoil shape, 31-35 % thick",
           "C trailing fairing": "30-24 mm chord with a printed lip, 22 % thick",
           "D thin airfoil finger": "22-18 mm chord, 1 mm flush pad, 19-20 % thick, faired knuckles  (NOW ON THE BLUEPRINTS)"}
    for i, (name, rows) in enumerate(out["plans"].items()):
        lo, hi = sorted(float(r["T_max"]) for r in rows); y = 3 - i
        ax.barh(y, hi - lo, left=lo, height=0.42, color=col[name], alpha=0.85)
        ax.text(hi + 0.5, y, f"{lo:.0f}-{hi:.0f} N", va="center", fontsize=11, fontweight="bold", color=col[name])
        ax.text(0.3, y + 0.30, name + ":  " + sub[name], fontsize=8.6, va="bottom", zorder=5, bbox=dict(fc="white", ec="none", pad=1.5))
    for x, lab, ls in ((5.2, "hand's own\nweight 5.2 N", ":"), (13.8, "hover share\n13.8 N", "--"), (26.5, "XL claw-prop\nceiling 26.5 N", "-")):
        ax.axvline(x, color="#111418", lw=1, ls=ls); ax.text(x, 3.82, lab, fontsize=8, ha="center", va="bottom")
    ax.set_xlim(0, 30); ax.set_ylim(-0.45, 3.78); ax.set_yticks([]); ax.set_xlabel("maximum net thrust per hand at full throttle, N (estimate; SE 2816 810KV, 6S, 55 A)")
    for s_ in ("top", "right", "left"): ax.spines[s_].set_visible(False)
    fig.subplots_adjust(top=0.88, bottom=0.12, left=0.04, right=0.97); fig.savefig("hand3_thrust.png"); plt.close(fig)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "polars":
        for name, src in (("A first finger", "nf_turb"), ("A first finger", "bluff"), ("B shaped, same chord", "nf_trip"), ("B shaped, same chord", "nf_turb"), ("C trailing fairing", "nf_trip"), ("C trailing fairing", "nf_turb"), ("D thin airfoil finger", "nf_trip"), ("D thin airfoil finger", "nf_turb")):
            cl, cd, conf = polar_tables(name, src); i = 4
            print(name, src, "t/c", [round(PLANS[name]["tc"](r), 2) for r in R_SEC], "conf", [round(c, 2) for c in conf])
            for a in (0, 4, 8, 12, 16, 20): print("   a", a, "cl", round(float(np.interp(a, ALPHAS, cl[i])), 2), "cd", round(float(np.interp(a, ALPHAS, cd[i])), 3))
        print(validate())
    else:
        if len(sys.argv) > 1 and sys.argv[1] == "chart": out = json.load(open("hand3_thrust.json"))
        else:
            out = study(); json.dump(out, open("hand3_thrust.json", "w"), indent=1, default=float)
        chart(out); print("done")
