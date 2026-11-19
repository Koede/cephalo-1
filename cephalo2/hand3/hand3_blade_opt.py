"""Three-phalanx hand-prop: search for a finger blade that flies (2026-11-13).  Uses the BEMT, polars, motor line and
drum download of hand3_thrust.py.  ESTIMATE.  python3 hand3_blade_opt.py -> hand3_blade_opt.json (ranked table on stdout).

Variables: chord root -> tip, printed thickness root -> tip, pad thickness (flush in the upper surface), camber, pitch.
Limits: the two fingers of a pair lie side by side when the hand is spread (lag pins 28.3 mm apart), so the chord is
capped; thickness is capped below by the Ø1 cord grooves and the knuckle hardware.
Score: maximum net thrust at full throttle with the LOW section data (transition forced, derated)."""
import json, itertools
import numpy as np
import hand3_design as H
import hand3_thrust as T

PIN_SEP = 2 * H.R_LAG * np.sin(np.radians(45))            # 28.3 mm between the lag pins of a pair


def pair_gap(c_root, c_tip, toe_deg):
    """Smallest gap between the two flat fingers of a pair, both pointing the same way and toed in by toe_deg (plan geometry)."""
    s = np.linspace(0, H.R_TIP - H.R_LAG, 200)
    c = np.interp(s, [0, s[-1]], [c_root, c_tip])
    return float(np.min(PIN_SEP - 2 * s * np.sin(np.radians(toe_deg)) - c / np.cos(np.radians(toe_deg))))


def max_toe(c_root, c_tip, gap=1.5):
    for toe in np.arange(6.0, -0.01, -0.1):
        if pair_gap(c_root, c_tip, toe) >= gap: return round(float(toe), 1)
    return None


def tables(c0, c1, t0, t1, pad, m, rpm_ref=10000.0):
    import aerosandbox as asb
    ch = lambda r: float(np.interp(r * 1e3, [H.R_LAG, H.R_TIP], [c0, c1])) * 1e-3
    tc = lambda r: (float(np.interp(r * 1e3, [H.R_LAG, H.R_TIP], [t0, t1])) + pad) * 1e-3 / ch(r)
    out = {}
    for src in ("nf_trip", "nf_turb"):
        cl = np.zeros((len(T.R_SEC), len(T.ALPHAS))); cd = cl.copy()
        for i, r in enumerate(T.R_SEC):
            V = rpm_ref * 2 * np.pi / 60 * r
            kw = dict(n_crit=4.0) if src == "nf_turb" else dict(xtr_upper=0.05, xtr_lower=0.05)
            a = asb.Airfoil(name="s", coordinates=T.naca4(m, 0.4, tc(r))).get_aero_from_neuralfoil(alpha=T.ALPHAS, Re=V * ch(r) / T.NU, mach=min(V / T.A_SND, 0.6), model_size="xlarge", **kw)
            cl[i], cd[i] = a["CL"], a["CD"]
        if src == "nf_trip": cl *= 0.9; cd = cd * 1.3 + 0.004
        out[src] = (cl, cd)
    return ch, tc, out


CD_BRIDGE, CD_BARREL = 0.30, 0.25      # saddle bridge under a printed teardrop fairing; barrel = streamwise cylinder with bullet ends (est.)
def prot(t0, t1, pad, lever=None, faired=True):
    """Knuckle hardware standing out of the section: (radius, drag area m^2) per blade."""
    lever = H.LEVER if lever is None else lever; out = []
    for k, rk in enumerate(H.KN):
        t = float(np.interp(rk, [H.R_LAG, H.R_TIP], [t0, t1]))
        bridge = max(lever[k] + 1.4 - (t / 2 + pad), 0) * 7.0
        barrel = max(H.BARREL[k] / 2 - t / 2, 0) * H.BARREL[k] * 2            # barrel stands out of both faces of a thin blade
        out.append((rk * 1e-3, (bridge * (CD_BRIDGE if faired else 1.0) + barrel * (CD_BARREL if faired else 1.0)) * 1e-6))
    return out


def evaluate(c0, c1, t0, t1, pad, m, pitches=np.arange(5.0, 20.01, 1.0), lever=None, faired=True):
    ch, tc, tb = tables(c0, c1, t0, t1, pad, m); pr = prot(t0, t1, pad, lever, faired)
    res = dict(chord=(c0, c1), thick=(t0, t1), pad=pad, camber=m, tc=(round(tc(T.R0), 3), round(tc(T.R), 3)), toe_max=max_toe(c0, c1))
    best = None
    for p in pitches:
        run = lambda rpm, src="nf_trip": T.bemt(rpm, ch, lambda r: p, *tb[src], prot=pr, gaps=T.GAPS)
        rf = T.full_throttle(run); Tn = T.net_thrust(run(rf))
        if best is None or Tn > best[1]: best = (float(p), Tn, rf)
    p = best[0]; res["pitch"] = p
    for src, key in (("nf_trip", "low"), ("nf_turb", "high")):
        run = lambda rpm, s_=src: T.bemt(rpm, ch, lambda r: p, *tb[s_], prot=pr, gaps=T.GAPS)
        rf = T.full_throttle(run); b = run(rf); Tn = T.net_thrust(b)
        d = dict(T_max=round(Tn, 1), rpm=round(rf), P_shaft=round(b["P"]), tip_mach=round(rf * 2 * np.pi / 60 * T.R / T.A_SND, 2))
        if Tn >= 13.8:
            rh = T.rpm_for(run, 13.8); bh = run(rh)
            d.update(hover_rpm=round(rh), hover_P_el=round(bh["P"] / 0.80), hover_FM=round(bh["T"] ** 1.5 / np.sqrt(2 * T.RHO * np.pi * T.R ** 2) / bh["P"], 2),
                     hover_alpha75=round(float(np.interp(0.75 * T.R, bh["rr"], bh["alpha"])), 1))
        res[key] = d
    return res


if __name__ == "__main__":
    import sys
    rows = []
    if len(sys.argv) > 1 and sys.argv[1] == "wide":          # first pass: hardware as on the blueprints, unfaired
        cases = [(c, th, m, None, False) for c, th, m in itertools.product([(18, 12), (20, 14), (22, 16), (24, 16), (22, 18), (24, 18), (26, 18)],
                 [(3.4, 2.4, 1.0), (3.8, 2.8, 1.0), (3.4, 2.4, 1.5), (3.0, 2.2, 1.0), (4.5, 3.2, 2.0)], [0.03, 0.05])]
    else:                                                     # second pass: faired hardware, lower middle and tip lever arms
        LV = [6.0, 4.0, 3.0]
        cases = [(c, th, m, LV, True) for c, th, m in itertools.product([(20, 16), (22, 16), (22, 18), (24, 18)], [(3.4, 2.4, 1.0), (3.2, 2.4, 1.0), (3.4, 2.4, 1.5)], [0.02, 0.03, 0.04])]
        cases += [((22, 18), (3.4, 2.4, 1.0), 0.03, None, False), ((22, 18), (3.4, 2.4, 1.0), 0.03, None, True)]
    for (c0, c1), (t0, t1, pad), m, lv, fa in cases:
        r = evaluate(c0, c1, t0, t1, pad, m, lever=lv, faired=fa); r.update(lever=lv or H.LEVER, faired=fa); rows.append(r)
        print(r["chord"], r["thick"], r["pad"], r["camber"], "lever", r["lever"], "faired", fa, "t/c", r["tc"], "toe", r["toe_max"], "pitch", r["pitch"], "| low", r["low"], "| high", r["high"], flush=True)
    rows.sort(key=lambda r: -r["low"]["T_max"])
    json.dump(rows, open("hand3_blade_opt.json", "w"), indent=1, default=float)
    print("\nbest by low estimate:")
    for r in rows[:14]: print(r["chord"], r["thick"], r["pad"], r["camber"], r["lever"], r["faired"], r["tc"], "toe", r["toe_max"], "pitch", r["pitch"], float(r["low"]["T_max"]), float(r["high"]["T_max"]), r["low"].get("hover_P_el"), r["high"].get("hover_P_el"))
