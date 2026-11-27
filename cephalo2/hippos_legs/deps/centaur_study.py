"""Centaur design study (2026-11-14): Cephalo-1 docks its Scabbard bars down into the neck of a four-legged body
("Equo-1") and becomes its head and torso.  Hand estimates only: no CAD, no sim.  Single source of the study numbers.
python3 centaur_study.py -> centaur_study.json + tables on stdout.   SI units in code (m, kg, N, N m); mm in docs.

Octopus numbers come from scabbard.py / scabbard-dock.md (frame: origin = centre of the plate underside, +z up).
Everything marked (est.) is an estimate to be replaced by CAD and MuJoCo.
"""
import json
import numpy as np

G = 9.81
# ------------------------------------------------------------------ octopus (from the project docs)
OCTO_M = 12.81          # kg, XL claw-props + 2 packs + Scabbard rotor (scabbard-dock.md; hand3 hubs may move this)
COM_H = 0.060           # CoM above the plate underside, arms low (est., scabbard.py)
PLATE_R = 0.125
HUB_FACE = 0.034        # hub pad face below the plate underside
PAD_R = 0.0475          # hub pad radius (seat of option A)
R_S = 0.070             # bar (blade) radius
TIP_STOW = 0.030        # stowed tip below the plate underside
STROKE_DOCK, STROKE_FULL = 0.108, 0.194
EYE_FROM_TIP = 0.061    # eyelet above the tip (X_TIP - X_LATCH)
PIN_CAP = 980.0         # N, eyelet bearing capacity (est., scabbard.py)
CINCH = 60.0            # N, elevator cinch (est.)
TURRET_TOP = 0.224
XL_T_HOVER = 15.7; XL_KP = 5.1          # N per prop in hover on 8 props; P = 5.1 W x T^1.5

# ------------------------------------------------------------------ neck dock options
COLLAR_R = (0.100, 0.118)               # collar seat ring on the plate underside (inside the plate rim, outside the push rods)
NECK = {
    "A dummy belly":  dict(seat_r=PAD_R, com=COM_H + HUB_FACE, ties=4, stroke=STROKE_DOCK, preload=CINCH,
                           path="elevator belts (2 GT2 loops, STS3215)"),
    "B deep collar":  dict(seat_r=np.mean(COLLAR_R), com=COM_H, ties=4, stroke=STROKE_FULL, preload=0.0,
                           path="blade heads seated in their trunnions (hard shoulders)"),
    "C mutual":       dict(seat_r=np.mean(COLLAR_R), com=COM_H, ties=8, stroke=STROKE_FULL, preload=0.0,
                           path="as B + 4 horse bars latched by the octopus's own pins"),
}
ARM_WORK = 12.0          # N m, two arms out at 0.45 m with 1 kg each + their own shift (est.)


def tie_per_Nm(seat_r, n):
    """Worst tie force per N m of moment beyond the gravity hold: ties at R_S (n of them, evenly spaced) stretch in
    proportion to their distance from the tipping edge of the seat (rigid parts, elastic ties)."""
    worst = 0.0
    for phi in np.linspace(0, np.pi / 2, 181):
        d = seat_r + R_S * np.cos(2 * np.pi * np.arange(n) / n - phi)
        worst = max(worst, d.max() / (d ** 2).sum())
    return worst


def neck_loads():
    W = OCTO_M * G
    out = {}
    for name, o in NECK.items():
        k = tie_per_Nm(o["seat_r"], o["ties"])
        hold = (W + o["preload"]) * o["seat_r"]                 # moment the seat resists before any tie loads up
        cases = {
            "trot, 0.5 g sideways (est.)": (0.5 * W * o["com"], hold),
            "hard stop / turn, 1 g": (1.0 * W * o["com"], hold),
            "two arms working + 0.5 g": (0.5 * W * o["com"] + ARM_WORK, hold),
            "horse lying on its side": (1.0 * W * o["com"], o["preload"] * o["seat_r"]),
            "tip-over hit, 3 g sideways (est.)": (3.0 * W * o["com"], 0.0),
        }
        rows = {}
        for c, (M, h) in cases.items():
            F = max(M - h, 0.0) * k
            rows[c] = dict(M=M, hold=h, tie=F, margin=(PIN_CAP / F if F > 0 else float("inf")))
        tip_z = -(TIP_STOW + o["stroke"])
        face = -HUB_FACE
        out[name] = dict(seat_r=o["seat_r"], gravity_hold=hold, tie_per_Nm=k, cases=rows, path=o["path"],
                         bar_below_pad=face - tip_z, pin_below_pad=face - (tip_z + EYE_FROM_TIP),
                         open_gap=(2 * HUB_FACE if name.startswith("A") else 0.0))
    return out


# ------------------------------------------------------------------ horse body (recommended proportions, est.)
L_HIP = 0.440            # front hip line to rear hip line
L1 = L2 = 0.190          # thigh, shank (to the foot centre)
EXT = 0.95               # stand at 95 % of full leg length ("column legs": short knee lever)
H_HIP = EXT * (L1 + L2)  # 0.361
Y_FOOT = 0.170           # foot from the centreline
BODY = dict(len=0.560, w=0.150, z0=0.310, z1=0.410)
STRIDE = 0.140
X_NECK = {"P1 over the front hips": 0.0, "P2 withers": 0.110, "P3 saddle (centre)": 0.220}   # behind the front hip line
X_HORSE = 0.250          # horse's own CoM behind the front hip line, packs at the croup (est.)
Z_HORSE = 0.310          # horse CoM height (est.)

# real actuators.  rated / peak (stall) torque N m, mass kg, speed at rated load rad/s, price USD (2026-11 listings)
ACT = {
    "STS3250":   dict(rated=1.63, peak=4.90, m=0.0745, w=np.radians(300), usd=(35, 45), src="Feetech datasheet (project); rated = stall/3, speed est."),
    "CyberGear": dict(rated=4.0, peak=12.0, m=0.317, w=240 * 2 * np.pi / 60, usd=(110, 180), src="Xiaomi manual: 4 / 12 N m, 317 g, 240 rpm, 24 V, dia 80.5 x 36.5"),
    "AK60-6":    dict(rated=3.0, peak=9.0, m=0.368, w=220 * 2 * np.pi / 60, usd=(299, 390), src="CubeMars AK60-6 V1.1 KV80: 3 / 9 N m, 368 g, 24 V, dia 79 x 39.5"),
    "AK70-10":   dict(rated=8.3, peak=24.8, m=0.521, w=148 * 2 * np.pi / 60, usd=(399, 549), src="CubeMars AK70-10 KV100: 8.3 / 24.8 N m, 521 g, 24/48 V, dia 89 x 50.25"),
}
BELT_EFF = 0.95
# leg options: (actuator, ratio) for abduction (haa), hip pitch (hfe), knee (kfe); extra = reduction hardware per leg
LEGS = {
    "S servo plodder": dict(haa=("STS3250", 2.0), hfe=("STS3250", 3.0), kfe=("STS3250", 3.0), extra=0.24, struct=0.20),
    "G CyberGear":     dict(haa=("CyberGear", 1.0), hfe=("CyberGear", 1.0), kfe=("CyberGear", 2.0), extra=0.06, struct=0.30),
    "K AK70 / AK60":   dict(haa=("AK60-6", 1.0), hfe=("AK70-10", 1.0), kfe=("AK70-10", 1.0), extra=0.0, struct=0.36),
}
M_FIXED = dict(frame=0.80, neck=0.35, packs=1.62, electronics=0.25, wiring=0.20, feet=0.12)
PACK_WH = 2 * 5.0 * 22.2


def horse_mass(opt):
    o = LEGS[opt]
    act = 4 * sum(ACT[o[j][0]]["m"] for j in ("haa", "hfe", "kfe"))
    legs = 4 * (o["struct"] + o["extra"])
    return act + legs + sum(M_FIXED.values()), act


def cost(opt):
    o = LEGS[opt]
    lo = 4 * sum(ACT[o[j][0]]["usd"][0] for j in ("haa", "hfe", "kfe"))
    hi = 4 * sum(ACT[o[j][0]]["usd"][1] for j in ("haa", "hfe", "kfe"))
    return lo, hi


def knee_xz(x, H, sign=1.0):
    """Knee position relative to the hip for a foot at (x, -H); sign +1 = knee ahead of the hip-foot line."""
    d = np.hypot(x, H)
    a = (L1 ** 2 - L2 ** 2 + d ** 2) / (2 * d)
    h = np.sqrt(max(L1 ** 2 - a ** 2, 0.0))
    u = np.array([x, -H]) / d
    n = np.array([H, x]) / d * sign
    return a * u + h * n


def stance(F, H=H_HIP, stride=STRIDE):
    """Hip-pitch and knee torque over a stance sweep for a vertical foot force F (conservative: no leg-strut effect)."""
    xs = np.linspace(stride / 2, -stride / 2, 41)
    th = np.array([F * x for x in xs])
    tk = np.array([F * (x - knee_xz(x, H)[0]) for x in xs])
    return np.abs(th), np.abs(tk)


def leg_case(m_tot, share):
    """Front-leg joint torques (the loaded pair) for stand, walk, trot and getting up.  Returns rms over the whole
    cycle and the peak, per joint."""
    W = m_tot * G
    Ff = share * W                      # one front foot when its partner is in the air (walk) or in a diagonal pair (trot)
    haa_arm = 0.015 + 0.05 * H_HIP      # foot 15 mm off the abduction axis + 5 % side force at hip height (est.)
    res = {}
    # stand
    th, tk = stance(Ff / 2, stride=0.0)
    res["stand"] = dict(hfe=(th.max(), th.max()), kfe=(tk.max(), tk.max()), haa=(Ff / 2 * 0.015,) * 2)
    # walk: duty 0.75; a third of the stance at Ff (the other front foot is swinging), the rest at Ff / 2; x1.2 peak
    th1, tk1 = stance(Ff); th2, tk2 = stance(Ff / 2)
    def rms_walk(a, b): return np.sqrt(0.75 * (np.mean(a ** 2) / 3 + 2 * np.mean(b ** 2) / 3))
    res["walk"] = dict(hfe=(rms_walk(th1, th2), 1.2 * th1.max()), kfe=(rms_walk(tk1, tk2), 1.2 * tk1.max()),
                       haa=(np.sqrt(0.75 * (1 / 3 + 2 / 12)) * Ff * haa_arm, 1.2 * Ff * haa_arm))
    # trot: duty 0.5 at Ff; x1.4 peak (est.)
    res["trot"] = dict(hfe=(np.sqrt(0.5 * np.mean(th1 ** 2)), 1.4 * th1.max()), kfe=(np.sqrt(0.5 * np.mean(tk1 ** 2)), 1.4 * tk1.max()),
                       haa=(np.sqrt(0.5) * Ff * haa_arm, 1.4 * Ff * haa_arm))
    # getting up from the kneel (hip at 45 % of leg length, four feet): peak only
    th, tk = stance(Ff / 2, H=0.45 * (L1 + L2), stride=0.0)
    res["get up"] = dict(hfe=(0.0, th.max()), kfe=(0.0, tk.max()), haa=(0.0, Ff / 2 * 0.015))
    return res


def layout(m_h, x_neck):
    m = OCTO_M + m_h
    xc = (OCTO_M * x_neck + m_h * X_HORSE) / m
    share = 1 - xc / L_HIP
    z_plate = BODY["z0"] + STROKE_FULL - (HUB_FACE - TIP_STOW) + HUB_FACE     # sockets end at the belly; see neck B
    zc = (OCTO_M * (z_plate + COM_H) + m_h * Z_HORSE) / m
    return dict(m=m, xc=xc, share=share, z_plate=z_plate, zc=zc, top=z_plate + TURRET_TOP,
                tip_side=np.degrees(np.arctan(Y_FOOT / zc)), tip_front=np.degrees(np.arctan(xc / zc)),
                tip_rear=np.degrees(np.arctan((L_HIP - xc) / zc)))


def study():
    out = dict(neck=neck_loads(), legs={}, layout={})
    for opt in LEGS:
        m_h, m_act = horse_mass(opt)
        lay = layout(m_h, X_NECK["P2 withers"])
        cases = leg_case(lay["m"], lay["share"])
        o = LEGS[opt]; rows = {}
        for gait, c in cases.items():
            rows[gait] = {}
            for j in ("haa", "hfe", "kfe"):
                a = ACT[o[j][0]]; r = o[j][1]; eff = BELT_EFF if r > 1 else 1.0
                rated, peak = a["rated"] * r * eff, a["peak"] * r * eff
                rms, pk = c[j]
                rows[gait][j] = dict(rms=rms, peak=pk, rated=rated, limit=peak,
                                     m_rms=(rated / rms if rms > 0 else float("inf")), m_peak=peak / pk)
        out["legs"][opt] = dict(m_horse=m_h, m_act=m_act, m_total=lay["m"], share=lay["share"], usd=cost(opt), gaits=rows,
                                knee_speed=np.degrees(ACT[o["kfe"][0]]["w"] / o["kfe"][1]))
    m_h = horse_mass("G CyberGear")[0]
    for p, x in X_NECK.items():
        lay = layout(m_h, x)
        c = leg_case(lay["m"], lay["share"])
        lay.update(front_foot_trot=lay["share"] * lay["m"] * G, rear_foot_trot=(1 - lay["share"]) * lay["m"] * G,
                   knee_rms_trot=c["trot"]["kfe"][0], knee_peak_trot=c["trot"]["kfe"][1])
        out["layout"][p] = lay
    # energy (rough): cost of transport 1.0 at 0.5 m/s + 20 W electronics; 80 % of two 6S 5000 mAh packs
    lay = out["layout"]["P2 withers"]
    p_walk = 1.0 * lay["m"] * G * 0.5 + 20.0
    out["energy"] = dict(P_walk=p_walk, minutes=0.8 * PACK_WH / p_walk * 60, km=0.8 * PACK_WH / p_walk * 0.5 * 3.6,
                         P_assist_half=8 * XL_KP * (XL_T_HOVER / 2) ** 1.5, P_hover=8 * XL_KP * XL_T_HOVER ** 1.5)
    return out


if __name__ == "__main__":
    S = study()
    json.dump(S, open("centaur_study.json", "w"), indent=1, default=float)
    print("NECK DOCK")
    for n, o in S["neck"].items():
        print(f"  {n:15s} seat r {o['seat_r']*1e3:5.1f} mm  gravity hold {o['gravity_hold']:5.1f} N m  tie {o['tie_per_Nm']:.2f} N per N m"
              f"  bar {o['bar_below_pad']*1e3:.0f} mm into the neck, pin {o['pin_below_pad']*1e3:.0f} mm down, open gap {o['open_gap']*1e3:.0f} mm")
        for c, r in o["cases"].items():
            print(f"      {c:36s} M {r['M']:5.1f}  hold {r['hold']:5.1f}  worst tie {r['tie']:6.1f} N  margin {r['margin']:.1f}")
    print("LAYOUT (leg option G)")
    for p, l in S["layout"].items():
        print(f"  {p:24s} total {l['m']:.1f} kg  front share {l['share']*100:4.1f} %  trot foot F/R {l['front_foot_trot']:.0f}/{l['rear_foot_trot']:.0f} N"
              f"  knee rms/peak {l['knee_rms_trot']:.1f}/{l['knee_peak_trot']:.1f} N m  plate {l['z_plate']*1e3:.0f}  top {l['top']*1e3:.0f}  CoM z {l['zc']*1e3:.0f}"
              f"  tip side {l['tip_side']:.1f} front {l['tip_front']:.1f} rear {l['tip_rear']:.1f} deg")
    print("LEGS (octopus at P2)")
    for n, o in S["legs"].items():
        print(f"  {n:16s} horse {o['m_horse']:.2f} kg (actuators {o['m_act']:.2f})  centaur {o['m_total']:.1f} kg  front share {o['share']*100:.0f} %"
              f"  ${o['usd'][0]}-{o['usd'][1]}  knee speed {o['knee_speed']:.0f} deg/s")
        for g, r in o["gaits"].items():
            s = "   ".join(f"{j} {v['rms']:4.1f}/{v['peak']:4.1f} of {v['rated']:4.1f}/{v['limit']:4.1f} (x{min(v['m_rms'],99):.2f}, x{v['m_peak']:.2f})" for j, v in r.items())
            print(f"      {g:7s} {s}")
    e = S["energy"]
    print(f"ENERGY  walk {e['P_walk']:.0f} W -> {e['minutes']:.0f} min, {e['km']:.1f} km;  half-lift assist {e['P_assist_half']:.0f} W, hover {e['P_hover']:.0f} W")
