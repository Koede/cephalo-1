#!/usr/bin/env python3
"""Equo-1 leg mechanism design study (2026-11-14).  Subsystem `legs` of the centaur body study.
Hand estimates only: no CAD, no sim.  Single source of every number in legs.md.
    python3 legs_study.py  ->  legs.json, legs_1..4_*.png, tables on stdout          (numpy + matplotlib only)
SI units in code (m, kg, N, N m, rad); mm and deg in the printed tables.  Everything marked (est.) is an estimate.

Frames.  Horse: x forward, y left, z up.  Leg sagittal plane (hind-leg convention): hip at the origin, angles are
measured from straight down, positive = distal end forward.  A front leg is the same module mirrored in x.
Torque sign: "support torque" = J^T F, the torque the ground load puts on a joint (positive on the knee = flexing).
"""
import os, sys, json
import numpy as np
sys.path.insert(0, "/home/claude/octodrone")
import sk3d
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle, Polygon, Arc

HERE = os.path.dirname(os.path.abspath(__file__))
R2D, D2R = 180 / np.pi, np.pi / 180
G = 9.81

# ====================================================================== 0. brief numbers (BRIEF.md, centaur_study.py)
M_TOT = 34.0                     # kg centaur, Equo-1 v2 with RS03 front hips + RS06 (est.: 32.6 rescale + 1.4 kg heavier legs)
W = M_TOT * G
SHARE_F = 0.622                  # front legs' share (brief: 62 %)
H = 0.490                        # v2 shoulder / hip height when walking (95 % of full leg length)
LEG_MAX = 0.516                  # full leg length (v2)
STRIDE, CLEAR = 0.190, 0.068     # v2: scaled with leg length
Z_COM = 0.650                    # CoM height (v2, est.)
HIP_X = {"front": 0.0, "rear": -0.491}
Y_FOOT = 0.190
BODY = dict(len=0.740, w=0.300, z0=0.440, z1=0.680, x1=0.080)      # body box; front face 60 mm ahead of the front hip line (est.)
NECK_X, PLATE_Z, COLLAR_R = 0.050, 0.729, 0.118
F_STAND = {"front": SHARE_F * W / 2, "rear": (1 - SHARE_F) * W / 2}            # four feet down
F_PAIR = {"front": SHARE_F * W, "rear": (1 - SHARE_F) * W}                     # trot diagonal pair / lone front foot in walk
PK = {"trot": 1.4, "walk": 1.2}                                                # peak factors (est., brief)
LAND_G = 2.5                                                                   # landing factor (est., brief)
GAITS = {"walk": dict(duty=0.75, v=0.25), "trot": dict(duty=0.50, v=0.50), "fast trot": dict(duty=0.50, v=0.80)}
LAT = 0.05                       # side force as a fraction of the vertical foot load (est., as the first pass)
ACC_G = 0.25                     # push-off / braking case: horizontal force = 0.25 x vertical load (est.)
SLOPE_DEG = 10.0

# ====================================================================== 0b. verified parts (source in the comment)
ACT = {
    # Xiaomi CyberGear manual (aifitlab-wiki.super.site/xiaomi-cybergear-docs): 4 / 12 N m, Kt 0.87 N m/Arms, line R 0.45 ohm,
    # 6.5 A rated / 23 A peak (phase, peak), 296 rpm no load, 240 rpm rated, 16-28 V, 317 g, 7.75:1, standby <= 18 mA.
    "CyberGear": dict(rated=4.0, peak=12.0, m=0.317, w_rated=240, w_free=296, kt=0.87, r_ll=0.45, d=0.0805, t=0.0365,
                      v=(16, 28), v_nom=24.0, usd=(110, 180), standby_w=0.018 * 24, j_rotor=1.5e-5, gear=7.75,
                      overload={12: 28, 11: 45, 10: 60, 9: 90, 8: 160, 7: 320, 6: 700, 5: 1800, 4.5: 2500}),
    # RobStride 02 (aifitlab.com/products/robstride-02-motor, 2026-11): 6 / 17 N m, 405 g, 78.5 x 78.5 x 45.5, 7.75:1,
    # 48 V rated (24-60 V), 360 rpm rated / 410 no load at 48 V, Kt 1.22 N m/Arms, R 0.58 ohm, USD 145.
    "RobStride 02": dict(rated=6.0, peak=17.0, m=0.405, w_rated=360, w_free=410, kt=1.22, r_ll=0.58, d=0.0785, t=0.0455,
                         v=(24, 60), v_nom=44.4, usd=(136, 145), standby_w=0.5, j_rotor=2.0e-5, gear=7.75, overload={}),
    # CubeMars AK70-10 KV100 (brief table): 8.3 / 24.8 N m, 521 g, 148 rpm at 24 V, dia 89 x 50.25.
    # RobStride 06 (openelab / aifitlab / roboticscenter listings, 2026-11): 11 / 36 N m, 621 g, 88 x 88 x 49, 9:1, 15-60 V (48 rated),
    # 480 rpm no load, 100 rpm at rated load, 14.3 Apk rated phase current -> Kt about 1.09 N m/Arms (derived, est.); line R not published (0.45 est.).
    "RobStride 06": dict(rated=11.0, peak=36.0, m=0.621, w_rated=360, w_free=480, kt=1.09, r_ll=0.45, d=0.088, t=0.049,
                         v=(15, 60), v_nom=44.4, usd=(210, 255), standby_w=0.5, j_rotor=3.0e-5, gear=9.0, overload={}),
    # RobStride 03 (aifitlab / openelab listings, 2026-11): 20 / 60 N m, 880 g, 106 x 106 x 56, 9:1, 15-60 V (48 rated), 195 rpm no load,
    # 180 rpm at rated torque, Kt 2.36 N m/Arms, line R 0.39 ohm, USD 225.
    "RobStride 03": dict(rated=20.0, peak=60.0, m=0.880, w_rated=180, w_free=195, kt=2.36, r_ll=0.39, d=0.106, t=0.056,
                         v=(15, 60), v_nom=44.4, usd=(225, 245), standby_w=0.5, j_rotor=6.0e-5, gear=9.0, overload={}),
    "AK70-10": dict(rated=8.3, peak=24.8, m=0.521, w_rated=148, w_free=180, kt=None, r_ll=None, d=0.089, t=0.05025,
                    v=(24, 48), v_nom=24.0, usd=(399, 549), standby_w=0.5, j_rotor=4.1e-5, gear=10.0, overload={}),
}
# looked at, not selected (maker / reseller listings, 2026-11): rated / peak N m, kg, size mm, volts, USD, note
OTHER_ACT = {
    "RobStride 00": (5.0, 14.0, 0.310, "57 x 57 x 51", "24-60 (48 rated)", "125", "10:1, 260 rpm rated at 48 V; aifitlab.com"),
    "SteadyWin GIM8108-8": (7.5, 22.0, 0.396, "dia 92 x 55 with driver", "12-56 (48 nominal)", "114-214", "8:1; speed at 24 V not verified; openelab.com"),
    "CubeMars AK60-6": (3.0, 9.0, 0.368, "dia 79 x 39.5", "24", "299-390", "brief table"),
}
PACK_V = (19.8, 25.2)            # one 6S pack (brief)
BELT_EFF = 0.95
# Gates "Belt Tensile Properties" (fab.cba.mit.edu/.../BeltTensileProperties.pdf): recommended working tension per mm width,
# neoprene / fibreglass, <300 rpm: 5M GT2 28.0, 5M HTD 17.9, 3M GT2 20.0, 2M GT2 4.4 N/mm (x0.9 for 6.4-12.7 mm widths).
BELT_TA = {"5M GT": 28.0e3, "5M HTD": 17.9e3, "3M GT": 20.0e3}           # N per m width
BELT_MOD = 86000 * 4.448 / 0.0254                                          # 5M tensile modulus, N per m width per unit strain
BELT_BREAK_15 = 4786.0           # N, 15 mm 5M neoprene/fibreglass tensile rating (AutomationDirect 5M-15 data sheet)
ELATECH_FU, ELATECH_FT15 = 36.8e2, 1500.0   # PU/steel HTD 5M: tooth load N per m width per tooth in mesh (0 rpm); allowable tension 15 mm
# deep-groove bearings, static rating C0r (N): Nachi catalogue pp. 77-78; 6805 also NSK (3150 N)
C0 = {"6800": 985, "6802": 1260, "6805": 2940, "6810": 5800, "6900": 1130}
BRG_M = {"6800": 0.0055, "6802": 0.007, "6805": 0.021, "6810": 0.052}      # kg (6805 NSK 21 g; others est.)
S_PACF, S_PACF_BRG, S_PETG_Z = 80e6, 60e6, 20e6                            # project strength values (est.)
S_CF_TUBE = 300e6                # roll-wrapped carbon tube, bending allowable (est.)
E_CF = 100e9                     # pultruded / roll-wrapped carbon along the fibre (est.)
PACK_WH = 2 * 5.0 * 22.2


def u(t):
    """Unit vector at angle t from straight down, positive forward (x, z)."""
    return np.array([np.sin(t), -np.cos(t)])


# ====================================================================== 1. leg architectures (sagittal kinematics)
class TwoLink:
    """A: plain 2-link leg.  q = (thigh angle, knee flexion).  Knee actuator rides on the thigh, belt down the thigh."""
    key, name = "A", "A  2-link + thigh belt"

    def __init__(s, la, lb, ratio=(1.0, 2.0)):
        s.la, s.lb, s.ratio = la, lb, ratio

    def pts(s, q):
        k = s.la * u(q[0]); f = k + s.lb * u(q[0] - q[1])
        return [np.zeros(2), k, f]

    def foot(s, q):
        return s.pts(q)[-1]

    def ik(s, x, z):
        r = np.hypot(x, z); phi = np.arctan2(x, -z)
        qk = np.pi - np.arccos(np.clip((s.la ** 2 + s.lb ** 2 - r ** 2) / (2 * s.la * s.lb), -1, 1))
        al = np.arccos(np.clip((s.la ** 2 + r ** 2 - s.lb ** 2) / (2 * s.la * r), -1, 1))
        return np.array([phi + al, qk])


class Panto:
    """B: three segments.  Femur L1, tibia L2, cannon + pastern L3 (hock to ground contact).  A parallel link slaves the
    hock to the stifle, so the cannon keeps the femur's direction plus a built-in offset `delta`.
    q = (femur angle, stifle angle between femur and tibia).  Kinematically a 2-link with a virtual thigh
    A = |L1 + L3 e^(i delta)| and shank L2."""
    key, name = "B", "B  3-segment pantograph"

    def __init__(s, l1, l2, l3, delta, ratio=(1.0, 2.0)):
        s.l1, s.l2, s.l3, s.delta, s.ratio = l1, l2, l3, delta, ratio
        s.A = np.hypot(l1 + l3 * np.cos(delta), l3 * np.sin(delta))
        s.dp = np.arctan2(l3 * np.sin(delta), l1 + l3 * np.cos(delta))      # virtual thigh direction minus femur direction
        s.v = TwoLink(s.A, l2)

    def pts(s, q):
        st = s.l1 * u(q[0]); hk = st + s.l2 * u(q[0] - q[1]); f = hk + s.l3 * u(q[0] + s.delta)
        return [np.zeros(2), st, hk, f]

    def foot(s, q):
        return s.pts(q)[-1]

    def ik(s, x, z):
        thv, qv = s.v.ik(x, z)
        return np.array([thv - s.dp, qv - s.dp])

    def qv(s, q):
        """virtual knee flexion (0 = leg at full length)"""
        return q[1] + s.dp


class FiveBar:
    """C: coaxial 5-bar.  Two body-fixed actuators turn two cranks l1; two links l2 meet at the foot.  q = (front crank, rear crank)."""
    key, name = "C", "C  coaxial 5-bar"

    def __init__(s, l1, l2, ratio=(1.0, 1.0)):
        s.l1, s.l2, s.ratio = l1, l2, ratio

    def pts(s, q):
        ka, kb = s.l1 * u(q[0]), s.l1 * u(q[1])
        m, d = (ka + kb) / 2, kb - ka
        dn = np.linalg.norm(d)
        if dn < 1e-9:
            return [np.zeros(2), ka, ka + s.l2 * u(q[0]), kb, np.zeros(2)]
        h = np.sqrt(max(s.l2 ** 2 - dn ** 2 / 4, 0.0))
        n = np.array([d[1], -d[0]]) / dn
        f1, f2 = m + h * n, m - h * n
        f = f1 if f1[1] < f2[1] else f2
        return [np.zeros(2), ka, f, kb, np.zeros(2)]

    def foot(s, q):
        return s.pts(q)[2]

    def ik(s, x, z):
        r = np.hypot(x, z); phi = np.arctan2(x, -z)
        al = np.arccos(np.clip((s.l1 ** 2 + r ** 2 - s.l2 ** 2) / (2 * s.l1 * r), -1, 1))
        return np.array([phi + al, phi - al])


def jac(arch, q, e=1e-6):
    J = np.zeros((2, 2))
    for j in range(2):
        dq = np.zeros(2); dq[j] = e
        J[:, j] = (arch.foot(q + dq) - arch.foot(q - dq)) / (2 * e)
    return J


def support_torque(arch, x, z, F):
    """Joint torques the ground force F = (Fx, Fz) on the foot at (x, z) puts on the two actuated joints (J^T F)."""
    q = arch.ik(x, z)
    return jac(arch, q).T @ np.asarray(F, float), q


# ---------------------------------------------------------------------- inertia seen by the joints (est.)
def joint_inertia(arch, m_distal, l_distal, act_hip, act_knee):
    """(hip, knee) inertia: actuator rotor reflected through its gear and the belt + link inertia (est.)."""
    jh = ACT[act_hip]["j_rotor"] * ACT[act_hip]["gear"] ** 2 * arch.ratio[0] ** 2
    jk = ACT[act_knee]["j_rotor"] * ACT[act_knee]["gear"] ** 2 * arch.ratio[1] ** 2
    if arch.key == "C":
        return np.array([jh + 0.04 * arch.l1 ** 2, jk + 0.04 * arch.l1 ** 2])
    return np.array([jh + m_distal * 0.20 ** 2 + 0.16 * 0.06 ** 2, jk + m_distal * l_distal ** 2])


# ====================================================================== 2. gait loads
def foot_path(gait, n=240):
    g = GAITS[gait]
    t_st = STRIDE / g["v"]; T = t_st / g["duty"]
    n_st = int(round(n * g["duty"])); n_sw = n - n_st
    s = (np.arange(n_st) + 0.5) / n_st
    s2 = (np.arange(n_sw) + 0.5) / n_sw
    vt = g["v"] * (T - t_st)                     # swing: quintic that leaves and meets the ground at the stance speed
    x = np.r_[STRIDE / 2 - STRIDE * s, -STRIDE / 2 - vt * s2 + (STRIDE + vt) * (10 * s2 ** 3 - 15 * s2 ** 4 + 6 * s2 ** 5)]
    z = np.r_[-H * np.ones(n_st), -H + CLEAR * (1 - np.cos(2 * np.pi * s2)) / 2]
    return dict(x=x, z=z, s=s, n_st=n_st, T=T, dt=T / n, t_st=t_st, t_sw=T - t_st)


def fz_profile(gait, leg, s):
    """M2 vertical load over the stance (s = 0 touchdown, 1 lift-off)."""
    if gait.endswith("trot"):                      # mean = pair load, peak 1.4 x at mid stance
        return F_PAIR[leg] * (1 + (PK["trot"] - 1) * np.cos(2 * np.pi * (s - 0.5)))
    lo, hi = F_STAND[leg], PK["walk"] * F_PAIR[leg]            # walk: the middle third carries the swinging partner's share
    w = np.clip((s - 1 / 3) / 0.06 + 0.5, 0, 1) * np.clip((2 / 3 - s) / 0.06 + 0.5, 0, 1)
    return lo + (hi - lo) * w


def m1_case(arch, leg, gait):
    """First-pass convention (centaur_study.py): vertical force, constant over the stance, peak factor on the stance maximum."""
    xs = np.linspace(STRIDE / 2, -STRIDE / 2, 41)
    F = F_PAIR[leg]
    t1 = np.array([np.abs(support_torque(arch, x, -H, (0, F))[0]) for x in xs])
    duty = GAITS[gait]["duty"]
    if gait == "walk":
        rms = np.sqrt(duty * (np.mean(t1 ** 2, 0) / 3 + 2 * np.mean((t1 / 2) ** 2, 0) / 3))
    else:
        rms = np.sqrt(duty * np.mean(t1 ** 2, 0))
    return dict(rms=rms, peak=PK[gait] * t1.max(0))


def m2_cycle(arch, leg, gait, inertia, spring=None, extra_fx=0.0):
    """Realistic convention: load shaped over the stance, force line aimed at a point Z_COM above the ground over the hip,
    swing inertia added.  spring(q_knee) = parallel extension torque on the knee.  extra_fx: horizontal force as a
    fraction of the vertical load (push-off or slope)."""
    p = foot_path(gait)
    n = len(p["x"]); n_st = p["n_st"]
    q = np.array([arch.ik(x, z) for x, z in zip(p["x"], p["z"])])
    qd = (np.roll(q, -1, 0) - np.roll(q, 1, 0)) / (2 * p["dt"])
    qdd = (np.roll(q, -1, 0) - 2 * q + np.roll(q, 1, 0)) / p["dt"] ** 2
    fz = np.zeros(n); fz[:n_st] = fz_profile(gait, leg, p["s"])
    fx = -fz * p["x"] / Z_COM + extra_fx * fz
    sup = np.array([jac(arch, q[i]).T @ np.array([fx[i], fz[i]]) for i in range(n)])
    tau = inertia * qdd - sup                                   # actuator torque in +q
    if spring is not None:
        tau[:, 1] += np.array([spring(a) for a in q[:, 1]])
    return dict(p=p, q=q, qd=qd, tau=tau, sup=sup, fz=fz, fx=fx, rms=np.sqrt(np.mean(tau ** 2, 0)), peak=np.abs(tau).max(0),
                speed=np.abs(qd).max(0), mu_need=np.max(np.abs(fx[:n_st]) / fz[:n_st]))


def stand_torque(arch, leg, h=H, x=0.0):
    return np.abs(support_torque(arch, x, -h, (0, F_STAND[leg]))[0])


def getup(arch, leg, h0, n=60):
    hs = np.linspace(h0, H, n)
    t = np.array([np.abs(support_torque(arch, 0.0, -h, (0, F_STAND[leg]))[0]) for h in hs])
    return dict(h=hs, tau=t, peak=t.max(0), work=F_STAND[leg] * (H - h0))


def landing(arch, leg):
    """2.5 g on four feet (stand pose) and 2.5 g on a diagonal pair (worst point of the stance), vertical force."""
    four = np.abs(support_torque(arch, 0.0, -H, (0, LAND_G * F_STAND[leg]))[0])
    xs = np.linspace(STRIDE / 2, -STRIDE / 2, 41)
    pair = np.array([np.abs(support_torque(arch, x, -H, (0, LAND_G * F_PAIR[leg]))[0]) for x in xs])
    mid = np.abs(support_torque(arch, 0.0, -H, (0, LAND_G * F_PAIR[leg]))[0])
    return dict(four=four, pair_mid=mid, pair_worst=pair.max(0))


def eff_mass_z(arch, inertia, x=0.0, z=-H):
    """Mass the ground feels when it pushes the foot up (joint inertias only, est.)."""
    J = jac(arch, arch.ik(x, z))
    lam_inv = J @ np.diag(1 / inertia) @ J.T
    return 1.0 / lam_inv[1, 1]


# ====================================================================== 3. the design: recommended leg (B) and the alternatives
PITCH, Z1, Z2, BELT_TEETH, BELT_W = 0.005, 30, 60, 105, 0.025       # v2: HTD 5M PU/steel belt 25 wide, 30T : 60T, 525 mm loop


def pd(z):
    return z * PITCH / np.pi


def belt_len(c, z1, z2):
    d, D = pd(z1), pd(z2)
    return 2 * c + np.pi * (D + d) / 2 + (D - d) ** 2 / (4 * c)


def belt_centre(teeth, z1, z2):
    lo, hi = 0.03, 0.5
    for _ in range(60):
        m = (lo + hi) / 2
        lo, hi = (m, hi) if belt_len(m, z1, z2) < teeth * PITCH else (lo, m)
    return lo


def solve_l3(l1, l2, delta, lmax=LEG_MAX):
    a = lmax - l2; b = 2 * l1 * np.cos(delta)
    return (-b + np.sqrt(b * b - 4 * (l1 ** 2 - a ** 2))) / 2


L1 = belt_centre(BELT_TEETH, Z1, Z2)        # femur = belt centre distance
L2 = 0.210                                  # tibia (v2) (and the two push rods)
DELTA = -10 * D2R                           # cannon line set 10 deg behind the femur direction
L3 = solve_l3(L1, L2, DELTA)                # hock to ground contact
LP, BETA_NOM = 0.045, 40 * D2R              # pastern (fetlock pin to contact), its angle ahead of the cannon tube at trot load
LC = (-2 * LP * np.cos(BETA_NOM) + np.sqrt((2 * LP * np.cos(BETA_NOM)) ** 2 - 4 * (LP ** 2 - L3 ** 2))) / 2     # cannon tube, hock to fetlock
OFF = np.arctan2(LP * np.sin(BETA_NOM), LC + LP * np.cos(BETA_NOM))
R_HUB, R_TUBE, R_HOCK = 0.043, 0.011, 0.015     # hip / stifle housing radius, carbon tube radius, hock lug radius
R_HORN, MU_LOCK = 0.032, 18 * D2R               # push-rod horn radius; horn angle from the tibia when the leg is straight
EPS_LOCK = 4 * D2R                              # over-centre angle of the stay stop
H_KNEEL = 0.190
Z_ABD = 0.029                                   # abduction axis above the hip-pitch axis
ARCH = {"A": TwoLink(0.258, 0.258), "B": Panto(L1, L2, L3, DELTA), "C": FiveBar(0.176, 0.346)}
# distal mass and its distance from the knee (est.), for the swing inertia
DISTAL = {"A": (0.16, 0.10), "B": (0.20, 0.11), "C": (0.0, 0.0)}


def inertia_of(key, act_hip="CyberGear", act_knee="CyberGear"):
    return joint_inertia(ARCH[key], *DISTAL[key], act_hip, act_knee)


def panto_points(B, q):
    hip, st, hk, ft = B.pts(q)
    th2, th3 = q[0] - q[1], q[0] + B.delta
    tube = th3 - OFF
    mu = MU_LOCK + B.qv(q)
    hd = u(th2 + mu)
    return dict(hip=hip, stifle=st, hock=hk, fet=hk + LC * u(tube), foot=ft, P=st + R_HORN * hd, Q=hk + R_HORN * hd, mu=mu,
                th1=q[0], th2=th2, th3=th3, tube=tube, psi_s=np.pi - q[1], psi_h=np.pi - q[1] - B.delta)


def rod_force(B, x, z, F):
    """Push-rod pair force (compression +) and hock moment for a ground force F on the foot at (x, z)."""
    q = B.ik(x, z); P = panto_points(B, q)
    r = P["foot"] - P["hock"]
    mh = r[0] * F[1] - r[1] * F[0]
    return mh / (R_HORN * np.sin(P["mu"])), mh, P


def fold_limit(arch):
    """Deepest fold: the lower link's tube must clear the hip housing."""
    if arch.key == "A":
        psi = np.arcsin((R_HUB + R_TUBE + 0.003) / arch.la)
        r_fold = np.sqrt(arch.la ** 2 + arch.lb ** 2 - 2 * arch.la * arch.lb * np.cos(psi))
        return max(r_fold, 2 * (R_HUB + 0.005)), psi           # the knee pulley housing must also stay off the ground
    if arch.key == "B":
        psi = np.arcsin((R_HUB + R_TUBE + 0.003) / arch.l1)
        qv_max = np.pi - psi + arch.dp
        return np.sqrt(arch.A ** 2 + arch.l2 ** 2 - 2 * arch.A * arch.l2 * np.cos(np.pi - qv_max)), psi
    return arch.l2 - arch.l1 + 0.010, 0.0


def kneel_envelope(arch, h=H_KNEEL):
    """Fore-aft reach of the folded leg (joint housings included) and the highest point, relative to the hip axis."""
    q = arch.ik(0.0, -h)
    if arch.key == "A":
        k = arch.pts(q)[1]
        return dict(x_min=-R_HUB, x_max=k[0] + R_HUB, z_top=R_HUB, low=h + k[1] - R_HUB)
    if arch.key == "B":
        P = panto_points(arch, q)
        return dict(x_min=min(P["hock"][0] - R_HOCK, -R_HUB), x_max=P["stifle"][0] + R_HUB, z_top=R_HUB,
                    low=min(h + P["stifle"][1] - R_HUB, h + P["hock"][1] - R_HOCK, h + P["P"][1] - 0.006, h + P["Q"][1] - 0.006))
    p = arch.pts(q)
    return dict(x_min=p[3][0] - 0.015, x_max=p[1][0] + 0.015, z_top=max(p[1][1], p[3][1]) + 0.015, low=h + min(p[1][1], p[3][1]) - 0.015)


def compare_arch():
    out = {}
    for key, arch in ARCH.items():
        inr = inertia_of(key)
        o = dict(name=arch.name, inertia=inr)
        for leg in ("front", "rear"):
            d = {}
            for gait in ("walk", "trot"):
                m1 = m1_case(arch, leg, gait); m2 = m2_cycle(arch, leg, gait, inr)
                d[gait] = dict(m1_rms=m1["rms"], m1_peak=m1["peak"], m2_rms=m2["rms"], m2_peak=m2["peak"], speed=m2["speed"], mu=m2["mu_need"])
            d["stand"] = stand_torque(arch, leg)
            d["getup"] = getup(arch, leg, H_KNEEL)["peak"]
            d["landing"] = landing(arch, leg)
            o[leg] = d
        o["m_eff"] = eff_mass_z(arch, inr)
        o["h_min"], _ = fold_limit(arch)
        o["env"] = kneel_envelope(arch)
        span = {"A": 0.190, "B": L1}.get(key)
        if span:
            o["belt_teeth"] = int(round(belt_len(span, Z1, Z2) / PITCH))
            o["belt_k"] = BELT_MOD * BELT_W * (pd(Z2) / 2) ** 2 / span          # N m/rad at the knee, one tight span
            o["longest_part"] = span + 2 * R_HUB
        else:
            o["belt_teeth"], o["belt_k"], o["longest_part"] = 0, float("inf"), arch.l2 + 0.03
        out[key] = o
    # proof that B is a 2-link in disguise: same torques as a 2-link with the virtual thigh
    B = ARCH["B"]; V = TwoLink(B.A, B.l2)
    xs = np.linspace(-0.07, 0.07, 15)
    out["B_equals_virtual"] = float(max(np.abs(np.abs(support_torque(B, x, -H, (20.0, 150.0))[0]) - np.abs(support_torque(V, x, -H, (20.0, 150.0))[0])).max() for x in xs))
    return out


def geometry_scan():
    """Why L2 = 155 and delta = -10 deg: torque hardly moves; kneel depth, cannon-tube length and its tilt decide."""
    rows = []
    for dl in (0, -10, -20, -30):
        for l2 in (0.145, 0.155, 0.165, 0.180):
            l3 = solve_l3(L1, l2, dl * D2R)
            lc = (-2 * LP * np.cos(BETA_NOM) + np.sqrt((2 * LP * np.cos(BETA_NOM)) ** 2 - 4 * (LP ** 2 - l3 ** 2))) / 2
            off = np.arctan2(LP * np.sin(BETA_NOM), lc + LP * np.cos(BETA_NOM))
            b = Panto(L1, l2, l3, dl * D2R); q = b.ik(0, -H)
            rows.append(dict(delta=dl, l2=l2, l3=l3, lc=lc, tube=(q[0] + dl * D2R - off) * R2D, h_min=fold_limit(b)[0],
                             knee_peak=m2_cycle(b, "front", "trot", inertia_of("B"))["peak"][1],
                             getup=getup(b, "front", max(fold_limit(b)[0], H_KNEEL))["peak"][1]))
    return rows


# ====================================================================== 4. sprung fetlock (suspensory ligament)
FET = dict(rs=0.020, b0=28 * D2R, b_design=55 * D2R, b_stop=58 * D2R, f0=35.0, wire={"front": 0.0032, "rear": 0.0026}, od=0.0185, v_td=0.40,
           k_rigid=3.0e5, m_unsprung=0.045, dyneema_break=3500.0)


def fetlock(leg="front"):
    B = ARCH["B"]; f = FET
    g = panto_points(B, B.ik(0.0, -H))["tube"]                 # cannon tube tilt at mid stance
    scale = F_PAIR[leg] / F_PAIR["front"]                      # rear legs get a softer spring, same geometry
    arm = lambda b: LP * np.sin(g + b)                         # lever of a vertical ground force about the fetlock pin
    t0 = f["f0"] * scale * arm(f["b0"]) / f["rs"]
    td = PK["trot"] * F_PAIR[leg] * arm(f["b_design"]) / f["rs"]
    k = (td - t0) / (f["rs"] * (f["b_design"] - f["b0"]))      # line spring rate N/m
    b = np.linspace(f["b0"], f["b_stop"], 600)
    T = t0 + k * f["rs"] * (b - f["b0"])
    F = T * f["rs"] / arm(b)
    dz = LP * (np.cos(g + f["b0"]) - np.cos(g + b))
    E = np.concatenate([[0], np.cumsum(0.5 * (T[1:] + T[:-1]) * f["rs"] * np.diff(b))])
    at = lambda Fq: dict(F=Fq, beta=float(np.interp(Fq, F, b)) * R2D, dz=float(np.interp(Fq, F, dz)), E=float(np.interp(Fq, F, E)),
                         T=float(np.interp(Fq, F, T)))
    kz = np.gradient(F, dz)
    # spring: music wire d, mean coil D (fits the 20 mm bore of the cannon tube)
    d = f["wire"][leg]; D = f["od"] - d; c = D / d
    kw = (4 * c - 1) / (4 * c - 4) + 0.615 / c
    n_act = 79e9 * d ** 4 / (8 * D ** 3 * k)
    stroke = f["rs"] * (f["b_stop"] - f["b0"])
    m_eff = eff_mass_z(B, inertia_of("B"))
    spike_rigid = f["v_td"] * np.sqrt(f["k_rigid"] * m_eff)
    spike_fet = max(F[0] + f["v_td"] * np.sqrt(kz[0] * m_eff), f["v_td"] * np.sqrt(f["k_rigid"] * f["m_unsprung"]))
    lever = stand_torque(B, leg)[1] / F_STAND[leg]
    return dict(tube_tilt=g * R2D, k_line=k, t0=t0, t_stop=float(T[-1]), F=F, dz=dz, b=b, E=E, monotonic=bool(np.all(np.diff(F) > 0)),
                stand=at(F_STAND[leg]), trot=at(F_PAIR[leg]), peak=at(PK["trot"] * F_PAIR[leg]), stop=dict(F=float(F[-1]), dz=float(dz[-1]), E=float(E[-1])),
                kz_first=float(kz[0]), kz_peak=float(np.interp(PK["trot"] * F_PAIR[leg], F, kz)),
                spring=dict(d=d, D=D, n=n_act, tau_peak=8 * at(PK["trot"] * F_PAIR[leg])["T"] * D * kw / (np.pi * d ** 3),
                            tau_stop=8 * T[-1] * D * kw / (np.pi * d ** 3), stroke=stroke, solid=(n_act + 2) * d,
                            free=(n_act + 2) * d + stroke + t0 / k + 0.003, mass=np.pi * D * (n_act + 2) * np.pi * d ** 2 / 4 * 7850),
                line_sf=f["dyneema_break"] / T[-1], m_eff=m_eff, spike_rigid=spike_rigid, spike_fet=spike_fet,
                spike_torque_rigid=spike_rigid * lever, spike_torque_fet=spike_fet * lever,
                pin_load=float(F[-1] + T[-1]))


# ====================================================================== 5. stay apparatus
def copper(tau_joint, act, ratio=1.0):
    """Copper loss (W) of one actuator holding tau_joint: 3 I^2 R_phase = 1.5 I^2 R_line, I = torque / Kt."""
    a = ACT[act]; eff = BELT_EFF if ratio > 1 else 1.0
    return 1.5 * a["r_ll"] * (np.abs(tau_joint) / (ratio * eff) / a["kt"]) ** 2


def lock_pose(B, eps=EPS_LOCK):
    """Leg at full length and eps past it (stifle on its stop), foot under the hip."""
    qs = -eps - B.dp
    f = B.foot(np.array([0.0, qs])); th1 = -np.arctan2(f[0], -f[1])
    q = np.array([th1, qs])
    return q, np.hypot(*f)


def stay(act_knee, act_abd, knee_ratio=2.0):
    B = ARCH["B"]; out = {}
    rows = {}
    for label, h in (("walking height (95 %)", H), ("tall stand (98 %)", 0.98 * LEG_MAX)):
        kn = {leg: stand_torque(B, leg, h)[1] for leg in ("front", "rear")}
        rows[label] = dict(h=h, knee=kn, p_knee=2 * sum(copper(kn[l], act_knee, knee_ratio) for l in kn))
    q, r = lock_pose(B)
    hold = {leg: (jac(B, q).T @ np.array([0, F_STAND[leg]]))[1] for leg in ("front", "rear")}      # negative = pushes onto the stop
    rows["over-centre stop (S1)"] = dict(h=r, knee=hold, p_knee=0.0)
    rows["patella pawl at 95 % (S2)"] = dict(h=H, knee={l: stand_torque(B, l)[1] for l in ("front", "rear")}, p_knee=0.0)
    # the offload cord (section 6: front 3.5 N m + 0.8 N m/rad, rear 2.0 + 0.5) balances most of the standing load on its own
    q_st = B.ik(0, -H)[1]; cord = {"front": (3.5, 0.8), "rear": (2.0, 0.5)}
    res = {l: stand_torque(B, l)[1] - cord[l][0] for l in cord}
    rows["offload cord only, walking height"] = dict(h=H, knee=res, p_knee=2 * sum(copper(res[l], act_knee, knee_ratio) for l in res))
    out["cord_at_lock"] = {l: cord[l][0] + cord[l][1] * (q[1] - q_st) for l in cord}                # the cord also presses the knee onto its stop
    out["rows"] = rows
    out["standby"] = 12 * ACT[act_knee]["standby_w"]
    out["abd_15mm"] = 2 * sum(copper(F_STAND[l] * 0.015, act_abd) for l in ("front", "rear"))      # what a 15 mm foot offset would cost
    out["hip_trickle"] = 8 * copper(0.3, act_abd)                                                 # 8 hip + abduction joints holding 0.3 N m (est.)
    out["sway_stiffness"] = M_TOT * G * r                                                         # N m/rad the hips must supply (inverted pendulum on locked legs)
    mag = 0.35                                                                                    # N m, patella magnet at 35 mm (est.)
    out["magnet"] = mag
    out["unlock_force"] = {l: (abs(hold[l]) + mag) / B.l2 for l in hold}                          # sideways force on the foot that would flex the knee
    out["unlock_force_cord"] = {l: (abs(hold[l]) + mag + out["cord_at_lock"][l]) / B.l2 for l in hold}
    out["release_torque_cord"] = abs(hold["front"]) + mag + out["cord_at_lock"]["front"]
    out["stop_force"] = {l: abs(hold[l]) / 0.035 for l in hold}
    out["stop_force_land"] = abs((jac(B, q).T @ np.array([0, LAND_G * F_PAIR["front"]]))[1]) / 0.035
    out["release_torque"] = abs(hold["front"]) + mag
    out["lift"] = LEG_MAX - r
    # sawhorse park (S2 + stops): legs raked 20 deg out, knees held by the pawl, hips on end stops
    rake = 20 * D2R
    out["park"] = dict(rake=20, h=LEG_MAX * np.cos(rake), hip_stop={l: F_STAND[l] * LEG_MAX * np.sin(rake) for l in ("front", "rear")},
                       knee={l: F_STAND[l] * B.l2 * np.sin(rake) for l in ("front", "rear")}, shove_g=np.tan(rake))
    out["pawl_force"] = {l: stand_torque(B, l)[1] / 0.036 for l in ("front", "rear")}
    return out


# ====================================================================== 6. gravity offload spring on the stifle
def offload(act="CyberGear"):
    B = ARCH["B"]; inr = inertia_of("B")
    q_stand = B.ik(0, -H)[1]
    res = {}

    def run(tau0, k):
        sp = (lambda a: max(tau0 + k * (a - q_stand), 0.0)) if tau0 > 0 else None
        o = {}
        for leg in ("front", "rear"):
            for gait in ("walk", "trot"):
                c = m2_cycle(B, leg, gait, inr, spring=sp)
                n_st = c["p"]["n_st"]
                o[(leg, gait)] = dict(rms=c["rms"][1], peak=c["peak"][1], swing=np.abs(c["tau"][n_st:, 1]).max())
            s = stand_torque(B, leg)[1]; q_k = B.ik(0, -H_KNEEL)[1]
            o[(leg, "stand")] = abs(s - (sp(q_stand) if sp else 0))
            gu = getup(B, leg, H_KNEEL)
            o[(leg, "getup")] = max(abs(t - (sp(B.ik(0, -h)[1]) if sp else 0)) for h, t in zip(gu["h"], gu["tau"][:, 1]))
            o[(leg, "kneel_spring")] = sp(q_k) if sp else 0.0
        return o
    base = run(0.0, 0.0)
    best, best_j = None, 1e9
    for tau0 in np.arange(1.0, 7.01, 0.5):
        for k in (0.0, 0.5, 1.0, 1.5, 2.0):
            o = run(tau0, k)
            j = o[("front", "walk")]["rms"] ** 2 + o[("front", "trot")]["rms"] ** 2
            if j < best_j:
                best, best_j = (tau0, k, o), j
    # the practical pick: two strands of 6 mm shock cord on a 30 mm sector for the front legs, one strand for the rear (est.)
    pick = {"front": (3.5, 0.8), "rear": (2.0, 0.5)}
    picked = {leg: run(*pick[leg]) for leg in pick}
    horse_alone = {"front": 8.6 * G * (1 - 0.250 / 0.440) / 2, "rear": 8.6 * G * (0.250 / 0.440) / 2}
    res.update(base=base, best=dict(tau0=best[0], k=best[1], o=best[2]), pick=pick, picked=picked, q_stand=q_stand,
               alone={l: abs(horse_alone[l] * stand_torque(B, l)[1] / F_STAND[l] - pick[l][0]) for l in pick},
               cord_force={l: pick[l][0] / 0.030 for l in pick})
    return res


# ====================================================================== 7. actuators, belt, bearings
SETS = {
    "R36 RS03 front hips, RS06 elsewhere, 12S": dict(abd="RobStride 06", hip={"front": "RobStride 03", "rear": "RobStride 06"}, knee="RobStride 06", volts=44.4),
    "R6 RS06 hips + knees, RS02 abduction, 12S": dict(abd="RobStride 02", hip={"front": "RobStride 06", "rear": "RobStride 06"}, knee="RobStride 06", volts=44.4),
    "R  12 x RobStride 02, 12S bus": dict(abd="RobStride 02", hip={"front": "RobStride 02", "rear": "RobStride 02"}, knee="RobStride 02", volts=44.4),
    "G  12 x CyberGear, 6S bus": dict(abd="CyberGear", hip={"front": "CyberGear", "rear": "CyberGear"}, knee="CyberGear", volts=22.2),
    "GK CyberGear + AK70-10 front hips, 6S": dict(abd="CyberGear", hip={"front": "AK70-10", "rear": "CyberGear"}, knee="CyberGear", volts=22.2),
}
KNEE_RATIO = Z2 / Z1


def joint_cases(leg, spring=None):
    """rms and peak joint torque for every load case, recommended leg.  Returns {case: {joint: (rms, peak)}}."""
    B = ARCH["B"]; inr = inertia_of("B"); z_a = H + Z_ABD
    c = {}
    st = stand_torque(B, leg)
    c["stand"] = dict(hip=(st[0], st[0]), knee=(st[1], st[1]), abd=(0.0, 0.0))
    for gait in ("walk", "trot"):
        m1 = m1_case(B, leg, gait); m2 = m2_cycle(B, leg, gait, inr, spring=spring)
        fr = np.sqrt(np.mean(m2["fz"] ** 2)); fp = m2["fz"].max()
        duty = GAITS[gait]["duty"]
        c[gait + " M1"] = dict(hip=(m1["rms"][0], m1["peak"][0]), knee=(m1["rms"][1], m1["peak"][1]),
                               abd=(np.sqrt(duty) * F_PAIR[leg] * LAT * z_a, PK[gait] * F_PAIR[leg] * LAT * z_a))
        c[gait + " M2"] = dict(hip=(m2["rms"][0], m2["peak"][0]), knee=(m2["rms"][1], m2["peak"][1]), abd=(fr * LAT * z_a, fp * LAT * z_a))
    gu = getup(B, leg, H_KNEEL)["peak"]
    c["get up"] = dict(hip=(0.0, gu[0]), knee=(0.0, gu[1]), abd=(0.0, 0.0))
    ld = landing(B, leg)
    c["landing 2.5 g, 4 feet"] = dict(hip=(0.0, ld["four"][0]), knee=(0.0, ld["four"][1]), abd=(0.0, LAND_G * F_STAND[leg] * LAT * z_a))
    c["landing 2.5 g, pair"] = dict(hip=(0.0, ld["pair_mid"][0]), knee=(0.0, ld["pair_mid"][1]), abd=(0.0, LAND_G * F_PAIR[leg] * LAT * z_a))
    push = m2_cycle(B, leg, "trot", inr, extra_fx=ACC_G)
    c["trot + 0.25 g push"] = dict(hip=(0.0, push["peak"][0]), knee=(0.0, push["peak"][1]),
                                   abd=(0.0, ACC_G * F_PAIR[leg] * z_a))        # abd column: a 0.25 g side shove on one pair
    sl = m2_cycle(B, leg, "walk", inr, extra_fx=np.sin(SLOPE_DEG * D2R))
    c["walk up 10 deg"] = dict(hip=(sl["rms"][0], sl["peak"][0]), knee=(sl["rms"][1], sl["peak"][1]), abd=(0.0, 0.0))
    return c


def margins(setname, spring=None):
    s = SETS[setname]; out = {}
    for leg in ("front", "rear"):
        rows = {}
        for case, jt in joint_cases(leg, spring).items():
            r = {}
            for j, (rms, pk) in jt.items():
                act = s["hip"][leg] if j == "hip" else s[j]
                ratio = KNEE_RATIO if j == "knee" else 1.0
                eff = BELT_EFF if ratio > 1 else 1.0
                a = ACT[act]
                rated, limit = a["rated"] * ratio * eff, a["peak"] * ratio * eff
                r[j] = dict(rms=rms, peak=pk, rated=rated, limit=limit, m_rms=(rated / rms if rms > 1e-6 else 99.0), m_peak=(limit / pk if pk > 1e-6 else 99.0),
                            i_rms=(rms / ratio / eff / a["kt"] if a["kt"] else None), i_peak=(pk / ratio / eff / a["kt"] if a["kt"] else None),
                            p_cu=(copper(rms, act, ratio) if a["kt"] else None))
            rows[case] = r
        out[leg] = rows
    return out


def speeds(setname):
    s = SETS[setname]; B = ARCH["B"]; inr = inertia_of("B"); out = {}
    a = ACT[s["knee"]]; volt_k = min(s["volts"] / (48.0 if a["v_nom"] > 30 else 24.0), 1.0)
    for gait in ("walk", "trot", "fast trot"):
        c = m2_cycle(B, "front", gait, inr)
        out[gait] = dict(knee_dps=c["speed"][1] * R2D, hip_dps=c["speed"][0] * R2D, knee_act_rpm=c["speed"][1] * KNEE_RATIO * 30 / np.pi,
                         hip_act_rpm=c["speed"][0] * 30 / np.pi, t_sw=c["p"]["t_sw"], v=GAITS[gait]["v"])
    out["knee_act_limit_rpm"] = a["w_rated"] * volt_k
    out["knee_joint_limit_dps"] = a["w_rated"] * volt_k / KNEE_RATIO * 6
    return out


def belt_check():
    B = ARCH["B"]; r2 = pd(Z2) / 2
    wrap = np.pi - 2 * np.arcsin((pd(Z2) - pd(Z1)) / (2 * L1))
    mesh = Z1 * wrap / (2 * np.pi)
    ta15 = BELT_TA["5M GT"] * BELT_W; ta25 = BELT_TA["5M GT"] * 0.025; ta_htd = BELT_TA["5M HTD"] * BELT_W
    cases = {}
    jc = joint_cases("front")
    for name in ("stand", "walk M2", "trot M2", "trot M1", "get up", "landing 2.5 g, 4 feet", "landing 2.5 g, pair"):
        tq = jc[name]["knee"][1]; T = tq / r2
        cases[name] = dict(torque=tq, tension=T, m_ta15=ta15 / T, m_ta25=ta25 / T, m_break=BELT_BREAK_15 / T)
    for act in ("CyberGear", "RobStride 02"):
        tq = ACT[act]["peak"] * KNEE_RATIO * BELT_EFF; T = tq / r2
        cases[f"{act} stall"] = dict(torque=tq, tension=T, m_ta15=ta15 / T, m_ta25=ta25 / T, m_break=BELT_BREAK_15 / T)
    rms_T = jc["trot M2"]["knee"][0] / r2
    return dict(pd1=pd(Z1), pd2=pd(Z2), centre=L1, wrap=wrap * R2D, mesh=mesh, ta15=ta15, ta25=ta25, ta_htd15=ta_htd, cases=cases,
                rms_tension=rms_T, m_rms=ta15 / rms_T, pu_tooth=ELATECH_FU * BELT_W * min(mesh, 12), pu_tension=ELATECH_FT15,
                torque_limit_15=ta15 * r2, torque_limit_25=ta25 * r2, k_theta=BELT_MOD * BELT_W * r2 ** 2 / L1,
                ratio_scan={f"{z2}:{Z1}": dict(ratio=z2 / Z1, pd2=pd(z2), tension_trot=jc["trot M2"]["knee"][1] / (pd(z2) / 2)) for z2 in (36, 48, 60)})


def bearing_check():
    B = ARCH["B"]; r2 = pd(Z2) / 2; out = {}
    for name, Fz in (("trot peak", PK["trot"] * F_PAIR["front"]), ("landing 2.5 g, pair", LAND_G * F_PAIR["front"])):
        worst = dict(rod=0.0, stifle=0.0, hock=0.0)
        for x in np.linspace(STRIDE / 2, -STRIDE / 2, 15):
            rod, mh, P = rod_force(B, x, -H, (0, Fz))
            T = abs(support_torque(B, x, -H, (0, Fz))[0][1]) / r2
            worst["rod"] = max(worst["rod"], abs(rod)); worst["hock"] = max(worst["hock"], Fz + abs(rod)); worst["stifle"] = max(worst["stifle"], Fz + abs(rod) + T)
        # abduction cradle: 6810 on the actuator side at x = -52, 6802 journal on the stifle side at x = +62 (hind leg), foot at x = +-70
        xa, xb = -0.052, 0.062
        r_b = Fz * (0.070 - xa) / (xb - xa); r_a = Fz * (xb + 0.070) / (xb - xa)
        out[name] = dict(Fz=Fz, rod=worst["rod"],
                         stifle=dict(load=worst["stifle"] / 2, c0=C0["6805"], sf=C0["6805"] / (worst["stifle"] / 2)),
                         hock=dict(load=worst["hock"] / 2, c0=C0["6800"], sf=C0["6800"] / (worst["hock"] / 2)),
                         hip_stub=dict(load=Fz / 2, c0=C0["6802"], sf=C0["6802"] / (Fz / 2)),
                         abd_6810=dict(load=r_a, c0=C0["6810"], sf=C0["6810"] / r_a), abd_6802=dict(load=r_b, c0=C0["6802"], sf=C0["6802"] / r_b),
                         act_bearing_with_stub=Fz / 2, act_bearing_without=dict(radial=Fz, moment=Fz * 0.040))
    return out


def rod_table():
    B = ARCH["B"]; rows = {}
    q, r = lock_pose(B)
    P = panto_points(B, q); rr = P["foot"] - P["hock"]
    rows["locked stand"] = dict(mu=P["mu"] * R2D, rod=rr[0] * F_STAND["front"] / (R_HORN * np.sin(P["mu"])))
    for name, x, h, F in (("stand", 0, H, F_STAND["front"]), ("trot peak, mid stance", 0, H, PK["trot"] * F_PAIR["front"]),
                          ("trot peak, foot ahead", 0.07, H, PK["trot"] * F_PAIR["front"]), ("trot peak, foot behind", -0.07, H, PK["trot"] * F_PAIR["front"]),
                          ("kneel", 0, H_KNEEL, F_STAND["front"]), ("landing 2.5 g, pair", 0, H, LAND_G * F_PAIR["front"])):
        rod, mh, P = rod_force(B, x, -h, (0, F))
        rows[name] = dict(mu=P["mu"] * R2D, rod=rod, m_hock=mh)
    return rows


# ====================================================================== 8. hip module: three actuators, cradle, abduction axis
FEM_HALF, GAP, CHEEK = 0.015, 0.002, 0.008        # femur housing half width, running gap, cradle cheek thickness
R_ACT = 0.04025                                   # actuator body radius used for every set (CyberGear 80.5; RobStride 78.5, shape not verified)
X_WEB, X_BRG, X_ABD, X_LUG = -0.044, -0.048, -0.058, 0.058      # hind-leg local x of the rear web, the 6810, the abduction actuator face, the front journal


def hip_module(setname, y_leg):
    s = SETS[setname]
    t_h, t_k, t_a = ACT[s["hip"]["rear"]]["t"], ACT[s["knee"]]["t"], ACT[s["abd"]]["t"]
    y_ax = Y_FOOT - y_leg                         # abduction axis in leg-local y (the axis sits over the foot)
    y_in = -(FEM_HALF + GAP + t_h); y_out = FEM_HALF + GAP + t_k
    S = {   # name: (x0, x1, y0, y1, z0, z1) boxes around each solid, leg-local (hind left: x forward, y outboard, z up from the hip axis)
        "hip-pitch actuator": (-R_ACT, R_ACT, y_in, -(FEM_HALF + GAP), -R_ACT, R_ACT),
        "femur housing": (-R_HUB, R_HUB, -FEM_HALF, FEM_HALF, -R_HUB, R_HUB),
        "knee actuator": (-R_ACT, R_ACT, FEM_HALF + GAP, y_out, -R_ACT, R_ACT),
        "stub axle + 6802": (-0.012, 0.012, y_out + GAP, y_out + GAP + 0.005, -0.012, 0.012),
        "inboard cheek": (-0.0435, 0.0435, y_in - CHEEK, y_in, -0.044, 0.0445),
        "outboard cheek": (-0.030, 0.030, y_out + GAP + 0.005, y_out + GAP + 0.011, -0.016, 0.0445),
        "bridge": (X_WEB - 0.004, X_LUG, y_in - CHEEK, y_out + GAP + 0.011, 0.045, 0.053),
        "rear web": (X_WEB - 0.004, X_WEB, y_ax - 0.045, y_ax + 0.045, -0.004, 0.0445),
        "6810 + hub": (X_BRG - 0.008, X_BRG, y_ax - 0.0325, y_ax + 0.0325, Z_ABD - 0.0325, Z_ABD + 0.0325),
        "abduction actuator": (X_ABD - t_a, X_ABD, y_ax - R_ACT, y_ax + R_ACT, Z_ABD - R_ACT, Z_ABD + R_ACT),
        "front journal + 6802": (X_LUG, X_LUG + 0.008, y_ax - 0.012, y_ax + 0.012, Z_ABD - 0.012, Z_ABD + 0.012),
    }
    names = list(S); clashes = []; gap_min = (1.0, None)
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            a, b = S[names[i]], S[names[j]]
            g = max(max(a[2 * k] - b[2 * k + 1], b[2 * k] - a[2 * k + 1]) for k in range(3))     # separation of the two boxes (negative = overlap)
            if g < -1e-9:
                clashes.append((names[i], names[j], g))
            if 0 <= g < gap_min[0]:
                gap_min = (g, (names[i], names[j]))
    camber = np.arcsin(y_ax / (H + Z_ABD))
    # inward abduction until the inboard cheek's lower corner is 3 mm from the body side
    yc, zc = y_in - CHEEK - y_ax, -0.044 - Z_ABD
    a_in = 0.0
    for a in np.linspace(0, 0.6, 601):
        y_w = Y_FOOT + yc * np.cos(a) + zc * np.sin(a)          # rotating the cradle inward (lower parts move toward the body)
        if y_w < BODY["w"] / 2 + 0.003:
            break
        a_in = a
    B = ARCH["B"]
    th_kneel = B.ik(0, -H_KNEEL)[0]
    th_gait = [m2_cycle(B, "front", g, inertia_of("B"))["q"][:, 0] for g in ("walk", "trot")]
    th_lo, th_hi = min(t.min() for t in th_gait), max(t.max() for t in th_gait)
    femur_top = -X_LUG / np.tan(th_kneel) + 0.020 / np.sin(th_kneel)        # top edge of the femur beam under the front journal, at the kneel
    return dict(solids=S, clashes=clashes, gap_min=gap_min, y_leg=y_leg, y_ax=y_ax, camber=camber * R2D, y_inner=y_leg + y_in - CHEEK,
                y_outer=y_leg + y_out + GAP + 0.011, width=2 * (y_leg + y_out + GAP + 0.011), width_no_stub=2 * (y_leg + y_out),
                length=abs(HIP_X["rear"]) + 2 * (-X_ABD + t_a), top=H + Z_ABD + R_ACT, bridge_top=H + 0.053, abd_in=a_in * R2D,
                th_gait=(th_lo * R2D, th_hi * R2D), th_kneel=th_kneel * R2D, th_lock=lock_pose(B)[0][0] * R2D,
                lug_clear=(Z_ABD - 0.012) - femur_top, stifle_range=(lock_pose(B)[0][1] * R2D, B.ik(0, -H_KNEEL)[1] * R2D),
                pod_mass_axis=None)


def kneel_check(mod):
    """Front legs are the hind module mirrored in x.  Kneel with the octopus on the neck: what comes near what."""
    B = ARCH["B"]; out = {}
    for h in (H, H_KNEEL):
        P = panto_points(B, B.ik(0, -h))
        hind = {k: np.array([HIP_X["rear"] + P[k][0], h + P[k][1]]) for k in ("stifle", "hock", "fet", "foot", "P", "Q")}
        front = {k: np.array([HIP_X["front"] - P[k][0], h + P[k][1]]) for k in ("stifle", "hock", "fet", "foot", "P", "Q")}
        out["stand" if h == H else "kneel"] = dict(
            hip=h, belly=h - (H - BODY["z0"]), plate=h + PLATE_Z - H,
            elbow_to_stifle_gap=(front["stifle"][0] - R_HUB) - (hind["stifle"][0] + R_HUB),
            lowest=min(hind["stifle"][1] - R_HUB, hind["hock"][1] - R_HOCK, hind["P"][1] - 0.006, hind["Q"][1] - 0.006),
            stifle_z=hind["stifle"][1], hock_z=hind["hock"][1], front_reach=front["hock"][0] + R_HOCK, rear_reach=hind["hock"][0] - R_HOCK,
            psi_s=P["psi_s"] * R2D, psi_h=P["psi_h"] * R2D, tube=P["tube"] * R2D, th1=P["th1"] * R2D)
    # front pod against the neck collar (r 118 around x = -110, from the body top up)
    corner = np.array([-(X_LUG), mod["y_inner"]])                     # mirrored bridge corner nearest the neck
    out["collar"] = dict(dist=float(np.hypot(corner[0] - NECK_X, corner[1])), r=COLLAR_R, bridge_top=mod["bridge_top"], collar_base=BODY["z1"],
                         overlap_z=max(mod["bridge_top"] - BODY["z1"], 0.0), chamfer=max(COLLAR_R + 0.004 - float(np.hypot(corner[0] - NECK_X, corner[1])), 0.0),
                         pod_to_plate=(PLATE_Z - mod["top"]))
    return out


# ====================================================================== 9. structure
def tube_sec(od, idn):
    I = np.pi * (od ** 4 - idn ** 4) / 64
    return dict(I=I, Z=I / (od / 2), A=np.pi * (od ** 2 - idn ** 2) / 4)


def box_sec(b, h, t):
    I = (b * h ** 3 - (b - 2 * t) * (h - 2 * t) ** 3) / 12
    return dict(I=I, Z=I / (h / 2), A=b * h - (b - 2 * t) * (h - 2 * t))


def structure():
    B = ARCH["B"]; out = {}
    jc = joint_cases("front")
    m_trot = max(jc["trot M1"]["knee"][1], jc["trot M2"]["knee"][1]); ld = landing(B, "front")
    m_land = ld["pair_worst"][1]
    tib = tube_sec(0.022, 0.020); fem = box_sec(0.030, 0.040, 0.003)
    rods = rod_table(); bc = bearing_check()
    rod_max = bc["landing 2.5 g, pair"]["rod"]; rod_trot = max(bc["trot peak"]["rod"], abs(rods["kneel"]["rod"]))
    mh_trot = max(abs(rod_force(B, x, -H, (0, PK["trot"] * F_PAIR["front"]))[1]) for x in np.linspace(-0.07, 0.07, 15))
    mh_land = mh_trot * LAND_G / PK["trot"]
    rod1 = tube_sec(0.006, 0.004)
    sock = lambda M, d, L: 6 * M / (d * L ** 2)
    fet = fetlock("front")
    out["tibia tube 22 x 20 carbon, bending at the stifle head"] = dict(load=m_trot, load_land=m_land, stress=m_trot / tib["Z"], stress_land=m_land / tib["Z"], allow=S_CF_TUBE)
    out["tibia tube in its printed socket (25 mm deep), bearing"] = dict(load=m_trot, load_land=m_land, stress=sock(m_trot, 0.022, 0.025), stress_land=sock(m_land, 0.022, 0.025), allow=S_PACF_BRG)
    out["femur box 30 x 40 x 3 printed PA-CF, bending"] = dict(load=m_trot, load_land=max(m_land, ld["pair_worst"][0]), stress=m_trot / fem["Z"], stress_land=max(m_land, ld["pair_worst"][0]) / fem["Z"], allow=S_PACF)
    out["cannon tube 22 x 20 carbon, bending at the hock"] = dict(load=mh_trot, load_land=mh_land, stress=mh_trot / tib["Z"], stress_land=mh_land / tib["Z"], allow=S_CF_TUBE)
    out["push rod 6 x 4 carbon (one of two), axial"] = dict(load=rod_trot / 2, load_land=rod_max / 2, stress=rod_trot / 2 / rod1["A"], stress_land=rod_max / 2 / rod1["A"], allow=S_CF_TUBE,
                                                            buckling=np.pi ** 2 * E_CF * rod1["I"] / L2 ** 2)
    out["rod horn 12 x 20 printed, bending at the root"] = dict(load=rod_trot * R_HORN, load_land=rod_max * R_HORN, stress=rod_trot * R_HORN / (0.012 * 0.020 ** 2 / 6),
                                                              stress_land=rod_max * R_HORN / (0.012 * 0.020 ** 2 / 6), allow=S_PACF)
    hk, hkl = bc["trot peak"]["hock"]["load"] * 2, bc["landing 2.5 g, pair"]["hock"]["load"] * 2
    out["hock pin, 10 mm shoulder bolt, double shear"] = dict(load=hk, load_land=hkl, stress=hk / (2 * np.pi * 0.005 ** 2), stress_land=hkl / (2 * np.pi * 0.005 ** 2), allow=200e6)
    out["hock lug, 2 x 6 mm printed walls on the bearings, bearing"] = dict(load=hk, load_land=hkl, stress=hk / (2 * 0.019 * 0.005), stress_land=hkl / (2 * 0.019 * 0.005), allow=S_PACF_BRG)
    out["fetlock pin, 6 mm shoulder bolt in 2 bushings 6 x 6, bearing"] = dict(load=fet["pin_load"], load_land=fet["pin_load"] + (LAND_G * F_PAIR["front"] - fet["stop"]["F"]),
                                                                              stress=fet["pin_load"] / (2 * 0.006 * 0.006), stress_land=(fet["pin_load"] + LAND_G * F_PAIR["front"] - fet["stop"]["F"]) / (2 * 0.006 * 0.006), allow=S_PACF_BRG)
    pm, pml = PK["trot"] * F_PAIR["front"] * LP * np.sin(51 * D2R), LAND_G * F_PAIR["front"] * LP * np.sin(54 * D2R)
    out["pastern 10 x 16 printed, bending at the fetlock"] = dict(load=pm, load_land=pml, stress=pm / (0.010 * 0.016 ** 2 / 6), stress_land=pml / (0.010 * 0.016 ** 2 / 6), allow=S_PACF)
    r2 = pd(Z2) / 2; mesh = belt_check()["mesh"]
    out[f"printed {Z1}T pulley teeth (3 x {BELT_W * 1e3:.0f} mm root), shear"] = dict(load=m_trot / r2, load_land=m_land / r2, stress=m_trot / r2 / (mesh * 0.003 * 0.015), stress_land=m_land / r2 / (mesh * 0.003 * 0.015), allow=S_PACF / 2)
    hs = stay("CyberGear", "CyberGear")
    out["stifle stay stop, 8 x 12 mm PA-CF face behind a TPU pad"] = dict(load=hs["stop_force"]["front"], load_land=hs["stop_force_land"], stress=hs["stop_force"]["front"] / (0.008 * 0.012),
                                                                          stress_land=hs["stop_force_land"] / (0.008 * 0.012), allow=S_PACF_BRG)
    for v in out.values():
        v["sf"] = v["allow"] / v["stress"]; v["sf_land"] = v["allow"] / v["stress_land"]
    return out


# ====================================================================== 10. parts: mass, print layout, cost (all est. unless a part is bought)
def parts(setname):
    s = SETS[setname]; t_k = ACT[s["knee"]]["t"]
    fet = fetlock("front")
    tube = lambda L: 1600 * tube_sec(0.022, 0.020)["A"] * L
    P = [   # name, material, qty, mass each (kg), bounding box (mm), print orientation / note, cost each (USD), only_B
        ("cradle bridge + rear web + hub", "PA-CF", 1, 0.095, (150, 160, 62), "web on the bed, axis vertical", 0, False),
        ("cradle cheek, inboard (actuator bolt circle)", "PA-CF", 1, 0.030, (100, 100, 8), "flat", 0, False),
        ("cradle cheek, outboard (6802 seat)", "PA-CF", 1, 0.016, (70, 70, 11), "flat", 0, False),
        ("femur shell half (split on the leg mid-plane)", "PA-CF", 2, 0.050, (195, 88, 15), "split face down, flat", 0, False),
        ("stirrup around the knee actuator + stub axle", "PA-CF", 1, 0.030, (100, 100, int(t_k * 1e3) + 12), "open side up", 0, False),
        (f"drive pulley {Z1}T 5M with flanges", "PA-CF or CF resin", 1, 0.022, (56, 56, 30), "axis vertical", 0, False),
        (f"tibia head: {Z2}T pulley, stifle bearing seats, tube socket", "PA-CF", 1, 0.085, (106, 120, 44), "pulley axis vertical", 0, False),
        ("stifle block (slides 3 mm on the femur for belt tension)", "PA-CF", 2, 0.012, (50, 44, 12), "flat", 0, False),
        ("tibia hock fork (2 x 6800 seats, tube socket)", "PA-CF", 1, 0.016, (48, 34, 40), "fork up", 0, True),
        ("cannon head with rod horn (hock pin, tube socket)", "PA-CF", 1, 0.018, (62, 46, 30), "side down", 0, True),
        ("fetlock lug (pin, line guide, bump-stop seat)", "PA-CF", 1, 0.011, (36, 28, 30), "side down", 0, False),
        ("pastern with sesamoid horn", "PA-CF", 1, 0.012, (64, 30, 22), "side down", 0, False),
        ("hoof core", "PA-CF", 1, 0.008, (40, 36, 20), "sole down", 0, False),
        ("hoof shoe (replaceable sole) + ergot bump stop + stop pads", "TPU 95A", 1, 0.017, (46, 42, 16), "sole down", 0, False),
        (f"tibia tube 22 x 20 carbon, {(L2 - 0.030) * 1e3:.0f} mm", "carbon tube", 1, tube(L2 - 0.030), (round((L2 - 0.030) * 1e3), 22, 22), "cut to length", 4.0, False),
        (f"cannon tube 22 x 20 carbon, {(LC - 0.005) * 1e3:.0f} mm (spring inside)", "carbon tube", 1, tube(LC - 0.005), (round((LC - 0.005) * 1e3), 22, 22), "cut to length", 2.0, False),
        (f"push rod 6 x 4 carbon, {(L2 - 0.020) * 1e3:.0f} mm + 2 M3 ball links", "carbon tube", 2, (0.0034 + 0.004) * L2 / 0.155, (round(L2 * 1e3), 8, 8), "cut to length", 3.0, True),
        (f"belt HTD 5M, {BELT_W * 1e3:.0f} wide, {BELT_TEETH * 5} long ({BELT_TEETH} teeth), PU with steel cords", "bought", 1, 0.045, (0, 0, 0), f"Megadyne {BELT_TEETH * 5}-5M-{BELT_W * 1e3:.0f} (thermoset PU, zinc-plated steel cord)", 9.0, False),
        ("bearing 6805-2RS (stifle)", "bought", 2, BRG_M["6805"], (0, 0, 0), "25 x 37 x 7", 3.0, False),
        ("bearing 6800-2RS (hock)", "bought", 2, BRG_M["6800"], (0, 0, 0), "10 x 19 x 5", 2.0, True),
        ("bearing 6802-2RS (hip stub, abduction journal)", "bought", 2, BRG_M["6802"], (0, 0, 0), "15 x 24 x 5", 2.5, False),
        ("bearing 6810-2RS (abduction hub)", "bought", 1, BRG_M["6810"], (0, 0, 0), "50 x 65 x 7", 7.0, False),
        ("shoulder bolt 10 x 40, M8 (hock)", "bought", 1, 0.028, (0, 0, 0), "ISO 7379", 2.5, True),
        ("shoulder bolt 6 x 25, M5 + 2 flanged bushings 6 x 8 x 6 (fetlock)", "bought", 1, 0.008, (0, 0, 0), "ISO 7379; igus GFM-0608-06 or bronze", 3.0, False),
        ("stifle through bolt M5 x 45 + nut", "bought", 1, 0.010, (0, 0, 0), "clamps the two stifle blocks", 0.5, False),
        ("fetlock spring, 3.2 wire, 18.5 OD (front) / 2.6 wire (rear) + plunger", "bought", 1, fet["spring"]["mass"] + 0.004, (0, 0, 0), "music wire, see fetlock table", 4.0, False),
        ("Dyneema line 2 mm, 0.3 m (suspensory ligament)", "bought", 1, 0.001, (0, 0, 0), "SK78", 0.5, False),
        ("shock cord 6 mm, 0.6 m (offload spring)", "bought", 1, 0.010, (0, 0, 0), "2 strands front, 1 rear", 1.0, False),
        ("magnet 8 x 3 N42 (patella catch)", "bought", 2, 0.0011, (0, 0, 0), "", 0.5, False),
        ("heat-set inserts M3/M4, bolts, dowels", "bought", 1, 0.055, (0, 0, 0), "about 40 pieces", 8.0, False),
    ]
    printed = sum(q * m for n, mat, q, m, *_ in P if mat.startswith("PA-CF") or mat.startswith("TPU"))
    struct = sum(q * m for n, mat, q, m, *_ in P)
    b_only = sum(q * m for n, mat, q, m, bb, o, c, ob in P if ob); b_only_usd = sum(q * c for n, mat, q, m, bb, o, c, ob in P if ob)
    a_extra = 0.045 + 0.011 + tube(0.065)          # what the 2-link needs instead: 82 mm more thigh shell, 32 more belt teeth, longer shank tube
    acts = [s["abd"], s["hip"]["front"], s["knee"]]
    m_act = {leg: ACT[s["abd"]]["m"] + ACT[s["hip"][leg]]["m"] + ACT[s["knee"]]["m"] for leg in ("front", "rear")}
    usd_parts = sum(q * c for n, mat, q, m, bb, o, c, ob in P) + printed * 60.0
    usd_act = {leg: tuple(sum(ACT[a]["usd"][i] for a in (s["abd"], s["hip"][leg], s["knee"])) for i in (0, 1)) for leg in ("front", "rear")}
    m_leg = {leg: struct + m_act[leg] for leg in m_act}
    m_legs = 2 * m_leg["front"] + 2 * m_leg["rear"]
    fixed = 0.80 + 0.35 + 1.62 + 0.25 + 0.20                  # frame, neck, packs, electronics, wiring (first pass, est.)
    moving = sum(q * m for n, mat, q, m, *_ in P[3:]) - 2 * BRG_M["6802"] / 2 - BRG_M["6810"]       # everything below the cradle
    return dict(list=P, printed=printed, struct=struct, m_act=m_act, m_leg=m_leg, m_legs=m_legs, m_horse=m_legs + fixed, m_centaur=m_legs + fixed + 12.81,
                growth=(m_legs + fixed + 12.81) / M_TOT, b_minus_a=b_only - a_extra, b_minus_a_usd=b_only_usd - 3.0, usd_parts=usd_parts, usd_act=usd_act,
                usd_leg={leg: (usd_parts + usd_act[leg][0], usd_parts + usd_act[leg][1]) for leg in usd_act},
                distal=sum(q * m for n, mat, q, m, *_ in P[6:17] if "block" not in n), unsprung=0.012 + 0.008 + 0.017 + 0.008)


# ====================================================================== 11. sketches
COL = sk3d.COL
REC = "R36 RS03 front hips, RS06 elsewhere, 12S"            # recommended actuator set
Y_LEG = {"R36 RS03 front hips, RS06 elsewhere, 12S": 0.200, "R6 RS06 hips + knees, RS02 abduction, 12S": 0.185, "R  12 x RobStride 02, 12S bus": 0.160, "G  12 x CyberGear, 6S bus": 0.150, "GK CyberGear + AK70-10 front hips, 6S": 0.150}


def w3(p2, hipx, hipz, y, sx=1.0):
    return np.array([hipx + sx * p2[0], y, hipz + p2[1]])


def draw_pod(sc, hipx, hipz, side, sx, setname=REC, alpha=1.0, cradle=True):
    s = SETS[setname]; t = ACT[s["knee"]]["t"]; yl = side * Y_LEG[setname]
    c = np.array([hipx, yl, hipz]); ey = np.array([0, side, 0.0]); ex = np.array([sx, 0, 0.0])
    sc.cyl(c - ey * (FEM_HALF + GAP + t), c - ey * (FEM_HALF + GAP), R_ACT, COL["act"], alpha)            # hip pitch
    sc.cyl(c - ey * FEM_HALF, c + ey * FEM_HALF, R_HUB, COL["horse"], alpha)                              # femur housing
    sc.cyl(c + ey * (FEM_HALF + GAP), c + ey * (FEM_HALF + GAP + t), R_ACT, COL["act"], alpha)            # knee
    if cradle:
        a = np.array([hipx, side * Y_FOOT, hipz + Z_ABD])
        sc.cyl(a + ex * X_ABD, a + ex * (X_ABD - t), R_ACT, COL["act"], alpha)                            # abduction
        sc.cyl(a + ex * X_BRG, a + ex * (X_BRG - 0.008), 0.0325, COL["frame"], alpha)
        sc.cyl(a + ex * X_LUG, a + ex * (X_LUG + 0.008), 0.012, COL["frame"], alpha)
        yc = yl + side * (GAP + 0.011) / 2
        sc.box((hipx + sx * (X_LUG + X_WEB - 0.004) / 2, yc, hipz + 0.049), (X_LUG - X_WEB + 0.004, 2 * (FEM_HALF + GAP + t) + CHEEK + GAP + 0.011, 0.008),
               fc=COL["frame"], alpha=alpha, sub=1)


def draw_leg(sc, key, q, hipx, hipz, y, sx=1.0, alpha=1.0, side=1.0):
    arch = ARCH[key]; P3 = lambda p: w3(p, hipx, hipz, y, sx)
    ey = np.array([0, side, 0.0])
    if key == "A":
        o, k, f = arch.pts(q)
        sc.tube([P3(o), P3(k)], 0.016, COL["horse"], alpha); sc.cyl(P3(k) - ey * 0.012, P3(k) + ey * 0.012, 0.040, COL["belt"], alpha)
        sc.tube([P3(k), P3(f)], 0.011, COL["bar"], alpha); sc.sphere(P3(f), 0.020, COL["tpu"], alpha, n=6)
        return
    if key == "C":
        o, ka, f, kb, _ = arch.pts(q)
        sc.tube([P3(o), P3(ka), P3(f)], 0.011, COL["horse"], alpha); sc.tube([P3(o) + ey * 0.02, P3(kb) + ey * 0.02, P3(f)], 0.011, COL["bar"], alpha)
        sc.sphere(P3(f), 0.020, COL["tpu"], alpha, n=6)
        return
    P = panto_points(arch, q)
    sc.tube([P3(P["hip"]), P3(P["stifle"]), P3(P["P"])], 0.016, COL["horse"], alpha)                       # femur with its rod horn
    sc.cyl(P3(P["stifle"]) - ey * 0.012, P3(P["stifle"]) + ey * 0.012, 0.040, COL["belt"], alpha)         # stifle pulley
    sc.tube([P3(P["stifle"]), P3(P["hock"])], 0.011, COL["bar"], alpha)                                    # tibia
    for d in (-0.017, 0.017):
        sc.tube([P3(P["P"]) + ey * d, P3(P["Q"]) + ey * d], 0.0035, COL["latch"], alpha, balls=False)      # push rods
    sc.tube([P3(P["Q"]), P3(P["hock"]), P3(P["fet"])], 0.011, COL["bar"], alpha)                           # cannon with its horn
    sc.sphere(P3(P["hock"]), 0.015, COL["horse"], alpha, n=6)
    sc.tube([P3(P["fet"]), P3(P["foot"]) + np.array([0, 0, 0.012])], 0.008, COL["spring"], alpha)          # pastern
    f = P3(P["foot"]); sc.cyl(f, f + np.array([0, 0, 0.014]), 0.021, COL["tpu"], alpha, n=14)             # hoof


def note(fig, x, y, lines, fs=11.5, title=None, color="k"):
    if title:
        fig.text(x, y, title, fontsize=fs + 1.5, fontweight="bold", va="top", ha="left", color=color); y -= 0.034
    fig.text(x, y, "\n".join(lines), fontsize=fs, va="top", ha="left", linespacing=1.38)


def sheet1(R):
    fig = sk3d.figure(16, 9, "Equo-1 legs 1/4: three leg architectures, same loads (front leg, trot at 0.5 m/s, 32.6 kg centaur)")
    ca = R["arch"]
    txt = {}
    for k in "ABC":
        o = ca[k]; f = o["front"]; t = f["trot"]
        kn = lambda a: a[1] if k != "C" else a.max()
        hp = lambda a: a[0]
        e = o["env"]
        if k == "C":
            txt[k] = [f"Each actuator: {t['m2_rms'].max():.1f} rms / {t['m2_peak'].max():.1f} peak N m, no belt",
                      f"   = {t['m2_rms'].max() / 4.0 * 100:.0f} % of a CyberGear's rated 4 N m",
                      f"Get up from the kneel: {f['getup'].max():.1f} N m each",
                      f"Kneel: cranks rise {e['z_top'] * 1e3:.0f} mm above the hip axis",
                      f"Foot feels {o['m_eff']:.1f} kg of drive inertia (est.)",
                      "No single lock can hold it; not a horse leg"]
        else:
            txt[k] = [f"Knee {kn(t['m2_rms']):.1f} rms / {kn(t['m2_peak']):.1f} peak N m  (vertical-force model {kn(t['m1_rms']):.1f} / {kn(t['m1_peak']):.1f})",
                      f"Hip {hp(t['m2_rms']):.1f} / {hp(t['m2_peak']):.1f} N m  (vertical-force model {hp(t['m1_rms']):.1f} / {hp(t['m1_peak']):.1f})",
                      f"Get up from the kneel: knee {kn(f['getup']):.1f} N m",
                      f"Kneel: hip down to {o['h_min'] * 1e3:.0f} mm; knee reaches {e['x_max'] * 1e3:.0f} mm ahead",
                      f"Belt {o['belt_teeth'] * 5} mm, knee drive {o['belt_k'] / 1e3:.1f} kN m/rad",
                      f"Longest printed part {o['longest_part'] * 1e3:.0f} mm; foot feels {o['m_eff']:.1f} kg (est.)"]
    titles = {"A": "A  2-link, belt down the thigh (baseline)", "B": "B  3-segment pantograph (recommended)", "C": "C  coaxial 5-bar"}
    for i, k in enumerate("ABC"):
        x0 = 0.005 + i * 0.333
        sc = sk3d.Scene(fig, [x0, 0.335, 0.325, 0.60], elev=10, azim=-78, title=None)
        fig.text(x0 + 0.012, 0.925, titles[k], fontsize=14, fontweight="bold", color=COL["hi"] if k == "B" else "k")
        sc.ground((-0.48, -0.27), (0.24, -0.05))
        arch = ARCH[k]
        for hx, h, al in ((0.0, H, 1.0), (-0.30, H_KNEEL, 0.55)):
            draw_pod(sc, hx, h, -1.0, 1.0, "G  12 x CyberGear, 6S bus", alpha=al, cradle=False)
            draw_leg(sc, k, arch.ik(0.0, -h), hx, h, -Y_LEG["G  12 x CyberGear, 6S bus"], 1.0, alpha=al, side=-1.0)
        sc.text((0.02, -0.25, H + 0.075), "stand 361", fs=11.5)
        sc.text((-0.37, -0.25, H_KNEEL + (0.165 if k == "C" else 0.075)), f"kneel {H_KNEEL * 1e3:.0f}", fs=11.5)
        sc.done((-0.48, -0.27, 0.0), (0.24, -0.05, 0.46))
        note(fig, x0 + 0.012, 0.335, txt[k], fs=11.2)
    dm = R["parts"][REC]
    fig.text(0.017, 0.062, f"B moves exactly like a 2-link whose thigh is {ARCH['B'].A * 1e3:.0f} mm and shank {L2 * 1e3:.0f} mm (torques agree to {R['arch']['B_equals_virtual']:.0e} N m), "
             f"so the third segment buys no torque.\nIt buys the horse shape, a {ca['B']['belt_k'] / ca['A']['belt_k']:.2f}x stiffer and "
             f"{(ca['A']['belt_teeth'] - ca['B']['belt_teeth']) * 5} mm shorter knee belt, parts that fit the 256 mm bed, and a lower leg with no drive parts in it. "
             f"Cost: {dm['b_minus_a'] * 1e3:+.0f} g and {dm['b_minus_a_usd']:+.0f} USD per leg (est.).",
             fontsize=12, va="center", ha="left", linespacing=1.4, bbox=dict(boxstyle="round,pad=0.5", fc="#f3f5f7", ec="#c8ced6"))
    sk3d.save(fig, os.path.join(HERE, "legs_1_architectures.png"))


def sheet2(R):
    fig = sk3d.figure(16, 9, "Equo-1 legs 2/4: the recommended leg (hind leg shown; a front leg is the same module turned end for end)")
    B = ARCH["B"]; q = B.ik(0.0, -H); P = panto_points(B, q)
    # ---- 3D, right hind leg seen from outboard
    sc = sk3d.Scene(fig, [0.0, 0.05, 0.40, 0.86], elev=13, azim=-58)
    sc.ground((-0.27, -0.36), (0.30, 0.0))
    draw_pod(sc, 0.0, H, -1.0, 1.0, REC)
    draw_leg(sc, "B", q, 0.0, H, -Y_LEG[REC], 1.0, side=-1.0)
    sc.box((-0.02, -0.035, 0.36), (0.36, 0.07, 0.10), fc=COL["ghost"], alpha=0.3, sub=1)
    sc.line((-0.16, -Y_FOOT, H + Z_ABD), (0.17, -Y_FOOT, H + Z_ABD), color=COL["hi"], lw=1.4, ls="--")
    sc.line((0.0, -Y_FOOT, H + Z_ABD), (0.0, -Y_FOOT, 0.0), color=COL["hi"], lw=1.0, ls=":")
    m = R["module"][REC]; yl = -Y_LEG[REC]
    def lead(part, at, s_, color="k"):
        sc.line(part, at, color="#555555", lw=0.9); sc.text(at, s_, fs=11, color=color)
    sc.text((0.175, -Y_FOOT, H + Z_ABD + 0.02), "abduction axis\nover the foot", fs=11, color=COL["hi"])
    lead((-0.08, -Y_FOOT, H + Z_ABD + 0.03), (-0.27, -0.16, H + 0.125), "abduction\nactuator")
    lead((0.02, yl - 0.05, H - 0.02), (0.19, -0.16, H - 0.075), "knee actuator\n(rides on the femur)")
    lead(w3(P["stifle"], 0, H, yl - 0.012) + np.array([0.03, 0, 0]), (0.19, -0.16, 0.235), "48T stifle pulley")
    lead(w3((P["P"] + P["Q"]) / 2, 0, H, yl - 0.017), (0.19, -0.16, 0.165), "2 push rods", COL["latch"])
    lead(w3(P["hock"], 0, H, yl), (-0.21, -0.16, 0.115), "hock")
    lead(w3(P["fet"], 0, H, yl), (0.19, -0.16, 0.06), "sprung pastern,\nTPU hoof")
    sc.text((-0.235, -0.02, 0.43), "body side", fs=11, color="#666666")
    sc.done((-0.27, -0.36, 0.0), (0.30, 0.0, 0.47))
    note(fig, 0.012, 0.115, [f"Pod: {m['width'] * 1e3:.0f} mm across both sides, top at z {m['top'] * 1e3:.0f}. Three RobStride 02 (or CyberGear).",
                             f"Leg plane y {m['y_leg'] * 1e3:.0f}, foot y {Y_FOOT * 1e3:.0f}: static abduction torque 0 (leans out {m['camber']:.1f} deg)."], fs=11.5)
    # ---- 2D mechanism diagram (mm, hind leg, forward to the right)
    ax = fig.add_axes([0.40, 0.04, 0.36, 0.86]); ax.set_aspect("equal"); ax.axis("off")
    mm = lambda p: np.asarray(p) * 1e3
    hip, st, hk, fe, ft, Pp, Qq = (mm(P[k]) for k in ("hip", "stifle", "hock", "fet", "foot", "P", "Q"))
    ax.plot([-150, 190], [-361, -361], color="#888888", lw=2)
    ax.add_patch(Circle(hip, 43, fc="#dfe6ee", ec=COL["horse"], lw=2)); ax.add_patch(Circle(hip, pd(Z1) * 500, fc="white", ec=COL["belt"], lw=1.5))
    ax.add_patch(Circle(st, pd(Z2) * 500, fc="#eceff2", ec=COL["belt"], lw=1.5))
    d = st - hip; n = np.array([-d[1], d[0]]) / np.linalg.norm(d); r1, r2 = pd(Z1) * 500, pd(Z2) * 500
    for sgn in (1, -1):
        ax.plot([hip[0] + sgn * n[0] * r1, st[0] + sgn * n[0] * r2], [hip[1] + sgn * n[1] * r1, st[1] + sgn * n[1] * r2], color=COL["belt"], lw=2.2)
    ax.plot(*zip(hip, st, Pp), color=COL["horse"], lw=7, solid_capstyle="round", alpha=0.75)
    ax.plot(*zip(st, hk), color=COL["bar"], lw=6, solid_capstyle="round")
    ax.plot(*zip(Qq, hk, fe), color=COL["bar"], lw=6, solid_capstyle="round")
    ax.plot(*zip(Pp, Qq), color=COL["latch"], lw=3.5)
    for c in (Pp, Qq, hk, fe):
        ax.add_patch(Circle(c, 5, fc="white", ec="k", lw=1.2, zorder=5))
    ax.add_patch(Circle(st, 6, fc="white", ec="k", lw=1.2, zorder=5)); ax.add_patch(Circle(hip, 6, fc="white", ec="k", lw=1.2, zorder=5))
    # pastern, hoof, spring, ligament
    ax.plot(*zip(fe, ft + np.array([0, 12])), color=COL["spring"], lw=6, solid_capstyle="round")
    ax.add_patch(Polygon([ft + np.array([-24, 0]), ft + np.array([22, 0]), ft + np.array([14, 15]), ft + np.array([-18, 15])], fc=COL["tpu"], ec="k", lw=1))
    tu = u(P["tube"]); tn = np.array([-tu[1], tu[0]])                       # along the cannon tube, and to its rear
    s0, s1 = hk + mm(tu) * 0.016, hk + mm(tu) * 0.058
    zz = [s0 + (s1 - s0) * i / 12 + mm(tn) * (0.0065 if i % 2 else -0.0065) for i in range(13)]
    ax.plot(*zip(*zz), color=COL["spring"], lw=1.8, zorder=6)
    ses = fe - mm(u(P["tube"] + BETA_NOM)) * 0.0 + mm(tn) * 0.020                 # sesamoid horn tip behind the fetlock
    ax.plot(*zip(fe, ses), color=COL["spring"], lw=4, solid_capstyle="round")
    ax.plot(*zip(s1, fe + mm(tn) * 0.011 - mm(tu) * 0.012, ses), color=COL["hi"], lw=1.6, zorder=7)
    # stay stop and magnet at the stifle, offload cord
    sd = (hk - st) / np.linalg.norm(hk - st); sn = np.array([-sd[1], sd[0]])
    stop = st - sd * 12 + sn * 44
    ax.add_patch(Polygon([stop + np.array([-7, -5]), stop + np.array([7, -5]), stop + np.array([0, 9])], fc=COL["hi"], ec="k", lw=0.8, zorder=6))
    c0 = hip + np.array([-30, -20]); c1 = st + np.array([-28, 18])
    ax.plot(*zip(c0, c1), color=COL["spring"], lw=2.2, ls=(0, (4, 2)))
    # labels
    lab = [(hip + np.array([0, 43]), (-150, 62), "hip housing: hip-pitch actuator inboard,\nknee actuator + 24T pulley outboard"),
           ((hip + st) / 2 + n * 30, (60, -30), f"5M PU/steel belt, {BELT_W * 1e3:.0f} wide, {BELT_TEETH * 5} long\n{Z1}T : {Z2}T, femur {L1 * 1e3:.0f}"),
           (stop, (88, -88), f"stay stop + magnet: stifle\n{EPS_LOCK * R2D:.0f} deg past straight, motor off"),
           ((c0 + c1) / 2, (-150, -42), "offload shock cord\n(2 strands front, 1 rear)"),
           ((st + hk) / 2, (-150, -175), f"tibia: carbon tube 22, {L2 * 1e3:.0f}"),
           ((Pp + Qq) / 2, (80, -165), f"2 push rods: the hock\nfollows the stifle 1 : 1\n(reciprocal apparatus)"),
           (hk, (-150, -252), "hock pin 10 mm,\n2 x 6800"),
           ((s0 + s1) / 2, (70, -262), f"fetlock spring {R['fet']['front']['k_line'] / 1e3:.0f} N/mm\ninside the cannon tube ({LC * 1e3:.0f})"),
           (ses, (-150, -318), "Dyneema suspensory\nline to the pastern horn"),
           ((fe + ft) / 2, (80, -330), f"pastern {LP * 1e3:.0f}: sinks {R['fet']['front']['peak']['dz'] * 1e3:.0f} mm\nat {PK['trot'] * F_PAIR['front']:.0f} N, stores {R['fet']['front']['peak']['E']:.1f} J")]
    for (pt, pos, s) in lab:
        ax.annotate(s, xy=pt, xytext=pos, fontsize=11, va="center", ha="left", arrowprops=dict(arrowstyle="-", color="#555555", lw=0.9, shrinkA=2, shrinkB=3),
                    bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=0.85))
    ax.set_xlim(-155, 215); ax.set_ylim(-385, 85)
    # ---- right column: fetlock curve + numbers
    a2 = fig.add_axes([0.815, 0.56, 0.165, 0.30])
    for leg, c in (("front", COL["hi"]), ("rear", COL["horse"])):
        f = R["fet"][leg]
        a2.plot(f["dz"] * 1e3, f["F"], color=c, lw=2.2, label=leg)
        for kk, mk in (("stand", "o"), ("trot", "s"), ("peak", "^")):
            a2.plot(f[kk]["dz"] * 1e3, f[kk]["F"], mk, color=c, ms=7)
    a2.set_xlabel("hoof sink (mm)", fontsize=11); a2.set_ylabel("foot load (N)", fontsize=11); a2.tick_params(labelsize=11)
    a2.set_title("Fetlock spring", fontsize=12.5, fontweight="bold", loc="left"); a2.legend(fontsize=11, frameon=False, loc="lower right"); a2.grid(alpha=0.3)
    a2.text(0.3, 172, "o stand  [] trot  ^ peak", fontsize=10.5)
    ff = R["fet"]["front"]; rt = R["rods"]
    note(fig, 0.785, 0.47, [f"Fetlock, front: preload {ff['F'][0]:.0f} N, bump stop",
                            f"at {ff['stop']['F']:.0f} N and {ff['stop']['dz'] * 1e3:.0f} mm.",
                            f"Touchdown spike on the knee gear:",
                            f"{ff['spike_torque_rigid']:.0f} -> {ff['spike_torque_fet']:.0f} N m (est.).",
                            f"Unsprung: {R['parts'][REC]['unsprung'] * 1e3:.0f} g instead of {ff['m_eff']:.1f} kg.",
                            "",
                            f"Push rods (pair): {rt['trot peak, foot ahead']['rod']:.0f} N at the trot",
                            f"peak, {rt['kneel']['rod']:.0f} N kneeling,",
                            f"{R['bearings']['landing 2.5 g, pair']['rod']:.0f} N in a 2.5 g landing.",
                            f"Buckling load of one rod {R['structure']['push rod 6 x 4 carbon (one of two), axial']['buckling']:.0f} N.",
                            "",
                            f"Stay: {R['stay']['rows']['walking height (95 %)']['p_knee']:.1f} W of knee copper",
                            f"at walking height, {R['stay']['rows']['offload cord only, walking height']['p_knee']:.1f} W with the",
                            f"offload cord, 0 W on the stay stop."], fs=11.2)
    sk3d.save(fig, os.path.join(HERE, "legs_2_recommended.png"))


def sheet3(R):
    fig = sk3d.figure(16, 9, "Equo-1 legs 3/4: torques, margins, speed, springs and standing power (recommended leg)")
    B = ARCH["B"]; inr = inertia_of("B")
    gs = dict(left=0.055, right=0.985, top=0.90, bottom=0.135, wspace=0.26, hspace=0.50)
    axs = fig.subplots(2, 3, gridspec_kw=gs)
    fs = 11
    # (a) torque over the stance
    ax = axs[0, 0]
    xs = np.linspace(STRIDE / 2, -STRIDE / 2, 61); s = np.linspace(0, 1, 61)
    t1 = np.array([np.abs(support_torque(B, x, -H, (0, F_PAIR["front"]))[0]) for x in xs])
    c = m2_cycle(B, "front", "trot", inr); n_st = c["p"]["n_st"]
    ax.plot(s, t1[:, 1] * PK["trot"], color=COL["hi"], lw=1.6, ls="--", label="knee, vertical force x 1.4")
    ax.plot(c["p"]["s"], np.abs(c["tau"][:n_st, 1]), color=COL["hi"], lw=2.4, label="knee, shaped load aimed at the CoM")
    ax.plot(s, t1[:, 0] * PK["trot"], color=COL["horse"], lw=1.6, ls="--", label="hip, vertical force x 1.4")
    ax.plot(c["p"]["s"], np.abs(c["tau"][:n_st, 0]), color=COL["horse"], lw=2.4, label="hip, shaped load aimed at the CoM")
    ax.set_title("Front leg, trot: joint torque over the stance", fontsize=12.5, fontweight="bold", loc="left")
    ax.set_xlabel("stance phase (0 touchdown, 1 lift-off)", fontsize=fs); ax.set_ylabel("N m", fontsize=fs); ax.legend(fontsize=10, frameon=False, loc="upper center", ncol=1)
    ax.set_ylim(0, 24); ax.grid(alpha=0.3); ax.tick_params(labelsize=fs)
    # (b, c) utilisation bars
    cases = ["walk M2", "trot M2", "trot M1", "get up", "landing 2.5 g, pair", "trot + 0.25 g push", "walk up 10 deg"]
    short = ["walk", "trot", "trot\nvert.", "get\nup", "land\n2.5 g", "push\n0.25 g", "slope\n10 deg"]
    for ax, sn, ttl in ((axs[0, 1], REC, "Set R: 12 x RobStride 02 (12S)"), (axs[0, 2], "G  12 x CyberGear, 6S bus", "Set G: 12 x CyberGear (6S)")):
        mg = R["margins"][sn]["front"]; x = np.arange(len(cases)); wd = 0.2
        series = [("knee rms", "knee", "m_rms", COL["hi"], None), ("knee peak", "knee", "m_peak", COL["hi"], "//"),
                  ("hip rms", "hip", "m_rms", COL["horse"], None), ("hip peak", "hip", "m_peak", COL["horse"], "//")]
        for i, (lb, j, key, col, hatch) in enumerate(series):
            v = [100 / mg[cs][j][key] if mg[cs][j][key] < 98 else 0 for cs in cases]
            ax.bar(x + (i - 1.5) * wd, v, wd, color=col if hatch is None else "white", edgecolor=col, hatch=hatch, label=lb, lw=1)
        ax.axhline(100, color="k", lw=1.2, ls="--")
        ax.set_xticks(x); ax.set_xticklabels(short, fontsize=10.5); ax.set_ylim(0, 150); ax.set_ylabel("% of rated (rms) or peak limit", fontsize=fs)
        ax.set_title(ttl + ", front leg", fontsize=12.5, fontweight="bold", loc="left"); ax.tick_params(axis="y", labelsize=fs); ax.grid(axis="y", alpha=0.3)
        ax.legend(fontsize=10, frameon=False, ncol=2, loc="upper left")
    # (d) knee speed
    ax = axs[1, 0]
    for gait, col in (("walk", COL["horse"]), ("trot", COL["hi"]), ("fast trot", COL["pack"])):
        c = m2_cycle(B, "front", gait, inr)
        ax.plot(np.arange(len(c["qd"])) / len(c["qd"]), np.abs(c["qd"][:, 1]) * R2D, color=col, lw=2, label=f"{gait} {GAITS[gait]['v']} m/s")
    for sn, ls_ in ((REC, "-"), ("G  12 x CyberGear, 6S bus", ":")):
        ax.axhline(R["speeds"][sn]["knee_joint_limit_dps"], color="k", lw=1.2, ls=ls_)
        ax.text(0.02, R["speeds"][sn]["knee_joint_limit_dps"] + (14 if sn == REC else -52), f"rated actuator speed, set {sn[0]}", fontsize=10.5)
    ax.set_title("Knee speed over one cycle (2:1 belt)", fontsize=12.5, fontweight="bold", loc="left")
    ax.set_xlabel("gait cycle", fontsize=fs); ax.set_ylabel("deg/s at the stifle", fontsize=fs); ax.legend(fontsize=10.5, frameon=False, loc="upper left", bbox_to_anchor=(0.0, 0.87), labelspacing=0.15)
    ax.set_ylim(0, 1150); ax.grid(alpha=0.3); ax.tick_params(labelsize=fs)
    # (e) offload spring
    ax = axs[1, 1]; o = R["offload"]
    keys = [("front", "stand"), ("front", "walk"), ("front", "trot"), ("front", "getup"), ("rear", "walk"), ("rear", "trot")]
    val = lambda d, k: d[k] if not isinstance(d[k], dict) else d[k]["rms"]
    b0 = [val(o["base"], k) for k in keys]; b1 = [val(o["picked"][k[0]], k) for k in keys]
    x = np.arange(len(keys))
    ax.bar(x - 0.2, b0, 0.4, color=COL["frame"], label="no spring"); ax.bar(x + 0.2, b1, 0.4, color=COL["spring"], label="shock-cord spring")
    for xi, a, b in zip(x, b0, b1):
        ax.text(xi + 0.2, b + 0.2, f"{b:.1f}", ha="center", fontsize=10.5); ax.text(xi - 0.2, a + 0.2, f"{a:.1f}", ha="center", fontsize=10.5)
    ax.set_xticks(x); ax.set_xticklabels(["front\nstand", "front\nwalk", "front\ntrot", "front\nget up", "rear\nwalk", "rear\ntrot"], fontsize=10.5)
    ax.set_ylabel("knee torque left to the motor (N m)", fontsize=fs); ax.set_title("Gravity offload spring on the stifle (rms; get up = peak)", fontsize=12.5, fontweight="bold", loc="left")
    ax.legend(fontsize=10.5, frameon=False); ax.set_ylim(0, 12.5); ax.grid(axis="y", alpha=0.3); ax.tick_params(axis="y", labelsize=fs)
    # (f) standing power
    ax = axs[1, 2]; st = R["stay"]
    labels = ["no stay,\nwalking\nheight", "no stay,\nstand tall\n98 %", "offload\ncord\nonly", "over-\ncentre\nstop S1", "patella\npawl\nS2", "pawl +\nstops,\nall off"]
    knee = [st["rows"]["walking height (95 %)"]["p_knee"], st["rows"]["tall stand (98 %)"]["p_knee"], st["rows"]["offload cord only, walking height"]["p_knee"], 0, 0, 0]
    trick = [st["hip_trickle"]] * 5 + [0]; sb = [st["standby"]] * 5 + [0]
    x = np.arange(6)
    ax.bar(x, knee, 0.55, color=COL["hi"], label="knee copper"); ax.bar(x, trick, 0.55, bottom=knee, color=COL["horse"], label="hip + abduction hold (est.)")
    ax.bar(x, sb, 0.55, bottom=np.add(knee, trick), color=COL["frame"], label="12 drivers on standby")
    for xi, tot in zip(x, np.add(np.add(knee, trick), sb)):
        ax.text(xi, tot + 0.3, f"{tot:.1f} W", ha="center", fontsize=11, fontweight="bold")
    ax.set_xticks(x); ax.set_xticklabels(labels, fontsize=10.5); ax.set_ylabel("W", fontsize=fs); ax.set_ylim(0, 20)
    ax.set_title("Standing power, octopus on board (CyberGear data)", fontsize=12.5, fontweight="bold", loc="left"); ax.legend(fontsize=10.5, frameon=False, loc="upper right")
    ax.grid(axis="y", alpha=0.3); ax.tick_params(axis="y", labelsize=fs)
    fig.text(0.055, 0.02, f"Loads at the brief's 32.6 kg. With the leg masses estimated here the centaur is {R['parts'][REC]['m_centaur']:.1f} kg (set R) or "
             f"{R['parts']['G  12 x CyberGear, 6S bus']['m_centaur']:.1f} kg (set G): every load bar grows by {(R['parts'][REC]['growth'] - 1) * 100:.0f} % or "
             f"{(R['parts']['G  12 x CyberGear, 6S bus']['growth'] - 1) * 100:.0f} %.", fontsize=11.5)
    sk3d.save(fig, os.path.join(HERE, "legs_3_torques.png"))


def draw_centaur(sc, h, setname=REC):
    dz = h - H; B = ARCH["B"]; q = B.ik(0.0, -h)
    bx = BODY["x1"] - BODY["len"] / 2
    sc.box((bx, 0, (BODY["z0"] + BODY["z1"]) / 2 + dz), (BODY["len"], BODY["w"], BODY["z1"] - BODY["z0"]), fc=COL["horse"], sub=1)
    sc.ring((NECK_X, 0), 0.100, COLLAR_R, BODY["z1"] + dz, PLATE_Z + dz, COL["frame"])
    yl = Y_LEG[setname]
    for leg, sx in (("front", -1.0), ("rear", 1.0)):
        for side in (1.0, -1.0):
            hx = HIP_X[leg]
            for xo in (X_ABD - 0.02, X_LUG + 0.004):
                sc.box((hx + sx * xo, side * (BODY["w"] / 2 + Y_FOOT) / 2, h + Z_ABD), (0.018, Y_FOOT - BODY["w"] / 2, 0.022), fc=COL["frame"], sub=1)
            draw_pod(sc, hx, h, side, sx, setname)
            draw_leg(sc, "B", q, hx, h, side * yl, sx, side=side)
    sc.octopus(base=(NECK_X, 0, PLATE_Z + dz), yaw=45, bars=None, arms="centaur", fwd=0)


def sheet4(R):
    fig = sk3d.figure(16, 9, "Equo-1 legs 4/4: stand, kneel and the stay lock (front legs fold with the carpus forward, hind legs with the hock back)")
    kc = R["kneel"]
    for i, (h, ttl) in enumerate(((H, f"Walking height: hip {H * 1e3:.0f} mm"), (H_KNEEL, f"Kneel: hip {H_KNEEL * 1e3:.0f} mm"))):
        sc = sk3d.Scene(fig, [0.0 + i * 0.335, 0.27, 0.345, 0.66], elev=9, azim=-68, title=None)
        fig.text(0.02 + i * 0.335, 0.925, ttl, fontsize=14, fontweight="bold")
        sc.ground((-0.72, -0.42), (0.40, 0.42))
        draw_centaur(sc, h)
        sc.done((-0.74, -0.42, 0.0), (0.50, 0.42, 0.80))
    k, s = kc["kneel"], kc["stand"]
    note(fig, 0.02, 0.30, [f"Belly {s['belly'] * 1e3:.0f} mm, plate underside {s['plate'] * 1e3:.0f} mm above the ground.",
                           f"Stifle angle {s['psi_s']:.0f} deg, hock {s['psi_h']:.0f} deg; cannon tube {abs(s['tube']):.0f} deg off vertical.",
                           f"Front elbow to hind stifle: {s['elbow_to_stifle_gap'] * 1e3:.0f} mm clear.",
                           f"Front knee {R['margins'][REC]['front']['stand']['knee']['rms']:.1f} N m, rear {R['margins'][REC]['rear']['stand']['knee']['rms']:.1f} N m to stand here."], fs=11.5)
    note(fig, 0.355, 0.30, [f"Belly {k['belly'] * 1e3:.0f} mm, plate underside {k['plate'] * 1e3:.0f} mm above the ground.",
                            f"Stifle folds to {k['psi_s']:.0f} deg (limit {R['fold_psi']:.0f}: tibia tube meets the hip housing).",
                            f"Lowest leg part {R['arch']['B']['env']['low'] * 1e3:.0f} mm off the ground; elbow to stifle {k['elbow_to_stifle_gap'] * 1e3:.0f} mm.",
                            f"Get up: front knee {R['margins'][REC]['front']['get up']['knee']['peak']:.1f} N m, {R['offload']['picked']['front'][('front', 'getup')]:.1f} with the offload spring.",
                            f"Nothing on the leg rises above the pod (top z = hip + {(R['module'][REC]['top'] - H) * 1e3:.0f} mm)."], fs=11.5)
    # ---- pose strip, hind leg, 2D
    ax = fig.add_axes([0.685, 0.27, 0.31, 0.63]); ax.set_aspect("equal"); ax.axis("off")
    B = ARCH["B"]; ql, rl = lock_pose(B)
    poses = [("locked\n(stay)", ql, rl, 0), ("walk", B.ik(0, -H), H, 215), ("kneel", B.ik(0, -H_KNEEL), H_KNEEL, 445)]
    ax.plot([-90, 590], [0, 0], color="#888888", lw=2)
    for name, q, h, x0 in poses:
        P = panto_points(B, q); o = np.array([x0, h * 1e3])
        g = lambda kk: o + np.asarray(P[kk]) * 1e3
        ax.add_patch(Circle(g("hip"), 43, fc="#dfe6ee", ec=COL["horse"], lw=1.5)); ax.add_patch(Circle(g("stifle"), 38, fc="#eceff2", ec=COL["belt"], lw=1.2))
        ax.plot(*zip(g("hip"), g("stifle"), g("P")), color=COL["horse"], lw=5, solid_capstyle="round")
        ax.plot(*zip(g("stifle"), g("hock")), color=COL["bar"], lw=4.5, solid_capstyle="round")
        ax.plot(*zip(g("Q"), g("hock"), g("fet")), color=COL["bar"], lw=4.5, solid_capstyle="round")
        ax.plot(*zip(g("P"), g("Q")), color=COL["latch"], lw=2.5)
        ax.plot(*zip(g("fet"), g("foot")), color=COL["spring"], lw=4.5, solid_capstyle="round")
        ax.text(x0, h * 1e3 + 58, f"{name}\nhip {h * 1e3:.0f}", ha="center", va="bottom", fontsize=11.5, fontweight="bold")
    ax.annotate("", xy=(590, 361), xytext=(590, 0), arrowprops=dict(arrowstyle="<->", color="#555555")); ax.text(578, 180, "361", fontsize=11, rotation=90, va="center", ha="right")
    ax.set_xlim(-95, 600); ax.set_ylim(-12, 500)
    st = R["stay"]
    note(fig, 0.69, 0.30, [f"Lock: stifle {EPS_LOCK * R2D:.0f} deg past straight on a padded stop.",
                           f"Load holds it shut with {abs(st['rows']['over-centre stop (S1)']['knee']['front']):.2f} N m (front leg);",
                           f"magnet {st['magnet']:.2f} N m, offload cord {st['cord_at_lock']['front']:.1f} N m more.",
                           f"Opened by the knee motor ({st['release_torque_cord']:.1f} N m)",
                           f"or by a {st['unlock_force_cord']['front']:.0f} N side push on the foot.",
                           f"Hips still balance {st['sway_stiffness']:.0f} N m/rad of sway (trickle)."], fs=11.5)
    fo, re_ = R["fet"]["front"], R["fet"]["rear"]; of = R["offload"]
    note(fig, 0.02, 0.10, [f"Front and hind legs are one module; the front one is mirrored fore and aft. Front legs carry {SHARE_F * 100:.0f} %, so they differ only in springs: "
                            f"fetlock {fo['k_line'] / 1e3:.0f} N/mm (front) and {re_['k_line'] / 1e3:.0f} N/mm (rear); offload cord {of['cord_force']['front']:.0f} N and {of['cord_force']['rear']:.0f} N.",
                            f"Front pod corner sits {kc['collar']['dist'] * 1e3:.0f} mm from the neck axis and {kc['collar']['overlap_z'] * 1e3:.0f} mm above the collar base: chamfer it {kc['collar']['chamfer'] * 1e3:.0f} mm "
                            f"(or raise the collar base). Pod top to the octopus plate: {kc['collar']['pod_to_plate'] * 1e3:.0f} mm."], fs=11.5)
    sk3d.save(fig, os.path.join(HERE, "legs_4_kneel_stay.png"))


# ====================================================================== 11b. rejected alternatives, sized so they can be compared
def alternatives():
    B = ARCH["B"]; f = fetlock("front"); out = {}
    g = f["tube_tilt"] * D2R
    m_pk = PK["trot"] * F_PAIR["front"] * LP * np.sin(g + FET["b_design"]); m0 = FET["f0"] * LP * np.sin(g + FET["b0"])
    k_th = (m_pk - m0) / (FET["b_design"] - FET["b0"])                       # N m/rad at the fetlock pin
    # (i) two steel torsion springs on the fetlock pin, bending stress 1000 MPa (est. allowable), coil dia 18
    d = (32 * (m_pk / 2) / (np.pi * 1000e6)) ** (1 / 3); n = 200e9 * d ** 4 / (64 * 0.018 * (k_th / 2))
    out["fetlock torsion springs"] = dict(moment=m_pk, rate=k_th, wire=d, coils=n, width=2 * (n + 1) * d, mass=2 * np.pi * 0.018 * (n + 1.5) * np.pi * d ** 2 / 4 * 7850)
    # (ii) TPU 95A flexure pastern: E 40 MPa (est.), 30 wide, 25 long hinge; strain limit 15 % (est.)
    E, bw, l = 40e6, 0.030, 0.025
    t = (12 * l * k_th / (E * bw)) ** (1 / 3)
    out["fetlock TPU flexure"] = dict(thickness=t, strain=t * (FET["b_design"] - FET["b0"]) / (2 * l), E=E, loss=0.3, energy_back=(1 - 0.3) * f["peak"]["E"])
    out["fetlock Dyneema + coil spring"] = dict(mass=f["spring"]["mass"] + 0.005, energy=f["peak"]["E"], energy_back=0.9 * f["peak"]["E"])
    # offload spring alternatives for 3.5 N m at stand and 0.8 N m/rad
    d = (32 * 5.0 / (np.pi * 1000e6)) ** (1 / 3); n = 200e9 * d ** 4 / (64 * 0.030 * 0.8)
    out["offload steel torsion spring"] = dict(wire=d, coils=n, width=(n + 1) * d, mass=np.pi * 0.030 * (n + 1.5) * np.pi * d ** 2 / 4 * 7850)
    out["offload shock cord"] = dict(force=3.5 / 0.030, mass=0.010, usd=1.0, loss=0.25)
    out["offload gas spring"] = dict(force=3.5 / 0.030, mass=0.060, usd=12.0)
    # non-backdrivable knee: T8 lead screw, 8 mm lead, 500 rpm (est.) against the leg-length speed the trot swing needs
    out["lead screw knee"] = dict(speed=0.008 * 500 / 60, need=np.pi * CLEAR / foot_path("trot")["t_sw"])
    out["pawl servo"] = dict(mass=0.020, usd=15.0, n=4)
    return out


# ====================================================================== 12. run: numbers, tables, json, sheets
def clean(o):
    if isinstance(o, dict):
        return {(" / ".join(map(str, k)) if isinstance(k, tuple) else str(k)): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return o.tolist() if o.size < 50 else None
    if isinstance(o, (np.floating, np.integer)):
        return float(o)
    if callable(o):
        return None
    return o


def study():
    R = dict(geom=dict(L1=L1, L2=L2, L3=L3, LC=LC, LP=LP, delta=DELTA * R2D, A=ARCH["B"].A, dp=ARCH["B"].dp * R2D, off=OFF * R2D, r_horn=R_HORN,
                       belt_teeth=BELT_TEETH, z1=Z1, z2=Z2, h_kneel=H_KNEEL, leg_max=LEG_MAX),
             loads=dict(F_stand=F_STAND, F_pair=F_PAIR, peak_trot={l: PK["trot"] * F_PAIR[l] for l in F_PAIR}, land_four={l: LAND_G * F_STAND[l] for l in F_PAIR},
                        land_pair={l: LAND_G * F_PAIR[l] for l in F_PAIR}))
    R["arch"] = compare_arch(); R["scan"] = geometry_scan()
    R["fold_psi"] = fold_limit(ARCH["B"])[1] * R2D
    R["fet"] = {l: fetlock(l) for l in ("front", "rear")}
    R["rods"] = rod_table()
    R["stay"] = stay("CyberGear", "CyberGear"); R["stay_R"] = stay("RobStride 02", "RobStride 02")
    R["offload"] = offload()
    R["margins"] = {s: margins(s) for s in SETS}
    spr = {l: (lambda a, t=R["offload"]["pick"][l], q0=R["offload"]["q_stand"]: max(t[0] + t[1] * (a - q0), 0.0)) for l in ("front", "rear")}
    R["margins_spring"] = {s: {l: margins(s, spr[l])[l] for l in ("front", "rear")} for s in SETS}
    R["speeds"] = {s: speeds(s) for s in SETS}
    R["belt"] = belt_check(); R["bearings"] = bearing_check()
    R["module"] = {s: hip_module(s, Y_LEG[s]) for s in SETS}
    R["kneel"] = kneel_check(R["module"][REC])
    R["structure"] = structure()
    R["parts"] = {s: parts(s) for s in SETS}
    R["alt"] = alternatives()
    # copper loss and bus power in a trot (M2), all 12 actuators
    R["power"] = {}
    for s in SETS:
        if any(ACT[a]["kt"] is None for a in (SETS[s]["abd"], SETS[s]["knee"], *SETS[s]["hip"].values())):
            continue
        for tag, mg in (("no spring", R["margins"][s]), ("with spring", R["margins_spring"][s])):
            R["power"][(s, tag)] = {g: 2 * sum(mg[l][g + " M2"][j]["p_cu"] for l in ("front", "rear") for j in ("hip", "knee", "abd")) for g in ("walk", "trot")}
    return R


def report(R):
    p = print; g = R["geom"]
    p(f"LOADS  stand F/R {F_STAND['front']:.1f}/{F_STAND['rear']:.1f} N; pair F/R {F_PAIR['front']:.1f}/{F_PAIR['rear']:.1f}; trot peak {PK['trot'] * F_PAIR['front']:.1f}/{PK['trot'] * F_PAIR['rear']:.1f}; "
      f"landing 2.5 g on 4 feet {LAND_G * F_STAND['front']:.0f}/{LAND_G * F_STAND['rear']:.0f}, on a pair {LAND_G * F_PAIR['front']:.0f}/{LAND_G * F_PAIR['rear']:.0f} N")
    p(f"GEOMETRY (B)  femur {g['L1'] * 1e3:.1f}  tibia {g['L2'] * 1e3:.0f}  hock-to-contact {g['L3'] * 1e3:.1f} (cannon tube {g['LC'] * 1e3:.1f} + pastern {g['LP'] * 1e3:.0f} at {BETA_NOM * R2D:.0f} deg)"
      f"  delta {g['delta']:.0f} deg  virtual thigh {g['A'] * 1e3:.1f} (turned {g['dp']:.1f} deg from the femur)  full length {(g['A'] + g['L2']) * 1e3:.0f}  horn {R_HORN * 1e3:.0f}")
    p("ACTUATORS (verified listings; est. where marked)")
    for k, a_ in ACT.items():
        p(f"  {k:13s} {a_['rated']:.1f} / {a_['peak']:.1f} N m, {a_['m'] * 1e3:.0f} g, dia {a_['d'] * 1e3:.1f} x {a_['t'] * 1e3:.1f}, {a_['v'][0]}-{a_['v'][1]} V, {a_['w_rated']} rpm rated, USD {a_['usd'][0]}-{a_['usd'][1]}"
          + (f", Kt {a_['kt']} N m/Arms, R {a_['r_ll']} ohm, copper at rated {copper(a_['rated'], k):.1f} W, rotor inertia {a_['j_rotor']:.1e} kg m2 (est.)" if a_["kt"] else ""))
    ov = ACT["CyberGear"]["overload"]
    p(f"  CyberGear limits (manual): 6.5 A rated and 23 A peak phase current; {ov[12]} s at 12 N m, {ov[6]} s at 6 N m, {ov[5]} s at 5 N m; standby {ACT['CyberGear']['standby_w']:.2f} W")
    for k, v in OTHER_ACT.items():
        p(f"  {k:19s} {v[0]} / {v[1]} N m, {v[2] * 1e3:.0f} g, {v[3]}, {v[4]} V, USD {v[5]}; {v[6]}")
    R["bus"] = dict(parallel=PACK_V, series=(2 * PACK_V[0], 2 * PACK_V[1]))
    p(f"  bus: two 6S packs in parallel {PACK_V[0]}-{PACK_V[1]} V (CyberGear 16-28 V fits, RobStride 24-60 V does not); in series {2 * PACK_V[0]:.1f}-{2 * PACK_V[1]:.1f} V (RobStride and AK70-10 fit, CyberGear does not)")
    p("ARCHITECTURES (front leg; M1 = vertical constant force x peak factor, M2 = shaped load aimed at the CoM + swing inertia)")
    for k in "ABC":
        o = R["arch"][k]; f = o["front"]; e = o["env"]
        p(f"  {o['name']:26s} eff. mass at foot {o['m_eff']:.2f} kg  kneel min hip {o['h_min'] * 1e3:.0f}  at {H_KNEEL * 1e3:.0f}: reach {e['x_min'] * 1e3:.0f}..{e['x_max'] * 1e3:.0f}, top {e['z_top'] * 1e3:.0f} above hip, lowest part {e['low'] * 1e3:.0f} off ground"
          f"  belt {o['belt_teeth'] * 5} mm, k {o['belt_k']:.0f} N m/rad  longest part {o['longest_part'] * 1e3:.0f}")
        for gait in ("walk", "trot"):
            t = f[gait]
            p(f"      {gait:5s} hip/knee  M1 rms {t['m1_rms'][0]:.2f}/{t['m1_rms'][1]:.2f} peak {t['m1_peak'][0]:.2f}/{t['m1_peak'][1]:.2f}   M2 rms {t['m2_rms'][0]:.2f}/{t['m2_rms'][1]:.2f} peak {t['m2_peak'][0]:.2f}/{t['m2_peak'][1]:.2f}"
              f"   speed {t['speed'][0] * R2D:.0f}/{t['speed'][1] * R2D:.0f} deg/s  friction needed {t['mu']:.2f}")
        p(f"      stand {f['stand'][0]:.2f}/{f['stand'][1]:.2f}  get up {f['getup'][0]:.2f}/{f['getup'][1]:.2f}  landing 4 feet {f['landing']['four'][1]:.1f}, pair mid {f['landing']['pair_mid'][1]:.1f}, pair worst {f['landing']['pair_worst'][0]:.1f}/{f['landing']['pair_worst'][1]:.1f}")
    p(f"  B against a 2-link with thigh {g['A'] * 1e3:.0f} / shank {g['L2'] * 1e3:.0f}: largest torque difference {R['arch']['B_equals_virtual']:.1e} N m")
    p("GEOMETRY SCAN (femur fixed by the belt)")
    for r in R["scan"]:
        p(f"  delta {r['delta']:4d}  tibia {r['l2'] * 1e3:.0f}  hock-contact {r['l3'] * 1e3:5.1f}  cannon tube {r['lc'] * 1e3:5.1f}  tube tilt at stand {r['tube']:6.1f} deg  min hip {r['h_min'] * 1e3:5.1f}  trot knee peak {r['knee_peak']:.2f}  get up {r['getup']:.2f}")
    p("PUSH RODS (pair; + = compression)")
    for k, v in R["rods"].items():
        p(f"  {k:26s} horn angle {v['mu']:6.1f} deg  rod pair {v['rod']:7.1f} N" + (f"  hock moment {v['m_hock']:.2f} N m" if "m_hock" in v else ""))
    for leg in ("front", "rear"):
        f = R["fet"][leg]; s = f["spring"]
        p(f"FETLOCK {leg}: tube tilt {f['tube_tilt']:.1f} deg  line spring {f['k_line'] / 1e3:.1f} N/mm  preload {f['t0']:.0f} N (foot {f['F'][0]:.0f} N)  monotonic {f['monotonic']}")
        for kk in ("stand", "trot", "peak"):
            v = f[kk]; p(f"    {kk:5s} {v['F']:6.1f} N  pastern {v['beta']:.1f} deg  sink {v['dz'] * 1e3:5.1f} mm  stored {v['E']:.2f} J  line {v['T']:.0f} N")
        p(f"    bump stop {f['stop']['F']:.0f} N  sink {f['stop']['dz'] * 1e3:.1f} mm  {f['stop']['E']:.2f} J   stiffness at the foot {f['kz_first'] / 1e3:.1f} -> {f['kz_peak'] / 1e3:.1f} N/mm  line at stop {f['t_stop']:.0f} N (Dyneema SF {f['line_sf']:.1f})  pin {f['pin_load']:.0f} N")
        p(f"    spring wire {s['d'] * 1e3:.1f}, OD {(s['D'] + s['d']) * 1e3:.1f}, {s['n']:.1f} active coils, stress {s['tau_peak'] / 1e6:.0f} MPa at peak / {s['tau_stop'] / 1e6:.0f} at the stop, solid {s['solid'] * 1e3:.0f}, free {s['free'] * 1e3:.0f} mm, {s['mass'] * 1e3:.0f} g")
        p(f"    touchdown at {FET['v_td']} m/s: spike {f['spike_rigid']:.0f} N rigid hoof -> {f['spike_fet']:.0f} N sprung; on the knee {f['spike_torque_rigid']:.1f} -> {f['spike_torque_fet']:.1f} N m; drive mass at the foot {f['m_eff']:.2f} kg")
    e4 = 2 * R["fet"]["front"]["stop"]["E"] + 2 * R["fet"]["rear"]["stop"]["E"]
    R["fet_drop"] = dict(E=e4, h=e4 / (M_TOT * G), trot_w=(2 * R["fet"]["front"]["peak"]["E"] + 2 * R["fet"]["rear"]["peak"]["E"]) / foot_path("trot")["T"])
    p(f"    four fetlocks to their stops store {e4:.1f} J = a free drop of {e4 / (M_TOT * G) * 1e3:.0f} mm; in a trot they cycle {R['fet_drop']['trot_w']:.1f} W that the knees would otherwise handle")
    for tag, st in (("CyberGear", R["stay"]), ("RobStride 02", R["stay_R"])):
        p(f"STAY ({tag} constants)  standby {st['standby']:.1f} W (12 drivers)  hip+abduction trickle {st['hip_trickle']:.2f} W (est.)  a 15 mm foot offset would cost {st['abd_15mm']:.1f} W")
        for k, v in st["rows"].items():
            p(f"    {k:28s} hip {v['h'] * 1e3:.1f}  knee torque F/R {v['knee']['front']:.2f}/{v['knee']['rear']:.2f} N m  knee copper {v['p_knee']:.2f} W  total {v['p_knee'] + st['hip_trickle'] + st['standby']:.1f} W")
    st = R["stay"]
    p(f"    with the offload cord the knee is pressed onto its stop by a further F/R {st['cord_at_lock']['front']:.2f}/{st['cord_at_lock']['rear']:.2f} N m: side force to unlock {st['unlock_force_cord']['front']:.0f}/{st['unlock_force_cord']['rear']:.0f} N, release torque {st['release_torque_cord']:.1f} N m")
    p(f"    lock: body {st['lift'] * 1e3:.2f} mm under full height; stop force {st['stop_force']['front']:.0f} N (2.5 g pair {st['stop_force_land']:.0f} N); magnet {st['magnet']} N m; release torque {st['release_torque']:.2f} N m;"
      f" unlocking side force at the foot F/R {st['unlock_force']['front']:.1f}/{st['unlock_force']['rear']:.1f} N; sway stiffness to hold {st['sway_stiffness']:.0f} N m/rad")
    pk = st["park"]
    p(f"    sawhorse park (pawl + end stops, all motors off): rake {pk['rake']} deg, hip {pk['h'] * 1e3:.0f} mm, hip stop F/R {pk['hip_stop']['front']:.1f}/{pk['hip_stop']['rear']:.1f} N m, knee lock F/R {pk['knee']['front']:.1f}/{pk['knee']['rear']:.1f} N m,"
      f" holds a {pk['shove_g']:.2f} g shove; pawl force at walking height F/R {st['pawl_force']['front']:.0f}/{st['pawl_force']['rear']:.0f} N")
    p(f"    one hour standing: {R['stay']['rows']['walking height (95 %)']['p_knee']:.1f} Wh of knee copper = {R['stay']['rows']['walking height (95 %)']['p_knee'] / PACK_WH * 100:.1f} % of the two packs")
    o = R["offload"]
    p(f"OFFLOAD SPRING  best constant torque for the front walk + trot: {o['best']['tau0']:.1f} N m (+{o['best']['k']:.1f} N m/rad);  picked front {o['pick']['front']}, rear {o['pick']['rear']} (N m at stand, N m/rad);"
      f" cord force F/R {o['cord_force']['front']:.0f}/{o['cord_force']['rear']:.0f} N on a 30 mm sector")
    for leg in ("front", "rear"):
        b, k = o["base"], o["picked"][leg]
        p(f"    {leg:5s} stand {b[(leg, 'stand')]:.2f} -> {k[(leg, 'stand')]:.2f}   walk rms {b[(leg, 'walk')]['rms']:.2f} -> {k[(leg, 'walk')]['rms']:.2f} (swing peak {b[(leg, 'walk')]['swing']:.1f} -> {k[(leg, 'walk')]['swing']:.1f})"
          f"   trot rms {b[(leg, 'trot')]['rms']:.2f} -> {k[(leg, 'trot')]['rms']:.2f} (peak {b[(leg, 'trot')]['peak']:.1f} -> {k[(leg, 'trot')]['peak']:.1f}, swing {b[(leg, 'trot')]['swing']:.1f} -> {k[(leg, 'trot')]['swing']:.1f})"
          f"   get up {b[(leg, 'getup')]:.1f} -> {k[(leg, 'getup')]:.1f}   spring at the kneel {k[(leg, 'kneel_spring')]:.1f} N m   horse alone standing: motor holds {o['alone'][leg]:.1f} N m")
    for s in SETS:
        m = R["margins"][s]; sp = R["speeds"][s]; pr = R["parts"][s]
        p(f"ACTUATOR SET {s}:  leg {pr['m_leg']['front']:.2f} kg  legs {pr['m_legs']:.2f} kg  horse {pr['m_horse']:.1f} kg  centaur {pr['m_centaur']:.1f} kg (x{pr['growth']:.3f} on every load)"
          f"  actuators USD {2 * pr['usd_act']['front'][0] + 2 * pr['usd_act']['rear'][0]}-{2 * pr['usd_act']['front'][1] + 2 * pr['usd_act']['rear'][1]}")
        for leg in ("front", "rear"):
            for case, r in m[leg].items():
                p(f"    {leg:5s} {case:22s} " + "   ".join(f"{j} {v['rms']:5.2f}/{v['peak']:5.2f} of {v['rated']:4.1f}/{v['limit']:4.1f} (x{min(v['m_rms'], 99):5.2f}, x{min(v['m_peak'], 99):5.2f})" for j, v in r.items()))
        worst_p = min(v["m_peak"] for leg in m for r in m[leg].values() for v in r.values()); worst_r = min(v["m_rms"] for leg in m for r in m[leg].values() for v in r.values())
        req = [c for c in m["front"] if not (c.startswith("trot +") or c.startswith("walk up"))]
        rq_p = min(v["m_peak"] for leg in m for c in req for v in m[leg][c].values()); rq_r = min(v["m_rms"] for leg in m for c in req for v in m[leg][c].values())
        R.setdefault("worst", {})[s] = dict(req_rms=rq_r, req_peak=rq_p, all_rms=worst_r, all_peak=worst_p, growth=pr["growth"])
        p(f"    worst margins, brief cases (stand, walk, trot, get up, landing): rms x{rq_r:.2f}, peak x{rq_p:.2f}; at the mass estimated here: x{rq_r / pr['growth']:.2f}, x{rq_p / pr['growth']:.2f}")
        p(f"    worst margins with the reserve cases (0.25 g push, 10 deg slope): rms x{worst_r:.2f}, peak x{worst_p:.2f}; at the mass estimated here: x{worst_r / pr['growth']:.2f}, x{worst_p / pr['growth']:.2f}")
        ms = R["margins_spring"][s]
        p(f"    with the offload spring: front knee trot M2 {ms['front']['trot M2']['knee']['rms']:.2f}/{ms['front']['trot M2']['knee']['peak']:.2f} (x{ms['front']['trot M2']['knee']['m_rms']:.2f}, x{ms['front']['trot M2']['knee']['m_peak']:.2f}),"
          f" walk M2 {ms['front']['walk M2']['knee']['rms']:.2f}/{ms['front']['walk M2']['knee']['peak']:.2f} (x{ms['front']['walk M2']['knee']['m_rms']:.2f})")
        for gait in ("walk", "trot", "fast trot"):
            v = sp[gait]
            p(f"    speed {gait:9s} {v['v']} m/s, swing {v['t_sw']:.3f} s: knee {v['knee_dps']:.0f} deg/s = {v['knee_act_rpm']:.0f} rpm at the actuator (rated {sp['knee_act_limit_rpm']:.0f} rpm at the pack voltage, x{sp['knee_act_limit_rpm'] / v['knee_act_rpm']:.2f}); hip {v['hip_dps']:.0f} deg/s = {v['hip_act_rpm']:.0f} rpm")
        if ACT[SETS[s]["knee"]]["kt"] and all(ACT[a]["kt"] for a in SETS[s]["hip"].values()):
            t = m["front"]["trot M2"]
            p("    currents, front leg trot M2 (Arms rms / peak; copper W): " + "   ".join(f"{j} {v['i_rms']:.1f}/{v['i_peak']:.1f} A, {v['p_cu']:.1f} W" for j, v in t.items()))
            for tag in ("no spring", "with spring"):
                pw = R["power"][(s, tag)]
                p(f"    copper loss of all 12, {tag}: walk {pw['walk']:.1f} W, trot {pw['trot']:.1f} W")
    b = R["belt"]
    p(f"BELT  5M, {BELT_W * 1e3:.0f} wide, {BELT_TEETH} teeth: pulleys {Z1}T (pitch dia {b['pd1'] * 1e3:.1f}) : {Z2}T ({b['pd2'] * 1e3:.1f}), centres {b['centre'] * 1e3:.1f}, wrap on the small pulley {b['wrap']:.0f} deg = {b['mesh']:.1f} teeth in mesh")
    p(f"    working tension (Gates, neoprene/fibreglass): 5M GT 15 mm {b['ta15']:.0f} N, 25 mm {b['ta25']:.0f} N, 5M HTD 15 mm {b['ta_htd15']:.0f} N; tensile rating 15 mm {BELT_BREAK_15:.0f} N;"
      f" PU/steel HTD 5M 15 mm: tooth limit {b['pu_tooth']:.0f} N, tension {b['pu_tension']:.0f} N")
    for k, v in b["cases"].items():
        p(f"    {k:24s} knee {v['torque']:5.1f} N m  tension {v['tension']:4.0f} N  x{v['m_ta15']:.2f} of the 15 mm working tension  (25 mm x{v['m_ta25']:.2f})  x{v['m_break']:.1f} to the tensile rating")
    p(f"    trot rms tension {b['rms_tension']:.0f} N (x{b['m_rms']:.1f}); knee torque at the 15 mm working tension {b['torque_limit_15']:.1f} N m, 25 mm {b['torque_limit_25']:.1f} N m; knee drive stiffness {b['k_theta']:.0f} N m/rad")
    for k, v in b["ratio_scan"].items():
        p(f"    ratio {k}: stifle pulley pitch dia {v['pd2'] * 1e3:.0f} mm, trot peak tension {v['tension_trot']:.0f} N (x{b['ta15'] / v['tension_trot']:.2f}), CyberGear knee {4 * v['ratio'] * BELT_EFF:.1f}/{12 * v['ratio'] * BELT_EFF:.1f} N m at {222 / v['ratio'] * 6:.0f} deg/s")
    p("BEARINGS (load per bearing, static rating, safety factor)")
    for case, v in R["bearings"].items():
        p(f"  {case} (foot {v['Fz']:.0f} N, rod pair {v['rod']:.0f} N): " + "; ".join(f"{k} {x['load']:.0f} N of {x['c0']} (x{x['sf']:.1f})" for k, x in v.items() if isinstance(x, dict) and "c0" in x))
        p(f"      actuator output bearing: {v['act_bearing_with_stub']:.0f} N radial with the stub axle; without it {v['act_bearing_without']['radial']:.0f} N and {v['act_bearing_without']['moment']:.1f} N m")
    for s in SETS:
        m = R["module"][s]
        p(f"HIP MODULE {s}: leg plane y {m['y_leg'] * 1e3:.0f}, axis y {Y_FOOT * 1e3:.0f} z {(H + Z_ABD) * 1e3:.0f}, leans out {m['camber']:.2f} deg; pod y {m['y_inner'] * 1e3:.1f}..{m['y_outer'] * 1e3:.1f}; width {m['width'] * 1e3:.0f} ({m['width_no_stub'] * 1e3:.0f} without the stub cheek);"
          f" length {m['length'] * 1e3:.0f}; top z {m['top'] * 1e3:.0f}; bridge top {m['bridge_top'] * 1e3:.0f}; clashes {len(m['clashes'])}; inward abduction {m['abd_in']:.1f} deg")
    m = R["module"][REC]
    p(f"    femur angle: gait {m['th_gait'][0]:.1f}..{m['th_gait'][1]:.1f} deg, kneel {m['th_kneel']:.1f}, locked {m['th_lock']:.1f}; stifle angle {m['stifle_range'][0]:.1f} (locked) .. {m['stifle_range'][1]:.1f} (kneel); femur clears the front journal by {m['lug_clear'] * 1e3:.0f} mm")
    for k, v in m["solids"].items():
        p(f"      {k:22s} x {v[0] * 1e3:6.1f}..{v[1] * 1e3:6.1f}  y {v[2] * 1e3:6.1f}..{v[3] * 1e3:6.1f}  z {v[4] * 1e3:6.1f}..{v[5] * 1e3:6.1f}")
    kc = R["kneel"]
    for k in ("stand", "kneel"):
        v = kc[k]
        p(f"POSE {k}: hip {v['hip'] * 1e3:.0f}  belly {v['belly'] * 1e3:.0f}  plate {v['plate'] * 1e3:.0f}  stifle z {v['stifle_z'] * 1e3:.0f}  hock z {v['hock_z'] * 1e3:.0f}  lowest housing {v['lowest'] * 1e3:.0f}"
          f"  elbow-stifle gap {v['elbow_to_stifle_gap'] * 1e3:.0f}  front carpus reaches x {v['front_reach'] * 1e3:.0f}, hind hock x {v['rear_reach'] * 1e3:.0f}  stifle {v['psi_s']:.1f} deg  hock {v['psi_h']:.1f} deg  tube {v['tube']:.1f} deg  femur {v['th1']:.1f} deg")
    c = kc["collar"]
    p(f"    front pod bridge corner {c['dist'] * 1e3:.0f} mm from the neck axis (collar r {c['r'] * 1e3:.0f}); bridge top {c['bridge_top'] * 1e3:.0f} against collar base {c['collar_base'] * 1e3:.0f}: {c['overlap_z'] * 1e3:.0f} mm overlap in height,"
      f" chamfer {c['chamfer'] * 1e3:.0f} mm; pod top to plate underside {c['pod_to_plate'] * 1e3:.0f} mm; fold limit stifle {R['fold_psi']:.1f} deg")
    mR = R["module"][REC]; t_a = ACT[SETS[REC]["abd"]]["t"]
    R["interface"] = dict(front_pod_x=(-(X_LUG + 0.008), -(X_ABD - t_a)), hind_pod_x=(HIP_X["rear"] + X_ABD - t_a, HIP_X["rear"] + X_LUG + 0.008),
                          axis_y=Y_FOOT, axis_z=H + Z_ABD, pod_y=(mR["y_inner"], mR["y_outer"]), pod_z=(H - 0.044, mR["top"]),
                          outrigger_x=(-X_ABD + t_a / 2, -(X_LUG + 0.004)), body_front_overhang=-(X_ABD - t_a) - BODY["x1"],
                          hip_travel=(mR["th_lock"] - 5, mR["th_kneel"] + 10), first_pass_horse=8.58)
    it = R["interface"]
    p(f"INTERFACE (set R, world mm): abduction axes along x at y +-{it['axis_y'] * 1e3:.0f}, z {it['axis_z'] * 1e3:.0f}; front pods x {it['front_pod_x'][0] * 1e3:.0f}..{it['front_pod_x'][1] * 1e3:.0f}, hind pods x {it['hind_pod_x'][0] * 1e3:.0f}..{it['hind_pod_x'][1] * 1e3:.0f},"
      f" y {it['pod_y'][0] * 1e3:.1f}..{it['pod_y'][1] * 1e3:.1f}, z {it['pod_z'][0] * 1e3:.0f}..{it['pod_z'][1] * 1e3:.0f}; actuator outrigger at {it['outrigger_x'][0] * 1e3:.0f} and journal outrigger at {it['outrigger_x'][1] * 1e3:.0f} mm from each hip line (toward the body end / toward the belly);"
      f" pods overhang the body front by {it['body_front_overhang'] * 1e3:.0f} mm; hip-pitch stops at {it['hip_travel'][0]:.0f} and {it['hip_travel'][1]:.0f} deg of femur angle")
    lever = R["stay"]["rows"]["walking height (95 %)"]["knee"]["front"] / F_STAND["front"]
    ff = R["fet"]["front"]
    R["sensing"] = dict(knee_lever=lever, force_per_0p2Nm=0.2 / lever, n_per_deg=(ff["trot"]["F"] - ff["stand"]["F"]) / (ff["trot"]["beta"] - ff["stand"]["beta"]),
                        n_per_count=(ff["trot"]["F"] - ff["stand"]["F"]) / (ff["trot"]["beta"] - ff["stand"]["beta"]) * 360 / 4096)
    sn_ = R["sensing"]
    p(f"SENSING  knee lever at walking height {sn_['knee_lever'] * 1e3:.1f} mm: a 0.2 N m torque error (est.) is {sn_['force_per_0p2Nm']:.1f} N at the foot; fetlock angle {sn_['n_per_deg']:.1f} N per deg, {sn_['n_per_count']:.2f} N per count of a 12-bit AS5600; no reading below the {ff['F'][0]:.0f} N preload")
    for s2 in SETS:
        q2 = R["parts"][s2]
        p(f"MASS {s2}: horse {q2['m_horse']:.2f} kg against the first pass {it['first_pass_horse']:.2f} kg ({q2['m_horse'] - it['first_pass_horse']:+.2f} kg); centaur {q2['m_centaur']:.1f} kg; printed PA-CF + TPU for four legs {4 * q2['printed']:.2f} kg")
    al = R["alt"]; v = al["fetlock torsion springs"]
    p(f"ALTERNATIVES  fetlock needs {v['moment']:.1f} N m at the trot peak, {v['rate']:.1f} N m/rad. Two steel torsion springs on the pin: wire {v['wire'] * 1e3:.1f} mm, {v['coils']:.1f} coils each, {v['width'] * 1e3:.0f} mm wide in total, {v['mass'] * 1e3:.0f} g at the foot")
    v = al["fetlock TPU flexure"]
    p(f"    TPU 95A flexure (E {v['E'] / 1e6:.0f} MPa est., 30 wide, 25 long): {v['thickness'] * 1e3:.0f} mm thick, strain {v['strain'] * 100:.0f} %, about {v['loss'] * 100:.0f} % lost per cycle (est.): {v['energy_back']:.1f} J back;"
      f" Dyneema + coil spring: {al['fetlock Dyneema + coil spring']['mass'] * 1e3:.0f} g, {al['fetlock Dyneema + coil spring']['energy_back']:.1f} J back (10 % loss est.)")
    v = al["offload steel torsion spring"]
    p(f"    offload: steel torsion spring wire {v['wire'] * 1e3:.1f} mm, {v['coils']:.0f} coils, {v['width'] * 1e3:.0f} mm wide, {v['mass'] * 1e3:.0f} g; shock cord {al['offload shock cord']['force']:.0f} N, {al['offload shock cord']['mass'] * 1e3:.0f} g, {al['offload shock cord']['usd']:.0f} USD,"
      f" about {al['offload shock cord']['loss'] * 100:.0f} % hysteresis (est.); gas spring {al['offload gas spring']['force']:.0f} N, about {al['offload gas spring']['mass'] * 1e3:.0f} g and {al['offload gas spring']['usd']:.0f} USD (est.)")
    v = al["lead screw knee"]
    p(f"    non-backdrivable knee (T8 screw, 8 mm lead, 500 rpm est.): {v['speed'] * 1e3:.0f} mm/s of leg length against {v['need'] * 1e3:.0f} mm/s needed in the trot swing;"
      f" patella pawl: one micro servo per leg, about {al['pawl servo']['mass'] * 1e3:.0f} g and {al['pawl servo']['usd']:.0f} USD each (est.)")
    p("STRUCTURE (trot peak / 2.5 g landing on a pair)")
    for k, v in R["structure"].items():
        p(f"  {k:62s} {v['load']:7.1f} / {v['load_land']:7.1f}   {v['stress'] / 1e6:6.1f} / {v['stress_land'] / 1e6:6.1f} MPa of {v['allow'] / 1e6:.0f}   x{v['sf']:.1f} / x{v['sf_land']:.1f}" + (f"   buckling {v['buckling']:.0f} N" if "buckling" in v else ""))
    pr = R["parts"][REC]
    p(f"PARTS per leg (set R): printed {pr['printed'] * 1e3:.0f} g, structure and hardware {pr['struct'] * 1e3:.0f} g, actuators {pr['m_act']['front'] * 1e3:.0f} g, leg {pr['m_leg']['front'] * 1e3:.0f} g; below the stifle {pr['distal'] * 1e3:.0f} g; unsprung {pr['unsprung'] * 1e3:.0f} g;"
      f" B minus A {pr['b_minus_a'] * 1e3:+.0f} g, {pr['b_minus_a_usd']:+.1f} USD; parts USD {pr['usd_parts']:.0f}; leg USD {pr['usd_leg']['front'][0]:.0f}-{pr['usd_leg']['front'][1]:.0f}")
    for n, mat, q, mass, bb, orient, usd, ob in pr["list"]:
        p(f"    {q} x {n:68s} {mat:18s} {mass * 1e3:6.1f} g  " + (f"{bb[0]} x {bb[1]} x {bb[2]}  " if bb[0] else "") + orient)
    for s in SETS:
        q = R["parts"][s]
        p(f"    {s}: leg F/R {q['m_leg']['front']:.2f}/{q['m_leg']['rear']:.2f} kg, four legs {q['m_legs']:.2f} kg, leg cost F {q['usd_leg']['front'][0]:.0f}-{q['usd_leg']['front'][1]:.0f} R {q['usd_leg']['rear'][0]:.0f}-{q['usd_leg']['rear'][1]:.0f} USD,"
          f" four legs {2 * q['usd_leg']['front'][0] + 2 * q['usd_leg']['rear'][0]:.0f}-{2 * q['usd_leg']['front'][1] + 2 * q['usd_leg']['rear'][1]:.0f} USD")


if __name__ == "__main__":
    R = study()
    report(R)
    json.dump(clean(R), open(os.path.join(HERE, "legs.json"), "w"), indent=1)
    print("wrote", os.path.join(HERE, "legs.json"))
    sheet1(R); sheet2(R); sheet3(R); sheet4(R)
