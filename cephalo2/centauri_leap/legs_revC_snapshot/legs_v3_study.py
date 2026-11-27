#!/usr/bin/env python3
"""Hippos-1 legs v3: fresh architecture study (2026-11-18).  Forest: "replan the legs"; goals picked: more agile, more
horse-like motion.  Hand estimates (quasi-static + swing inertia), no CAD, no sim.  Everything marked (est.) is an estimate.
    python3 legs_v3_study.py  ->  legs_v3.json, tables on stdout
Frames: leg sagittal plane, pivot (hip / scapula pivot) at the origin, x forward, z up; angles from straight down,
positive = distal end forward.  A front leg mirrored fore-aft is drawn with sx = -1 where the option says so.
Four concepts, same body (0.75 scale: front hip x 219 z 368, rear hip x -150 z 378), same mass budget:
  A  current pantograph (both legs one module, front turned end for end), knee motor on the femur's scapula arm, belt 2:1
  B  horse-true: a different fore and hind leg.  Fore: scapula from a high pivot, humerus, forearm-cannon column slaved
     parallel to the scapula (push rod), carpus locked straight in stance and folded in swing by a cable.  Hind: femur,
     tibia, cannon slaved parallel to the femur through a spring rod (reciprocal apparatus).  Both motors of every leg in
     the body (knee / shoulder motor coaxial with the pivot, belt down the upper segment).
  C  agile two-link: thigh + shank, knee motor coaxial at the hip with a belt, sprung pastern; elbows back, stifles forward
  D  one horse-hind Z module on all four (front turned end for end as in A), both motors in the body, spring push rods
"""
import os, json
import numpy as np
from scipy.optimize import least_squares

HERE = os.path.dirname(os.path.abspath(__file__))
R2D, D2R, G = 180 / np.pi, np.pi / 180, 9.81

# ====================================================================== 0. body and mass (hippos1.md, tailarm_checks)
M0 = 33.4                         # Centauri-1 with the current legs, hand3 on all arms and the arm tail (est.)
SHARE_F = 0.60                    # front legs' share docked (hippos1: 59.6 %, est.)
HIP = {"front": np.array([0.219, 0.3675]), "rear": np.array([-0.150, 0.378])}     # chassis_v3 at S = 0.75 (x, z)
X_COM = HIP["rear"][0] + SHARE_F * (HIP["front"][0] - HIP["rear"][0])
Z_COM = 0.50                      # docked CoM height (octopus on the neck, est.)
Z_ABD = 0.022                     # abduction axis above the pitch pivot (v2 29 mm x 0.75)
Y_TRACK = 0.1785
# current legs (4u): RS06 hips, knees, front abduction; RS02 rear abduction; structure 0.55 kg per leg (est.)
M_LEGS_NOW = 10 * 0.621 + 2 * 0.405 + 4 * 0.55

# ====================================================================== 0b. actuators (verified listings in actuators.json; see the notes there)
ACT = json.load(open(os.path.join(HERE, "act_table.json")))       # name -> rated, peak, m, w_free (rpm at bus), usd, d, t, kt, r_ll, j_rotor, gear
BUS_V = 39.6                                                     # 12S nearly empty (3.3 V/cell): torque-speed checked here
BELT_EFF = 0.95
EXCLUDE = set()                                                 # actuators left out of the choice (unverified listings)
RATIOS = (1.0, 1.5, 2.0, 2.5, 3.0)                              # knee belt ratios tried (1.0 = direct / 1:1 belt)

# ====================================================================== 1. gaits and load cases
GAITS = {   # v m/s, duty factor, stride m, swing height m (0.75 scale; est.)
    "walk": dict(v=0.30, duty=0.70, stride=0.14, clear=0.045),
    "trot": dict(v=0.60, duty=0.50, stride=0.18, clear=0.055),
    "fast trot": dict(v=0.90, duty=0.42, stride=0.21, clear=0.060),
    "gallop": dict(v=1.60, duty=0.32, stride=0.26, clear=0.070),
}
JUMP_H = 0.10                     # required jump (hooves 100 mm off the ground)
DROP_H = 0.25                     # drop landing onto four legs
STOP = 0.08                       # leg compression the controller uses to stop a landing (est.)
REAR_DEG = 40.0                   # rearing: body pitched nose-up
PUSH_G = 0.25                     # reserve: horizontal push in a trot
SIDE_G = 0.25                     # side shove on one pair (abduction)
KNEEL_H = 0.14                    # kneel: hip pivot height (front pivot in B is 100 mm higher)


# ====================================================================== 2. leg mechanisms
def u(t):
    return np.array([np.sin(t), -np.cos(t)])


class Panto:
    """Three segments; the third is slaved parallel to the first plus delta (push rod across the middle segment).
    q = (upper angle, angle between upper and middle segment).  Foot = end of the third segment."""
    def __init__(s, l1, l2, l3, delta):
        s.l1, s.l2, s.l3, s.delta = l1, l2, l3, delta
        s.A = np.hypot(l1 + l3 * np.cos(delta), l3 * np.sin(delta))
        s.dp = np.arctan2(l3 * np.sin(delta), l1 + l3 * np.cos(delta))
        s.lmax = s.A + l2

    def pts(s, q):
        a = s.l1 * u(q[0]); b = a + s.l2 * u(q[0] - q[1]); f = b + s.l3 * u(q[0] + s.delta)
        return [np.zeros(2), a, b, f]

    def foot(s, q):
        return s.pts(q)[-1]

    def ik(s, x, z):
        r = np.hypot(x, z); phi = np.arctan2(x, -z)
        qv = np.pi - np.arccos(np.clip((s.A ** 2 + s.l2 ** 2 - r ** 2) / (2 * s.A * s.l2), -1, 1))
        al = np.arccos(np.clip((s.A ** 2 + r ** 2 - s.l2 ** 2) / (2 * s.A * r), -1, 1))
        return np.array([phi + al - s.dp, qv - s.dp])


def h_min(m):
    """Lowest pivot height with the foot under the pivot: the leg folded until its middle segment meets the upper one's housing."""
    return max(abs(m.A - m.l2), 0.30 * m.lmax) + 0.005


def h_run(key, leg, gait):
    """Pivot height while running a gait: the body drops if the stride ends would need more than 96 % of the full leg."""
    o = OPT[key]; m = o["legs"][leg]; L = GAITS[gait]["stride"]
    return min(o["pivot"][leg][1], np.sqrt((0.96 * m.lmax) ** 2 - (L / 2 + abs(o["x0"][leg])) ** 2))


class TwoLink(Panto):
    def __init__(s, la, lb):
        super().__init__(la, lb, 0.0, 0.0)

    def pts(s, q):
        a = s.l1 * u(q[0]); f = a + s.l2 * u(q[0] - q[1])
        return [np.zeros(2), a, f]


def jac(m, q, e=1e-6):
    J = np.zeros((2, 2))
    for j in range(2):
        d = np.zeros(2); d[j] = e
        J[:, j] = (m.foot(q + d) - m.foot(q - d)) / (2 * e)
    return J


def solve_scale(ratios, angles, height):
    """Segment lengths k * ratios so the chain at the given segment angles (from down, + forward) spans `height`."""
    z = sum(r * np.cos(a) for r, a in zip(ratios, angles))
    k = height / z
    return [k * r for r in ratios], k * sum(r * np.sin(a) for r, a in zip(ratios, angles))


# horse segment proportions (equine anatomy, est.; refined from the research notes where available)
HORSE = {"fore": dict(names=("scapula", "humerus", "forearm + cannon + pastern"), ratio=(0.40, 0.30, 0.74), ang=(30, -55, 0)),
         "hind": dict(names=("femur", "tibia", "cannon + pastern"), ratio=(0.42, 0.40, 0.45), ang=(35, -45, 0))}

LEG_A = dict(l1=0.75 * 0.14808, l2=0.75 * 0.210, l3=0.75 * 0.15909, delta=-10 * D2R)        # legs_v2 B at S = 0.75
CARPUS = 0.50                     # B fore: carpus at half the column (forearm 0.37 of the horse's 0.74)
B_RAISE = 0.100                                                                             # B: scapula pivot above the old front hip


def build():
    O = {}
    a = Panto(LEG_A["l1"], LEG_A["l2"], LEG_A["l3"], LEG_A["delta"])
    O["A"] = dict(name="A  current pantograph", short="A current", legs={"front": a, "rear": a}, sx={"front": -1, "rear": 1},
                  pivot={k: HIP[k].copy() for k in HIP}, x0={"front": 0.0, "rear": 0.0},
                  knee_motor="on the femur's scapula arm (72 mm above the hip, swings with the femur)", knee_r=0.072,
                  spring_rod=False, carpus=False, coaxial=False, ratio=2.0)
    # B fore: scapula / humerus / column, pivot raised; B hind: horse hind proportions
    hf = HIP["front"][1] + B_RAISE
    (s1, s2, s3), xf = solve_scale(HORSE["fore"]["ratio"], np.radians(HORSE["fore"]["ang"]), hf)
    (h1, h2, h3), xh = solve_scale(HORSE["hind"]["ratio"], np.radians(HORSE["hind"]["ang"]), HIP["rear"][1])
    fore = Panto(s1, s2, s3, np.radians(HORSE["fore"]["ang"][2] - HORSE["fore"]["ang"][0]))
    hind = Panto(h1, h2, h3, np.radians(HORSE["hind"]["ang"][2] - HORSE["hind"]["ang"][0]))
    O["B"] = dict(name="B  horse-true fore and hind", short="B horse-true", legs={"front": fore, "rear": hind}, sx={"front": 1, "rear": 1},
                  pivot={"front": np.array([HIP["front"][0] - xf, hf]), "rear": np.array([HIP["rear"][0] - xh, HIP["rear"][1]])},
                  x0={"front": xf, "rear": xh}, knee_motor="in the body, coaxial with the pivot; belt down the scapula / femur", knee_r=0.0,
                  spring_rod=True, carpus=True, coaxial=True, ratio=2.0)
    c = TwoLink(0.215, 0.235)                    # stands at 82 % of its length (a two-link near straight is singular)
    O["C"] = dict(name="C  agile two-link", short="C two-link", legs={"front": c, "rear": c}, sx={"front": -1, "rear": 1},
                  pivot={k: HIP[k].copy() for k in HIP}, x0={"front": 0.0, "rear": 0.0},
                  knee_motor="in the body, coaxial with the hip; belt down the thigh", knee_r=0.0, spring_rod=False, carpus=False, coaxial=True, ratio=2.0)
    O["D"] = dict(name="D  one horse-hind module", short="D one module", legs={"front": hind, "rear": hind}, sx={"front": -1, "rear": 1},
                  pivot={"front": np.array([HIP["front"][0] + xh, HIP["front"][1]]), "rear": np.array([HIP["rear"][0] - xh, HIP["rear"][1]])},
                  x0={"front": xh, "rear": xh}, knee_motor="in the body, coaxial with the hip; belt down the femur", knee_r=0.0,
                  spring_rod=True, carpus=False, coaxial=True, ratio=2.0)
    return O


OPT = build()


# ---------------------------------------------------------------------- masses and inertia (est.)
LIN = 0.55                 # kg/m of printed / carbon segment incl. joint ends (est.)
M_HOOF = 0.060             # pastern, fetlock spring, hoof (est.)
M_JOINT = 0.030            # each slaved joint (bearings, pin, lug) (est.)
M_ROD = 0.020              # push rod pair (spring rod +0.025) (est.)
M_PULLEY = 0.045           # driven pulley at the knee joint (est.)
M_BELT = 0.040


def leg_parts(key, leg):
    o = OPT[key]; m = o["legs"][leg]
    segs = [m.l1, m.l2] + ([m.l3] if m.l3 > 0 else [])
    seg_m = [LIN * l for l in segs]
    extra = dict(hoof=M_HOOF, pulley=M_PULLEY, belt=M_BELT, joints=M_JOINT * (len(segs) - 1))
    if len(segs) == 3:
        extra["rods"] = M_ROD + (0.025 if o["spring_rod"] else 0.0)
    if o["carpus"] and leg == "front":
        extra["carpus lock + cable"] = 0.035
    return segs, seg_m, extra


def inertia(key, leg, act_hip, act_knee):
    """(pivot, knee) inertia seen by the two motors in the stance pose: links + reflected rotors (est.)."""
    o = OPT[key]; m = o["legs"][leg]
    q = m.ik(o["x0"][leg], -o["pivot"][leg][1])
    P = m.pts(q)
    segs, seg_m, extra = leg_parts(key, leg)
    I_piv, I_knee = 0.0, 0.0
    for i, (ms, l) in enumerate(zip(seg_m, segs)):
        c = (P[i] + P[i + 1]) / 2
        I_piv += ms * (c @ c + l ** 2 / 12)
        if i >= 1:
            ck = c - P[1]; I_knee += ms * (ck @ ck + l ** 2 / 12)
    for k, mm in extra.items():
        at = P[-1] if k in ("hoof",) else (P[1] if k in ("pulley", "joints") else (P[1] + P[-1]) / 2)
        if k == "belt":
            at = P[1] / 2
        I_piv += mm * (at @ at)
        if k != "belt":
            I_knee += mm * ((at - P[1]) @ (at - P[1]))
    if not o["coaxial"]:          # A: the knee motor rides on the femur's scapula arm
        I_piv += ACT[act_knee]["m"] * o["knee_r"] ** 2
    a_h, a_k = ACT[act_hip], ACT[act_knee]
    I_piv += a_h["j_rotor"] * a_h["gear"] ** 2
    I_knee += a_k["j_rotor"] * a_k["gear"] ** 2 * RATIO_NOW.get((key, leg), o["ratio"]) ** 2
    return np.array([I_piv, I_knee])


# ====================================================================== 3. load cases -> joint torques (+ = in +q)
def weight(key):
    return M_TOTAL[key] * G


def share(leg):
    return SHARE_F if leg == "front" else 1 - SHARE_F


def foot_path(key, leg, gait, n=240):
    o = OPT[key]; g = GAITS[gait]
    h = h_run(key, leg, gait); x0 = o["x0"][leg]; L = g["stride"]
    t_st = L / g["v"]; T = t_st / g["duty"]
    n_st = int(round(n * g["duty"])); n_sw = n - n_st
    s = (np.arange(n_st) + 0.5) / n_st; s2 = (np.arange(n_sw) + 0.5) / n_sw
    vt = g["v"] * (T - t_st)
    x = x0 + np.r_[L / 2 - L * s, -L / 2 - vt * s2 + (L + vt) * (10 * s2 ** 3 - 15 * s2 ** 4 + 6 * s2 ** 5)]
    z = np.r_[-h * np.ones(n_st), -h + g["clear"] * (1 - np.cos(2 * np.pi * s2)) / 2]
    # the swing hoof cannot go past the leg's reach: where the path would need more than 97 % of the full leg, the hoof
    # rides up on that circle (the leg folds), then the path is smoothed so the joint accelerations stay finite
    m = o["legs"][leg]; rr = 0.97 * m.lmax
    r = np.hypot(x - 0.0, z); over = np.where(r > rr)[0]
    if len(over):
        sw = np.arange(n_st, n)
        zc = np.where(np.hypot(x[sw], z[sw]) > rr, -np.sqrt(np.maximum(rr ** 2 - x[sw] ** 2, 1e-6)), z[sw])
        k = np.hanning(9); k /= k.sum()
        zs = np.convolve(np.r_[zc[:4][::-1], zc, zc[-4:][::-1]], k, mode="same")[4:-4]
        z[sw] = np.minimum(np.maximum(zs, -np.sqrt(np.maximum(rr ** 2 - x[sw] ** 2, 1e-6))), zc.max())
    return dict(x=x, z=z, s=s, n_st=n_st, T=T, dt=T / n, t_sw=T - t_st)


def gait_cycle(key, leg, gait, inr, extra_fx=0.0, spring=None):
    o = OPT[key]; m = o["legs"][leg]; p = foot_path(key, leg, gait)
    n, n_st = len(p["x"]), p["n_st"]
    q = np.array([m.ik(x, z) for x, z in zip(p["x"], p["z"])])
    qd = (np.roll(q, -1, 0) - np.roll(q, 1, 0)) / (2 * p["dt"])
    qdd = (np.roll(q, -1, 0) - 2 * q + np.roll(q, 1, 0)) / p["dt"] ** 2
    g = GAITS[gait]
    fz = np.zeros(n); fz[:n_st] = (np.pi / 2) * share(leg) / 2 * weight(key) / g["duty"] * np.sin(np.pi * p["s"])      # half-sine
    # force line aimed at the CoM (x measured from the leg's stand point, in the leg frame: front legs mirrored)
    fx = -fz * (p["x"] - o["x0"][leg]) / Z_COM + extra_fx * fz
    sup = np.array([jac(m, q[i]).T @ np.array([fx[i], fz[i]]) for i in range(n)])
    tau = inr * qdd - sup
    if spring is not None:
        tau = tau + spring
    return dict(q=q, qd=qd, tau=tau, rms=np.sqrt(np.mean(tau ** 2, 0)), peak=np.abs(tau).max(0), speed=np.abs(qd).max(0),
                fz_peak=fz.max(), t_sw=p["t_sw"], n_st=n_st)


def static(key, leg, x, z, F):
    m = OPT[key]["legs"][leg]; q = m.ik(x, z)
    return -(jac(m, q).T @ np.asarray(F, float)), q


def jump_case(key, leg):
    """Required jump: 100 mm hoof clearance from a crouch at 70 % of the stand height (constant force over the stroke)."""
    o = OPT[key]; h = o["pivot"][leg][1]; x0 = o["x0"][leg]
    m = o["legs"][leg]
    z_top = min(0.97 * m.lmax, np.sqrt((0.97 * m.lmax) ** 2 - x0 ** 2))
    zc = max(0.70 * h, h_min(m) + 0.02)
    stroke = z_top - zc
    v = np.sqrt(2 * G * JUMP_H)                       # leaves at full leg length; the hooves then rise JUMP_H
    a = v ** 2 / (2 * stroke)
    F = share(leg) / 2 * M_TOTAL[key] * (G + a)
    taus = [np.abs(static(key, leg, x0, -z, (0, F))[0]) for z in np.linspace(zc, z_top, 25)]
    return np.max(taus, 0), dict(F=F, a=a, stroke=stroke, v=v)


def rear_case(key):
    """Rearing: body pitched REAR_DEG nose-up about the rear hips, hind feet under the CoM, 0.2 g balancing force."""
    o = OPT[key]; m = o["legs"]["rear"]; th = REAR_DEG * D2R
    piv = o["pivot"]["rear"]
    c = np.array([X_COM, Z_COM]) - piv
    Rb = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]])
    cw = Rb @ c
    reach = 0.92 * m.lmax
    fw = np.array([cw[0], -np.sqrt(reach ** 2 - cw[0] ** 2)])            # foot under the CoM, world, from the pivot
    fb = Rb.T @ fw
    out = []
    for bal in (-0.2, 0.0, 0.2):
        Fw = np.array([bal, 1.0]) * weight(key) / 2
        t, q = static(key, "rear", fb[0], fb[1], Rb.T @ Fw)
        out.append(np.abs(t))
    return np.max(out, 0), dict(foot_body=fb, reach=reach)


def landing_case(key, leg):
    o = OPT[key]; h = o["pivot"][leg][1]
    stop = STOP + (0.025 if o["spring_rod"] else 0.0) + 0.010            # spring rods add 25 mm, the sprung pastern 10 mm (est.)
    F = share(leg) / 2 * weight(key) * (1 + DROP_H / stop)
    t, q = static(key, leg, o["x0"][leg], -(h - STOP / 2), (0, F))
    return np.abs(t), dict(F=F, factor=1 + DROP_H / stop)


def getup_case(key, leg):
    """From the kneel up to the stand, foot under the pivot.  B's foreleg kneels on its carpus (as a horse does): first
    the carpus pad is the contact (column cut at the carpus), then the hoof."""
    o = OPT[key]; m = o["legs"][leg]; h = o["pivot"][leg][1]
    h0 = KNEEL_H + (B_RAISE if (key == "B" and leg == "front") else 0.0)
    F = share(leg) / 2 * weight(key)
    out = []
    if o["carpus"] and leg == "front":
        mc = Panto(m.l1, m.l2, CARPUS * m.l3, m.delta)
        for z in np.linspace(max(h0, h_min(mc)), 0.97 * mc.lmax, 20):
            q = mc.ik(o["x0"][leg], -z); out.append(np.abs(jac(mc, q).T @ np.array([0, F])))
        h0 = h_min(m)
    for z in np.linspace(max(h0, h_min(m)), h, 30):
        out.append(np.abs(static(key, leg, o["x0"][leg], -z, (0, F))[0]))
    return np.max(out, 0)


def kneel_height(key, leg):
    o = OPT[key]; m = o["legs"][leg]
    if o["carpus"] and leg == "front":
        return h_min(Panto(m.l1, m.l2, CARPUS * m.l3, m.delta))
    return h_min(m)


OFFLOAD = 0.8                     # knee offload spring (shock cord / gas spring) = 80 % of the standing knee torque, constant (est.)


def joint_cases(key, leg, acts, spring=True):
    """Motor-side joint torques (rms, peak) per case.  The knee offload spring carries OFFLOAD x the standing knee torque
    all the time: it helps every support load and costs the motor that much in the swing."""
    inr = inertia(key, leg, acts["hip"][leg], acts["knee"][leg])
    o = OPT[key]; c = {}
    st, _ = static(key, leg, o["x0"][leg], -o["pivot"][leg][1], (0, share(leg) / 2 * weight(key)))
    sp = np.array([0.0, -OFFLOAD * st[1]]) if spring else np.zeros(2)
    help_ = np.abs(sp)
    sub = lambda t: np.maximum(np.abs(t) - help_, 0.0)
    c["stand"] = (sub(st), sub(st))
    for gname in GAITS:
        g = gait_cycle(key, leg, gname, inr, spring=sp)
        c[gname] = (g["rms"], g["peak"])
    g = gait_cycle(key, leg, "trot", inr, extra_fx=PUSH_G, spring=sp)
    c["trot + 0.25 g push"] = (g["rms"], g["peak"])
    c["get up"] = (np.zeros(2), sub(getup_case(key, leg)))
    c["jump 100 mm"] = (np.zeros(2), sub(jump_case(key, leg)[0]))
    c["land from 250 mm"] = (np.zeros(2), sub(landing_case(key, leg)[0]))
    if leg == "rear":
        r = sub(rear_case(key)[0]); c[f"rear up {REAR_DEG:.0f} deg"] = (r, r)
    return c


CORE = ("stand", "walk", "trot")                            # continuous duty: rated torque with 1.2 margin
AGILE = ("fast trot", "gallop", "jump 100 mm", "land from 250 mm", f"rear up {REAR_DEG:.0f} deg", "trot + 0.25 g push")


# ====================================================================== 4. actuator choice per joint
def pick(req_rms, req_peak, ratio=1.0, speed_rad=0.0, names=None):
    """Lightest actuator with rated >= 1.2 x rms and peak >= 1.1 x peak (through `ratio`), and no-load speed at the bus
    >= 1.5 x the joint speed."""
    best = None
    for n, a in ACT.items():
        if n in EXCLUDE:
            continue
        if names and n not in names:
            continue
        eff = BELT_EFF if ratio != 1 else 1.0
        rated, peak = a["rated"] * ratio * eff, a["peak"] * ratio * eff
        w_free = a["w_free_48"] * BUS_V / 48 * 2 * np.pi / 60 / ratio
        ok = rated >= 1.2 * req_rms and peak >= 1.1 * req_peak and w_free >= 1.5 * speed_rad
        score = a["m"]
        if ok and (best is None or score < best[1]):
            best = (n, score)
    return best[0] if best else None


def size(key):
    """Iterate: actuator masses change the leg mass and the total, which changes the loads."""
    o = OPT[key]
    acts = {"hip": {"front": "RobStride 06", "rear": "RobStride 06"}, "knee": {"front": "RobStride 06", "rear": "RobStride 06"},
            "abd": {"front": "RobStride 06", "rear": "RobStride 02"}}
    for it in range(4):
        legs_mass = legs_total(key, acts)
        M_TOTAL[key] = M0 - M_LEGS_NOW + legs_mass
        res = {}
        for leg in ("front", "rear"):
            c = joint_cases(key, leg, acts)
            rms = np.max([c[k][0] for k in c if k in CORE], 0)
            pk = np.max([c[k][1] for k in c], 0)
            spd = max(gait_cycle(key, leg, "gallop", inertia(key, leg, acts["hip"][leg], acts["knee"][leg]))["speed"][j] for j in (0,)), \
                max(gait_cycle(key, leg, "gallop", inertia(key, leg, acts["hip"][leg], acts["knee"][leg]))["speed"][j] for j in (1,))
            acts["hip"][leg] = pick(rms[0], pk[0], 1.0, spd[0]) or "none"
            best = None
            for ratio in RATIOS:
                n_ = pick(rms[1], pk[1], ratio, spd[1])
                if n_ and (best is None or ACT[n_]["m"] < ACT[best[0]]["m"] - 1e-9 or (abs(ACT[n_]["m"] - ACT[best[0]]["m"]) < 1e-9 and ratio < best[1])):
                    best = (n_, ratio)
            acts["knee"][leg] = best[0] if best else "none"
            acts.setdefault("ratio", {})[leg] = best[1] if best else o["ratio"]
            RATIO_NOW[(key, leg)] = acts["ratio"][leg]
            z_a = o["pivot"][leg][1] + Z_ABD
            ab_pk = max(SIDE_G * share(leg) * weight(key) * z_a, 0.05 * share(leg) / 2 * weight(key) * (np.pi / 2) / GAITS["gallop"]["duty"] * z_a)
            ab_rms = 0.05 * share(leg) / 2 * weight(key) * z_a
            acts["abd"][leg] = pick(ab_rms, ab_pk) or "none"
            res[leg] = dict(cases=c, rms=rms, peak=pk, speed=spd, abd=(ab_rms, ab_pk))
        if "none" in [acts[j][l] for j in ("hip", "knee", "abd") for l in ("front", "rear")]:
            break
    return acts, res


def legs_total(key, acts):
    tot = 0.0
    for leg in ("front", "rear"):
        segs, seg_m, extra = leg_parts(key, leg)
        a = sum(ACT[acts[j][leg]]["m"] for j in ("hip", "knee", "abd") if acts[j][leg] in ACT)
        pod = 0.20                                                     # cradle, bearings, pulley at the motor (est.)
        tot += 2 * (sum(seg_m) + sum(extra.values()) + a + pod)
    return tot


M_TOTAL = {k: M0 for k in OPT}
RATIO_NOW = {}


# ====================================================================== 5. agility measures
def max_jump(key, acts):
    """Four-leg vertical jump, no pitch: both pivots rise together over a common stroke that ends with every leg at 97 %
    of its full length; actuators at peak torque cut by a linear torque-speed line at 39.6 V (est.); front and rear push
    in their weight shares.  Returns hoof clearance (body rise after the hooves leave), take-off speed, push time, stroke."""
    o = OPT[key]; m_tot = M_TOTAL[key]
    ztop = {leg: 0.97 * o["legs"][leg].lmax for leg in ("front", "rear")}
    stroke = min(ztop[l] - max(0.60 * o["pivot"][l][1], h_min(o["legs"][l]) + 0.02) for l in ztop)
    zc = {l: ztop[l] - stroke for l in ztop}
    dz = v = t = 0.0; dt = 2e-4
    while dz < stroke and t < 1.0:
        Fl = {}
        for leg in ("front", "rear"):
            mm = o["legs"][leg]; z = zc[leg] + dz
            q = mm.ik(o["x0"][leg], -z); J = jac(mm, q)
            qd = np.linalg.solve(J, np.array([0.0, -v]))
            Fmax = 1e9
            for j, (name, ratio) in enumerate(((acts["hip"][leg], 1.0), (acts["knee"][leg], acts["ratio"][leg]))):
                a = ACT[name]; eff = BELT_EFF if ratio != 1 else 1.0
                w0 = a["w_free_48"] * BUS_V / 48 * 2 * np.pi / 60
                t_av = a["peak"] * max(0.0, 1 - abs(qd[j]) * ratio / w0) * ratio * eff
                if abs(J[1, j]) > 1e-6:
                    Fmax = min(Fmax, t_av / abs(J[1, j]))
            Fl[leg] = Fmax
        F_tot = min(2 * Fl["front"] / share("front"), 2 * Fl["rear"] / share("rear"))
        acc = F_tot / m_tot - G
        if acc <= 0 and v <= 1e-9:
            return 0.0, 0.0, t, stroke
        v = max(v + acc * dt, 0.0); dz += v * dt; t += dt
    return v ** 2 / (2 * G), v, t, stroke


def swing_time(key, leg, acts):
    """Fastest swing through the gallop stride arc, bang-bang at peak torque (pivot joint), est."""
    o = OPT[key]; inr = inertia(key, leg, acts["hip"][leg], acts["knee"][leg]); m = o["legs"][leg]
    L = GAITS["gallop"]["stride"]; h = o["pivot"][leg][1]
    dth = abs(m.ik(o["x0"][leg] + L / 2, -h)[0] - m.ik(o["x0"][leg] - L / 2, -h)[0])
    a = ACT[acts["hip"][leg]]
    return 2 * np.sqrt(dth * inr[0] / a["peak"]), inr


def eff_mass(key, leg, acts):
    o = OPT[key]; m = o["legs"][leg]; inr = inertia(key, leg, acts["hip"][leg], acts["knee"][leg])
    J = jac(m, m.ik(o["x0"][leg], -o["pivot"][leg][1]))
    return 1.0 / (J @ np.diag(1 / inr) @ J.T)[1, 1]


# ====================================================================== 6. horse-likeness
def horse_score(key):
    o = OPT[key]; sc = {}
    # proportions: segment fractions against the horse's (fore and hind), 1 - mean abs error / 0.25
    def frac(m):
        s = [m.l1, m.l2] + ([m.l3] if m.l3 > 0 else []); t = sum(s); return np.array(s) / t
    hf = np.array(HORSE["fore"]["ratio"]) / sum(HORSE["fore"]["ratio"]); hh = np.array(HORSE["hind"]["ratio"]) / sum(HORSE["hind"]["ratio"])
    err = []
    for leg, ref in (("front", hf), ("rear", hh)):
        f = frac(o["legs"][leg])
        err.append(np.mean(np.abs(f - ref)) if len(f) == 3 else 0.25)
    sc["segment proportions"] = float(max(0.0, 1 - np.mean(err) / 0.25))
    # joint directions: fore elbow back + carpus forward; hind stifle forward + hock back
    sc["joint directions (elbow back, carpus forward, stifle forward, hock back)"] = {"A": 0.75, "B": 1.0, "C": 0.5, "D": 0.75}[key]
    sc["foreleg is a straight column in stance (carpus locked)"] = 1.0 if o["carpus"] else 0.0
    sc["hind reciprocal apparatus (stifle and hock move together)"] = 0.0 if key == "C" else 1.0
    sc["fore and hind legs differ like a horse's"] = 1.0 if key == "B" else 0.0
    sc["hoof folds up under the body in swing"] = {"A": 0.5, "B": 1.0, "C": 0.3, "D": 0.5}[key]
    sc["sprung fetlock / pastern"] = 1.0
    return sc


# ====================================================================== 7. run
def clean(o):
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return float(o)
    if isinstance(o, (Panto,)):
        return dict(l1=o.l1, l2=o.l2, l3=o.l3, delta=o.delta * R2D, lmax=o.lmax)
    return o


def study():
    R = {}
    for key, o in OPT.items():
        acts, res = size(key)
        r = dict(name=o["name"], acts=acts, mass_total=M_TOTAL[key], legs_mass=legs_total(key, acts), res=res)
        usd = sum(2 * ACT[acts[j][leg]]["usd"] for j in ("hip", "knee", "abd") for leg in ("front", "rear") if acts[j][leg] in ACT)
        r["usd_act"] = usd
        if "none" not in [acts[j][l] for j in ("hip", "knee", "abd") for l in ("front", "rear")]:
            r["jump"] = max_jump(key, acts)
            r["swing"] = {leg: swing_time(key, leg, acts)[0] for leg in ("front", "rear")}
            r["inertia"] = {leg: swing_time(key, leg, acts)[1] for leg in ("front", "rear")}
            r["m_eff"] = {leg: eff_mass(key, leg, acts) for leg in ("front", "rear")}
        r["horse"] = horse_score(key); r["horse_total"] = sum(r["horse"].values())
        r["geom"] = {leg: dict(mech=o["legs"][leg], pivot=o["pivot"][leg], x0=o["x0"][leg]) for leg in ("front", "rear")}
        # margins per case (rated for core rms, peak for every case)
        mg = {}
        for leg in ("front", "rear"):
            for case, (rms, pk) in res[leg]["cases"].items():
                row = {}
                for j, jn in enumerate(("hip", "knee")):
                    a = ACT.get(acts[jn][leg]);
                    if a is None:
                        continue
                    ratio = acts["ratio"][leg] if jn == "knee" else 1.0; eff = BELT_EFF if ratio != 1 else 1.0
                    row[jn] = dict(rms=rms[j], peak=pk[j], m_rated=a["rated"] * ratio * eff / max(rms[j], 1e-6),
                                   m_peak=a["peak"] * ratio * eff / max(pk[j], 1e-6))
                mg[f"{leg} | {case}"] = row
        r["margins"] = mg
        R[key] = r
    return R


def report(R):
    p = print
    for key, r in R.items():
        o = OPT[key]
        p(f"\n=== {r['name']}   total {r['mass_total']:.1f} kg, four legs {r['legs_mass']:.2f} kg, actuators USD {r['usd_act']:.0f}")
        for leg in ("front", "rear"):
            m = o["legs"][leg]; g = r["geom"][leg]
            p(f"  {leg}: segments {m.l1 * 1e3:.0f} / {m.l2 * 1e3:.0f} / {m.l3 * 1e3:.0f} mm, delta {m.delta * R2D:.0f} deg, full {m.lmax * 1e3:.0f}, pivot ({g['pivot'][0] * 1e3:.0f}, {g['pivot'][1] * 1e3:.0f}), foot offset {g['x0'] * 1e3:.0f}")
            p(f"     actuators: pivot {r['acts']['hip'][leg]}, knee {r['acts']['knee'][leg]} (belt {r['acts']['ratio'][leg]:.1f}:1), abduction {r['acts']['abd'][leg]}")
            for case, (rms, pk) in r["res"][leg]["cases"].items():
                p(f"     {case:20s} pivot {rms[0]:6.2f} / {pk[0]:6.2f}   knee {rms[1]:6.2f} / {pk[1]:6.2f} N m (rms / peak)")
            p(f"     abduction rms / peak {r['res'][leg]['abd'][0]:.2f} / {r['res'][leg]['abd'][1]:.2f} N m; gallop joint speed {r['res'][leg]['speed'][0] * R2D:.0f} / {r['res'][leg]['speed'][1] * R2D:.0f} deg/s")
        if "jump" in r:
            p(f"  jump {r['jump'][0] * 1e3:.0f} mm (take-off {r['jump'][1]:.2f} m/s, push {r['jump'][2] * 1e3:.0f} ms over {r['jump'][3] * 1e3:.0f} mm); fastest gallop swing F/R {r['swing']['front'] * 1e3:.0f} / {r['swing']['rear'] * 1e3:.0f} ms;"
              f" inertia pivot/knee F {r['inertia']['front'][0] * 1e3:.1f}/{r['inertia']['front'][1] * 1e3:.1f} g m2; foot effective mass F/R {r['m_eff']['front']:.2f}/{r['m_eff']['rear']:.2f} kg")
        p(f"  horse score {r['horse_total']:.2f} / 7: " + ", ".join(f"{k.split(' (')[0]} {v:.2f}" for k, v in r["horse"].items()))
        worst = sorted((min(v["m_rated"] / 1.2 if c.split(' | ')[1] in CORE else 99, v["m_peak"] / 1.1), c, j) for c, row in r["margins"].items() for j, v in row.items())[:4]
        p("  tightest (margin / required): " + "; ".join(f"{c} {j} x{w:.2f}" for w, c, j in worst))
        p("  run height F/R (body drop to gallop): " + ", ".join(f"{leg} {h_run(key, leg, 'gallop') * 1e3:.0f} ({(OPT[key]['pivot'][leg][1] - h_run(key, leg, 'gallop')) * 1e3:.0f})" for leg in ("front", "rear"))
          + "; kneel pivot height F/R " + ", ".join(f"{kneel_height(key, leg) * 1e3:.0f}" for leg in ("front", "rear")))


if __name__ == "__main__":
    R = study()
    report(R)
    json.dump(clean(dict(R=R, gaits=GAITS, M0=M0, share=SHARE_F, horse=HORSE)), open(os.path.join(HERE, "legs_v3.json"), "w"), indent=1)
