"""Centaur design study, subsystem INTEG (2026-11-14): the octopus side of the centaur.
How Cephalo-1 gets on and off the horse (Equo-1), what its arms do as the centaur's arms, and the behaviours of the pair.

python3 integ_study.py  ->  integ.json, integ_1_mount.png, integ_2_arms.png, integ_3_behaviours.png
                            and all tables on stdout.  numpy + matplotlib only.  SI units in code.

ARM MODEL = STAND-IN.  The project file claude/code/cephalo4.py could not be read from this session (no Projects tool), so
the arm is the stand-in given in the task: 8 revolute joints at 56 mm pitch from the plate rim (r 125, 20 above the plate
underside), joint 0 yaws about the vertical (+-60 deg), joints 1..7 pitch (+-100 deg, positive = tip down), a hand or pod adds
100 mm, arm 0.75 kg spread evenly + 190 g at the tip, STS3250 at joints 0 and 1, STS3215 beyond.  Everything computed from
the arm kinematics, masses or servo ratings is therefore a stand-in result (marked "S" in the tables).
Hand estimates only: no CAD, no dynamics sim.  Everything marked (est.) is an estimate.
"""
import sys, json, time, warnings
warnings.filterwarnings("ignore", category=RuntimeWarning)
import numpy as np
sys.path.insert(0, "/home/claude/octodrone")
import sk3d
import matplotlib.pyplot as plt

T0 = time.time()
G = 9.81
RNG = np.random.default_rng(7)
OUT = "/home/claude/octodrone/centaur/integ/"
C = sk3d.COL

# ------------------------------------------------------------------ octopus (brief)
OCTO_M = 12.81
PLATE_R = 0.125
HUB_FACE = 0.034                 # hub pad face below the plate underside
STROKE_FULL = 0.194
TURRET_TOP = 0.224
XL_T_HOVER, XL_T_MAX, XL_KP, XL_R = 15.7, 26.5, 5.1, 0.095      # N per prop in hover on 8, max N, P = 5.1 W x T^1.5, disc r
HAND_R = 0.114                   # hand3 9 in rotor
PACK_WH = 2 * 5.0 * 22.2         # two 6S 5000 mAh
USABLE = 0.8
PIN_CAP = 980.0                  # N, eyelet bearing capacity (est., brief)
CAPTURE_XY, CAPTURE_YAW = 0.0076, 5.75      # hub pad cones (brief)
STATION = dict(calm=0.001, turbulence=0.014)  # hover station error (task)

# ------------------------------------------------------------------ arm stand-in (task prompt)
N_ARM = 8
ROOT_Z = 0.020
PITCH = 0.056
HAND = 0.100
ARM_LEN = 8 * PITCH              # 448: joint 0 .. bayonet
TCP_LEN = ARM_LEN + HAND         # 548: joint 0 .. fingertip / prop hub
ARM_M, POD_M = 0.75, 0.19
YAW_LIM, PITCH_LIM = np.radians(60.0), np.radians(100.0)
RATED = np.array([1.63, 1.63] + [0.98] * 6)
STALL = np.array([4.90, 4.90] + [2.94] * 6)
SERVO_SPEED = np.radians(60.0) / 0.222      # STS3215 no load, rad/s (270 deg/s)
ARM_AZ = np.radians(22.5 + 45.0 * np.arange(8))                 # in the HORSE frame (octopus-forward = horse-forward)
ARM_NAME = ["L1", "L2", "L3", "L4", "R4", "R3", "R2", "R1"]     # L1/R1 = the front pair at +-22.5 deg
CORE_M = OCTO_M - N_ARM * (ARM_M + POD_M)                       # body without the arms
CORE_Z = 0.080                   # CoM of the body core above the plate underside (est.)
R_CHAIN, R_POD = 0.022, 0.035    # collision radii: servo chain (half diagonal of 35 x 24.7), parked pod / closed hand (est.)

# ------------------------------------------------------------------ horse baseline (brief / centaur_study.py)
L_HIP, L1, L2 = 0.440, 0.190, 0.190
H_HIP = 0.95 * (L1 + L2)         # 0.361
H_BOW = 0.200                    # front hip height in the bow (task)
Y_HIP, Y_FOOT = 0.140, 0.170
BODY_X = (-0.500, 0.060); BODY_W = 0.150; BODY_Z = (0.310, 0.410)
X_NECK, Z_PLATE = -0.110, 0.534
COLLAR_R = 0.118
HIP_R, HIP_Y = 0.040, (0.075, 0.185)       # hip actuator cluster: dia 80, from the body side out past the leg (est.)
STRIDE = 0.140
HORSE_M, X_HORSE, Z_HORSE = 8.58, -0.250, 0.310
M_TOT = OCTO_M + HORSE_M
W_TOT = M_TOT * G
W_OCTO = OCTO_M * G
BASE_ZC, BASE_XC = 0.480, -0.166            # baseline centaur CoM (first pass)
# CyberGear (maker's manual, re-read 2026-11-14): 4 N m rated at 240 rpm, 12 N m peak, 296 rpm no load, 0.87 N m/Arms,
# 0.45 ohm line resistance; overload times: 12 N m 28 s, 10 N m 60 s, 8 N m 160 s, 6 N m 700 s, 5 N m 1800 s
CG = dict(rated=4.0, peak=12.0, n_rated=240.0, n0=296.0, kt=0.87, r_line=0.45,
          overload=[(12, 28), (11, 45), (10, 60), (9, 90), (8, 160), (7, 320), (6, 700), (5, 1800), (4.5, 2500)])
KNEE_RATIO, BELT_EFF = 2.0, 0.95
KNEE_RATED, KNEE_PEAK = CG["rated"] * KNEE_RATIO * BELT_EFF, CG["peak"] * KNEE_RATIO * BELT_EFF     # 7.6 / 22.8
BASE = dict(knee_trot_rms=5.5, knee_trot_peak=15.0, hip_trot_rms=3.8, hip_trot_peak=12.8, front_share=0.622,
            foot_front_stand=65.0, P_walk=125.0, getup_knee=11.1)       # first-pass numbers (centaur_study.py, option G)


def roty(a):                     # positive a = nose down
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rotx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def knee_xz(x, H, sign=1.0):
    d = np.hypot(x, H)
    a = (L1 ** 2 - L2 ** 2 + d ** 2) / (2 * d)
    h = np.sqrt(max(L1 ** 2 - a ** 2, 0.0))
    u = np.array([x, -H]) / d
    n = np.array([H, x]) / d * sign
    return a * u + h * n


# ================================================================== scenarios: where the octopus is, relative to the ground
def scen(hf=H_HIP, hr=H_HIP, neck=0.0, name=""):
    """Horse pose (front / rear hip height) + neck pitch (deg, nose down, pivot at the body top under the collar, an
    interface assumption).  Returns the octopus->standing-frame and octopus->world transforms and the leg boxes."""
    beta = np.arcsin((hr - hf) / L_HIP)
    Rb, Rn = roty(beta), roty(np.radians(neck))
    O = np.array([X_NECK, 0, Z_PLATE]); piv = np.array([X_NECK, 0, BODY_Z[1]])
    ts = piv + Rn @ (O - piv)
    base = np.array([0, 0, hf]); ref = np.array([0, 0, H_HIP])
    Rw = Rb @ Rn; tw = base + Rb @ (ts - ref)
    kf = knee_xz(0.0, hf, 1.0)[0]; kr = knee_xz(0.0, hr, 1.0)[0]
    xr = -L_HIP * np.cos(beta)
    legs = []                                                   # leg sweep boxes in the world: (lo, hi)
    for s in (1, -1):
        y = sorted([s * 0.110, s * 0.200])
        legs.append((np.array([-STRIDE / 2 - 0.03, y[0], 0.0]), np.array([max(kf, STRIDE / 2) + 0.03, y[1], hf])))
        legs.append((np.array([xr - max(kr, STRIDE / 2) - 0.03, y[0], 0.0]), np.array([xr + STRIDE / 2 + 0.03, y[1], hr])))
    return dict(name=name, hf=hf, hr=hr, neck=neck, beta=beta, Rs=Rn, ts=ts, Rw=Rw, tw=tw, Rb=Rb, base=base, ref=ref, legs=legs)


SCEN = {
    "stand":            scen(name="stand"),
    "stand, neck 15":   scen(neck=15, name="stand, neck 15"),
    "stand, neck 30":   scen(neck=30, name="stand, neck 30"),
    "bow":              scen(hf=H_BOW, name="bow"),
    "bow, neck 30":     scen(hf=H_BOW, neck=30, name="bow, neck 30"),
}
FREE = dict(name="free", Rw=np.eye(3), tw=np.zeros(3), free=True)      # octopus alone, level (frame = octopus frame)


# ================================================================== arm kinematics and statics (stand-in)
LINK = np.array([PITCH] * 6 + [PITCH + HAND])                    # links after joints 1..7 (the last one carries the hand)


def chain(psi, q, az):
    """psi (N,), q (N,7) [rad], az = root azimuth.  Octopus frame.  Returns J (N,9,3) = joints 0..7 + TCP,
    d (N,3) = heading of the arm plane, u (N,3) = tip axis (direction of the last link)."""
    psi = np.atleast_1d(psi); q = np.atleast_2d(q); n = len(psi)
    head = az + psi
    d = np.stack([np.cos(head), np.sin(head), np.zeros(n)], -1)
    e = -np.cumsum(q, axis=1)
    s = np.concatenate([np.zeros((n, 1)), np.full((n, 1), PITCH), PITCH + np.cumsum(np.cos(e) * LINK, 1)], 1)
    z = np.concatenate([np.full((n, 2), ROOT_Z), ROOT_Z + np.cumsum(np.sin(e) * LINK, 1)], 1)
    root = np.array([PLATE_R * np.cos(az), PLATE_R * np.sin(az), 0.0])
    J = root + s[..., None] * d[:, None, :] + z[..., None] * np.array([0, 0, 1.0])
    u = np.cos(e[:, -1:]) * d + np.sin(e[:, -1:]) * np.array([0, 0, 1.0])
    return J, d, u


def mass_points(J, u, tip_m=POD_M, payload=0.0):
    """Point masses of one arm: 8 link midpoints (0.75 kg evenly), the pod / hand 50 mm past the bayonet, payload at the TCP."""
    mids = 0.5 * (J[:, :7] + J[:, 1:8])                         # links 0..6
    last = J[:, 7] + 0.5 * PITCH * u                            # link 7 (joint 7 .. bayonet)
    tip = J[:, 7] + (PITCH + 0.5 * HAND) * u
    P = np.concatenate([mids, last[:, None], tip[:, None], J[:, 8:9]], 1)       # (N,10,3)
    m = np.array([ARM_M / 8] * 8 + [tip_m, payload])
    return P, m


def torques(J, d, u, R=np.eye(3), tip_m=POD_M, payload=0.0, F_ext=None):
    """Static joint torques (N,8) [N m], signed, about each joint axis.  R = octopus->world rotation (gravity is world -z).
    F_ext = force on the TCP in the world (3,) or (N,3)."""
    n = len(J)
    P, m = mass_points(J, u, tip_m, payload)
    Jw, Pw = J @ R.T, P @ R.T
    A = np.empty((n, 8, 3))
    A[:, 0] = R[:, 2]
    A[:, 1:] = (np.cross(np.array([0, 0, 1.0]), d) @ R.T)[:, None, :]
    gv = np.array([0, 0, -G])
    tau = np.empty((n, 8))
    for k in range(8):
        S = ((Pw[:, k:] - Jw[:, k:k + 1]) * m[k:, None]).sum(1)
        tv = np.cross(S, gv)
        if F_ext is not None:
            tv = tv + np.cross(Jw[:, 8] - Jw[:, k], np.broadcast_to(F_ext, (n, 3)))
        tau[:, k] = (A[:, k] * tv).sum(1)
    return tau


def ratio(tau):
    """Worst |torque| / rated and / stall over the 8 joints."""
    a = np.abs(tau)
    return (a / RATED).max(1), (a / STALL).max(1)


# ================================================================== collision model
def sd_box(P, lo, hi):
    c, h = 0.5 * (lo + hi), 0.5 * (hi - lo)
    q = np.abs(P - c) - h
    return np.linalg.norm(np.maximum(q, 0), axis=-1) + np.minimum(q.max(-1), 0)


def sd_cyl(P, c, r, half, axis):
    """Finite cylinder, axis along coordinate `axis`, centre c."""
    v = P - c
    ax = v[..., axis]
    rad = np.linalg.norm(np.delete(v, axis, axis=-1), axis=-1)
    dr, da = rad - r, np.abs(ax) - half
    return np.minimum(np.maximum(dr, da), 0) + np.hypot(np.maximum(dr, 0), np.maximum(da, 0))


BODY_LO = np.array([BODY_X[0], -BODY_W / 2, BODY_Z[0]]); BODY_HI = np.array([BODY_X[1], BODY_W / 2, BODY_Z[1]])
HIPS = [np.array([x, s * 0.5 * (HIP_Y[0] + HIP_Y[1]), H_HIP]) for x in (0.0, -L_HIP) for s in (1, -1)]


def sd_horse(Ps, Pw, sc):
    """Signed distance of points to the horse.  Ps = standing-frame coordinates (body-fixed solids), Pw = world (legs)."""
    d = sd_box(Ps, BODY_LO, BODY_HI)
    d = np.minimum(d, sd_cyl(Ps, np.array([X_NECK, 0, 0.5 * (BODY_Z[1] + Z_PLATE)]), COLLAR_R, 0.5 * (Z_PLATE - BODY_Z[1]), 2))
    for h in HIPS:
        d = np.minimum(d, sd_cyl(Ps, h, HIP_R, 0.5 * (HIP_Y[1] - HIP_Y[0]), 1))
    for lo, hi in sc["legs"]:
        d = np.minimum(d, sd_box(Pw, lo, hi))
    return d


def sd_octo(Po):
    """Octopus body in its own frame: plate + hub envelope, dome, turret (approximate distances)."""
    rho = np.hypot(Po[..., 0], Po[..., 1]); z = Po[..., 2]
    d = sd_cyl(Po, np.array([0, 0, -0.014]), PLATE_R, 0.020, 2)
    d = np.minimum(d, (np.sqrt((rho / 0.125) ** 2 + (np.maximum(z - 0.006, 0) / 0.150) ** 2) - 1) * 0.125 + (z < 0.006) * 1.0)
    d = np.minimum(d, sd_cyl(Po, np.array([0, 0, 0.162]), 0.090, 0.062, 2))
    return d


CHAIN_R = np.array([R_CHAIN] * 15 + [R_POD, 0.010])


def body_points(J, u):
    """Collision points along one arm: joints 2..7, link midpoints 1..6, three on the last link, pod centre, TCP."""
    pts = [J[:, k] for k in range(2, 8)] + [0.5 * (J[:, k] + J[:, k + 1]) for k in range(1, 7)]
    pts += [J[:, 7] + f * u for f in (0.028, 0.056, 0.081)]
    pts += [J[:, 7] + (PITCH + 0.5 * HAND) * u, J[:, 8]]
    return np.stack(pts, 1)


def shape_gap(J, u, P=None):
    """Gap of an arm to the octopus body and to itself (does not depend on the horse)."""
    P = body_points(J, u) if P is None else P
    g = (sd_octo(P[:, 2:]) - CHAIN_R[2:]).min(1)
    for i in range(1, 6):                                       # chain against itself (joints three or more apart)
        for j in range(i + 3, 9):
            g = np.minimum(g, np.linalg.norm(J[:, i] - J[:, j], axis=1) - 2 * R_CHAIN)
    return g


def clearance(J, u, sc, ground=True, sg=None):
    """Smallest gap [m] between one arm and the horse / the ground / the octopus body / itself.  (N,)
    J, u in the octopus frame (arm azimuth already applied)."""
    P = body_points(J, u)
    g = shape_gap(J, u, P) if sg is None else sg.copy()
    Pw = P @ sc["Rw"].T + sc["tw"]
    if not sc.get("free"):
        Ps = P @ sc["Rs"].T + sc["ts"]
        g = np.minimum(g, (sd_horse(Ps, Pw, sc) - CHAIN_R).min(1))
    if ground:
        g = np.minimum(g, (Pw[..., 2] - CHAIN_R).min(1))
    return g


def to_world(X, sc):
    return X @ sc["Rw"].T + sc["tw"]


# ================================================================== joint-space sampling and local search
def sample(n):
    """Mix of uniform, smooth and sparse-bend arm shapes inside the joint limits."""
    n3 = n // 3
    qa = RNG.uniform(-PITCH_LIM, PITCH_LIM, (n3, 7))
    k = np.arange(7)
    qb = (RNG.uniform(-1.2, 1.2, (n3, 1)) + RNG.uniform(-0.6, 0.6, (n3, 1)) * (k - 3) / 3 + RNG.normal(0, 0.25, (n3, 7)))
    qc = RNG.normal(0, 0.12, (n - 2 * n3, 7))
    for _ in range(3):
        idx = RNG.integers(0, 7, len(qc))
        qc[np.arange(len(qc)), idx] = RNG.uniform(-PITCH_LIM, PITCH_LIM, len(qc))
    q = np.clip(np.concatenate([qa, qb, qc]), -PITCH_LIM, PITCH_LIM)
    psi = RNG.uniform(-YAW_LIM, YAW_LIM, n)
    return np.column_stack([psi, q])


X_LO = np.array([-YAW_LIM] + [-PITCH_LIM] * 7); X_HI = -X_LO


def hill(cost, x0, iters=40, batch=200, s0=0.30):
    """Stochastic hill climb on x = (psi, q1..q7).  cost(X (n,8)) -> (n,)."""
    x = np.array(x0, float); c = cost(x[None])[0]
    for it in range(iters):
        s = s0 * 0.90 ** it
        dX = RNG.normal(0, s, (batch, 8))
        dX[: batch // 2] *= (RNG.random((batch // 2, 8)) < 0.3)             # half of the trials move only a few joints
        X = np.clip(x + dX, X_LO, X_HI)
        cc = cost(X); i = int(np.argmin(cc))
        if cc[i] < c:
            x, c = X[i], cc[i]
    return x, c


CLOUD = None            # shared joint-space sample (set in main)
PRE = {}                # per (arm, scenario): which samples are collision-free and where their TCP is


def solve(arm, sc, target, payload=0.0, tip_m=POD_M, axis=None, axis_deg=60.0, F_ext=None, starts=3, fixed_psi=None,
          free_x=None, pre=None, want="rated"):
    """Find the arm shape that puts the TCP on `target` (world) with the lowest worst torque / rated.
    axis = required tip axis direction in the world (within axis_deg).  free_x = (lo, hi): x of the target is free in that band.
    pre = entry of PRE for this arm and scenario (speeds up the start search).  Returns a dict."""
    az = ARM_AZ[ARM_NAME.index(arm)]
    target = np.asarray(target, float)

    def parts(X):
        psi = X[:, 0] if fixed_psi is None else np.full(len(X), fixed_psi)
        J, d, u = chain(psi, X[:, 1:], az)
        tw = to_world(J[:, 8], sc)
        dv = tw - target
        if free_x is not None:
            dv[:, 0] = np.maximum(free_x[0] - tw[:, 0], 0) + np.maximum(tw[:, 0] - free_x[1], 0)
        err = np.linalg.norm(dv, axis=1)
        tau = torques(J, d, u, sc["Rw"], tip_m, payload, F_ext)
        r, s = ratio(tau)
        gap = clearance(J, u, sc)
        ax = np.zeros(len(X))
        if axis is not None:
            ca = np.clip((u @ sc["Rw"].T) @ np.asarray(axis, float), -1, 1)
            ax = np.maximum(np.degrees(np.arccos(ca)) - axis_deg, 0)
        return err, r, s, gap, ax, tau, tw

    def cost(X):
        err, r, s, gap, ax, _, _ = parts(X)
        return r + (err / 0.004) ** 2 + (ax / 3.0) ** 2 + 1e3 * np.maximum(0.004 - gap, 0) / 0.004

    if pre is not None and fixed_psi is None:
        dv = pre["tw"] - target
        if free_x is not None:
            dv[:, 0] = np.maximum(free_x[0] - pre["tw"][:, 0], 0) + np.maximum(pre["tw"][:, 0] - free_x[1], 0)
        e0 = np.linalg.norm(dv, axis=1) + 9.0 * (~pre["ok"])
        cand = CLOUD[np.argsort(e0)[:1500]]
    else:
        cand = CLOUD[:12000]
    err, r, s, gap, ax, _, _ = parts(cand)
    c0 = r + (err / 0.03) ** 2 + (ax / 10.0) ** 2 + 1e3 * (gap < 0.004)
    best = None
    for i in np.argsort(c0)[:starts]:
        x, c = hill(cost, cand[i])
        if best is None or c < best[1]:
            best = (x, c)
    x = best[0].copy()
    err, r, s, gap, ax, tau, tw = parts(x[None])
    if fixed_psi is not None:
        x[0] = fixed_psi
    return dict(x=x, err=float(err[0]), rated=float(r[0]), stall=float(s[0]), gap=float(gap[0]), tau=tau[0], tcp=tw[0],
                worst=int(np.argmax(np.abs(tau[0]) / RATED)), ok=bool(err[0] < 0.006 and gap[0] > 0.003 and ax[0] < 1.0))


def force_cap(arm, sc, x, dirs):
    """Largest steady force [N] the hand can take from outside along each world direction with every joint inside RATED."""
    az = ARM_AZ[ARM_NAME.index(arm)]
    J, d, u = chain(x[:1], x[None, 1:], az)
    t0 = torques(J, d, u, sc["Rw"])[0]
    out = {}
    for name, v in dirs.items():
        c = torques(J, d, u, sc["Rw"], F_ext=np.asarray(v, float))[0] - t0
        with np.errstate(divide="ignore", invalid="ignore"):
            f = np.where(np.abs(c) > 1e-9, (RATED * np.sign(c) - t0) / c, np.inf)
        out[name] = float(max(f.min(), 0.0))
    return out


# ================================================================== D1a  reach envelope of the working arms
def reach_study():
    """Sampled workspace of the front-left arm (L1) and the next arm (L2) in each horse pose."""
    psi, q = CLOUD[:, 0], CLOUD[:, 1:]
    out = {}
    for an in ("L1", "L2"):
        az = ARM_AZ[ARM_NAME.index(an)]
        J, d, u = chain(psi, q, az)
        sg = shape_gap(J, u)
        for sn, sc in SCEN.items():
            if an == "L2" and sn not in ("stand", "bow"):
                continue
            ok = clearance(J, u, sc, sg=sg) > 0.004
            tw = to_world(J[:, 8], sc); uw = u @ sc["Rw"].T
            down = np.degrees(np.arccos(np.clip(-uw[:, 2], -1, 1)))            # tip axis from straight down
            pick = ok & (tw[:, 2] < 0.035) & (down < 60)
            touch = ok & (tw[:, 2] < 0.035)

            def cz(X, sc=sc, az=az):
                Jx, dx, ux = chain(X[:, 0], X[:, 1:], az)
                return to_world(Jx[:, 8], sc)[:, 2] + 1e3 * np.maximum(0.004 - clearance(Jx, ux, sc), 0)
            i0 = np.argsort(np.where(ok, tw[:, 2], 9))[:2]
            zmin = min(hill(cz, CLOUD[i], iters=60, s0=0.15)[1] for i in i0)
            cell = 0.02
            cells = set(map(tuple, np.floor(tw[pick][:, :2] / cell).astype(int)))
            both = cells | {(a, -b - 1) for a, b in cells}
            fwd = ok & (np.abs(tw[:, 1]) < 0.20)
            out[f"{an} {sn}"] = dict(
                z_min=float(zmin), ground=bool(zmin < 0.020), touch=bool(touch.any()), pick=bool(pick.any()),
                pick_area_cm2=len(both) * cell * cell * 1e4,
                pick_x=(float(tw[pick][:, 0].min()), float(tw[pick][:, 0].max())) if pick.any() else None,
                pick_y=float(np.abs(tw[pick][:, 1]).max()) if pick.any() else None,
                x_max=float(tw[ok][:, 0].max()), y_max=float(tw[ok][:, 1].max()), z_max=float(tw[ok][:, 2].max()),
                x_max_chest=float(tw[fwd & (np.abs(tw[:, 2] - 0.45) < 0.03)][:, 0].max()))
            PRE[(an, sn)] = dict(ok=ok, tw=tw, pick=pick)
    return out


# ================================================================== D1b  payload map of one working arm
def payload_map(cell=0.025):
    """Largest payload [kg] one arm can hold at each hand position with every joint inside its RATED (and its STALL) torque.
    Front-left arm pointing straight ahead (plane y = +48 mm), horse standing.  Best sampled shape per cell."""
    az = ARM_AZ[0]; sc = SCEN["stand"]
    psi = np.full(len(CLOUD), -az)                              # heading 0 = straight ahead
    J, d, u = chain(psi, CLOUD[:, 1:], az)
    ok = clearance(J, u, sc) > 0.004
    J, d, u = J[ok], d[ok], u[ok]
    t0 = torques(J, d, u, sc["Rw"])                             # own weight
    t1 = torques(J, d, u, sc["Rw"], payload=1.0) - t0           # per kg at the TCP
    tw = to_world(J[:, 8], sc)

    def cap(lim):
        with np.errstate(divide="ignore", invalid="ignore"):
            hi = np.where(t1 > 1e-9, (lim - t0) / t1, np.where(t1 < -1e-9, (-lim - t0) / t1, np.inf))
            lo = np.where(t1 > 1e-9, (-lim - t0) / t1, np.where(t1 < -1e-9, (lim - t0) / t1, -np.inf))
            dead = (np.abs(t1) <= 1e-9) & (np.abs(t0) > lim)
        m_hi, m_lo = hi.min(1), lo.max(1)
        return np.where((m_lo <= m_hi) & (m_hi >= 0) & ~dead.any(1), m_hi, -1.0), (m_lo <= 0) & (m_hi >= 0)
    capR, selfR = cap(RATED); capS, _ = cap(STALL)
    ix = np.floor((tw[:, 0] + 0.2) / cell).astype(int); iz = np.floor(tw[:, 2] / cell).astype(int)
    nx, nz = int(0.9 / cell), int(1.1 / cell)
    grid = {}
    good = (ix >= 0) & (ix < nx) & (iz >= 0) & (iz < nz)
    for name, c in (("rated", capR), ("stall", capS)):
        g = np.full((nz, nx), -2.0)
        np.maximum.at(g, (iz[good], ix[good]), c[good])
        g[g < -1.5] = np.nan
        grid[name] = g
    ext = (-0.2, -0.2 + nx * cell, 0.0, nz * cell)
    rows = []
    band = (np.abs(tw[:, 2] - 0.45) < 0.04)
    for mp in (0.0, 0.25, 0.5, 1.0, 1.5, 2.0):
        xr = tw[band & (capR >= mp)][:, 0]; xs = tw[band & (capS >= mp)][:, 0]
        rows.append(dict(payload=mp, x_rated=float(xr.max()) if len(xr) else None, x_stall=float(xs.max()) if len(xs) else None))
    return dict(grid=grid, extent=ext, cell=cell, rows=rows)


# ================================================================== D1c  task poses and their servo loads
def straight_arm():
    """Arm held straight out, level: the worst case that sizes the root pitch servos."""
    J, d, u = chain(np.zeros(1), np.zeros((1, 7)), 0.0)
    rows = []
    for mp in (0.0, 0.5, 1.0):
        t = np.abs(torques(J, d, u, payload=mp)[0])
        rows.append(dict(payload=mp, j1=float(t[1]), j2=float(t[2]), j1_rated=float(t[1] / RATED[1]), j1_stall=float(t[1] / STALL[1]),
                         j2_rated=float(t[2] / RATED[2]), j2_stall=float(t[2] / STALL[2])))
    return rows


def task_poses(zmin_stand):
    T = []

    def add(name, sn, target, axis=None, free_x=None, arm="L1", payloads=(0.0, 0.5, 1.0)):
        sc = SCEN[sn]
        rec = dict(task=name, scen=sn, arm=arm, rows=[])
        for mp in payloads:
            r = solve(arm, sc, target, payload=mp, axis=axis, free_x=free_x, pre=PRE.get((arm, sn)))
            rec["rows"].append(dict(payload=mp, rated=r["rated"], stall=r["stall"], err=r["err"], gap=r["gap"], ok=r["ok"],
                                    worst=r["worst"], tau=[float(v) for v in r["tau"]], x=r["x"].tolist(), tcp=r["tcp"].tolist()))
        T.append(rec)
    dn = (0, 0, -1)
    add("hold at the chest", "stand", (0.22, 0.10, 0.45))
    add("reach forward", "stand", (0.40, 0.10, 0.45))
    add("high reach", "stand", (0.25, 0.10, 0.75))
    add("lowest front reach", "stand", (0.20, 0.10, zmin_stand + 0.010), free_x=(0.10, 0.40))
    add("ground pick", "stand, neck 30", (0.25, 0.10, 0.02), axis=dn, free_x=(0.12, 0.45))
    add("ground pick", "bow", (0.30, 0.10, 0.02), axis=dn, free_x=(0.15, 0.50))
    add("helper hand at the chest", "stand", (0.22, 0.17, 0.45), arm="L2", payloads=(0.0, 0.5))
    add("side reach", "stand", (-0.05, 0.50, 0.35), arm="L2", payloads=(0.0, 0.5))
    return T


# ================================================================== D1d  what the other arms do: rest poses
def deg(*a):
    return np.radians(np.array(a, float))


TULIP = (0.0, deg(*[-90.0 / 7] * 7))                                    # flight posture: smooth rise to a vertical tip axis (stand-in)
CANDLE = (0.0, deg(-55, -7, -7, -7, -7, -7, 0))                         # steep "guard" posture: discs just clear of each other
HANG = (0.0, deg(90, 0, 0, 0, 0, 0, 0))                                 # straight down from joint 1
FLANK = (np.radians(-22.5), deg(90, 0, 0, 0, 0, -45, -45))              # L3: down the flank, tip curled out and up (clear of a bow)
TAIL = (np.radians(22.5), deg(60, 30, -90, 0, 0, 0, 70))                # L4: down onto the back, along it, tip hanging behind the croup
GIRTH_L = (np.radians(-2.5), deg(90, 0, 0, 0, 0, 45, 45))               # L3: down the flank and under the belly (heading 110 deg)
GIRTH_R = (np.radians(42.5), deg(90, 0, 0, 0, 0, 45, 45))               # R3: the same, heading -70 deg, so the tips pass 96 mm apart
BACK = (np.radians(22.5), deg(0, 0, 0, 0, 0, 0, 0))                     # L4: straight back and level (counterweight)
BACK3 = (np.radians(60.0), deg(0, 0, 0, 0, 0, 0, 0))                    # L3: as far back as its yaw allows, level
WORK = None                                                             # front arm at rest: set in main (solved)


def mirror(i):
    return 7 - i


def set_pose(poses):
    """poses: {arm name: (psi, q)} for the left side; mirrored to the right (psi sign flips)."""
    full = {}
    for n, (psi, q) in poses.items():
        if n.startswith("L"):
            i = ARM_NAME.index(n)
            full[i] = (psi, q); full[mirror(i)] = (-psi, q)
    for n, (psi, q) in poses.items():                          # explicit right-side shapes override the mirror
        if n.startswith("R"):
            full[ARM_NAME.index(n)] = (psi, q)
    return full


def pose_report(poses, sc=None, rested=()):
    """Centaur CoM, tip angles, worst holding torque and the prop discs for one set of arm shapes.
    rested = arms lying on the horse (their servos are relaxed, so they are left out of the worst holding torque)."""
    sc = SCEN["stand"] if sc is None else sc
    full = set_pose(poses)
    mx = np.array([CORE_M * X_NECK, 0.0, CORE_M * (Z_PLATE + CORE_Z)]) + HORSE_M * np.array([X_HORSE, 0, Z_HORSE])
    pts = []
    arms = {}
    for i in range(8):
        psi, q = full[i]
        J, d, u = chain(np.array([psi]), q[None], ARM_AZ[i])
        P, m = mass_points(J, u)
        Pw = to_world(P[0], sc)
        mx = mx + (Pw * m[:, None]).sum(0)
        pts.append((Pw, m))
        tau = torques(J, d, u, sc["Rw"])
        r, s = ratio(tau)
        gap = clearance(J, u, sc)[0]
        Jw = to_world(J[0], sc); uw = u[0] @ sc["Rw"].T
        rd = HAND_R if ARM_NAME[i] in ("L1", "R1") else XL_R
        e1 = np.cross(uw, [0, 0, 1.0]); e1 = e1 / np.linalg.norm(e1) if np.linalg.norm(e1) > 1e-6 else np.array([1.0, 0, 0])
        e2 = np.cross(uw, e1)
        th = np.linspace(0, 2 * np.pi, 49)[:-1]
        rim = Jw[8] + rd * (np.cos(th)[:, None] * e1 + np.sin(th)[:, None] * e2)
        rim_s = (rim - sc["tw"]) @ sc["Rw"] @ sc["Rs"].T + sc["ts"]
        dgap = float(min(sd_horse(rim_s, rim, sc).min(), rim[:, 2].min()))
        arms[ARM_NAME[i]] = dict(rated=float(r[0]), stall=float(s[0]), gap=float(gap), disc=Jw[8].tolist(), axis=uw.tolist(),
                                 disc_gap=dgap, disc_r=rd, J=Jw, tilt=float(np.degrees(np.arccos(np.clip(uw[2], -1, 1)))),
                                 tau=[float(v) for v in tau[0]], rested=ARM_NAME[i] in rested)
    com = mx / M_TOT
    Ixx = CORE_M * 0.09 ** 2 + CORE_M * ((Z_PLATE + CORE_Z - com[2]) ** 2) + HORSE_M * (0.13 ** 2 + (Z_HORSE - com[2]) ** 2)
    for Pw, m in pts:
        Ixx += (m * (Pw[:, 1] ** 2 + (Pw[:, 2] - com[2]) ** 2)).sum()
    octo_z = (CORE_M * CORE_Z + sum((m * (Pw[:, 2] - Z_PLATE)).sum() for Pw, m in pts)) / OCTO_M
    up = [a for a in arms.values() if a["tilt"] < 30]
    return dict(com=com.tolist(), octo_com_above_plate=float(octo_z), Ixx=float(Ixx),
                tip_side=float(np.degrees(np.arctan(Y_FOOT / com[2]))), tip_front=float(np.degrees(np.arctan(-com[0] / com[2]))),
                tip_rear=float(np.degrees(np.arctan((L_HIP + com[0]) / com[2]))), front_share=float(1 + com[0] / L_HIP),
                worst_rated=max(a["rated"] for a in arms.values() if not a["rested"]),
                worst_gap=min(a["gap"] for a in arms.values()),
                props_up=len(up), min_disc_gap=(min(a["disc_gap"] for a in up) if up else None),
                top=max(float(a["J"][:, 2].max()) for a in arms.values()), arms=arms, full=full)


def rest_poses():
    R = {}
    R["A crown (six in the flight tulip)"] = pose_report({"L1": WORK, "L2": TULIP, "L3": TULIP, "L4": TULIP})
    R["B guard (six held steep)"] = pose_report({"L1": WORK, "L2": CANDLE, "L3": CANDLE, "L4": CANDLE})
    R["C pack (four folded on the horse)"] = pose_report({"L1": WORK, "L2": CANDLE, "L3": GIRTH_L, "R3": GIRTH_R, "L4": TAIL}, rested=("L4", "R4"))
    R["C2 pack, flank arms curled outward"] = pose_report({"L1": WORK, "L2": CANDLE, "L3": FLANK, "L4": TAIL}, rested=("L4", "R4"))
    R["take-off (all eight in the tulip)"] = pose_report({"L1": TULIP, "L2": TULIP, "L3": TULIP, "L4": TULIP})
    R["counterweight (rear four straight back)"] = pose_report({"L1": WORK, "L2": CANDLE, "L3": BACK3, "L4": BACK})
    # strap force of a girth hug: what the hooked tip can pull up on the belly before a tip joint passes its rating
    lever = PITCH + HAND
    R["_girth_tie"] = dict(rated=RATED[7] / lever, stall=STALL[7] / lever, pin_cap=PIN_CAP)
    # tips of the two tucked arms under the belly
    f = R["C pack (four folded on the horse)"]["arms"]
    R["_girth_tips_apart"] = float(np.linalg.norm(np.array(f["L3"]["disc"]) - np.array(f["R3"]["disc"])))
    # outriggers: side arm to the ground beside the horse.  Standing it does not reach (see the reach table); crouched it does
    sc = scen(hf=H_BOW, hr=H_BOW)
    r = solve("L2", sc, (-0.05, 0.45, 0.016), starts=2)
    fc = force_cap("L2", sc, r["x"], {"up": (0, 0, 1)})
    R["_outrigger"] = dict(ok=r["ok"], y=0.45, force_rated=fc["up"], moment_rated=2 * fc["up"] * (0.45 - Y_FOOT),
                           tip_moment=W_TOT * Y_FOOT)
    return R


# ================================================================== D2  mount and dismount
def scen_free(h_hip, z_plate, name=""):
    """Octopus NOT docked: level, plate underside at z_plate above the ground, on the neck axis; horse level at hip height h_hip."""
    s = scen(hf=h_hip, hr=h_hip)
    drop = H_HIP - h_hip
    s.update(name=name, Rs=np.eye(3), ts=np.array([X_NECK, 0, z_plate + drop]), Rw=np.eye(3), tw=np.array([X_NECK, 0, z_plate]))
    return s


H_LIE = H_HIP - BODY_Z[0]                   # hip height with the belly on the ground (51 mm)
CLEAR_HUB = HUB_FACE + 0.020                # plate must be this far above the collar top before it comes down (hub + 20 mm)
COLLAR_TOP = {"stand": Z_PLATE, "crouch (hips 200)": Z_PLATE - (H_HIP - H_BOW), "lie (belly down)": Z_PLATE - BODY_Z[0]}
FEET = {"L1": (0.200, 0.060), "L2": (-0.030, 0.270), "L3": (-0.200, 0.270), "L4": (-0.330, 0.270)}   # eight feet astride the horse (world x, y)
GRIP = np.array([0.090, 0.060, 0.370])      # breast bar: across the chest, 30 mm ahead of the body front (interface assumption)
# durations (all est.)
ELEV_SPEED = 0.025                          # m/s, bars on the elevator (STS3215 on GT2 loops)
T_BARS = STROKE_FULL / ELEV_SPEED
T_LATCH = 0.3                               # release ring (STS3032)
T_SPIN = 0.25                               # props idle -> hover thrust
T_DETECT = 0.05                             # IMU + filter
OCTO_WALK = 0.05                            # m/s on its arms
T_HORSE_UPDOWN = 3.0                        # lie down or stand up (0.31 m)
V_DESCEND = 0.10                            # m/s, last part of a flown approach


def stance(sc, n_feet=8, skip=()):
    """Octopus standing on its arms astride the horse: worst torque ratio per arm."""
    out = {}
    for an, (x, y) in FEET.items():
        if an in skip:
            continue
        r = solve(an, sc, (x, y, 0.016), F_ext=np.array([0, 0, W_OCTO / n_feet]), starts=2)
        out[an] = dict(rated=r["rated"], stall=r["stall"], gap=r["gap"], ok=r["ok"], x=r["x"], worst=r["worst"])
    return out


def stand_height(rho=0.28, n_feet=8):
    """Octopus alone on level ground: how high it can hold its plate with its feet on a circle of radius rho."""
    rows = []
    for h in (0.25, 0.30, 0.35, 0.40, 0.43, 0.46):
        sc = dict(name="free", Rw=np.eye(3), tw=np.array([0, 0, h]), free=True)
        r = solve("L1", sc, (rho * np.cos(ARM_AZ[0]), rho * np.sin(ARM_AZ[0]), 0.016), F_ext=np.array([0, 0, W_OCTO / n_feet]),
                  fixed_psi=0.0, starts=2)
        rows.append(dict(h=h, rated=r["rated"], stall=r["stall"], ok=r["ok"]))
    geo = float(np.sqrt((TCP_LEN - PITCH) ** 2 - (rho - PLATE_R - PITCH) ** 2) - ROOT_Z)
    return dict(rho=rho, n=n_feet, rows=rows, h_geo=geo)


def wash(arms, T_each):
    """Share of each prop's wake that lands on the horse (plan view), and the download.  Wake radius 0.75 R (est.)."""
    rects = [(BODY_X[0], BODY_X[1], -BODY_W / 2, BODY_W / 2)]
    for x in (0.0, -L_HIP):
        for s in (1, -1):
            y = sorted([s * HIP_Y[0], s * 0.200])
            rects.append((x - 0.05, x + 0.05, y[0], y[1]))
    g = np.linspace(-1, 1, 61); gx, gy = np.meshgrid(g, g); m = gx ** 2 + gy ** 2 <= 1
    tot = 0.0; rows = {}
    for n, a in arms.items():
        if a["tilt"] > 30:
            continue
        rw = 0.75 * a["disc_r"]
        px = a["disc"][0] + rw * gx[m]; py = a["disc"][1] + rw * gy[m]
        hit = np.zeros(len(px), bool)
        for x0, x1, y0, y1 in rects:
            hit |= (px > x0) & (px < x1) & (py > y0) & (py < y1)
        rows[n] = float(hit.mean()); tot += hit.mean() * T_each
    return dict(share=rows, download=float(tot), n=len(rows))


def mount_study(rp):
    M = dict(collar_top=COLLAR_TOP, clear_hub=CLEAR_HUB, plate_needed={k: v + CLEAR_HUB for k, v in COLLAR_TOP.items()})
    M["stand_height"] = stand_height()
    # (a) horse lies down, octopus walks over it on six arms
    M["a_over"] = stance(scen_free(H_LIE, COLLAR_TOP["lie (belly down)"] + CLEAR_HUB))
    M["a_six"] = stance(scen_free(H_LIE, COLLAR_TOP["lie (belly down)"] + CLEAR_HUB), n_feet=6, skip=("L1",))     # hands on the bar
    sc_l = scen_free(H_LIE, COLLAR_TOP["lie (belly down)"] + CLEAR_HUB)
    gl = GRIP - np.array([0, 0, BODY_Z[0]])
    r = solve("L1", sc_l, gl, starts=3)
    M["a_grip"] = dict(rated=r["rated"], gap=r["gap"], ok=r["ok"], x=r["x"])
    sc_s = scen_free(H_LIE, COLLAR_TOP["lie (belly down)"])
    M["a_seated"] = stance(sc_s, n_feet=6, skip=("L1",))
    r = solve("L1", sc_s, gl, starts=3)
    M["a_grip_seated"] = dict(rated=r["rated"], gap=r["gap"], ok=r["ok"], x=r["x"])
    # (d) / quick variant: horse only crouches
    M["d_over"] = stance(scen_free(H_BOW, COLLAR_TOP["crouch (hips 200)"] + CLEAR_HUB))
    # (c) two hands on the breast bar while the octopus hovers dz above its seat
    dirs = {"fore-aft": (1, 0, 0), "aft": (-1, 0, 0), "left": (0, 1, 0), "right": (0, -1, 0), "down": (0, 0, -1)}
    M["c_grip"] = []
    for dz in (0.20, 0.10, CLEAR_HUB, 0.0):
        sc = scen_free(H_HIP, Z_PLATE + dz)
        r = solve("L1", sc, GRIP, starts=3)
        f = force_cap("L1", sc, r["x"], dirs)
        M["c_grip"].append(dict(dz=dz, rated=r["rated"], gap=r["gap"], ok=r["ok"], x=r["x"],
                                pull_side=min(f["fore-aft"], f["aft"], f["left"], f["right"]), pull_down=f["down"]))
    # flown approach numbers
    tk = rp["take-off (all eight in the tulip)"]
    M["tulip_disc_gap"] = tk["min_disc_gap"]; M["tulip_disc_z"] = tk["arms"]["L2"]["disc"][2]
    M["wash_hover"] = wash(tk["arms"], XL_T_HOVER)
    zd = tk["arms"]["L4"]["disc"][2]
    M["ground_effect"] = dict(over_ground=1 / (1 - (XL_R / (4 * zd)) ** 2), over_back=1 / (1 - (XL_R / (4 * (zd - BODY_Z[1]))) ** 2))
    M["P_hover"] = 8 * XL_KP * XL_T_HOVER ** 1.5
    M["TW"] = 8 * XL_T_MAX / W_OCTO
    M["capture"] = dict(xy=CAPTURE_XY, yaw=CAPTURE_YAW, station=STATION, funnel_needed=STATION["turbulence"] + 0.006)
    M["arm_backlash_mm"] = 1e3 * np.radians(0.3) * np.sqrt(8) * 0.30        # 0.3 deg per joint (est.), 8 joints rss, 0.3 m lever
    M["gust_force"] = 0.5 * 1.225 * 5.0 ** 2 * 0.08 * 1.0                   # 5 m/s gust on 0.08 m2 of octopus, Cd 1 (est.)
    # horse getting up from lying with the octopus on board (knee, foot under the hip)
    lev = np.sqrt(L1 ** 2 - (H_LIE / 2) ** 2)
    kt = BASE["foot_front_stand"] * lev
    M["getup"] = dict(knee=kt, of_rated=kt / KNEE_RATED, of_peak=kt / KNEE_PEAK, motor=kt / (KNEE_RATIO * BELT_EFF),
                      knee_horse_alone=HORSE_M * G * (1 + X_HORSE / L_HIP) / 2 * lev)
    # sequences: (step, seconds, sensed end condition)
    walk_d = 0.45
    M["seq_ground"] = [
        ("Horse lies down, legs folded", T_HORSE_UPDOWN, "belly switches closed, knee currents near zero"),
        ("Octopus walks astride the chest (450 mm)", walk_d / OCTO_WALK, "back tag centred within 10 mm (belly camera)"),
        ("Two hands take the breast bar", 2.0, "finger currents rise, both hands closed"),
        ("Body lowers 54 mm onto the collar", 3.0, "three collar seat switches closed"),
        ("Bars slide down 194 mm", T_BARS, "elevator at full stroke, four latch pins clicked"),
        ("Pull test, link check, arms to pack", 2.0, "collar CAN heartbeat, both IMUs agree"),
        ("Horse stands up", T_HORSE_UPDOWN, "hip height 361, roll and pitch under 2 deg"),
    ]
    M["seq_air"] = [
        ("Horse stands square and holds still", 1.0, "horse IMU quiet, link by radio"),
        ("Octopus flies in and hovers 200 mm above the seat", 5.0, "back tag centred within 20 mm"),
        ("Descends to 100 mm, two hands take the breast bar", 0.10 / V_DESCEND + 2.0, "both hands closed, arm loads steady"),
        ("Arms pull in while thrust eases; plate onto the collar", 0.10 / V_DESCEND + 1.0, "three seat switches closed"),
        ("Props stop; bars slide down 194 mm", T_BARS, "four latch pins clicked"),
        ("Pull test, link check, arms to pack", 2.0, "collar CAN heartbeat, both IMUs agree"),
    ]
    M["seq_takeoff"] = [
        ("Horse stands square, legs stiff; all eight arms to the tulip", 1.5, "arm angles in the tulip set, discs clear"),
        ("Two hands back on the breast bar; props to idle", 1.5, "eight ESC rpm reports"),
        ("Release ring pulls the four pins", T_LATCH, "four pin-out switches"),
        ("Bars slide up 194 mm", T_BARS, "elevator at stow, bar-stowed switch"),
        ("Thrust ramps to 90 % of weight", 1.0, "horse knee currents fall by the octopus's share"),
        ("Lift 54 mm, hands guide, then let go", 0.5, "seat switches open, hub clear of the collar"),
        ("Climb away", 1.0, "300 mm above the horse"),
    ]
    for k in ("seq_ground", "seq_air", "seq_takeoff"):
        M[k + "_total"] = float(sum(s[1] for s in M[k]))
    t_fly = sum(s[1] for s in M["seq_air"][1:4])
    M["air_energy_wh"] = M["P_hover"] * t_fly / 3600
    t_to = sum(s[1] for s in M["seq_takeoff"][4:])
    M["takeoff_energy_wh"] = M["P_hover"] * t_to / 3600
    # lift-off: time to clear the collar and room for drift
    a = 0.2 * G
    M["liftoff"] = dict(t_clear=float(np.sqrt(2 * CLEAR_HUB / a)), accel=a, bore_gap=0.100 - 0.092, a_max=(M["TW"] - 1) * G,
                        unload=W_OCTO, horse_after=HORSE_M * G)
    return M


# ================================================================== D3  behaviours of the centaur (first-pass feasibility)
def cg_torque(n_rpm):
    """CyberGear output torque available at speed n (est.): 12 N m peak, limited by the line through the two published
    points (4 N m at 240 rpm, 0 at 296 rpm)."""
    return np.clip(CG["rated"] * (CG["n0"] - n_rpm) / (CG["n0"] - CG["n_rated"]), 0.0, CG["peak"])


def cg_time(tq):
    """Seconds the CyberGear may hold a torque (maker's overload table, log-interpolated)."""
    if tq <= CG["rated"]:
        return float("inf")
    if tq > CG["peak"]:
        return 0.0
    t = np.array([a for a, _ in CG["overload"]][::-1] ); s = np.array([b for _, b in CG["overload"]][::-1])
    t = np.r_[CG["rated"], t]; s = np.r_[2 * s[0], s]
    return float(np.exp(np.interp(tq, t, np.log(s))))


def gaits():
    rows = []
    for name, duty, f in (("walk", 0.75, 1.0), ("walk", 0.75, 1.5), ("trot", 0.50, 2.0), ("trot", 0.50, 3.0)):
        v = STRIDE * f / duty
        swing = (1 - duty) / f
        rows.append(dict(gait=name, hz=f, v=v, swing=swing, froude=v ** 2 / (G * H_HIP),
                         hip_rate=np.degrees(STRIDE / H_HIP) / swing))
    r = np.hypot(L_HIP / 2, Y_FOOT)
    turn = dict(r_foot=r, deg_per_cycle=np.degrees(STRIDE / r), rate=np.degrees(STRIDE / r) * 1.25 / 0.75)
    turn["t_360"] = 360 / turn["rate"]
    return dict(rows=rows, turn=turn)


def carry(com):
    """Load held in the arms at x ahead of the front hip line: front foot load, tip angle, neck moment."""
    rows = []
    for mp in (1.0, 2.0, 4.0):
        for xp in (0.15, 0.25, 0.40):
            m = M_TOT + mp
            xc = (M_TOT * com[0] + mp * xp) / m
            zc = (M_TOT * com[2] + mp * 0.45) / m
            ff = m * G * (1 + xc / L_HIP) / 2
            rows.append(dict(payload=mp, x=xp, front_foot=ff, front_up=ff / BASE["foot_front_stand"] - 1,
                             tip_front=float(np.degrees(np.arctan(-xc / zc))), neck_moment=mp * G * (xp - X_NECK)))
    tip_mass = -M_TOT * com[0] / 0.40                            # payload at 400 mm that brings the CoM over the front feet
    return dict(rows=rows, tip_mass_at_400=tip_mass)


X_F_MAX = 0.20                  # how far ahead of its hip a hind foot can stand (est.)
FRONT4 = 4 * XL_T_MAX           # the four front props can stay level in a rear-up (their arm planes run fore-aft)


def levade(rp):
    out = {}
    px, pz = L_HIP + X_NECK + 0.10, Z_PLATE + ROOT_Z - H_HIP    # thrust acts above the front arm roots (arms hang from their props)
    for label, key in (("guard", "B guard (six held steep)"), ("counterweight", "counterweight (rear four straight back)")):
        com = rp[key]["com"]
        cx, cz = L_HIP + com[0], com[2] - H_HIP
        rows = []
        for th in (20, 30, 40, 50, 60):
            a = np.radians(th)
            xc, xt = cx * np.cos(a) - cz * np.sin(a), px * np.cos(a) - pz * np.sin(a)
            for f in (0.0, 0.25, 0.50):
                T = f * W_OCTO
                xf = (W_TOT * xc - T * xt) / (W_TOT - T)
                hip = (W_TOT * xc - T * xt) / 2
                hr = np.sqrt(max((0.90 * (L1 + L2)) ** 2 - xf ** 2, 1e-6))
                kn = knee_xz(xf, hr, -1.0)
                knee = (W_TOT - T) / 2 * abs(xf - kn[0])
                tk = cg_time(knee / (KNEE_RATIO * BELT_EFF))
                rows.append(dict(theta=th, f=f, T=T, x_foot=xf, hip=hip, hip_time=cg_time(abs(hip)), knee=knee, knee_time=tk,
                                 time=min(cg_time(abs(hip)), tk), knee_of_peak=knee / KNEE_PEAK, reach_ok=bool(-0.05 <= xf <= X_F_MAX),
                                 hold_ok=bool(abs(hip) <= CG["peak"] and knee <= KNEE_PEAK and -0.05 <= xf <= X_F_MAX)))
        # getting there: front legs push from the bow to standing height, then the body coasts up
        a0 = -np.arcsin((H_HIP - H_BOW) / L_HIP)
        zc = lambda a: cx * np.sin(a) + cz * np.cos(a)
        zt = lambda a: px * np.sin(a) + pz * np.cos(a)
        push = []
        for th in (30, 45, 60):
            for f in (0.0, 0.50):
                T = f * W_OCTO; a = np.radians(th)
                E = W_TOT * (zc(a) - zc(a0)) - T * (zt(a) - zt(a0))
                Fm = E / (H_HIP - H_BOW)
                dphi = 2 * (np.arcsin(H_HIP / (2 * L1)) - np.arcsin(H_BOW / (2 * L1)))      # knee travel, crouch to stand
                push.append(dict(theta=th, f=f, E=E, F_mean=Fm, of_static=Fm / (2 * BASE["foot_front_stand"]),
                                 work_avail=2 * KNEE_PEAK * dphi, share=E / (2 * KNEE_PEAK * dphi)))
        # props alone, from standing square: thrust that takes all load off the front feet
        T0 = W_TOT * cx / px
        out[label] = dict(cx=cx, cz=cz, rows=rows, push=push, T_props_only=T0, T_of_front4=T0 / FRONT4,
                          hold=rp[key]["worst_rated"], theta_balance_feet_under_hips=float(np.degrees(np.arctan2(cx, cz))))
    return out


def leap(f, h0=H_BOW, h1=H_HIP, dt=2e-4):
    """Vertical jump from a crouch, all four knees pushing, thrust f x octopus hover weight held throughout.
    Front legs carry 62 %, so they set the total.  Knee = CyberGear x 2:1 belt; hip actuators not counted (conservative)."""
    T = f * W_OCTO
    h, v, t = h0, 0.0, 0.0; pk_t = 0.0; pk_p = 0.0
    while h < h1:
        lev = np.sqrt(max(L1 ** 2 - (h / 2) ** 2, 1e-9))
        n = (v / lev) * KNEE_RATIO * 60 / (2 * np.pi)
        tq = cg_torque(n) * KNEE_RATIO * BELT_EFF
        F = tq / lev / (BASE["front_share"] / 2)
        a = (F + T - W_TOT) / M_TOT
        if a <= 0 and v <= 0:
            break
        pk_t = max(pk_t, tq); pk_p = max(pk_p, tq * v / lev)
        v += a * dt; h += v * dt; t += dt
        if v < 0:
            break
    g_eff = (W_TOT - T) / M_TOT
    rise = v ** 2 / (2 * g_eff) if v > 0 else 0.0
    P = 8 * XL_KP * (T / 8) ** 1.5
    air = 2 * v / g_eff if v > 0 else 0.0
    amps = P / 22.2 / 2                                         # per pack, at the nominal 22.2 V
    return dict(f=f, T=T, eff_weight=W_TOT - T, eff_share=(W_TOT - T) / W_TOT, v=v, t_push=t, rise=rise, air=air, P=P,
                amps=amps, c_rate=amps / 5.0, sag=amps * 0.019,
                E_burst=P * (t + air + 0.5) / 3600, knee_peak=pk_t, knee_power=pk_p, per_prop=T / 8,
                land_force=(W_TOT - T) + 0.5 * M_TOT * v ** 2 / (h1 - h0), length_at_trot=0.84 * air)


def steps():
    s_level = H_HIP - H_BOW
    s_pitch = s_level + L_HIP * np.sin(np.radians(10))
    stair = np.degrees(np.arctan(0.175 / 0.280))
    need = L_HIP * np.tan(np.radians(stair))
    return dict(level=s_level, pitch10=s_pitch, stair_deg=stair, stair_leg_diff=need, recommended=0.120)


def scout():
    P_hover = 8 * XL_KP * XL_T_HOVER ** 1.5
    use = USABLE * PACK_WH
    P_horse = 1.0 * HORSE_M * G * 0.5 + 20.0                    # same rule as the first pass: cost of transport 1.0 + 20 W
    sortie = 2.0
    e = P_hover * sortie / 60
    i_charge = 10.0                                             # 1C on two 5 Ah packs
    p_charge = i_charge * 22.2
    return dict(P_hover=P_hover, hover_min=use / P_hover * 60, P_horse_alone=P_horse, horse_alone_min=use / P_horse * 60,
                sortie_min=sortie, sortie_wh=e, sortie_share=e / use, recharge_min=e / p_charge * 60, i_charge=i_charge,
                horse_walk_lost_min=e / 0.9 / BASE["P_walk"] * 60, pair_walk_min=2 * use / BASE["P_walk"] * 60,
                pins_per_pole=int(np.ceil(i_charge / 9.0 * 1.5)))


def rest_power():
    out = {}
    tot = 0.0
    for name, F in (("front", BASE["foot_front_stand"]), ("rear", W_TOT * (1 - BASE["front_share"]) / 2)):
        lev = np.sqrt(L1 ** 2 - (H_HIP / 2) ** 2)
        tk = F * lev; tm = tk / (KNEE_RATIO * BELT_EFF); i = tm / CG["kt"]
        p = 3 * i ** 2 * (CG["r_line"] / 2)
        out[name] = dict(knee=tk, motor=tm, amps=i, watts=p); tot += 2 * p
    out["standing_w"] = tot
    return out


# ================================================================== D4  thrust assist
def assist(rp):
    g = rp["B guard (six held steep)"]
    w6 = wash({k: v for k, v in g["arms"].items() if k not in ("L1", "R1")}, 1.0)
    w8 = wash(rp["take-off (all eight in the tulip)"]["arms"], 1.0)
    rows = []
    for f in (0.05, 0.10, 0.25, 0.50, 0.75, 1.00):
        T = f * W_OCTO
        for n, w in ((6, w6), (8, w8)):
            per = T / n
            if per > XL_T_MAX:
                continue
            dl = w["download"] / w["n"] * 1.0                   # share of the thrust that lands on the horse
            net = T * (1 - dl)
            k = 1 - net / W_TOT
            P = n * XL_KP * per ** 1.5
            rows.append(dict(f=f, n=n, T=T, per_prop=per, net=net, leg_factor=k, P=P, minutes=USABLE * PACK_WH / P * 60,
                             knee_rms_margin=KNEE_RATED / (BASE["knee_trot_rms"] * k),
                             hip_peak_margin=CG["peak"] / (BASE["hip_trot_peak"] * k),
                             w_per_n=P / net, db_vs_hover=27.5 * np.log10(per / XL_T_HOVER)))
    need = W_TOT * (1 - CG["peak"] / BASE["hip_trot_peak"])     # net lift that brings the trot hip peak to the actuator limit
    dl6 = w6["download"] / w6["n"]
    Tn = need / (1 - dl6)
    yl = [abs(g["arms"][a]["disc"][1]) for a in ("L2", "L3", "L4")]
    roll = XL_T_MAX * sum(yl)
    return dict(rows=rows, wash6=w6, wash8=w8, hip_fix=dict(net=need, T=Tn, P=6 * XL_KP * (Tn / 6) ** 1.5),
                roll_moment=roll, tip_moment=W_TOT * Y_FOOT, roll_share=roll / (W_TOT * Y_FOOT), walk_W=BASE["P_walk"])


# ================================================================== D6  falls (sideways, about the feet of one side)
def fall_sim(zc, I, omega0, theta0=0.0, discs=None, t_on=None, strike=None, dt=5e-4, tmax=4.0):
    """Roll about the line of the left feet.  theta = roll, omega0 = roll rate at the start.
    discs = [(y, z)] of props that can push (arms keep them level); they go to full thrust after t_on with a ramp T_SPIN."""
    th, om, t = theta0, omega0, 0.0
    t_tip = None; tip = np.arctan(Y_FOOT / zc)
    while t < tmax:
        tau = -W_TOT * (Y_FOOT * np.cos(th) - zc * np.sin(th))
        if discs is not None:
            ramp = np.clip((t - t_on) / T_SPIN, 0, 1)
            for y, z in discs:
                eta = (y - Y_FOOT) * np.cos(th) + z * np.sin(th)
                if eta > 0:
                    tau -= XL_T_MAX * ramp * eta
        om += tau / I * dt; th += om * dt; t += dt
        if t_tip is None and th >= tip:
            t_tip = t
        if th <= 0 and om < 0:
            return dict(fell=False, t=t, t_tip=t_tip, th_max=None)
        if strike is not None and th >= strike:
            return dict(fell=True, t=t, t_tip=t_tip, omega=om, E=0.5 * I * om ** 2)
        if th < tip and om < 0 and t_tip is None and discs is None:
            return dict(fell=False, t=t, t_tip=None)
    return dict(fell=False, t=t, t_tip=t_tip)


def falls(rp):
    out = {}
    for label, key in (("guard", "B guard (six held steep)"), ("pack", "C pack (four folded on the horse)")):
        p = rp[key]; zc = p["com"][2]; I = p["Ixx"] + M_TOT * (Y_FOOT ** 2 + zc ** 2)
        tip = np.arctan(Y_FOOT / zc); r = np.hypot(Y_FOOT, zc)
        # first thing to touch the ground on the low side
        pts = [(PLATE_R, Z_PLATE + 0.006), (HIP_Y[1], H_HIP + HIP_R), (0.200, 0.20)]
        for a in ("L1", "L2", "L3", "L4"):
            arm = p["arms"][a]
            for Jk in arm["J"]:
                pts.append((abs(Jk[1]) + R_CHAIN, Jk[2]))
            if arm["tilt"] < 30:
                pts.append((abs(arm["disc"][1]) + arm["disc_r"], arm["disc"][2]))
        strike = min(np.pi / 2 - np.arctan2(y - Y_FOOT, z) for y, z in pts)
        discs = [(abs(p["arms"][a]["disc"][1]) * s, p["arms"][a]["disc"][2]) for a in ("L2", "L3", "L4") for s in (1, -1)
                 if p["arms"][a]["tilt"] < 30]
        rec = dict(zc=zc, I=I, tip_deg=float(np.degrees(tip)), strike_deg=float(np.degrees(strike)), n_props=len(discs),
                   omega_crit=float(np.sqrt(2 * W_TOT * (r - zc) / I)))
        rec["push_crit_Ns"] = rec["omega_crit"] * I / zc
        # time from the tip point to the first strike, without help
        rec["times"] = []
        for om in (0.1, 0.5, 1.0):
            s = fall_sim(zc, I, om, theta0=tip + 1e-4, strike=strike)
            s30 = fall_sim(zc, I, om, theta0=tip + 1e-4, strike=np.radians(30))
            s45 = fall_sim(zc, I, om, theta0=tip + 1e-4, strike=np.radians(45))
            rec["times"].append(dict(omega0=om, t_30=s30["t"], t_45=s45["t"], t_strike=s["t"], omega_strike=s["omega"], E=s["E"]))
        # push recovery from standing square: largest shove (roll rate at theta = 0) that still comes back
        def survives(om, use_props):
            s = fall_sim(zc, I, om, discs=(discs if use_props else None), t_on=T_DETECT, strike=np.radians(45))
            return not s["fell"]
        res = {}
        for use in (False, True):
            if use and not discs:
                continue
            lo, hi = 0.0, 6.0
            for _ in range(22):
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if survives(mid, use) else (lo, mid)
            res["props" if use else "legs only"] = dict(omega=lo, push_Ns=lo * I / zc, dv=lo * I / zc / M_TOT)
        rec["recovery"] = res
        if discs:
            s = fall_sim(zc, I, rec["omega_crit"] * 1.3, discs=discs, t_on=T_DETECT, strike=strike)
            s0 = fall_sim(zc, I, rec["omega_crit"] * 1.3, strike=strike)
            rec["hard_shove"] = dict(omega0=rec["omega_crit"] * 1.3, fell_with_props=s["fell"], t_no=s0["t"], E_no=s0["E"],
                                     E_props=s.get("E"), t_props=s["t"])
        out[label] = rec
    out["responses"] = dict(detect=T_DETECT, spin=T_SPIN, prop_total=T_DETECT + T_SPIN, latch=T_LATCH, bars=T_BARS,
                            fly_off=T_LATCH + T_BARS + T_SPIN,
                            arm_brace=np.radians(120) / (0.5 * SERVO_SPEED),        # swing an arm 120 deg at half the no-load speed (est.)
                            arm_tuck=np.radians(90) / (0.5 * SERVO_SPEED),
                            brace_force=STALL[1] / 0.40, brace_energy=STALL[1] / 0.40 * 0.25 * 3,
                            tie_3g=70.6, pin_margin=PIN_CAP / 70.6)
    return out


# ================================================================== D5  sensing and control: parts (specs as read 2026-11-14)
PARTS = [
    dict(part="Adafruit BNO085 9-DOF IMU breakout, product 4754", use="horse body IMU (one; a second on the collar is optional)",
         spec="I2C / SPI / UART-RVC, 25.6 x 22.7 x 4.6 mm", usd=29.50, src="adafruit.com/product/4754", ok="verified"),
    dict(part="Adafruit CAN Pal, TJA1051T/3, product 5708", use="CAN to the 12 leg actuators and across the collar",
         spec="3.3 V logic, about 1 Mbit/s", usd=3.95, src="adafruit.com/product/5708", ok="verified (out of stock on the day)"),
    dict(part="Mill-Max 08xx series spring pin (for example 0852 / 0855)", use="collar power contacts, 2 per pole + 1 spare",
         spec="9 A continuous at 10 C rise, stroke up to 2.28 mm", usd=None, src="mill-max.com/new_products/detail/79",
         ok="rating verified, exact part number and price not"),
    dict(part="Adafruit DIY magnetic connector, 8 contacts, 2.2 mm pitch, product 5469", use="collar signal contacts (CAN H/L, latch, ID)",
         spec="spring contacts with magnets; current rating not listed", usd=9.95, src="adafruit.com/product/5469",
         ok="verified, signal only"),
    dict(part="Adafruit VL53L4CD time-of-flight sensor, product 5396", use="collar bore: distance to the hub pad, last 200 mm",
         spec="about 1 to 1300 mm, I2C, 25.5 x 17.7 x 4.6 mm", usd=14.95, src="adafruit.com/product/5396", ok="verified"),
    dict(part="Hall switch US5881LUA, Adafruit product 158 (x3) + 6 x 3 magnets", use="collar seat switches, 120 deg apart",
         spec="3.5 to 24 V, unipolar, open collector", usd=2.00, src="adafruit.com/product/158", ok="verified"),
    dict(part="AprilTag 36h11, 80 mm, printed two-colour plate", use="fiducial on the horse's back for the octopus's belly and tip cameras",
         spec="80 x 80 mm tag, 100 x 100 x 2 plate", usd=1.0, src="printed", ok="est."),
    dict(part="Carbon tube dia 16 x 14, 170 long", use="breast bar (hand hold for mount and lift-off)",
         spec="stock pultruded tube", usd=6.0, src="hobby stock", ok="est. price; diameter to suit hand3"),
]
PRINT = [
    dict(part="Breast bar bracket (x2)", mat="PA-CF", box="40 x 30 x 45", orient="flat on the bolt face, bar bore vertical", note="2 x M4 into heat-set inserts in the chest"),
    dict(part="Tag plate", mat="PETG, two colours", box="100 x 100 x 2", orient="flat, tag face down on a smooth sheet", note="2 x M3"),
    dict(part="Arm cradle, back (x2)", mat="TPU 95A", box="120 x 50 x 30", orient="flat", note="arms L4/R4 lie in it; 2 x M3"),
    dict(part="Arm cradle, flank (x2)", mat="TPU 95A", box="60 x 50 x 40", orient="on its back", note="clips the hanging arm L3/R3 to the body side"),
    dict(part="Contact carrier (collar top, neck agent's part)", mat="PETG", box="60 x 24 x 14", orient="flat", note="holds 5 spring pins and the 8-way connector"),
    dict(part="Contact pad block (octopus plate underside)", mat="PETG + brass M4 pan screws as pads", box="60 x 24 x 6", orient="flat",
         note="sits at r 100 to 118 between two arm roots; the only octopus change"),
]
STATES = [
    ("APART", "no link, or radio only", "each body runs alone; horse may follow by radio"),
    ("SEATED", "seat switches closed, pins not in", "props may idle; horse holds still; no walking"),
    ("LATCHED, LINK UP", "4 pins in, CAN heartbeat", "centaur modes allowed; prop state set by arm posture"),
    ("LATCHED, NO LINK", "4 pins in, no heartbeat for 0.2 s", "horse: stop, square stance, lie down after 5 s; octopus: arms freeze, props off"),
    ("LATCHED, UNPOWERED", "either pack off", "lock holds with no power; horse actuators limp, so only when lying"),
    ("GUARD", "six arms in the guard set, link up", "props armed for assist and catch"),
    ("PACK", "any prop disc within 150 mm of the horse or below the plate", "props locked out in the ESC arming logic"),
]


def clean(o):
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in o.items() if k not in ("J", "full", "grid", "x", "tau")}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer, np.bool_)):
        return o.item()
    return o


# ================================================================== sketches
def dome_R(sc, c, a, cz, R, fc, alpha=1.0, n=7):
    ph = np.linspace(0, np.pi / 2, n + 1); th = np.linspace(0, 2 * np.pi, 2 * n + 1)
    Pt = lambda p, t: c + R @ np.array([a * np.cos(p) * np.cos(t), a * np.cos(p) * np.sin(t), cz * np.sin(p)])
    for i in range(n):
        for j in range(2 * n):
            sc.poly([Pt(ph[i], th[j]), Pt(ph[i], th[j + 1]), Pt(ph[i + 1], th[j + 1]), Pt(ph[i + 1], th[j])], fc, alpha, ec=(0, 0, 0, 0))


def draw_octo(sc, R, t, full, bars=False, alpha=1.0, spin=()):
    """Cephalo-1 stand-in with the arm shapes of this study.  R, t = octopus -> world."""
    t = np.asarray(t, float)
    Wd = lambda p: t + R @ np.asarray(p, float)
    sc.cyl(Wd((0, 0, 0)), Wd((0, 0, 0.006)), PLATE_R, C["frame"], alpha, n=24)
    dome_R(sc, Wd((0, 0, 0.006)), 0.125, 0.150, R, C["octo"], alpha)
    sc.cyl(Wd((0, 0, 0.10)), Wd((0, 0, TURRET_TOP)), 0.090, C["octo2"], alpha, n=20)
    sc.cyl(Wd((0, 0, -HUB_FACE)), Wd((0, 0, -0.003)), 0.057, C["act"], alpha, n=16)
    if bars:
        for k in range(4):
            a = np.radians(45 + 90 * k)
            p = np.array([0.07 * np.cos(a), 0.07 * np.sin(a), 0])
            sc.cyl(Wd(p + [0, 0, -0.030 - STROKE_FULL]), Wd(p), 0.010, C["bar"], 1.0, n=8)
    for i in range(8):
        psi, q = full[i]
        J, d, u = chain(np.array([psi]), q[None], ARM_AZ[i])
        Jw = J[0] @ R.T + t; uw = R @ u[0]
        end = Jw[7] + PITCH * uw
        sc.tube(list(Jw[:8]) + [end], 0.016, C["octo2"], alpha, n=8)
        hand = ARM_NAME[i] in ("L1", "R1")
        sc.cyl(end, Jw[8], 0.024 if hand else 0.018, C["tpu"] if hand else C["act"], alpha, n=10)
        if ARM_NAME[i] in spin:
            sc.disc(Jw[8], HAND_R if hand else XL_R, uw, C["octo"], 0.32)


def draw_horse(sc, Rb=np.eye(3), base=(0, 0, H_HIP), ref=(0, 0, H_HIP), feet=None, alpha=1.0, collar_alpha=1.0):
    base, ref = np.asarray(base, float), np.asarray(ref, float)
    W = lambda p: base + Rb @ (np.asarray(p, float) - ref)
    sc.box(W((0.5 * (BODY_X[0] + BODY_X[1]), 0, 0.5 * (BODY_Z[0] + BODY_Z[1]))), (BODY_X[1] - BODY_X[0], BODY_W, BODY_Z[1] - BODY_Z[0]),
           R=Rb, fc=C["horse"], alpha=alpha)
    sc.cyl(W((X_NECK, 0, BODY_Z[1])), W((X_NECK, 0, Z_PLATE)), COLLAR_R, C["frame"], collar_alpha, n=24)
    sc.cyl(W((GRIP[0], -0.085, GRIP[2])), W((GRIP[0], 0.085, GRIP[2])), 0.009, C["hi"], alpha, n=8)
    for x, front in ((0.0, True), (-L_HIP, False)):
        for s in (1, -1):
            hip = W((x, s * Y_HIP, H_HIP))
            foot = np.array([hip[0], s * Y_FOOT, 0.022]) if feet is None or (front, s) not in feet else np.asarray(feet[(front, s)], float)
            dv = foot - hip; dist = min(np.linalg.norm(dv), L1 + L2 - 1e-3); uh = dv / np.linalg.norm(dv)
            n = np.cross(uh, [0, 1.0, 0]); n = n / np.linalg.norm(n) * (1 if front else -1)
            knee = hip + 0.5 * dist * uh + np.sqrt(L1 ** 2 - (0.5 * dist) ** 2) * n
            sc.cyl(hip - np.array([0, 0.02 * s, 0]), hip + np.array([0, 0.02 * s, 0]), HIP_R, C["act"], alpha, n=14)
            sc.tube([hip, knee, foot], 0.014, C["horse"], alpha, n=8); sc.sphere(foot, 0.022, C["tpu"], alpha, n=5)


def xpose(d, names=("L1", "L2", "L3", "L4")):
    """{arm: solved x} -> full eight-arm pose."""
    return set_pose({a: (d[a][0], d[a][1:]) for a in names})


def sheet_mount(S):
    M = S["mount"]
    fig = sk3d.figure(16, 9, "Mount (recommended): the horse lies down, the octopus walks on, hands first")
    z_hi, z_lo = COLLAR_TOP["lie (belly down)"] + CLEAR_HUB, COLLAR_TOP["lie (belly down)"]
    over = xpose({a: M["a_over"][a]["x"] for a in FEET})
    grip = xpose(dict(L1=M["a_grip"]["x"], **{a: M["a_six"][a]["x"] for a in ("L2", "L3", "L4")}))
    seat = xpose(dict(L1=M["a_grip_seated"]["x"], **{a: M["a_seated"][a]["x"] for a in ("L2", "L3", "L4")}))
    pack = S["rest"]["C pack (four folded on the horse)"]["full"]
    lie = dict(base=(0, 0, H_LIE))
    I3 = np.eye(3)
    frames = [
        (lie, (I3, (X_NECK + 0.45, 0, z_hi)), over, {}, 0),
        (lie, (I3, (X_NECK, 0, z_hi)), over, {}, 1),
        (lie, (I3, (X_NECK, 0, z_hi)), grip, {}, 2),
        (lie, (I3, (X_NECK, 0, z_lo)), seat, {}, 3),
        (lie, (I3, (X_NECK, 0, z_lo)), seat, dict(bars=True, collar_alpha=0.30), 4),
        ({}, (SCEN["stand"]["Rw"], SCEN["stand"]["tw"]), pack, {}, 6),
    ]
    seq = M["seq_ground"]
    cap = ["Horse lies down", "Octopus walks astride", "Two hands take the breast bar", "Body lowers onto the collar",
           "Bars slide down and latch", "Arms fold, horse stands up"]
    end = ["until: belly switches closed", "until: back tag centred within 10 mm", "until: both hands closed",
           "until: three seat switches closed", "until: four latch pins clicked", "until: hip height 361, level"]
    dur = [seq[0][1], seq[1][1], seq[2][1], seq[3][1], seq[4][1], seq[5][1] + seq[6][1]]
    for k, (hk, (R, t), full, opt, _) in enumerate(frames):
        col, row = k % 3, k // 3
        x0, y0 = 0.005 + 0.33 * col, 0.625 - 0.375 * row
        sc = sk3d.Scene(fig, [x0, y0, 0.33, 0.315], elev=13, azim=-64)
        sc.ground((-0.70, -0.52), (0.72, 0.52))
        draw_horse(sc, base=hk.get("base", (0, 0, H_HIP)), collar_alpha=opt.get("collar_alpha", 1.0))
        draw_octo(sc, R, t, full, bars=opt.get("bars", False))
        if k == 0:
            sc.arrow((X_NECK + 0.42, -0.50, 0.0), (-0.30, 0, 0)); sc.text((X_NECK + 0.50, -0.50, 0.0), "450 mm", color=C["hi"], fs=11, weight="bold")
        if k == 3:
            sc.arrow((X_NECK, 0, z_hi + 0.42), (0, 0, -0.14)); sc.text((X_NECK + 0.10, 0, z_hi + 0.36), "54 mm", color=C["hi"], fs=11, weight="bold")
        if k == 4:
            sc.arrow((X_NECK, 0, z_lo + 0.44), (0, 0, -0.16)); sc.text((X_NECK + 0.10, 0, z_lo + 0.38), "194 mm", color=C["hi"], fs=11, weight="bold")
        sc.done((-0.70, -0.52, 0.0), (0.72, 0.52, 0.86))
        fig.text(x0 + 0.02, y0 - 0.002, f"{k + 1}  {cap[k]}", fontsize=14, fontweight="bold", va="top")
        fig.text(x0 + 0.02, y0 - 0.034, f"{dur[k]:.0f} s    {end[k]}", fontsize=12, va="top")
    a8 = max(v["rated"] for v in M["a_over"].values()); a6 = max(v["rated"] for v in M["a_six"].values())
    s6 = max(v["stall"] for v in M["a_six"].values())
    sk3d.panel_text(fig, 0.02, 0.150, [
        f"Total {M['seq_ground_total']:.0f} s.  No prop turns near the horse.  Every step can stop and hold.",
        f"Plate must pass {mm(M['plate_needed']['lie (belly down)'])} mm; the octopus can stand to {mm(M['stand_height']['h_geo'])} mm.",
        f"Arm servos (stand-in model): {a8*100:.0f} % of rated on eight feet,",
        f"{a6*100:.0f} % of rated ({s6*100:.0f} % of stall) on six while the hands hold the bar.",
    ], fs=12.5)
    sk3d.panel_text(fig, 0.52, 0.150, [
        f"Horse gets up with the octopus on: front knee {M['getup']['knee']:.1f} N m, {M['getup']['of_peak']*100:.0f} % of peak.",
        f"Fast option: fly in, two hands on the bar, pull down ({M['seq_air_total']:.0f} s, {M['air_energy_wh']:.0f} Wh).",
        f"Leaving: take off from the horse's back in {M['seq_takeoff_total']:.0f} s,",
        "or run this sheet backwards.",
    ], fs=12.5)
    sk3d.save(fig, OUT + "integ_1_mount.png")


def side2d(ax, sc_, legs=True):
    """Side view (x, z) of the horse and the docked octopus in one scenario."""
    Ws = lambda P: (np.asarray(P, float) - sc_["ref"]) @ sc_["Rb"].T + sc_["base"]
    Wo = lambda P: np.asarray(P, float) @ sc_["Rw"].T + sc_["tw"]
    b = Ws([[BODY_X[0], 0, BODY_Z[0]], [BODY_X[1], 0, BODY_Z[0]], [BODY_X[1], 0, BODY_Z[1]], [BODY_X[0], 0, BODY_Z[1]]])
    ax.fill(b[:, 0], b[:, 2], color=C["horse"], lw=0, zorder=3)
    c = Wo([[-COLLAR_R, 0, BODY_Z[1] - Z_PLATE], [COLLAR_R, 0, BODY_Z[1] - Z_PLATE], [COLLAR_R, 0, 0], [-COLLAR_R, 0, 0]])
    ax.fill(c[:, 0], c[:, 2], color=C["frame"], lw=0, zorder=3)
    tt = np.linspace(0, np.pi, 30)
    d = Wo(np.column_stack([0.125 * np.cos(tt), 0 * tt, 0.006 + 0.150 * np.sin(tt)]))
    ax.fill(d[:, 0], d[:, 2], color=C["octo"], lw=0, zorder=3)
    tr = Wo([[-0.09, 0, 0.10], [0.09, 0, 0.10], [0.09, 0, TURRET_TOP], [-0.09, 0, TURRET_TOP]])
    ax.fill(tr[:, 0], tr[:, 2], color=C["octo2"], lw=0, zorder=3)
    if legs:
        for x, front in ((0.0, True), (-L_HIP, False)):
            hip = Ws([x, 0, H_HIP]); k = knee_xz(0.0, hip[2] - 0.022, 1.0 if front else -1.0)
            ax.plot([hip[0], hip[0] + k[0], hip[0]], [hip[2], hip[2] + k[1], 0.022], color=C["horse"], lw=5, solid_capstyle="round", zorder=2)
            ax.add_patch(plt.Circle((hip[0], hip[2]), HIP_R, color=C["act"], zorder=4))
    ax.axhline(0, color="#555555", lw=1.2)


def envelope(ax, key, fc, lw=1.6, fill=True, ls="-"):
    p = PRE[key]; tw = p["tw"][p["ok"]]
    H, xe, ze = np.histogram2d(tw[:, 0], tw[:, 2], bins=[np.arange(-0.72, 0.84, 0.02), np.arange(0.0, 1.18, 0.02)])
    X, Z = np.meshgrid(0.5 * (xe[:-1] + xe[1:]), 0.5 * (ze[:-1] + ze[1:]))
    V = (H.T > 0).astype(float)
    if fill:
        ax.contourf(X, Z, V, levels=[0.5, 2], colors=[fc], alpha=0.28, zorder=1)
    ax.contour(X, Z, V, levels=[0.5], colors=[fc], linewidths=lw, linestyles=ls, zorder=5)
    if p["pick"].any():
        px = p["tw"][p["pick"]][:, 0]
        ax.plot([px.min(), px.max()], [0.006, 0.006], color=fc, lw=6, solid_capstyle="butt", zorder=6)


def sheet_arms(S):
    fig = sk3d.figure(16, 9, "Arm roles: two working hands, six arms with two postures; where the hands reach")
    R = S["rest"]; rc = R["C pack (four folded on the horse)"]; rb = R["B guard (six held steep)"]
    for k, (r, title, spin, sub) in enumerate((
            (rc, "Travel posture: pack", (), "props locked out"),
            (rb, "Guard posture: six props ready", ("L2", "L3", "L4", "R2", "R3", "R4"), "for assist, leap, catch"))):
        y0 = 0.535 - 0.455 * k
        sc = sk3d.Scene(fig, [-0.01, y0, 0.33, 0.37], elev=12, azim=-58)
        sc.ground((-0.72, -0.50), (0.60, 0.50))
        draw_horse(sc)
        draw_octo(sc, SCEN["stand"]["Rw"], SCEN["stand"]["tw"], r["full"], spin=spin)
        sc.done((-0.72, -0.50, 0.0), (0.60, 0.50, 1.10))
        fig.text(0.015, y0 + 0.385, title, fontsize=14, fontweight="bold", va="top")
        fig.text(0.015, y0 + 0.002, f"CoM {mm(r['com'][2])} mm, tips sideways at {r['tip_side']:.1f} deg", fontsize=12, va="top")
        fig.text(0.015, y0 - 0.028, f"worst arm hold {r['worst_rated']*100:.0f} % of rated; {sub}", fontsize=12, va="top")
    # reach envelopes, side view
    for k, (sn, title) in enumerate((("stand", "Standing"), ("bow", "Bow"))):
        yb = 0.515 - 0.455 * k
        ax = fig.add_axes([0.335, yb, 0.335, 0.325])
        side2d(ax, SCEN[sn])
        envelope(ax, ("L1", sn), "#2e9e5b")
        rr = S["reach"][f"L1 {sn}"]
        if sn == "stand":
            envelope(ax, ("L1", "stand, neck 30"), "#7a5cc0", fill=False, ls="--")
            r30 = S["reach"]["L1 stand, neck 30"]; r15 = S["reach"]["L1 stand, neck 15"]
            fig.text(0.345, yb + 0.378, f"green: neck upright; hands stop {mm(rr['z_min'])} mm above the ground", fontsize=11.5, va="top", color="#1f7a43")
            fig.text(0.345, yb + 0.352, f"dashed: neck 30 deg; picks {r30['pick_area_cm2']:.0f} cm2 of ground (15 deg: {r15['pick_area_cm2']:.0f} cm2)", fontsize=11.5, va="top", color="#5b3fa0")
        else:
            fig.text(0.345, yb + 0.378, f"hands pick {rr['pick_area_cm2']:.0f} cm2 of ground, x {mm(rr['pick_x'][0])} to {mm(rr['pick_x'][1])} mm (thick line)", fontsize=11.5, va="top", color="#1f7a43")
        ax.set_xlim(-0.72, 0.82); ax.set_ylim(-0.03, 1.13); ax.set_aspect("equal"); ax.axis("off")
        fig.text(0.345, yb + 0.410, f"{title}: reach of a front hand, side view", fontsize=14, fontweight="bold", va="top")
    # payload map
    pm = S["pmap"]
    ax = fig.add_axes([0.705, 0.50, 0.27, 0.36])
    g = np.clip(pm["grid"]["rated"], 0, 2.0)
    im = ax.imshow(g, origin="lower", extent=pm["extent"], cmap="YlGnBu", vmin=0, vmax=2.0, zorder=1)
    X = np.linspace(pm["extent"][0], pm["extent"][1], g.shape[1] + 1)[:-1] + pm["cell"] / 2
    Z = np.linspace(pm["extent"][2], pm["extent"][3], g.shape[0] + 1)[:-1] + pm["cell"] / 2
    cs = ax.contour(X, Z, np.nan_to_num(pm["grid"]["rated"], nan=-1), levels=[0.5, 1.0], colors=["#d43a2f", "#15171a"], linewidths=1.6, zorder=5)
    ax.clabel(cs, fmt={0.5: "0.5 kg", 1.0: "1 kg"}, fontsize=11)
    side2d(ax, SCEN["stand"])
    ax.set_xlim(-0.30, 0.66); ax.set_ylim(-0.03, 1.10); ax.set_aspect("equal"); ax.axis("off")
    cb = fig.colorbar(im, ax=ax, fraction=0.045, pad=0.02); cb.set_label("kg per hand, all joints inside rated", fontsize=11); cb.ax.tick_params(labelsize=11)
    fig.text(0.705, 0.92, "Payload one hand can hold", fontsize=14, fontweight="bold", va="top")
    st = S["straight"][0]; rows = {r["payload"]: r for r in pm["rows"]}
    hold = S["tasks"][0]["rows"]; bow = [t for t in S["tasks"] if t["task"] == "ground pick" and t["scen"] == "bow"][0]["rows"]
    o = R["_outrigger"]; gt = R["_girth_tie"]
    sk3d.panel_text(fig, 0.705, 0.455, [
        f"Arm straight out, empty: joint 2 at {st['j2_rated']*100:.0f} % of rated.",
        "So the hands work close in and low:",
        f"  1 kg inside rated to x {mm(rows[1.0]['x_rated'])} mm, stall {mm(rows[1.0]['x_stall'])}",
        f"  0.5 kg inside rated to x {mm(rows[0.5]['x_rated'])} mm, stall {mm(rows[0.5]['x_stall'])}",
        f"  (chest front is at x {mm(BODY_X[1])} mm)",
        f"Ground pick in the bow, 1 kg: {bow[2]['rated']*100:.0f} % of rated.",
        f"Wrap as a tie: {gt['rated']:.0f} N per arm; a latch pin: {gt['pin_cap']:.0f} N.",
        f"Outrigger: {o['force_rated']:.0f} N per arm, and only if crouched.",
        "All arm numbers: stand-in arm model.",
    ], fs=12)
    sk3d.save(fig, OUT + "integ_2_arms.png")


def sheet_behave(S):
    fig = sk3d.figure(16, 9, "Behaviours: rear-up, prop-assisted leap, thrust assist, falls")
    # ---- rear-up sketch
    L = S["levade"]["counterweight"]
    row = [r for r in L["rows"] if r["theta"] == 40 and r["f"] == 0.25][0]
    row0 = [r for r in L["rows"] if r["theta"] == 40 and r["f"] == 0.0][0]
    th = np.radians(40); xf = row["x_foot"]; hr = np.sqrt((0.90 * (L1 + L2)) ** 2 - xf ** 2)
    Rb = roty(-th); ref = np.array([-L_HIP, 0, H_HIP]); base = np.array([-L_HIP, 0, hr])
    W = lambda p: base + Rb @ (np.asarray(p, float) - ref)
    feet = {(False, 1): (-L_HIP + xf, Y_FOOT, 0.022), (False, -1): (-L_HIP + xf, -Y_FOOT, 0.022)}
    for s in (1, -1):
        feet[(True, s)] = W((0, s * Y_HIP, H_HIP)) + np.array([0.13, 0.03 * s, -0.17])
    full = set_pose({"L1": (np.radians(-22.5), deg(-50, 0, 0, 0, 0, 0, 0)), "L2": (np.radians(-60.0), deg(-20, -30, 0, 0, 0, 0, 0)),
                     "L3": BACK3, "L4": BACK})
    sc = sk3d.Scene(fig, [-0.02, 0.27, 0.40, 0.63], elev=8, azim=-78)
    sc.ground((-0.95, -0.45), (0.45, 0.45))
    draw_horse(sc, Rb=Rb, base=base, ref=ref, feet=feet)
    t = W((X_NECK, 0, Z_PLATE))
    draw_octo(sc, Rb, t, full, bars=False, spin=("L1", "R1", "L2", "R2"))
    for i in (0, 7, 1, 6):
        psi, q = full[i]; J, d, u = chain(np.array([psi]), q[None], ARM_AZ[i])
        tip = J[0, 8] @ Rb.T + t
        sc.arrow(tip, (0, 0, 0.13), color=C["hi"])
    sc.done((-0.95, -0.45, 0.0), (0.45, 0.45, 1.30))
    p45 = [r for r in L["push"] if r["theta"] == 45]
    sk3d.panel_text(fig, 0.02, 0.275, [
        f"Hind feet {mm(row['x_foot'])} mm ahead of the hips; rear arms swing back.",
        f"{row['f']*100:.0f} % thrust ({row['T']:.0f} N): hind hip {row['hip']:.1f} N m per leg, hold {row['time']:.0f} s.",
        f"No thrust: hind hip {row0['hip']:.1f} N m (limit {CG['peak']:.0f}), hold {row0['time']:.0f} s.",
        f"Getting up to 45 deg: {p45[0]['E']:.0f} J = {p45[0]['share']*100:.0f} % of the front knees'",
        f"peak work; with 50 % thrust {p45[1]['E']:.0f} J = {p45[1]['share']*100:.0f} %.",
        "Props alone cannot lift the forehand.",
    ], fs=12, title="Rear-up at 40 deg")
    # ---- leap
    ax = fig.add_axes([0.435, 0.555, 0.235, 0.30])
    lp = S["leap"]
    xs = np.arange(len(lp))
    ax.bar(xs, [r["rise"] * 1e3 for r in lp], color=[C["horse"]] + [C["octo"]] * (len(lp) - 1), width=0.62)
    for i, r in enumerate(lp):
        ax.text(i, r["rise"] * 1e3 + 8, f"{mm(r['rise'])} mm\n{r['P']/1e3:.1f} kW" if r["P"] > 0 else f"{mm(r['rise'])} mm\nlegs only", ha="center", fontsize=11)
    ax.set_xticks(xs); ax.set_xticklabels([f"{r['f']*100:.0f} %" for r in lp], fontsize=11.5)
    ax.set_ylim(0, max(r["rise"] for r in lp) * 1e3 * 1.32); ax.set_ylabel("rise of the body, mm", fontsize=11.5)
    ax.set_xlabel("prop thrust, % of the octopus's hover thrust", fontsize=11.5); ax.tick_params(labelsize=11)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    ll = S["leap_limit"]
    fig.text(0.40, 0.92, "Pegasus leap", fontsize=14, fontweight="bold", va="top")
    fig.text(0.40, 0.892, f"props flat out: {ll['T_max']:.0f} N = {ll['tw']:.2f} x its weight. It cannot fly.", fontsize=11.5, va="top")
    # ---- thrust assist
    ax = fig.add_axes([0.435, 0.115, 0.235, 0.28])
    A = S["assist"]; rows = [r for r in A["rows"] if r["n"] == 6]
    ax.plot([r["net"] for r in rows], [r["P"] for r in rows], "-o", color=C["octo2"], lw=2.2, ms=6)
    ax.text(0.04, 0.97, "\n".join(f"{r['net']:.0f} N lift: {r['P']:.0f} W, legs carry {r['leg_factor']*100:.0f} %" for r in rows if r["f"] in (0.25, 0.50, 0.75)),
            transform=ax.transAxes, va="top", fontsize=11, linespacing=1.4)
    ax.axhline(A["walk_W"], color=C["horse"], lw=1.6, ls="--")
    ax.text(rows[-1]["net"], A["walk_W"] + 40, f"walking: {A['walk_W']:.0f} W", fontsize=11, color=C["horse"], ha="right")
    ax.set_xlabel("net lift on the centaur, N (six props)", fontsize=11.5); ax.set_ylabel("prop power, W", fontsize=11.5); ax.tick_params(labelsize=11)
    ax.set_xlim(0, rows[-1]["net"] * 1.08); ax.set_ylim(0, rows[-1]["P"] * 1.08)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    h = A["hip_fix"]
    fig.text(0.40, 0.475, "Thrust assist", fontsize=14, fontweight="bold", va="top")
    fig.text(0.40, 0.447, f"{h['net']:.0f} N of lift ({h['P']:.0f} W) puts the trot hip peak inside its limit;", fontsize=11.5, va="top")
    fig.text(0.40, 0.421, "more than that is for bursts, not for cruising", fontsize=11.5, va="top")
    # ---- falls
    F = S["falls"]; g = F["guard"]; r = F["responses"]; tm = g["times"][1]
    ax = fig.add_axes([0.80, 0.585, 0.185, 0.27])
    items = [("unlatch + bars out", r["latch"] + r["bars"], C["hi"]), ("arms tucked", r["arm_tuck"], C["tpu"]), ("props at full thrust", r["prop_total"], C["spring"]),
             ("ground strike", tm["t_strike"], "#555555"), ("past 45 deg", tm["t_45"], "#888888"), ("past 30 deg", tm["t_30"], "#aaaaaa")]
    lim = 1.25
    for i, (n, v, c) in enumerate(items):
        ax.barh(i, min(v, lim), color=c, height=0.62)
        ax.text(min(v, lim) + 0.03, i, f"{v:.2f} s" if v < lim else f"{v:.1f} s  >>", va="center", fontsize=11, ha="left" if v < lim else "right",
                color="k" if v < lim else "white", fontweight="normal" if v < lim else "bold") if v < lim else ax.text(lim - 0.03, i, f"{v:.1f} s  >>", va="center", ha="right", fontsize=11, color="white", fontweight="bold")
    ax.set_yticks(range(len(items))); ax.set_yticklabels([n for n, _, _ in items], fontsize=11.5)
    ax.set_xlim(0, lim + 0.22); ax.set_xlabel("seconds after it passes the tip angle", fontsize=11.5); ax.tick_params(labelsize=11)
    for sp in ("top", "right"):
        ax.spines[sp].set_visible(False)
    fig.text(0.70, 0.92, f"Sideways fall (tips at {g['tip_deg']:.0f} deg)", fontsize=14, fontweight="bold", va="top")
    fig.text(0.70, 0.892, "stay latched; props catch; no time to fly off", fontsize=11.5, va="top")
    rec = g["recovery"]
    sk3d.panel_text(fig, 0.70, 0.475, [
        f"Shove it survives: {rec['legs only']['push_Ns']:.0f} N s standing still,",
        f"  {rec['props']['push_Ns']:.0f} N s with six props armed (guard).",
        f"Latch pins in a 3 g hit: margin x{r['pin_margin']:.0f}.",
    ], fs=12)
    gt = S["gaits"]; c = S["scout"]; st = S["steps"]; rw = S["rest_power"]
    lv = [x for x in S["leap"] if x["f"] == 0.5][0]
    sk3d.panel_text(fig, 0.70, 0.355, [
        f"Walk {gt['rows'][0]['v']:.2f} to {gt['rows'][1]['v']:.2f} m/s, trot {gt['rows'][2]['v']:.2f} to {gt['rows'][3]['v']:.2f} m/s",
        f"Turn in place {gt['turn']['rate']:.0f} deg/s",
        f"Bow and pick: {S['reach']['L1 bow']['pick_area_cm2']:.0f} cm2 of ground in reach",
        f"Rear-up 40 deg, {row['time']:.0f} s with {row['f']*100:.0f} % thrust",
        f"Leap {mm(S['leap'][0]['rise'])} mm alone, {mm(lv['rise'])} mm at 50 % thrust",
        f"Step up {mm(st['recommended'])} mm; house stairs: no",
        f"Scout: {c['sortie_min']:.0f} min flight = {c['sortie_share']*100:.0f} % of the octopus packs",
        f"Lie down to rest: 0 W (standing {rw['standing_w']:.0f} W in the knees)",
    ], fs=12, title="Modes to build and film")
    sk3d.save(fig, OUT + "integ_3_behaviours.png")


def sheets(S):
    sheet_mount(S)
    sheet_arms(S)
    sheet_behave(S)


# ================================================================== run
def mm(v):
    return f"{v * 1e3:.0f}"


def compute():
    global CLOUD, WORK
    S = {}
    CLOUD = sample(150000)
    S["reach"] = reach_study()
    w = solve("L1", SCEN["stand"], (0.20, 0.10, 0.32), pre=PRE[("L1", "stand")])
    WORK = (w["x"][0], w["x"][1:])
    S["work_pose"] = dict(rated=w["rated"], gap=w["gap"], tcp=w["tcp"])
    S["straight"] = straight_arm()
    S["pmap"] = payload_map()
    S["tasks"] = task_poses(S["reach"]["L1 stand"]["z_min"])
    S["rest"] = rest_poses()
    S["mount"] = mount_study(S["rest"])
    com_pack = S["rest"]["C pack (four folded on the horse)"]["com"]
    S["gaits"] = gaits()
    S["carry"] = carry(com_pack)
    S["levade"] = levade(S["rest"])
    S["leap"] = [leap(f) for f in (0.0, 0.25, 0.50, 0.75, 1.00)]
    S["leap_limit"] = dict(T_max=8 * XL_T_MAX, tw=8 * XL_T_MAX / W_TOT, P_max=8 * XL_KP * XL_T_MAX ** 1.5,
                           amps=8 * XL_KP * XL_T_MAX ** 1.5 / 22.2 / 2)
    S["steps"] = steps()
    S["scout"] = scout()
    S["rest_power"] = rest_power()
    S["assist"] = assist(S["rest"])
    S["falls"] = falls(S["rest"])
    S["parts"] = PARTS; S["print"] = PRINT; S["states"] = STATES
    S["model"] = dict(arm="STAND-IN (task prompt), cephalo4.py not readable from this session", tcp_len=TCP_LEN, arm_len=ARM_LEN,
                      arm_mass=ARM_M + POD_M, core_mass=CORE_M, core_z=CORE_Z, rated=RATED.tolist(), stall=STALL.tolist(),
                      m_tot=M_TOT, w_tot=W_TOT, w_octo=W_OCTO, knee_rated=KNEE_RATED, knee_peak=KNEE_PEAK)
    return S


def report(S):
    P = print
    P("\n== MODEL =="); m = S["model"]
    P(f"arm {mm(m['arm_len'])} mm + hand {mm(HAND)} = {mm(m['tcp_len'])} mm to the fingertip; arm + pod {m['arm_mass']:.2f} kg; octopus core {m['core_mass']:.2f} kg"
      f" at {mm(m['core_z'])} mm (est.); centaur {m['m_tot']:.2f} kg, {m['w_tot']:.0f} N; octopus {m['w_octo']:.0f} N")
    P("\n== D1 REACH (S) ==")
    P("| arm, horse pose | lowest hand point above ground mm | reaches the ground | pick area, both hands cm2 | pick zone x mm | reach at chest height x mm |")
    for k, v in S["reach"].items():
        px = f"{mm(v['pick_x'][0])} to {mm(v['pick_x'][1])}" if v["pick_x"] else "-"
        P(f"| {k} | {mm(v['z_min'])} | {'yes' if v['ground'] else 'no'} | {v['pick_area_cm2']:.0f} | {px} | {mm(v['x_max_chest'])} |")
    P(f"   side reach y_max L1 {mm(S['reach']['L1 stand']['y_max'])}, L2 {mm(S['reach']['L2 stand']['y_max'])}; top z L1 {mm(S['reach']['L1 stand']['z_max'])}")
    P("\nstraight arm, level:")
    for r in S["straight"]:
        P(f"   payload {r['payload']:.1f} kg: joint 1 {r['j1']:.2f} N m = {r['j1_rated']*100:.0f} % rated, {r['j1_stall']*100:.0f} % stall;"
          f" joint 2 {r['j2']:.2f} N m = {r['j2_rated']*100:.0f} % rated, {r['j2_stall']*100:.0f} % stall")
    P("\npayload against reach at chest height (z 450), one arm straight ahead:")
    P("| payload kg | reach inside rated, x mm | reach inside stall, x mm |")
    for r in S["pmap"]["rows"]:
        P(f"| {r['payload']:.2f} | {mm(r['x_rated']) if r['x_rated'] is not None else 'none'} | {mm(r['x_stall']) if r['x_stall'] is not None else 'none'} |")
    P(f"\nfront arm at rest (hand low in front of the chest): x{S['work_pose']['rated']:.2f} rated, gap {mm(S['work_pose']['gap'])} mm, hand at {np.round(np.array(S['work_pose']['tcp'])*1e3)}")
    P("\ntask poses:")
    P("| task | horse pose | arm | payload kg | worst joint | % of rated | % of stall | gap to horse mm | hand x mm |")
    for t in S["tasks"]:
        for r in t["rows"]:
            P(f"| {t['task']} | {t['scen']} | {t['arm']} | {r['payload']:.1f} | j{r['worst']} | {r['rated']*100:.0f} | {r['stall']*100:.0f} | {mm(r['gap'])} | {mm(r['tcp'][0])} |"
              + ("" if r["ok"] else "  NOT REACHED"))
    P("\nrest poses of the six other arms:")
    P("| pose | CoM height mm | tip sideways deg | tip forward deg | front share % | worst hold % rated | props ready | lowest disc gap to horse mm | arm gap mm | top mm |")
    for k, v in S["rest"].items():
        if k.startswith("_"):
            continue
        dg = mm(v["min_disc_gap"]) if v["min_disc_gap"] is not None else "-"
        P(f"| {k} | {mm(v['com'][2])} | {v['tip_side']:.1f} | {v['tip_front']:.1f} | {v['front_share']*100:.0f} | {v['worst_rated']*100:.0f} | {v['props_up']} | {dg} | {mm(v['worst_gap'])} | {mm(v['top'])} |")
        P("     " + "; ".join(f"{a} hold {v['arms'][a]['rated']*100:.0f} %, gap {mm(v['arms'][a]['gap'])}, tip at ({mm(v['arms'][a]['disc'][0])}, {mm(v['arms'][a]['disc'][1])}, {mm(v['arms'][a]['disc'][2])})"
                              for a in ("L1", "L2", "L3", "L4")) + f"; octopus CoM {mm(v['octo_com_above_plate'])} above the plate; Ixx {v['Ixx']:.2f}")
    g = S["rest"]["_girth_tie"]
    P(f"girth hug as a tie: {g['rated']:.1f} N rated, {g['stall']:.1f} N stall per arm, against {g['pin_cap']:.0f} N per latch pin; tucked tips {mm(S['rest']['_girth_tips_apart'])} mm apart")
    o = S["rest"]["_outrigger"]
    P(f"outrigger (horse crouched, side arm on the ground at y {mm(o['y'])}): reached {o['ok']}, {o['force_rated']:.1f} N per arm at rated,"
      f" two arms give {o['moment_rated']:.1f} N m against {o['tip_moment']:.1f} N m to tip")
    M = S["mount"]
    P("\n== D2 MOUNT ==")
    for k in M["collar_top"]:
        P(f"   horse {k}: collar top {mm(M['collar_top'][k])}, plate must first reach {mm(M['plate_needed'][k])}")
    sh = M["stand_height"]
    P(f"octopus alone on {sh['n']} arms, feet on r {mm(sh['rho'])}: geometric limit plate {mm(sh['h_geo'])};  " +
      "; ".join(f"h {mm(r['h'])}: {r['rated']*100:.0f} % rated" + ("" if r["ok"] else " (not reached)") for r in sh["rows"]))
    for key, lab in (("a_over", "(a) over the lying horse, plate " + mm(M["plate_needed"]["lie (belly down)"]) + ", eight feet"),
                     ("a_six", "(a) same, hands on the bar, six feet"), ("a_seated", "(a) seated on the collar, six feet (before the weight comes off)"),
                     ("d_over", "(d) over the crouched horse, plate " + mm(M["plate_needed"]["crouch (hips 200)"]) + ", eight feet")):
        P(f"{lab}: " + "; ".join(f"{a} {v['rated']*100:.0f} % rated / {v['stall']*100:.0f} % stall, gap {mm(v['gap'])}" + ("" if v["ok"] else " NOT OK")
                                 for a, v in M[key].items()))
    P(f"(a) hand on the breast bar, horse lying: {M['a_grip']['rated']*100:.0f} % rated, reached {M['a_grip']['ok']}; seated {M['a_grip_seated']['rated']*100:.0f} % rated, reached {M['a_grip_seated']['ok']}")
    P(f"gust 5 m/s on the octopus: {M['gust_force']:.1f} N (est.)")
    P("(c) hands on the breast bar while hovering dz above the seat:")
    for r in M["c_grip"]:
        P(f"   dz {mm(r['dz'])}: hold {r['rated']*100:.0f} % rated, gap {mm(r['gap'])}, side pull {r['pull_side']:.1f} N, down pull {r['pull_down']:.1f} N per hand" + ("" if r["ok"] else "  NOT REACHED"))
    P(f"flown: discs {mm(M['tulip_disc_z'])} above the ground, nearest disc edge {mm(M['tulip_disc_gap'])} from the horse; wash shares {M['wash_hover']['share']}"
      f" -> download {M['wash_hover']['download']:.1f} N in hover; ground effect x{M['ground_effect']['over_ground']:.4f} (ground), x{M['ground_effect']['over_back']:.4f} (back)")
    P(f"   hover {M['P_hover']:.0f} W, T/W {M['TW']:.2f}; capture {M['capture']['xy']*1e3:.1f} mm / {M['capture']['yaw']} deg against station error"
      f" {M['capture']['station']['calm']*1e3:.0f} / {M['capture']['station']['turbulence']*1e3:.0f} mm; funnel needed {mm(M['capture']['funnel_needed'])} mm; hand position error {M['arm_backlash_mm']:.1f} mm (est.)")
    gu = M["getup"]
    P(f"get up from lying with the octopus on: front knee {gu['knee']:.1f} N m = {gu['of_rated']*100:.0f} % rated, {gu['of_peak']*100:.0f} % peak (motor {gu['motor']:.1f} N m,"
      f" allowed {cg_time(gu['motor']):.0f} s); horse alone {gu['knee_horse_alone']:.1f} N m")
    for k, lab in (("seq_ground", "GROUND MOUNT (recommended)"), ("seq_air", "AIR MOUNT (fast option)"), ("seq_takeoff", "TAKE-OFF FROM THE HORSE")):
        P(f"{lab}: total {M[k + '_total']:.0f} s")
        for i, (s, t, c) in enumerate(M[k], 1):
            P(f"   | {i} | {s} | {t:.1f} | {c} |")
    lo = M["liftoff"]
    P(f"air mount flight energy {M['air_energy_wh']:.1f} Wh; take-off energy {M['takeoff_energy_wh']:.1f} Wh; lift-off clears the collar in {lo['t_clear']:.2f} s at {lo['accel']:.1f} m/s2"
      f" (max {lo['a_max']:.1f}); bore gap {mm(lo['bore_gap'])} mm; horse load drops by {lo['unload']:.0f} N to {lo['horse_after']:.0f} N")
    P("\n== D3 BEHAVIOURS ==")
    for r in S["gaits"]["rows"]:
        P(f"   {r['gait']} {r['hz']:.1f} Hz: {r['v']:.2f} m/s, swing {r['swing']:.2f} s, hip {r['hip_rate']:.0f} deg/s, Froude {r['froude']:.3f}")
    t = S["gaits"]["turn"]
    P(f"   turn in place: {t['deg_per_cycle']:.0f} deg per cycle, {t['rate']:.0f} deg/s at 1.25 Hz, full turn {t['t_360']:.1f} s")
    P("carry: | payload kg | x mm | front foot N | change % | tip forward deg | neck moment N m |")
    for r in S["carry"]["rows"]:
        P(f"   | {r['payload']:.0f} | {mm(r['x'])} | {r['front_foot']:.0f} | {r['front_up']*100:+.0f} | {r['tip_front']:.1f} | {r['neck_moment']:.1f} |")
    P(f"   payload at 400 mm that tips the centaur forward: {S['carry']['tip_mass_at_400']:.1f} kg")
    for lab, L in S["levade"].items():
        P(f"levade, arms in '{lab}': CoM {mm(L['cx'])} ahead of and {mm(L['cz'])} above the rear hips; arm hold {L['hold']*100:.0f} % rated;"
          f" props alone from standing {L['T_props_only']:.0f} N = {L['T_of_front4']*100:.0f} % of the four front props; balance angle with feet under hips {L['theta_balance_feet_under_hips']:.0f} deg")
        P("   | pitch deg | thrust % hover | hind foot ahead of hip mm | hind hip N m per leg | allowed s | hind knee N m | ok |")
        for r in L["rows"]:
            ts = "cont." if r["time"] == float("inf") else f"{r['time']:.0f}"
            P(f"   | {r['theta']} | {r['f']*100:.0f} | {mm(r['x_foot'])} | {r['hip']:.1f} | {ts} | {r['knee']:.1f} | {'yes' if r['hold_ok'] else 'no'} |")
        for r in L["push"]:
            P(f"   push to {r['theta']} deg, thrust {r['f']*100:.0f} %: {r['E']:.0f} J = {r['share']*100:.0f} % of the {r['work_avail']:.0f} J the two front knees can do at peak torque; mean front force x{r['of_static']:.2f} static")
    ll = S["leap_limit"]
    P(f"all eight props flat out: {ll['T_max']:.0f} N = {ll['tw']:.2f} x centaur weight, {ll['P_max']:.0f} W, {ll['amps']:.0f} A per pack")
    P("leap: | thrust % hover | effective weight N (%) | prop power W | A per pack (C) | take-off m/s | rise mm | air time s | burst Wh | landing force N |")
    for r in S["leap"]:
        P(f"   | {r['f']*100:.0f} | {r['eff_weight']:.0f} ({r['eff_share']*100:.0f}) | {r['P']:.0f} | {r['amps']:.0f} ({r['c_rate']:.0f}C, sag {r['sag']:.1f} V) | {r['v']:.2f} | {mm(r['rise'])} | {r['air']:.2f} | {r['E_burst']:.2f} | {r['land_force']:.0f} |"
          f"   push {r['t_push']:.2f} s, knee peak {r['knee_peak']:.1f} N m, {r['knee_power']:.0f} W, per prop {r['per_prop']:.1f} N, length at 0.84 m/s {mm(r['length_at_trot'])}")
    s = S["steps"]
    P(f"step: body level {mm(s['level'])}, with 10 deg pitch {mm(s['pitch10'])}, recommended {mm(s['recommended'])}; stairs {s['stair_deg']:.0f} deg need {mm(s['stair_leg_diff'])} leg length difference")
    c = S["scout"]
    P(f"scout: hover {c['P_hover']:.0f} W -> {c['hover_min']:.1f} min; {c['sortie_min']:.0f} min sortie {c['sortie_wh']:.0f} Wh = {c['sortie_share']*100:.0f} % of the octopus's usable energy;"
      f" recharge {c['recharge_min']:.0f} min at {c['i_charge']:.0f} A ({c['pins_per_pole']} pins per pole); costs the horse {c['horse_walk_lost_min']:.0f} min of walking;"
      f" horse alone {c['P_horse_alone']:.0f} W -> {c['horse_alone_min']:.0f} min; pair walking on all four packs {c['pair_walk_min']:.0f} min")
    rp_ = S["rest_power"]
    P(f"rest: standing knee copper loss front {rp_['front']['watts']:.1f} W ({rp_['front']['knee']:.1f} N m, {rp_['front']['amps']:.1f} A), rear {rp_['rear']['watts']:.1f} W; total {rp_['standing_w']:.0f} W; lying 0 W")
    A = S["assist"]
    P("\n== D4 THRUST ASSIST ==")
    P(f"wash onto the horse: guard six props {A['wash6']['share']} -> {A['wash6']['download']/A['wash6']['n']*100:.0f} % of thrust lost; tulip eight {A['wash8']['download']/A['wash8']['n']*100:.0f} %")
    P("| thrust % of octopus hover | props | N per prop | net lift N | leg load % | power W | minutes on the octopus packs | knee rms margin (trot) | hip peak margin (trot) | W per N | dB vs hover (est.) |")
    for r in A["rows"]:
        P(f"| {r['f']*100:.0f} | {r['n']} | {r['per_prop']:.1f} | {r['net']:.0f} | {r['leg_factor']*100:.0f} | {r['P']:.0f} | {r['minutes']:.0f} | x{r['knee_rms_margin']:.2f} | x{r['hip_peak_margin']:.2f} | {r['w_per_n']:.1f} | {r['db_vs_hover']:.0f} |")
    h = A["hip_fix"]
    P(f"smallest assist that brings the trot hip peak inside the CyberGear limit: net {h['net']:.1f} N, thrust {h['T']:.1f} N, {h['P']:.0f} W (walking {A['walk_W']:.0f} W)")
    P(f"roll: guard props on one side at full thrust {A['roll_moment']:.1f} N m against {A['tip_moment']:.1f} N m to tip the standing centaur ({A['roll_share']*100:.0f} %)")
    F = S["falls"]
    P("\n== D6 FALLS ==")
    for lab in ("guard", "pack"):
        f = F[lab]
        P(f"{lab}: CoM {mm(f['zc'])}, I about the feet {f['I']:.2f} kg m2, tips at {f['tip_deg']:.1f} deg, first strike at {f['strike_deg']:.0f} deg;"
          f" shove that tips it from square: {f['omega_crit']:.2f} rad/s = {f['push_crit_Ns']:.1f} N s")
        for r in f["times"]:
            P(f"   from the tip point at {r['omega0']:.1f} rad/s: 30 deg after {r['t_30']:.2f} s, 45 deg after {r['t_45']:.2f} s, strike after {r['t_strike']:.2f} s at {r['omega_strike']:.1f} rad/s, {r['E']:.0f} J")
        for k, r in f["recovery"].items():
            P(f"   recoverable shove, {k}: {r['omega']:.2f} rad/s = {r['push_Ns']:.1f} N s (a {r['dv']:.2f} m/s bump)")
        if "hard_shove" in f:
            hs = f["hard_shove"]
            P(f"   shove 1.3 x critical ({hs['omega0']:.2f} rad/s): with props fell = {hs['fell_with_props']}; strike energy {hs['E_no']:.0f} J without, {hs['E_props'] if hs['E_props'] is None else round(hs['E_props'])} J with")
    r = F["responses"]
    P(f"responses: detect {r['detect']:.2f} s, prop spin-up {r['spin']:.2f} s (total {r['prop_total']:.2f} s); latch {r['latch']:.1f} s + bars {r['bars']:.1f} s -> fly off {r['fly_off']:.1f} s;"
      f" arm brace {r['arm_brace']:.2f} s, tuck {r['arm_tuck']:.2f} s; brace force {r['brace_force']:.0f} N per arm, {r['brace_energy']:.0f} J for three arms; latch pin margin in a 3 g hit x{r['pin_margin']:.1f}")
    P("\n== PARTS ==")
    for p in PARTS:
        P(f"| {p['part']} | {p['use']} | {p['spec']} | {'$%.2f' % p['usd'] if p['usd'] else '-'} | {p['src']} | {p['ok']} |")
    for p in PRINT:
        P(f"| {p['part']} | {p['mat']} | {p['box']} | {p['orient']} | {p['note']} |")


if __name__ == "__main__":
    S = compute()
    report(S)
    json.dump(clean(S), open(OUT + "integ.json", "w"), indent=1)
    print(f"\nnumbers done in {time.time() - T0:.0f} s")
    if "--nofig" not in sys.argv:
        sheets(S)
    print(f"all done in {time.time() - T0:.0f} s")
