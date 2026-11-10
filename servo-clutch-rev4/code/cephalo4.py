"""Cephalo-1 rev 4: the integrated design (single source of truth for geometry, masses, thrust and static loads).

Brings together (see project docs):
  * arm  = 8 servo-in-link joints p-y-p-y-p-y-p-y, 56 mm apart (s = 50..442 mm), each servo body inside its link with
           the horn on the joint axis driving the next link through the adopted radial detent clutch (clutch set v1).
           (The #5 geared double hinge was checked and does not package: an STS3215 + 2:1 capstan needs ~90 mm per
           joint vs the study's 42-67 mm, and without the 2:1 stage the joint only gets half the servo torque.)
  * impact protection = winch-tensioned detent clutches (impact-compliant-joint.md), unchanged parts:
           j0 STS3250 clutch 1.2 x stall, j1 root yaw rigid, j2-j7 STS3215 clutch 0.9 x stall (at the 60 N 'firm' line)
  * no STS3032 anywhere in the arm (clutches on STS3032 hit their stops in radial hits)
  * pod  = 2207-class 6S motor + 6 in claw-prop (3 finger-blades, fold 0-60 deg), puller
  * body = rev 3 mantle (plate r 125, dome a 125 / c 150), 90 mm EDF under the plate, Teensy 4.1
Frames: body origin = centre of the plate underside, +z up, metres.  Arm i root at azimuth 22.5 + 45 i deg, wall face
x = 104 mm, axis z = 39 mm.  Arm local x along the arm, z = arm up; positive pitch = tip DOWN.
Masses marked (est.) are estimates; thrust of the claw-prop is an unmeasured estimate band.
"""
import numpy as np
from scipy.optimize import least_squares, nnls

G = 9.81
DEG = np.pi / 180
MM = 1e-3
# ------------------------------------------------------------------ geometry
ROOT_R, ROOT_Z = 104 * MM, 39 * MM
S_J = (50 + 56 * np.arange(8)) * MM   # joint axes (arc length from the wall face): 56 mm = STS3215 body + horn clearance
TYP = "pypypypy"
RG = 0.0                         # single pivot (kept as a parameter: RG > 0 gives a synchronised double hinge)
SPIGOT = 478 * MM                # tip coupler end (electronics bay 454..478)
NJ = 8
LIM = np.radians([[-59, 55], [-30, 30]] + [[-100, 100]] * 6)       # j0 from the mantle check, j1 from neighbours, shells +-100
CLAW_R = 0.0762                  # 6 in claw-prop blade radius
BLADE_OFF = 0.036                # spigot -> blade root plane (2207 motor is 1-2 mm taller than the 2306 pod)
FOLD_STAND = 60 * DEG

# ------------------------------------------------------------------ servos and clutches
#             rated   stall   mass    L     W     H (mm)        datasheets; STS3250 rated = stall/3 (assumed)
SV = {"STS3215": (0.981, 2.94, 0.055, 45.2, 24.7, 35.0),
      "STS3250": (1.63, 4.90, 0.0745, 45.2, 24.7, 35.0),
      "STS3032": (0.147, 0.441, 0.020, 23.2, 12.1, 28.5)}
SERVO = ["STS3250"] + ["STS3215"] * 7
# Servo horn on the joint axis: joint moment = servo torque (N = 1).  (A double hinge driven directly on one pivot gives
# N = 0.5; tried on j4-j7, j4 overloads in the walk.)
N_DRIVE = np.ones(8)
STALL_S = np.array([SV[n][1] for n in SERVO]); RATED_S = np.array([SV[n][0] for n in SERVO])
STALL = STALL_S * N_DRIVE                     # joint-level moment limits (N m)
RATED = RATED_S * N_DRIVE
CLUTCH_FRAC = np.array([1.2, 0.0, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])     # x servo stall at T_FIRM (at the servo output); 0 = rigid
CLUTCH_TRAVEL = np.radians([25, 0, 40, 40, 40, 40, 40, 40])
T_LINE = {"soft": 25.0, "firm": 60.0, "tight": 150.0}                  # winch modes, N
T_FIRM = 60.0
ARMATURE_S = np.array([0.02] + [0.012] * 7)        # reflected rotor inertia at the servo output, kg m^2 (est., ~1:345)
ARMATURE = ARMATURE_S * N_DRIVE ** 2               # ... seen at the joint
GEAR_LIMIT = 3.0 * STALL                           # assumed servo gear-train strip limit (unmeasured), joint level

# ------------------------------------------------------------------ masses (kg)
# per joint: printed servo cradle/shell ends, horn-side and idler-side bosses, hollow brass pin for the line (est.)
M_HINGE_J = np.full(8, 0.012)
M_HINGE = float(M_HINGE_J.mean())
M_CLUTCH = {"STS3215": 0.0238, "STS3250": 0.038}  # clutch set v1: cam + rocker + 624/623/6800 (626/623/6802 root) + bolts
LAM = 0.070              # link structure kg/m (printed PETG cradles/shells, est.)
M_TIPBAY = 0.020         # tip coupler: XIAO node, IMU, 6S 45 A ESC (est.)
M_POD = 0.115            # 2207 motor (34 g) + 6 in claw blades + fold servo + slider + palm (est.)
M_LINE = 0.013           # Dyneema line, series spring, hollow pins
M_WINCH = 0.024          # N20 worm winch + block (mounted on the mantle sector)
M_BODY = 2.62            # mantle, dome, EDF 450 g, battery, Teensy, wiring (8.6 kg rev 3 robot - 8 x 747 g arms)


def arm_mass_table():
    rows = []
    for k, n in enumerate(SERVO):
        rows.append((f"j{k} {n}", SV[n][2]))
    rows.append(("joint shells/bosses/pins x8", float(M_HINGE_J.sum())))
    rows.append(("clutches (j0 root, j2-j7)", sum(M_CLUTCH[SERVO[k]] for k in range(NJ) if CLUTCH_FRAC[k] > 0)))
    rows.append(("link structure", LAM * SPIGOT))
    rows.append(("tip bay", M_TIPBAY)); rows.append(("pod (2207 + 6 in claw)", M_POD)); rows.append(("line/spring/pins", M_LINE))
    return rows


M_ARM = sum(v for _, v in arm_mass_table())          # moving arm (excl. winch)
M_ROBOT = M_BODY + 8 * (M_ARM + M_WINCH)
W_ROBOT = M_ROBOT * G
# ------------------------------------------------------------------ thrust (N)
EDF_MAX = 42.2                                      # JP Hobby 90 mm 12-blade, 6S
FAN_MAX = {"pess": 7.8 * 1.30, "mid": 0.5 * (7.8 * 1.30 + 10.8 * 1.40), "opt": 10.8 * 1.40}   # 6 in claw-prop (est.)


def tw(case):
    return (EDF_MAX + 8 * FAN_MAX[case]) / W_ROBOT


# ------------------------------------------------------------------ kinematics
def arm_base(i, body_p=np.zeros(3), body_R=np.eye(3)):
    a = np.radians(22.5 + 45 * i)
    x = np.array([np.cos(a), np.sin(a), 0]); y = np.array([-np.sin(a), np.cos(a), 0]); z = np.array([0, 0, 1.0])
    return body_R @ np.stack([x, y, z], 1), body_p + body_R @ (ROOT_R * x + ROOT_Z * z)


def _rot(ax, ang):
    c, s = np.cos(ang), np.sin(ang)
    if ax == "p":   # about local y, +ve -> x toward -z
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def arm_fk(i, q, body_p=np.zeros(3), body_R=np.eye(3), clutch=None):
    """Segments [(s0, s1, R, p0)] along the arm (straight pieces between pivots) + tip (R, p) at the spigot.
    Joint k turns q_k + clutch_k.  Written for an optional double hinge (pivots at s_k -/+ RG turning q/2 + c and q/2);
    rev 4 uses RG = 0, i.e. one pivot (the MuJoCo model keeps the two coupled half-joints at the same point)."""
    R, p = arm_base(i, body_p, body_R)
    segs, s = [], 0.0
    for k in range(NJ):
        c = 0.0 if clutch is None else clutch[k]
        sa = S_J[k] - RG
        segs.append((s, sa, R.copy(), p.copy())); p = p + R[:, 0] * (sa - s)
        R = R @ _rot(TYP[k], q[k] / 2 + c)
        segs.append((sa, sa + 2 * RG, R.copy(), p.copy())); p = p + R[:, 0] * 2 * RG
        R = R @ _rot(TYP[k], q[k] / 2)
        s = sa + 2 * RG
    segs.append((s, SPIGOT, R.copy(), p.copy())); p = p + R[:, 0] * (SPIGOT - s)
    return segs, (R, p)


def at_s(segs, s):
    for s0, s1, R, p0 in segs:
        if s <= s1 + 1e-12:
            return p0 + R[:, 0] * (s - s0), R
    s0, s1, R, p0 = segs[-1]
    return p0 + R[:, 0] * (s - s0), R


def joint_frames(segs):
    """Per joint: (centre, axis world, R of the coupler)."""
    out = []
    for k in range(NJ):
        c, R = at_s(segs, S_J[k])
        out.append((c, R[:, 1] if TYP[k] == "p" else R[:, 2], R))
    return out


def foot_drop(fold=FOLD_STAND):
    return BLADE_OFF + CLAW_R * np.sin(fold) + 0.004


def tip_point(i, q, body_p, body_R=np.eye(3), lead=None):
    _, (R, p) = arm_fk(i, q, body_p, body_R)
    return p + R[:, 0] * (foot_drop() if lead is None else lead), R[:, 0]


def ik(i, target, axis, q0, q_nom, body_p, body_R=np.eye(3), lead=None, w_ax=0.3, w_reg=0.02, lim=None):
    lo, hi = (LIM if lim is None else lim).T
    def res(q):
        f, u = tip_point(i, q, body_p, body_R, lead)
        r = [(f - target) * 10]
        if axis is not None:
            r.append((u - axis) * w_ax)
        r.append(w_reg * (q - q_nom))
        return np.concatenate(r)
    sol = least_squares(res, np.clip(q0, lo + 1e-6, hi - 1e-6), bounds=(lo, hi), xtol=1e-12, ftol=1e-12, max_nfev=300)
    return sol.x


# ------------------------------------------------------------------ mass points and static loads
def arm_points(segs, tip):
    """(positions, masses, arc s) of the arm's lumped masses.  Servo k sits in the link just proximal of joint k."""
    P, m, s = [], [], []
    for k, n in enumerate(SERVO):
        sk = S_J[k] - (SV[n][3] / 2 - 11.4) * MM          # servo body centre: shaft 11.4 mm from its output end
        P.append(at_s(segs, sk)[0]); m.append(SV[n][2]); s.append(sk)
        P.append(at_s(segs, S_J[k] - RG - 0.002)[0]); m.append(M_HINGE_J[k] / 2 + (M_CLUTCH[n] if CLUTCH_FRAC[k] > 0 else 0)); s.append(S_J[k] - RG - 0.002)
        P.append(at_s(segs, S_J[k] + RG + 0.002)[0]); m.append(M_HINGE_J[k] / 2); s.append(S_J[k] + RG + 0.002)
    for x in np.linspace(0.0, SPIGOT, 20)[:-1] + SPIGOT / 38:
        P.append(at_s(segs, x)[0]); m.append((LAM * SPIGOT + M_LINE) / 19); s.append(x)
    R, p = tip
    P.append(p - R[:, 0] * 0.012); m.append(M_TIPBAY); s.append(SPIGOT - 0.012)
    P.append(p + R[:, 0] * 0.024); m.append(M_POD); s.append(SPIGOT + 0.024)
    return np.array(P), np.array(m), np.array(s)


def joint_torques(i, q, body_p, body_R, F_ext, P_ext, acc=np.array([0, 0, -G])):
    """Static joint moments (N m, about each joint axis, servo-side = average over the two pivots) from gravity/
    acceleration on the arm masses plus an external force F_ext applied at P_ext (world)."""
    segs, tip = arm_fk(i, q, body_p, body_R)
    P, m, s = arm_points(segs, tip)
    F = m[:, None] * acc[None, :]
    P = np.vstack([P, P_ext]); F = np.vstack([F, F_ext]); s = np.append(s, 9.0)
    tau = np.zeros(NJ)
    for k in range(NJ):
        acc_t = 0.0
        for sp in (S_J[k] - RG, S_J[k] + RG):
            c, R = at_s(segs, sp)
            a = R[:, 1] if TYP[k] == "p" else R[:, 2]
            sel = s > sp
            acc_t += a @ np.cross(P[sel] - c, F[sel]).sum(0)
        tau[k] = acc_t / 2
    return tau


def foot_forces(feet, body_c, Fz_needed):
    """Vertical foot reactions (>= 0) that carry Fz_needed and balance roll/pitch about body_c (least-norm, NNLS)."""
    n = len(feet)
    A = np.vstack([np.ones(n), feet[:, 1] - body_c[1], feet[:, 0] - body_c[0]])
    b = np.array([Fz_needed, 0, 0])
    # min ||f||^2 s.t. A f = b, f >= 0  -> weighted NNLS on the stacked system
    Aw = np.vstack([A * 100, np.eye(n) * 0.05]); bw = np.concatenate([b * 100, np.zeros(n)])
    f, _ = nnls(Aw, bw)
    return f


def robot_com(arms_q, body_p, body_R=np.eye(3)):
    tot, mm = M_BODY * (body_p + body_R @ np.array([0, 0, 0.05])), M_BODY
    for i, q in enumerate(arms_q):
        segs, tip = arm_fk(i, q, body_p, body_R)
        P, m, _ = arm_points(segs, tip)
        tot = tot + (P * m[:, None]).sum(0); mm += m.sum()
    return tot / mm


# ------------------------------------------------------------------ collision helpers
def arm_capsules(i, q, body_p, body_R=np.eye(3), n=40, s_min=None):
    segs, tip = arm_fk(i, q, body_p, body_R)
    ss = np.linspace(S_J[0] + 0.015 if s_min is None else s_min, SPIGOT, n)   # from just past the root pivot boss
    P = np.array([at_s(segs, s)[0] for s in ss])
    r = np.interp(ss, [0, 0.2, SPIGOT], [0.027, 0.025, 0.020])
    return P, r, tip


def body_clearance(P, r, body_p):
    """Signed distance (approx) from arm capsule points to the dome ellipsoid / plate disc / EDF can."""
    q = P - body_p
    # dome: ellipsoid a=0.125, c=0.150 above z=0.006 (approximate distance by scaled radius)
    e = np.sqrt((q[:, 0] ** 2 + q[:, 1] ** 2) / 0.125 ** 2 + np.clip(q[:, 2] - 0.006, 0, None) ** 2 / 0.150 ** 2)
    d_dome = (e - 1) * 0.125
    rr = np.hypot(q[:, 0], q[:, 1])
    d_plate = np.where((q[:, 2] > -0.004) & (q[:, 2] < 0.010), rr - 0.125, np.hypot(np.clip(rr - 0.125, 0, None), np.minimum(np.abs(q[:, 2] - 0.003) - 0.003, 1)))
    d_edf = np.where((q[:, 2] < 0) & (q[:, 2] > -0.095), rr - 0.056, 1.0)
    return np.minimum(np.minimum(d_dome, d_plate), d_edf) - r


if __name__ == "__main__":
    print(f"arm (moving) {M_ARM*1000:.0f} g + winch {M_WINCH*1000:.0f} g | robot {M_ROBOT:.2f} kg | W {W_ROBOT:.1f} N")
    for k, v in arm_mass_table():
        print(f"   {k:28s} {v*1000:6.1f} g")
    for c in FAN_MAX:
        print(f"T/W {c}: fan {FAN_MAX[c]:.1f} N -> {tw(c):.2f}")
