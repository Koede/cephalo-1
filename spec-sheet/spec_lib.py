"""Model of one Cephalo-1 pin hinge arm for the spec sheet (no helix: hinge axes alternate 90 deg).

Own forward kinematics with CAD masses (spec-sheet/cad_alt90, root_gear.json, horn_bearing.json, fan_pod.json).
World frame: robot centre at the origin, z up. The arm root sits on the hub circle at (HUB_R, 0, 0) and points
radially out along +x when straight. Base frame columns: local x = world up, local y = -world y, local z = radial.
Joint k sits one pitch along the parent's z; its axis in the parent frame is
  alt90 : k even -> local y (pitch, raises and lowers the arm when straight), k odd -> local x (yaw)
  helix60: (cos(90 + 60k), sin(90 + 60k), 0)    [the old main design, for comparison]
Units SI. [A] marks an assumption.
"""
import math, json, os
import numpy as np

G = 9.81
HERE = os.path.dirname(os.path.abspath(__file__))
OCTO = os.path.dirname(HERE)

# ---------------- geometry (CAD) ----------------
N = 13
PITCH = 1.000 / N            # 76.92 mm
OD = 0.0571; R_OUT = OD / 2
HUB_R = 0.1245               # m: smallest hub radius where the eight root flanges clear (mantle ports check) [A: mantle not drawn]
ROOT_SERVOS = 5              # STS3250 in joints 0-4

# ---------------- masses (g, from CAD json) ----------------
M_VERT, M_TIP_VERT, M_LID, HW = 31.0, 37.2, 2.5, 5.0
SERVO_G = {"STS3250": 74.5, "STS3215": 55.0}
HB_LID, HB_FORK, HB_BRG = 0.83, 0.94, 6.0       # horn bearing extras on joints 1-4 (horn_bearing.json)
IDLER_PIN_EXTRA = 0.8                            # 7075 idler pin instead of the printed stub, joints 1-4 [A, DESIGN]
WIRE_G_PER_ARM = 25.0                            # 16 AWG pair for the claw ESC along the arm [A, DESIGN]
ROOT_FIXED_G = dict(socket=30.1, end_cap=1.9, bearing_cap=2.1, pinion=6.4, idler_pin=1.3, bearings=9.7, hw=5.4, servo0=74.5)
GEAR_G = 15.0                                    # 40T sector: turns with joint 0, sits on its axis
# tip: folding fan pod (fan_pod.json): palm+finger+post 30.9, pod 13.6, STS3032 20, hw 8, pads 2, motor 132, prop 26, ESC 20
TIP_G = 252.5
TIP_CG = 0.045               # m beyond the tip plate, pod upright [A: motor and prop sit 34-102 mm out, palm at 0-30 mm]
THRUST_PT = 0.102            # m: prop plane above the tip plate; thrust acts along the arm axis through the pod hinge
SERVO_CG_BACK = 0.0116       # m: servo case centre sits 11.6 mm proximal of its own joint (spline 11 mm from the case end)
VERT_CG = 0.0404             # m: vertebra centroid from its proximal joint (STL centroid)
TIP_VERT_CG = 0.0488

def servo_name(j):
    return "STS3250" if j < ROOT_SERVOS else "STS3215"

# ---------------- servos ----------------
SERVO = {   # 12 V; stall and stall current from the makers' pages (catalog/servo_and_tip_sources.json)
    "STS3250": dict(stall=4.90, speed=4.4, I_stall=4.2, backlash_deg=0.5, g=74.5, price=40),
    "STS3215": dict(stall=2.94, speed=4.7, I_stall=2.7, backlash_deg=0.5, g=55.0, price=19),
    "STS3032": dict(stall=0.44, speed=8.7, I_stall=1.0, backlash_deg=0.8, g=20.0, price=11),   # stall current [A]
}
GEAR_RATIO = 2.0
GEAR_ETA = 0.98              # spur mesh + two ball bearings, lubricated anodized 7075 [A]
CAP = np.array([SERVO["STS3250"]["stall"] * GEAR_RATIO * GEAR_ETA] + [SERVO[servo_name(j)]["stall"] for j in range(1, N)])
CAP[0] = 9.8                 # DESIGN uses 9.8 N.m (no mesh loss); keep that for continuity, the 2 % mesh loss is reported separately
SPEED = np.array([SERVO["STS3250"]["speed"] / GEAR_RATIO] + [SERVO[servo_name(j)]["speed"] for j in range(1, N)])

def node_masses():
    """kg per node k (k = 0..12): everything between joint k and joint k+1, and its CG distance from joint k along the link"""
    m = np.zeros(N); c = np.zeros(N)
    wire = WIRE_G_PER_ARM / N
    for k in range(N):
        parts = []   # (g, distance from joint k along the link, m)
        if k < N - 1:
            sv = servo_name(k + 1)
            parts += [(M_VERT, VERT_CG), (SERVO_G[sv], PITCH - SERVO_CG_BACK), (M_LID, PITCH - SERVO_CG_BACK), (HW, PITCH * 0.5)]
            if 1 <= k + 1 <= 4:   # horn bearing on the lid of joint k+1 and the 7075 idler pin there
                parts += [(HB_LID + HB_BRG + IDLER_PIN_EXTRA, PITCH)]
            if 1 <= k <= 4:       # the fork of joint k carries the bearing sleeve
                parts += [(HB_FORK, 0.0)]
        else:
            parts += [(M_TIP_VERT, TIP_VERT_CG), (HW, 0.02)]
        parts += [(wire, PITCH / 2)]
        if k == 0: parts += [(GEAR_G, 0.0)]
        g = sum(p[0] for p in parts); m[k] = g / 1000; c[k] = sum(p[0] * p[1] for p in parts) / g
    return m, c

MASS, CGD = node_masses()
M_TIP = TIP_G / 1000

def node_inertia(k):
    """3x3 inertia (kg m^2) about the node CG in the node frame: ring tube + servo box, rough [A]"""
    m = MASS[k]; r = R_OUT - 0.0012; L = PITCH
    Izz = m * r * r * 0.7; Ixx = Iyy = m * (r * r / 2 * 0.7 + L * L / 12 * 0.6)
    return np.diag([Ixx, Iyy, Izz])

# ---------------- kinematics ----------------
BASE_R = np.column_stack([[0, 0, 1.0], [0, -1.0, 0], [1.0, 0, 0]])

def axis_local(k, arr="alt90"):
    if arr == "alt90":
        return np.array([0, 1.0, 0]) if k % 2 == 0 else np.array([1.0, 0, 0])
    if arr == "helix60":
        p = math.pi / 2 + k * math.pi / 3; return np.array([math.cos(p), math.sin(p), 0])
    raise ValueError(arr)

def rot(a, t):
    a = a / np.linalg.norm(a); K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * K + (1 - math.cos(t)) * K @ K

class Arm:
    def __init__(s, arr="alt90", hub_r=HUB_R, tip_g=TIP_G, masses=None):
        s.arr = arr; s.hub_r = hub_r; s.m_tip = tip_g / 1000
        s.mass = MASS.copy() if masses is None else masses
        s.cgd = CGD.copy(); s.base = np.array([hub_r, 0.0, 0.0])
        s.ax_loc = [axis_local(k, arr) for k in range(N)]

    def fk(s, q):
        """joint origins o[k], world axes a[k], node frames R[k] (after joint k), node CGs, tip plate p, tip frame"""
        R = BASE_R.copy(); p = s.base.copy()
        o = np.zeros((N, 3)); a = np.zeros((N, 3)); Rn = np.zeros((N, 3, 3)); cg = np.zeros((N, 3)); marks = [p.copy()]
        for k in range(N):
            o[k] = p; a[k] = R @ s.ax_loc[k]
            R = R @ rot(s.ax_loc[k], q[k]); Rn[k] = R
            cg[k] = p + R[:, 2] * s.cgd[k]
            p = p + R[:, 2] * PITCH; marks.append(p.copy())
        return dict(o=o, a=a, R=Rn, cg=cg, tip=p, tipR=R, marks=np.array(marks))

    def tip_points(s, f):
        z = f["tipR"][:, 2]
        return f["tip"] + z * TIP_CG, f["tip"] + z * THRUST_PT

    def wrench(s, q, F=np.zeros(3), f=None, Fpt="thrust", extra=None):
        """moment (world, N.m) and force (world, N) that everything outboard of each joint puts on its parent, about the joint.
        F acts at the prop (Fpt='thrust'), the tip plate ('tip') or a given world point. extra: list of (point, force)."""
        f = f or s.fk(q); g = np.array([0, 0, -G])
        tcg, tth = s.tip_points(f)
        pF = tth if isinstance(Fpt, str) and Fpt == "thrust" else (f["tip"] if isinstance(Fpt, str) else np.asarray(Fpt))
        Ms = np.zeros((N, 3)); Fs = np.zeros((N, 3))
        for j in range(N):
            oj = f["o"][j]
            M = np.cross(f["cg"][j:] - oj, s.mass[j:, None] * g).sum(0) + np.cross(tcg - oj, s.m_tip * g) + np.cross(pF - oj, F)
            Fo = s.mass[j:].sum() * g + s.m_tip * g + F
            if extra:
                for pt, fe in extra: M += np.cross(pt - oj, fe); Fo = Fo + fe
            Ms[j] = M; Fs[j] = Fo
        return Ms, Fs, f

    def joint_frame(s, f, j):
        """rows alpha (hinge axis), beta, zeta (link direction of the parent at the joint)"""
        a = f["a"][j] / np.linalg.norm(f["a"][j])
        z = (BASE_R if j == 0 else f["R"][j - 1])[:, 2]
        z = z - a * (a @ z); z /= np.linalg.norm(z); b = np.cross(z, a)
        return np.stack([a, b, z])

    def need(s, q, F=np.zeros(3), Fpt="thrust", extra=None, cap=CAP):
        Ms, Fs, f = s.wrench(q, F, Fpt=Fpt, extra=extra)
        tau = np.einsum("ij,ij->i", Ms, f["a"])
        return np.abs(tau) / cap, tau, Ms, Fs, f

    def jac_points(s, q, f=None):
        """Jacobians of the tip plate and each node CG, and angular Jacobian columns"""
        f = f or s.fk(q)
        Jt = np.zeros((3, N)); Jc = np.zeros((N, 3, N)); Jw = np.zeros((3, N))
        for j in range(N):
            a = f["a"][j]; Jw[:, j] = a
            Jt[:, j] = np.cross(a, f["tip"] - f["o"][j])
            for k in range(j, N): Jc[k][:, j] = np.cross(a, f["cg"][k] - f["o"][j])
        return Jt, Jc, Jw, f

    def mass_matrix(s, q):
        Jt, Jc, Jw, f = s.jac_points(q)
        M = np.zeros((N, N))
        for k in range(N):
            Jwk = Jw.copy(); Jwk[:, k + 1:] = 0
            I = f["R"][k] @ node_inertia(k) @ f["R"][k].T
            M += s.mass[k] * Jc[k].T @ Jc[k] + Jwk.T @ I @ Jwk
        tcg, _ = s.tip_points(f)
        Jtc = np.zeros((3, N))
        for j in range(N): Jtc[:, j] = np.cross(f["a"][j], tcg - f["o"][j])
        Itip = s.m_tip * 0.06 ** 2 * np.eye(3)        # pod with motor and prop, about 60 mm radius of gyration [A]
        M += s.m_tip * Jtc.T @ Jtc + Jw.T @ Itip @ Jw
        return M, f

def self_clear(f, shrink=0.9):
    """smallest centre-line distance between non-adjacent links minus their diameter (m); negative = overlap"""
    P = f["marks"]; best = 9.0
    for i in range(len(P) - 1):
        for k in range(i + 2, len(P) - 1):
            d = seg_dist(P[i], P[i + 1], P[k], P[k + 1]); best = min(best, d - OD * shrink)
    return best

def seg_dist(p1, q1, p2, q2):
    d1 = q1 - p1; d2 = q2 - p2; r = p1 - p2; a = d1 @ d1; e = d2 @ d2; f_ = d2 @ r
    c = d1 @ r; b = d1 @ d2; den = a * e - b * b
    s = np.clip((b * f_ - c * e) / den, 0, 1) if den > 1e-12 else 0.0
    t = (b * s + f_) / e
    if t < 0: t = 0; s = np.clip(-c / a, 0, 1)
    elif t > 1: t = 1; s = np.clip((b - c) / a, 0, 1)
    return float(np.linalg.norm((p1 + d1 * s) - (p2 + d2 * t)))

def sphere(n):
    i = np.arange(n) + 0.5; phi = np.arccos(1 - 2 * i / n); th = math.pi * (1 + 5 ** 0.5) * i
    return np.stack([np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)], 1)
