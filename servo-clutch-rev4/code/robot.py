"""Simplified Cephalo-1 (current design state, see project doc claude/cephalo1-design-state.md) with a swappable tip wrist.

Frame: body origin = centre of the plate underside, +z up, metres.  Arm i root at azimuth 22.5 + 45 i deg.
Arm: 9 Feetech bus-servo joints, 4 universal nodes (pitch then yaw) + tip pitch.  Pitch +ve = tip goes DOWN.
The in-line pod is replaced by: wrist (BallDrive / OmniWrist) -> 2306 motor -> 5 in claw-prop.
"""
import numpy as np
from scipy.optimize import least_squares
from geom import frustum, sphere, torus, box, merge, frame_z, rot, Scene

MM = 1e-3
ROOT_R, ROOT_Z = 104 * MM, 39 * MM
J_POS = np.array([43.5, 78, 161, 196, 279, 307, 356, 378, 426]) * MM      # joint axes from wall face
J_TYPE = "pypypypyp"                                                       # p = pitch, y = yaw
SPIGOT = 458 * MM
J_LIM = np.radians([[-41, 41], [-19, 19], [-90, 90], [-90, 90], [-90, 90], [-80, 90], [-90, 90], [-90, 90], [-90, 90]])
SERVO = [(0.046, 0.034, 0.030)] * 3 + [(0.040, 0.030, 0.027)] * 2 + [(0.023, 0.013, 0.023)] * 4   # STS3250/3215/3032
STAND = np.radians([30, 0, 30, 0, 0, 0, 30, 0, 0])

WHITE = [0.90, 0.90, 0.88]; GREY = [0.55, 0.57, 0.60]; DARK = [0.20, 0.21, 0.23]; RED = [0.90, 0.15, 0.12]
CLAW_R = 0.0635         # 5 in claw-prop blade radius
BLADE_OFF = 0.034       # stalk end -> blade root plane


def arm_base(i):
    a = np.radians(22.5 + 45 * i)
    x = np.array([np.cos(a), np.sin(a), 0]); y = np.array([-np.sin(a), np.cos(a), 0]); z = np.array([0, 0, 1.0])
    return np.stack([x, y, z], 1), ROOT_R * x + ROOT_Z * z


def arm_fk(i, q, body_p=np.zeros(3), body_R=np.eye(3)):
    """Returns list of joint frames (R, p) world, plus tip (R, p).  Local x = along arm, z = arm 'up'."""
    R0, p0 = arm_base(i)
    R = body_R @ R0; p = body_p + body_R @ p0
    frames = []
    s_prev = 0.0
    for k in range(9):
        p = p + R[:, 0] * (J_POS[k] - s_prev); s_prev = J_POS[k]
        if J_TYPE[k] == "p":
            R = R @ rot([0, 1, 0], q[k])       # +ve rotates x toward -z (tip down)
        else:
            R = R @ rot([0, 0, 1], q[k])
        frames.append((R.copy(), p.copy()))
    p = p + R[:, 0] * (SPIGOT - s_prev)
    return frames, (R, p)


FREE = [0, 1, 2, 3, 4, 6, 8]


def arm_ik(i, target_c, lead, q0, q_nom, body_p, body_R=np.eye(3), w_reg=0.02):
    """Solve joints so that wrist centre (tip + t*lead) reaches target_c."""
    def res(x):
        q = q_nom.copy(); q[FREE] = x
        _, (R, p) = arm_fk(i, q, body_p, body_R)
        c = p + R[:, 0] * lead
        return np.concatenate([(c - target_c) * 10, w_reg * (x - q_nom[FREE])])
    lo, hi = J_LIM[FREE, 0], J_LIM[FREE, 1]
    lo = lo.copy(); hi = hi.copy(); lo[3], hi[3] = np.radians(-40), np.radians(40)   # j3 kept within +-40 as before
    x0 = np.clip(q0[FREE], lo + 1e-6, hi - 1e-6)
    sol = least_squares(res, x0, bounds=(lo, hi), xtol=1e-10, ftol=1e-10, max_nfev=200)
    q = q_nom.copy(); q[FREE] = sol.x
    return q


def body_mesh(S, body_p, edf_ang):
    o = body_p
    S.add(frustum(o, o + [0, 0, 0.006], 0.125, 0.125, GREY, n=40))
    # dome: ellipsoid a=125, c=150
    from geom import sphere as sph
    V, F, C = sph(np.zeros(3), 1.0, WHITE, n=36, m=18)
    V[:, 2] = np.maximum(V[:, 2], 0.0)
    V = V * [0.125, 0.125, 0.150] + o + [0, 0, 0.006]
    S.add((V, F, C))
    S.add(torus(o + [0, 0, 0.11], [0, 0, 1], 0.06, 0.006, GREY))          # crown vent ring
    # EDF under the plate
    S.add(frustum(o - [0, 0, 0.004], o - [0, 0, 0.09], 0.055, 0.046, DARK, n=28))
    S.add(torus(o - [0, 0, 0.004], [0, 0, 1], 0.05, 0.005, GREY))
    for k in range(12):
        a = edf_ang + k * np.pi / 6
        d = np.array([np.cos(a), np.sin(a), 0])
        S.add(box(o - [0, 0, 0.088] + d * 0.022, frame_z([0, 0, 1], d), [0.04, 0.008, 0.002], GREY), same=k > 0)


def arm_mesh(S, frames, tip, col_accent):
    pts = [frames[0][1] - frames[0][0][:, 0] * J_POS[0]] + [f[1] for f in frames] + [tip[1]]
    for k in range(len(pts) - 1):
        r = 0.016 - 0.007 * k / 9
        S.add(frustum(pts[k], pts[k + 1], r, r * 0.95, WHITE, n=12))
    for k, (R, p) in enumerate(frames):
        sz = SERVO[k]
        S.add(box(p, R, sz, DARK if J_TYPE[k] == "p" else GREY))
        ax = R[:, 1] if J_TYPE[k] == "p" else R[:, 2]
        S.add(frustum(p - ax * sz[1] * 0.62, p + ax * sz[1] * 0.62, sz[2] * 0.36, sz[2] * 0.36, col_accent, n=12))


def pod_mesh(S, stalk_end, u, roll_ref, prop_ang, fold, warn):
    S.add(frustum(stalk_end, stalk_end + u * 0.004, 0.02, 0.02, DARK, n=16))
    S.add(frustum(stalk_end + u * 0.004, stalk_end + u * 0.026, 0.0145, 0.0145, RED if warn else GREY, n=16))
    hub = stalk_end + u * BLADE_OFF
    S.add(frustum(stalk_end + u * 0.026, hub + u * 0.004, 0.006, 0.006, DARK, n=10))
    e1 = roll_ref - np.dot(roll_ref, u) * u; e1 /= np.linalg.norm(e1) + 1e-12
    e2 = np.cross(u, e1)
    for k in range(3):
        a = prop_ang + k * 2 * np.pi / 3
        rdir = np.cos(a) * e1 + np.sin(a) * e2
        bdir = np.cos(fold) * rdir + np.sin(fold) * u
        c = hub + bdir * CLAW_R / 2
        R = frame_z(np.cross(bdir, np.cross(u, rdir)) + 1e-9, bdir)
        S.add(box(c, np.stack([bdir, np.cross(R[:, 2], bdir), R[:, 2]], 1) @ rot([1, 0, 0], 0.35), [CLAW_R, 0.016, 0.0025], [0.25, 0.27, 0.30]))
        S.add(sphere(hub + bdir * CLAW_R, 0.004, DARK, n=8, m=6), same=True)


def foot_drop(fold):
    """Stalk end -> fingertip axial distance with the claw folded by `fold`."""
    return BLADE_OFF + CLAW_R * np.sin(fold) + 0.004
