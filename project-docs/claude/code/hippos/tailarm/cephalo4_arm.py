"""Cephalo rev 4 arm geometry (geometry subset of the project's claude/code/cephalo4.py, verbatim kinematics).
Body origin = centre of the plate underside, +z up, metres.  Arm i root at azimuth 22.5 + 45 i deg."""
import os
import numpy as np

MM = 1e-3
ROOT_R, ROOT_Z = 104 * MM, 39 * MM
S_J = (50 + 56 * np.arange(8)) * MM
TYP = "pypypypy"
RG = 0.0
SPIGOT = 478 * MM
NJ = 8
LIM = np.radians([[-59, 55], [-30, 30]] + [[-100, 100]] * 6)
CLAW_R = 0.0762
BLADE_OFF = 0.036
SV = {"STS3215": (0.981, 2.94, 0.055, 45.2, 24.7, 35.0),
      "STS3250": (1.63, 4.90, 0.0745, 45.2, 24.7, 35.0)}
SERVO = ["STS3250"] + ["STS3215"] * 7
if os.environ.get("CEPH_J1") == "STS3250":
    SERVO[1] = "STS3250"
CLUTCH_FRAC = np.array([1.2, 0.0, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9])


def arm_base(i, body_p=np.zeros(3), body_R=np.eye(3)):
    a = np.radians(22.5 + 45 * i)
    x = np.array([np.cos(a), np.sin(a), 0]); y = np.array([-np.sin(a), np.cos(a), 0]); z = np.array([0, 0, 1.0])
    return body_R @ np.stack([x, y, z], 1), body_p + body_R @ (ROOT_R * x + ROOT_Z * z)


def _rot(ax, ang):
    c, s = np.cos(ang), np.sin(ang)
    if ax == "p":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def arm_fk(i, q, body_p=np.zeros(3), body_R=np.eye(3), clutch=None):
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
    out = []
    for k in range(NJ):
        c, R = at_s(segs, S_J[k])
        out.append((c, R[:, 1] if TYP[k] == "p" else R[:, 2], R))
    return out


# ------------------------------------------------------------------ masses and static loads (verbatim from the project's cephalo4.py)
G = 9.81
M_HINGE_J = np.full(8, 0.012)
M_CLUTCH = {"STS3215": 0.0238, "STS3250": 0.038}
LAM = 0.070
M_TIPBAY = 0.020 + 0.004
M_POD = 0.115
M_LINE = 0.013
RATED = np.array([SV[n][0] for n in SERVO]); STALL = np.array([SV[n][1] for n in SERVO])


def arm_points(segs, tip, pod=None):
    """pod = (mass kg, distance past the spigot m); default the rev 4 pod (0.115 kg at 24 mm)."""
    m_pod, d_pod = (M_POD, 0.024) if pod is None else pod
    P, m, s = [], [], []
    for k, n in enumerate(SERVO):
        sk = S_J[k] - (SV[n][3] / 2 - 11.4) * MM
        P.append(at_s(segs, sk)[0]); m.append(SV[n][2]); s.append(sk)
        P.append(at_s(segs, S_J[k] - RG - 0.002)[0]); m.append(M_HINGE_J[k] / 2 + (M_CLUTCH[n] if CLUTCH_FRAC[k] > 0 else 0)); s.append(S_J[k] - RG - 0.002)
        P.append(at_s(segs, S_J[k] + RG + 0.002)[0]); m.append(M_HINGE_J[k] / 2); s.append(S_J[k] + RG + 0.002)
    for x in np.linspace(0.0, SPIGOT, 20)[:-1] + SPIGOT / 38:
        P.append(at_s(segs, x)[0]); m.append((LAM * SPIGOT + M_LINE) / 19); s.append(x)
    R, p = tip
    P.append(p - R[:, 0] * 0.012); m.append(M_TIPBAY); s.append(SPIGOT - 0.012)
    P.append(p + R[:, 0] * d_pod); m.append(m_pod); s.append(SPIGOT + d_pod)
    return np.array(P), np.array(m), np.array(s)


def joint_torques(i, q, body_p, body_R, pod=None, acc=np.array([0, 0, -G])):
    segs, tip = arm_fk(i, q, body_p, body_R)
    P, m, s = arm_points(segs, tip, pod)
    F = m[:, None] * acc[None, :]
    tau = np.zeros(NJ)
    for k in range(NJ):
        c, R = at_s(segs, S_J[k])
        a = R[:, 1] if TYP[k] == "p" else R[:, 2]
        sel = s > S_J[k]
        tau[k] = a @ np.cross(P[sel] - c, F[sel]).sum(0)
    return tau
