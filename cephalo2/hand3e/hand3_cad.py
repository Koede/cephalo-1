"""hand3 rev D solids, hand3e version (workspace copy of claude/code/hand3e/hand3_cad.py, 2026-11-15; finger functions live in
hand3_finger.py).  Pod frame: z = 0 at the arm's spigot face, +z = thrust, rotor axis = z."""
import numpy as np
import manifold3d as mf
import hand3_design as H
from hand3_finger import (cyl, box, half, wedge, T, rot, app, about, loft, to_trimesh, root_grip, phalanx, pad)
import json

M_ = mf.Manifold
KN, RT, RL = H.KN, H.R_TIP, H.R_LAG
ZB = H.Z["blade"]; Zs = H.Z
FAZ = H.FINGER_AZ


def lag_tube():
    t = cyl(H.LAG_TUBE[0] / 2, -5.0, 7.5, c=(RL, 0, 0)) - cyl(H.LAG_TUBE[1] / 2, -6, 8, c=(RL, 0, 0))
    for z0, z1 in ((-0.8, 0.8), (5.3, 6.9)): t -= box([RL, -3, z0], [RL + 3.5, 3, z1])
    return t


def follower_az():
    return [c["az"] if c["finger"] is None else FAZ[c["finger"]] + H.FOLLOW_DAZ[c["role"]] for c in H.CHANNELS]


def stop_posts():
    out = []
    for k in range(4):
        s = H.LAG_SIGN[k]
        for lag, y in ((0.0, s * 7.55), (s * H.SPREAD_MAX, -s * 7.55)):
            p = rot("z", FAZ[k]) @ (np.array([RL, 0, 0]) + rot("z", lag) @ np.array([-4.0, y, 0]))
            out.append(cyl(1.5, ZB - 2.5 - 0.3, ZB + 2.5, c=(p[0], p[1], 0)))
    return out


def hub_holes(m):
    m -= cyl(2.6, ZB - 15, ZB + 15)
    for a in FAZ: m -= app(cyl(H.LAG_TUBE[0] / 2 + 0.02, ZB - 15, ZB + 15, c=(RL, 0, 0)), T(rot("z", a)))
    return m


def hub_lower():
    rp = H.HUB["plate_d"] / 2
    m = cyl(rp, Zs["hub_lo"], Zs["hub_lo"] + 2.5) + cyl(7.0, Zs["motor_top"], ZB + 2.5)
    for p in stop_posts(): m += p
    return hub_holes(m)


SPRING_SCREW = dict(r=13.0, tap=1.6)


def hub_upper():
    m = hub_holes(cyl(H.HUB["plate_d"] / 2, ZB + 2.5, ZB + 5.0))
    for a in FAZ: m -= app(cyl(SPRING_SCREW["tap"] / 2, ZB + 2.0, ZB + 5.5, c=(SPRING_SCREW["r"], 0, 0), n=20), T(rot("z", a)))
    return m


HEEL_V = dict(alpha=30.0, w=8.0, x_half=9.0, y_half=8.5, corner_r=3.0, lip_chamfer=0.4, apex_r=0.8)
HEEL_LID_CLR = 0.3


def heel_apex():
    v = HEEL_V; return Zs["heel"] - v["w"] * np.tan(np.radians(v["alpha"]))


def heel():
    bore = cyl(4.7, ZB + 4, Zs["shaft_tip"] + 0.8)
    if HEEL_V is None: return cyl(8.0, ZB + 5.0, Zs["heel"]) - bore
    v = HEEL_V; z0, z1 = ZB + 5.0, Zs["heel"]; xr, yr, cr = v["x_half"], v["y_half"], v["corner_r"]
    m = box([-xr + cr, -yr, z0], [xr - cr, yr, z1]) + box([-xr, -yr + cr, z0], [xr, yr - cr, z1])
    for sx in (-1, 1):
        for sy in (-1, 1): m += cyl(cr, z0, z1, c=(sx * (xr - cr), sy * (yr - cr), 0), n=24)
    a = np.radians(v["alpha"]); za = heel_apex(); L = 3 * xr
    cut = box([-L, -yr - 1, za], [L, yr + 1, z1 + 20])
    for sx in (-1, 1): cut = half(cut, [-sx * np.sin(a), 0.0, np.cos(a)], [0, 0, za])
    m -= cut
    if v["apex_r"] > 0:
        r = v["apex_r"]; zc = za + r / np.cos(a)
        m += box([-r * np.sin(a), -yr, za - 0.5], [r * np.sin(a), yr, zc - r * np.cos(a)]) - cyl(r, -yr - 1, yr + 1, "y", (0, 0, zc))
    if v["lip_chamfer"] > 0:
        c = v["lip_chamfer"]
        for sx in (-1, 1):
            m -= half(half(box([-L, -yr - 1, z1 - 2 * c], [L, yr + 1, z1 + 1]), [sx, 0, 1.0], [sx * (xr - c), 0, z1]), [sx, 0, 0], [0, 0, 0])
    z_lid = ZB + 15.0 + HEEL_LID_CLR
    m = (m ^ (cyl(8.0, z0 - 1, z_lid, n=96) + box([-20, -20, z_lid], [20, 20, z1 + 1]))) if z_lid < z1 else m
    return m - bore


_P = None
_B = None


def base_parts():
    global _B
    if _B is None:
        _B = dict(root_grip=root_grip(), lag_tube=lag_tube(), hub_lower=hub_lower(), hub_upper=hub_upper(), heel=heel())
        for j in range(3): _B[f"phalanx{j + 1}"] = phalanx(j); _B[f"pad{j + 1}"] = pad(j)
    return _B


try:
    import hand3_springs as SPRINGS
except ImportError:
    SPRINGS = None


def parts():
    global _P
    if _P is None:
        b = dict(base_parts())
        _P = SPRINGS.patch_parts(b) if (SPRINGS is not None and hasattr(SPRINGS, "patch_parts")) else b
    return _P


def finger_frames(k, lag=0.0, fold=(0, 0, 0)):
    G = T(t=[0, 0, ZB]) @ T(rot("z", FAZ[k])) @ about("z", lag, [RL, 0, 0])
    C = G @ T(rot("x", H.PITCH)); out = [G]
    for j in range(3):
        C = C @ about("y", -fold[j], [KN[j], 0, 0]); out.append(C)
    return out
