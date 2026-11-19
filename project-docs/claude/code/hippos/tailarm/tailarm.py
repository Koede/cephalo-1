"""Hippos-1 tail = one Cephalo-2 arm (single source of truth for the tail, 2026-11-16; replaces tail.py's 4-joint tail).

Forest (2026-11-16): "The tail should be the same exact makeup as the arms of the octopus", picked option C: a whole arm,
8 joints + hand3, and the AUX pin as a second 12 V feed.

What it is (all from the arm's own definition, cephalo4.py rev 4 / Cephalo-2):
  * 8 servo-in-link joints p-y-p-y-p-y-p-y, 56 mm apart, first joint 50 mm from the root face, spigot 478 mm
  * STS3250 at j0, STS3215 at j1..j7, detent clutches on j0 (1.2 x stall) and j2..j7 (0.9 x stall), j1 rigid, Dyneema
    winch line through the hollow pins with an N20 worm winch at the root (as on the mantle sector)
  * tip bay with the tip camera, SFF connector (arm socket + hand base) on the spigot, hand3 (hub C2) on it
  * the arm link cradles are the arm's (their printable CAD is the open item shared with Cephalo-2: fall-load rules)
Tail-specific parts (CAD here):
  * root_cradle_arm  replaces the STS3095 root cradle: disc on the tail_adapter servo flange (4 x M3 at r 25), walls that
                     grip the STS3250 case on its proximal half (2 x M2.5 clamp slots per side), N20 winch bay between the
                     disc and the case, wire / line hole
Frames: connector frame of tailmount.py (mm; z_c = 0 the SFF mouth, +z_c into the rump, x_c up, y_c = -y_horse).  The arm's
root face (s = 0) is the adapter flange underside z_c -35; the arm runs along -z_c, arm z (pitch-up side) = +x_c, arm y = +y_c,
so j0 pitches about y_c.  Positive pitch = tip DOWN (cephalo4).  j0 axis at z_c -85.
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [HERE, os.path.dirname(HERE), "/home/claude/octodrone/ceph2"]
import numpy as np
import manifold3d as mf
import cephalo4 as C
import tail as TL0                                  # box / ycyl / cradle constants of the old tail (CL, WALL, LIP_R)
from sff_core import cyl

MM = 1e-3
Z_FLANGE = -35.0; DISC_T = 4.0; SEAT = 3.8
N = C.NJ
S_J = C.S_J / MM                                    # 50 .. 442
SPIGOT = C.SPIGOT / MM                              # 478
ZJ0 = Z_FLANGE - S_J[0]                             # -85
STS3250 = dict(L=45.2, W=24.7, H=35.0, a=11.4)      # case, shaft 11.4 mm from the output end (cephalo4)
N20 = dict(x=12.0, y=34.0, z=10.0)                  # N20 worm gearmotor + spool block envelope (est.; measure)
M_TAIL_EXTRA = dict(winch=0.024, root_cradle=None)  # root cradle mass from the solid
POD_H3 = (0.426, (9.9 + 59.0) * MM)                 # hand3 + connector: mass, COM past the spigot (cephalo2 / build kit)
HEEL = (9.9 + 149.2) * MM                           # spigot -> hand3 heel (the foot the hand stands on)
FLANGE_HOLES = TL0.FLANGE_HOLES


def box(x0, x1, y0, y1, z0, z1):
    return TL0.box(x0, x1, y0, y1, z0, z1)


# ---------------------------------------------------------------- CAD (connector frame, mm)
def servo_j0():
    L, W, Hh, a = STS3250["L"], STS3250["W"], STS3250["H"], STS3250["a"]
    return box(-W / 2, W / 2, -Hh / 2, Hh / 2, ZJ0 - a, ZJ0 + L - a)          # long side along z_c, shaft along y_c


def j0_side_parts():
    """Arm hardware beside the j0 servo (viz4 / clutch set v1 envelopes): clutch cam r 25 on the horn side, idler boss r 11."""
    hw = STS3250["H"] / 2
    return TL0.ycyl(25.0, hw + 1.0, hw + 7.0, z=ZJ0) + TL0.ycyl(11.0, -hw - 6.0, -hw - 1.0, z=ZJ0)


def winch_env():
    zc0 = Z_FLANGE - DISC_T - 0.6
    return box(-N20["x"] / 2, N20["x"] / 2, -N20["y"] / 2, N20["y"] / 2, zc0 - N20["z"], zc0)


def root_cradle_arm():
    L, W, Hh, a = STS3250["L"], STS3250["W"], STS3250["H"], STS3250["a"]
    CL, WALL, LIP = TL0.CL, TL0.WALL, TL0.LIP_R
    xi, yi = W / 2 + CL, Hh / 2 + CL
    z0, z1 = ZJ0 + LIP, Z_FLANGE - DISC_T + 0.01                               # walls from 17 mm above the axis to the disc
    m = cyl(28.0, Z_FLANGE - DISC_T, Z_FLANGE, 96)
    zl = ZJ0 + 25.0 + 3.0                                                       # lips start 3 mm above the j0 clutch cam (r 25)
    yw = 16.0                                                                   # side walls stop 2.5 mm short of the clutch cam face below zl
    m += box(-xi - WALL, -xi, -yw, yw, z0, z1) + box(xi, xi + WALL, -yw, yw, z0, z1)
    m += box(-xi - WALL, -xi, -yi, yi, zl, z1) + box(xi, xi + WALL, -yi, yi, zl, z1)
    m += box(-xi - WALL, xi + WALL, -yi - 2.2, -yi, zl, z1) + box(-xi - WALL, xi + WALL, yi, yi + 2.2, zl, z1)
    # case stop: a 2 x 1.5 mm ledge on each long inner wall above the case's proximal end (the case seats against it)
    zt = ZJ0 + L - a + CL
    m += box(xi - 2.0, xi, -yi, yi, zt, zt + 1.5) + box(-xi, -xi + 2.0, -yi, yi, zt, zt + 1.5)
    # winch bay above the case: two rails hold the N20 block against the disc (2 x M2 strap each, est.)
    w = winch_env(); wb = np.asarray(w.bounding_box()).reshape(-1)
    hx = N20["x"] / 2
    m += box(hx + 0.3, hx + 2.3, -yi, yi, wb[2] - 1.5, z1) + box(-hx - 2.3, -hx - 0.3, -yi, yi, wb[2] - 1.5, z1)
    m -= box(-hx - 0.3, hx + 0.3, -N20["y"] / 2 - 0.3, N20["y"] / 2 + 0.3, wb[2] - 0.3, wb[5] + 0.3)
    # clamp screws (2 x M2.5 per side wall into the case side holes, positions est.) and flange holes
    for zz in (ZJ0 + LIP + 5, zt - 5):
        m -= box(-xi - WALL - 1, xi + WALL + 1, -1.35, 1.35, zz - 1.35, zz + 1.35)
    for x, y in FLANGE_HOLES:
        m -= cyl(1.7, Z_FLANGE - DISC_T - 1, Z_FLANGE + 1, 24).translate([x, y, 0])
    # wire + winch line: Ø8 through the disc beside the winch, down the cradle's open side
    m -= cyl(4.0, Z_FLANGE - DISC_T - 1, Z_FLANGE + 1, 32).translate([0, yi - 5.0, 0])
    return m


# ---------------------------------------------------------------- kinematics (horse frame, metres)
def frames(ORG, M):
    """Arm root frame in the horse frame from the connector placement (ORG m, M = connector -> horse rotation)."""
    R0 = np.column_stack([M @ [0, 0, -1.0], M @ [0, 1.0, 0], M @ [1.0, 0, 0]])
    p0 = ORG + M @ np.array([0, 0, Z_FLANGE]) * MM
    return R0, p0


def fk(q, R0, p0):
    """cephalo4.arm_fk with the arm rooted at (R0, p0).  q in rad."""
    save = C.arm_base
    C.arm_base = lambda i, body_p=None, body_R=None: (R0.copy(), p0.copy())
    try:
        return C.arm_fk(0, q, p0, R0)
    finally:
        C.arm_base = save


def torques(q, R0, p0, F_ext=None, P_ext=None, g=1.0):
    """Static joint moments (N m) from gravity (g x 9.81 down) on the arm + hand3, plus an external force at a point."""
    segs, tip = fk(q, R0, p0)
    P, m, s = C.arm_points(segs, tip, POD_H3)
    F = m[:, None] * np.array([0, 0, -9.81 * g])[None, :]
    if F_ext is not None:
        P = np.vstack([P, P_ext]); F = np.vstack([F, F_ext]); s = np.append(s, 9.0)
    tau = np.zeros(N)
    for k in range(N):
        c, R = C.at_s(segs, C.S_J[k])
        ax = R[:, 1] if C.TYP[k] == "p" else R[:, 2]
        sel = s > C.S_J[k]
        tau[k] = ax @ np.cross(P[sel] - c, F[sel]).sum(0)
    return tau, (P, F)


def heel(q, R0, p0):
    _, (Rt, pt) = fk(q, R0, p0)
    return pt + Rt[:, 0] * HEEL, Rt[:, 0]


def capsules(q, R0, p0, n=40):
    """Arm envelope points + radii (drawn envelope r 31.5 at the servos / clutch cams, as fall4 LINK_R) and the hand3."""
    segs, (Rt, pt) = fk(q, R0, p0)
    ss = np.linspace(C.S_J[0] - 0.02, C.SPIGOT, n)
    P = [C.at_s(segs, s)[0] for s in ss]; r = list(np.full(n, 0.0315))
    for d, rr in ((0.03, 0.030), (0.06, 0.045), (0.09, 0.062), (0.12, 0.062), (0.15, 0.030)):      # hand3 (frame, rings, curled fingers)
        P.append(pt + Rt[:, 0] * (0.0099 + d)); r.append(rr)
    return np.array(P), np.array(r)


def meshes(q, ORG, M):
    """Tail meshes in the horse frame (metres): [(family, rgb, trimesh)] = root cradle, the arm (viz4 arm body), the SFF
    socket + hand base on the spigot, hand3 (grasp-ready).  ORG, M: connector placement."""
    import trimesh, viz4_arm as VA, hand3_mesh as H3M
    from geom import Scene
    import sff_core as SFF
    R0, p0 = frames(ORG, M)
    T = np.eye(4); T[:3, :3] = M; T[:3, 3] = ORG
    def tm(man):
        me = man.to_mesh(); m = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3] * MM, np.asarray(me.tri_verts), process=False)
        m.apply_transform(T); return m
    out = [("pa_cf", (0.17, 0.18, 0.20), tm(root_cradle_arm())), ("metal_dark", (0.20, 0.21, 0.23), tm(winch_env()))]
    save = C.arm_base
    C.arm_base = lambda i, body_p=None, body_R=None: (R0.copy(), p0.copy())
    try:
        S = Scene(); segs, (Rt, pt) = VA.arm_body(S, 0, p0, R0, q)
    finally:
        C.arm_base = save
    by = {}
    for V, F, Cc, _ in S.prims:
        cols = np.round(np.asarray(Cc, float), 2)
        for c in np.unique(cols, axis=0):
            msk = np.all(cols == c, axis=1); Ff = F[msk]; used = np.unique(Ff)
            remap = -np.ones(len(V), int); remap[used] = np.arange(len(used))
            by.setdefault(tuple(c), []).append(trimesh.Trimesh(V[used], remap[Ff], process=False))
    for c, lst in by.items(): out.append(("arm", c, trimesh.util.concatenate(lst)))
    u = Rt[:, 0]
    Rc = np.stack([Rt[:, 1], np.cross(-u, Rt[:, 1]), -u], 1); T4 = np.eye(4); T4[:3, :3] = Rc; T4[:3, 3] = pt
    for fam, col, man in (("petg_orange", (0.95, 0.55, 0.15), SFF.arm_socket()), ("petg_dark", (0.30, 0.31, 0.34), SFF.hand_base())):
        me = man.to_mesh(); m = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3] * MM, np.asarray(me.tri_verts), process=False)
        m.apply_transform(T4); out.append((fam, col, m))
    Th = np.eye(4); Th[:3, :3] = np.stack([Rt[:, 1], Rt[:, 2], u], 1); Th[:3, 3] = pt + u * (-SFF.P["flange_bot"] + SFF.P["gap"]) * MM
    Th = Th @ np.diag([MM, MM, MM, 1.0])
    for m, c, k in H3M.hand(lag=0.0, fold=(28, 40, 35)):
        m = m.copy(); m.apply_transform(Th); out.append(("hand3:" + k, c, m))
    return out
