"""LOCAL WORKING COPY of claude/code/snapdigit.py (Snap-Digit claw-prop, concept C11: geometry, parts, kinematics).
Typed back in from the project file for the XL render (the STL export and the __main__ block are left out here; the
project file is the complete one and stays the single source of truth).

Frames: pod frame, z = pod axis (+z = thrust = the side the fingers fold to), hub mid-plane z = 0.  Blade frame:
X = span, Y = tangential (leading edge side = +Y), Z = axial.  All CAD in mm.
"""
import numpy as np
import manifold3d as mf
import trimesh
import clawblade as B

# ------------------------------------------------------------------ parameters (mm)
R_TIP = 76.2
R_PIN, Z_PIN = 13.0, 1.5
R_K0, R_K1 = 44.0, 61.0            # knuckle fold zone (free tape)
LF = R_K1 - R_K0                   # 17 mm free tape
TAPE_R = B.TAPE_R * 1e3            # 12 mm fold radius ~ transverse radius
TAPE_T = B.TAPE_T * 1e3
TAPE_W, TAPE_WAIST = 8.0, B.TAPE_WAIST * 1e3     # 1/2 in tape trimmed to 8 mm (bond zones) and 6 mm (fold waist)
TAPE_BOND = (32.0, 69.0)
PRECONE = 2.0                      # deg, both seats
FOLD1_MAX, FOLD2_MAX = 85.0, 80.0
HUB_R, HUB_Z = 18.5, (-3.0, 3.0)
BELL_R, BELL_Z = 13.95, (-17.0, -3.2)
STATOR_Z = (-24.0, -17.0)
POST_R, POST_Z = 7.9, 8.5
HOLE = (15.6, 6.0)
SKIRT = dict(ri=14.5, ro=17.5, h=6.0, travel=20.0)
BRG = dict(name="6707ZZ-class thin bearing 35x44x5 (est. 11 g)", ri=17.5, ro=22.0, w=5.0)
ROD_R, ROD_D = 24.5, 2.0
SERVO = dict(name="Feetech STS3032 (bus servo, 0.441 N m stall, 20 g)", L=23.2, W=12.1, H=28.5, horn=8.0)
STALL = 0.441


def pitch(r):
    return np.radians(B.pitch_deg(r * 1e-3))


def chord(r):
    c = B.chord(r * 1e-3) * 1e3
    tip = np.clip((R_TIP - r) / 4.0, 0, 1)            # rounded tip
    return c * (0.45 + 0.55 * np.sqrt(tip))


def thick(r):
    return B.thick(r * 1e-3) * 1e3


def section(r, n=28):
    """Cambered airfoil polygon in (Y, Z) at radius r, pitched (leading edge +Y raised toward +Z)."""
    c, t, ph = chord(r), max(thick(r), 1.2), pitch(r)
    u = 0.5 * c * np.cos(np.linspace(0, np.pi, n))                  # +c/2 (LE) .. -c/2 (TE)
    s = np.clip(1 - (2 * u / c) ** 2, 0, 1)
    cam = 0.04 * c * s
    h = 0.5 * t * s ** 0.6 + 0.12
    up = np.stack([u, cam + h], 1)
    lo = np.stack([u[::-1], (cam - h)[::-1]], 1)
    P = np.concatenate([up, lo[1:-1]])
    Y = P[:, 0] * np.cos(ph) - P[:, 1] * np.sin(ph)
    Z = P[:, 0] * np.sin(ph) + P[:, 1] * np.cos(ph)
    return np.stack([Y, Z], 1)


def loft(rs, n=28):
    secs = [section(r, n) for r in rs]
    secs = [p if _area(p) > 0 else p[::-1] for p in secs]          # CCW seen from +X
    m = len(secs[0])
    V = np.concatenate([np.column_stack([np.full(m, r), s]) for r, s in zip(rs, secs)])
    F = []
    for k in range(len(rs) - 1):
        a, b = k * m, (k + 1) * m
        for i in range(m):
            j = (i + 1) % m
            F += [[a + i, b + j, b + i], [a + i, a + j, b + j]]
    c0 = len(V); c1 = c0 + 1
    V = np.vstack([V, [rs[0], *secs[0].mean(0)], [rs[-1], *secs[-1].mean(0)]])
    L = (len(rs) - 1) * m
    for i in range(m):
        j = (i + 1) % m
        F += [[c0, j, i], [c1, L + i, L + j]]
    return to_mf(V, np.array(F))


def _area(p):
    return 0.5 * np.sum(p[:, 0] * np.roll(p[:, 1], -1) - np.roll(p[:, 0], -1) * p[:, 1])


def to_mf(V, F):
    tm = trimesh.Trimesh(V, F, process=True)
    if tm.volume < 0:
        tm.invert()
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(tm.vertices, np.float32), tri_verts=np.asarray(tm.faces, np.uint32)))


def cube(c, size):
    s = np.asarray(size, float)
    return mf.Manifold.cube(s.tolist(), center=True).translate(list(map(float, c)))


def cyl(p0, p1, r, n=24):
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    d = p1 - p0; L = np.linalg.norm(d)
    c = mf.Manifold.cylinder(L, r, r, n)
    Rm = frame(d)
    return c.transform(np.column_stack([Rm, p0]).tolist())


def frame(d):
    z = d / np.linalg.norm(d)
    ref = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = ref - z * (ref @ z); x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], 1)


def rotm(axis, ang):
    a = np.asarray(axis, float); a /= np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def tf(R=np.eye(3), t=(0, 0, 0)):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T


def apply(m, T):
    return m.transform(np.asarray(T)[:3].tolist())


def top_z(r):
    """Front-face (upper surface) height on the chord line at the section centre."""
    return 0.5 * max(thick(r), 1.2) + 0.12 + 0.04 * chord(r)


# ------------------------------------------------------------------ knuckle kinematics (exact tape fold)
PHK = pitch(R_K0)                                         # chord-plane pitch at the knuckle
T_K = np.array([1.0, 0, 0])
N_K = np.array([0, -np.sin(PHK), np.cos(PHK)])            # chord-plane normal toward the front
P_K0 = np.array([R_K0, 0, 0]) + N_K * top_z(R_K0)          # tape leaves the proximal here


def knuckle_T(theta_deg, s0=None):
    """Distal-phalanx transform (blade frame) for a fold of theta: a circular tape arc of radius TAPE_R that starts
    s0 into the free zone (default: centred), straight tape either side."""
    th = np.radians(theta_deg)
    arc = TAPE_R * abs(th)
    if s0 is None:
        s0 = max(0.0, 0.5 * (LF - arc))
    P0 = P_K0 + s0 * T_K
    C = P0 + TAPE_R * N_K
    Rm = rotm(np.cross(T_K, N_K), th)
    t2 = Rm @ T_K
    M = tf(Rm, C - Rm @ C)
    M[:3, 3] += -arc * t2
    return M


def tape_points(theta_deg, n=40):
    """Tape centre-line (blade frame) from r 32 to 70 for a fold of theta."""
    th = np.radians(theta_deg)
    arc = TAPE_R * abs(th)
    s0 = max(0.0, 0.5 * (LF - arc))
    pts = []
    for s in np.linspace(TAPE_BOND[0] - R_K0, TAPE_BOND[1] - R_K0, n):
        if s <= s0:
            p = P_K0 + s * T_K
        elif s <= s0 + arc:
            a = (s - s0) / TAPE_R
            p = P_K0 + s0 * T_K + TAPE_R * np.sin(a) * T_K + TAPE_R * (1 - np.cos(a)) * N_K
        else:
            M = knuckle_T(theta_deg)
            p = (M @ np.r_[P_K0 + s * T_K, 1])[:3]
        if s < 0:
            p = np.array([R_K0 + s, 0, 0]) + N_K * top_z(R_K0 + s) * 0 + np.array([0, -np.sin(pitch(R_K0 + s)), np.cos(pitch(R_K0 + s))]) * top_z(R_K0 + s)
        pts.append(p)
    return np.array(pts)


def root_T(a_deg):
    """Blade transform in the pod frame (blade 0 along +x) for a root fold a (deg, + = forward)."""
    R = rotm([0, -1, 0], np.radians(a_deg))          # rotating +X toward +Z about -Y
    piv = np.array([R_PIN, 0, Z_PIN])
    return tf(R, piv - R @ piv)


# ------------------------------------------------------------------ printed parts (blade frame, mm)
ANCHOR_R = R_K1 + 3.5
FL_H, AN_H = 3.0, 2.8
LUG_X0, LUG_W, SLOT_W = 9.5, 8.0, 8.5


def nrm_at(r):
    ph = pitch(r)
    return np.array([0, -np.sin(ph), np.cos(ph)])


def fence_pt(r, h):
    ph = pitch(r)
    cdir = np.array([0, np.cos(ph), np.sin(ph)])
    return np.array([r, 0, 0]) + cdir * fence_u(r) + nrm_at(r) * (top_z(r) - 0.3 + h)


def fence_u(r):
    return -0.5 * chord(r) + 1.3


def bridge(r, h, L=2.8):
    ph = pitch(r)
    R = rotm([1, 0, 0], ph)
    cdir = np.array([0, np.cos(ph), np.sin(ph)])
    ctr = np.array([r, 0, 0]) + cdir * fence_u(r) + nrm_at(r) * (top_z(r) + h / 2 - 0.9)
    return apply(cube([0, 0, 0], [L, 2.4, h + 1.8]), tf(R, ctr))


def proximal():
    rs = np.r_[np.linspace(19.0, 30, 6), np.linspace(31, R_K0 + 0.6, 7)]
    body = loft(rs)
    lug = cube([0.5 * (LUG_X0 + 20.5), 0, 1.0], [20.5 - LUG_X0, LUG_W, 4.0])
    blend = cube([20.5, 0, 0.6], [3.0, LUG_W, 3.4])
    tongue, key = tongue_parts()
    tongue = apply(tongue + key, knuckle_T(PRECONE))
    p = body + lug + blend + tongue
    p = p - cyl([R_PIN, -5, Z_PIN], [R_PIN, 5, Z_PIN], 1.52)
    ph = pitch(38.0)
    rec = apply(cube([0, 0, 0], [R_K0 + 1 - TAPE_BOND[0] + 0.5, TAPE_W + 0.4, 0.5]),
                tf(rotm([1, 0, 0], ph), np.array([0.5 * (TAPE_BOND[0] + R_K0 + 1), 0, 0]) + np.array([0, -np.sin(ph), np.cos(ph)]) * (top_z(38) - 0.2)))
    p = p - rec
    p = p - rec + bridge(R_K0 - 4.0, FL_H + 0.8)
    p = p - apply(cyl([-2.5, 0, 0], [2.5, 0, 0], 0.45), tf(t=fence_pt(R_K0 - 4, FL_H)))
    return p


def distal():
    rs = np.r_[np.linspace(R_K1 + 0.3, R_TIP - 4, 8), np.linspace(R_TIP - 3, R_TIP - 0.05, 5)]
    body = loft(rs)
    d = body
    ph = pitch(66.0)
    rec = apply(cube([0, 0, 0], [TAPE_BOND[1] - R_K1 + 1.0, TAPE_W + 0.4, 0.5]),
                tf(rotm([1, 0, 0], ph), np.array([0.5 * (TAPE_BOND[1] + R_K1 - 1.0), 0, 0]) + np.array([0, -np.sin(ph), np.cos(ph)]) * (top_z(66) - 0.2)))
    d = d - rec
    d = d - rec + bridge(ANCHOR_R, AN_H + 0.8)
    d = d - apply(cyl([-2.5, 0, 0], [2.5, 0, 0], 0.45), tf(t=fence_pt(ANCHOR_R, AN_H)))
    return d


def distal_seated():
    d = apply(distal(), knuckle_T(PRECONE))
    tg = tongue_envelope()
    return d - tg


def tongue_parts(grow=0.0, back=0.0):
    phm = pitch(0.5 * (R_K0 + R_K1))
    Rt = rotm([1, 0, 0], phm)
    nrm = np.array([0, -np.sin(phm), np.cos(phm)])
    zc = top_z(R_K0) - 0.12 - 1.0 - 0.15
    t0, t1 = R_K0 - 1.0, R_K1 - 0.4
    tongue = apply(cube([0, 0, 0], [t1 - t0, 7.5, 2.0]), tf(Rt, np.array([0.5 * (t0 + t1), 0, 0]) + nrm * zc))
    k0, k1 = t1 - 0.5, R_K1 + 2.2
    kz = top_z(R_K1) - 0.12 - 0.55 - 0.1
    kh = 0.9 + back
    key = apply(cube([0, 0, 0], [k1 - k0 + 2 * grow, 3.0 + 2 * grow, kh + 2 * grow]),
                tf(Rt, np.array([0.5 * (k0 + k1), 0, 0]) + nrm * (kz - 0.45 - back / 2)))
    return tongue, key


def tongue_envelope():
    _, key = tongue_parts(grow=0.15, back=4.0)
    return apply(key, knuckle_T(PRECONE))


def hub():
    h = cyl([0, 0, HUB_Z[0]], [0, 0, HUB_Z[1]], HUB_R, 48)
    h = h - cyl([0, 0, -10], [0, 0, 10], 2.6)
    for k in range(3):
        Rz = rotm([0, 0, 1], np.radians(120 * k))
        slot = apply(cube([R_PIN + 2.5, 0, 0], [15.0, SLOT_W, 20]), tf(Rz))
        pin = apply(cyl([R_PIN, -9, Z_PIN], [R_PIN, 9, Z_PIN], 1.0), tf(Rz))
        hole = apply(cyl([HOLE[0], HOLE[1], -6], [HOLE[0], HOLE[1], 6], 0.8), tf(Rz))
        key = apply(cyl([HOLE[0], -HOLE[1], -12], [HOLE[0], -HOLE[1], 6], 0.8), tf(Rz))
        h = h - slot - pin - hole - key
    return h


def stop_z():
    T = root_T(PRECONE)
    pts = np.array([[x, y, 3.0] for x in (LUG_X0, 9.8) for y in (-LUG_W / 2, LUG_W / 2)])
    return float(max((T[:3, :3] @ p + T[:3, 3])[2] for p in pts))


def palm_cap():
    zs = stop_z()
    c = cyl([0, 0, HUB_Z[1]], [0, 0, HUB_Z[1] + 2.0], 11.5, 40) - cyl([0, 0, 0], [0, 0, 20], 4.6)
    for k in range(3):
        Rz = rotm([0, 0, 1], np.radians(120 * k))
        c = c - apply(cube([R_PIN + 2.5, 0, HUB_Z[1] + 1], [15.0, SLOT_W, 3.0]), tf(Rz))
    for k in range(3):
        Rz = rotm([0, 0, 1], np.radians(120 * k))
        x0, x1 = 5.6, 9.8
        tower = apply(cube([0.5 * (x0 + x1), 0, 0.5 * (zs + 10.4)], [x1 - x0, 6.4, 10.4 - zs]), tf(Rz))
        tower = tower - apply(cyl([POST_R, -5, POST_Z], [POST_R, 5, POST_Z], 0.85), tf(Rz))
        turn = apply(cube([HOLE[0] - 1.0, HOLE[1], HUB_Z[1] + 1.0], [3.4, 3.0, 2.0]), tf(Rz))
        turn = turn - apply(cyl([HOLE[0] - 1.6, HOLE[1] - 3, HUB_Z[1] + 1.4], [HOLE[0] - 1.6, HOLE[1] + 3, HUB_Z[1] + 1.4], 0.55), tf(Rz))
        c = c + tower + turn
    return c


def skirt():
    s = cyl([0, 0, -SKIRT["h"]], [0, 0, 0], SKIRT["ro"], 56) - cyl([0, 0, -10], [0, 0, 1], SKIRT["ri"], 56)
    for k in range(3):
        Rz = rotm([0, 0, 1], np.radians(120 * k))
        lug = apply(cube([HOLE[0] + 0.4, HOLE[1], 0.6], [3.2, 3.0, 1.2]), tf(Rz))
        lug = lug - apply(cyl([HOLE[0], HOLE[1], -2], [HOLE[0], HOLE[1], 3], 0.5), tf(Rz))
        s = s + lug
        s = s - apply(cyl([HOLE[0], -HOLE[1], -10], [HOLE[0], -HOLE[1], 2], 0.85), tf(Rz))
    return s


def ring():
    r = cyl([0, 0, -BRG["w"] - 1.0], [0, 0, 0], BRG["ro"] + 1.6, 56) - cyl([0, 0, -10], [0, 0, 1], BRG["ro"] - 0.6, 56)
    for k in range(2):
        a = np.radians(90 + 180 * k)
        e = np.array([np.cos(a), np.sin(a), 0])
        lug = cube(e * (ROD_R - 0.5) + [0, 0, -BRG["w"] / 2 - 0.5], [5.0, 5.0, BRG["w"] + 1.0])
        r = r + lug - cyl(e * ROD_R + [0, 0, -9], e * ROD_R + [0, 0, 2], ROD_D / 2 + 0.05)
    return r


SKIRT_TOP_OPEN = -7.0
PARTS = None


def build_parts():
    global PARTS
    PARTS = dict(hub=hub(), palm_cap=palm_cap(), skirt=skirt(), ring=ring(), proximal=proximal(),
                 distal=apply(distal_seated(), np.linalg.inv(knuckle_T(PRECONE))))
    return PARTS
