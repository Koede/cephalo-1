"""Scabbard rotor + Scabbard dock for Cephalo-1 (single source of truth; draft CAD v1, 2026-11-12).

Forest's ask: the belly rotor's blades stow vertically up in the dome in four slits, slide straight down (parallel to the
rotor axis, tips down), then swing out under control to be the propeller or a leg stand.  Four empty slits sit 45 deg
between them.  Two drones dock belly to belly, turned 45 deg, and push their blades into each other's empty slits in
sync; a mutual lock holds them, with no power, until both unlock.  Four blades.  Docked flight is the goal.

How it is built (everything in mm, body frame: origin = centre of the plate underside, +z up):
  * hub      bolted to the bell of the rotor motor (T-Motor MN605-S, recessed in a well above the plate, bell down).
             Four lobes; each holds a TRUNNION: a barrel on a tangential hinge axis (r 70, z -16) with a rectangular
             passage through it.
  * blade    slides through its trunnion.  Stowed, it stands in its own scabbard in the dome (tip 2 mm inside the
             barrel).  Pushed fully down, its head seats in the barrel (shoulders; centrifugal force seats it harder).
  * elevator a ring in the dome with four fingers (T-bars) hooked into the blade heads, moved by two GT2 belt loops
             from one STS3215.  It slides the blades, and it cinches the dock.  The T-bars leave the heads by a
             6.5 deg turn of the rotor (bayonet), done by the rotor motor itself.
  * fold     four push rods come down through the plate onto a heel on each trunnion.  A spring holds each blade up
             (flight position); the rods push it down to any droop: 0 flight, 60 legs, 90 hanging/stow/dock.  The rods
             hang from a ring on three T8x2 lead screws (self-locking) turned by one STS3215 through a GT2 belt.
             In flight the rods are retracted and nothing touches the rotor.
  * dock     four GUEST sockets (45 deg between the blades' own scabbards).  Each has a spring latch pin (dia 4 dowel)
             that snaps into a brass eyelet in the partner's blade, 62 mm from its tip.  One STS3032 turns a release
             ring that pulls all four pins.  The latch is in the socket, not in the blade tip (which runs at 137 m/s).
Estimates are marked (est.).  Thrust and power are scaled from the Trivet rotor's basis (T-Motor P22x6.6 table).
"""
import numpy as np
import manifold3d as m3
import cephalo4 as C

G = 9.81; DEG = np.pi / 180; RHO = 1.2
# ------------------------------------------------------------------ geometry (mm)
R_S = 70.0            # hinge radius = scabbard / socket centre radius
Z_H = -16.0           # hinge axis below the plate underside
BAR_R = 14.0          # trunnion barrel radius
BAR_HL = 26.0         # barrel half length (tangential)
PASS = (38.0, 14.0)   # passage through the barrel: tangential x radial
SHANK = (37.0, 13.0)  # blade shank section
HEAD = (45.0, 13.0, 5.0)   # head flange: width, thickness, length (shoulders 4 mm each side seat on the barrel)
X_HEAD = -BAR_R       # head top in the blade frame (flush with the barrel)
L_B = 222.0           # head top -> tip (fits a 256 mm bed)
X_TIP = X_HEAD + L_B  # 208
X_SHANK = 30.0; X_AIR = 50.0
CHORD = (36.0, 26.0); THICK = (0.18, 0.11); PITCH_IN = 6.6; PITCH_CAP = 9.0   # deg cap inboard so the blade passes the trunnion
R_TIP = R_S + X_TIP   # 278 mm = 21.9 in
NB = 4
BETA_F, BETA_L, BETA_H = 1.0, 60.0, 90.0       # droop: flight stop, legs, hanging
CAM_PHI, CAM_D = 30.0, -0.6                    # fold cam on the trunnion: a straight edge, 30 deg above the outboard horizontal at droop 0, 0.6 mm under the hinge axis
CAM_S = (5.0, 19.5)                            # cam edge extent along itself (from the foot of the perpendicular)
HEEL_Y = (19.5, 25.5)                          # cam arm between the passage and the cheek (6 mm wide, steel-capped edge)
FOL_X, FOL_R = 12.0, 4.0                       # follower: dia 8 x 15 dowel across the rod foot, 12 mm outboard of the hinge
FOL_Y = (19.3, 31.5)                           # follower: over the cam arm (19.5-25.5) and out past the barrel end to the rod
ROD_Y = 29.0                                   # push rod centre line (tangential offset)
Z_HUB_FACE = -34.0                             # hub pad face (centring cones); docked plates are 2 x 34 = 68 mm apart
GAP = -2 * Z_HUB_FACE
CONE_R, CONE_H, CONE_RAD = 9.0, 7.0, 26.0      # centring cones on the hub pad (4 cones + 4 dimples): they catch a 7.6 mm offset
CONE_CAPTURE = CONE_R + 0.6 - 2.0
INSERT = 70.0                                  # blade tip depth inside the partner's plate underside
Z_TIP_STOW = -30.0
U_STOW = X_TIP - Z_H + Z_TIP_STOW              # blade lift above seated when stowed (194)
U_DOCK = X_TIP - Z_H - (GAP + INSERT)          # ... when docked (86)
STROKE_DOCK = U_STOW - U_DOCK                  # 108
LATCH_Z = 9.0                                  # latch pin axis above the partner's plate underside
X_LATCH = X_TIP - (INSERT - LATCH_Z)           # eyelet station on the blade (147)
LATCH_CH = 0.55                                # eyelet at 55 % chord
PIN_D, PIN_OUT = 6.0, 6.0                      # latch pin: dia 6 dowel with a 45 deg ramp ground on its nose; protrusion into the socket
EYE_R = 3.6                                    # eyelet outer radius
OWN = (46.0, 15.0)                             # own scabbard inner section: tangential x radial
GUEST = (36.0, 9.0)                            # guest socket inner section (tangential x radial)
GUEST_R = (65.0, 74.6)                         # ... its radial walls: the partner blade runs 0.6 mm from the outboard (latch) wall
GUEST_H = 82.0                                 # guest socket top
WALL = 1.5
PLATE_T = 6.0; PLATE_R = 125.0
Z_CAGE_TOP = 212.0
WELL_R = (38.0, 41.0); WELL_H = 35.0
MOTOR = dict(name="T-Motor MN605-S KV320", m=0.350, d=70.0, h=34.0, P=1400.0)
ROD_D = 5.0
BETA_UNHOOK = 76.0                             # droop at which the head lips have swung clear of the elevator toes
Z_FING = -0.5                                  # finger rod centre in the blade frame (radial offset from the hinge radius)
TURRET_R = 90.0; Z_TURRET = 224.0              # mantle turret over the cage (outer radius, top)
DOME = (125.0, 150.0, 6.0)                     # rev 4 dome: a, c, base z

RHO_M = dict(pacf=1150.0, petg=1250.0, alu=2700.0, steel=7850.0, tpu=1200.0, cf=1550.0, brass=8500.0)
S_PACF, S_BEAR = 80.0, 60.0                    # MPa (est.)
# thrust basis as trivet.py
T_REF, Q_REF, RPM_REF = 1.502 * G, 0.34, 2832.0
K_SOL = 1.60          # 4 blades, mean chord 32 vs trivet's 3 x 39 (est.)
FM = 0.50             # disc is 16 mm under the plate instead of 142 mm: more blockage than the Trivet's 0.55 (est.)
ETA = 0.85
A_EFF = np.pi * ((R_TIP * 1e-3) ** 2 - 0.125 ** 2)
RPM_MAX = 4700.0; PROOF = 1.15
# robot basis (project docs): rev 4 + XL claw-props + 2 packs
XL = dict(R=0.09525, T_MAX=26.5, K_P=5.1, Q_PER_T=0.0173, POD_EXTRA=0.075)    # power4.py "xl" (est.)
M_PACK = 0.81; EDF_MASS = 0.450; TRIVET_MASS = 0.904


# ------------------------------------------------------------------ CAD helpers
def ycyl(r, y0, y1, x=0.0, z=0.0, n=32):
    return m3.Manifold.cylinder(y1 - y0, r, r, n).rotate([-90, 0, 0]).translate([x, y0, z])


def xcyl(r, x0, x1, y=0.0, z=0.0, n=24):
    return m3.Manifold.cylinder(x1 - x0, r, r, n).rotate([0, 90, 0]).translate([x0, y, z])


def zcyl(r, z0, z1, n=64, x=0.0, y=0.0):
    return m3.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])


def ring(r0, r1, z0, z1, n=64):
    return zcyl(r1, z0, z1, n) - zcyl(r0, z0 - 1, z1 + 1, n)


def bx(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def hull(*parts):
    return m3.Manifold.batch_hull(list(parts))


def union(parts):
    return m3.Manifold.batch_boolean(list(parts), m3.OpType.Add)


def rz(man, deg):
    return man.rotate([0, 0, deg])


def four(man, off=0.0):
    return union([rz(man, off + 90 * k) for k in range(4)])


# ------------------------------------------------------------------ blade
def station(x):
    u = np.clip((x - X_AIR) / (X_TIP - X_AIR), 0, 1)
    c = CHORD[0] + (CHORD[1] - CHORD[0]) * u
    t = THICK[0] + (THICK[1] - THICK[0]) * u
    th = min(np.arctan(PITCH_IN * 25.4 / (2 * np.pi * (R_S + x))), PITCH_CAP * DEG)
    return c, t, th


def section(x, n=18, pitched=True):
    """NACA 44xx section at station x as (y, z); leading edge +y, pitch axis at mid chord (so it passes the trunnion)."""
    c, t, th = station(x)
    xi = 0.5 * (1 - np.cos(np.linspace(0, np.pi, n)))
    yt = 5 * t * (0.2969 * np.sqrt(xi) - 0.126 * xi - 0.3516 * xi ** 2 + 0.2843 * xi ** 3 - 0.1036 * xi ** 4)
    m, p = 0.04, 0.4
    yc = np.where(xi < p, m / p ** 2 * (2 * p * xi - xi ** 2), m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * xi - xi ** 2))
    up = np.stack([(0.5 - xi) * c, (yc + yt) * c], 1); lo = np.stack([(0.5 - xi) * c, (yc - yt) * c], 1)[::-1][1:-1]
    P = np.concatenate([up, lo])
    if pitched:
        P = P @ np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]])
    return P


def section_props(x):
    P = section(x, n=60, pitched=False)
    y, z = P[:, 0], P[:, 1]; y2, z2 = np.roll(y, -1), np.roll(z, -1)
    cr = y * z2 - y2 * z
    A = cr.sum() / 2; zc = ((z + z2) * cr).sum() / (6 * A)
    I = abs(((z ** 2 + z * z2 + z2 ** 2) * cr).sum() / 12) - abs(A) * zc ** 2
    return abs(A), I, I / np.abs(z - zc).max()


def latch_yz():
    """Eyelet centre (y, z) on the blade at X_LATCH (on the camber line at LATCH_CH chord, pitched)."""
    c, t, th = station(X_LATCH)
    xi = LATCH_CH; m, p = 0.04, 0.4
    yc = m / (1 - p) ** 2 * (1 - 2 * p + 2 * p * xi - xi ** 2)
    v = np.array([(0.5 - xi) * c, yc * c]) @ np.array([[np.cos(th), np.sin(th)], [-np.sin(th), np.cos(th)]])
    return v


def blade_solid(holes=True):
    """Blade in its own frame: origin on the hinge axis when seated, x along the span, y to the leading edge, z up."""
    xs = np.linspace(X_AIR, X_TIP - 3, 12)
    secs = [np.column_stack([np.full(len(section(x)), x), section(x)]) for x in xs]
    air = [m3.Manifold.hull_points(np.vstack([a, b])) for a, b in zip(secs[:-1], secs[1:])]
    tip = secs[-1]
    toe = m3.Manifold.hull_points(np.vstack([tip, tip * [1, 0.75, 0.6] + [3.0, 0, 0]]))     # rounded lead-in tip
    w, t = SHANK
    shank = bx(X_HEAD, X_SHANK, -w / 2, w / 2, -t / 2, t / 2)
    trans = hull(bx(X_SHANK - 0.5, X_SHANK, -w / 2, w / 2, -t / 2, t / 2), m3.Manifold.hull_points(np.vstack([secs[0], secs[0] + [0.5, 0, 0]])))
    hw, ht, hl = HEAD
    head = bx(X_HEAD, X_HEAD + hl, -hw / 2, hw / 2, -ht / 2, ht / 2)
    body = union(air + [toe, shank, trans, head])
    body = body - (bx(X_HEAD - 1, 0, -30, 30, -10, 10) - ycyl(BAR_R, -30, 30))        # head top rounded to the barrel radius
    if holes:
        ly, lz = latch_yz()
        body = body - zcyl(EYE_R, -8, 8, 20, X_LATCH, ly)                       # eyelet bore (brass eyelet OD 7.2 / ID 6.2)
        body = body - bx(X_HEAD - 1, X_HEAD + 7.2, -2.6, 2.6, Z_FING - 2.3, 7.0) - bx(X_HEAD + 2.7, X_HEAD + 7.2, -2.6, 2.6, Z_FING - 5.4, Z_FING - 2.2)
    return body


def eyelet_solid():
    ly, lz = latch_yz()
    return zcyl(EYE_R, lz - 2.2, lz + 2.2, 20, X_LATCH, ly) - zcyl(PIN_D / 2 + 0.1, lz - 4, lz + 4, 20, X_LATCH, ly)


def place_blade(man, k, beta, u=0.0, psi=0.0):
    """Blade k at droop beta (deg), lifted u mm back through the trunnion, rotor azimuth psi."""
    return man.translate([-u, 0, 0]).rotate([0, beta, 0]).translate([R_S, 0, Z_H]).rotate([0, 0, psi + 90 * k])


# ------------------------------------------------------------------ trunnion and hub (rotating)
def follower_z(beta):
    """Height of the follower axis above the hinge axis when it rests on the cam at droop beta (deg)."""
    th = (CAM_PHI - beta) * DEG
    return (CAM_D + FOL_R + FOL_X * np.sin(th)) / np.cos(th)


def cam_lever(beta):
    """Hinge moment per unit push-rod force (mm) and the side-force ratio on the rod guide."""
    th = (CAM_PHI - beta) * DEG
    z = follower_z(beta)
    s_c = FOL_X * np.cos(th) + (z - 0.0) * np.sin(th)
    return s_c / np.cos(th), abs(np.tan(th))


def barrel_solid():
    """Trunnion in the blade frame (rotates with the blade about y): barrel, passage, head seat, journal bores, cam arm."""
    pw, pt = PASS; hw, ht, hl = HEAD
    b = ycyl(BAR_R, -BAR_HL, BAR_HL)
    b = b - bx(-BAR_R - 1, BAR_R + 1, -pw / 2, pw / 2, -pt / 2, pt / 2)                       # passage
    b = b - bx(-BAR_R - 1, X_HEAD + hl, -hw / 2 - 0.4, hw / 2 + 0.4, -ht / 2 - 0.4, ht / 2 + 0.4)   # head seat
    b = b - ycyl(3.0, -BAR_HL - 1, -pw / 2 - 0.01) - ycyl(3.0, pw / 2 + 0.01, BAR_HL + 1)     # journal bores (M6 shoulder bolts, dia 6 in bushings)
    a = CAM_PHI * DEG; n = np.array([-np.sin(a), np.cos(a)]); t = np.array([np.cos(a), np.sin(a)])
    pts = [CAM_D * n + CAM_S[0] * t, CAM_D * n + CAM_S[1] * t, (CAM_D - 7.0) * n + (CAM_S[1] - 2.0) * t, (CAM_D - 7.0) * n + CAM_S[0] * t]
    arm = hull(*[ycyl(0.05, HEEL_Y[0], HEEL_Y[1], p[0], p[1], 8) for p in pts])
    neck = bx(-2.0, BAR_R + 1, HEEL_Y[0] - 0.3, BAR_HL + 1, -2.0, BAR_R + 1) - ycyl(7.0, HEEL_Y[0] - 1, BAR_HL + 2)
    return (b - neck) + (arm - ycyl(6.5, -BAR_HL, BAR_HL))


def place_barrel(man, k, beta, psi=0.0):
    return man.rotate([0, beta, 0]).translate([R_S, 0, Z_H]).rotate([0, 0, psi + 90 * k])


def hub_solid():
    """Hub (one PA-CF print + TPU pad): top plate on the motor bell, ring wall, pad plate with 4 cones + 4 dimples,
    8 cheeks.  The concave backing wall behind each barrel (r 14.6 about the hinge) stops a blade sliding inward
    whenever its droop is below ~75 deg."""
    top = zcyl(57.0, -9.0, -3.0)
    wall = ring(47.0, 57.0, Z_HUB_FACE + 2, -9.0)
    pad = zcyl(47.5, Z_HUB_FACE, Z_HUB_FACE + 9)
    h = top + wall + pad
    cheeks = []
    for s in (1, -1):
        y0, y1 = (BAR_HL + 0.5, BAR_HL + 5.5) if s > 0 else (-BAR_HL - 5.5, -BAR_HL - 0.5)
        cheeks.append(hull(bx(50, R_S, y0, y1, Z_H - 12, -3.0), ycyl(7.0, y0, y1, R_S, Z_H, 24)))
    lobe = union(cheeks) + bx(40, R_S - 7.2, -BAR_HL - 5.5, BAR_HL + 5.5, -9, -3)
    h = h + four(lobe)
    cut = ycyl(BAR_R + 0.6, -BAR_HL - 0.5, BAR_HL + 0.5, R_S, Z_H) + bx(R_S - 7.2, R_S + 20, -BAR_HL - 0.5, BAR_HL + 0.5, Z_H, 0)
    h = h - four(cut)
    h = h - four(ycyl(3.0, -BAR_HL - 7, BAR_HL + 7, R_S, Z_H, 16))                 # hinge bolt bores
    cone = m3.Manifold.cylinder(CONE_H, 2.0, CONE_R, 32).translate([CONE_RAD, 0, Z_HUB_FACE - CONE_H])
    dimp = m3.Manifold.cylinder(CONE_H + 0.4, CONE_R + 0.6, 2.4, 32).translate([CONE_RAD, 0, Z_HUB_FACE - 0.01])
    h = h + four(cone) - four(dimp, 45.0)
    return h - zcyl(6.0, -10, -2)


# ------------------------------------------------------------------ static parts (body)
def own_slot(clear=0.0, z0=-1.0, z1=1.0):
    w, t = OWN
    return bx(R_S - t / 2 - clear, R_S + t / 2 + clear, -w / 2 - clear, w / 2 + clear, z0, z1)


def guest_slot(clear=0.0, z0=-1.0, z1=1.0):
    w, t = GUEST
    return bx(GUEST_R[0] - clear, GUEST_R[1] + clear, -w / 2 - clear, w / 2 + clear, z0, z1)


def rod_xy():
    return R_S + FOL_X, ROD_Y


def rod_window(z0, z1):
    return bx(R_S + FOL_X - FOL_R - 0.5, R_S + FOL_X + FOL_R + 0.5, FOL_Y[0] - 0.5, FOL_Y[1] + 0.5, z0, z1)


def pin_y():
    """Tangential offset of the latch pin in a guest socket = mirrored eyelet position of the partner's blade."""
    return -latch_yz()[0]


def plate_solid():
    p = zcyl(PLATE_R, 0, PLATE_T, 96) - zcyl(WELL_R[0], -1, PLATE_T + 1)
    p = p - four(own_slot(0, -1, PLATE_T + 1)) - four(guest_slot(0, -1, PLATE_T + 1), 45.0)
    rx, ry = rod_xy()
    p = p - four(rod_window(-1, PLATE_T + 1))     # push-rod shoe windows
    w, t = GUEST
    mouth = hull(bx(GUEST_R[0] - 3, GUEST_R[1] + 3, -w / 2 - 3, w / 2 + 3, -0.01, 0.0), bx(GUEST_R[0], GUEST_R[1], -w / 2, w / 2, 3.0, 3.01))
    return p - four(mouth, 45.0)


def cage_solid():
    """Scabbard cage: 4 own scabbards (full height, windowed), 4 guest sockets with latch housings, base flange,
    top ring, motor well.  One PETG print (168 x 168 x 206, fits a 256 mm bed)."""
    w, t = OWN
    own = bx(R_S - t / 2 - WALL, R_S + t / 2 + WALL, -w / 2 - WALL, w / 2 + WALL, PLATE_T, Z_CAGE_TOP) - own_slot(0, 0, Z_CAGE_TOP + 1)
    own = own - bx(R_S - t / 2 - WALL - 1, R_S - t / 2 + 0.01, -3, 3, PLATE_T + 1.5, Z_CAGE_TOP - 1)       # finger web slot (inboard wall)
    own = own - rod_window(0, PLATE_T + 4.0)                                                  # follower pocket
    for z0 in np.arange(24.0, Z_CAGE_TOP - 40, 44.0):                                          # lightening windows
        own = own - bx(R_S + t / 2 - 0.01, R_S + t / 2 + WALL + 1, -w / 2 + 6, w / 2 - 6, z0, z0 + 34.0)
        own = own - bx(R_S - t / 2 - WALL - 1, R_S - t / 2 + 0.01, -w / 2 + 5, -6, z0, z0 + 34.0) - bx(R_S - t / 2 - WALL - 1, R_S - t / 2 + 0.01, 6, w / 2 - 5, z0, z0 + 34.0)
    w, t = GUEST; py = pin_y()
    gst = bx(GUEST_R[0] - 2.0, GUEST_R[1] + 2.0, -w / 2 - 2.0, w / 2 + 2.0, PLATE_T, GUEST_H) - guest_slot(0, 0, GUEST_H - 2.0)
    gst = gst + bx(GUEST_R[1], GUEST_R[1] + 24, py - 7, py + 7, PLATE_T, PLATE_T + 9) - xcyl(PIN_D / 2 + 0.1, GUEST_R[1] - 3, GUEST_R[1] + 25, py, LATCH_Z, 20)
    base = ring(WELL_R[0], 100.0, PLATE_T, PLATE_T + 2.0) - four(own_slot(0, 0, 20)) - four(guest_slot(0, 0, 20), 45.0) \
        - four(rod_window(0, 20))
    top = ring(56.0, 82.0, Z_CAGE_TOP, Z_CAGE_TOP + 3) - four(own_slot(0, Z_CAGE_TOP - 1, Z_CAGE_TOP + 5))
    well = ring(WELL_R[0], WELL_R[1], PLATE_T, WELL_H) + zcyl(WELL_R[1], WELL_H, WELL_H + 4) - zcyl(8, WELL_H - 1, WELL_H + 5)
    return four(own) + four(gst, 45.0) + base + top + well


def latch_pin(retract=0.0):
    x0 = GUEST_R[1] - PIN_OUT + retract
    py = pin_y()
    pin = xcyl(PIN_D / 2, x0, x0 + 30.0, py, LATCH_Z, 20)
    cutter = bx(-10, 0, -5, 5, -12, 0).rotate([0, -45, 0]).translate([x0, py, LATCH_Z - PIN_D / 2])   # removes the lower nose
    return pin - cutter


Z_FOL_UP = 0.5 + FOL_R                         # follower axis when retracted for flight (inside the plate)


def rods_solid(beta=None):
    """Four push rods with their follower pins; beta None = retracted for flight.  Returns (solid, follower axis z)."""
    rx, ry = rod_xy()
    zc = Z_H + follower_z(beta) if beta is not None else Z_FOL_UP
    zc = min(zc, Z_FOL_UP)
    fol = ycyl(FOL_R, FOL_Y[0], FOL_Y[1], rx, zc, 24)
    rod = zcyl(ROD_D / 2, zc, zc + 66.0, 16, rx, ry)
    return four(fol + rod), zc


def fold_ring(zt):
    """Ring that carries the four rods, on three T8x2 lead screws (r 92, between the rods)."""
    return ring(84.0, 97.0, zt + 60.0, zt + 66.0)


def elevator_solid(u):
    """Elevator ring + 4 fingers for blade lift u."""
    zh = Z_H - X_HEAD + u                                   # head top
    rg = ring(44.0, 53.0, zh + 10.0, zh + 18.0)
    rf = R_S + Z_FING
    f = []
    for k in range(4):
        web = bx(49.0, rf + 1.0, -2.5, 2.5, zh + 10.0, zh + 18.0)
        rod = zcyl(2.0, zh - 6.9, zh + 13.0, 12, rf, 0.0)
        toe = bx(rf - 5.0, rf, -2.0, 2.0, zh - 6.9, zh - 3.9)
        f.append(rz(web + rod + toe, 90 * k))
    return rg + union(f)


def motor_solid():
    return zcyl(MOTOR["d"] / 2, -3.0, -3.0 + MOTOR["h"] - 6, 48) + zcyl(22.0, MOTOR["h"] - 9.0, MOTOR["h"] - 3.0 + 4, 32)


def mantle_profile(turret=True, n=60):
    """(r, z) outline of the outer shell: rev 4 dome, with the turret over the cage when turret=True."""
    a, c, z0 = DOME
    pts = []
    for th in np.linspace(0, np.pi / 2, n):
        r, z = a * np.cos(th), z0 + c * np.sin(th)
        if turret and r < TURRET_R:
            break
        pts.append((r, z))
    if turret:
        pts.append((TURRET_R, pts[-1][1]))
        zc = Z_TURRET - 22.0
        pts.append((TURRET_R, zc))
        for th in np.linspace(0, np.pi / 2, 12)[1:]:
            pts.append((TURRET_R - 22.0 * (1 - np.cos(th)) * (TURRET_R / 22.0) ** 0 - (TURRET_R - 22.0) * 0 - 0.0, zc + 22.0 * np.sin(th)))
        pts[-1] = (TURRET_R - 22.0, Z_TURRET)
        pts.append((0.0, Z_TURRET))
    return np.array(pts)


def mantle_solid(turret=True, t=1.6):
    prof = mantle_profile(turret)
    cs = m3.CrossSection([np.vstack([[0.0, prof[0][1]], prof, ])])
    outer = cs.revolve(72)
    inner = outer.scale([(prof[0][0] - t) / prof[0][0]] * 2 + [1.0]).translate([0, 0, -t])
    return outer - inner - zcyl(200, -50, DOME[2] + 0.01)


# ------------------------------------------------------------------ assemblies
def rotating(beta=BETA_F, u=0.0, psi=0.0, blades=True):
    out = {"hub": rz(hub_solid(), psi)}
    bar = barrel_solid(); bl = blade_solid() if blades else None
    for k in range(4):
        out[f"barrel{k}"] = place_barrel(bar, k, beta, psi)
        if blades:
            out[f"blade{k}"] = place_blade(bl, k, beta, u, psi)
    return out


def static(beta_rod=None, u_elev=None, pins=0.0):
    rods, zt = rods_solid(beta_rod)
    out = {"plate": plate_solid(), "cage": cage_solid(), "motor": motor_solid(), "rods": rods, "fold_ring": fold_ring(zt),
           "pins": four(latch_pin(pins), 45.0)}
    if u_elev is not None:
        out["elevator"] = elevator_solid(u_elev)
    return out


def partner(man):
    return man.rotate([180, 0, 0]).rotate([0, 0, 45]).translate([0, 0, -GAP])


def mass_props(man, rho):
    import trimesh
    me = man.to_mesh()
    tm = trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=False)
    return tm.volume * 1e-9 * rho, tm.center_mass
# (masses, loads, thrust, docked flight and STL export: see the project copy claude/code/scabbard.py; this workspace
#  copy carries the geometry the turret-camera work needs)
