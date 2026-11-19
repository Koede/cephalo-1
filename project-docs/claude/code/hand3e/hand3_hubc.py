"""hand3 hub E (2026-11-15, integrated): hub C2's drive + frame (alt_drive) with the cord routing, ring anchors / keys, series links
and length trim of alt_cord, on one geometry.  This is THE hub module of the design folder (hand3_hubc.py: same API).
Integration changes on top of hub C2: cage / rings replaced by the cord-routing versions (wide cage slot per finger, curl lug on
ring_in, base hanger + lug on ring_out, TPU frame links on M2 trim screws), the lag tube is the base cord's pulley (handed tubes,
side hole at 150 deg), E_LAG = 2.8 mm (tube radius + cord), all heights relative to H.Z (STACK_DZ follows).

--- hub C2 notes (drive and frame) ---
hand3 hub C2 (2026-11-15): hub C's two pull rings with a drive that works with ONE servo output per ring.
Same API as hand3_hubc.py (parts, hardware, assembly, overlaps, SKIP, sizing, TRAVEL_B / TRAVEL_C, Z_FL_IN, Z_SERVO_TOP);
python3 hand3_hubc.py -> hand3_hubc.json + hand3_hubc_stl/.  Units mm, N, g.  Estimates.

What changed against hub C (hand3_hubc.py):
  * Stack 4 mm longer (hand3_design.STACK_DZ): the servos stand on the 2 mm base plate (they were drawn 1.9 mm into it) and a
    2 mm bridge above them joins the motor column to the end walls (the frame was two loose bodies).  All rotor-side heights
    follow H.Z.
  * Each yoke stays two halves (there is no room for a joining ring: the four legs, eight rods and both flanges share r 30-38),
    but every half is pulled by its own cord and both cords of a ring wind on ONE two-groove spool on the servo's output:
    equal travel on both sides.  The near cord leaves the top of the spool, the far cord the bottom, so one direction of rotation
    winds both.  Cords turn on MR63 ball-bearing idlers (3 x 6 x 2.5) on Ø3 axles in frame brackets: near cord 1 bend, far cord 2.
  * Halves are placed where their cords can run straight up to them: in-yoke (curl) halves at 34.9 / 214.9 deg, out-yoke (base)
    halves at 145.1 / 327.8 deg.  Each half rides on TWO Ø3 rods at +-10 deg (no yaw), its cord pulls midway between them.
  * Lips +-18 deg (were +-38) so the out and in lips never share an azimuth; arms 1.0 mm clear of the flanges (were 0.5).
  * Servos: CURL servo below (shaft x +12), BASE servo above (shaft x -12): the low cords then never meet the in-yoke legs.
  * Rods end 1 mm above their bosses (were at the rotor plane: phalanx 1 passed 0.35 mm above them).
"""
import os, json
import numpy as np
import manifold3d as mf
import hand3_design as H
import hand3_cad as C
from hand3_cad import cyl, box, app, T, rot, to_trimesh

M_ = mf.Manifold
Z = H.Z
SERVO = dict(name="Feetech STS3215", stall=2.94, rated=0.981, m=55.0, size=(45.2, 24.7, 35.0), volts="12 V", stall_A=2.7, deg=360.0, sec60=0.222)
SPOOL_R = 4.0                                # cord centre radius on the drum
LEVER = list(H.LEVER)
E_LAG = 2.8                                  # base cord wraps the Ø5 lag tube: arm = tube radius + cord radius (alt_cord)
STRETCH = 2.0
LAG_SPRING, EXT_K1, EXT_K2, EXT_K3 = 0.0054, 0.02, 0.004, 0.016     # as built by hand3_springs.py (O-ring picks; K3 0.016: lane length limit)
TRAVEL_B = E_LAG * np.radians(H.SPREAD_MAX) + LEVER[0] * np.radians(H.FOLD_MAX[0]) + STRETCH
TRAVEL_C = LEVER[1] * np.radians(H.FOLD_MAX[1]) + LEVER[2] * np.radians(H.FOLD_MAX[2]) + STRETCH
GAP = 1.0
# efficiency: spool + cord 0.75 (as hub C) x rod guides 0.80 (two rods per half, pull midway) x idlers (MR63, 0.97 per bend)
ETA_SPOOL, ETA_GUIDE, ETA_IDLER = 0.75, 0.80, 0.97
# radii
R_CAGE = (19.5, 21.5); R_IN = (21.9, 23.9); R_OUT = (24.3, 26.3); R_FL_OUT = 30.0; R_FL_IN = 32.5
R_LIP_OUT = (27.1, 30.0); R_LIP_IN = (30.5, 32.5); R_LEG = (33.5, 36.0); R_ROD = 37.6
ARM_CLR = 1.0
R_TIE = (36.3, 40.3)                         # rod tie ring: the eight rod tops glued into one stationary ring (rods fixed both ends)                                # yoke arm to the flange it steps down beside
LIP_HALF, LEG_HALF, ROD_DAZ = 18.0, 7.0, 10.0

# axial stack (flight), from the hub
Z_OUT = (Z["hub_lo"] - 7.0, Z["hub_lo"] - 4.0); Z_FL_OUT = (Z_OUT[0], Z_OUT[0] + 2.0); Z_LIP_OUT = (Z_FL_OUT[1] + GAP, Z_FL_OUT[1] + GAP + 2.0)
Z_FL_IN = (Z_OUT[0] - TRAVEL_B - GAP - 2.0, Z_OUT[0] - TRAVEL_B - GAP); Z_IN = (Z_FL_IN[0], Z_OUT[1])
Z_LIP_IN = (Z_FL_IN[1] + GAP, Z_FL_IN[1] + GAP + 2.0)
Z_TIE = (Z_LIP_OUT[0] - 7.0, Z_LIP_OUT[0] - 5.0)          # tie ring 1 mm above the out-yoke bosses at rest, under the out-yoke arms' path? (checked)
LEG = dict(out=(18.0, 8.0), inn=(12.0, 1.0))  # (leg drop = boss bottom, boss top) below the lip: the in-yoke is short so the base
                                             # ring's cords (z 35-47) pass under it even at full curl stroke
# servos: base plate 0-2, curl servo 2.0-26.7 (shaft x +12), base servo 26.7-51.4 (shaft x -12), bridge 51.4-53.4
SL, SW, SH = SERVO["size"]
PLATE_T, BRIDGE_T, WALL_T = 2.0, 2.0, 2.0
Z_CURL = PLATE_T; Z_BASE = PLATE_T + SW; Z_SERVO_TOP = PLATE_T + 2 * SW
SHAFT_X = {"curl": SL / 2 - 10.6, "base": -(SL / 2 - 10.6)}
SHAFT_Z = {"curl": Z_CURL + SW / 2, "base": Z_BASE + SW / 2}
Y_HALF = SH / 2 + 0.5                        # frame (walls, plate, bridge) half width in y: 18.0; spools and cords live at y > 18
SPOOL_Y = (Y_HALF, Y_HALF + 6.0)             # flange 18.0-18.6 | groove 1 18.6-21.0 | flange 21.0-21.6 | groove 2 21.6-23.4 | flange 23.4-24.0
Y1, Y2 = 19.8, 22.5                          # near / far cord planes
X_N, X_F = 29.5, 31.5                        # x of the near-cord idlers (both rings) and of the base ring's far route (cord eyes >= 0.8 mm off the flanges)
IDLER_R, IDLER_W, CORD_R = 3.0, 2.5, 0.35    # MR63 ball bearing; cord drawn as Ø0.7
Z_LOW = 8.0                                  # the base ring's far cord crosses to the far side at this height (over the rod sockets)
X_DROP = 25.5                                # ... after dropping just outside the +x end wall (wall face at x 24.6)


def _az(x, y): return float(np.degrees(np.arctan2(y, x)))


# cord routes (stationary frame), each ends straight below its yoke half's attach point
def routes():
    """ring -> {half: (yoke half az, attach (x, y) , [polyline points from the spool tangent point to the bottom of the vertical run])}."""
    out = {}
    for ring in ("curl", "base"):
        xs, zs = SHAFT_X[ring], SHAFT_Z[ring]; s = 1.0 if ring == "curl" else -1.0      # curl: near cord heads +x, base: -x
        zt, zb = zs + SPOOL_R, zs - SPOOL_R
        near_xy = (s * X_N, Y1)
        far_x = -s * X_F; far_xy = (far_x, -Y1)                                            # far side: 1.1 mm outside the end wall for the idler
        near = [np.array([xs, Y1, zt]), np.array([near_xy[0], Y1, zt])]                      # top of the spool -> idler N, then up
        if ring == "curl":                                                                  # bottom -> idler F1 -> idler F2, then up
            far = [np.array([xs, Y2, zb]), np.array([far_x, Y2, zb]), np.array([far_x, -Y1, zb])]
        else:                                                                               # base: drops beside the wall, crosses low, rises
            far = [np.array([xs, Y2, zb]), np.array([X_DROP, Y2, zb]), np.array([X_DROP, Y2, Z_LOW]), np.array([far_x, -Y1, Z_LOW])]
        out[ring] = dict(near=(_az(*near_xy), near_xy, near, zt), far=(_az(*far_xy), far_xy, far, zb))
    return out


ROUTES = routes()
HALVES = {("in", k): ROUTES["curl"][k] for k in ("near", "far")}
HALVES.update({("out", k): ROUTES["base"][k] for k in ("near", "far")})


def lip_z(which): return Z_LIP_OUT if which == "out" else Z_LIP_IN
def leg(which): return LEG["out" if which == "out" else "inn"]


def sizing():
    out = {}
    for nm, trav in (("base", TRAVEL_B), ("curl", TRAVEL_C)):
        # the two halves share the ring force equally (equal travel, rigid ring); spool torque = r (T/2 / eta_near + T/2 / eta_far)
        nf = 2 if nm == "curl" else 3
        eta_idl = 2.0 / (1.0 / ETA_IDLER + 1.0 / ETA_IDLER ** nf)
        eta = ETA_SPOOL * ETA_GUIDE * eta_idl
        Fs, Fr = SERVO["stall"] * eta / (SPOOL_R * 1e-3), SERVO["rated"] * eta / (SPOOL_R * 1e-3)
        need = H.joint_torques(H.F_TIP_PEAK)
        Fp = need[0] / (LEVER[0] * 1e-3) if nm == "base" else max(need[1] / (LEVER[1] * 1e-3), need[2] / (LEVER[2] * 1e-3))
        deg = float(np.degrees((trav + GAP) / SPOOL_R))
        out[nm] = dict(travel=round(float(trav), 1), spool_deg=round(deg), eta=round(eta, 3), F_stall_ring=round(Fs), F_rated_ring=round(Fr),
                       F_stall_finger=round(Fs / 4, 1), F_rated_finger=round(Fr / 4, 1), F_peak_need=round(float(Fp), 1), margin=round(Fs / 4 / float(Fp), 2),
                       tpu_k=round(float(Fp) / STRETCH, 1), t_noload_s=round(deg / 60 * SERVO["sec60"], 2),
                       t_at_peak_s=round(deg / 60 * SERVO["sec60"] / max(1 - 4 * Fp / Fs, 0.05), 2))
    return out


ETA = round(ETA_SPOOL * ETA_GUIDE * 2.0 / (1.0 / ETA_IDLER + 1.0 / ETA_IDLER ** 3), 3)       # worst ring (base: far cord 3 bends)


def arc(r0, r1, z0, z1, az, half):
    m = cyl(r1, z0, z1, n=96) - cyl(r0, z0 - 1, z1 + 1, n=96)
    h = np.radians(half)
    m = C.half(C.half(m, np.array([np.sin(h), np.cos(h), 0.0]), [0, 0, 0]), np.array([np.sin(h), -np.cos(h), 0.0]), [0, 0, 0])
    return app(m, T(rot("z", az)))


def cage():
    m = cyl(R_CAGE[1], Z["hub_lo"] - 31.5, Z["hub_lo"], n=96) - cyl(R_CAGE[0], Z["hub_lo"] - 40, Z["hub_lo"] + 3, n=96)
    for a in C.FAZ:
        for d in (-8.0, 8.0): m -= app(box([R_CAGE[0] - 1, -1.3, Z["hub_lo"] - 30.0], [R_CAGE[1] + 1, 1.3, Z["hub_lo"] - 2.0]), T(rot("z", a + d)))
    return m


def ring_in():
    m = cyl(R_IN[1], *Z_IN, n=96) - cyl(R_IN[0], Z_IN[0] - 1, Z_IN[1] + 1, n=96)
    m += cyl(R_FL_IN, *Z_FL_IN, n=96) - cyl(R_IN[0], Z_FL_IN[0] - 1, Z_FL_IN[1] + 1, n=96)
    for a in C.FAZ: m += app(box([R_CAGE[0] + 0.3, -1.0, Z_IN[1] - 4.0], [R_IN[0] + 0.2, 1.0, Z_IN[1]]), T(rot("z", a + 8.0)))
    return m


def ring_out():
    m = cyl(R_OUT[1], *Z_OUT, n=96) - cyl(R_OUT[0], Z_OUT[0] - 1, Z_OUT[1] + 1, n=96)
    m += cyl(R_FL_OUT, *Z_FL_OUT, n=96) - cyl(R_OUT[0], Z_FL_OUT[0] - 1, Z_FL_OUT[1] + 1, n=96)
    return m


def rod_xy(az, k): a = np.radians(az + k * ROD_DAZ); return np.array([R_ROD * np.cos(a), R_ROD * np.sin(a)])


def yoke_half(which, side):
    """One half: lip over the flange, arm stepping down beside it, leg, two rod bosses at +-10 deg joined by a tab, cord eye."""
    az, (ax, ay), _, _ = HALVES[(which, side)]
    lip_r = R_LIP_OUT if which == "out" else R_LIP_IN; lz = lip_z(which)
    fl_r = R_FL_OUT if which == "out" else R_FL_IN
    m = arc(lip_r[0], lip_r[1] + 1.0, lz[0], lz[1], az, LIP_HALF)
    m += arc(fl_r + ARM_CLR, R_LEG[1], lz[0] - 2.5, lz[0] + 0.5, az, LEG_HALF + 2.0)
    zb0, zb1 = lz[0] - leg(which)[0], lz[0] - leg(which)[1]
    m += arc(R_LEG[0], R_LEG[1], zb0, lz[0] + 0.5, az, LEG_HALF)
    bosses = [cyl(3.2, zb0, zb1, c=(*rod_xy(az, k), 0)) for k in (-1, 1)]
    ra = np.hypot(ax, ay)
    eye = cyl(2.2, zb0, zb0 + 4.0, c=(ax, ay, 0))                                     # cord eye (attach) under the tab
    tab = M_.batch_hull(bosses + [app(box([R_LEG[0], -3.0, zb0], [R_LEG[0] + 1.0, 3.0, zb1]), T(rot("z", az))), eye])
    m += tab
    for k in (-1, 1): m -= cyl(1.6, zb0 - 1, zb1 + 1, c=(*rod_xy(az, k), 0))           # rod bores (Ø3.2 on a Ø3 rod)
    m -= cyl(0.6, zb0 - 1, zb0 + 5.0, c=(ax, ay, 0)) + cyl(1.4, zb0 + 2.0, zb0 + 5.0, c=(ax, ay, 0))   # cord hole + knot pocket
    return m


def tie_ring():
    """Stationary ring on the eight rod tops (Ø3 holes, glued): the rods become fixed-fixed beams, so the yoke halves' tilt
    couple (cord at r 35-37 vs lip contact at r 28-33) is carried in bending at both ends."""
    m = cyl(R_TIE[1], *Z_TIE, n=96) - cyl(R_TIE[0], Z_TIE[0] - 1, Z_TIE[1] + 1, n=96)
    for (w, sd), (az, _, _, _) in HALVES.items():
        for k in (-1, 1): m -= cyl(1.52, Z_TIE[0] - 1, Z_TIE[1] - 0.5, c=(*rod_xy(az, k), 0), n=24)
    return m


def spool():
    """Two-groove spool on the STS3215 output (axis y, servo face at y = 17.5).  Built on the y axis at x = z = 0."""
    y0, y1 = SPOOL_Y; fr, dr = SPOOL_R + 1.5, SPOOL_R - 0.35
    m = cyl(fr, y0, y0 + 0.6, "y") + cyl(dr, y0 + 0.6, y0 + 3.0, "y") + cyl(fr, y0 + 3.0, y0 + 3.6, "y") + cyl(dr, y0 + 3.6, y0 + 5.4, "y") + cyl(fr, y0 + 5.4, y1, "y")
    m -= cyl(2.95, y0 - 1, y0 + 2.5, "y", n=48) + cyl(1.7, y0 - 1, y1 + 1, "y", n=24)     # 25T spline (bore 5.9, print-to-fit) + M3 screw
    for yc in (Y1, Y2): m -= cyl(0.45, -fr - 1, fr + 1, "z", (0, yc, 0), n=12)          # cord anchor holes
    return m


def corners():
    """Idler positions: (name, corner point, cord in-direction, out-direction)."""
    out = []
    for ring, R in ROUTES.items():
        for side in ("near", "far"):
            pts = R[side][2]; seq = list(pts) + [pts[-1] + np.array([0, 0, 10.0])]
            for i in range(1, len(seq) - 1):
                din = seq[i] - seq[i - 1]; dout = seq[i + 1] - seq[i]
                out.append((f"{ring}_{side}{i}", seq[i], din / np.linalg.norm(din), dout / np.linalg.norm(dout)))
    return out


def idler_pose(c, din, dout):
    ctr = c + (IDLER_R + CORD_R) * (dout - din)                                     # tangent to both runs, inside the bend
    ax = np.cross(din, dout); ax = ax / np.linalg.norm(ax)
    return ctr, ax


def _cyl_axis(r, h0, h1, ctr, ax, n=32):
    """Cylinder of radius r along unit axis ax through ctr, from h0 to h1 along the axis."""
    ax = np.asarray(ax, float); z = np.array([0, 0, 1.0])
    m = M_.cylinder(h1 - h0, r, r, n).translate([0, 0, h0])
    v = np.cross(z, ax); s = np.linalg.norm(v); c = float(z @ ax)
    if s < 1e-9: R = np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    else:
        k = v / s; K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]]); R = np.eye(3) + s * K + (1 - c) * K @ K
    return app(m, T(R, ctr))


def _seg(p, q, r=CORD_R):
    d = q - p; L = float(np.linalg.norm(d))
    return _cyl_axis(r, 0.0, L, p, d / L, n=12)


def frame():
    """Base plate, end walls, bridge, motor column, 8 rod sockets on arms, 6 idler brackets.  One body."""
    xw = SL / 2 + WALL_T
    m = box([-xw, -Y_HALF, 0], [xw, Y_HALF, PLATE_T])
    m += box([-xw, -Y_HALF, 0], [-SL / 2, Y_HALF, Z_SERVO_TOP + BRIDGE_T]) + box([SL / 2, -Y_HALF, 0], [xw, Y_HALF, Z_SERVO_TOP + BRIDGE_T])
    m += box([-xw, -Y_HALF, Z_SERVO_TOP], [xw, Y_HALF, Z_SERVO_TOP + BRIDGE_T])
    m += cyl(20.0, Z_SERVO_TOP, Z["pack1"], n=64)
    for k in range(4):
        a = np.radians(45 + 90 * k); p = 19.0 / 2 * np.sqrt(2) * np.array([np.cos(a), np.sin(a)])
        m -= cyl(1.7, Z_SERVO_TOP - 1, Z["pack1"] + 1, c=(p[0], p[1], 0), n=24) + cyl(3.0, Z_SERVO_TOP - 1, Z_SERVO_TOP + 1.8, c=(p[0], p[1], 0), n=32)
    for a in (90.0, 210.0, 330.0):
        p = 9.5 * np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
        m -= cyl(1.7, -1, 3, c=(p[0], p[1], 0), n=24) + M_.cylinder(1.7, 1.6, 3.2, 32).translate([p[0], p[1], 0.31])
    m -= cyl(4.0, -1, 3, n=32)
    for (which, side), (az, _, _, _) in HALVES.items():                              # rod sockets: arm along the plate to each rod
        for k in (-1, 1):
            q = rod_xy(az, k); u = q / np.linalg.norm(q)
            m += M_.batch_hull([cyl(3.0, 0, PLATE_T, c=(*q, 0)), cyl(3.0, 0, PLATE_T, c=(*(u * 15.0), 0))]) + cyl(3.0, 0, 6.0, c=(*q, 0))
            m -= cyl(1.52, 1.0, 7.0, c=(*q, 0), n=24)                                 # rod glued in 5 mm
    for name, c, din, dout in corners():                                              # idler brackets: axle boss + web to the nearest wall / plate
        m += bracket(name)[0]
    return m


_BR = {}
def bracket(name):
    """(bracket solid, axle side, leftover overlap with the keep-out volumes) for one idler: the side whose boss + web clears the idler, all cords, rods, servos and spools best."""
    if name in _BR: return _BR[name]
    c, din, dout = next((cc, d1, d2) for n, cc, d1, d2 in corners() if n == name)
    ctr, ax = idler_pose(c, din, dout); xw = SL / 2 + WALL_T
    keep = [_cyl_axis(IDLER_R + 0.6, -IDLER_W / 2 - 0.3, IDLER_W / 2 + 0.3, ctr, ax)] + [_seg(p, q, CORD_R + 0.4) for p, q in cord_segments(0.0, 0.0) + cord_segments(TRAVEL_B, TRAVEL_C)]
    for (w, sd), (az, _, _, _) in HALVES.items():
        for k in (-1, 1): keep.append(cyl(2.0, 0, 90, c=(*rod_xy(az, k), 0), n=16))
    for ring, z0 in (("curl", Z_CURL), ("base", Z_BASE)):
        keep.append(app(cyl(SPOOL_R + 2.1, SPOOL_Y[0] - 0.5, SPOOL_Y[1] + 0.5, "y"), T(t=[SHAFT_X[ring], 0, SHAFT_Z[ring]])))
        keep.append(box([-SL / 2 - 0.3, -SH / 2 - 0.3, z0], [SL / 2 + 0.3, SH / 2 + 0.3, z0 + SW]))      # servo bodies (+ axle stubs must clear them)
    for n2, c2, d2i, d2o in corners():                                               # the other idlers (+ their axle space) and brackets already placed
        if n2 == name: continue
        c2c, a2 = idler_pose(c2, d2i, d2o)
        keep.append(_cyl_axis(IDLER_R + 0.6, -IDLER_W / 2 - 3.6, IDLER_W / 2 + 3.6, c2c, a2))
        if n2 in _BR: keep.append(_BR[n2][0])
    axle = lambda sg: _cyl_axis(1.5, IDLER_W / 2 + 0.3, IDLER_W / 2 + 3.3, ctr, sg * ax, n=16)
    best = None
    for sgn in (1.0, -1.0):
        boss = _cyl_axis(2.6, IDLER_W / 2 + 0.3, IDLER_W / 2 + 3.3, ctr, sgn * ax)
        tip = ctr + sgn * ax * (IDLER_W / 2 + 1.8); bb = np.asarray(boss.bounding_box())
        xs = float(np.clip(tip[0], -xw + 1.0, xw - 1.0)); ys = float(np.clip(tip[1], -Y_HALF + 1.0, Y_HALF - 1.0))
        feet = [box([xs - 1.0, ys - 1.0, max(bb[2], 0.0)], [xs + 1.0, ys + 1.0, max(bb[5], PLATE_T)])]             # to the wall / plate, at the boss's height
        post = box([tip[0] - 1.6, tip[1] - 1.6, 0.0], [tip[0] + 1.6, tip[1] + 1.6, bb[5]])                       # or a post down to a pad on the plate
        pad = M_.batch_hull([box([tip[0] - 1.6, tip[1] - 1.6, 0.0], [tip[0] + 1.6, tip[1] + 1.6, PLATE_T]), box([xs - 1.0, ys - 1.0, 0.0], [xs + 1.0, ys + 1.0, PLATE_T])])
        feet.append(post + pad)
        for k_, foot in enumerate(feet):
            web = (M_.batch_hull([boss, foot]) if k_ == 0 else M_.batch_hull([boss, box([tip[0] - 1.6, tip[1] - 1.6, bb[2]], [tip[0] + 1.6, tip[1] + 1.6, bb[5]])]) + foot)
            web -= _cyl_axis(1.6, -12, 12, ctr, ax, n=24)
            bad = sum((web ^ k).volume() + (axle(sgn) ^ k).volume() for k in keep)
            if best is None or bad < best[0] - 1e-6: best = (bad, web, sgn)
    _BR[name] = (best[1], best[2], best[0]); return _BR[name]


def cord_runs(pull_b, pull_c):
    """name -> (p, q) for every straight cord run; name = cord_<ring>_<side><i>."""
    out = {}
    for (which, side), (az, (ax, ay), pts, z0) in HALVES.items():
        ring = "base" if which == "out" else "curl"; pull = pull_b if which == "out" else pull_c
        dz = -(pull + GAP) if pull > 0 else 0.0
        seq = list(pts) + [np.array([ax, ay, lip_z(which)[0] - leg(which)[0] + dz])]
        for i in range(len(seq) - 1): out[f"cord_{ring}_{side}{i}"] = (seq[i], seq[i + 1])
    return out


def cord_segments(pull_b, pull_c):
    """All cord runs as (p, q) pairs for the given ring strokes (the yoke halves sit pull + GAP lower when pulled)."""
    segs = []
    for (which, side), (az, (ax, ay), pts, z0) in HALVES.items():
        pull = pull_b if which == "out" else pull_c
        dz = -(pull + GAP) if pull > 0 else 0.0
        top = np.array([ax, ay, lip_z(which)[0] - leg(which)[0] + dz])
        seq = list(pts) + [top]
        for i in range(len(seq) - 1): segs.append((seq[i], seq[i + 1]))
    return segs


def hardware():
    out = {}
    for ring, z0 in (("curl", Z_CURL), ("base", Z_BASE)):
        out[f"servo_{ring}"] = box([-SL / 2, -SH / 2, z0 + 0.05], [SL / 2, SH / 2, z0 + SW - 0.05])
        out[f"{ring}_spool"] = app(spool(), T(t=[SHAFT_X[ring], 0, SHAFT_Z[ring]]))
    for (which, side), (az, _, _, _) in HALVES.items():
        for k in (-1, 1): out[f"rod_{which}_{side}{'ab'[k > 0]}"] = cyl(1.5, 1.0, Z_TIE[1] - 0.5, c=(*rod_xy(az, k), 0), n=24)
    for name, c, din, dout in corners():
        ctr, ax = idler_pose(c, din, dout)
        sg = bracket(name)[1]
        out[f"idler_{name}"] = _cyl_axis(IDLER_R, -IDLER_W / 2, IDLER_W / 2, ctr, ax) + _cyl_axis(1.5, -IDLER_W / 2 - 0.3, IDLER_W / 2 + 3.3, ctr, sg * ax, n=16)
    out["motor_base"] = cyl(17, Z["pack1"], Z["pack1"] + 8)
    return out


RHO = dict(pacf=1.25, alu=2.70, steel=7.85, cfrp=1.55)
YOKES = [("out", "near"), ("out", "far"), ("in", "near"), ("in", "far")]


def parts():
    d = dict(cage=(cage(), "pacf", True), ring_in=(ring_in(), "pacf", True), ring_out=(ring_out(), "pacf", True), frame=(frame(), "pacf", False))
    for w, s in YOKES: d[f"yoke_{w}_{s}"] = (yoke_half(w, s), "pacf", False)
    d["spool"] = (spool(), "pacf", False); d["tie_ring"] = (tie_ring(), "pacf", False)
    return d


_PC = {}
def _parts():
    if not _PC: _PC.update(parts())
    return _PC


def assembly(pull_b=0.0, pull_c=0.0, lag=(0, 0, 0, 0), fold=((0, 0, 0),) * 4, spin=0.0):
    P = _parts(); S = T(rot("z", spin)); R = C.parts(); out = {}
    out["cage"] = (app(P["cage"][0], S), True)
    out["ring_out"] = (app(P["ring_out"][0], S @ T(t=[0, 0, -pull_b])), True); out["ring_in"] = (app(P["ring_in"][0], S @ T(t=[0, 0, -pull_c])), True)
    for w, s in YOKES:
        pull = pull_b if w == "out" else pull_c
        out[f"yoke_{w}_{s}"] = (app(P[f"yoke_{w}_{s}"][0], T(t=[0, 0, -(pull + GAP) if pull > 0 else 0.0])), False)
    out["frame"] = (P["frame"][0], False); out["tie_ring"] = (P["tie_ring"][0], False)
    for n, m in hardware().items(): out["hw_" + n] = (m, False)
    cords = {}
    for n, (p, q) in cord_runs(pull_b, pull_c).items(): cords.setdefault(n.rstrip("0123456789"), []).append(_seg(p, q))
    for n, segs in cords.items(): out[n] = (M_.batch_boolean(segs, mf.OpType.Add), False)      # one solid per cord (its runs meet at the idlers)
    out["hw_motor_bell"] = (app(cyl(17, Z["pack1"] + 8.5, Z["motor_top"]) + cyl(2.5, Z["motor_top"], Z["shaft_tip"]), S), True)
    for n in ("hub_lower", "hub_upper", "heel"): out[n] = (app(R[n], S), True)
    for k in range(4):
        Fr = C.finger_frames(k, lag[k], fold[k])
        out[f"grip{k}"] = (app(R["root_grip"], S @ Fr[0]), True); out[f"tube{k}"] = (app(R["lag_tube"], S @ T(t=[0, 0, C.ZB]) @ T(rot("z", C.FAZ[k]))), True)
        for j in range(3):
            out[f"ph{k}_{j + 1}"] = (app(R[f"phalanx{j + 1}"], S @ Fr[j + 1]), True); out[f"pad{k}_{j + 1}"] = (app(R[f"pad{j + 1}"], S @ Fr[j + 1]), True)
    return out


DRIVE = ("cage", "ring_out", "ring_in", "frame", "tie_ring") + tuple(f"yoke_{w}_{s}" for w, s in YOKES)


def _allowed(a, b):
    """Pairs that touch by design: a cord at its own spool / idlers / yoke eye; rods in their sockets and bosses; servos in the
    frame; axles in their brackets; spools on their servo; the motor's own base / bell (bought part, its own 0.5 mm gap)."""
    x, y = sorted((a, b))
    if x.startswith("cord") or y.startswith("cord"):                                 # a cord touches only its own spool, idlers, eye and runs
        cn = x if x.startswith("cord") else y; o = y if cn == x else x
        ring, side = cn.split("_")[1], cn.split("_")[2].rstrip("0123456789")
        if o.startswith("cord"): return False
        if o == f"hw_{ring}_spool" or o.startswith(f"hw_idler_{ring}_{side}"): return True
        return o == f"yoke_{'out' if ring == 'base' else 'in'}_{side}"
    if "hw_rod" in (x[:6], y[:6]) or x.startswith("hw_rod") or y.startswith("hw_rod"):
        o = y if x.startswith("hw_rod") else x
        return o in ("frame", "tie_ring") or o.startswith("yoke_")
    if {x, y} & {"frame"} and ({x, y} - {"frame"}).pop().startswith(("hw_servo", "hw_motor_base")): return True
    if x.startswith("hw_") and y.startswith("hw_") and ("spool" in x or "spool" in y) and ("servo" in x or "servo" in y): return True
    if {x, y} == {"hw_motor_base", "hw_motor_bell"}: return True
    return (x, y) in SKIP or (y, x) in SKIP


SKIP = {("cage", "ring_in")}


def overlaps(a, skip=()):
    """Overlap volume (mm3) of every pair that is not touching by design, and the smallest rotating-to-stationary gap."""
    names = list(a); bad = {}; gap = 9e9; gpair = None
    bb = {n: np.asarray(a[n][0].bounding_box()) for n in names}
    for i, n in enumerate(names):
        for m in names[i + 1:]:
            fin = lambda q: q.startswith(("ph", "pad", "grip", "tube"))
            if a[n][1] and a[m][1]: continue                                      # rotor parts among themselves: hand3_cad_checks
            if (n, m) in skip or (m, n) in skip or _allowed(n, m): continue
            b1, b2 = bb[n], bb[m]
            sep = max(max(b1[k] - b2[k + 3], b2[k] - b1[k + 3]) for k in range(3))
            if sep > 3.0: continue
            v = (a[n][0] ^ a[m][0]).volume()
            if v > 0.01: bad[f"{n}|{m}"] = round(float(v), 2)
            elif a[n][1] != a[m][1]:
                g = a[n][0].min_gap(a[m][0], 3.0)
                if g < gap: gap, gpair = float(g), f"{n}|{m}"
    return bad, round(gap, 2), gpair


def stack_margin():
    """How far each ring could still travel at full stroke before it touches a stationary part (bisection)."""
    out = {}
    for nm, kw, ring in (("base_full", dict(pull_b=TRAVEL_B), "ring_out"), ("curl_full", dict(pull_c=TRAVEL_C), "ring_in")):
        a = assembly(**kw); r = a[ring][0]
        fixed = [m for n, (m, rot_) in a.items() if not rot_ and not n.startswith(("cord", "yoke_")) and n != "hw_motor_bell"]
        lo, hi = 0.0, 15.0
        hit = lambda d: any((r.translate([0, 0, -d]) ^ f).volume() > 0.01 for f in fixed)
        if hit(lo): out[ring] = 0.0; continue
        for _ in range(16):
            c = (lo + hi) / 2
            if hit(c): hi = c
            else: lo = c
        out[ring] = round(lo, 2)
    return out




# ====================================================================================================================
# CORD ROUTING, ANCHORS, KEYS, LINKS, TRIM (ported from alt_cord/hand3_cords.py onto the hub C2 geometry; heights relative
# to H.Z["hub_lo"], which moves with STACK_DZ).  These definitions replace hub C2's cage / ring_in / ring_out.
# ====================================================================================================================
ZB, RL, FAZ, LS = C.ZB, H.R_LAG, C.FAZ, H.LAG_SIGN
HL = Z["hub_lo"]                              # 91 with STACK_DZ 4 (87 in hub C / alt_cord)
RC = 0.3                                      # 0.6 mm Dyneema (rotating cords)
Z_CORD = 6.0                                  # base cord height above the blade plane (above the upper hub plate)
HOLE_DEG, HOLE_D = 150.0, 1.0                 # base-cord side hole in the lag tube, deg from the finger's radial line on the LAG_SIGN side
R_WRAP = H.LAG_TUBE[0] / 2 + 0.3             # 2.8 mm: the base cord's arm about the lag axis (= E_LAG)
K1_TOP = np.array([H.KN[0], H.CORD_Y[0] * np.cos(np.radians(H.PITCH)) - H.LEVER[0] * np.sin(np.radians(H.PITCH))])
W_PT = np.array([RL, 0.0])


def upper_path(k, lag_deg):
    """Base cord, finger hub frame (x, y): side hole -> wrap round the fixed lag tube (radius R_WRAP) -> tangent -> K1 sector top."""
    s = LS[k]; th = np.radians(lag_deg)
    Rz = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); S = W_PT + Rz @ (K1_TOP - W_PT)
    d = S - W_PT; L = np.linalg.norm(d); beta = np.arctan2(d[1], d[0]); g = np.arccos(R_WRAP / L)
    at = beta + s * g; ah = np.radians(s * HOLE_DEG); da = at - ah
    arc_ = [W_PT + R_WRAP * np.array([np.cos(ah + f * da), np.sin(ah + f * da)]) for f in np.linspace(0, 1, 13)]
    return np.array(arc_ + [S]), S


def path_len(p): return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def spread_arm(k, lag_deg):
    """Moment arm of the base cord about the lag axis (mm), positive = spreading."""
    h = 0.5; L1 = path_len(upper_path(k, lag_deg - h)[0]); L2 = path_len(upper_path(k, lag_deg + h)[0])
    return -(L2 - L1) / np.radians(2 * h) * LS[k]


TUBE_WIN = (-1.25, 1.25)                       # blade-plane window (z from the blade plane): curl cord + 3 return cords stacked on the lag axis (was -0.8..0.8)


def lag_tube2(k):
    """Lag tube for finger k (finger frame): blade-plane window for the curl cord; Ø1 side hole on the LAG_SIGN side for the base
    cord (two handed versions: L = LAG_SIGN -1, fingers 0 / 2; R = +1, fingers 1 / 3)."""
    t = cyl(H.LAG_TUBE[0] / 2, -5.0, 7.5, c=(RL, 0, 0)) - cyl(H.LAG_TUBE[1] / 2, -6, 8, c=(RL, 0, 0))
    t -= box([RL, -3, TUBE_WIN[0]], [RL + 3.5, 3, TUBE_WIN[1]])
    t -= app(cyl(HOLE_D / 2, 0, 4.0, "x", (0, 0, 0), n=20), T(t=[RL, 0, Z_CORD]) @ T(rot("z", LS[k] * HOLE_DEG)))
    return t


LINK_AZ = 12.5                                # deg from the finger: base at -LINK_AZ, curl at +LINK_AZ
R_LINK = 19.6
LINK = dict(t=2.1, strand_w=dict(base=1.1, curl=1.0), gap=5.0, block=1.5, active=5.0, top_dz=-1.0)   # link top 1 mm under the hub plate
E_TPU = 30.0
LUG = dict(dz=(-13.0, -9.0), r=(18.3, 21.5))  # lug block under each link (at rest), relative to hub_lo
SCREW = dict(d=2.0, head_d=3.8, head_h=1.3, L=8.0, tap=1.6)
NUT = dict(af=4.0, h=1.6)
CAGE_SLOT_AZ = (-34.0, 27.0)
CAGE_SLOT_DZ = -30.0                          # slot bottom relative to hub_lo
HANGER_W = 2.0


def lug_z(): return (HL + LUG["dz"][0], HL + LUG["dz"][1])
def link_top(): return HL + LINK["top_dz"]
def adeg(mm, r=R_LINK): return float(np.degrees(mm / r))
def link_width(role): return 2 * LINK["strand_w"][role] + LINK["gap"]
def link_k(role): return E_TPU * 2 * LINK["strand_w"][role] * LINK["t"] / LINK["active"]
def link_az(role): return -LINK_AZ if role == "base" else LINK_AZ


def _wedge_az(r0, r1, z0, z1, a0, a1, n=96):
    w = cyl(r1, z0, z1, n=n) - cyl(r0, z0 - 1, z1 + 1, n=n)
    return C.half(C.half(w, [-np.sin(np.radians(a0)), np.cos(np.radians(a0)), 0], [0, 0, 0]), [np.sin(np.radians(a1)), -np.cos(np.radians(a1)), 0], [0, 0, 0])


def cage():
    """Cage (turns with the rotor, on the lower hub plate): one wide slot per finger, open at the top, for the two links, the
    curl lug and the base hanger (they key both rings to the cage)."""
    m = cyl(R_CAGE[1], HL - 31.5, HL, n=96) - cyl(R_CAGE[0], HL - 40, HL + 3, n=96)
    a0, a1 = CAGE_SLOT_AZ
    for a in FAZ: m -= _wedge_az(R_CAGE[0] - 1, R_CAGE[1] + 1, HL + CAGE_SLOT_DZ, HL + 3, a + a0, a + a1)
    return m


def _lug(role, a_f):
    az = a_f + link_az(role); w = link_width(role); hw = adeg(w / 2 + 0.3)
    a0, a1 = az - hw, az + hw
    if role == "base": a0 = az - adeg(w / 2 + 0.5 + HANGER_W)
    z0, z1 = lug_z()
    m = _wedge_az(LUG["r"][0], LUG["r"][1] if role == "base" else R_IN[0] + 0.3, z0, z1, a0, a1)
    return m - app(cyl(SCREW["tap"] / 2, z0 - 1, z1 + 1, c=(R_LINK, 0, 0), n=20), T(rot("z", az)))


def _hanger_az(a): return a + link_az("base") - adeg(link_width("base") / 2 + 0.5 + HANGER_W / 2)


def ring_in():
    m = cyl(R_IN[1], *Z_IN, n=96) - cyl(R_IN[0], Z_IN[0] - 1, Z_IN[1] + 1, n=96)
    m += cyl(R_FL_IN, *Z_FL_IN, n=96) - cyl(R_IN[0], Z_FL_IN[0] - 1, Z_FL_IN[1] + 1, n=96)
    for a in FAZ:
        m += _lug("curl", a)
        ab = _hanger_az(a); hb = adeg(HANGER_W / 2 + 0.6, 22.9)
        m -= _wedge_az(R_IN[0] - 1, R_IN[1] + 1, Z_FL_IN[1] + 0.5, HL + 20, ab - hb, ab + hb)     # passage for the base hanger
    return m


def ring_out():
    m = cyl(R_OUT[1], *Z_OUT, n=96) - cyl(R_OUT[0], Z_OUT[0] - 1, Z_OUT[1] + 1, n=96)
    m += cyl(R_FL_OUT, *Z_FL_OUT, n=96) - cyl(R_OUT[0], Z_FL_OUT[0] - 1, Z_FL_OUT[1] + 1, n=96)
    z0, _ = lug_z()
    for a in FAZ:
        ab = _hanger_az(a); hw = adeg(HANGER_W / 2)
        m += _wedge_az(LUG["r"][0], R_OUT[0] + 0.3, Z_OUT[0], Z_OUT[1], ab - adeg(HANGER_W / 2, 22.0), ab + adeg(HANGER_W / 2, 22.0))   # bridge
        m += _wedge_az(LUG["r"][0], LUG["r"][1], z0, Z_OUT[1], ab - hw, ab + hw) + _lug("base", a)                                 # leg + lug
    return m


def link(role):
    """TPU series link (own frame: x tangential, y radial, z up); bottom block slides on the M2 trim screw, cord knotted in the top."""
    sw, g, t, b, act = LINK["strand_w"][role], LINK["gap"], LINK["t"], LINK["block"], LINK["active"]
    w = 2 * sw + g; H_ = 2 * b + act
    m = box([-w / 2, -t / 2, 0], [w / 2, t / 2, H_]) - box([-g / 2, -t / 2 - 1, b], [g / 2, t / 2 + 1, b + act])
    m -= cyl(SCREW["d"] / 2 + 0.15, -1, b + 1, n=20)
    m -= cyl(0.5, -t, t, "y", (0, 0, H_ - b / 2), n=16)
    return m


def link_placed(role, a_f, drop=0.0):
    H_ = 2 * LINK["block"] + LINK["active"]; z0 = link_top() - H_
    return app(link(role), T(rot("z", a_f + link_az(role)), [0, 0, -drop]) @ T(t=[R_LINK, 0, z0]) @ T(rot("z", 90)))


def trim_hw(role, a_f, drop=0.0):
    az = a_f + link_az(role); A = T(rot("z", az), [0, 0, -drop])
    z0 = lug_z()[0]; H_ = 2 * LINK["block"] + LINK["active"]; zb = link_top() - H_ + LINK["block"]
    screw = cyl(SCREW["head_d"] / 2, z0 - SCREW["head_h"], z0 - 0.01, c=(R_LINK, 0, 0), n=20) + cyl(SCREW["tap"] / 2, z0 - 0.01, z0 + SCREW["L"], c=(R_LINK, 0, 0), n=20)      # shank drawn at the thread minor (tap) diameter: the thread cuts into the tapped lug
    nut = app(cyl(NUT["af"] / 2 / np.cos(np.pi / 6), zb + 0.01, zb + NUT["h"], n=6) - cyl(SCREW["tap"] / 2 + 0.01, zb - 1, zb + NUT["h"] + 1, n=20), T(t=[R_LINK, 0, 0]) @ T(rot("z", 30)))
    return app(screw, A), app(nut, A)


def _seg_r(p, q, r=RC): return _seg(p, q, r)


def lower_cord(role, a_f, drop):
    p = rot("z", a_f) @ np.array([RL, np.sign(link_az(role)) * 0.75, ZB - 5.0 - RC - 0.1])     # leaves the bore just below the tube end (drawn straight)
    q = rot("z", a_f + link_az(role)) @ np.array([R_LINK, 0, link_top() + RC + 0.05 - drop])      # drawn to the link top; the knot through the Ø1 hole is not drawn
    return p, q


def upper_cord_solid(k, lag_deg):
    p, S = upper_path(k, lag_deg); pts = [pp for pp in p]
    a, b = pts[-2], pts[-1]; d = b - a; Ld = np.linalg.norm(d); pts[-1] = a + d * max(0.05, (Ld - 5.0) / Ld)
    A = rot("z", FAZ[k]); m = None
    for i in range(len(pts) - 1):
        s_ = _seg_r(A @ np.r_[pts[i], ZB + Z_CORD], A @ np.r_[pts[i + 1], ZB + Z_CORD]); m = s_ if m is None else m + s_
    return m


# --- empty-hand curl limit (fingertips vs the upper hub plate with the base ring at full stroke) -------------------
# With every knuckle at its stop (90/90/85) and the fingers spread 47 deg, phalanx 3 + pad reach the upper hub plate.  On an
# object the fingers stop at contact long before; with an empty hand the curl ring must stop short.  Firmware limit: curl
# ring travel <= CURL_LIMIT_EMPTY when the base ring is past BASE_FOLDED (K1 > ~80 deg).  Value set by curl_limit() below.
CURL_LIMIT_EMPTY = 1.79                       # mm: recomputed by __main__ (curl_limit()); it warns if the geometry moved
BASE_FOLDED = None


def _curl_fold(stroke):
    """K2, K3 angles for a curl-ring stroke on an empty hand (staging: K2 first, then K3; link stretch not taken up)."""
    L1, L2 = LEVER[1], LEVER[2]; a2 = min(H.FOLD_MAX[1], np.degrees(stroke / L1))
    rest = stroke - L1 * np.radians(a2); a3 = min(H.FOLD_MAX[2], np.degrees(max(rest, 0) / L2))
    return float(a2), float(a3)


def tip_vs_hub(stroke, k1=90.0, lag=None, gap=0.0):
    """Overlap (mm3) of the folded fingers with the hub plates / heel and with each other on an empty close: K1 at k1, the curl
    ring at 'stroke'.  gap > 0 grows every finger part by that much (a clearance check)."""
    R = C.parts(); a2, a3 = _curl_fold(stroke); lag = lag if lag is not None else [s * H.SPREAD_MAX for s in LS]
    hub = R["hub_upper"] + R["hub_lower"] + R["heel"]; v = 0.0; F = []
    for k in range(4):
        Fr = C.finger_frames(k, lag[k], (k1, a2, a3))
        parts_k = [app(R[f"phalanx{j}"] + R[f"pad{j}"], Fr[j]) for j in (1, 2, 3)]
        if gap > 0: parts_k = [p.offset(gap, circular_segments=8) if hasattr(p, "offset") else p for p in parts_k]
        F.append(parts_k)
        for p in parts_k[1:]: v += (p ^ hub).volume()
    for i in range(4):
        for k in range(i + 1, 4):
            for x in F[i]:
                for y in F[k]: v += (x ^ y).volume()
    return v


def curl_limit(clear=0.5):
    """Largest curl stroke (ring travel, mm) with K1 at 90 deg and full spread that keeps the finger 'clear' mm off the hub."""
    lo, hi = 0.0, TRAVEL_C - STRETCH
    if tip_vs_hub(hi) <= 0.01: return hi
    if tip_vs_hub(lo) > 0.01: return 0.0
    for _ in range(14):
        c = (lo + hi) / 2
        if tip_vs_hub(c) > 0.01: hi = c
        else: lo = c
    return round(max(lo - clear, 0.0), 2)                      # 'clear' mm of ring travel short of first contact


def k1_limit_full_curl(clear=2.0):
    """Largest K1 angle (deg) at full spread with a full empty curl (K2 90, K3 85) that keeps the fingers off the hub and off
    each other; 'clear' deg short of first contact."""
    full = TRAVEL_C - STRETCH; lo, hi = 0.0, 90.0
    if tip_vs_hub(full, k1=hi) <= 0.01: return hi
    for _ in range(12):
        c = (lo + hi) / 2
        if tip_vs_hub(full, k1=c) > 0.01: hi = c
        else: lo = c
    return round(lo - clear, 1)


def base_travel(k1):
    """Base ring travel (mm, empty hand: spread first to 47 deg, then K1 to k1; link stretch not taken up)."""
    return round(float(E_LAG * np.radians(H.SPREAD_MAX) + LEVER[0] * np.radians(k1)), 2)


# --- parts / assembly (replace hub C2's) ----------------------------------------------------------------------------
_parts_c2 = parts


def parts():
    d = _parts_c2()
    d["cage"] = (cage(), "pacf", True); d["ring_in"] = (ring_in(), "pacf", True); d["ring_out"] = (ring_out(), "pacf", True)
    d["link_base"] = (link("base"), "tpu", True); d["link_curl"] = (link("curl"), "tpu", True)
    return d


_assembly_c2 = assembly


def assembly(pull_b=0.0, pull_c=0.0, lag=(0, 0, 0, 0), fold=((0, 0, 0),) * 4, spin=0.0):
    if not _PC: _PC.update(parts())
    a = _assembly_c2(pull_b, pull_c, lag, fold, spin)
    S = T(rot("z", spin))
    for k in range(4):
        a[f"tube{k}"] = (app(lag_tube2(k), S @ T(t=[0, 0, ZB]) @ T(rot("z", FAZ[k]))), True)
    for k, f in enumerate(FAZ):
        for role, dr in (("base", pull_b), ("curl", pull_c)):
            a[f"rlink_{role}{k}"] = (app(link_placed(role, f, dr), S), True)
            sc, nu = trim_hw(role, f, dr); a[f"hw_rscrew_{role}{k}"] = (app(sc, S), True); a[f"hw_rnut_{role}{k}"] = (app(nu, S), True)
            p, q = lower_cord(role, f, dr); a[f"rcordlo_{role}{k}"] = (app(_seg_r(p, q), S), True)
        a[f"rcordup{k}"] = (app(upper_cord_solid(k, lag[k]), S), True)
    R = C.parts()
    for k in range(4):                                          # return-spring boxes (hand3_springs.py): handed by LAG_SIGN, placed like the tube
        hand = "R" if LS[k] > 0 else "L"; A = S @ T(rot("z", FAZ[k]))
        for nm in ("tray1", "tray2", "lid"):
            if f"spring_{nm}_{hand}" in R: a[f"s{nm}{k}"] = (app(R[f"spring_{nm}_{hand}"], A), True)
        if f"spring_lid_{hand}" in R:                           # M2 x 12 from the lid top into the tapped upper plate (2 mm of thread)
            sr, zt = C.SPRING_SCREW["r"], ZB + SPRING_LID_TOP
            scr = cyl(0.8, zt - 12.0, zt, c=(sr, 0, 0), n=20) + cyl(1.9, zt, zt + 1.3, c=(sr, 0, 0), n=24)
            a[f"hw_sscrew{k}"] = (app(scr, A), True)
    return a


NEW_ROT = ("cage", "ring_in", "ring_out", "hub_upper", "rlink_", "hw_rscrew_", "hw_rnut_", "rcordlo_", "rcordup", "tube", "stray", "slid", "hw_sscrew")


def check_rotating(a, ignore_fingers=False):
    """Overlaps among rotating parts that the drive check skips (rings, cage, links, trim hardware, cords vs each other and the
    hub / fingers); intended contacts excluded (link on its screw and nut, screw in its lug, cord in its link / tube bore)."""
    names = list(a); new = [n for n in names if n.startswith(NEW_ROT) and a[n][1]]
    rot_ = [n for n in names if a[n][1]]; bad = {}
    for i, n in enumerate(new):
        for m in rot_:
            if m == n or (m in new and new.index(m) <= i): continue
            if ignore_fingers and m.startswith(("ph", "pad", "grip")): continue
            pair = {n, m}; role = "base" if ("base" in n or "base" in m) else ("curl" if ("curl" in n or "curl" in m) else None)
            k = "".join(ch for ch in (n + m) if ch.isdigit())[-1:] if any(ch.isdigit() for ch in n + m) else ""
            if role and pair <= {f"rlink_{role}{k}", f"hw_rscrew_{role}{k}", f"hw_rnut_{role}{k}", f"rcordlo_{role}{k}"}: continue
            if pair in ({f"hw_rscrew_curl{k}", "ring_in"}, {f"hw_rscrew_base{k}", "ring_out"}): continue
            if (n.startswith("rcord") and m.startswith("tube")) or (m.startswith("rcord") and n.startswith("tube")): continue
            if n.startswith("rcordup") and m.startswith("grip"): continue        # the cord's free span ends on the grip-side K1 sector
            if pair == {"cage", "ring_in"} or pair == {"tube" + k, "hub_upper"} or pair == {"tube" + k, "hub_lower"}: continue
            if pair <= {f"hw_sscrew{k}", f"stray1{k}", f"stray2{k}", f"slid{k}", "hub_upper"} and f"hw_sscrew{k}" in pair: continue   # box screw: in its boss / tapped plate
            if pair == {f"stray1{k}", f"tube{k}"}: continue          # tray 1 sits on the tube end (bore fit)
            if n.startswith("tube") and m.startswith(("grip", "hub_")): continue
            if m.startswith("tube") and n.startswith(("grip", "hub_")): continue
            v = (a[n][0] ^ a[m][0]).volume()
            if v > 0.01: bad[f"{n}|{m}"] = round(float(v), 3)
    return bad


def _allowed_e(a, b):
    """hub C2's allowed contacts + the rotating trim hardware / cords (they are rotating, so the drive check never pairs them
    with each other; against stationary parts they must clear)."""
    return _allowed_c2(a, b)


_allowed_c2 = _allowed
# contacts that are there by design (threads and knots): trim screw in its tapped ring lug, M2 nut on its screw, cord knotted
# through its link's top block.  Declared here so every check (this file's and the test harness) skips exactly these pairs.
for _k in range(4):
    for _r, _ring in (("base", "ring_out"), ("curl", "ring_in")):
        SKIP |= {(_ring, f"hw_rscrew_{_r}{_k}"), (f"hw_rscrew_{_r}{_k}", f"hw_rnut_{_r}{_k}"), (f"rlink_{_r}{_k}", f"rcordlo_{_r}{_k}"),
                 (f"rlink_{_r}{_k}", f"hw_rscrew_{_r}{_k}"), (f"rlink_{_r}{_k}", f"hw_rnut_{_r}{_k}")}
def extra_poses():
    """Poses the test harness adds to its finger checks: the empty-hand close (base ring full, fingers spread 47 deg) with the
    curl ring at its firmware limit CURL_LIMIT_EMPTY (K2 90 deg, K3 what the remaining stroke gives)."""
    cl = curl_limit() if CURL_LIMIT_EMPTY is None else CURL_LIMIT_EMPTY
    k1 = k1_limit_full_curl() if K1_LIMIT is None else K1_LIMIT
    lagS = [s * H.SPREAD_MAX for s in H.LAG_SIGN]; fc = TRAVEL_C - STRETCH
    return [("empty close: base full, curl at its limit", dict(pull_b=TRAVEL_B, pull_c=cl, lag=lagS, fold=((90.0,) + _curl_fold(cl),) * 4)),
            ("empty close: full curl, base at its limit", dict(pull_b=base_travel(k1), pull_c=TRAVEL_C, lag=lagS, fold=((k1,) + _curl_fold(fc),) * 4))]


K1_LIMIT = 45.1                               # deg: recomputed by __main__ (k1_limit_full_curl())


ROD_MATERIAL = "cfrp"                          # the eight Ø3 guide rods are pultruded carbon (read by the test harness)


def mass_table():
    """Mass and COM.  Uses the test harness's own mass check when it is next door (../tests/hand3_tests.py) so the design file and
    the harness agree (PA12 phalanges, AlSi10Mg grips / plates, pins + bushings, carbon rods, all modelled hardware + allowances)."""
    import importlib.util, sys as _sys
    here = os.path.dirname(os.path.abspath(__file__)); hp = os.path.join(here, "..", "tests", "hand3_tests.py")
    if not os.path.exists(hp): return None
    spec = importlib.util.spec_from_file_location("hand3_tests_h", hp); T_ = importlib.util.module_from_spec(spec)
    old = list(_sys.argv); _sys.argv = [hp, here]
    try:
        spec.loader.exec_module(T_); D = T_.load(here) if hasattr(T_, "load") else None
        if D is None: return None
        rows = T_.t8_mass(D, {"tol": 0.02})
    finally:
        _sys.argv = old
    tot = rows[0]["data"]; com = rows[1]["data"]["com"]
    return dict(total=round(tot["total"], 1), by_group={k: round(v, 1) for k, v in tot["by_group"].items()}), round(float(com[2]), 1)



try:
    import hand3_springs as _SP
    SPRING_LID_TOP = _SP.tray_z(2)[2] + _SP.BOX['lid']; assert tuple(_SP.BOX['screw']) == (C.SPRING_SCREW['r'], 0.0)
except ImportError:
    SPRING_LID_TOP = None
for _k in range(4):                             # spring-box screw: threaded into the tapped upper plate, passes its boss / lid holes
    SKIP |= {("hub_upper", f"hw_sscrew{_k}"), (f"stray1{_k}", f"hw_sscrew{_k}"), (f"stray2{_k}", f"hw_sscrew{_k}"), (f"slid{_k}", f"hw_sscrew{_k}"), (f"stray1{_k}", f"tube{_k}")}
_springs_hooked = True                          # hand3_springs.py: tube window and spring boxes are wired here, no monkey-patch

if __name__ == "__main__":
    sz = sizing(); P = parts(); mass = {n: round(float(m.volume()) * RHO.get(mat, 1.2) / 1000, 1) for n, (m, mat, _) in P.items()}
    lagS = [s * H.SPREAD_MAX for s in H.LAG_SIGN]
    _cl, _k1 = curl_limit(), k1_limit_full_curl()
    if abs(_cl - CURL_LIMIT_EMPTY) > 0.01 or abs(_k1 - K1_LIMIT) > 0.1:
        print(f"WARNING: empty-hand interlock moved: curl {_cl} (file {CURL_LIMIT_EMPTY}), K1 {_k1} (file {K1_LIMIT}): update the constants")
    CURL_LIMIT_EMPTY, K1_LIMIT = _cl, _k1; globals().update(CURL_LIMIT_EMPTY=_cl, K1_LIMIT=_k1); BASE_FOLDED = base_travel(K1_LIMIT)
    chk = {}
    states = (("flight", {}), ("base_full", dict(pull_b=TRAVEL_B, lag=lagS, fold=((90, 0, 0),) * 4)),
              ("curl_full", dict(pull_c=TRAVEL_C, fold=((0, 90, 85),) * 4)),
              ("grasp", dict(pull_b=TRAVEL_B, pull_c=TRAVEL_C * 0.75, lag=lagS, fold=((55, 60, 50),) * 4)),
              ("both_full", dict(pull_b=TRAVEL_B, pull_c=TRAVEL_C, lag=lagS, fold=((90, 90, 85),) * 4)),
              ("half_spread", dict(pull_b=TRAVEL_B * 0.3, lag=[s * 23.5 for s in H.LAG_SIGN])),
              ) + tuple((nm.replace(" ", "_").replace(",", "").replace(":", ""), kw) for nm, kw in extra_poses())
    for nm, kw in states:
        a = assembly(**kw)
        fing = nm in ("base_full", "curl_full", "both_full")          # unreal finger poses for the drive check (fingers alone)
        ad = {k: v for k, v in a.items() if not k.startswith(("ph", "pad", "grip"))} if fing else a
        bad, g, gp = overlaps(ad, SKIP); rb = check_rotating(a, ignore_fingers=(nm in ("base_full", "curl_full", "both_full")))
        chk[nm] = dict(overlaps=bad, rotating_overlaps=rb, min_gap_rot_stat=g, at=gp)
    spin = 9e9; sp_at = None
    for s in np.arange(0, 90, 3.75):
        a = assembly(spin=s); bad, g, gp = overlaps(a, SKIP)
        if bad: spin = -1; chk["spin_overlap"] = bad; break
        if g < spin: spin, sp_at = g, gp
    chk["flight_spin_min_gap"] = spin; chk["flight_spin_at"] = sp_at
    chk["stack_margin"] = stack_margin()
    chk["bodies"] = {n: len(m.decompose()) for n, (m, _, _) in P.items()}
    chk["lag_tube_bodies"] = [len(lag_tube2(k).decompose()) for k in range(4)]
    hwb = hardware(); chk["hardware_empty"] = [n for n, m in hwb.items() if m.is_empty() or m.volume() <= 0]
    chk["tip_vs_hub_full_curl_mm3"] = round(tip_vs_hub(TRAVEL_C - STRETCH), 2); chk["tip_vs_hub_at_limit_mm3"] = round(tip_vs_hub(CURL_LIMIT_EMPTY), 3)
    chk["spread_arm_mm"] = {k: [round(spread_arm(k, LS[k] * a_), 2) for a_ in (0, 23.5, 47)] for k in range(4)}
    vtx = np.vstack([np.asarray(m.to_mesh().vert_properties)[:, :3] for m in [P["frame"][0], P["tie_ring"][0]] + [P[f"yoke_{w}_{s}"][0] for w, s in YOKES] + list(hwb.values())])
    env = dict(r_max=round(float(np.hypot(vtx[:, 0], vtx[:, 1]).max()), 2), d_max=round(2 * float(np.hypot(vtx[:, 0], vtx[:, 1]).max()), 1),
               block=f"{SL + 2 * WALL_T:.1f} x {2 * Y_HALF:.0f} mm frame, {Z_SERVO_TOP + BRIDGE_T:.1f} mm to the bridge top; motor mount at z {Z['pack1']:.1f}")
    mh = mass_table()
    tot = dict(total=mh[0]["total"], **mh[0]["by_group"]) if mh else {}
    com = mh[1] if mh else None
    links = {r: dict(k_N_per_mm=round(link_k(r), 1), stretch_at_peak_mm=round(sz[r]["F_peak_need"] / link_k(r), 2)) for r in ("base", "curl")}
    out = dict(servo=SERVO, spool_r=SPOOL_R, levers=LEVER, e_lag=E_LAG, lag_spring_Nm=LAG_SPRING, ext_k1_Nm=EXT_K1, ext_k2_Nm=EXT_K2, ext_k3_Nm=EXT_K3, stretch=STRETCH,
               stack_dz=H.STACK_DZ, travel_b=round(float(TRAVEL_B), 2), travel_c=round(float(TRAVEL_C), 2), base=sz["base"], curl=sz["curl"], links=links,
               curl_limit_empty_mm=CURL_LIMIT_EMPTY, base_folded_mm=BASE_FOLDED,
               k1_limit_full_curl_deg=K1_LIMIT,
               empty_hand_rule=(f"firmware interlock (empty hand: the two rings may not both pass their limits): curl ring travel <= {CURL_LIMIT_EMPTY} mm "
                                f"while the base ring is past {BASE_FOLDED} mm (K1 > {K1_LIMIT} deg), and base ring travel <= {BASE_FOLDED} mm while the curl ring "
                                f"is past {CURL_LIMIT_EMPTY} mm.  On an object the fingers stop at contact first; the interlock only stops an empty fist from "
                                f"driving fingertips into the hub plate and into each other at stall force."),
               halves={f"{w}_{s}": dict(az=round(HALVES[(w, s)][0], 1), attach=[round(float(v), 2) for v in HALVES[(w, s)][1]]) for w, s in YOKES},
               z=dict(out=Z_OUT, fl_out=Z_FL_OUT, fl_in=Z_FL_IN, lip_out=Z_LIP_OUT, lip_in=Z_LIP_IN, servo_top=Z_SERVO_TOP, bridge_top=Z_SERVO_TOP + BRIDGE_T, blade=Z["blade"], hub_lo=HL),
               mass=tot, com_from_spigot=com, mass_source="tests/hand3_tests.py t8 (PA12 phalanges, AlSi10Mg grips + plates, pins + bushings, carbon rods, steel hardware, allowances)",
               part_mass=mass, checks=chk, envelope=env, rod_material=ROD_MATERIAL, electrical=dict(servo_stall_A=2 * SERVO["stall_A"], rail="12 V arm bus"))
    json.dump(out, open("hand3_hubc.json", "w"), indent=1, default=float)
    os.makedirs("hand3_hubc_stl", exist_ok=True)
    for n, (m, _, _) in P.items(): to_trimesh(m).export(f"hand3_hubc_stl/hubc_{n}.stl")
    for k, hand in ((0, "L"), (1, "R")): to_trimesh(lag_tube2(k)).export(f"hand3_hubc_stl/lag_tube_{hand}_x2_stainless.stl")
    print("sizing", json.dumps(sz)); print("mass", tot, "com", com); print("checks", json.dumps(chk, default=float)); print("envelope", env)
    print("empty-hand interlock: curl <=", CURL_LIMIT_EMPTY, "mm when base >", BASE_FOLDED, "mm (K1 >", K1_LIMIT, "deg)")
