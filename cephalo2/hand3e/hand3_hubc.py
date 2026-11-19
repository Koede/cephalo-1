"""hand3 hub E (2026-11-15, integrated): hub C2's drive + frame with the cord routing, ring anchors / keys, series links and
length trim, on one geometry.  Workspace copy of claude/code/hand3e/hand3_hubc.py: the geometry, parts and assembly() are the
project file's; its checks / mass table / __main__ (which need ../tests) are left in the project copy.
Two pull rings (base: spread + K1, curl: K2 + K3), each pulled by one STS3215 through a two-groove spool and MR63 idlers to two
yoke halves on Ø3 carbon rods; a tie ring clamps the rod tops.  Units mm.  Estimates."""
import os, json
import numpy as np
import manifold3d as mf
import hand3_design as H
import hand3_cad as C
from hand3_cad import cyl, box, app, T, rot, to_trimesh

M_ = mf.Manifold
Z = H.Z
SERVO = dict(name="Feetech STS3215", stall=2.94, rated=0.981, m=55.0, size=(45.2, 24.7, 35.0), volts="12 V", stall_A=2.7, deg=360.0, sec60=0.222)
SPOOL_R = 4.0
LEVER = list(H.LEVER)
E_LAG = 2.8
STRETCH = 2.0
LAG_SPRING, EXT_K1, EXT_K2, EXT_K3 = 0.0054, 0.02, 0.004, 0.016
TRAVEL_B = E_LAG * np.radians(H.SPREAD_MAX) + LEVER[0] * np.radians(H.FOLD_MAX[0]) + STRETCH
TRAVEL_C = LEVER[1] * np.radians(H.FOLD_MAX[1]) + LEVER[2] * np.radians(H.FOLD_MAX[2]) + STRETCH
GAP = 1.0
ETA_SPOOL, ETA_GUIDE, ETA_IDLER = 0.75, 0.80, 0.97
R_CAGE = (19.5, 21.5); R_IN = (21.9, 23.9); R_OUT = (24.3, 26.3); R_FL_OUT = 30.0; R_FL_IN = 32.5
R_LIP_OUT = (27.1, 30.0); R_LIP_IN = (30.5, 32.5); R_LEG = (33.5, 36.0); R_ROD = 37.6
ARM_CLR = 1.0
R_TIE = (36.3, 40.3)
LIP_HALF, LEG_HALF, ROD_DAZ = 18.0, 7.0, 10.0

Z_OUT = (Z["hub_lo"] - 7.0, Z["hub_lo"] - 4.0); Z_FL_OUT = (Z_OUT[0], Z_OUT[0] + 2.0); Z_LIP_OUT = (Z_FL_OUT[1] + GAP, Z_FL_OUT[1] + GAP + 2.0)
Z_FL_IN = (Z_OUT[0] - TRAVEL_B - GAP - 2.0, Z_OUT[0] - TRAVEL_B - GAP); Z_IN = (Z_FL_IN[0], Z_OUT[1])
Z_LIP_IN = (Z_FL_IN[1] + GAP, Z_FL_IN[1] + GAP + 2.0)
Z_TIE = (Z_LIP_OUT[0] - 7.0, Z_LIP_OUT[0] - 5.0)
LEG = dict(out=(18.0, 8.0), inn=(12.0, 1.0))
SL, SW, SH = SERVO["size"]
PLATE_T, BRIDGE_T, WALL_T = 2.0, 2.0, 2.0
Z_CURL = PLATE_T; Z_BASE = PLATE_T + SW; Z_SERVO_TOP = PLATE_T + 2 * SW
SHAFT_X = {"curl": SL / 2 - 10.6, "base": -(SL / 2 - 10.6)}
SHAFT_Z = {"curl": Z_CURL + SW / 2, "base": Z_BASE + SW / 2}
Y_HALF = SH / 2 + 0.5
SPOOL_Y = (Y_HALF, Y_HALF + 6.0)
Y1, Y2 = 19.8, 22.5
X_N, X_F = 29.5, 31.5
IDLER_R, IDLER_W, CORD_R = 3.0, 2.5, 0.35
Z_LOW = 8.0
X_DROP = 25.5


def _az(x, y): return float(np.degrees(np.arctan2(y, x)))


def routes():
    out = {}
    for ring in ("curl", "base"):
        xs, zs = SHAFT_X[ring], SHAFT_Z[ring]; s = 1.0 if ring == "curl" else -1.0
        zt, zb = zs + SPOOL_R, zs - SPOOL_R
        near_xy = (s * X_N, Y1)
        far_x = -s * X_F; far_xy = (far_x, -Y1)
        near = [np.array([xs, Y1, zt]), np.array([near_xy[0], Y1, zt])]
        if ring == "curl":
            far = [np.array([xs, Y2, zb]), np.array([far_x, Y2, zb]), np.array([far_x, -Y1, zb])]
        else:
            far = [np.array([xs, Y2, zb]), np.array([X_DROP, Y2, zb]), np.array([X_DROP, Y2, Z_LOW]), np.array([far_x, -Y1, Z_LOW])]
        out[ring] = dict(near=(_az(*near_xy), near_xy, near, zt), far=(_az(*far_xy), far_xy, far, zb))
    return out


ROUTES = routes()
HALVES = {("in", k): ROUTES["curl"][k] for k in ("near", "far")}
HALVES.update({("out", k): ROUTES["base"][k] for k in ("near", "far")})


def lip_z(which): return Z_LIP_OUT if which == "out" else Z_LIP_IN
def leg(which): return LEG["out" if which == "out" else "inn"]


def arc(r0, r1, z0, z1, az, half):
    m = cyl(r1, z0, z1, n=96) - cyl(r0, z0 - 1, z1 + 1, n=96)
    h = np.radians(half)
    m = C.half(C.half(m, np.array([np.sin(h), np.cos(h), 0.0]), [0, 0, 0]), np.array([np.sin(h), -np.cos(h), 0.0]), [0, 0, 0])
    return app(m, T(rot("z", az)))


def rod_xy(az, k): a = np.radians(az + k * ROD_DAZ); return np.array([R_ROD * np.cos(a), R_ROD * np.sin(a)])


def yoke_half(which, side):
    az, (ax, ay), _, _ = HALVES[(which, side)]
    lip_r = R_LIP_OUT if which == "out" else R_LIP_IN; lz = lip_z(which)
    fl_r = R_FL_OUT if which == "out" else R_FL_IN
    m = arc(lip_r[0], lip_r[1] + 1.0, lz[0], lz[1], az, LIP_HALF)
    m += arc(fl_r + ARM_CLR, R_LEG[1], lz[0] - 2.5, lz[0] + 0.5, az, LEG_HALF + 2.0)
    zb0, zb1 = lz[0] - leg(which)[0], lz[0] - leg(which)[1]
    m += arc(R_LEG[0], R_LEG[1], zb0, lz[0] + 0.5, az, LEG_HALF)
    bosses = [cyl(3.2, zb0, zb1, c=(*rod_xy(az, k), 0)) for k in (-1, 1)]
    eye = cyl(2.2, zb0, zb0 + 4.0, c=(ax, ay, 0))
    tab = M_.batch_hull(bosses + [app(box([R_LEG[0], -3.0, zb0], [R_LEG[0] + 1.0, 3.0, zb1]), T(rot("z", az))), eye])
    m += tab
    for k in (-1, 1): m -= cyl(1.6, zb0 - 1, zb1 + 1, c=(*rod_xy(az, k), 0))
    m -= cyl(0.6, zb0 - 1, zb0 + 5.0, c=(ax, ay, 0)) + cyl(1.4, zb0 + 2.0, zb0 + 5.0, c=(ax, ay, 0))
    return m


def tie_ring():
    m = cyl(R_TIE[1], *Z_TIE, n=96) - cyl(R_TIE[0], Z_TIE[0] - 1, Z_TIE[1] + 1, n=96)
    for (w, sd), (az, _, _, _) in HALVES.items():
        for k in (-1, 1): m -= cyl(1.52, Z_TIE[0] - 1, Z_TIE[1] - 0.5, c=(*rod_xy(az, k), 0), n=24)
    return m


def spool():
    y0, y1 = SPOOL_Y; fr, dr = SPOOL_R + 1.5, SPOOL_R - 0.35
    m = cyl(fr, y0, y0 + 0.6, "y") + cyl(dr, y0 + 0.6, y0 + 3.0, "y") + cyl(fr, y0 + 3.0, y0 + 3.6, "y") + cyl(dr, y0 + 3.6, y0 + 5.4, "y") + cyl(fr, y0 + 5.4, y1, "y")
    m -= cyl(2.95, y0 - 1, y0 + 2.5, "y", n=48) + cyl(1.7, y0 - 1, y1 + 1, "y", n=24)
    for yc in (Y1, Y2): m -= cyl(0.45, -fr - 1, fr + 1, "z", (0, yc, 0), n=12)
    return m


def corners():
    out = []
    for ring, R in ROUTES.items():
        for side in ("near", "far"):
            pts = R[side][2]; seq = list(pts) + [pts[-1] + np.array([0, 0, 10.0])]
            for i in range(1, len(seq) - 1):
                din = seq[i] - seq[i - 1]; dout = seq[i + 1] - seq[i]
                out.append((f"{ring}_{side}{i}", seq[i], din / np.linalg.norm(din), dout / np.linalg.norm(dout)))
    return out


def idler_pose(c, din, dout):
    ctr = c + (IDLER_R + CORD_R) * (dout - din)
    ax = np.cross(din, dout); ax = ax / np.linalg.norm(ax)
    return ctr, ax


def _cyl_axis(r, h0, h1, ctr, ax, n=32):
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
    xw = SL / 2 + WALL_T
    m = box([-xw, -Y_HALF, 0], [xw, Y_HALF, PLATE_T])
    m += box([-xw, -Y_HALF, 0], [-SL / 2, Y_HALF, Z_SERVO_TOP + BRIDGE_T]) + box([SL / 2, -Y_HALF, 0], [xw, Y_HALF, Z_SERVO_TOP + BRIDGE_T])
    m += box([-xw, -Y_HALF, Z_SERVO_TOP], [xw, Y_HALF, Z_SERVO_TOP + BRIDGE_T])
    m += cyl(20.0, Z_SERVO_TOP, Z["pack1"], n=64)
    for k in range(4):
        a = np.radians(45 + 90 * k); p = 19.0 / 2 * np.sqrt(2) * np.array([np.cos(a), np.sin(a)])
        m -= cyl(1.7, Z_SERVO_TOP - 1, Z["pack1"] + 1, c=(p[0], p[1], 0), n=24) + cyl(3.0, Z_SERVO_TOP - 1, Z_SERVO_TOP + 1.8, c=(p[0], p[1], 0), n=32)
    # coupler rev B.7 (2026-11-19, ../hubE_leads.py): the plate bolts to the coupler hand base's 4 M3 inserts (r 13), M3 x 8
    # DIN 7991 countersunk from inside.  No centre hole: the servos lie across the middle, so the motor leads and the signal
    # wires leave the hand base sideways and run up the back face (y = -Y_HALF) in a cover rail.  Plate notch for the leads'
    # dive, top-block notch for the motor's 18 AWG stubs, two bosses for the rail's M2 thread-forming screws.
    import sys as _sys
    _sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    import hubE_leads as HL
    for a in HL.P["hand_screw_az"]:
        ah = np.radians(180.0 - (a - HL.PSI)); p = HL.P["hand_screw_r"] * np.array([np.cos(ah), np.sin(ah)])
        m -= cyl(1.7, -1, 3, c=(p[0], p[1], 0), n=24) + M_.cylinder(1.7, 1.6, 3.2, 32).translate([p[0], p[1], 0.31])
    for (which, side), (az, _, _, _) in HALVES.items():
        for k in (-1, 1):
            q = rod_xy(az, k); u = q / np.linalg.norm(q)
            m += M_.batch_hull([cyl(3.0, 0, PLATE_T, c=(*q, 0)), cyl(3.0, 0, PLATE_T, c=(*(u * 15.0), 0))]) + cyl(3.0, 0, 6.0, c=(*q, 0))
            m -= cyl(1.52, 1.0, 7.0, c=(*q, 0), n=24)
    for name, c, din, dout in corners():
        m += bracket(name)[0]
    return m - HL.frame_cuts() + HL.frame_adds()


_BR = {}


def bracket(name):
    if name in _BR: return _BR[name]
    c, din, dout = next((cc, d1, d2) for n, cc, d1, d2 in corners() if n == name)
    ctr, ax = idler_pose(c, din, dout); xw = SL / 2 + WALL_T
    keep = [_cyl_axis(IDLER_R + 0.6, -IDLER_W / 2 - 0.3, IDLER_W / 2 + 0.3, ctr, ax)] + [_seg(p, q, CORD_R + 0.4) for p, q in cord_segments(0.0, 0.0) + cord_segments(TRAVEL_B, TRAVEL_C)]
    for (w, sd), (az, _, _, _) in HALVES.items():
        for k in (-1, 1): keep.append(cyl(2.0, 0, 90, c=(*rod_xy(az, k), 0), n=16))
    for ring, z0 in (("curl", Z_CURL), ("base", Z_BASE)):
        keep.append(app(cyl(SPOOL_R + 2.1, SPOOL_Y[0] - 0.5, SPOOL_Y[1] + 0.5, "y"), T(t=[SHAFT_X[ring], 0, SHAFT_Z[ring]])))
        keep.append(box([-SL / 2 - 0.3, -SH / 2 - 0.3, z0], [SL / 2 + 0.3, SH / 2 + 0.3, z0 + SW]))
    for n2, c2, d2i, d2o in corners():
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
        feet = [box([xs - 1.0, ys - 1.0, max(bb[2], 0.0)], [xs + 1.0, ys + 1.0, max(bb[5], PLATE_T)])]
        post = box([tip[0] - 1.6, tip[1] - 1.6, 0.0], [tip[0] + 1.6, tip[1] + 1.6, bb[5]])
        pad = M_.batch_hull([box([tip[0] - 1.6, tip[1] - 1.6, 0.0], [tip[0] + 1.6, tip[1] + 1.6, PLATE_T]), box([xs - 1.0, ys - 1.0, 0.0], [xs + 1.0, ys + 1.0, PLATE_T])])
        feet.append(post + pad)
        for k_, foot in enumerate(feet):
            web = (M_.batch_hull([boss, foot]) if k_ == 0 else M_.batch_hull([boss, box([tip[0] - 1.6, tip[1] - 1.6, bb[2]], [tip[0] + 1.6, tip[1] + 1.6, bb[5]])]) + foot)
            web -= _cyl_axis(1.6, -12, 12, ctr, ax, n=24)
            bad = sum((web ^ k).volume() + (axle(sgn) ^ k).volume() for k in keep)
            if best is None or bad < best[0] - 1e-6: best = (bad, web, sgn)
    _BR[name] = (best[1], best[2], best[0]); return _BR[name]


def cord_runs(pull_b, pull_c):
    out = {}
    for (which, side), (az, (ax, ay), pts, z0) in HALVES.items():
        ring = "base" if which == "out" else "curl"; pull = pull_b if which == "out" else pull_c
        dz = -(pull + GAP) if pull > 0 else 0.0
        seq = list(pts) + [np.array([ax, ay, lip_z(which)[0] - leg(which)[0] + dz])]
        for i in range(len(seq) - 1): out[f"cord_{ring}_{side}{i}"] = (seq[i], seq[i + 1])
    return out


def cord_segments(pull_b, pull_c):
    return list(cord_runs(pull_b, pull_c).values())


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


YOKES = [("out", "near"), ("out", "far"), ("in", "near"), ("in", "far")]

# --- cord routing, anchors, keys, links, trim (hub E) -------------------------------------------------------------
ZB, RL, FAZ, LS = C.ZB, H.R_LAG, C.FAZ, H.LAG_SIGN
HL = Z["hub_lo"]
RC = 0.3
Z_CORD = 6.0
HOLE_DEG, HOLE_D = 150.0, 1.0
R_WRAP = H.LAG_TUBE[0] / 2 + 0.3
K1_TOP = np.array([H.KN[0], H.CORD_Y[0] * np.cos(np.radians(H.PITCH)) - H.LEVER[0] * np.sin(np.radians(H.PITCH))])
W_PT = np.array([RL, 0.0])


def upper_path(k, lag_deg):
    s = LS[k]; th = np.radians(lag_deg)
    Rz = np.array([[np.cos(th), -np.sin(th)], [np.sin(th), np.cos(th)]]); S = W_PT + Rz @ (K1_TOP - W_PT)
    d = S - W_PT; L = np.linalg.norm(d); beta = np.arctan2(d[1], d[0]); g = np.arccos(R_WRAP / L)
    at = beta + s * g; ah = np.radians(s * HOLE_DEG); da = at - ah
    arc_ = [W_PT + R_WRAP * np.array([np.cos(ah + f * da), np.sin(ah + f * da)]) for f in np.linspace(0, 1, 13)]
    return np.array(arc_ + [S]), S


TUBE_WIN = (-1.25, 1.25)


def lag_tube2(k):
    t = cyl(H.LAG_TUBE[0] / 2, -5.0, 7.5, c=(RL, 0, 0)) - cyl(H.LAG_TUBE[1] / 2, -6, 8, c=(RL, 0, 0))
    t -= box([RL, -3, TUBE_WIN[0]], [RL + 3.5, 3, TUBE_WIN[1]])
    t -= app(cyl(HOLE_D / 2, 0, 4.0, "x", (0, 0, 0), n=20), T(t=[RL, 0, Z_CORD]) @ T(rot("z", LS[k] * HOLE_DEG)))
    return t


LINK_AZ = 12.5
R_LINK = 19.6
LINK = dict(t=2.1, strand_w=dict(base=1.1, curl=1.0), gap=5.0, block=1.5, active=5.0, top_dz=-1.0)
LUG = dict(dz=(-13.0, -9.0), r=(18.3, 21.5))
SCREW = dict(d=2.0, head_d=3.8, head_h=1.3, L=8.0, tap=1.6)
NUT = dict(af=4.0, h=1.6)
CAGE_SLOT_AZ = (-34.0, 27.0)
CAGE_SLOT_DZ = -30.0
HANGER_W = 2.0


def lug_z(): return (HL + LUG["dz"][0], HL + LUG["dz"][1])
def link_top(): return HL + LINK["top_dz"]
def adeg(mm, r=R_LINK): return float(np.degrees(mm / r))
def link_width(role): return 2 * LINK["strand_w"][role] + LINK["gap"]
def link_az(role): return -LINK_AZ if role == "base" else LINK_AZ


def _wedge_az(r0, r1, z0, z1, a0, a1, n=96):
    w = cyl(r1, z0, z1, n=n) - cyl(r0, z0 - 1, z1 + 1, n=n)
    return C.half(C.half(w, [-np.sin(np.radians(a0)), np.cos(np.radians(a0)), 0], [0, 0, 0]), [np.sin(np.radians(a1)), -np.cos(np.radians(a1)), 0], [0, 0, 0])


def cage():
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
        m -= _wedge_az(R_IN[0] - 1, R_IN[1] + 1, Z_FL_IN[1] + 0.5, HL + 20, ab - hb, ab + hb)
    return m


def ring_out():
    m = cyl(R_OUT[1], *Z_OUT, n=96) - cyl(R_OUT[0], Z_OUT[0] - 1, Z_OUT[1] + 1, n=96)
    m += cyl(R_FL_OUT, *Z_FL_OUT, n=96) - cyl(R_OUT[0], Z_FL_OUT[0] - 1, Z_FL_OUT[1] + 1, n=96)
    z0, _ = lug_z()
    for a in FAZ:
        ab = _hanger_az(a); hw = adeg(HANGER_W / 2)
        m += _wedge_az(LUG["r"][0], R_OUT[0] + 0.3, Z_OUT[0], Z_OUT[1], ab - adeg(HANGER_W / 2, 22.0), ab + adeg(HANGER_W / 2, 22.0))
        m += _wedge_az(LUG["r"][0], LUG["r"][1], z0, Z_OUT[1], ab - hw, ab + hw) + _lug("base", a)
    return m


def link(role):
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
    screw = cyl(SCREW["head_d"] / 2, z0 - SCREW["head_h"], z0 - 0.01, c=(R_LINK, 0, 0), n=20) + cyl(SCREW["d"] / 2, z0 - 0.01, z0 + SCREW["L"], c=(R_LINK, 0, 0), n=20)
    nut = app(cyl(NUT["af"] / 2 / np.cos(np.pi / 6), zb + 0.01, zb + NUT["h"], n=6), T(t=[R_LINK, 0, 0]) @ T(rot("z", 30)))
    return app(screw, A), app(nut, A)


def lower_cord(role, a_f, drop):
    p = rot("z", a_f) @ np.array([RL, np.sign(link_az(role)) * 0.75, ZB - 5.0])
    q = rot("z", a_f + link_az(role)) @ np.array([R_LINK, 0, link_top() - LINK["block"] / 2 - drop])
    return p, q


def upper_cord_solid(k, lag_deg):
    p, S = upper_path(k, lag_deg); pts = [pp for pp in p]
    a, b = pts[-2], pts[-1]; d = b - a; Ld = np.linalg.norm(d); pts[-1] = a + d * max(0.05, (Ld - 5.0) / Ld)
    A = rot("z", FAZ[k]); m = None
    for i in range(len(pts) - 1):
        s_ = _seg(A @ np.r_[pts[i], ZB + Z_CORD], A @ np.r_[pts[i + 1], ZB + Z_CORD], RC); m = s_ if m is None else m + s_
    return m


def parts():
    d = dict(cage=(cage(), "pacf", True), ring_in=(ring_in(), "pacf", True), ring_out=(ring_out(), "pacf", True), frame=(frame(), "pacf", False))
    for w, s in YOKES: d[f"yoke_{w}_{s}"] = (yoke_half(w, s), "pacf", False)
    d["spool"] = (spool(), "pacf", False); d["tie_ring"] = (tie_ring(), "pacf", False)
    d["link_base"] = (link("base"), "tpu", True); d["link_curl"] = (link("curl"), "tpu", True)
    return d


_PC = {}


def _parts():
    if not _PC: _PC.update(parts())
    return _PC


try:
    import hand3_springs as _SP
    SPRING_LID_TOP = _SP.tray_z(2)[2] + _SP.BOX["lid"]
except ImportError:
    SPRING_LID_TOP = None


def assembly(pull_b=0.0, pull_c=0.0, lag=(0, 0, 0, 0), fold=((0, 0, 0),) * 4, spin=0.0):
    """name -> (solid, rotating?) for the whole hand in the pod frame (mm)."""
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
    for n, segs in cords.items(): out[n] = (M_.batch_boolean(segs, mf.OpType.Add), False)
    out["hw_motor_bell"] = (app(cyl(17, Z["pack1"] + 8.5, Z["motor_top"]) + cyl(2.5, Z["motor_top"], Z["shaft_tip"]), S), True)
    for n in ("hub_lower", "hub_upper", "heel"): out[n] = (app(R[n], S), True)
    for k in range(4):
        Fr = C.finger_frames(k, lag[k], fold[k])
        out[f"grip{k}"] = (app(R["root_grip"], S @ Fr[0]), True)
        out[f"tube{k}"] = (app(lag_tube2(k), S @ T(t=[0, 0, ZB]) @ T(rot("z", FAZ[k]))), True)
        for j in range(3):
            out[f"ph{k}_{j + 1}"] = (app(R[f"phalanx{j + 1}"], S @ Fr[j + 1]), True); out[f"pad{k}_{j + 1}"] = (app(R[f"pad{j + 1}"], S @ Fr[j + 1]), True)
    for k, f in enumerate(FAZ):
        for role, dr in (("base", pull_b), ("curl", pull_c)):
            out[f"rlink_{role}{k}"] = (app(link_placed(role, f, dr), S), True)
            sc, nu = trim_hw(role, f, dr); out[f"hw_rscrew_{role}{k}"] = (app(sc, S), True); out[f"hw_rnut_{role}{k}"] = (app(nu, S), True)
            p, q = lower_cord(role, f, dr); out[f"rcordlo_{role}{k}"] = (app(_seg(p, q, RC), S), True)
        out[f"rcordup{k}"] = (app(upper_cord_solid(k, lag[k]), S), True)
    for k in range(4):
        hand = "R" if LS[k] > 0 else "L"; A = S @ T(rot("z", FAZ[k]))
        for nm in ("tray1", "tray2", "lid"):
            if f"spring_{nm}_{hand}" in R: out[f"s{nm}{k}"] = (app(R[f"spring_{nm}_{hand}"], A), True)
        if f"spring_lid_{hand}" in R and SPRING_LID_TOP is not None:
            sr, zt = C.SPRING_SCREW["r"], ZB + SPRING_LID_TOP
            scr = cyl(1.0, zt - 12.0, zt, c=(sr, 0, 0), n=20) + cyl(1.9, zt, zt + 1.3, c=(sr, 0, 0), n=24)
            out[f"hw_sscrew{k}"] = (app(scr, A), True)
    return out
