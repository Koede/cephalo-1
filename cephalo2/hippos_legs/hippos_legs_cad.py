#!/usr/bin/env python3
"""Hippos-1 legs v3 (concept B + added DOFs), full assembly model rev C (2026-11-20).
Rev C (more jump power): the fore flex (shoulder) joint motor is a RobStride 04 driving a 2.09:1 speed-up belt
(46T printed pulley on the motor, 22T on the humerus shaft, 576-8M-20, toothed 22T idler inside the loop); the fore yoke is
7 mm wider and its flange 7.5 mm higher to clear the larger motor; fore plates 60 mm radius at the pivot.  Hind leg unchanged.
Rev B: every fit is modelled (bearing seats, bores, running fits with FIT clearance), so no two solids share volume; lines,
springs and the carpus toggle have real routes; no pod covers.
Single source of the leg geometry: every part is a manifold3d solid, authored in the leg's REST frame (mm):
origin = pitch pivot, x forward, y outboard (left leg), z up, all segments hanging straight down (-z).
Joints are a tree of rotations about points in rest coordinates (NODES); Python (checks) and the viewer (JS) evaluate
the same tree.  Bought parts are drawn at their datasheet envelopes.
Layers along y at the pivot: motor 1 (pivot RS03 fore / RS06 hind) from -19 inboard | plate A -19..-13 | pulley / crank
-12..14 | plate B 15..21 | motor 2 (RS06) 21..70 | stub to the outboard yoke cheek 75..81.  The middle segment has one arm
each side of the upper segment (23..35 and -45..-33), because the belt and push rod run between the plates.
    python3 hippos_legs_cad.py -> legs_cad.json (tree, parts, masses)"""
import os, json, importlib.util, io, contextlib
import numpy as np
from manifold3d import Manifold, CrossSection, JoinType
import hardware as H

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("S", os.path.join(HERE, "legs_v3_study.py")); S = importlib.util.module_from_spec(spec)
with contextlib.redirect_stdout(io.StringIO()):
    spec.loader.exec_module(S)
O = S.OPT["B"]
D2R = np.pi / 180
SEG = 40

# ------------------------------------------------------------------ dimensions (mm)
LEG = {}
for name, key in (("fore", "front"), ("hind", "rear")):
    m = O["legs"][key]
    LEG[name] = dict(L1=m.l1 * 1e3, L2=m.l2 * 1e3, L3=m.l3 * 1e3, delta=m.delta / D2R,
                     pivot=np.array([O["pivot"][key][0] * 1e3, S.Y_TRACK * 1e3, O["pivot"][key][1] * 1e3]))
LEG["fore"]["LC"] = S.CARPUS * LEG["fore"]["L3"]          # elbow to carpus
RS04 = dict(name="RS04", r=60.0, t=55.7, m=1.420)        # RobStride 04: dia 120 x 55.7 (STEP), 40 / 120 N m
RS06 = dict(name="RS06", r=43.5, t=50.5, m=0.621); RS03 = dict(name="RS03", r=49.3, t=56.6, m=0.880); RS02 = dict(name="RS02", r=39.25, t=45.5, m=0.405)
STS = dict(a=45.2, b=24.7, c=35.0, m=0.055)               # STS3215 case
Y_A, Y_B = (-19.0, -13.0), (15.0, 21.0)                   # the two plates of the upper segment
Y_M2 = (21.0, 70.0)
WEB_L, WEB_R = -86.0, -112.0                              # rev C fore: tops of the two edge webs (z)
Y_CH_OUT = (75.0, 83.0)
Y_DRUM = (-31.0, -19.0)                                   # tendon drums (elbow / hock slaving), r 30
Y_ARM_O, Y_ARM_I = (23.0, 35.0), (-45.0, -33.0)           # humerus arms straddle the scapula (belt runs between the plates)
R_DRUM = 30.0
R_PLATE = 46.0
YAW_BELT_T = 98                                           # stock 5M belt 490-5M-15 (yaw)
SH_BELT_T, SH_ZM, SH_ZJ, SH_ZI = 72, 46, 22, 22           # shoulder: 576-8M-20 (72 teeth), 46T on the motor, 22T on the joint, 22T idler
FIT = 0.05                                                # radial clearance modelled in every fit (press or running)
BUF_ARM, BUF_AREA, BUF_T = 20.0, 196.0, 7.55                 # fetlock buffer: horn lever, pad area (14 x 14), pad thickness
TOG_E, TOG_A = 12.0, 36.0                                 # carpus toggle: anchors 12 mm behind the axis, 36 mm above / below it
ABD_AXES = dict(fore=np.array([-65.0, 22.0]), hind=np.array([-82.0, 22.0]))   # abduction axis (y, z) in rest coords, along x
# (hind 17 mm further inboard: the femur's front edge passes the front journal when the leg twists inward at full protraction)
X_LUG_F, X_LUG_R = (78.0, 86.0), (-192.0, -184.0)
BEAM_Y = (-88.0, -76.0)
ROCK_PIV = np.array([-310.0, -70.0])                      # sling rocker pivot (x, z), axis y (front only)
HOOF_H = 24.0                                             # ball centre above the sole
L_PAST, A_PAST = 40.0, 35.0                               # pastern length and its forward angle from the column


# ------------------------------------------------------------------ solid helpers
def cyl_y(r, y0, y1, x=0.0, z=0.0, seg=SEG):
    return Manifold.cylinder(y1 - y0, r, r, seg).rotate([-90, 0, 0]).translate([x, y0, z])


def cyl_x(r, x0, x1, y=0.0, z=0.0, seg=SEG):
    return Manifold.cylinder(x1 - x0, r, r, seg).rotate([0, 90, 0]).translate([x0, y, z])


def cyl_z(r, z0, z1, x=0.0, y=0.0, seg=SEG, r1=None):
    return Manifold.cylinder(z1 - z0, r, r if r1 is None else r1, seg).translate([x, y, z0])


def box(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def hull(*ms):
    return Manifold.batch_hull(list(ms))


def union(ms):
    return Manifold.batch_boolean(list(ms), __import__("manifold3d").OpType.Add) if len(ms) > 1 else ms[0]


def rod(a, b, r, seg=20):
    a, b = np.asarray(a, float), np.asarray(b, float); d = b - a; L = np.linalg.norm(d); z = d / L
    t = np.cross(z, [0, 1, 0] if abs(z[1]) < 0.9 else [1, 0, 0]); t /= np.linalg.norm(t); s = np.cross(z, t)
    M = np.column_stack([t, s, z, a])
    return Manifold.cylinder(L, r, r, seg).transform(M)


def plate_y(pts, y0, y1):
    """Hull of y-cylinders: pts = [(x, z, r), ...]."""
    return hull(*[cyl_y(r, y0, y1, x, z) for x, z, r in pts])


def pd(z):
    return z * 5.0 / np.pi


COL = dict(brass="#b5933a", alu="#c3c9d0", pacf="#2c2f34", light="#dfe3e8", act="#17181a", servo="#2a5fb0", belt="#3b3b3b", steel="#9aa3ad", carbon="#22252a", tpu="#e08a2e",
           line="#2e9e5b", spring="#c9a227", hoof="#444a52", bearing="#b8c0c8", orange="#d4572a", gas="#7a8794", shell="#dfe5ec", pod="#c9d2dc", octo="#f0a060")


def _h(r, seg=SEG):
    """Radius of an n-gon hole that clears a round part of radius r by FIT everywhere."""
    return (r + FIT) / np.cos(np.pi / seg)


def hole_y(r, y0, y1, x=0.0, z=0.0):
    return cyl_y(_h(r), y0, y1, x, z)


def hole_x(r, x0, x1, y=0.0, z=0.0):
    return cyl_x(_h(r), x0, x1, y, z)


def hole_z(r, z0, z1, x=0.0, y=0.0):
    return cyl_z(_h(r), z0, z1, x, y)


# ------------------------------------------------------------------ timing belts (5M, 8M): pitch line, stock lengths
T_IN, T_OUT = 0.57, 1.2                                   # 5M: pitch line to pulley OD / to the belt's back
T_IN8, T_OUT8 = 0.686, 2.0                                # 8M (5.6 to 6.0 mm thick, tooth 3.2 to 3.4: est. back height)


def pd8(z):
    return z * 8.0 / np.pi


def _arc(c, r, p0, p1, via, step=2.0):
    a0, a1, av = (np.arctan2(p[1] - c[1], p[0] - c[0]) for p in (p0, p1, via))
    ccw = (a1 - a0) % (2 * np.pi); v = (av - a0) % (2 * np.pi)
    sweep = ccw if v <= ccw else ccw - 2 * np.pi
    n = max(2, int(abs(np.degrees(sweep)) / step))
    return [c + r * np.array([np.cos(a0 + sweep * i / n), np.sin(a0 + sweep * i / n)]) for i in range(n + 1)]


def belt_path(c1, r1, c2, r2, delta=0.0, side=1, frac=0.5):
    """Closed pitch line round two pulleys (2D).  The span on `side` is pushed inward by `delta` at mid-span (idler);
    a negative delta pushes it outward (idler inside the loop).  frac: where along the span the idler sits (0 = at pulley 1)."""
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float); d = c2 - c1; C = np.linalg.norm(d); d = d / C; n = np.array([-d[1], d[0]])
    sb = (r2 - r1) / C; cb = np.sqrt(1 - sb * sb)
    m = lambda s: -sb * d + s * cb * n
    A1, A2 = c1 + r1 * m(-side), c2 + r2 * m(-side)                          # straight span
    far1, far2 = c1 - r1 * d, c2 + r2 * d
    if delta == 0:
        B1, B2 = c1 + r1 * m(side), c2 + r2 * m(side)
        return np.array([A1] + _arc(c2, r2, A2, B2, far2) + _arc(c1, r1, B1, A1, far1)[:-1]), None, None
    K = (c1 + r1 * m(side)) * (1 - frac) + (c2 + r2 * m(side)) * frac - m(side) * delta
    def tang(c, r):
        v = K - c; Lv = np.linalg.norm(v); a = np.arccos(r / Lv); b = np.arctan2(v[1], v[0])
        c_ = [c + r * np.array([np.cos(b + s * a), np.sin(b + s * a)]) for s in (1, -1)]
        return max(c_, key=lambda p: np.dot(p - c, m(side)))
    T1, T2 = tang(c1, r1), tang(c2, r2)
    return np.array([A1] + _arc(c2, r2, A2, T2, far2) + [K] + _arc(c1, r1, T1, A1, far1)[:-1]), K, m(side)


def path_len(P):
    return float(np.sum(np.linalg.norm(np.roll(P, -1, 0) - P, axis=1)))


def belt_solid(P, w0, w1, plane, t_out=T_OUT, t_in=T_IN):
    cs = CrossSection([[(float(x), float(y)) for x, y in P]])
    band = cs.offset(t_out, JoinType.Round, 2.0, 96) - cs.offset(-(t_in - FIT), JoinType.Round, 2.0, 96)
    m = Manifold.extrude(band, w1 - w0)
    return m.translate([0, 0, w0]) if plane == "xy" else m.rotate([90, 0, 0]).translate([0, w1, 0])


def solve(f, lo, hi, n=60):
    for _ in range(n):
        mid = (lo + hi) / 2
        if (f(lo) > 0) == (f(mid) > 0): lo = mid
        else: hi = mid
    return (lo + hi) / 2


YAW_C = solve(lambda C: path_len(belt_path((0, 0), pd(56) / 2, (-C, 0), pd(28) / 2)[0]) - 5.0 * YAW_BELT_T, 120.0, 160.0)


class Leg:
    """Collects parts: (name, node, group, colour, solid, printed, mass_kg or None), struts, toggles and mechanism data."""
    def __init__(s, kind):
        s.kind = kind; s.parts = []; s.struts = []; s.toggles = []; s.g = LEG[kind]; s.mech = {}

    def add(s, name, node, group, col, solid, printed=False, mass=None):
        s.parts.append(dict(name=name, node=node, group=group, col=COL[col], solid=solid, printed=printed, mass=mass))


def stack_dims(a1, a2=RS06, r_drive=0.0):
    """Heights and widths that follow the pivot motor's size, the joint motor riding on the upper segment and whatever else turns
    about the pivot axis (r_drive: belt back on the motor pulley).  Rev D: the motors are RobStride's own STEP envelopes."""
    zf = max(R_PLATE, a1["r"], a2["r"], r_drive, 48.0) + 2.0             # 48: the stifle crank pin + rod end swing r 48 under the flange
    D1, D2 = H.MOTORS[a1["name"]], H.MOTORS[a2["name"]]
    y_back2 = Y_M2[0] + D2["length"] - D2["front_face"]                    # joint motor: housing front face on plate B (y 21)
    dy = y_back2 + 0.5 + 3.0 + 1.5 - 75.0                                  # back cup end disk 3, then 1.5 to the 6805 seat
    y_sh1 = -21.0 - D1["back_face"]                                         # pivot motor: back shoulder (mount face)
    return dict(Y_M2=(Y_M2[0], y_back2), Y_CH_OUT=(Y_CH_OUT[0] + dy, Y_CH_OUT[1] + dy), DY=dy, Z_FLANGE=(zf, zf + 4), Z_HUB=(zf + 4, zf + 40), Z_BELT=(zf + 14, zf + 32), Z_DECK=(zf + 42.5, zf + 48.5),
                Y_M1=(-21.0 - D1["length"], -21.0), Y_CH_IN=(y_sh1 - 6.0, y_sh1))


AL_RHO = 2.70e-6


def upper_stack(L, hip_act, joint_act=RS06, r_drive=0.0, r_plate=R_PLATE):
    """Yoke, motors, upper segment (scapula / femur) with its two plates, and the body-side yaw + abduction hardware."""
    g = L.g; L1 = g["L1"]; kind = L.kind
    a1 = hip_act; a2 = joint_act; K = stack_dims(a1, a2, r_drive); L.K = K; L.a2 = a2; ABD_AX = ABD_AXES[kind]; Y_M2, Y_CH_OUT, dy = K["Y_M2"], K["Y_CH_OUT"], K["DY"]
    Z_FLANGE, Z_HUB, Z_BELT, Z_DECK, Y_M1, Y_CH_IN = (K[k] for k in ("Z_FLANGE", "Z_HUB", "Z_BELT", "Z_DECK", "Y_M1", "Y_CH_IN"))
    zh = Z_HUB[0]; OD56, OD28 = pd(56) / 2 - T_IN, pd(28) / 2 - T_IN
    D1, D2 = H.MOTORS[a1["name"]], H.MOTORS[a2["name"]]
    # ---- pivot motor, hung by its back shoulder from the yoke's inboard cheek (a ring round the back cover with four bosses).
    # Clocked so the four back holes sit above and below the axis: the cheek stays narrow in x, clear of the front lug in yaw.
    M1 = H.motor_frame([0.0, -21.0 + D1["out_face"], 0.0], [0, -1, 0], [0, 0, 1] if a1["name"] == "RS03" else [1, 0, 0])
    back1, f1 = H.pattern(a1["name"], "back_pat", M1)
    r_bc1 = D1["profile"][-1][0]
    cheek_in = hull(box(-32, 32, Y_CH_IN[0], Y_CH_IN[1], Z_FLANGE[0], Z_FLANGE[1]), cyl_y(r_bc1 + 3.5, *Y_CH_IN),
                    *[cyl_y(6.0, Y_CH_IN[0], Y_CH_IN[1], p[0], p[2]) for p in back1]) - hole_y(r_bc1 + 0.45, Y_CH_IN[0] - 1, Y_CH_IN[1] + 1)
    # ---- yoke (rotates in yaw): flange, hub = 56T pulley on two 61810, two cheeks
    cheek = lambda y, r: hull(box(-32, 32, y[0], y[1], Z_FLANGE[0], Z_FLANGE[1]), cyl_y(r, y[0], y[1], 0, 0))
    yoke = union([cyl_z(44, *Z_FLANGE), box(-32, 32, Y_CH_IN[0], Y_CH_OUT[1], Z_FLANGE[0], Z_FLANGE[1]), cheek_in, cheek(Y_CH_OUT, 24),
                  cyl_z(OD56, *Z_HUB) - hole_z(32.5, zh + 6.45, Z_HUB[1] + 1) - hole_z(30.0, Z_FLANGE[0] - 1, zh + 6.6) - hole_z(34.0, zh + 33.05, zh + 35.65)])
    yoke = yoke + (cyl_z(44, *Z_FLANGE) - hole_z(26.0, Z_FLANGE[0] - 1, Z_FLANGE[1] + 1)) - hole_z(26.0, Z_FLANGE[0] - 1, Z_FLANGE[1] + 0.01)
    yoke = yoke - hole_y(18.5, Y_CH_OUT[0] - 0.1, 82.0 + dy) - cyl_y(15, 81.9 + dy, Y_CH_OUT[1] + 0.1)           # 6805 seat with a 1 mm lip
    L.add("yaw yoke + 56T hub", "yoke", "structure", "pacf", yoke, True)
    H.add_motor(L, "pivot motor " + a1["name"], a1["name"], M1, "yoke", "upper", a1["m"])
    H.fasten(L, "pivot motor to yoke", "yoke", D1["back_pat"][3], 12.0 if D1["back_pat"][3] == 3 else 14.0, [p + np.array([0, -6.0, 0]) for p in back1], [0, 1, 0],
             "yaw yoke +", 6.0, None, kind="tapped", head="bhcs")
    # pivot output: a 2 mm steel disc on the motor's output, plate A screwed through it (button heads inside the plates: the drive
    # pulley / crank has a pocket for them)
    L.add("pivot output adapter (2 mm steel, dowelled)", "upper", "hardware", "steel", cyl_y(25, -20.95, -19.0), mass=0.03)
    out1, _ = H.pattern(a1["name"], "out_pat", M1)
    d1 = D1["out_pat"][3]; l1 = 16.0 if D1["out_pat"][4] >= 8 else 12.0
    H.fasten(L, "plate A to pivot output", "upper", d1, l1, [p + np.array([0, 8.0, 0]) for p in out1], [0, -1, 0],
             ["plates A + B", "pivot output adapter"], 8.0, None, kind="tapped", head="bhcs")
    L.pivot_screw_r = D1["out_pat"][1] / 2 + H.BHCS[d1][0] / 2                # pocket the drive pulley / crank must leave
    L.add("yaw bearings 2 x 61810", "cradle", "hardware", "bearing", cyl_z(32.5, zh + 6.5, zh + 13.5) + cyl_z(32.5, zh + 26, zh + 33) - hole_z(25, zh + 5, zh + 34), mass=0.104)
    L.add("yaw yoke circlip DIN 472 65 mm", "yoke", "hardware", "steel", cyl_z(33.95, zh + 33.1, zh + 35.6) - hole_z(29.0, zh + 33, zh + 36), mass=0.008)
    L.add("stub bearing 6805", "yoke", "hardware", "bearing", cyl_y(18.5, 75 + dy, 82 + dy) - hole_y(12.5, 74 + dy, 83 + dy), mass=0.021)
    # ---- upper segment: plates A and B joined by edge webs, drum A, motor 2 on plate B with its stub
    re_ = 50.0
    if kind == "fore":      # belt between the plates: webs on both edges (the +x one only far from the pivot: it passes under the yaw hub)
        pts = [(0, 0, R_PLATE), (0, -L1, re_)]
        if r_plate > R_PLATE:   # rev C: the 8M loop is wider near the pivot, so the edge webs start lower (clear of the belt and the idler's bulge)
            webs = [box(-52, -46, Y_A[0], Y_B[1], -L1 + 12, WEB_L), box(46, 52, Y_A[0], Y_B[1], -L1 + 12, WEB_R)]
        else: webs = [box(-52, -46, Y_A[0], Y_B[1], -L1 + 12, -52), box(46, 52, Y_A[0], Y_B[1], -L1 + 12, -100)]
    else:                   # push rod on the +x side, tibia folds to -x: one web, on the +x edge, outside the rod
        pts = [(0, 0, R_PLATE), (0, -L1, re_)]
        pts += [(32, -62, 24), (32, -L1 + 14, 24)]
        webs = [box(50, 56, Y_A[0], Y_B[1], -L1 + 14, -62)]
    lobe = [cyl_y(r_plate, *Y_B)] if r_plate > R_PLATE else []          # plate B carries the RS04's flange: full 60 mm disc at the pivot
    r_boreB = D2["out_r"] + 1.6                                               # plate B bore: the joint motor's spinning output and the hub flange pass it
    up = union([plate_y(pts, *Y_A), plate_y(pts, *Y_B)] + webs + lobe) - hole_y(r_boreB, Y_B[0] - 1, Y_B[1] + 1) - hole_y(18.5, Y_A[0] - 1, Y_B[1] + 1, 0, -L1)
    up = up - hole_y(12.6, Y_A[0] - 1, Y_A[1] + 1)                          # plate A: pilot for the adapter's dowel boss / grease
    up = up + (cyl_y(R_DRUM, Y_DRUM[0], Y_DRUM[1], 0, -L1) - cyl_y(15.0, Y_DRUM[0] - 1, Y_DRUM[1] + 1, 0, -L1))     # tendon drum A, fixed to the upper segment
    L.up_name = ("scapula" if kind == "fore" else "femur") + " (plates A + B, drum A)"
    L.up_solid = up
    # joint motor: housing front face on plate B's outer face (y 21), 10 x M4 (RS04) / 8 x M3 (RS06) from inside the plates
    M2 = H.motor_frame([0.0, Y_M2[0] - D2["front_face"], 0.0], [0, 1, 0], [1, 0, 0])
    H.add_motor(L, "joint motor " + a2["name"] + " (rides on the upper segment)", a2["name"], M2, "upper", "drive", a2["m"])
    fr2, _ = H.pattern(a2["name"], "front_pat", M2); dfr = D2["front_pat"][3]
    if kind == "fore":      # the 46T pulley runs 1.5 mm inside plate B: button heads sunk flush
        H.fasten(L, "joint motor to plate B", "upper", dfr, 10.0, [p - np.array([0, 6.0, 0]) for p in fr2], [0, 1, 0], "plates A + B", 6.0, None,
                 kind="tapped", head="bhcs", cbore=2.3)
    else:
        H.fasten(L, "joint motor to plate B", "upper", dfr, 12.0, [p - np.array([0, 6.0, 0]) for p in fr2], [0, 1, 0], "plates A + B", 6.0, None,
                 kind="tapped", head="bhcs")
    L.M2 = M2
    # back cup + 25 mm stub, one machined 6061 part: screwed to the back shoulder, round the back cover, stub into the 6805 in the yoke
    r_bc2 = D2["profile"][-1][0]; y_sh2 = Y_M2[0] - D2["front_face"] + D2["back_face"]; bk2, _ = H.pattern(a2["name"], "back_pat", M2)
    r_pcd = min(np.hypot(p[0], p[2]) for p in bk2); r_ring = r_pcd + 5.0
    r_wall = min(r_bc2 + 3.0, r_pcd - H.BHCS[D2["back_pat"][3]][0] / 2 - 0.3)      # shell wall inside the back screw heads
    cup = union([cyl_y(r_ring, y_sh2, y_sh2 + 4.0), cyl_y(r_wall, y_sh2, Y_M2[1] + 3.4), cyl_y(12.5, Y_M2[1] + 3.0, 81.9 + dy)])
    cup = cup - hole_y(r_bc2 + 0.45, y_sh2 - 1, Y_M2[1] + 0.5)
    L.add("joint motor back cup + 25 mm stub (6061)", "upper", "structure", "alu", cup, mass=None)
    L.parts[-1]["mass"] = float(cup.volume() * AL_RHO)
    dbk = D2["back_pat"][3]
    H.fasten(L, "joint motor back cup", "upper", dbk, 8.0 if dbk == 4 else 10.0, [p + np.array([0, 4.0, 0]) for p in bk2], [0, -1, 0],
             "joint motor back cup", 4.0, None, kind="tapped", head="bhcs")
    # ---- cradle (rotates in abduction): fixed yaw spindle, deck, RS02 clamp ring on three posts, lugs on the abduction stubs
    xr = -YAW_C; ya = ABD_AX[0]
    deck = hull(cyl_z(46, *Z_DECK), cyl_z(51, Z_DECK[0], Z_DECK[1], xr, 0))
    spindle = cyl_z(25, zh + 6.5, zh + 33.05) + cyl_z(27, zh + 33.05, Z_DECK[0] + 1) - cyl_z(19, zh + 5, Z_DECK[1] + 1)   # shoulder on the upper inner race
    lug = lambda xs: hull(box(xs[0], xs[1], ya - 14, ya + 14, Z_DECK[0], Z_DECK[1]), cyl_x(14, xs[0], xs[1], ya, ABD_AX[1])) - hole_x(7.5, xs[0] - 1, xs[1] + 1, ya, ABD_AX[1])
    lugR = hull(box(X_LUG_R[0], X_LUG_R[1], ya - 14, ya + 14, Z_DECK[0], Z_DECK[1]), cyl_x(18, X_LUG_R[0], X_LUG_R[1], ya, ABD_AX[1]))
    arm_f = hull(box(X_LUG_F[0], X_LUG_F[1], ya - 14, ya + 14, *Z_DECK), box(0, 30, ya - 14, 0, *Z_DECK))
    arm_r = box(X_LUG_R[0], xr, ya - 14, 14, *Z_DECK)
    # yaw motor RS02, hung output up: its output flange face carries the 28T pulley; a 4.5 mm ring on the housing's front face
    # (9 x M3, heads sunk flush) hangs it from three posts under the deck
    z_out = Z_BELT[0] - 1.0
    DY_ = H.MOTORS["RS02"]; Myaw = H.motor_frame([xr, 0.0, z_out + DY_["out_face"]], [0, 0, -1], [1, 0, 0])
    zr = z_out - (DY_["front_face"] - DY_["out_face"])                    # housing front face
    rr = RS02["r"]
    ring = cyl_z(rr + 9, zr, z_out + 1.0, xr, 0) - hole_z(22.4, zr - 1, z_out + 2, xr, 0)
    posts = [box(xr + dx * (rr + 5) - 4, xr + dx * (rr + 5) + 4, dy_ * (rr + 5) - 4, dy_ * (rr + 5) + 4, zr, Z_DECK[0] + 0.5) for dx, dy_ in ((-1, 0), (0, 1), (0, -1))]
    cradle = union([deck, spindle, lug(X_LUG_F), lugR, arm_f, arm_r, ring] + posts)
    L.add("abduction cradle (deck, yaw spindle, lugs)", "cradle", "structure", "light", cradle, True)
    L.add("yaw bearing clamp washer (2 mm steel)", "cradle", "hardware", "steel", cyl_z(28, zh + 4.45, zh + 6.45) - hole_z(16, zh + 4, zh + 7), mass=0.02)
    H.fasten(L, "yaw clamp washer", "cradle", 3, 10.0, [np.array([22 * np.cos(a), 22 * np.sin(a), zh + 4.45]) for a in np.radians([90, 210, 330])], [0, 0, 1],
             "yaw bearing clamp washer", 2.0, "abduction cradle", kind="insert")
    H.add_motor(L, "yaw motor RS02 (hung, output up)", "RS02", Myaw, "cradle", "cradle", RS02["m"])
    fy, _ = H.pattern("RS02", "front_pat", Myaw)
    H.fasten(L, "yaw motor ring", "cradle", 3, 6.0, [p + np.array([0, 0, 4.5]) for p in fy], [0, 0, -1], "abduction cradle", 4.5, None,
             kind="tapped", head="bhcs", cbore=1.8)
    p28 = cyl_z(OD28, z_out + 0.05, Z_BELT[1] + 1, xr, 0) - hole_z(14.3, z_out - 1, z_out + 0.6, xr, 0)
    L.add("yaw pulley 28T-5M (steel, on the RS02 output)", "cradle", "drive", "steel", p28, mass=0.03)
    oy, _ = H.pattern("RS02", "out_pat", Myaw)
    top = Z_BELT[1] + 1
    H.fasten(L, "yaw pulley", "cradle", 4, 25.0, [np.array([p[0], p[1], top]) for p in oy], [0, 0, -1], "yaw pulley 28T", top - z_out, None, kind="tapped", head="shcs")
    Pb = belt_path((0, 0), pd(56) / 2, (xr, 0), pd(28) / 2)[0]
    L.add("yaw belt 490-5M-15 (28T : 56T)", "cradle", "drive", "belt", belt_solid(Pb, Z_BELT[0] + 1.5, Z_BELT[1] - 1.5, "xy"), mass=0.03)
    # ---- abduction: RS06 (fore) / RS02 (hind) on the rear bracket, hung by a 4.5 mm ring on its housing front face; its output
    # flange is bolted through the cradle's rear lug (the lug + a 4 mm steel flange are the rear stub); front stub in a 6802
    abd = RS06 if kind == "fore" else RS02; DA = H.MOTORS[abd["name"]]
    xm1 = X_LUG_R[0] - 4.0                                                  # output flange face (4 mm steel flange, then the 8 mm lug)
    MA = H.motor_frame([xm1 + DA["out_face"], ya, ABD_AX[1]], [-1, 0, 0], [0, 0, 1])
    xf = xm1 - (DA["front_face"] - DA["out_face"])                          # housing front face
    xa0 = xm1 + DA["out_face"] - DA["length"]
    flange = cyl_x(16, xm1 + 0.05, X_LUG_R[0], ya, ABD_AX[1])
    if DA["out_face"] > 0:                                                  # pilot recess for the output boss (RS02)
        flange = flange - hole_x(DA["profile"][0][0], xm1 - 1, xm1 + DA["out_face"] + 0.1, ya, ABD_AX[1])
    L.add("abduction output flange (4 mm steel)", "cradle", "hardware", "steel", flange, mass=0.03)
    L.add("abduction front stub 15 mm (in a 6802)", "cradle", "hardware", "steel", cyl_x(7.5, X_LUG_F[0], X_LUG_F[1] + 11, ya, ABD_AX[1]), mass=0.025)
    oa, _ = H.pattern(abd["name"], "out_pat", MA)
    la = 16.0                                                               # 12 clamp + 4 thread (RS06 holes 4.7 deep)
    H.fasten(L, "abduction lug to output", "cradle", 4, la, [np.array([X_LUG_R[1], p[1], p[2]]) for p in oa], [-1, 0, 0],
             ["abduction cradle", "abduction output flange"], X_LUG_R[1] - xm1, None, kind="tapped", head="shcs")
    rr_a = DA["front_pat"][1] / 2 + 6.0
    tr = min(4.5, X_LUG_R[0] - 1.5 - xf)                                     # ring thickness: 1.5 mm off the cradle's rear lug
    rear = hull(cyl_x(rr_a, xf, xf + tr, ya, ABD_AX[1]), box(xf, xf + tr, ya - 45, ya - 20, ABD_AX[1] - 30, ABD_AX[1] + 30))
    rear = rear - hole_x(DA["out_r"] + 1.6, xf - 1, xf + tr + 1, ya, ABD_AX[1])
    xb = X_LUG_F[1] + 2
    front = hull(cyl_x(16, xb, xb + 10, ya, ABD_AX[1]), box(xb, xb + 10, ya - 30, ya - 14, ABD_AX[1] - 12, ABD_AX[1] + 40))
    front = front - hole_x(12, xb - 0.1, xb + 7, ya, ABD_AX[1]) - cyl_x(9.5, xb - 1, xb + 11, ya, ABD_AX[1])
    L.add("hip bracket, rear (abduction motor ring)", "rocker", "structure", "pacf", rear, True)
    L.add("hip bracket, front (journal)", "rocker", "structure", "pacf", front, True)
    H.add_motor(L, "abduction motor " + abd["name"], abd["name"], MA, "rocker", "cradle", abd["m"])
    fa_, _ = H.pattern(abd["name"], "front_pat", MA)
    H.fasten(L, "abduction motor ring", "rocker", DA["front_pat"][3], 8.0, [np.array([xf + tr, p[1], p[2]]) for p in fa_], [-1, 0, 0],
             "hip bracket, rear", tr, None, kind="tapped", head="bhcs", cbore=1.8)
    L.add("abduction bearing 6802", "rocker", "hardware", "bearing", cyl_x(12, xb + 2, xb + 7, ya, ABD_AX[1]) - hole_x(7.5, xb + 1, xb + 8, ya, ABD_AX[1]), mass=0.007)
    L.abd = dict(xa0=xa0, xm1=xm1, xf=xf)


def keyway(zS, y0, y1, x1):
    """DIN 6885 8 mm keyway on the 25 mm shaft at zS, key on +z: x1 = 13.5 cuts the shaft (4.0 deep), 15.85 a hub (3.3 deep)."""
    return box(-4.05, 4.05, y0 - 0.05, y1 + 0.05, zS + 8.45, zS + x1)


def middle_arms(L, zS, zJ, tag, hub=(-11.5, 13.5)):
    """Middle segment (humerus / tibia): a 25 mm shaft through the upper segment's bearings with one arm each side of it;
    each arm carries a 6800 for the 10 mm pin of the next joint."""
    # The shaft carries the joint torque: DIN 6885 A 8 x 7 keys (shaft keyway 4.0 deep, hub 3.3) into the pulley / driven crank and
    # into both arms, which are 6061 plate (a printed arm can't take a key).  M5 cup-point set screws on the arm keys hold the arms
    # axially; spacer sleeves between the arms, the pulley / crank and the 6805 inner races make the stack.
    keys = [(hub[0] + 1, hub[1] - 1), (Y_ARM_O[0] + 1, Y_ARM_O[1] - 1), (Y_ARM_I[0] + 1, Y_ARM_I[1] - 1)]
    kw = lambda y0, y1, x1: keyway(zS, y0, y1, x1)
    shaft = cyl_y(12.5, Y_ARM_I[0], Y_ARM_O[1], 0, zS) - cyl_y(6.0, Y_ARM_I[0] - 1, Y_ARM_O[1] + 1, 0, zS)     # 25 / 12 tube, 4140 (2.5 mm under the keyways)
    for y0, y1 in keys:
        shaft = shaft - kw(y0, y1, 13.5)
    L.add(tag + " shaft 25 / 12 mm 4140 tube (3 keyways)", "middle", "hardware", "steel", shaft, mass=None); L.parts[-1]["mass"] = float(shaft.volume() * H.STEEL_RHO)
    L.add(tag + " keys DIN 6885 A 8 x 7 (3)", "middle", "hardware", "steel", union([box(-4, 4, y0, y1, zS + 8.5, zS + 15.5) for y0, y1 in keys]), mass=0.012)
    sp = union([cyl_y(14.5, Y_B[1] + 0.05, Y_ARM_O[0] - 0.05, 0, zS), cyl_y(13.3, Y_ARM_I[1] + 0.05, Y_A[0] - 0.05, 0, zS),
                cyl_y(14.5, Y_A[1] + 0.05, hub[0] - 0.05, 0, zS), cyl_y(14.5, hub[1] + 0.05, Y_B[0] - 0.05, 0, zS)]) - hole_y(12.5, Y_ARM_I[0], Y_ARM_O[1], 0, zS)
    L.add(tag + " shaft spacers (4 sleeves, steel)", "middle", "hardware", "steel", sp, mass=None); L.parts[-1]["mass"] = float(sp.volume() * H.STEEL_RHO)
    for nm, yy, seat in (("outer", Y_ARM_O, (Y_ARM_O[0] - 0.1, Y_ARM_O[0] + 5)), ("inner", Y_ARM_I, (Y_ARM_I[1] - 5, Y_ARM_I[1] + 0.1))):
        arm = plate_y([(0, zS, 22), (0, zJ, 14)], *yy) - hole_y(12.5, yy[0] - 1, yy[1] + 1, 0, zS) - hole_y(9.5, seat[0], seat[1], 0, zJ) - cyl_y(7.0, yy[0] - 1, yy[1] + 1, 0, zJ)
        arm = arm - kw(yy[0] - 1, yy[1] + 1, 15.85) - cyl_z(2.5, zS + 15, zS + 23, 0, (yy[0] + yy[1]) / 2)          # keyway, M5 set screw hole
        arm = arm - hull(cyl_y(8.0, yy[0] - 1, yy[1] + 1, 0, zS - 36), cyl_y(5.0, yy[0] - 1, yy[1] + 1, 0, zJ + 26))   # lightening slot
        L.add(f"{tag} arm, {nm} (6061 plate, keyed)", "middle", "structure", "alu", arm, False, mass=float(arm.volume() * AL_RHO))
    L.add(tag + " arm set screws 2 x M5 x 6 cup point", "middle", "hardware", "steel",
          union([cyl_z(2.45, zS + 15.55, zS + 21.55, 0, (yy[0] + yy[1]) / 2) for yy in (Y_ARM_O, Y_ARM_I)]), mass=0.002)
    L.add(tag + " pin bearings 2 x 6800", "middle", "hardware", "bearing",
          cyl_y(9.5, Y_ARM_O[0], Y_ARM_O[0] + 5, 0, zJ) + cyl_y(9.5, Y_ARM_I[1] - 5, Y_ARM_I[1], 0, zJ) - hole_y(5, Y_ARM_I[0] - 1, Y_ARM_O[1] + 1, 0, zJ), mass=0.01)
    xt = R_DRUM + 1.5 + FIT                                                 # tendon centre line: lying on the drums
    L.add(("elbow" if tag == "humerus" else "hock") + " tendons Dyneema 3 mm (parallel pair)", "middle", "drive", "line",
          union([rod([sx * xt, y, zS], [sx * xt, y, zJ], 1.5, 12) for sx, y in ((1, -28), (-1, -22))]), mass=0.004)


ECLIP = {10: (4.0, 7.2, 0.9), 6: (2.5, 4.7, 0.7)}                 # DIN 6799 E-clip for a pin of d: (groove radius, outer radius, thickness)


def clipped_pin(L, name, node, d, y0, y1, x, z, ends, mass, gap=0.2):
    """Pin from y0 to y1 at (x, z), lengthened 1.6 mm at each end in `ends` (-1 / +1) for a DIN 6799 E-clip in a groove."""
    rg, ro, t = ECLIP[d]; pin = None; clips = []
    ext = gap + t + 0.5                                     # clip sits `gap` off the face (gap 1.6 where that face turns on the pin)
    a = y0 - (ext if -1 in ends else 0.0); b = y1 + (ext if 1 in ends else 0.0)
    pin = cyl_y(d / 2, a, b, x, z)
    for e in ends:
        g0 = (y1 + gap) if e > 0 else (y0 - gap - t)
        pin = pin - (cyl_y(d / 2 + 1, g0, g0 + t, x, z) - cyl_y(rg, g0 - 1, g0 + t + 1, x, z))
        clips.append(cyl_y(ro, g0, g0 + t, x, z) - hole_y(rg, g0 - 1, g0 + t + 1, x, z))
    L.add(name, node, "hardware", "steel", pin, mass=mass)
    L.add(name.split(" (")[0] + " E-clips DIN 6799 (%d)" % len(ends), node, "hardware", "steel", union(clips), mass=0.0005 * len(ends))


def gas_spring(L, zS, q_lo, q_hi, Le=114.0, Lc=84.0):
    """Offload strut between a stud on plate B and a stud on the middle segment's outer arm, in the plane y = 45.
    Needs a 6/15 strut 114 / 84 mm between the eye centres (30 stroke).  No stock 6/15 does that: eye to eye is about 2 x stroke + 63
    (Lesjoefors F9002 + L-E11 eyes: 103.5 / 83.5, 20 stroke; Struts Australia K128-15-30: 128 + fittings), and with 20 or 25 stroke the
    middle stud would have to sit 10 to 12 mm from the joint, inside the 25 mm shaft.  Open: custom stroke, lost-motion slot, or no spring.
    Middle stud at least 18 mm from the joint (shaft r 12.5 + M6 tapped hole + wall).
    Anchors from a grid search: inside the stroke over the joint range, most lever at the stand."""
    L1 = L.g["L1"]; q = np.radians(np.arange(q_lo - 3, q_hi + 3.1, 1.0)); best = None
    for rho in (18, 20, 22, 24, 26):
        for phi in np.radians((-20, -10, 0, 10, 20)):
            mx, mz = rho * np.sin(phi), -rho * np.cos(phi)
            Mx, Mz = mx * np.cos(q) + mz * np.sin(q), -mx * np.sin(q) + mz * np.cos(q)
            for ux in np.arange(-38, 38.1, 2):
                for uz in np.arange(60, L1 - 56 + 0.1, 2):
                    if np.hypot(ux, L1 - uz) < L.a2["r"] + 7.5 + 3: continue
                    Ls = np.hypot(Mx - ux, Mz - uz)
                    if Ls.max() > Le - 1.5 or Ls.min() < Lc + 1.5 or np.any(np.diff(Ls) > 0): continue
                    arm = np.abs((Mx * (Mz - uz) - Mz * (Mx - ux))) / Ls            # lever of the strut line about the joint
                    sc = arm.min()
                    if best is None or sc > best[0]: best = (sc, rho, phi, ux, uz, Ls.max(), Ls.min(), arm)
    sc, rho, phi, ux, uz, lmax, lmin, arm = best
    U = [float(ux), 45.0, float(zS + uz)]; M = [float(rho * np.sin(phi)), 45.0, float(zS - rho * np.cos(phi))]
    # studs: ISO 7379 shoulder screws, 8 mm shoulder, M6 x 9.5 thread, head 13 x 5.5.  The eye (8 mm thick, y 41..49) sits on the shoulder between
    # steel spacer sleeves.  Upper: 8 x 16 into an M6 heat-set insert in a 20 mm boss standing on plate B's outer face (y 21..35; inside the
    # plates the push rod / belt sweeps, so nothing pokes inward); middle: 8 x 16 into an M6 tapped 10.5 deep in the outer arm.
    def shoulder(P, y_face, ls, nm, node):
        sol = cyl_y(6.5, y_face + ls, y_face + ls + 5.5, P[0], P[2]) + cyl_y(4.0 - FIT, y_face, y_face + ls + 0.05, P[0], P[2]) + cyl_y(3.0 - FIT, y_face - 9.5, y_face + 0.05, P[0], P[2])
        L.add(nm, node, "hardware", "steel", sol, mass=float(sol.volume() * H.STEEL_RHO))
        sp = (cyl_y(6.0, y_face + 0.05, 40.95, P[0], P[2]) + cyl_y(6.0, 49.05, y_face + ls - 0.05, P[0], P[2])) - hole_y(4.0 + 0.05, y_face - 1, y_face + ls + 1, P[0], P[2])
        L.add(nm.split(",")[0] + " spacer sleeves (2, steel)", node, "hardware", "steel", sp, mass=float(sp.volume() * H.STEEL_RHO))
    yb = Y_ARM_O[1]                                                     # boss face, level with the outer arm's face
    up = H.part_by(L, L.up_name)
    up["solid"] = (up["solid"] + cyl_y(10.0, Y_B[1] - 0.5, yb, U[0], U[2])) - hole_y(4.0, yb - 12.7, yb + 1, U[0], U[2])
    shoulder(U, yb, 16.0, "gas spring upper stud, ISO 7379 8 x 16 M6 shoulder screw", "upper")
    ins = cyl_y(4.0 - FIT, yb - 12.7, yb, U[0], U[2]) - hole_y(3.0, yb - 13.2, yb + 1, U[0], U[2])
    L.add("gas spring upper stud heat-set insert M6 x 12.7", "upper", "hardware", "brass", ins, mass=float(ins.volume() * H.BRASS_RHO))
    ao = H.part_by(L, "arm, outer")
    ao["solid"] = ao["solid"] - hole_y(3.0, Y_ARM_O[1] - 10.5, Y_ARM_O[1] + 1, M[0], M[2])
    shoulder(M, Y_ARM_O[1], 16.0, "gas spring middle stud, ISO 7379 8 x 16 M6 shoulder screw", "middle")
    nm = ("shoulder" if L.kind == "fore" else "stifle") + " gas spring 6/15, 30 stroke, 400 N, 8 mm eyes (114 eye to eye: no stock part yet)"
    L.struts.append(dict(name=nm, a=("upper", U), b=("middle", M), r=7.5, col=COL["gas"], trim=(9.0, 9.0), group="hardware"))
    L.mech["gas"] = dict(U=U, M=M, rho=rho, Le=Le, Lc=Lc, len_range=[float(lmin), float(lmax)], q=[q_lo, q_hi], lever_mm=[float(arm.min()), float(arm.max())])


def column_head(zJ):
    """Head of the lower segment between the middle segment's arms: eye, drum B, neck.  The 10 mm pin is pressed in the eye."""
    h = union([cyl_y(16, -20.5, 21, 0, zJ), cyl_y(R_DRUM, Y_DRUM[0], -20.5, 0, zJ), hull(cyl_y(16, -13, 13, 0, zJ), box(-12, 12, -13, 13, zJ - 34, zJ - 24))])
    return h - hole_y(5, -32, 22, 0, zJ)


def distal(L, node, body, z_top, length, tag):
    """Fetlock fork on `body` (the cannon), pastern with flexor horn and buffer horn, ball-joint hoof.
    The column's ground point is z_top - length; the ball centre sits HOOF_H above it on the column line."""
    zF = z_top - length
    ball = np.array([0.0, 0.0, zF + HOOF_H])
    d = np.array([np.sin(A_PAST * D2R), 0.0, -np.cos(A_PAST * D2R)])
    fet = ball - L_PAST * d; fx, fz = fet[0], fet[2]
    fork = union([hull(cyl_y(10, y0, y1, fx, fz), box(fx - 7, fx + 9, y0, y1, fz + 8, fz + 20)) for y0, y1 in ((8, 13), (-13, -8))]
                 + [box(fx - 7, fx + 9, -13, 13, fz + 14, fz + 22), box(fx + 9, fx + 27, -7, 7, fz + 16, fz + 22)])
    can = union([body, fork]) - hole_y(3, -14, 14, fx, fz)
    horn = fet + 20 * np.array([-np.cos(A_PAST * D2R), 0, -np.sin(A_PAST * D2R)])     # flexor horn: behind the fetlock
    bh = fet + np.array([BUF_ARM, 0.0, 4.5])                                                # buffer horn: ahead of it
    # flexor crank: a side plate outside the fork cheek, from the horn's cross pin up to Q above the axis, so the line to the spool
    # in front pulls square to it (lever about 17 mm) and pulling flexes the fetlock
    Q = fet + 18.0 * np.array([-0.5, 0.0, 0.866])
    crank = hull(cyl_y(3.5, -24.5, -21.5, horn[0], horn[2]), cyl_y(3.5, -24.5, -21.5, Q[0], Q[2]))
    past = union([cyl_y(8, -6.5, 6.5, fx, fz), rod(fet, ball, 6.5), rod(fet, horn, 4.5), cyl_y(2.5, -24.5, 0, horn[0], horn[2]), crank, rod(fet, bh, 4.0)])
    past = past - hole_y(3, -7, 7, fx, fz) - Manifold.sphere(8.3, 48).translate(list(ball))
    clipped_pin(L, tag + " fetlock pin 6 mm", node, 6, -13, 13, fx, fz, (-1, 1), 0.006)
    L.add(tag + " fetlock buffer (TPU 95A)", node, "structure", "tpu", box(fx + 13, fx + 27, -7, 7, fz + 8.45, fz + 16), True)
    L.add(tag + " pastern (flexor horn, buffer horn)", "pastern", "structure", "pacf", past, True)
    L.add(tag + " hoof ball joint SKF GE 8 E", "pastern", "hardware", "bearing", Manifold.sphere(8, 48).translate(list(ball)), mass=0.008)
    core = cyl_z(25, zF + 6, zF + 20, r1=21).translate([ball[0], 0, 0]) - Manifold.sphere(8.3, 48).translate(list(ball))
    L.add(tag + " hoof core", "hoof", "structure", "hoof", core, True)
    L.add(tag + " hoof shoe (TPU 95A)", "hoof", "structure", "tpu", cyl_z(26, zF, zF + 6).translate([ball[0], 0, 0]), True)
    H.fasten(L, tag + " hoof shoe to core", "hoof", 3, 8.0, [[ball[0] + 15 * np.cos(a), 15 * np.sin(a), zF] for a in np.radians((90, 210, 330))], [0, 0, 1],
             tag + " hoof shoe", 6.0, tag + " hoof core", kind="insert", head="shcs", cbore=4.0)
    L.add(tag + " hoof centring ring (TPU)", "hoof", "structure", "tpu", cyl_z(17, zF + 20, zF + 25).translate([ball[0], 0, 0]) - cyl_z(11.5, zF + 19, zF + 26).translate([ball[0], 0, 0]), True)
    return can, fet, ball, Q


def fetlock_servo(L, node, x0, z0, tag, horn, back_x):          # horn: the crank's line point Q
    """STS3215 (real case shape) on the cranial face, x0 = its back face, shaft along -y, spool on it; the cannon grows a plate
    under the servo's bottom face (+y) joined to its back plate (back_x .. 12.45), and 4 M2 x 6 self-tappers hold the servo."""
    R = [[0, 1, 0], [0, 0, -1], [-1, 0, 0]]                       # model x = S.Y, y = -S.Z, z = -S.X
    M = H.sts_place(R, [x0 + 12.41, 0.0, z0 + 21.5])
    sname = f"fetlock servo STS3215 (on the {tag})"
    L.add(sname, node, "actuators", "servo", H.sts_apply(M, H.sts3215()), mass=STS["m"])
    plate, pz0 = H.sts_bottom_plate(-1, 12.41 + x0 - back_x - 12.4 + 0.0)
    zc = z0 + 21.5
    web = box(back_x, 12.45, 11.0, -pz0, zc - 22.7, zc + 22.7)
    cp = H.part_by(L, tag + " (")
    cp["solid"] = cp["solid"] + H.sts_apply(M, plate) + web
    H.fasten(L, tag + " servo to plate", node, 2, 6.0, [H.sts_point(M, (hx, hy, pz0)) for hx, hy in H.STS_HOLES_BOTTOM], M[:3, :3] @ [0, 0, 1],
             tag + " (", H.STS_BOTTOM_Z - pz0, sname, kind="tapped", head="shcs")
    sc = H.sts_point(M, (12.5, 0, 0)); sc = [sc[0], sc[2]]
    ys0, ys1 = -(H.STS_DISC_TOP + 0.1), -27.5
    spool = cyl_y(9, ys1, ys0, sc[0], sc[1]) - hole_y(2.8, -(H.STS_SPLINE_TOP + 0.1), ys0 + 1, sc[0], sc[1])
    L.add(tag + " flexor spool (on the servo spline)", node, "drive", "steel", spool, mass=0.006)
    H.fasten(L, tag + " spool to spline", node, 3, 8.0, [[sc[0], ys1, sc[1]]], [0, 1, 0], tag + " flexor spool", -ys1 - H.STS_SPLINE_TOP,
             sname, kind="tapped", head="shcs", cbore=3.5)
    L.struts.append(dict(name=tag + " fetlock flexor line 2 mm", a=("tan", node, [sc[0], -23.0, sc[1]], 9.0 + 1.0 + FIT, "maxz"), b=("pastern", [float(horn[0]), -23.0, float(horn[2])]),
                         r=1.0, col=COL["line"], trim=(0.0, 3.6), group="drive"))


def build_fore():
    L = Leg("fore"); g = L.g; L1, L2, LC, L3 = g["L1"], g["L2"], g["LC"], g["L3"]
    RM, RJ, RI = pd8(SH_ZM) / 2, pd8(SH_ZJ) / 2, pd8(SH_ZI) / 2
    upper_stack(L, RS03, RS04, RM + T_OUT8, 60.0)
    zS, zE, zC = -L1, -L1 - L2, -L1 - L2 - LC
    # shoulder drive (rev C): RS04, 46T on the motor, 22T on the humerus shaft (2.09:1 speed-up), 576-8M-20 belt between the plates,
    # a toothed 22T idler INSIDE the loop pushing the +x span outward (8M wants no small back idler)
    FR_I = 0.586                                                       # idler centred in the gap between the two pulleys
    f = lambda dl: path_len(belt_path((0, 0), RM, (0, zS), RJ, -dl, side=1, frac=FR_I)[0]) - 8.0 * SH_BELT_T
    dl = solve(f, 0.2, 25.0); Pb, Kk, mo = belt_path((0, 0), RM, (0, zS), RJ, -dl, side=1, frac=FR_I)
    L.mech["belt"] = dict(teeth=SH_BELT_T, pitch=8, deflect_mm=float(dl), slack_free_len=float(path_len(belt_path((0, 0), RM, (0, zS), RJ)[0])), ratio=SH_ZJ / SH_ZM, r_joint=float(RJ), r_motor=float(RM))
    belt = belt_solid(Pb, -9.0, 11.0, "xz", T_OUT8, T_IN8)
    ic = Kk - mo * (RI + 0.1)
    yo = L.M2[:3, 3][1] + H.MOTORS["RS04"]["out_face"]                     # RS04 output flange face (y 20.5)
    pul = cyl_y(RM - T_IN8, -11.5, 13.5) + cyl_y(24, 13.5, yo - 0.05)
    pul = pul - hole_y(L.pivot_screw_r + 1.5, -12, -9.2) - hole_y(14.6, 17.4, yo + 1)          # pocket for plate A's screw heads; output pilot
    L.add("shoulder drive pulley 46T-8M (aluminium, on the RS04 output)", "drive", "drive", "alu", pul, mass=None)
    L.parts[-1]["mass"] = float(pul.volume() * AL_RHO)
    o2, _ = H.pattern("RS04", "out_pat", L.M2)
    H.fasten(L, "drive pulley to RS04 output", "drive", 5, 30.0, [np.array([p[0], -9.2, p[2]]) for p in o2], [0, 1, 0],
             "shoulder drive pulley", yo + 9.2, None, kind="tapped", head="shcs", cbore=5.0)
    L.add("shoulder belt 576-8M-20 (46T : 22T)", "upper", "drive", "belt", belt, mass=0.09)
    idl = cyl_y(RI - T_IN8, -11.5, 13.5, ic[0], ic[1]) + cyl_y(4, Y_A[1], Y_B[0], ic[0], ic[1])          # unflanged: the flanged 22T sits too close
    L.add("belt idler 22T-8M on a slotted pin", "upper", "hardware", "steel", idl, mass=0.07)
    L.add(L.up_name, "upper", "structure", "orange", L.up_solid, True)
    L.add("shoulder bearings 2 x 6805", "upper", "hardware", "bearing", cyl_y(18.5, *Y_A, 0, zS) + cyl_y(18.5, *Y_B, 0, zS) - hole_y(12.5, -20, 22, 0, zS), mass=0.042)
    p22 = cyl_y(RJ - T_IN8, -11.5, 13.5, 0, zS) + cyl_y(RJ + 3.0, -11.5, -10.5, 0, zS) + cyl_y(RJ + 3.0, 12.5, 13.5, 0, zS)          # flanged
    p22 = p22 - hole_y(12.5, -12, 14, 0, zS) - keyway(zS, -10.5, 12.5, 15.85)
    L.add("shoulder pulley 22T-8M, flanged, keyed (6061)", "middle", "drive", "alu", p22, mass=None); L.parts[-1]["mass"] = float(p22.volume() * AL_RHO)
    middle_arms(L, zS, zE, "humerus")
    gas_spring(L, zS, 50, 132)
    # ---- forearm: elbow head, tube, C-shaped carpus fork (open behind, so the folded cannon has room), toggle tab A
    cheek = lambda y0, y1: hull(cyl_y(11, y0, y1, 0, zC), cyl_y(11, y0, y1, 13, zC + 30)) + hull(cyl_y(11, y0, y1, 13, zC + 30), cyl_y(11, y0, y1, 0, zC + 56))
    tabA = hull(cyl_y(6, 13, 17.5, 13, zC + 30), cyl_y(6, 13, 17.5, -TOG_E, zC + TOG_A))
    fa = union([column_head(zE), rod([0, 0, zE - 28], [0, 0, zC + 52], 11), box(-11, 11, -13, 13, zC + 48, zC + 60), cheek(8, 13), cheek(-13, -8), tabA])
    fa = fa - hole_y(5, -14, 14, 0, zC) - hole_y(4, 12, 18, -TOG_E, zC + TOG_A)
    # carpus servo: STS3215 behind the forearm tube, back face on x -11.5, shaft along +y; a plate under its bottom face (-y) and a web to the tube
    zs0 = zC + 58
    YC = 0.0                                    # servo frame origin y: plate under it clears the folded pastern's crank
    Mc = H.sts_place([[0, 1, 0], [0, 0, 1], [1, 0, 0]], [-11.5 - 12.41, YC, zs0 + 23.5])          # model x = S.Y, y = S.Z, z = S.X
    plate, pz0 = H.sts_bottom_plate(+1, 3.0, t=4.0)          # 4 thick: the M2 heads sink 2 mm, clear of the folded pastern
    zc = zs0 + 23.5
    fa = fa + H.sts_apply(Mc, plate) + box(-11.4, -5.0, pz0 + YC, -8.0, zc - 22.7, zc + 22.7)
    L.add("forearm (elbow head, drum B, carpus fork)", "lower", "structure", "carbon", fa, True)
    clipped_pin(L, "elbow pin 10 mm", "lower", 10, Y_ARM_I[0], Y_ARM_O[1], 0, zE, (-1, 1), 0.05, gap=1.6)
    clipped_pin(L, "carpus pin 10 mm (its +y end is the toggle's stop)", "lower", 10, -21.5, 26.5, 0, zC, (-1,), 0.03)
    L.add("carpus return spring (torsion, on the pin)", "lower", "hardware", "spring", cyl_y(8.5, -21, -14, 0, zC) - hole_y(5, -22, -13, 0, zC), mass=0.006)
    L.add("toggle pin A 8 mm", "lower", "hardware", "steel", cyl_y(4, 13, 22.6, -TOG_E, zC + TOG_A), mass=0.004)
    L.add("carpus servo STS3215 (on the forearm)", "lower", "actuators", "servo", H.sts_apply(Mc, H.sts3215()), mass=STS["m"])
    H.fasten(L, "carpus servo to plate", "lower", 2, 6.0, [H.sts_point(Mc, (hx, hy, pz0)) for hx, hy in H.STS_HOLES_BOTTOM], Mc[:3, :3] @ [0, 0, 1],
             "forearm (elbow", H.STS_BOTTOM_Z - pz0, "carpus servo STS3215", kind="tapped", head="shcs", cbore=2.0)
    sp = H.sts_point(Mc, (12.5, 0, 0)); ys0 = YC + H.STS_DISC_TOP + 0.1; ysp = YC + H.STS_SPLINE_TOP
    spool = cyl_y(9, ys0, 30, sp[0], sp[2]) - hole_y(2.8, ys0 - 1, ysp + 0.1, sp[0], sp[2])
    L.add("carpus release spool (on the servo spline)", "lower", "drive", "steel", spool, mass=0.006)
    H.fasten(L, "carpus spool to spline", "lower", 3, 10.0, [[sp[0], 30.0, sp[2]]], [0, -1, 0], "carpus release spool", 30.0 - ysp, "carpus servo STS3215", kind="tapped", head="shcs", cbore=6.0)
    # ---- cannon: eye on two 6800 between the cheeks, neck, loft to the fetlock fork, toggle tab B, fetlock servo in front
    zF = zC - (L3 - LC); fz = zF + HOOF_H + L_PAST * np.cos(A_PAST * D2R); fx = -L_PAST * np.sin(A_PAST * D2R)
    tabB = hull(cyl_y(6, 0, 17.5, -TOG_E, zC - TOG_A), cyl_y(6, 0, 17.5, -4, zC - TOG_A))
    body = union([cyl_y(12.5, -6.5, 6.5, 0, zC), box(-9, 9, -6.5, 6.5, zC - 34, zC - 8), hull(box(-9, 9, -6.5, 6.5, zC - 34, zC - 30), box(fx - 7, fx + 9, -13, 13, fz + 14, fz + 22)),
                  tabB, box(8.9, 12.45, -12, 12, zC - 58, zC - 15)])
    body = body - hole_y(9.5, -6.6, 6.6, 0, zC) - hole_y(4, -1, 18, -TOG_E, zC - TOG_A)
    can, fet, ball, horn = distal(L, "cannon", body, zC, L3 - LC, "fore")
    L.add("fore cannon (carpus eye, fetlock fork, toggle tab B)", "cannon", "structure", "carbon", can, True)
    L.add("carpus bearings 2 x 6800", "cannon", "hardware", "bearing", cyl_y(9.5, -6.5, -1.5, 0, zC) + cyl_y(9.5, 1.5, 6.5, 0, zC) - hole_y(5, -7, 7, 0, zC), mass=0.01)
    L.add("toggle pin B 8 mm", "cannon", "hardware", "steel", cyl_y(4, 11, 26.4, -TOG_E, zC - TOG_A), mass=0.006)
    fetlock_servo(L, "cannon", 12.5, zC - 59.2, "fore cannon", horn, 8.9)
    lk = float(np.hypot(TOG_A, TOG_E - (10.0 + FIT)))
    L.toggles.append(dict(name="carpus toggle", a=("lower", [-TOG_E, 0, zC + TOG_A]), b=("cannon", [-TOG_E, 0, zC - TOG_A]), l=lk, r=3.5, col="#c0392b",
                          y1=(19.0, 22.4), y2=(22.8, 26.2), ypin=(19.0, 29.5), lock_below=1e-3))
    L.struts.append(dict(name="carpus release line 1.5 mm", a=("tan", "lower", [-24.0, 28.0, zs0 + 36], 9.0 + 0.75 + FIT, "minx"), b=("knee", 0, 28.0), r=0.75, col=COL["line"], trim=(0.0, 3.6), group="drive"))
    L.joints = dict(zS=zS, zE=zE, zC=zC, fet=fet, ball=ball)
    H.finish(L, COL)
    return L


def build_hind():
    L = Leg("hind"); g = L.g; L1, L2, L3 = g["L1"], g["L2"], g["L3"]
    upper_stack(L, RS06)
    zS, zH = -L1, -L1 - L2
    # stifle drive: 1:1 parallelogram push rod, cranks 40 mm between the plates.  Rest crank direction: perpendicular to the femur at q1 = 110 deg
    a0 = -110.0 * D2R
    c0 = 40.0 * np.array([np.cos(a0), 0.0, -np.sin(a0)])
    crank = lambda z, y0=-11.5: hull(cyl_y(14, y0, -2, 0, z), cyl_y(7, y0, -2, c0[0], z + c0[2])) + cyl_y(4, y0, 11.5, c0[0], z + c0[2])
    yo = L.M2[:3, 3][1] + H.MOTORS["RS06"]["out_face"]                     # RS06 output face (y 19.5)
    dc = crank(0.0, -9.0) + cyl_y(18, -9.0, yo - 0.05)
    sweep = []                                                              # the rod in the crank's frame over q1 48..148: hub cut 1.5 clear of it
    for q in np.arange(48.0, 149.0, 2.0):
        A = np.eye(4); A[:3, 3] = c0; B = np.eye(4); B[:3, 3] = -c0; R = (A @ rot("y", -q) @ B)[:3, :4]
        sweep.append(rod(c0 + [0, 5, -2.0], c0 + [0, 5, -75.0], 6.5).transform(R))
    dc = dc - union(sweep)
    L.add("stifle drive crank 40 mm (6061, on the RS06 output)", "drive", "drive", "alu", dc, mass=None)
    L.parts[-1]["mass"] = float(dc.volume() * AL_RHO)
    o2, _ = H.pattern("RS06", "out_pat", L.M2)
    H.fasten(L, "drive crank to RS06 output", "drive", 4, 25.0, [np.array([p[0], -9.0, p[2]]) for p in o2], [0, 1, 0],
             "stifle drive crank", yo + 9.0, None, kind="tapped", head="shcs", cbore=8.0)
    eye = lambda z: cyl_y(8, 0, 10, c0[0], z + c0[2]) - hole_y(4, -1, 11, c0[0], z + c0[2])
    L.add("stifle push rod 10 x 8 carbon, rod ends", "rodn", "drive", "carbon", union([rod(c0 + [0, 5, -7.5], c0 + [0, 5, zS + 7.5], 5), eye(0.0), eye(zS)]), mass=0.03)
    L.add(L.up_name, "upper", "structure", "orange", L.up_solid, True)
    L.add("stifle bearings 2 x 6805", "upper", "hardware", "bearing", cyl_y(18.5, *Y_A, 0, zS) + cyl_y(18.5, *Y_B, 0, zS) - hole_y(12.5, -20, 22, 0, zS), mass=0.042)
    dk = crank(zS) + cyl_y(20, -11.5, -2, 0, zS) - hole_y(12.5, -12, -1, 0, zS) - keyway(zS, -10.5, -3.0, 15.85)
    L.add("stifle driven crank 40 mm (6061, keyed)", "middle", "drive", "alu", dk, mass=None); L.parts[-1]["mass"] = float(dk.volume() * AL_RHO)
    middle_arms(L, zS, zH, "tibia", hub=(-11.5, -2.0))
    gas_spring(L, zS, 50, 146)
    z_top = zH - 28; zF = z_top - (L3 - 28); fz = zF + HOOF_H + L_PAST * np.cos(A_PAST * D2R); fx = -L_PAST * np.sin(A_PAST * D2R)
    body = union([column_head(zH), rod([0, 0, zH - 28], [fx, 0, fz + 20], 11),
                  hull(box(11.5, 12.45, -12, 12, zH - 84, zH - 40), rod([0.4 * fx, 0, zH - 48], [0.9 * fx, 0, zH - 76], 5))])
    can, fet, ball, horn = distal(L, "lower", body, z_top, L3 - 28, "hind")
    L.add("hind cannon (hock head, drum B, fetlock fork)", "lower", "structure", "carbon", can, True)
    clipped_pin(L, "hock pin 10 mm", "lower", 10, Y_ARM_I[0], Y_ARM_O[1], 0, zH, (-1, 1), 0.04, gap=1.6)
    fetlock_servo(L, "lower", 12.5, zH - 84.5, "hind cannon", horn, 11.5)
    L.joints = dict(zS=zS, zH=zH, fet=fet, ball=ball, c0=c0)
    H.finish(L, COL)
    return L




# ------------------------------------------------------------------ joint tree (same for Python and the viewer)
# node: (parent, origin in rest coords, axis, {param: coefficient}, constant deg).  Angles in degrees.
def tree(kind, L):
    g = LEG[kind]; j = L.joints
    fore = kind == "fore"
    N = {
        "rocker": ("leg", [ROCK_PIV[0], 0, ROCK_PIV[1]], "y", {"sling": 1.0}, 0.0),
        "cradle": ("rocker", [0, ABD_AXES[kind][0], ABD_AXES[kind][1]], "x", {"abd": 1.0}, 0.0),
        "yoke": ("cradle", [0, 0, 0], "z", {"yaw": 1.0}, 0.0),
        "upper": ("yoke", [0, 0, 0], "y", {"q0": -1.0}, 0.0),
        "drive": ("upper", [0, 0, 0], "y", {"q1": 1.5 if fore else 1.0}, 0.0),
        "middle": ("upper", [0, 0, j["zS"]], "y", {"q1": 1.0}, 0.0),
        "lower": ("middle", [0, 0, j["zE"] if fore else j["zH"]], "y", {"q1": -1.0}, -g["delta"]),
    }
    last = "lower"
    if fore:
        N["cannon"] = ("lower", [0, 0, j["zC"]], "y", {"fold": 1.0}, 0.0); last = "cannon"
    else:
        N["rodn"] = ("drive", [float(j["c0"][0]), 0, float(j["c0"][2])], "y", {"q1": -1.0}, 0.0)
    N["pastern"] = (last, [float(j["fet"][0]), 0, float(j["fet"][2])], "y", {"fet": 1.0}, 0.0)
    N["hoofp"] = ("pastern", [float(j["ball"][0]), 0, float(j["ball"][2])], "y", {"hp": 1.0}, 0.0)
    N["hoof"] = ("hoofp", [float(j["ball"][0]), 0, float(j["ball"][2])], "x", {"hr": 1.0}, 0.0)
    return N


def rot(axis, deg):
    a = deg * D2R; c, s = np.cos(a), np.sin(a)
    R = {"x": [[1, 0, 0], [0, c, -s], [0, s, c]], "y": [[c, 0, s], [0, 1, 0], [-s, 0, c]], "z": [[c, -s, 0], [s, c, 0], [0, 0, 1]]}[axis]
    M = np.eye(4); M[:3, :3] = R
    return M


def fk(N, p):
    """Node -> 4x4 (rest coords to leg coords) for the parameter dict p (deg)."""
    T = {"leg": np.eye(4)}
    def get(n):
        if n in T:
            return T[n]
        par, o, ax, coef, const = N[n]
        ang = const + sum(c * p.get(k, 0.0) for k, c in coef.items())
        A = np.eye(4); A[:3, 3] = o; B = np.eye(4); B[:3, 3] = -np.asarray(o, float)
        T[n] = get(par) @ A @ rot(ax, ang) @ B
        return T[n]
    for n in N:
        get(n)
    return T


def mesh_of(m):
    me = m.to_mesh()
    return np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts)


FORE, HIND = build_fore(), build_hind()
TREE = {"fore": tree("fore", FORE), "hind": tree("hind", HIND)}
STAND = {}
for kind, key in (("fore", "front"), ("hind", "rear")):
    m = O["legs"][key]; q = m.ik(O["x0"][key], -O["pivot"][key][1]) / D2R
    STAND[kind] = dict(q0=float(q[0]), q1=float(q[1]))

PRINT_RHO, PRINT_FILL = 1.20e-6, 0.55                       # kg/mm3 PA-CF; printed parts as shells + infill (est.)


def masses(L):
    rows = []
    for p in L.parts:
        v = p["solid"].volume()
        m = p["mass"] if p["mass"] is not None else (v * PRINT_RHO * PRINT_FILL if p["printed"] else 0.0)
        rows.append((p["name"], p["node"], bool(p["printed"]), round(v / 1e3, 1), round(m, 3)))
    return rows


if __name__ == "__main__":
    out = {}
    for L in (FORE, HIND):
        rows = masses(L); bad = [p["name"] for p in L.parts if p["solid"].is_empty() or (p["printed"] and len(p["solid"].decompose()) != 1)]
        tot = sum(r[4] for r in rows) + 0.045 + 0.055             # gas spring, toggle / lines / fasteners (est.)
        below = sum(r[4] for r in rows if r[1] in ("upper", "drive", "middle", "lower", "cannon", "pastern", "hoof", "hoofp", "rodn"))
        out[L.kind] = dict(parts=rows, total=round(tot, 3), swinging=round(below, 3), not_single_body=bad)
        print(L.kind, "parts", len(rows), "total %.2f kg, pitching with the leg %.2f kg" % (tot, below), "not single body:", bad)
        for r in rows:
            print("   %-52s %-8s %s %7.1f cm3 %6.0f g" % (r[0], r[1], "P" if r[2] else " ", r[3], r[4] * 1e3))
    out["four_legs"] = round(2 * out["fore"]["total"] + 2 * out["hind"]["total"], 2)
    print("four legs with yaw, abduction, carriers:", out["four_legs"], "kg")
    json.dump(out, open(os.path.join(HERE, "legs_cad.json"), "w"), indent=1)
