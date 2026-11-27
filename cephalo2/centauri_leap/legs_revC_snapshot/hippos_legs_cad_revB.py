#!/usr/bin/env python3
"""Hippos-1 legs v3 (concept B + added DOFs), full assembly model rev B (2026-11-19).
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
RS06 = dict(r=44.0, t=49.0, m=0.621); RS03 = dict(r=53.0, t=56.0, m=0.880); RS02 = dict(r=39.25, t=45.5, m=0.405)
STS = dict(a=45.2, b=24.7, c=35.0, m=0.055)               # STS3215 case
Y_A, Y_B = (-19.0, -13.0), (15.0, 21.0)                   # the two plates of the upper segment
Y_M2 = (21.0, 70.0)
Y_CH_OUT = (75.0, 83.0)
Y_DRUM = (-31.0, -19.0)                                   # tendon drums (elbow / hock slaving), r 30
Y_ARM_O, Y_ARM_I = (23.0, 35.0), (-45.0, -33.0)           # humerus arms straddle the scapula (belt runs between the plates)
R_DRUM = 30.0
R_PLATE = 46.0
YAW_BELT_T, SH_BELT_T = 98, 105                           # stock 5M belts: 490-5M-15 (yaw), 525-5M-25 (shoulder)
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


COL = dict(pacf="#2c2f34", light="#dfe3e8", act="#17181a", servo="#2a5fb0", belt="#3b3b3b", steel="#9aa3ad", carbon="#22252a", tpu="#e08a2e",
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


# ------------------------------------------------------------------ timing belts (5M): pitch line, stock lengths
T_IN, T_OUT = 0.57, 1.2                                   # pitch line to pulley OD / to the belt's back


def _arc(c, r, p0, p1, via, step=2.0):
    a0, a1, av = (np.arctan2(p[1] - c[1], p[0] - c[0]) for p in (p0, p1, via))
    ccw = (a1 - a0) % (2 * np.pi); v = (av - a0) % (2 * np.pi)
    sweep = ccw if v <= ccw else ccw - 2 * np.pi
    n = max(2, int(abs(np.degrees(sweep)) / step))
    return [c + r * np.array([np.cos(a0 + sweep * i / n), np.sin(a0 + sweep * i / n)]) for i in range(n + 1)]


def belt_path(c1, r1, c2, r2, delta=0.0, side=1):
    """Closed pitch line round two pulleys (2D).  The span on `side` is pushed inward by `delta` at mid-span (idler)."""
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float); d = c2 - c1; C = np.linalg.norm(d); d = d / C; n = np.array([-d[1], d[0]])
    sb = (r2 - r1) / C; cb = np.sqrt(1 - sb * sb)
    m = lambda s: -sb * d + s * cb * n
    A1, A2 = c1 + r1 * m(-side), c2 + r2 * m(-side)                          # straight span
    far1, far2 = c1 - r1 * d, c2 + r2 * d
    if delta <= 0:
        B1, B2 = c1 + r1 * m(side), c2 + r2 * m(side)
        return np.array([A1] + _arc(c2, r2, A2, B2, far2) + _arc(c1, r1, B1, A1, far1)[:-1]), None, None
    K = (c1 + r1 * m(side) + c2 + r2 * m(side)) / 2 - m(side) * delta
    def tang(c, r):
        v = K - c; Lv = np.linalg.norm(v); a = np.arccos(r / Lv); b = np.arctan2(v[1], v[0])
        c_ = [c + r * np.array([np.cos(b + s * a), np.sin(b + s * a)]) for s in (1, -1)]
        return max(c_, key=lambda p: np.dot(p - c, m(side)))
    T1, T2 = tang(c1, r1), tang(c2, r2)
    return np.array([A1] + _arc(c2, r2, A2, T2, far2) + [K] + _arc(c1, r1, T1, A1, far1)[:-1]), K, m(side)


def path_len(P):
    return float(np.sum(np.linalg.norm(np.roll(P, -1, 0) - P, axis=1)))


def belt_solid(P, w0, w1, plane):
    cs = CrossSection([[(float(x), float(y)) for x, y in P]])
    band = cs.offset(T_OUT, JoinType.Round, 2.0, 96) - cs.offset(-(T_IN - FIT), JoinType.Round, 2.0, 96)
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


def stack_dims(a1):
    """Heights and widths that follow the pivot motor's size (RS06 r 44 / RS03 r 53)."""
    zf = max(R_PLATE, a1["r"], 48.0) + 2.0             # 48: the stifle crank pin + rod end swing r 48 under the flange
    y0 = -21.0 - a1["t"]
    return dict(Z_FLANGE=(zf, zf + 4), Z_HUB=(zf + 4, zf + 40), Z_BELT=(zf + 14, zf + 32), Z_DECK=(zf + 42.5, zf + 48.5),
                Y_M1=(y0, -21.0), Y_CH_IN=(y0 - 6, y0))


def upper_stack(L, hip_act):
    """Yoke, motors, upper segment (scapula / femur) with its two plates, and the body-side yaw + abduction hardware."""
    g = L.g; L1 = g["L1"]; kind = L.kind
    a1 = hip_act; K = stack_dims(a1); L.K = K; ABD_AX = ABD_AXES[kind]
    Z_FLANGE, Z_HUB, Z_BELT, Z_DECK, Y_M1, Y_CH_IN = (K[k] for k in ("Z_FLANGE", "Z_HUB", "Z_BELT", "Z_DECK", "Y_M1", "Y_CH_IN"))
    zh = Z_HUB[0]; OD56, OD28 = pd(56) / 2 - T_IN, pd(28) / 2 - T_IN
    # ---- yoke (rotates in yaw): flange, hub = 56T pulley on two 61810, two cheeks
    cheek = lambda y, r: hull(box(-32, 32, y[0], y[1], Z_FLANGE[0], Z_FLANGE[1]), cyl_y(r, y[0], y[1], 0, 0))
    yoke = union([cyl_z(44, *Z_FLANGE), box(-32, 32, Y_CH_IN[0], Y_CH_OUT[1], Z_FLANGE[0], Z_FLANGE[1]), cheek(Y_CH_IN, 30), cheek(Y_CH_OUT, 24),
                  cyl_z(OD56, *Z_HUB) - hole_z(32.5, zh + 4, Z_HUB[1] + 1)])
    yoke = yoke - hole_y(18.5, Y_CH_OUT[0] - 0.1, 82.0) - cyl_y(15, 81.9, Y_CH_OUT[1] + 0.1)           # 6805 seat with a 1 mm lip
    L.add("yaw yoke + 56T hub", "yoke", "structure", "pacf", yoke, True)
    L.add("pivot motor " + ("RS03" if a1 is RS03 else "RS06"), "yoke", "actuators", "act", cyl_y(a1["r"], *Y_M1), mass=a1["m"])
    L.add("pivot output adapter", "upper", "hardware", "steel", cyl_y(25, -20.9, -19.0), mass=0.03)
    L.add("yaw bearings 2 x 61810", "cradle", "hardware", "bearing", cyl_z(32.5, zh + 6.5, zh + 13.5) + cyl_z(32.5, zh + 26, zh + 33) - hole_z(25, zh + 5, zh + 34), mass=0.104)
    L.add("stub bearing 6805", "yoke", "hardware", "bearing", cyl_y(18.5, 75, 82) - hole_y(12.5, 74, 83), mass=0.021)
    # ---- upper segment: plates A and B joined by edge webs, drum A, motor 2 on plate B with its stub
    re_ = 50.0
    if kind == "fore":      # belt between the plates: webs on both edges (the +x one only far from the pivot: it passes under the yaw hub)
        pts = [(0, 0, R_PLATE), (0, -L1, re_)]
        webs = [box(-52, -46, Y_A[0], Y_B[1], -L1 + 12, -52), box(46, 52, Y_A[0], Y_B[1], -L1 + 12, -100)]
    else:                   # push rod on the +x side, tibia folds to -x: one web, on the +x edge, outside the rod
        pts = [(0, 0, R_PLATE), (0, -L1, re_), (32, -62, 24), (32, -L1 + 14, 24)]
        webs = [box(50, 56, Y_A[0], Y_B[1], -L1 + 14, -62)]
    up = union([plate_y(pts, *Y_A), plate_y(pts, *Y_B)] + webs) - cyl_y(14.5, Y_B[0] - 1, Y_B[1] + 1) - hole_y(18.5, Y_A[0] - 1, Y_B[1] + 1, 0, -L1)
    up = up + (cyl_y(R_DRUM, Y_DRUM[0], Y_DRUM[1], 0, -L1) - cyl_y(15.0, Y_DRUM[0] - 1, Y_DRUM[1] + 1, 0, -L1))     # tendon drum A, fixed to the upper segment
    L.up_name = ("scapula" if kind == "fore" else "femur") + " (plates A + B, drum A)"
    L.up_solid = up
    L.add("joint motor RS06 (rides on the upper segment)", "upper", "actuators", "act", cyl_y(RS06["r"], *Y_M2) + cyl_y(12.5, 70, 81), mass=RS06["m"])
    # ---- cradle (rotates in abduction): fixed yaw spindle, deck, RS02 clamp ring on three posts, lugs on the abduction stubs
    xr = -YAW_C; ya = ABD_AX[0]
    deck = hull(cyl_z(46, *Z_DECK), cyl_z(51, Z_DECK[0], Z_DECK[1], xr, 0))
    spindle = cyl_z(25, zh + 6.5, Z_DECK[0] + 1) - cyl_z(19, zh + 5, Z_DECK[1] + 1)
    lug = lambda xs: hull(box(xs[0], xs[1], ya - 14, ya + 14, Z_DECK[0], Z_DECK[1]), cyl_x(14, xs[0], xs[1], ya, ABD_AX[1])) - hole_x(7.5, xs[0] - 1, xs[1] + 1, ya, ABD_AX[1])
    arm_f = hull(box(X_LUG_F[0], X_LUG_F[1], ya - 14, ya + 14, *Z_DECK), box(0, 30, ya - 14, 0, *Z_DECK))
    arm_r = box(X_LUG_R[0], xr, ya - 14, 14, *Z_DECK)
    zr = Z_BELT[0] - 2                                                     # RS02 output face (pulley above it)
    rr = RS02["r"]
    ring = cyl_z(rr + 9, zr - 8, zr, xr, 0) - hole_z(rr, zr - 9, zr + 1, xr, 0)
    posts = [box(xr + dx * (rr + 5) - 4, xr + dx * (rr + 5) + 4, dy * (rr + 5) - 4, dy * (rr + 5) + 4, zr - 8, Z_DECK[0] + 0.5) for dx, dy in ((-1, 0), (0, 1), (0, -1))]
    cradle = union([deck, spindle, lug(X_LUG_F), lug(X_LUG_R), arm_f, arm_r, ring] + posts)
    L.add("abduction cradle (deck, yaw spindle, lugs)", "cradle", "structure", "light", cradle, True)
    L.add("yaw motor RS02 (hung, output up)", "cradle", "actuators", "act", cyl_z(rr, zr - RS02["t"], zr, xr, 0), mass=RS02["m"])
    L.add("yaw pulley 28T", "cradle", "drive", "steel", cyl_z(OD28, Z_BELT[0] - 1, Z_BELT[1] + 1, xr, 0), mass=0.03)
    Pb = belt_path((0, 0), pd(56) / 2, (xr, 0), pd(28) / 2)[0]
    L.add("yaw belt 490-5M-15 (28T : 56T)", "cradle", "drive", "belt", belt_solid(Pb, Z_BELT[0] + 1.5, Z_BELT[1] - 1.5, "xy"), mass=0.03)
    abd = RS06 if kind == "fore" else RS02
    xm1 = X_LUG_R[0] - 5.0; xa0 = xm1 - abd["t"]
    L.add("abduction stubs 15 mm (rear on the motor, front in a 6802)", "cradle", "hardware", "steel",
          cyl_x(7.5, xm1 + 0.1, X_LUG_R[1], ya, ABD_AX[1]) + cyl_x(7.5, X_LUG_F[0], X_LUG_F[1] + 11, ya, ABD_AX[1]), mass=0.05)
    # ---- carrier: two brackets on the body frame (abduction motor + rear journal, front journal).  No beam: anything
    # under or beside the stack is in the leg's way when it yaws inward.  The front sling rocker is not in this build.
    rear = hull(cyl_x(abd["r"] + 3, xa0 - 8, xa0, ya, ABD_AX[1]), box(xa0 - 8, xa0, ya - 45, ya - 20, ABD_AX[1] - 30, ABD_AX[1] + 30))
    xb = X_LUG_F[1] + 2
    front = hull(cyl_x(16, xb, xb + 10, ya, ABD_AX[1]), box(xb, xb + 10, ya - 30, ya - 14, ABD_AX[1] - 12, ABD_AX[1] + 40))
    front = front - hole_x(12, xb - 0.1, xb + 7, ya, ABD_AX[1]) - cyl_x(9.5, xb - 1, xb + 11, ya, ABD_AX[1])
    L.add("hip bracket, rear (abduction motor mount)", "rocker", "structure", "pacf", rear, True)
    L.add("hip bracket, front (journal)", "rocker", "structure", "pacf", front, True)
    L.add("abduction motor " + ("RS06" if abd is RS06 else "RS02"), "rocker", "actuators", "act", cyl_x(abd["r"], xa0, xm1, ya, ABD_AX[1]), mass=abd["m"])
    L.add("abduction bearing 6802", "rocker", "hardware", "bearing", cyl_x(12, xb + 2, xb + 7, ya, ABD_AX[1]) - hole_x(7.5, xb + 1, xb + 8, ya, ABD_AX[1]), mass=0.007)


def middle_arms(L, zS, zJ, tag):
    """Middle segment (humerus / tibia): a 25 mm shaft through the upper segment's bearings with one arm each side of it;
    each arm carries a 6800 for the 10 mm pin of the next joint."""
    L.add(tag + " shaft 25 mm", "middle", "hardware", "steel", cyl_y(12.5, Y_ARM_I[0], Y_ARM_O[1], 0, zS), mass=0.09)
    for nm, yy, seat in (("outer", Y_ARM_O, (Y_ARM_O[0] - 0.1, Y_ARM_O[0] + 5)), ("inner", Y_ARM_I, (Y_ARM_I[1] - 5, Y_ARM_I[1] + 0.1))):
        arm = plate_y([(0, zS, 18), (0, zJ, 14)], *yy) - hole_y(12.5, yy[0] - 1, yy[1] + 1, 0, zS) - hole_y(9.5, seat[0], seat[1], 0, zJ) - cyl_y(7.0, yy[0] - 1, yy[1] + 1, 0, zJ)
        L.add(f"{tag} arm, {nm}", "middle", "structure", "pacf", arm, True)
    L.add(tag + " pin bearings 2 x 6800", "middle", "hardware", "bearing",
          cyl_y(9.5, Y_ARM_O[0], Y_ARM_O[0] + 5, 0, zJ) + cyl_y(9.5, Y_ARM_I[1] - 5, Y_ARM_I[1], 0, zJ) - hole_y(5, Y_ARM_I[0] - 1, Y_ARM_O[1] + 1, 0, zJ), mass=0.01)
    xt = R_DRUM + 1.5 + FIT                                                 # tendon centre line: lying on the drums
    L.add(("elbow" if tag == "humerus" else "hock") + " tendons Dyneema 3 mm (parallel pair)", "middle", "drive", "line",
          union([rod([sx * xt, y, zS], [sx * xt, y, zJ], 1.5, 12) for sx, y in ((1, -28), (-1, -22))]), mass=0.004)


def gas_spring(L, zS, q_lo, q_hi):
    """Offload strut between a stud on plate B and a stud on the middle segment's outer arm, in the plane y = 45.
    6/15 gas spring, 30 mm stroke: 114 mm extended / 84 mm compressed between the eye centres (est. from the series' 2 x stroke + 32 body
    plus two 11 mm eyes).  Anchors from a grid search: inside the stroke over the joint range, most lever at the stand."""
    Le, Lc = 114.0, 84.0; L1 = L.g["L1"]; q = np.radians(np.arange(q_lo - 3, q_hi + 3.1, 1.0)); best = None
    for rho in (18, 20, 22, 24, 26):
        for phi in np.radians((-20, -10, 0, 10, 20)):
            mx, mz = rho * np.sin(phi), -rho * np.cos(phi)
            Mx, Mz = mx * np.cos(q) + mz * np.sin(q), -mx * np.sin(q) + mz * np.cos(q)
            for ux in np.arange(-38, 38.1, 2):
                for uz in np.arange(60, L1 - 56 + 0.1, 2):
                    if np.hypot(ux, L1 - uz) < RS06["r"] + 7.8 + 3: continue
                    Ls = np.hypot(Mx - ux, Mz - uz)
                    if Ls.max() > Le - 1.5 or Ls.min() < Lc + 1.5 or np.any(np.diff(Ls) > 0): continue
                    arm = np.abs((Mx * (Mz - uz) - Mz * (Mx - ux))) / Ls            # lever of the strut line about the joint
                    sc = arm.min()
                    if best is None or sc > best[0]: best = (sc, rho, phi, ux, uz, Ls.max(), Ls.min(), arm)
    sc, rho, phi, ux, uz, lmax, lmin, arm = best
    U = [float(ux), 45.0, float(zS + uz)]; M = [float(rho * np.sin(phi)), 45.0, float(zS - rho * np.cos(phi))]
    L.add("gas spring stud, upper (8 mm)", "upper", "hardware", "steel", cyl_y(4, Y_B[1], 53.0, U[0], U[2]), mass=0.012)
    L.add("gas spring stud, middle (6 mm)", "middle", "hardware", "steel", cyl_y(3, Y_ARM_O[1], 53.0, M[0], M[2]), mass=0.007)
    nm = ("shoulder" if L.kind == "fore" else "stifle") + " gas spring 6/15, 30 stroke, 400 N"
    L.struts.append(dict(name=nm, a=("upper", U), b=("middle", M), r=7.8, col=COL["gas"], trim=(9.0, 9.0), group="hardware"))
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
    L.add(tag + " fetlock pin 6 mm", node, "hardware", "steel", cyl_y(3, -13, 13, fx, fz), mass=0.006)
    L.add(tag + " fetlock buffer (TPU 95A)", node, "structure", "tpu", box(fx + 13, fx + 27, -7, 7, fz + 8.45, fz + 16), True)
    L.add(tag + " pastern (flexor horn, buffer horn)", "pastern", "structure", "pacf", past, True)
    L.add(tag + " hoof ball joint SKF GE 8 E", "pastern", "hardware", "bearing", Manifold.sphere(8, 48).translate(list(ball)), mass=0.008)
    core = cyl_z(25, zF + 6, zF + 20, r1=21).translate([ball[0], 0, 0]) - Manifold.sphere(8.3, 48).translate(list(ball))
    L.add(tag + " hoof core", "hoof", "structure", "hoof", core, True)
    L.add(tag + " hoof shoe (TPU 95A)", "hoof", "structure", "tpu", cyl_z(26, zF, zF + 6).translate([ball[0], 0, 0]), True)
    L.add(tag + " hoof centring ring (TPU)", "hoof", "structure", "tpu", cyl_z(17, zF + 20, zF + 25).translate([ball[0], 0, 0]) - cyl_z(11.5, zF + 19, zF + 26).translate([ball[0], 0, 0]), True)
    return can, fet, ball, Q


def fetlock_servo(L, node, x0, z0, tag, horn):          # horn: the crank's line point Q
    """STS3215 on the cranial face (x0 = its back face), shaft along y, spool on the -y end; flexor line to the horn's cross pin."""
    L.add(f"fetlock servo STS3215 (on the {tag})", node, "actuators", "servo", box(x0, x0 + STS["b"], -STS["c"] / 2, STS["c"] / 2, z0, z0 + STS["a"]), mass=STS["m"])
    sc = [x0 + 12.0, z0 + 9.0]
    L.add(tag + " flexor spool", node, "drive", "steel", cyl_y(9, -27.5, -17.7, sc[0], sc[1]), mass=0.006)
    L.struts.append(dict(name=tag + " fetlock flexor line 2 mm", a=("tan", node, [sc[0], -23.0, sc[1]], 9.0 + 1.0 + FIT, "maxz"), b=("pastern", [float(horn[0]), -23.0, float(horn[2])]),
                         r=1.0, col=COL["line"], trim=(0.0, 3.6), group="drive"))


def build_fore():
    L = Leg("fore"); g = L.g; L1, L2, LC, L3 = g["L1"], g["L2"], g["LC"], g["L3"]
    upper_stack(L, RS03)
    zS, zE, zC = -L1, -L1 - L2, -L1 - L2 - LC
    OD36, OD54 = pd(36) / 2 - T_IN, pd(54) / 2 - T_IN
    # shoulder drive: 36T on the joint motor, 54T on the humerus shaft, 525-5M-25 belt between the plates with an idler on the +x span
    f = lambda dl: path_len(belt_path((0, 0), pd(36) / 2, (0, zS), pd(54) / 2, dl, side=1)[0]) - 5.0 * SH_BELT_T
    dl = solve(f, 0.5, 25.0); Pb, Kk, mo = belt_path((0, 0), pd(36) / 2, (0, zS), pd(54) / 2, dl, side=1)
    L.mech["belt"] = dict(teeth=SH_BELT_T, deflect_mm=float(dl), slack_free_len=float(path_len(belt_path((0, 0), pd(36) / 2, (0, zS), pd(54) / 2)[0])))
    belt = belt_solid(Pb, -11.5, 13.5, "xz")
    ic = Kk + mo * (8.0 + T_OUT + 0.3)
    L.add("shoulder drive pulley 36T", "drive", "drive", "steel", cyl_y(OD36, -11.5, 13.5) + cyl_y(12, 13.5, 20.9), mass=0.05)
    L.add("shoulder belt 525-5M-25 (36T : 54T)", "upper", "drive", "belt", belt, mass=0.06)
    L.add("belt idler 16 mm on a slotted pin", "upper", "hardware", "steel", cyl_y(8, -11.5, 13.5, ic[0], ic[1]) + cyl_y(2.5, Y_A[1], Y_B[0], ic[0], ic[1]), mass=0.02)
    L.add(L.up_name, "upper", "structure", "orange", L.up_solid, True)
    L.add("shoulder bearings 2 x 6805", "upper", "hardware", "bearing", cyl_y(18.5, *Y_A, 0, zS) + cyl_y(18.5, *Y_B, 0, zS) - hole_y(12.5, -20, 22, 0, zS), mass=0.042)
    L.add("shoulder pulley 54T", "middle", "drive", "steel", cyl_y(OD54, -11.5, 13.5, 0, zS) - hole_y(12.5, -12, 14, 0, zS), mass=0.09)
    middle_arms(L, zS, zE, "humerus")
    gas_spring(L, zS, 50, 132)
    # ---- forearm: elbow head, tube, C-shaped carpus fork (open behind, so the folded cannon has room), toggle tab A
    cheek = lambda y0, y1: hull(cyl_y(11, y0, y1, 0, zC), cyl_y(11, y0, y1, 13, zC + 30)) + hull(cyl_y(11, y0, y1, 13, zC + 30), cyl_y(11, y0, y1, 0, zC + 56))
    tabA = hull(cyl_y(6, 13, 17.5, 13, zC + 30), cyl_y(6, 13, 17.5, -TOG_E, zC + TOG_A))
    fa = union([column_head(zE), rod([0, 0, zE - 28], [0, 0, zC + 52], 11), box(-11, 11, -13, 13, zC + 48, zC + 60), cheek(8, 13), cheek(-13, -8), tabA])
    fa = fa - hole_y(5, -14, 14, 0, zC) - hole_y(4, 12, 18, -TOG_E, zC + TOG_A)
    L.add("forearm (elbow head, drum B, carpus fork)", "lower", "structure", "carbon", fa, True)
    L.add("elbow pin 10 mm", "lower", "hardware", "steel", cyl_y(5, Y_ARM_I[0], Y_ARM_O[1], 0, zE), mass=0.05)
    L.add("carpus pin 10 mm (its +y end is the toggle's stop)", "lower", "hardware", "steel", cyl_y(5, -21.5, 26.5, 0, zC), mass=0.03)
    L.add("carpus return spring (torsion, on the pin)", "lower", "hardware", "spring", cyl_y(8.5, -21, -14, 0, zC) - hole_y(5, -22, -13, 0, zC), mass=0.006)
    L.add("toggle pin A 8 mm", "lower", "hardware", "steel", cyl_y(4, 13, 22.6, -TOG_E, zC + TOG_A), mass=0.004)
    zs0 = zC + 58
    L.add("carpus servo STS3215 (on the forearm)", "lower", "actuators", "servo", box(-11.5 - STS["b"], -11.5, -20, 15, zs0, zs0 + STS["a"]), mass=STS["m"])
    L.add("carpus release spool", "lower", "drive", "steel", cyl_y(9, 15.2, 30, -24, zs0 + 36), mass=0.006)
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
    fetlock_servo(L, "cannon", 12.5, zC - 59.2, "fore cannon", horn)
    lk = float(np.hypot(TOG_A, TOG_E - (10.0 + FIT)))
    L.toggles.append(dict(name="carpus toggle", a=("lower", [-TOG_E, 0, zC + TOG_A]), b=("cannon", [-TOG_E, 0, zC - TOG_A]), l=lk, r=3.5, col="#c0392b",
                          y1=(19.0, 22.4), y2=(22.8, 26.2), ypin=(19.0, 29.5), lock_below=1e-3))
    L.struts.append(dict(name="carpus release line 1.5 mm", a=("tan", "lower", [-24.0, 28.0, zs0 + 36], 9.0 + 0.75 + FIT, "minx"), b=("knee", 0, 28.0), r=0.75, col=COL["line"], trim=(0.0, 3.6), group="drive"))
    L.joints = dict(zS=zS, zE=zE, zC=zC, fet=fet, ball=ball)
    return L


def build_hind():
    L = Leg("hind"); g = L.g; L1, L2, L3 = g["L1"], g["L2"], g["L3"]
    upper_stack(L, RS06)
    zS, zH = -L1, -L1 - L2
    # stifle drive: 1:1 parallelogram push rod, cranks 40 mm between the plates.  Rest crank direction: perpendicular to the femur at q1 = 110 deg
    a0 = -110.0 * D2R
    c0 = 40.0 * np.array([np.cos(a0), 0.0, -np.sin(a0)])
    crank = lambda z: hull(cyl_y(14, -11.5, -2, 0, z), cyl_y(7, -11.5, -2, c0[0], z + c0[2])) + cyl_y(4, -11.5, 11.5, c0[0], z + c0[2])
    L.add("stifle drive crank 40 mm", "drive", "drive", "steel", crank(0.0) + cyl_y(10, -2, 20.9), mass=0.05)
    eye = lambda z: cyl_y(8, 0, 10, c0[0], z + c0[2]) - hole_y(4, -1, 11, c0[0], z + c0[2])
    L.add("stifle push rod 10 x 8 carbon, rod ends", "rodn", "drive", "carbon", union([rod(c0 + [0, 5, -7.5], c0 + [0, 5, zS + 7.5], 5), eye(0.0), eye(zS)]), mass=0.03)
    L.add(L.up_name, "upper", "structure", "orange", L.up_solid, True)
    L.add("stifle bearings 2 x 6805", "upper", "hardware", "bearing", cyl_y(18.5, *Y_A, 0, zS) + cyl_y(18.5, *Y_B, 0, zS) - hole_y(12.5, -20, 22, 0, zS), mass=0.042)
    L.add("stifle driven crank 40 mm", "middle", "drive", "steel", crank(zS) - hole_y(12.5, -12, -1, 0, zS), mass=0.04)
    middle_arms(L, zS, zH, "tibia")
    gas_spring(L, zS, 50, 146)
    z_top = zH - 28; zF = z_top - (L3 - 28); fz = zF + HOOF_H + L_PAST * np.cos(A_PAST * D2R); fx = -L_PAST * np.sin(A_PAST * D2R)
    body = union([column_head(zH), rod([0, 0, zH - 28], [fx, 0, fz + 20], 11),
                  hull(box(11.5, 12.45, -12, 12, zH - 84, zH - 40), rod([0.4 * fx, 0, zH - 48], [0.9 * fx, 0, zH - 76], 5))])
    can, fet, ball, horn = distal(L, "lower", body, z_top, L3 - 28, "hind")
    L.add("hind cannon (hock head, drum B, fetlock fork)", "lower", "structure", "carbon", can, True)
    L.add("hock pin 10 mm", "lower", "hardware", "steel", cyl_y(5, Y_ARM_I[0], Y_ARM_O[1], 0, zH), mass=0.04)
    fetlock_servo(L, "lower", 12.5, zH - 84.5, "hind cannon", horn)
    L.joints = dict(zS=zS, zH=zH, fet=fet, ball=ball, c0=c0)
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
