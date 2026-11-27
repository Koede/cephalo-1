"""Equo-1 chassis v3: clean horse profile with a designed shoulder girdle (front bulkhead that carries the two front
hip pods and the neck socket block), scapula fairings over the front pods, hip fairings over the rear pods.
Same collar position and tilt as v2 (equo_v2 / sketch_body C).  Metres, x forward, y left, z up."""
import sys, io, contextlib

import numpy as np, sk3d
with contextlib.redirect_stdout(io.StringIO()):
    import equo_v2 as V
C = sk3d.COL
import os
S = float(os.environ.get("EQUO_S", "1.0"))   # horse scale: body, legs, positions scale; bought parts, octopus, collar,
                                             # neck sheaths keep their size (arm_scale_study.py picks S)
V.cc = V.cc * S
cc = V.cc; n_up = V.n_up; TILT = V.TILT
band = np.array([n_up[1], -n_up[0]])                       # along the collar plane, pointing forward-down
cl, cr = cc - 0.118 * band, cc + 0.118 * band
# leg tracks (hip plane |y|): front narrowed so the chest is slimmer; the front abduction actuators then sit AHEAD of the
# front hips (behind them is the neck sheath block).  Abduction actuator: axis x, 75..124 mm from the hip axis, 30 mm up,
# r 44 (RS06), 70 mm inboard of the hip plane.
ABD_MODE = os.environ.get("EQUO_ABDMODE", "tube")    # "tube": front abduction actuators behind the neck sheath, torque tubes to the hips
# actuator choice at this scale (per-joint check in build_preview): RobStride 06 front hip pitch (was 03), RobStride 02 rear
# abduction (was 06); everything else stays RS06.  (name, rim radius, length along the axis, mass, rated, peak)
ACT = {"RS03": ("RobStride 03", 0.053, 0.056, 0.880, 20.0, 60.0), "RS06": ("RobStride 06", 0.044, 0.049, 0.621, 11.0, 36.0),
       "RS02": ("RobStride 02", 0.03925, 0.0455, 0.405, 6.0, 17.0)}
SEL = {("front", "hip"): os.environ.get("EQUO_HIPF", "RS06"), ("front", "knee"): "RS06", ("front", "abd"): "RS06",
       ("rear", "hip"): "RS06", ("rear", "knee"): "RS06", ("rear", "abd"): os.environ.get("EQUO_ABDR", "RS02")}
HIPF_R, HIPF_L = ACT[SEL[("front", "hip")]][1:3]
ABDR_R, ABDR_L = ACT[SEL[("rear", "abd")]][1:3]
ABD_X1_R = 0.075 + ABDR_L
# knee drive: "scapula" = the knee RS06 sits IN the femur plane on a short arm 72 mm above the hip (shoulder-blade style),
# belt plane just outboard (30T there, 60T at the stifle, longer belt); the pauldron covers it.  The thigh is then only the
# belt plane wide outboard, but the motor's inner face (hip plane - 24.5) sweeps past the body, so the pod faces sit
# POD_IN below the hip plane and the hip pitch actuators 10 mm further in.
KNEE_MODE = os.environ.get("EQUO_KNEE", "scapula")
R_K = 0.072                                             # knee actuator axis above the hip, along the femur line
POD_IN = 0.0395 if KNEE_MODE == "scapula" else 0.005    # pod face below the hip plane: thigh inner face clears the front girth bulge by 5
ACT_OUT_IN = POD_IN + 0.010                              # hip pitch actuator outer face below the hip plane
Y_LEG_F = float(os.environ.get("EQUO_YF", f"{0.075 + 0.005 + HIPF_L + ACT_OUT_IN:.4f}" if ABD_MODE == "tube" else "0.190"))   # receiver 150 sq at the hip line   # block hw + 5 + RS03 + ...
# rear track matched to the front: the rear abduction actuators move BEHIND the rear hips (into the croup), axis 99 mm off
# the centreline; the swing arms (their carriers) run inside the hips at |y| 75 mm.
Y_LEG_R = float(os.environ.get("EQUO_YR", str(Y_LEG_F)))
ABD_DIR_R = -1.0 if Y_LEG_R < 0.19 else 1.0          # -1: behind the rear hip
ABD_IN_R = Y_LEG_R - min(0.099, Y_LEG_R - POD_IN - 0.010 - ABDR_R) if ABD_DIR_R < 0 else 0.070   # rear abduction stays inside the pod
ABD_DZ_R = 0.065 if ABD_DIR_R < 0 else 0.03          # rear abduction axis centred in the rump section
SW_Y = 0.0715 if ABD_DIR_R < 0 else 0.080          # swing arms (r 11) end 3 mm inside the rear pitch actuators
BC_Y = 0.030 if ABD_DIR_R < 0 else 0.078            # bell-crank and shock line: inboard of the front abduction actuators (inner face 49)
ABD_DIR_F = float(os.environ.get("EQUO_ABDF", "-1.0" if (ABD_MODE == "tube" or Y_LEG_F >= 0.185) else "1.0"))   # +1: ahead of the front hip
ABD_X0, ABD_X1, ABD_DZ, ABD_R = 0.075, 0.124, 0.03, 0.044
ABD_IN = float(os.environ.get("EQUO_ABDIN", "0.059" if Y_LEG_F < 0.199 else "0.070"))      # abduction axis inboard of the hip plane
HIPF = np.array([0.292, 0.490]) * S; HIPR = np.array([-0.200, 0.504]) * S

# ------------------------------------------------------------------ side profile (control points, closed, clockwise from the dock)
CTRL = np.array([
    (-0.372 * S, 0.600 * S), (-0.310 * S, 0.655 * S), (-0.180 * S, 0.668 * S), (-0.040 * S, 0.660 * S), (0.050 * S, 0.675 * S),   # dock, croup, back, withers
    tuple(cl - 0.020 * band - 0.03 * n_up), tuple(cl), tuple(cr),                          # back of the neck, collar
    tuple(cr - 0.035 * n_up + 0.012 * band), (0.372 * S, 0.600 * S), (0.398 * S, 0.540 * S),               # throat, breast
    (0.392 * S, 0.478 * S), (0.355 * S, 0.430 * S), (0.270 * S, 0.408 * S), (0.130 * S, 0.410 * S), (-0.020 * S, 0.420 * S),        # point of shoulder, brisket, girth, belly
    (-0.150 * S, 0.440 * S), (-0.255 * S, 0.470 * S), (-0.330 * S, 0.515 * S), (-0.368 * S, 0.560 * S)])                    # flank, stifle fold, buttock

def catmull(P, n=14):
    out = []
    for i in range(len(P)):
        p0, p1, p2, p3 = P[i - 1], P[i], P[(i + 1) % len(P)], P[(i + 2) % len(P)]
        for t in np.linspace(0, 1, n, endpoint=False):
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t ** 2 + (-p0 + 3 * p1 - 3 * p2 + p3) * t ** 3))
    return np.array(out)
OUT = catmull(CTRL)
# the neck sheath block (fixed size: 228 deep, r 82 = bars at r 70 + 6.5 + wall) must sit inside the profile: wherever the
# outline would cut it, the outline moves out to the block + 14 mm (wall + gap).  At full size nothing moves.
# neck sheath block: four sheaths (46 x 15 inside, 3 mm walls) for the 37 x 13 blade shanks at r 70, 45 deg to the body
# axis -> they sit at the corners of a square; the block is a rounded square, half-width 74 mm (was a r 82 cylinder)
SHEATH_L, SHEATH_M = 0.228, 0.014
SHEATH_HW = round(0.070 * np.cos(np.pi / 4) + (0.023 + 0.0075) * np.cos(np.pi / 4) + 0.003, 4)     # 0.0741
SHEATH_R = SHEATH_HW            # half-width in both directions across the axis (square section)
_end = cc - n_up * SHEATH_L
# receiver cartridge envelope (centaur-neck.md sec. 6), by depth below the seat (= the collar top, plate underside):
#   crown ring dia 236 (pads, cup wall)          0 .. 33
#   throat 192 sq (deck, 4 funnels 21 deep, horn guides, wedge bars and their springs; horn feet end ~75)   33 .. 80
#   sheath block 150 sq (lower funnels-to-sheaths, bar bolts at the eyelet level, cassette bay)            80 .. 228
RECV = [(0.033, 0.118), (0.080, 0.096), (SHEATH_L + 0.001, 0.075)]
def recv_sdf(q):
    """signed distance of a 3D point (x, y, z) to the receiver cartridge (stepped square / crown), m"""
    e0 = np.array([_end[0], 0, _end[1]]); w = np.array([n_up[0], 0, n_up[1]]); u = np.array([n_up[1], 0, -n_up[0]])
    r = np.asarray(q) - e0; a = r @ w; b = r @ u; c = r[1]; hw = recv_hw(np.clip(a, 0, SHEATH_L))
    d = np.array([abs(b) - hw, abs(c) - hw, max(-a, a - SHEATH_L)])
    return np.linalg.norm(np.maximum(d, 0)) + min(d.max(), 0)

def recv_hw(a):
    """half-width of the receiver at axial position a above the sheath end (0 .. SHEATH_L)"""
    d = SHEATH_L - a
    for dmax, hw in RECV:
        if d <= dmax: return hw
    return RECV[-1][1]
for _ in range(2):
    for i, p in enumerate(OUT):
        a = (p - _end) @ n_up; b = (p - _end) @ band
        if -SHEATH_M < a < SHEATH_L and abs(b) < recv_hw(a) + SHEATH_M:
            mv_b, mv_a = recv_hw(a) + SHEATH_M - abs(b), a + SHEATH_M
            OUT[i] = p + (np.sign(b if b != 0 else 1) * band * mv_b if mv_b < mv_a else -n_up * mv_a)
    OUT = np.array([OUT[i - 1] * 0.25 + OUT[i] * 0.5 + OUT[(i + 1) % len(OUT)] * 0.25 for i in range(len(OUT))])
if ABD_DIR_F > 0:          # the breast moves forward just enough to cover the forward abduction actuators (+ 14 mm)
    _x1 = HIPF[0] + ABD_X1 + SHEATH_M; _z0, _z1 = HIPF[1] + ABD_DZ - ABD_R, HIPF[1] + ABD_DZ + ABD_R
    for _ in range(8):
        def _front_x(zq):
            dz = np.abs(OUT[:, 1] - zq); nb = OUT[dz < max(0.006, np.sort(dz)[3])]
            return nb[:, 0].max()
        d = _x1 - min(_front_x(zq) for zq in np.linspace(_z0, _z1, 7))
        if d <= 0.001: break
        zm = 0.5 * (_z0 + _z1)
        for j in range(len(CTRL)):
            if CTRL[j, 0] > HIPF[0] - 0.02 and CTRL[j, 1] < cc[1] - 0.05:
                CTRL[j, 0] += d * 1.1 * np.exp(-((CTRL[j, 1] - zm) / 0.12) ** 2)
        OUT = catmull(CTRL)
        _end = cc - n_up * SHEATH_L
        for _k in range(2):
            for i, p in enumerate(OUT):
                a = (p - _end) @ n_up; b = (p - _end) @ band
                if -SHEATH_M < a < SHEATH_L and abs(b) < recv_hw(a) + SHEATH_M:
                    mv_b, mv_a = recv_hw(a) + SHEATH_M - abs(b), a + SHEATH_M
                    OUT[i] = p + (np.sign(b if b != 0 else 1) * band * mv_b if mv_b < mv_a else -n_up * mv_a)
            OUT = np.array([OUT[i - 1] * 0.25 + OUT[i] * 0.5 + OUT[(i + 1) % len(OUT)] * 0.25 for i in range(len(OUT))])

def hits(p, d):
    ts = []
    for a, c in zip(OUT, np.roll(OUT, -1, axis=0)):
        e = c - a; M = np.array([d, -e]).T
        if abs(np.linalg.det(M)) < 1e-12: continue
        t, u = np.linalg.solve(M, a - p)
        if 0 <= u <= 1: ts.append(t)
    return sorted(ts)

# spine: along the body, then up the neck axis; rings perpendicular to it
base = cc - n_up * 0.22
body = []
for x in np.linspace(-0.345 * S, base[0] - 0.02, 14):
    t = hits(np.array([x, 0.0]), np.array([0, 1.0])); body.append(np.array([x, 0.5 * (t[0] + t[-1])]))
SPINE = np.array(body + [base + (cc - base) * u for u in np.linspace(0.25, 1.0, 5)])
Ls = np.r_[0, np.cumsum(np.linalg.norm(np.diff(SPINE, axis=0), axis=1))]; us = Ls / Ls[-1]
W = np.interp(us, [0, 0.07, 0.25, 0.5, 0.68, 0.78, 0.88, 1.0], [0.06, 0.12, 0.14, 0.15, 0.15, 0.14, 0.128, 0.118])
RINGS = []
for i, p in enumerate(SPINE):
    tg = SPINE[min(i + 1, len(SPINE) - 1)] - SPINE[max(i - 1, 0)]; tg /= np.linalg.norm(tg); nr = np.array([-tg[1], tg[0]])
    t = hits(p, nr); lo, hi = (t[0], t[-1]) if len(t) >= 2 else (-0.05, 0.05)
    if i == len(SPINE) - 1: lo, hi = -0.118, 0.118
    c2 = p + nr * 0.5 * (lo + hi); h = min(0.5 * (hi - lo), 0.15)
    th = np.linspace(0, 2 * np.pi, 29)
    RINGS.append(np.array([[c2[0] + nr[0] * h * np.sin(a), W[i] * np.cos(a), c2[1] + nr[1] * h * np.sin(a)] for a in th]))

if ABD_DIR_R < 0:          # the rump reaches just past the rear abduction actuators behind the rear hips (+ 14 mm)
    _xr = HIPR[0] - ABD_X1_R - 0.050; _zr0, _zr1 = HIPR[1] + ABD_DZ_R - ABDR_R, HIPR[1] + ABD_DZ_R + ABDR_R
    for _ in range(8):
        def _rear_x(zq):
            dz = np.abs(OUT[:, 1] - zq); nb = OUT[dz < max(0.006, np.sort(dz)[3])]
            return nb[:, 0].min()
        d = min(_rear_x(zq) for zq in np.linspace(_zr0, _zr1, 7)) - _xr
        if d <= 0.001: break
        zm = 0.5 * (_zr0 + _zr1)
        for j in range(len(CTRL)):
            if CTRL[j, 0] < HIPR[0] + 0.02:
                CTRL[j, 0] -= d * 1.1 * np.exp(-((CTRL[j, 1] - zm) / 0.12) ** 2)
        OUT = catmull(CTRL)
        for _k in range(2):                                   # re-apply the sheath-block fit to the rebuilt outline
            for i, p in enumerate(OUT):
                a = (p - _end) @ n_up; b = (p - _end) @ band
                if -SHEATH_M < a < SHEATH_L and abs(b) < recv_hw(a) + SHEATH_M:
                    mv_b, mv_a = recv_hw(a) + SHEATH_M - abs(b), a + SHEATH_M
                    OUT[i] = p + (np.sign(b if b != 0 else 1) * band * mv_b if mv_b < mv_a else -n_up * mv_a)
            OUT = np.array([OUT[i - 1] * 0.25 + OUT[i] * 0.5 + OUT[(i + 1) % len(OUT)] * 0.25 for i in range(len(OUT))])

# ---- front abduction by torque tube (ABD_MODE "tube"): RS06 behind the sheath block, coaxial 20 x 2 steel tube forward to the
# hip module's rear bearing (75 mm behind the hip axis).  Axis |y| = sheath r + hub r + 4 mm, 60 mm above the hip axis.
TT_R = 0.010; TT_HUB_R = 0.015;     # 6802 (24 OD) + 3 mm clamp
TT_Y = None; TT_DZ = 0.060   # the bearing hub sets the axis
def _block_rear_x(z):                       # rear boundary of the sheath block in side view at height z
    xa = _end[0] + (z - _end[1]) * n_up[0] / n_up[1]
    a_ = (z - _end[1]) / n_up[1]
    return xa - recv_hw(np.clip(a_, 0, SHEATH_L)) / n_up[1]
TT_Z = HIPF[1] + TT_DZ
TT_X1 = HIPF[0] - 0.075
_a_hub = (np.array([TT_X1, TT_Z]) - _end) @ n_up
TT_Y = recv_hw(np.clip(_a_hub + 0.02, 0, SHEATH_L)) + TT_HUB_R + 0.004       # hub (its top edge) beside the receiver
TT_ACT_X1 = min(_block_rear_x(TT_Z - ABD_R), _block_rear_x(TT_Z + ABD_R)) - 0.006     # actuator front face
TT_ACT_X0 = TT_ACT_X1 - 0.049                                                       # RS06 is 49 mm long
TT_X1 = HIPF[0] - 0.075                                                             # tube ends at the hip module's rear bearing
KEEL_FRONT_TT = _block_rear_x(0.47 * S) - 0.006                                      # keel tubes land on the sheath block

GRAB_R = 0.141
GRAB_POST_COL = "#d9782a"

def led_ring(sc):
    """dock-state LED ring in the crown, in the (tilted) seat plane: r 122, 8 mm under the seat.  Amber = bars seated
    (Hall switches on the bolts), red = released, blue = solo / hood closed."""
    w = np.array([n_up[0], 0, n_up[1]]); u = np.array([n_up[1], 0, -n_up[0]]); v = np.array([0, 1.0, 0]); c = np.array([cc[0], 0, cc[1]]) - 0.008 * w
    sc.tube([c + 0.122 * (np.cos(a) * u + np.sin(a) * v) for a in np.linspace(0, 2 * np.pi, 49)], 0.003, "#c9661f", n=6, balls=False)

def P3(p, y=0.0): return np.array([p[0], y, p[1]])

def frame(sc, skin_alpha=0.16):
    for r in RINGS[1:]:
        sc.tube(list(r), 0.006, C["frame"], n=6, balls=False)
    for j in (0, 7, 14, 21, 4, 10, 18, 24):
        sc.tube([r[j] for r in RINGS], 0.005, C["frame"], n=6, balls=False)
    for r0, r1 in zip(RINGS[:-1], RINGS[1:]):
        for j in range(28):
            sc.poly([r0[j], r0[j + 1], r1[j + 1], r1[j]], C["horse"], alpha=skin_alpha, ec=(0, 0, 0, 0))
    # nose cap of the chest is closed by the profile; close the dock
    tl = RINGS[0]; ctr = tl[:-1].mean(0)
    for j in range(28): sc.poly([ctr, tl[j], tl[j + 1]], C["horse"], alpha=skin_alpha, ec=(0, 0, 0, 0))

GIRDLE_W = 0.135; GIRDLE_Y = 0.20; PACK_Z = 0.47; KEEL_TAIL = 0.05; KEEL_Z = 0.47 * S; KEEL_Y = 0.058; GIRDLE_PLATE = False;   # keel rails at +-58: inside the rear hip actuators (inner face 76), under the 110 mm packs
DECK_X = None; DECK_Z = None; KEEL_FRONT = KEEL_FRONT_TT if ABD_MODE == "tube" else None; PACK_X = None; CROSS_DZ = 0.0
def girdle(sc):
    """Shoulder girdle: a PA-CF bulkhead across the chest at the front hip line, carrying both front hip pods on a
    cross tube; struts up to the neck socket block; the keel tubes end in it."""
    x, z = HIPF
    # bulkhead plate dropped: the neck sheath block now passes the hip line, so the girdle is the cross tube + struts + block
    if not GIRDLE_PLATE: th = []
    t = hits(np.array([x, 0.0]), np.array([0, 1.0])); zb, zt = t[0] + 0.01, min(t[-1], 0.64 * S)
    th = np.linspace(0, 2 * np.pi, 33) if GIRDLE_PLATE else np.zeros(0); zc, hz = 0.5 * (zb + zt), 0.5 * (zt - zb)
    for a0, a1 in zip(th[:-1], th[1:]):
        for y0, z0, y1, z1 in ((GIRDLE_W * np.cos(a0), zc + hz * np.sin(a0), GIRDLE_W * np.cos(a1), zc + hz * np.sin(a1)),):
            sc.poly([(x - 0.006, y0, z0), (x - 0.006, y1, z1), (x - 0.006, 0.8 * y1, zc + 0.8 * (z1 - zc)), (x - 0.006, 0.8 * y0, zc + 0.8 * (z0 - zc))], "#6f7c8a")
    sc.cyl([x, -GIRDLE_Y, z], [x, GIRDLE_Y, z], 0.016, C["bar"], n=14)            # cross tube: ends at the hip actuator housings
    for s in (1, -1):                                                               # struts to the neck socket block
        _p0 = np.array([x, s * 0.06, z + 0.02]); _p1 = np.array([cc[0] - 0.02, s * 0.05, cc[1] - 0.20])
        for _f in np.linspace(0, 1, 201):                                        # end the strut on the receiver's face (bolted pad)
            _q = _p0 + (_p1 - _p0) * _f
            if recv_sdf(_q) < 0.009 + 0.001: break
        sc.cyl(_p0, _p0 + (_p1 - _p0) * max(_f - 0.005, 0), 0.009, C["bar"], n=8)                          # strut to the receiver face
        sc.cyl([x - 0.02 if KEEL_FRONT is None else KEEL_FRONT, s * KEEL_Y, KEEL_Z], [HIPR[0] - KEEL_TAIL, s * KEEL_Y, KEEL_Z], 0.015, C["bar"], n=12)   # keel tubes 30 x 27
    _u = np.array([n_up[1], 0, -n_up[0]]); _v = np.array([0, 1.0, 0]); _w = np.array([n_up[0], 0, n_up[1]])
    _sq = [(np.cos(t) / max(abs(np.cos(t)), abs(np.sin(t))), np.sin(t) / max(abs(np.cos(t)), abs(np.sin(t)))) for t in np.linspace(0, 2 * np.pi, 33)]
    _rd = [(np.cos(t), np.sin(t)) for t in np.linspace(0, 2 * np.pi, 33)]
    d0 = 0.0
    if hasattr(sc, 'group'): sc.group('neck receiver')
    for dmax, hw in RECV:                          # receiver cartridge: crown ring, throat, sheath block (by depth)
        c0, c1 = P3(cc - n_up * min(dmax, SHEATH_L)), P3(cc - n_up * d0)
        shp = _rd if hw > 0.11 else _sq
        r0 = [c0 + hw * (a * _u + b * _v) for a, b in shp]; r1 = [c1 + hw * (a * _u + b * _v) for a, b in shp]
        for k in range(32): sc.poly([r0[k], r0[k + 1], r1[k + 1], r1[k]], "#7b8794", alpha=0.35)
        d0 = dmax
    if hasattr(sc, 'ungroup'): sc.ungroup()
    _top = P3(cc)
    for ang in (0, 90, 180, 270):                  # draw horns (on the axes, r 70, 25 above the seat) and rim posts (R 125.4..139, 48 tall)
        e = np.cos(np.radians(ang)) * _u * -1 + np.sin(np.radians(ang)) * _v
        sc.cyl(_top + e * 0.070, _top + e * 0.070 + _w * 0.025, 0.006, "#c9661f", n=8)
        if ang in (90, 270):        # side posts are the octopus's grab posts: dia 20 TPU-sleeved, 48 tall, dia 28 cap, at R 141 so the
            # cap clears the plate rim (r 125) by 2 mm and the post body by 6 mm -- the plate never touches them
            sc.cyl(_top + e * GRAB_R, _top + e * GRAB_R + _w * 0.048, 0.010, GRAB_POST_COL, n=12)
            sc.sphere(_top + e * GRAB_R + _w * 0.052, 0.014, GRAB_POST_COL, n=8)
        else:
            sc.cyl(_top + e * 0.132, _top + e * 0.132 + _w * 0.048, 0.007, "#7b8794", n=8)
    sc.box(P3(np.array([HIPR[0] + 0.09 if PACK_X is None else PACK_X, PACK_Z])), (0.16, 0.11, 0.05), fc=C["pack"])   # packs on the sled (10 mm above the belly pan)
    sc.box([0.0 if DECK_X is None else DECK_X, 0, KEEL_Z + 0.03 if DECK_Z is None else DECK_Z], (0.10, 0.07, 0.02), fc=C["spring"])                                         # controller deck

def fairings(sc):
    """Scapula fairings over the front pods (sloping back from the point of shoulder to the withers, like a horse's
    shoulder blade) and round hip fairings over the rear pods.  Thin printed shells, est. 60 g each."""
    for s in (1, -1):
        base_ = np.array([HIPF[0] + 0.01, s * 0.215, HIPF[1] - 0.01]); top = np.array([0.10, s * 0.165, 0.655])
        ax = top - base_; L = np.linalg.norm(ax); ax /= L
        u = np.cross(ax, [0, 1.0, 0]); u /= np.linalg.norm(u); v = np.cross(ax, u)
        for i in range(10):
            for j in range(16):
                def pt(ii, jj):
                    t = ii / 10; a = 2 * np.pi * jj / 16
                    w = 0.075 * np.sin(np.pi * (0.15 + 0.85 * t)) + 0.02; d = 0.045 * np.sin(np.pi * (0.15 + 0.85 * t)) + 0.01
                    return base_ + ax * L * t + u * w * np.cos(a) + v * d * np.sin(a) * 0.7
                sc.poly([pt(i, j), pt(i + 1, j), pt(i + 1, j + 1), pt(i, j + 1)], "#4f6f93", alpha=0.55, ec=(0, 0, 0, 0))
        c = np.array([HIPR[0] - 0.01, s * 0.20, HIPR[1] + 0.015])
        n0 = len(sc.polys); sc.dome(c, 0.085, 0.07, "#4f6f93", alpha=0.55, n=9)
        R = sk3d.rot("x", -s * 90)
        for k in range(n0, len(sc.polys)):
            sc.polys[k] = (sc.polys[k] - c) @ R.T * [1, 1, 1] + c

def tail(sc):
    t0 = np.array([OUT[:, 0].min() + 0.009, 0, 0.60 * S])            # tail root at the rump
    pts = [t0, t0 + [-0.08, 0, 0.03], t0 + [-0.15, 0, 0.0], t0 + [-0.19, 0, -0.08], t0 + [-0.18, 0, -0.17]]
    sc.tube(pts, 0.013, C["octo2"], n=8)
    for q in pts[1:-1]: sc.sphere(q, 0.018, C["act"], n=5)
    sc.sphere(pts[-1], 0.022, C["tpu"], n=6)

def checks():
    """Front-end checks: hip inside the chest, sheath inside the neck, pod clearances (mm)."""
    t = hits(HIPF, np.array([0, 1.0])); tx = hits(HIPF, np.array([1.0, 0]))
    sheath_end = cc - n_up * 0.228
    from matplotlib.path import Path
    inside = Path(OUT).contains_point(sheath_end)
    return dict(hip_above_belly=-t[0] * 1e3, chest_ahead_of_hip=tx[-1] * 1e3,
                sheath_inside=bool(inside), sheath_to_hip_pod=(np.linalg.norm(sheath_end - HIPF) - 0.053) * 1e3,
                collar_x=cc[0] * 1e3, hip_x=HIPF[0] * 1e3)
