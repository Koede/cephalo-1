"""hand3 hub C return springs (drop-in for hand3e: hand3_cad.parts() = patch_parts(base_parts())), 2026-11-15.
Workspace copy of claude/code/hand3e/hand3_springs.py (geometry and ring picks; the standalone checks are left in the project copy).
Every spring lives in a small box on top of its finger's lag tube, above the hub; saddles in each knuckle's centre gap; the lag
spring is an O-ring between a peg on the lower hub plate and a post in a recess in the grip's top face."""
import os, sys, json
import numpy as np
import manifold3d as mf
from scipy.spatial import ConvexHull
import hand3_design as H
import hand3_finger as F
from hand3_finger import cyl, box, rot, T, app, about

M_ = mf.Manifold
KN = H.KN; RL = H.R_LAG; ZB = H.Z["blade"]; FAZ = H.FINGER_AZ; LS = H.LAG_SIGN
TARGET = dict(K1=0.020, K2=0.004, K3=0.020, lag=0.006)
WINDOW = dict(K1=(0.018, 0.025), K2=(0.003, 0.008), K3=(0.015, 0.025))
RATED_CORD = 31.9
ETA_EXT = 0.72
CORD_R = 0.3
SPRING_FRAC = 0.40

SAD = {0: dict(y=(-1.45, -0.35), r_s=2.0, r_in=1.0, a=(200.0, 268.0), anchor=262.0, E=(-6.95, -0.8), core=False),
       1: dict(y=(-1.45, -0.35), r_s=2.8, r_in=2.05, a=(200.0, 268.0), anchor=262.0, E=(-4.9, -0.5), core=False, chan=(49.0, -0.9)),
       2: dict(y=(0.10, 1.10), r_s=2.0, r_in=0.0, a=(200.0, 268.0), anchor=262.0, E=(-3.9, -0.4), core=True, chan=(80.5, 1.0))}
Y_K3 = 1.0
Y_K2 = H.CORD_Y[2]


def arc_wedge(m, c, a0, a1):
    a0r, a1r = np.radians(a0), np.radians(a1)
    m = F.half(m, [-np.sin(a0r), 0, np.cos(a0r)], c)
    return F.half(m, [np.sin(a1r), 0, -np.cos(a1r)], c)


def saddle(j):
    s = SAD[j]; xk = KN[j]; c = (xk, 0, 0); y0, y1 = s["y"]; r_s = s["r_s"]
    lip = r_s + 2 * CORD_R + 0.2
    ring = cyl(r_s, y0, y1, "y", c, n=96) - (cyl(s["r_in"], y0 - 1, y1 + 1, "y", c, n=48) if s["r_in"] > 0 else M_())
    m = arc_wedge(ring, c, *s["a"])
    m += arc_wedge(cyl(lip, y0, y0 + 0.25, "y", c, n=96) + cyl(lip, y1 - 0.25, y1, "y", c, n=96), c, *s["a"])
    if s["core"]:
        m += arc_wedge(cyl(r_s, y0, y1, "y", c, n=96), c, s["a"][0] - 5, s["a"][1]) + cyl(1.75, y0, y1, "y", c)
    else:
        m += arc_wedge(cyl(min(r_s + 1.2, H.BARREL[j] / 2 - 0.3), y0, y1, "y", c, n=96) - cyl(s["r_in"], y0 - 1, y1 + 1, "y", c, n=48), c, s["a"][1] - 8, 330.0)
    t = np.radians(s["anchor"]); d = np.array([np.cos(t), 0, np.sin(t)]); yc = (y0 + y1) / 2
    if not s["core"] and j > 0:
        for r in np.linspace(s["r_in"] - 0.3, r_s + 0.6, 6):
            m -= M_.sphere(0.45, 16).translate([xk + r * d[0], yc, r * d[2]])
    if s["core"]:
        for r in np.linspace(1.2, r_s + 0.6, 8):
            m -= M_.sphere(0.45, 16).translate([xk + r * d[0], yc, r * d[2]])
        pk = xk + 1.4 * d[0], 1.4 * d[2]
        side = (y1 + 0.5) if y1 < 0 else (y0 - 0.5)
        m -= box([pk[0] - 0.7, min(yc, side), pk[1] - 0.7], [pk[0] + 0.7, max(yc, side), pk[1] + 0.7])
    return m


def zback(m, x, y):
    s = m ^ box([x - 0.05, y - 0.05, -20], [x + 0.05, y + 0.05, 20])
    return float(np.asarray(s.bounding_box())[2])


def k1_anchor_cut(ph1):
    s = SAD[0]; xk = KN[0]; t = np.radians(s["anchor"]); yc = (s["y"][0] + s["y"][1]) / 2
    a = np.array([xk + (s["r_s"] + 0.6) * np.cos(t), yc, (s["r_s"] + 0.6) * np.sin(t)])
    kx = xk + 3.2; zb = zback(ph1, kx, yc); b = np.array([kx, yc, zb])
    return M_.batch_hull([M_.sphere(0.45, 16).translate(list(a)), M_.sphere(0.45, 16).translate(list(b))]) + cyl(0.75, zb - 1, zb + 0.9, c=(kx, yc, 0), n=24)


def ext_points(j, fold):
    s = SAD[j]; xk = KN[j]; rc = s["r_s"] + CORD_R
    E = np.array([xk + s["E"][0], s["E"][1]])
    t = np.radians(np.linspace(s["a"][0], s["anchor"], 120) + fold)
    return E, np.c_[xk + rc * np.cos(t), rc * np.sin(t)]


def ext_path(j, fold):
    E, arc = ext_points(j, fold)
    pts = np.vstack([E, arc]); h = ConvexHull(pts); v = list(h.vertices); iA = len(pts) - 1
    if 0 not in v or iA not in v: return None, None
    k = v.index(0); path = [pts[0]]
    while v[k] != iA: k = (k + 1) % len(v); path.append(pts[v[k]])
    path = np.array(path)
    return path, float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def ext_curve(j, n=19):
    out = []
    for f in np.linspace(0, H.FOLD_MAX[j], n):
        _, L = ext_path(j, f); _, L2 = ext_path(j, f + 0.5)
        out.append((float(f), L, (L2 - L) / np.radians(0.5)))
    return out


def ext_bore(j):
    s = SAD[j]; xk = KN[j]; ye = (s["y"][0] + s["y"][1]) / 2
    E = np.array([xk + s["E"][0] + 0.8, ye, s["E"][1]]); Cn = np.array([s["chan"][0], s["chan"][1], 0.0])
    b = M_.batch_hull([M_.sphere(0.45, 16).translate(list(E)), M_.sphere(0.45, 16).translate(list(Cn))])
    return b + M_.batch_hull([M_.sphere(0.45, 16).translate(list(E)), M_.sphere(0.45, 16).translate([xk + s["E"][0] - 0.6, ye, s["E"][1]])])


def recut_front(m, j):
    xk = KN[j]; c = (xk, 0, 0); R = H.LEVER[j]; yc = H.CORD_Y[j]
    return m - app(cyl(0.55, 1.0, R + 1.5), about("y", 100.0, c) @ T(t=[xk, yc, 0]))


X_CONV = 30.0
X_SLOT = RL + 3.4
SLOT_Z = dict(curl=-0.9, K2=-0.3, K3=0.3, K1=0.9)


def f_of(p):
    return rot("x", H.PITCH) @ np.asarray(p, float)


def grip_paths():
    e = SAD[0]["E"]; E1 = f_of([KN[0] + e[0], (SAD[0]["y"][0] + SAD[0]["y"][1]) / 2, e[1]])
    end = lambda k: [np.array([X_SLOT, 0, SLOT_Z[k]]), np.array([RL + 1.0, 0, SLOT_Z[k]])]
    P = dict(curl=[f_of([KN[0], 0, 0]), f_of([X_CONV + 1.0, 0, 0]), np.array([X_CONV - 3.0, 0, SLOT_Z["curl"]])] + end("curl"),
             K2=[f_of([KN[0], Y_K2, 0]), f_of([X_CONV + 1.0, Y_K2, 0]), np.array([X_CONV - 3.0, f_of([0, Y_K2, 0])[1], SLOT_Z["K2"]])] + end("K2"),
             K3=[f_of([KN[0], Y_K3, 0]), f_of([X_CONV, Y_K3, 0])] + end("K3"),
             K1=[E1, np.array([X_CONV - 2.0, -2.0, -1.0]), np.array([X_CONV - 3.5, -2.4, 0.85])] + end("K1"))
    return P


def _tube(pts, r):
    out = None
    for a, b in zip(pts[:-1], pts[1:]):
        s = M_.batch_hull([M_.sphere(r, 16).translate(list(a)), M_.sphere(r, 16).translate(list(b))])
        out = s if out is None else out + s
    return out


def grip_cuts():
    fill = None
    for y in (0.0, Y_K2):
        a = f_of([RL + 2.6, y, 0]); b = f_of([KN[0] - 7.6, y, 0])
        s = M_.batch_hull([M_.sphere(0.56, 16).translate(list(a)), M_.sphere(0.56, 16).translate(list(b))])
        fill = s if fill is None else fill + s
    cut = None
    for k, p in grip_paths().items():
        s = _tube(p, 0.45); cut = s if cut is None else cut + s
    cut += box([RL + 0.8, -0.45, -1.25], [X_SLOT, 0.45, 1.25])
    cut += app(cyl(0.5, KN[0] - 6.0, X_CONV + 0.3, "x", (0, Y_K3, 0), n=16), T(rot("x", H.PITCH)))
    return fill, cut


LAG = dict(peg=(12.0, 5.5), peg_d=2.0, post=(18.0, -5.0), post_d=1.6, recess=1.4)


def lag_parts(sign):
    P = LAG; py = sign * P["post"][1]; ppy = sign * P["peg"][1]
    z0, z1 = 2.4 - P["recess"], 2.5
    pts = [np.array([P["post"][0], py])]
    for lag in np.linspace(0, H.SPREAD_MAX, 6):
        a = np.radians(-sign * lag); R2 = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        pg = np.array([RL, 0]) + R2 @ (np.array([P["peg"][0], ppy]) - [RL, 0])
        u = pg - pts[0]; tx = (14.0 - pts[0][0]) / u[0]; pts.append(pts[0] + u * tx)
    rec = M_.batch_hull([cyl(1.9, z0, z1, c=(q[0], q[1], 0), n=32) for q in pts])
    post = cyl(P["post_d"] / 2, z0 - 0.01, z1 - 0.1, c=(P["post"][0], py, 0), n=24)
    peg = cyl(P["peg_d"] / 2, ZB - 2.5 - 0.3, ZB + 2.5, c=(P["peg"][0], ppy, 0), n=24)
    return rec, post, peg


BOX = dict(z0=6.8, floor=0.8, room=3.0, lid=0.6, wall=0.8, col_r=1.8, pin_r=0.75, screw=(13.0, 0.0))
LANE = {0: dict(phi=90.0, d_end=14.0, tray=1), 1: dict(phi=0.0, d_end=13.0, tray=2), 2: dict(phi=-45.0, d_end=15.0, tray=1)}
_IDS = {1.0: (2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 7, 8, 9, 10, 12), 1.5: (2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 7, 8, 9, 10, 12),
        1.78: (1.42, 2.06, 2.57, 2.9, 3.68, 4.47, 5.28, 6.07, 7.65, 9.25, 10.82),
        2.0: (3, 3.5, 4, 4.5, 5, 6, 7, 8, 9, 10, 12), 2.5: (4, 5, 6, 7, 8, 9, 10, 12)}
STOCK = [(float(ID), cs, mat) for mat in ("silicone", "NBR") for cs, ids in _IDS.items() for ID in ids]
E_MOD = dict(silicone=2.0, NBR=5.5)


def tray_z(t):
    B = BOX; z0 = B["z0"] + (t - 1) * (B["floor"] + B["room"])
    return z0, z0 + B["floor"], z0 + B["floor"] + B["room"]


def lane_geom(j):
    L = LANE[j]; d_post = L["d_end"] - BOX["wall"] - BOX["pin_r"] - 0.2; d_hook = BOX["col_r"] + BOX["pin_r"] + 0.3
    return d_post, d_hook


def ring_eval(j, ID, cs, mat, tau0):
    cur = ext_curve(j); arm0 = cur[0][2] * 1e-3; L0c = cur[0][1]
    F0 = tau0 / (arm0 * ETA_EXT); EA2 = 2 * E_MOD[mat] * np.pi * cs * cs / 4; Lf = np.pi * (ID + cs) / 2
    turn = np.pi * (BOX["pin_r"] + cs / 2)
    e0 = F0 / EA2; span0 = Lf * (1 + e0) - turn; travel = cur[-1][1] - L0c
    d_post, d_hook = lane_geom(j)
    rows = []
    for f, L, arm in cur:
        e = e0 + (L - L0c) / Lf; Fr = e * EA2
        rows.append((round(f, 1), round(Fr * arm * 1e-3, 5), round(e, 3), round(arm, 3), round(Fr, 2), round(Fr * arm * 1e-3 / ETA_EXT, 5)))
    rated = RATED_CORD * H.LEVER[j] * 1e-3
    ok = dict(fits=span0 + travel <= d_post - d_hook and span0 >= 1.0, strain=max(r[2] for r in rows) <= 1.0,
              frac=max(r[1] for r in rows) <= (SPRING_FRAC - 0.02) * rated)
    return dict(ID=ID, cs=cs, mat=mat, F0=F0, e0=e0, span0=span0, travel=travel, room=d_post - d_hook, rows=rows, rated=rated, ok=ok,
                tau0=tau0, tau0_spring=F0 * arm0, tau_max=max(r[1] for r in rows), tau_max_cord=max(r[5] for r in rows))


_PICK = {}


def ring_pick(j):
    if j in _PICK: return _PICK[j]
    key = ["K1", "K2", "K3"][j]; lo, hi = WINDOW[key]; best = None
    taus = sorted(np.arange(lo, hi + 1e-9, 0.001), key=lambda t: abs(t - TARGET[key]))
    for tau in taus:
        for ID, cs, mat in STOCK:
            r = ring_eval(j, ID, cs, mat, tau)
            if not all(r["ok"].values()): continue
            emax = max(x[2] for x in r["rows"])
            sc = abs(tau - TARGET[key]) * 1e3 + abs(r["e0"] - 0.4) + 0.4 * (ID < 3.0) + 0.1 * (mat == "NBR") + 2.0 * max(0.0, emax - 0.9)
            if best is None or sc < best[0]: best = (sc, r)
        if best is not None: break
    _PICK[j] = best[1] if best else None
    return _PICK[j]


def _poly(pts):
    return mf.CrossSection([[tuple(p) for p in pts]])


def _lanes2d(s, tray, grow=0.0):
    out = None
    for j, L in LANE.items():
        if L["tray"] != tray: continue
        r = ring_pick(j); w = BOX["pin_r"] + (r["cs"] if r else 2.0) + 0.35 + grow
        phi = np.radians(s * L["phi"]); u = np.array([np.cos(phi), np.sin(phi)]); n = np.array([-u[1], u[0]])
        c = np.array([RL, 0.0]); d0 = 0.5 - grow; d1 = L["d_end"] - BOX["wall"] + grow
        cs_ = _poly([c + u * d0 - n * w, c + u * d1 - n * w, c + u * d1 + n * w, c + u * d0 + n * w])
        out = cs_ if out is None else out + cs_
    return out


def _outline(s, tray):
    B = BOX; sx, sy = B["screw"]
    o = _lanes2d(s, tray, B["wall"]) + mf.CrossSection.circle(B["col_r"] + B["wall"] + 0.8, 48).translate([RL, 0])
    o += mf.CrossSection.circle(2.2, 32).translate([sx, sy]) + _poly([(sx, -1.5), (RL - 1.5, -1.5), (RL - 1.5, 1.5), (sx, 1.5)])
    return o


def _posts(s, tray, z0, z1, r):
    m = None
    for j, L in LANE.items():
        if L["tray"] != tray: continue
        d_post, _ = lane_geom(j); phi = np.radians(s * L["phi"])
        c = cyl(r, z0, z1, c=(RL + d_post * np.cos(phi), d_post * np.sin(phi), 0), n=20); m = c if m is None else m + c
    return m


def spring_tray(s, tray):
    B = BOX; z0, zf, zl = (ZB + z for z in tray_z(tray)); sx, sy = B["screw"]
    m = _outline(s, tray).extrude(zl - z0).translate([0, 0, z0])
    if tray == 1:
        m += cyl(2.2, ZB + 5.0, z0 + 0.01, c=(sx, sy, 0), n=32)
        m -= cyl(H.LAG_TUBE[0] / 2 + 0.05, z0 - 0.1, zf + 0.01, c=(RL, 0, 0), n=48)
    m -= _lanes2d(s, tray).extrude(zl - zf + 0.1).translate([0, 0, zf])
    m -= cyl(B["col_r"], z0 - 0.1 if tray == 2 else zf - 0.2, zl + 0.1, c=(RL, 0, 0), n=48)
    m -= cyl(1.1, ZB + 4.0, zl + 1, c=(sx, sy, 0), n=24)
    p = _posts(s, tray, zf - 0.6, zl + 0.1, 0.72)
    if p is not None: m -= p
    p = _posts(s, tray - 1, z0 - 0.1, z0 + 0.4, 0.77) if tray == 2 else None
    if p is not None: m -= p
    return m


def spring_lid(s):
    B = BOX; zl = ZB + tray_z(2)[2]; zt = zl + B["lid"]; sx, sy = B["screw"]
    m = _outline(s, 2).extrude(zt - zl).translate([0, 0, zl])
    m -= cyl(1.1, zl - 1, zt + 1, c=(sx, sy, 0), n=24)
    p = _posts(s, 2, zl - 0.1, zl + 0.4, 0.77)
    return m - p if p is not None else m


WIN = (-1.25, 1.25)


def lag_tube_ext(k=None):
    t = cyl(H.LAG_TUBE[0] / 2, -5.0, 7.5, c=(RL, 0, 0)) - cyl(H.LAG_TUBE[1] / 2, -6, 8, c=(RL, 0, 0))
    t -= box([RL, -3, WIN[0]], [RL + 3.5, 3, WIN[1]])
    if k is not None:
        try:
            import hand3_hubc as HC
            t -= app(cyl(HC.HOLE_D / 2, 0, 4.0, "x", (0, 0, 0), n=20), T(t=[RL, 0, HC.Z_CORD]) @ T(rot("z", LS[k] * HC.HOLE_DEG)))
        except Exception:
            pass
    return t


_P = {}


def patch_root_grip(g):
    fill, cut = grip_cuts()
    m = g + (fill ^ g.hull()) - cut
    for sign in (-1, 1):
        rec, post, _ = lag_parts(sign); m = m - rec + post
    return m


def patch_phalanx(j, m):
    if j == 0:
        m = m + saddle(0); m -= k1_anchor_cut(m)
        m -= ext_bore(1) + cyl(0.5, KN[0] - 0.5, KN[1] + 0.5, "x", (0, Y_K3, 0), n=16)
    if j == 1:
        m = recut_front(m + saddle(1), 1)
        m -= ext_bore(2) + cyl(0.5, KN[1] - 0.5, SAD[2]["chan"][0] + 0.5, "x", (0, Y_K3, 0), n=16)
    if j == 2:
        m = recut_front(m + saddle(2), 2)
    return m


def patch_hub_lower(m):
    for k, az0 in enumerate(FAZ):
        _, _, peg = lag_parts(LS[k]); m += app(peg, T(rot("z", az0)))
    return m


def patch_parts(base):
    if _P: return dict(_P)
    out = dict(base)
    out["root_grip"] = patch_root_grip(base["root_grip"])
    for j in range(3): out[f"phalanx{j + 1}"] = patch_phalanx(j, base[f"phalanx{j + 1}"])
    out["hub_lower"] = patch_hub_lower(base["hub_lower"])
    out["lag_tube"] = lag_tube_ext()
    for hand, s in (("L", -1), ("R", 1)):
        out[f"spring_tray1_{hand}"] = spring_tray(s, 1); out[f"spring_tray2_{hand}"] = spring_tray(s, 2); out[f"spring_lid_{hand}"] = spring_lid(s)
    _P.update(out)
    return dict(out)
