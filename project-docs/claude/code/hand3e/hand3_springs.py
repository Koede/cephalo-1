"""hand3 hub C return springs (drop-in for hand3e: hand3_cad.parts() = patch_parts(base_parts())), 2026-11-15.

Nothing on the blade surfaces.  Every spring lives in a small box on top of its finger's lag tube, above the hub, where no
air flows past a blade:
  K1, K2, K3  a printed saddle on the distal part's back, in the knuckle's centre gap, inside the knuckle's own outline as
              the airflow sees it (barrel, bosses, stop teeth).  A 0.6 mm Dyneema extensor cord runs from the saddle into
              the proximal part, along on-axis channels past the inner knuckles (no torque there), down the root grip into a
              stacked slot that meets the lag axis, through the lag tube's blade-plane window (cord line through the lag
              axis: no lag torque at any spread), up the tube and out of its top into the SPRING BOX: one lane per knuckle,
              an O-ring between a Ø1.5 steel post and a Ø1.5 hook pin the cord is tied to.  Swap ring sizes to tune.
  lag         one O-ring per finger between a Ø2 peg on the lower hub plate and a post in a recess in the grip's top face,
              inside the hub-plate gap (the grip carries both recesses, so one grip fits every finger).
Extensor friction (4 bends, Dyneema on polished steel / printed walls, mu ~0.08 est.): ETA_EXT 0.72 of the ring force
reaches the knuckle as return torque; the cord sees ring force / ETA_EXT when folding.
python3 hand3_springs.py -> hand3_springs.json + STLs in springs_stl/."""
import os, sys, json
import numpy as np
import manifold3d as mf
from scipy.spatial import ConvexHull
import hand3_design as H
import hand3_finger as F
from hand3_finger import cyl, box, rot, T, app, about

M_ = mf.Manifold
KN = H.KN; RL = H.R_LAG; ZB = H.Z["blade"]; FAZ = H.FINGER_AZ; LS = H.LAG_SIGN
TARGET = dict(K1=0.020, K2=0.004, K3=0.020, lag=0.006)          # N m (hub C), window K1 / K3 0.015-0.025
WINDOW = dict(K1=(0.018, 0.025), K2=(0.003, 0.008), K3=(0.015, 0.025))   # K1 >= 0.018 keeps the spread-first margin
RATED_CORD = 31.9                                               # N per finger, rated (STS3215 on the ring)
ETA_EXT = 0.72
CORD_R = 0.3
SPRING_FRAC = 0.40

# ---------------------------------------------------------------- saddles (distal part of knuckle j, finger frame)
# y band in the centre gap (+-GAP/2), arc radius, angles (deg from +x towards +z; 180-270 = back face), E = bore entry in the
# proximal part's knuckle room (x from the axis, z), core = fill the gap void over the band (K2, K3; K1 carries on-axis
# cords so its arc is held by a web to the blade back instead)
SAD = {0: dict(y=(-1.45, -0.35), r_s=2.0, r_in=1.0, a=(200.0, 268.0), anchor=262.0, E=(-6.95, -0.8), core=False),
       1: dict(y=(-1.45, -0.35), r_s=2.8, r_in=2.05, a=(200.0, 268.0), anchor=262.0, E=(-4.9, -0.5), core=False, chan=(49.0, -0.9)),
       2: dict(y=(0.10, 1.10), r_s=2.0, r_in=0.0, a=(200.0, 268.0), anchor=262.0, E=(-3.9, -0.4), core=True, chan=(80.5, 1.0))}
Y_K3 = 1.0                                     # K3 extensor channel (new), z 0: phalanx 2, phalanx 1, root grip
Y_K2 = H.CORD_Y[2]                             # K2 extensor uses the free rev D tip-cord channel (y -0.9) in phalanx 1 and the grip


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
    else:                                       # web to the blade back, r_in..r_s+1.2, inside the lug barrels' outline
        m += arc_wedge(cyl(min(r_s + 1.2, H.BARREL[j] / 2 - 0.3), y0, y1, "y", c, n=96) - cyl(s["r_in"], y0 - 1, y1 + 1, "y", c, n=48), c, s["a"][1] - 8, 330.0)
    t = np.radians(s["anchor"]); d = np.array([np.cos(t), 0, np.sin(t)]); yc = (y0 + y1) / 2
    if not s["core"] and j > 0:                 # web-held K2: radial hole through the arc, the knot sits inside it (open gap)
        for r in np.linspace(s["r_in"] - 0.3, r_s + 0.6, 6):
            m -= M_.sphere(0.45, 16).translate([xk + r * d[0], yc, r * d[2]])
    if s["core"]:                               # radial anchor hole + knot pocket opening sideways into the free gap
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
    """K1: hole from the saddle surface (anchor angle) through the web to the blade back at x+3.2, knot counterbore flush."""
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
    """K2 / K3: bore in the proximal part from the room (E) to the on-axis channel."""
    s = SAD[j]; xk = KN[j]; ye = (s["y"][0] + s["y"][1]) / 2
    E = np.array([xk + s["E"][0] + 0.8, ye, s["E"][1]]); Cn = np.array([s["chan"][0], s["chan"][1], 0.0])
    b = M_.batch_hull([M_.sphere(0.45, 16).translate(list(E)), M_.sphere(0.45, 16).translate(list(Cn))])
    return b + M_.batch_hull([M_.sphere(0.45, 16).translate(list(E)), M_.sphere(0.45, 16).translate([xk + s["E"][0] - 0.6, ye, s["E"][1]])])


def recut_front(m, j):
    xk = KN[j]; c = (xk, 0, 0); R = H.LEVER[j]; yc = H.CORD_Y[j]
    return m - app(cyl(0.55, 1.0, R + 1.5), about("y", 100.0, c) @ T(t=[xk, yc, 0]))


# ---------------------------------------------------------------- root grip: cords to the lag axis (frame F of the grip)
X_CONV = 30.0                                  # convergence starts (x, frame F)
X_SLOT = RL + 3.4                              # stacked slot: 4 cords on y 0, z = SLOT_Z, from here into the tube
SLOT_Z = dict(curl=-0.9, K2=-0.3, K3=0.3, K1=0.9)


def f_of(p):
    """finger-frame point -> grip frame F."""
    return rot("x", H.PITCH) @ np.asarray(p, float)


def grip_paths():
    """Cord centrelines in frame F (polylines), from where they enter the grip to the slot end inside the tube bore."""
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
    """(fill, cut) in frame F: fill the old straight curl / tip channels between the tube and X_CONV, cut the new paths and
    the slot (y +-0.45, z -1.25..1.25, from X_SLOT into the bore)."""
    fill = None
    for y in (0.0, Y_K2):
        a = f_of([RL + 2.6, y, 0]); b = f_of([KN[0] - 7.6, y, 0])          # stop short of the K1 knuckle room (r 7)
        s = M_.batch_hull([M_.sphere(0.56, 16).translate(list(a)), M_.sphere(0.56, 16).translate(list(b))])
        fill = s if fill is None else fill + s
    cut = None
    for k, p in grip_paths().items():
        s = _tube(p, 0.45); cut = s if cut is None else cut + s
    cut += box([RL + 0.8, -0.45, -1.25], [X_SLOT, 0.45, 1.25])
    cut += app(cyl(0.5, KN[0] - 6.0, X_CONV + 0.3, "x", (0, Y_K3, 0), n=16), T(rot("x", H.PITCH)))        # K3 channel, grip part
    return fill, cut


def path_min_dist():
    """Smallest centreline distance between the cords in the grip (segment sampling)."""
    P = grip_paths(); names = list(P); out = {}
    smp = {k: np.vstack([np.linspace(a, b, 40) for a, b in zip(v[:-2], v[1:-1])]) for k, v in P.items()}   # up to the slot
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            d = np.min(np.linalg.norm(smp[a][:, None, :] - smp[b][None, :, :], axis=2)); out[f"{a}-{b}"] = round(float(d), 2)
    return out


# ---------------------------------------------------------------- lag spring (in the hub-plate gap, frame F)
LAG = dict(peg=(12.0, 5.5), peg_d=2.0, post=(18.0, -5.0), post_d=1.6, recess=1.4)   # for LAG_SIGN +1 (mirror y for -1)


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


def lag_curve(sign=1, ring=(5.5, 0.8), E=2.0, n=12):
    ID, cs = ring; P = LAG; Lf = np.pi * (ID + cs) / 2; EA2 = 2 * E * np.pi * cs * cs / 4
    peg = np.array([P["peg"][0], sign * P["peg"][1]]); out = []
    for lag in np.linspace(0, H.SPREAD_MAX, n):
        a = np.radians(sign * lag); R2 = np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])
        post = np.array([RL, 0]) + R2 @ (np.array([P["post"][0], sign * P["post"][1]]) - [RL, 0])
        span = np.linalg.norm(post - peg); L = span + np.pi * (P["peg_d"] / 2 + P["post_d"] / 2) / 2 + np.pi * cs / 2
        Fr = max(L - Lf, 0) / Lf * EA2; u = (peg - post) / span; r = post - [RL, 0]
        tau = (r[0] * u[1] - r[1] * u[0]) * Fr * 1e-3 * sign
        out.append((round(float(lag), 1), round(-tau, 4), round(float((L - Lf) / Lf), 2), round(float(Fr), 2)))
    return out


def lag_pick():
    best = None
    for cs in (0.8, 1.0):
        for ID in np.arange(4.0, 10.01, 0.5):
            cur = lag_curve(1, (ID, cs))
            if cur[0][1] <= 0 or max(c[2] for c in cur) > 1.0: continue
            sc = abs(cur[0][1] - TARGET["lag"])
            if best is None or sc < best[0]: best = (sc, float(ID), cs, cur)
    return best


# ---------------------------------------------------------------- spring box (finger hub frame: tube at (RL, 0), z from ZB)
BOX = dict(z0=6.8, floor=0.8, room=3.0, lid=0.6, wall=0.8, col_r=1.8, pin_r=0.75, screw=(13.0, 0.0))
# lanes, finger hub frame for LAG_SIGN +1 (phi mirrored for -1): tray 1 holds K1 (towards the neighbour, inside the bisector)
# and K3 (outboard diagonal); tray 2 on top holds K2 (radial).  Lengths from the finger-sweep map of hand3e (all states).
LANE = {0: dict(phi=90.0, d_end=14.0, tray=1), 1: dict(phi=0.0, d_end=13.0, tray=2), 2: dict(phi=-45.0, d_end=15.0, tray=1)}
_IDS = {1.0: (2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 7, 8, 9, 10, 12), 1.5: (2, 2.5, 3, 3.5, 4, 4.5, 5, 6, 7, 8, 9, 10, 12),
        1.78: (1.42, 2.06, 2.57, 2.9, 3.68, 4.47, 5.28, 6.07, 7.65, 9.25, 10.82),      # AS568-001..-012 (and -2xx-free)
        2.0: (3, 3.5, 4, 4.5, 5, 6, 7, 8, 9, 10, 12), 2.5: (4, 5, 6, 7, 8, 9, 10, 12)}  # common metric O-ring IDs (DIN 3771 / ISO 3601 ranges)
STOCK = [(float(ID), cs, mat) for mat in ("silicone", "NBR") for cs, ids in _IDS.items() for ID in ids]
E_MOD = dict(silicone=2.0, NBR=5.5)          # MPa, estimates (50-60A silicone, 70A NBR)


def tray_z(t):
    """(bottom, floor top, lane top) of tray t (1, 2), from ZB; lid on top of tray 2."""
    B = BOX; z0 = B["z0"] + (t - 1) * (B["floor"] + B["room"])
    return z0, z0 + B["floor"], z0 + B["floor"] + B["room"]


def lane_geom(j):
    L = LANE[j]; d_post = L["d_end"] - BOX["wall"] - BOX["pin_r"] - 0.2; d_hook = BOX["col_r"] + BOX["pin_r"] + 0.3
    return d_post, d_hook


def ring_eval(j, ID, cs, mat, tau0):
    """Ring in lane j giving the delivered preload tau0: torque / strain / force over the fold."""
    cur = ext_curve(j); arm0 = cur[0][2] * 1e-3; L0c = cur[0][1]
    F0 = tau0 / (arm0 * ETA_EXT); EA2 = 2 * E_MOD[mat] * np.pi * cs * cs / 4; Lf = np.pi * (ID + cs) / 2
    turn = np.pi * (BOX["pin_r"] + cs / 2)                                 # two half turns round the Ø1.5 pins (centreline)
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
    """Lane interiors (grow 0) / outlines (grow = wall) of a tray, 2-D, finger hub frame."""
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
    """Tray 1 (sits on the tube end, M2 boss down to the upper hub plate) or tray 2 (on tray 1, its floor closes tray 1).
    PA12, lanes open at the top; the lid closes tray 2.  Posts: Ø1.5 x 4 steel dowels, blind holes."""
    B = BOX; z0, zf, zl = (ZB + z for z in tray_z(tray)); sx, sy = B["screw"]
    m = _outline(s, tray).extrude(zl - z0).translate([0, 0, z0])
    if tray == 1:
        m += cyl(2.2, ZB + 5.0, z0 + 0.01, c=(sx, sy, 0), n=32)                       # boss down to the upper plate
        m -= cyl(H.LAG_TUBE[0] / 2 + 0.05, z0 - 0.1, zf + 0.01, c=(RL, 0, 0), n=48)    # on the tube end (top ZB+7.5)
    m -= _lanes2d(s, tray).extrude(zl - zf + 0.1).translate([0, 0, zf])
    m -= cyl(B["col_r"], z0 - 0.1 if tray == 2 else zf - 0.2, zl + 0.1, c=(RL, 0, 0), n=48)
    m -= cyl(1.1, ZB + 4.0, zl + 1, c=(sx, sy, 0), n=24)
    if tray == 1:                                                                     # tray 2's floor holds tray 1's post tops
        pass
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


def spring_box(s):
    return spring_tray(s, 1) + spring_tray(s, 2)


WIN = (-1.25, 1.25)                            # lag tube window (z from the blade plane): was -0.8..0.8


def lag_tube_ext(k=None):
    """Lag tube with the taller blade-plane window (4 cords stacked on the lag axis); base-cord hole as hand3_hubc.lag_tube2."""
    t = cyl(H.LAG_TUBE[0] / 2, -5.0, 7.5, c=(RL, 0, 0)) - cyl(H.LAG_TUBE[1] / 2, -6, 8, c=(RL, 0, 0))
    t -= box([RL, -3, WIN[0]], [RL + 3.5, 3, WIN[1]])
    if k is not None:
        try:
            import hand3_hubc as HC
            t -= app(cyl(HC.HOLE_D / 2, 0, 4.0, "x", (0, 0, 0), n=20), T(t=[RL, 0, HC.Z_CORD]) @ T(rot("z", LS[k] * HC.HOLE_DEG)))
        except Exception:
            pass
    return t


# ---------------------------------------------------------------- the patch
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
    """base = hand3_cad.base_parts() -> same keys with the spring features, plus spring_tray1_L/R, spring_tray2_L/R, spring_lid_L/R (finger hub
    frame, place like the lag tube: T(t=[0,0,0]) @ rot z FINGER_AZ[k]; L = LAG_SIGN -1, R = +1) and lag_tube (taller window)."""
    if _P: return dict(_P)
    out = dict(base)
    out["root_grip"] = patch_root_grip(base["root_grip"])
    for j in range(3): out[f"phalanx{j + 1}"] = patch_phalanx(j, base[f"phalanx{j + 1}"])
    out["hub_lower"] = patch_hub_lower(base["hub_lower"])
    out["lag_tube"] = lag_tube_ext()
    for hand, s in (("L", -1), ("R", 1)):
        out[f"spring_tray1_{hand}"] = spring_tray(s, 1); out[f"spring_tray2_{hand}"] = spring_tray(s, 2); out[f"spring_lid_{hand}"] = spring_lid(s)
    _P.update(out)
    _hook_hubc()
    return dict(out)


def _hook_hubc():
    """Wire the boxes and the taller tube window into hand3_hubc.assembly() (if the coordinator has not done it)."""
    HC = sys.modules.get("hand3_hubc")
    if HC is None or getattr(HC, "_springs_hooked", False): return
    HC.lag_tube2 = lag_tube_ext
    asm0 = HC.assembly

    def assembly(*a, **kw):
        out = asm0(*a, **kw); spin = kw.get("spin", a[4] if len(a) > 4 else 0.0); S = T(rot("z", spin))
        for k in range(4):
            hand = "R" if LS[k] > 0 else "L"; A = S @ T(rot("z", FAZ[k]))
            for nm in ("tray1", "tray2", "lid"): out[f"s{nm}{k}"] = (app(_P[f"spring_{nm}_{hand}"], A), True)
        return out
    HC.assembly = assembly; HC._springs_hooked = True


# ---------------------------------------------------------------- tables for the harness
def spring_table():
    rows = []
    for j in range(3):
        r = ring_pick(j)
        rows.append(dict(joint=f"K{j + 1}", part=f"O-ring {r['ID']:g} x {r['cs']:g} {r['mat']}", tau0_Nm=round(r["tau0"], 4), tau_max_Nm=r["tau_max"],
                         tau_max_cord_Nm=r["tau_max_cord"], frac_rated=round(r["tau_max"] / r["rated"], 3), F0_N=round(r["F0"], 2),
                         strain_pre=round(r["e0"], 2), strain_max=max(x[2] for x in r["rows"]), span0_mm=round(r["span0"], 2), travel_mm=round(r["travel"], 2),
                         lane_room_mm=round(r["room"], 2), arm0_mm=round(r["rows"][0][3], 2), arm_end_mm=round(r["rows"][-1][3], 2)))
    sc, ID, cs, cur = lag_pick()
    rows.append(dict(joint="lag", part=f"O-ring {ID:g} x {cs:g} silicone", tau0_Nm=cur[0][1], tau_max_Nm=max(c[1] for c in cur), tau_full_spread_Nm=cur[-1][1],
                     strain_max=max(c[2] for c in cur), curve=cur))
    return rows


def pick_ring(j):
    r = ring_pick(j)
    return (0.0, r["ID"], r["cs"], [(x[0], x[1], x[2], x[3]) for x in r["rows"]])


# ---------------------------------------------------------------- own checks (python3 hand3_springs.py)
def _cord2d(path, y, r=CORD_R):
    parts = [M_.batch_hull([M_.sphere(r, 10).translate([a[0], y, a[1]]), M_.sphere(r, 10).translate([b[0], y, b[1]])]) for a, b in zip(path[:-1], path[1:])]
    return M_.batch_boolean(parts, mf.OpType.Add)


def frontal_cords(P):
    """Harness frontal measure (area along the chord beyond the airfoil, +-12 mm round each knuckle) with the free cord spans
    from saddle to bore added (straight finger)."""
    clean = F.loft(np.linspace(KN[0] - 12, H.R_TIP, 12), F.foil); V = T(rot("x", 90))
    ch = app(P["root_grip"], T(rot("x", -H.PITCH)))
    for i in (1, 2, 3): ch = ch + P[f"phalanx{i}"] + P[f"pad{i}"]
    cords = None
    for j in range(3):
        path, _ = ext_path(j, 0.0); c = _cord2d(path, sum(SAD[j]["y"]) / 2); cords = c if cords is None else cords + c
    out = []
    for j in range(3):
        sl = box([KN[j] - 12, -30, -30], [KN[j] + 12, 30, 30])
        a0 = app((ch + clean) ^ sl, V).project().area(); a1 = app((ch + cords + clean) ^ sl, V).project().area()
        out.append(round(a1 - a0, 3))
    return out


def fold_check(P, B):
    """Each knuckle 0..max: patched distal vs patched proximal overlap (base for comparison); the free cord span vs both parts."""
    G = lambda Q: app(Q["root_grip"], T(rot("x", -H.PITCH)))
    out = {}
    for j in range(3):
        prox = G(P) if j == 0 else P[f"phalanx{j}"]; proxb = G(B) if j == 0 else B[f"phalanx{j}"]
        w = wb = wc = 0.0
        for f in np.linspace(0, H.FOLD_MAX[j], 10):
            A = about("y", -f, [KN[j], 0, 0]); dist = app(P[f"phalanx{j + 1}"], A)
            w = max(w, (dist ^ prox).volume()); wb = max(wb, (app(B[f"phalanx{j + 1}"], A) ^ proxb).volume())
            path, _ = ext_path(j, f)
            tb = _cord2d(path[1:], sum(SAD[j]["y"]) / 2, CORD_R * 0.9)
            wc = max(wc, (tb ^ prox).volume(), (tb ^ dist).volume())
        out[f"K{j + 1}"] = dict(patched_mm3=round(w, 3), base_mm3=round(wb, 3), cord_mm3=round(wc, 3))
    return out


def lag_torque_check():
    """Extensor lines at the tube: distance of each cord's last grip segment from the lag axis (zero = no lag torque)."""
    out = {}
    for k, p in grip_paths().items():
        a, b = p[-2], p[-1]; d = b[:2] - a[:2]; r = np.array([RL, 0.0]) - a[:2]           # the span that leaves the grip (slot)
        out[k] = round(float(abs(d[0] * r[1] - d[1] * r[0]) / np.linalg.norm(d)), 3)
    return out


if __name__ == "__main__":
    import hand3_cad as C
    B = C.base_parts(); P = C.parts()
    rep = dict(springs=spring_table(), grip_cord_spacing_mm=path_min_dist(), cord_line_to_lag_axis_mm=lag_torque_check(),
               frontal_cords_mm2=frontal_cords(P), fold=fold_check(P, B))
    lagc = lag_pick()[3]; k1 = ring_pick(0)
    rep["staging"] = dict(T_spread_start_N=round(lagc[0][1] / 2.5e-3, 2), T_K1_start_N=round(k1["tau0"] / (H.LEVER[0] * 1e-3), 2),
                          margin=round(k1["tau0"] / (H.LEVER[0] * 1e-3) / (lagc[0][1] / 2.5e-3), 2))
    os.makedirs("springs_stl", exist_ok=True); rep["parts"] = {}
    for n in ("root_grip", "phalanx1", "phalanx2", "phalanx3", "hub_lower", "lag_tube", "spring_tray1_L", "spring_tray1_R", "spring_tray2_L", "spring_tray2_R", "spring_lid_L", "spring_lid_R"):
        t = F.to_trimesh(P[n]); t.export(f"springs_stl/{n}.stl")
        rep["parts"][n] = dict(bodies=len(P[n].decompose()), watertight=bool(t.is_watertight), vol_mm3=round(P[n].volume(), 1),
                               d_vol_mm3=round(P[n].volume() - B[n].volume(), 1) if n in B else None)
    rep["knuckles"] = [dict(knuckle=f"K{j + 1}", frontal_mm2=rep["frontal_cords_mm2"][j]) for j in range(3)]
    json.dump(rep, open("hand3_springs.json", "w"), indent=1, default=float)
    print(json.dumps({k: v for k, v in rep.items() if k != "springs"}, indent=1, default=float))
    for r in rep["springs"]: print({k: v for k, v in r.items() if k != "curve"})
