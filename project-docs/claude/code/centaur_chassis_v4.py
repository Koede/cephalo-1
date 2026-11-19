"""Equo-1 chassis v4 (look): one continuous shell over the v3 profile.  The surface swells smoothly over the shoulder
and hip pods (Gaussian bumps along the spine and around the section), sections are superellipses (slightly squared,
robotic), radii are smoothed along the spine, two-tone finish with panel-line seams.  Leg covers: tapered shells over
femur (hiding the knee belt and both pulleys) and shank.  Mechanisms are unchanged underneath (chassis_v3, legs_v2)."""
import numpy as np, sk3d
import chassis_v3 as C3
C = sk3d.COL
LIGHT, DARK, SEAM = "#e4e8ee", "#3a3f47", "#20242a"
HIPF, HIPR, cc, n_up = C3.HIPF, C3.HIPR, C3.cc, C3.n_up
NA = 48                                                      # points around a section
# purposeful dimensions (m)
Y_ACT_OUT = 0.185      # outer face of the hip pitch actuators (RS03 front, moved 8 mm inboard; RS06 rear 0.180)
WALL = 0.0025          # PETG/PA-CF shell wall
Y_POD = 0.195          # pod face: actuator face + 7.5 mm clearance + wall
POD_R = 0.100          # pod radius: front hip actuator rim + 47 mm (RS03: 53 -> 100); set per actuator below
THIGH_OFF = 0.014 if C3.KNEE_MODE == 'scapula' else 0.053   # thigh cover centre outboard of the hip plane
Y_THIGH = 0.253        # thigh cover centre plane (femur plane 0.200 + knee actuator and belt stack outboard)
THIGH_RY = 0.0465 if C3.KNEE_MODE == 'scapula' else 0.048   # scapula: stack -24.5..+52.5 (motor | belt) + 3 air + wall
_OLD_RY = 0.048       # half-thickness: stack 0.2125..0.2885 + 5 mm + wall; inner face 0.205 (10 mm swept gap), outer 0.297
THIGH_E = 8.0          # rounded-rectangle thigh section: hugs the square belt + actuator stack
R_THIGH = (0.064, 0.062)   # thigh half-depth in the leg plane at hip / stifle (60T pulley r 47.8 + 5 + wall)
STIFLE_EXT = 0.052      # thigh runs straight 52 mm past the stifle axis (60T r 47.8 + 4)
BED = 0.256            # printer bed: every shell panel must fit

# body slices: vertical cuts of the v3 side profile (chest, withers, neck rise, croup all come from the profile)
_ox = C3.OUT[:, 0]
NX = 44
XS = 0.5 * (_ox.min() + _ox.max()) - 0.5 * (_ox.max() - _ox.min()) * np.cos(np.linspace(0.02, np.pi - 0.02, NX))  # denser at the ends

def sections():
    zlo, zhi = [], []
    for x in XS:
        t = C3.hits(np.array([x, 0.30 * C3.S]), np.array([0, 1.0]))
        t = [v + 0.30 * C3.S - 0.30 for v in t]
        zlo.append(0.30 + t[0]); zhi.append(0.30 + t[-1])
    zlo, zhi = np.array(zlo), np.array(zhi)
    for _ in range(3):                                        # smear the profile a little (no kinks at the withers)
        zhi[1:-1] = 0.25 * zhi[:-2] + 0.5 * zhi[1:-1] + 0.25 * zhi[2:]
        zlo[1:-1] = 0.25 * zlo[:-2] + 0.5 * zlo[1:-1] + 0.25 * zlo[2:]
    u = (XS - XS[0]) / (XS[-1] - XS[0])
    W = C3.S * np.interp(u, [0, 0.04, 0.2, 0.5, 0.75, 0.93, 1.0], [0.03, 0.115, 0.14, 0.15, 0.15, 0.12, 0.04])
    return zlo, zhi, W

# each pod houses the hip pitch actuator (axis y) and the abduction actuator (axis x, 99.5 mm toward the body centre)
POD_R = C3.HIPF_R + 0.047
Y_POD_F = C3.Y_LEG_F - C3.POD_IN; Y_POD_R = min(Y_POD, C3.Y_LEG_R - C3.POD_IN)                 # pod faces 5 mm inside each hip plane
PODS = [(HIPF, HIPF + np.array([C3.ABD_DIR_F * 0.0995, 0.03]), Y_POD_F), (HIPR, HIPR + np.array([C3.ABD_DIR_R * 0.0995, C3.ABD_DZ_R]), Y_POD_R)]
X_MID = 0.5 * (C3.HIPF[0] + C3.HIPR[0])
# girth pods: the barrel bulges just enough over the two front abduction actuators (torque-tube layout)
GIRTH = [] if C3.ABD_MODE != "tube" else [(np.array([C3.TT_ACT_X0, C3.TT_Z]), np.array([C3.TT_ACT_X1, C3.TT_Z]),
                                         C3.TT_Y + C3.ABD_R + 0.0075 + WALL, C3.ABD_R + 0.018)]
def y_cap(x): return Y_POD_F if x > X_MID else Y_POD_R
def seg_dist(p, a, b):
    t = np.clip((p - a) @ (b - a) / ((b - a) @ (b - a)), 0, 1); return np.linalg.norm(p - a - t * (b - a))

def bump(x, x0, sx):
    return np.exp(-0.5 * ((x - x0) / sx) ** 2)

def shell_points():
    zlo, zhi, W = sections()
    th = np.linspace(0, 2 * np.pi, NA + 1)
    P = np.zeros((NX, NA + 1, 3))
    for i, x in enumerate(XS):
        zc, hz = 0.5 * (zlo[i] + zhi[i]), 0.5 * (zhi[i] - zlo[i])
        for k, a in enumerate(th):
            ca, sa = np.cos(a), np.sin(a)
            e = 2.6                                              # superellipse: slightly squared, robotic
            cy = np.sign(ca) * abs(ca) ** (2 / e); sz = np.sign(sa) * abs(sa) ** (2 / e)
            z = zc + hz * sz
            for hp, dr in ((HIPF, 0.04), (HIPR, 0.065)):     # pods reach down past the actuator rims on the lower flanks
                z -= dr * np.exp(-((x - hp[0]) / 0.075) ** 4) * max(-sa, 0.0) * abs(ca) ** 0.5
            fz = np.clip((z - 0.64 * C3.S) / (0.12 * C3.S), 0, 1)
            wz = W[i] + (0.118 - W[i]) * fz * (W[i] > 0.1)   # the neck blends to the collar (236 mm, fixed)
            y = wz * cy
            a_s = (np.array([x, z]) - C3._end) @ n_up; b_s = (np.array([x, z]) - C3._end) @ C3.band   # neck holds the sheath block
            hw_ = C3.recv_hw(np.clip(a_s, 0, C3.SHEATH_L))
            if -C3.SHEATH_M < a_s < C3.SHEATH_L + 0.02 and abs(b_s) < hw_ + C3.SHEATH_M:
                y = np.sign(cy if cy != 0 else 1) * max(abs(y), (hw_ + C3.SHEATH_M) * min(1.0, abs(cy) / 0.35))   # receiver section at this depth
            for hp, ab, yp in PODS:                              # hip pods: capsules around pitch + abduction actuators
                r = seg_dist(np.array([x, z]), hp, ab)
                w = np.exp(-(r / POD_R) ** 8) * min(1.0, abs(cy) / 0.6)
                y = np.sign(cy) * (abs(y) + max(0.0, yp - abs(y)) * w)
            for ga, gb, gy, gr in GIRTH:
                r = seg_dist(np.array([x, z]), ga, gb)
                w = np.exp(-(r / gr) ** 8) * min(1.0, abs(cy) / 0.6)
                y = np.sign(cy if cy != 0 else 1) * (abs(y) + max(0.0, gy - abs(y)) * w)
            y = np.sign(y) * min(abs(y), y_cap(x) if not GIRTH or x > C3.TT_ACT_X1 + 0.03 else max(y_cap(x), GIRTH[0][2]))
            # collar land: inside r 155 of the neck axis the shell stays 3 mm under the seat plane (the plate lands on the pads)
            q3 = np.array([x, y, z]); w3 = np.array([n_up[0], 0, n_up[1]]); c3 = np.array([cc[0], 0, cc[1]])
            hh = (q3 - c3) @ w3; rr3 = np.linalg.norm((q3 - c3) - hh * w3)
            if rr3 < 0.155 and hh > -0.003: q3 = q3 - (hh + 0.003) * w3
            P[i, k] = q3
    return P

def _breaks():
    k = [kk for kk in range(NA) if (np.sin(2 * np.pi * kk / NA) < -0.35) != (np.sin(2 * np.pi * (kk - 1) / NA) < -0.35)]
    return sorted(k)
K_BREAK = _breaks()
K_MID = (2, NA // 2 - 2)          # shoulder lines (sin = +0.26): roof panels above, flank panels below
def _bands():
    kb1, kb2 = K_BREAK; km2, km1 = K_MID; kd, kv = NA // 4, 3 * NA // 4
    return {"roof R": range(km2, kd + 1), "roof L": range(kd, km1 + 1), "flank L": range(km1, kb1 + 1),
            "belly pan L": range(kb1, kv + 1), "belly pan R": range(kv, kb2 + 1), "flank R": range(kb2, km2 + NA + 1)}
SPLITS = ()                       # set below by _auto_splits(): every panel fits the bed

def panels(P=None):
    """Shell panels between splits and parting lines; returns list of (name, sorted extents mm) using the best of
    axis-aligned and principal-axis boxes (how it would sit on the bed)."""
    P = shell_points() if P is None else P
    st = (0,) + SPLITS + (NX - 1,)
    kq = [K_BREAK[0], 3 * NA // 4, K_BREAK[1], NA + NA // 4]    # belly L, belly R, upper R->dorsal->upper L wrap
    out = []
    for a, b in zip(st[:-1], st[1:]):
        bands = _bands()
        for nm, ks in bands.items():
            pts = np.array([P[i, k % (NA + 1)] for i in range(a, b + 1) for k in ks])
            ext_aa = np.sort(np.ptp(pts, axis=0))
            c = pts - pts.mean(0); _, _, vt = np.linalg.svd(c, full_matrices=False); ext_pc = np.sort(np.ptp(c @ vt.T, axis=0))
            ext = ext_aa if ext_aa.max() < ext_pc.max() else ext_pc
            out.append((f"stations {a}-{b} {nm}", (1e3 * ext).round()))
    return out

def _band_ext(P, a, b):
    out = []
    for ks in _bands().values():
        pts = np.array([P[i, k % (NA + 1)] for i in range(a, b + 1) for k in ks])
        ext_aa = np.sort(np.ptp(pts, axis=0)); c = pts - pts.mean(0); _, _, vt = np.linalg.svd(c, full_matrices=False)
        ext_pc = np.sort(np.ptp(c @ vt.T, axis=0)); out.append(min(ext_aa.max(), ext_pc.max()))
    return max(out)

def _auto_splits():
    P = shell_points(); sp, a = [], 0
    while True:
        b = a + 1
        while b + 1 < NX and _band_ext(P, a, b + 1) <= BED - 0.006: b += 1
        if b >= NX - 1: return tuple(sp)
        sp.append(b); a = b
SPLITS = _auto_splits()

def in_cup(q, r_open=0.120, depth=0.040):
    """inside the neck opening: within r_open of the neck axis and less than `depth` below the seat plane"""
    w3 = np.array([n_up[0], 0, n_up[1]]); c3 = np.array([cc[0], 0, cc[1]]); q = np.asarray(q)
    hh = (q - c3) @ w3; rr = np.linalg.norm((q - c3) - hh * w3)
    return rr < r_open and hh > -depth

def shell(sc, rings=True):
    P = shell_points()
    for i in range(NX - 1):
        for k in range(NA):
            if in_cup(0.25 * (P[i, k] + P[i, k + 1] + P[i + 1, k] + P[i + 1, k + 1])): continue     # the cup opening (crown ring is its rim)
            zc = 0.25 * (P[i, k, 2] + P[i, k + 1, 2] + P[i + 1, k, 2] + P[i + 1, k + 1, 2])
            lower = np.sin(2 * np.pi * (k + 0.5) / NA) < -0.35
            sc.poly([P[i, k], P[i, k + 1], P[i + 1, k + 1], P[i + 1, k]], DARK if lower else LIGHT, ec=(0, 0, 0, 0))
    for i in (0, NX - 1):                                        # close rump and chest
        ctr = P[i, :-1].mean(0)
        for k in range(NA): sc.poly([ctr, P[i, k], P[i, k + 1]], LIGHT, ec=(0, 0, 0, 0))
    if rings: shell_trim(sc, P)
    return P

def shell_trim(sc, P=None):
    """panel-line seams, pod caps and the dock LED ring (drawn as their own parts)"""
    P = shell_points() if P is None else P
    for i in SPLITS:                                             # panel splits: every panel fits the print bed
        ring = [p + (p - P[i].mean(0)) * 0.012 for p in P[i]]
        run = []
        for q in ring:
            if in_cup(q, 0.160, 0.045):
                if len(run) > 1: sc.tube(run, 0.0025, SEAM, n=4, balls=False)
                run = []
            else: run.append(q)
        if len(run) > 1: sc.tube(run, 0.0025, SEAM, n=4, balls=False)
    for kk in (K_BREAK[0], K_BREAK[1], K_MID[0], K_MID[1], NA // 4, 3 * NA // 4):   # parting lines: belly pan, flank/roof, halves
        run = []                                                  # (they stop at the collar land around the neck opening)
        for i in range(NX):
            q = P[i, kk] + (P[i, kk] - P[i].mean(0)) * 0.012
            if in_cup(q, 0.160, 0.045):
                if len(run) > 1: sc.tube(run, 0.0025, SEAM, n=4, balls=False)
                run = []
            else: run.append(q)
        if len(run) > 1: sc.tube(run, 0.0025, SEAM, n=4, balls=False)
    for hp, _, yp in PODS:                                       # pod caps: bolt-on access to each hip actuator
        for sgn in (1, -1):
            c = np.array([hp[0], sgn * (yp + 0.0015), hp[1]])
            sc.tube([c + 0.072 * np.array([np.cos(a), 0, np.sin(a)]) for a in np.linspace(0, 2 * np.pi, 33)], 0.0022, SEAM, n=4, balls=False)
            sc.cyl(c - [0, sgn * 0.001, 0], c + [0, sgn * 0.002, 0], 0.03, DARK, n=20)     # cap boss over the output bearing
    C3.led_ring(sc)
    return P

TOP_ROUND = 0.025       # pauldron top: rounded off 25 mm past the knee motor
BOT_ROUND = 0.025       # thigh bottom: rounded off 25 mm past the 60T pulley stretch
def cover_t0(L):
    """start of the thigh cover along the femur line (t = 0 at the hip, 1 at the stifle): above the hip over the knee motor"""
    return -(C3.R_K + 0.044 + 0.0055) / L if C3.KNEE_MODE == "scapula" else 0.0

def leg_cover(sc, hip, st, hk, fe, side):
    """Pauldron + thigh: one shell from above the hip (over the knee motor on its shoulder-blade arm) down past the stifle
    (60T pulley).  Rounded-rectangle section; the pauldron part flares 8 % and is split into three lames; tapered shank and
    cannon covers below."""
    off = np.array([0, side * THIGH_OFF, 0])
    a0, a1 = hip + off, st + off
    L = np.linalg.norm(a1 - a0); d = (a1 - a0) / L; u = np.cross([0, 1.0, 0], d); u /= np.linalg.norm(u); yv = np.array([0, 1.0, 0])
    T0, T1 = cover_t0(L), 1 + STIFLE_EXT / L
    def tp(t, a):
        tt = np.clip(t, 0, 1); rs = R_THIGH[0] + (R_THIGH[1] - R_THIGH[0]) * tt
        rs *= 1 + 0.08 * np.clip(-t / max(-T0, 1e-6), 0, 1) if T0 < 0 else 1          # pauldron flare
        te = np.clip(t, T0, T1)
        rr_ = TOP_ROUND if t < T0 else BOT_ROUND
        e = np.sqrt(max(0.0, 1 - ((t - te) * L / rr_) ** 2)) if (t < T0 or t > T1) else 1.0
        ca, sa = np.cos(a), np.sin(a); ca, sa = np.sign(ca) * abs(ca) ** (2 / THIGH_E), np.sign(sa) * abs(sa) ** (2 / THIGH_E)
        return a0 + (a1 - a0) * t + e * (rs * ca * u + THIGH_RY * sa * yv)
    ts = np.r_[T0 - np.linspace(TOP_ROUND, 0, 4)[:-1] / L, np.linspace(T0, T1, 18), T1 + np.linspace(0, BOT_ROUND, 4)[1:] / L]
    na = 24
    if hasattr(sc, 'group'): sc.group('pauldron+thigh cover')
    for i in range(len(ts) - 1):
        for j in range(na):
            A, B_ = 2 * np.pi * j / na, 2 * np.pi * (j + 1) / na
            q = [tp(ts[i], A), tp(ts[i], B_), tp(ts[i + 1], B_), tp(ts[i + 1], A)]
            col = DARK if (ts[i] >= 0.97 or ts[i] < 0.0) else LIGHT                    # pauldron and stifle cap dark
            sc.poly(q, col, ec=(0, 0, 0, 0))
    if hasattr(sc, 'ungroup'): sc.ungroup()
    _ds = (hk - st) / np.linalg.norm(hk - st)
    sc.cyl(st + _ds * 0.090, hk, 0.025, LIGHT, n=18, r1=0.020, caps=False)   # shank cover: leg plane, starts below the thigh cover's stifle cap
    sc.sphere(hk, 0.024, DARK, n=8)
    sc.cyl(hk, fe, 0.019, DARK, n=14, r1=0.016, caps=False)                 # cannon
    sc.sphere(fe, 0.018, DARK, n=6)
    # parting line in the leg plane (inner / outer halves, printed flat) and the pauldron lames
    sc.tube([tp(t, 0) + 0.0012 * u for t in np.linspace(T0 - TOP_ROUND / L, T1 + 0.2, 21)], 0.0018, SEAM, n=4, balls=False)
    sc.tube([tp(t, np.pi) - 0.0012 * u for t in np.linspace(T0 - TOP_ROUND / L, T1 + 0.2, 21)], 0.0018, SEAM, n=4, balls=False)
    if T0 < 0:
        for f in (0.33, 0.66, 1.0):
            t = T0 * (1 - f) + 0.0 * f
            ring = [tp(t, a) * 1.0 + (tp(t, a) - (a0 + (a1 - a0) * t)) * 0.02 for a in np.linspace(0, 2 * np.pi, 25)]
            sc.tube(ring, 0.0018, SEAM, n=4, balls=False)

POD_RING = 0.066        # pod ring radius about the actuator axes: RS03 rim 53 + 13 mm
FRAME_INSET = WALL + 0.0045 + 0.006    # rib centreline inside the skin: wall + flange gap + tube radius
def frame(sc):
    """Ribs where panels meet: one ring under every panel split (the panels bolt to it), plus the two end rings;
    stringers under the parting lines (belly-pan, shoulder and dorsal lines) tie them together.  6 mm PA-CF tube /
    printed ring, M3 heat-set inserts at 40 mm pitch along each flange."""
    P = shell_points()
    from matplotlib.path import Path as _Path
    need = FRAME_INSET + 0.001
    def keep_inside(q):
        """move a frame point inward until it sits FRAME_INSET under the skin, checked in the sections at x and
        x +- 7 mm (a tube has width along x too; matters where the profile is steep, e.g. the throat)"""
        q = np.array(q, float)
        for _ in range(12):
            moved = False
            for dx in (-0.007, 0.0, 0.007):
                x = q[0] + dx
                if x <= XS[0] or x >= XS[-1]: continue
                i = np.searchsorted(XS, x) - 1; f = (x - XS[i]) / (XS[i + 1] - XS[i])
                poly = ((1 - f) * P[i, :-1] + f * P[i + 1, :-1])[:, 1:]; c = poly.mean(0)
                p2 = q[1:]; seg = np.r_[poly, poly[:1]]; a_, b_ = seg[:-1], seg[1:]
                t = np.clip(np.einsum("ij,ij->i", p2 - a_, b_ - a_) / np.einsum("ij,ij->i", b_ - a_, b_ - a_), 0, 1)
                cl = a_ + t[:, None] * (b_ - a_); dd = np.linalg.norm(cl - p2, axis=1); j = np.argmin(dd)
                if _Path(poly).contains_point(p2) and dd[j] >= need: continue
                n_ = (c - cl[j]); n_ /= np.linalg.norm(n_); q[1:] = cl[j] + n_ * need * 1.05; moved = True
            if not moved: break
        return q
    _tube = sc.tube
    def tube(pts, *a, **k):
        """frame line kept under the skin and broken where it meets the neck receiver (it bolts to the receiver there)"""
        run = []
        dense = [np.asarray(pts[0], float)]                      # densify to <= 4 mm so no segment can jump an obstacle
        for p0_, p1_ in zip(pts[:-1], pts[1:]):
            p0_, p1_ = np.asarray(p0_, float), np.asarray(p1_, float); n_ = max(1, int(np.ceil(np.linalg.norm(p1_ - p0_) / 0.004)))
            dense += [p0_ + (p1_ - p0_) * (j / n_) for j in range(1, n_ + 1)]
        for p in dense:
            q = keep_inside(p)
            blocked = C3.recv_sdf(q) < 0.006 + 0.004 or in_cup(q, 0.175, 0.045)
            for (c0, c1, rr_) in getattr(sc, "obstacles", []):
                ab = c1 - c0; tt = np.clip((q - c0) @ ab / (ab @ ab), 0, 1)
                if np.linalg.norm(c0 + tt * ab - q) < rr_ + 0.006 + 0.003: blocked = True; break
            if run and np.linalg.norm(q - run[-1]) > 0.012:   # keep_inside jumped (a skin fold): end the line, don't bridge
                if len(run) > 1: _tube(run, *a, **k)
                run = []
            if blocked:
                if len(run) > 1: _tube(run, *a, **k)
                run = []
            else: run.append(q)
        if len(run) > 1: _tube(run, *a, **k)
    def inset(i, k):
        c = P[i, :-1].mean(0); p = P[i, k]; d = p - c; n = np.linalg.norm(d[1:])
        return p - d * (FRAME_INSET / max(n, 1e-6)) * np.array([0, 1, 1])
    rings = (6,) + tuple(i for i in SPLITS if 6 < i < NX - 10) + (NX - 10,)   # no rings on the tiny end sections
    for i in rings:
        tube([inset(i, k) for k in range(NA + 1)], 0.006, C["frame"], n=6, balls=False)
    def in_pod(p):
        return min(np.hypot(p[0] - hp[0], p[2] - hp[1]) for hp, ab, _ in PODS) < POD_RING
    for kk in (K_BREAK[0], K_BREAK[1], K_MID[0], K_MID[1], NA // 4):     # stringers stop at the pod rings
        run = []
        for i in range(6, NX - 9):            # stringers land on the end rings
            q = inset(i, kk)
            if in_pod(q) and abs(q[1]) > 0.10:
                if len(run) > 1: tube(run, 0.005, C["frame"], n=6, balls=False)
                run = []
            else: run.append(q)
        if len(run) > 1: tube(run, 0.005, C["frame"], n=6, balls=False)
    for hp, ab, yp in PODS:                                # pod rings: carry the pod caps and catch the stringers
        for sgn in (1, -1):
            yv = sgn * (yp - FRAME_INSET)
            pts = []
            for a in np.linspace(0, 2 * np.pi, 41):
                # stadium around the capsule, radius POD_R - 4 mm
                c = np.array([np.cos(a), np.sin(a)]); t = 0.0     # ring around the hip pitch actuator (the cap's job)
                base = hp + (ab - hp) * t; q2 = base + POD_RING * c
                pts.append([q2[0], yv, q2[1]])
            tube(pts, 0.005, C["frame"], n=6, balls=False)
    return rings
