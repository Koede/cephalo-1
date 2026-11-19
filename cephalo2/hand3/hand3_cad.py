"""Three-phalanx hand-prop rev B: true solids (manifold3d), built from hand3_design.py.  python3 hand3_cad.py -> hand3_stl/*.stl
Frames.  Pod: z = 0 at the arm's spigot face, +z = thrust, rotor axis = z.  Finger frame F: origin on the rotor axis in the
blade plane (pod z = Z['blade']), x = radius at lag 0, z = pod z.  Chain frame C = F pitched PITCH about x (leading edge
+y, grip face +z); knuckle pins lie along C y at x = KN[j]; a positive fold turns the outer part toward +z.
First solid model: servo bodies are boxes from the makers' outline sizes (shaft assumed mid-length), spread-cord routing and
screws are not modelled.  Overlap before unions >= 0.3 mm; clearances are stated where they are cut."""
import os, json
import numpy as np
import manifold3d as mf
import trimesh
import hand3_design as H

M_ = mf.Manifold
KN, RT, RL = H.KN, H.R_TIP, H.R_LAG
ZB = H.Z["blade"]; Zs = H.Z
RHO = dict(pacf=1.25, tpu=1.20, alu=2.70, steel=7.85, brass=8.5)            # g/cm3


# ------------------------------------------------------------------ helpers
def cyl(r, a, b, ax="z", c=(0, 0, 0), n=48):
    m = M_.cylinder(b - a, r, r, n).translate([0, 0, a])
    if ax == "x": m = m.rotate([0, 90, 0])
    elif ax == "y": m = m.rotate([-90, 0, 0])
    return m.translate(list(c))


def box(lo, hi):
    lo = np.asarray(lo, float); hi = np.asarray(hi, float)
    return M_.cube(list(np.abs(hi - lo))).translate(list(np.minimum(lo, hi)))


def half(m, n, p0):
    """Keep the side of the plane through p0 that the normal n points to."""
    n = np.asarray(n, float); n = n / np.linalg.norm(n)
    return m.trim_by_plane(list(n), float(n @ np.asarray(p0, float)))


def wedge(m, c, a0, a1):
    """Keep the angular range a0..a1 (deg, from +z toward +x, span < 180) about the y-parallel axis through c."""
    for ph, sg in ((a0, 1), (a1, -1)):
        p = np.radians(ph); m = half(m, [sg * np.cos(p), 0, -sg * np.sin(p)], c)
    return m


def T(R=None, t=(0, 0, 0)):
    A = np.eye(4)
    if R is not None: A[:3, :3] = R
    A[:3, 3] = t
    return A


def rot(axis, deg):
    a = np.radians(deg); c, s = np.cos(a), np.sin(a)
    return {"x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]), "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]), "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])}[axis]


def app(m, A):
    return m.transform(np.ascontiguousarray(A[:3, :4]))


def about(axis, deg, p):
    return T(t=p) @ T(rot(axis, deg)) @ T(t=-np.asarray(p, float))


def loft(xs, poly):
    """Solid through polygons poly(x) -> (n, 2) points (y, z), counter-clockwise seen from +x, the same n at every station."""
    P = [np.asarray(poly(x), float) for x in xs]; n = len(P[0])
    V = [np.c_[np.full(n, x), p] for x, p in zip(xs, P)]
    V += [[[xs[0], *P[0].mean(0)]], [[xs[-1], *P[-1].mean(0)]]]
    V = np.vstack(V); c0, c1 = len(V) - 2, len(V) - 1; F = []
    for i in range(len(xs) - 1):
        for k in range(n):
            a, b, c, d = i * n + k, i * n + (k + 1) % n, (i + 1) * n + (k + 1) % n, (i + 1) * n + k
            F += [[a, b, c], [a, c, d]]
    for k in range(n):
        F += [[c0, (k + 1) % n, k], [c1, (len(xs) - 1) * n + k, (len(xs) - 1) * n + (k + 1) % n]]
    m = M_(mf.Mesh(vert_properties=np.asarray(V, np.float32), tri_verts=np.asarray(F, np.uint32)))
    assert m.status() == mf.Error.NoError and m.volume() > 0, (m.status(), m.volume())
    return m


def to_trimesh(m):
    me = m.to_mesh()
    return trimesh.Trimesh(np.asarray(me.vert_properties)[:, :3], np.asarray(me.tri_verts), process=False)


# ------------------------------------------------------------------ finger (chain frame)
def foil(r, n=34):
    xs, yu, yl = H.section(r, n); c = H.chord(r)
    yc = (yu + yl) / 2; ht = np.maximum((yu - yl) / 2, 0.25)                 # 0.5 mm blunt leading and trailing edge
    zc = float(np.interp(c / 2, xs, yc))                                    # pin axis = mid-thickness at mid-chord
    return np.vstack([np.c_[c / 2 - xs, yc + ht - zc], np.c_[c / 2 - xs, yc - ht - zc][::-1]])


def pad_poly(r, lift=0.0, n=12):
    xs, yu, yl = H.section(r, 60); c = H.chord(r); zc = float(np.interp(c / 2, xs, (yu + yl) / 2))
    xp = np.linspace(H.PAD_X[0] * c, H.PAD_X[1] * c, n); top = np.interp(xp, xs, yu) - zc
    return np.vstack([np.c_[c / 2 - xp, top + lift], np.c_[c / 2 - xp, top - H.PAD_T][::-1]])


def pad_span(j):
    x1 = KN[j + 1] - H.PAD_MARGIN if j < 2 else RT - 2.0
    return KN[j] + H.PAD_MARGIN, x1


def cord_slot(j):
    """Open slot in the top of phalanx j (and its pad) where the next knuckle's cord rises from the centre line to its fairlead."""
    if j == 2: return None
    R = H.LEVER[j + 1]; xk = KN[j + 1]; y = H.CORD_Y[j + 1]
    return box([xk - R - 11, y - 0.55, -0.5], [xk - R - 3.5, y + 0.55, 8])


def pad(j, lift=0.0):
    a, b = pad_span(j)
    m = loft(np.linspace(a, b, 5), lambda r: pad_poly(r, lift))
    s = cord_slot(j)
    return m - s if (s is not None and lift == 0.0) else m


def end_trim(m, xk, s, fold):
    """End faces at a knuckle: below the pin axis a stop face (0.3 deg each side), above it a relief of fold / 2."""
    m = half(m, [s, 0, np.tan(np.radians(0.3))], [xk + 0.02 * s, 0, 0])
    return half(m, [s, 0, -np.tan(np.radians(fold / 2))], [xk + 0.02 * s, 0, 0])


def yl(j):
    g2 = H.GAP[j] / 2
    return g2, g2 + H.LUG[j], H.W_KN[j] / 2            # gap edge, lug outer face, cheek outer face


def bores(m, j):
    g2, lo, W2 = yl(j)
    for s in (-1, 1): m -= cyl(H.BUSH_OD[j] / 2 + 0.02, min(s * (g2 - 0.01), s * (W2 + 1)), max(s * (g2 - 0.01), s * (W2 + 1)), "y", (KN[j], 0, 0))
    return m


def distal_end(m, j):
    """Inboard end of the part outboard of knuckle j: two lugs with webs, the cord pulley sector between them."""
    g2, lo, W2 = yl(j); B = H.BARREL[j]; R = H.LEVER[j]; xk = KN[j]; yc = H.CORD_Y[j]; c = (xk, 0, 0)
    for s in (-1, 1):
        m -= cyl(B / 2 + 0.3, min(s * lo, s * (W2 + 0.3)), max(s * lo, s * (W2 + 0.3)), "y", c)          # room for the cheeks
    for s in (-1, 1):
        a, b = sorted((s * g2, s * lo))
        lug = cyl(B / 2, a, b, "y", c)
        m += M_.batch_hull([lug, box([xk + 6.5, a, -0.8], [xk + 7.5, b, 0.3])])
        m += wedge(cyl(B / 2 + H.STOP_TOOTH, a, b, "y", c) - cyl(B / 2 - 0.6, a - 1, b + 1, "y", c), c, 180.0, 215.0)     # back-stop tooth under the lug
    sec = wedge(cyl(R + 0.6, -(g2 + 0.5), g2 + 0.5, "y", c), c, -5.0, 115.0)
    sec -= cyl(R + 1.2, yc - 0.45, yc + 0.45, "y", c) - cyl(R - 0.35, yc - 0.5, yc + 0.5, "y", c)        # rim groove: cord centre at R
    m += sec
    m -= cyl(1.7, -(g2 + 0.6), g2 + 0.6, "y", c)                                                        # pass-through cords cross on the axis
    m -= app(cyl(0.55, 1.0, R + 1.5), about("y", 100.0, c) @ T(t=[xk, yc, 0]))                      # anchor hole at the end of the wrap
    return bores(m, j)


def proximal_end(m, j, fairlead=True):
    """Outboard end of the part inboard of knuckle j: two cheeks with webs, clearance for lugs and sector, the cord fairlead."""
    g2, lo, W2 = yl(j); B = H.BARREL[j]; R = H.LEVER[j]; xk = KN[j]; yc = H.CORD_Y[j]; c = (xk, 0, 0)
    for s in (-1, 1):
        a, b = sorted((s * (W2 - H.CHEEK[j]), s * W2)); a2, b2 = sorted((s * (W2 - H.CHEEK[j] - 2.5), s * (W2 - 0.5)))
        wx = 3.0 if j == 0 else 7.5                                                                      # K1: short webs, inside the grip
        m += M_.batch_hull([cyl(B / 2, a, b, "y", c), box([xk - wx, a2, -0.8], [xk - wx + 1.0, b2, 0.3])])
    m -= cyl(B / 2 + 0.3, -(lo + 0.1), lo + 0.1, "y", c)                                               # room for the lugs (also trims the webs)
    m -= cyl(R + 1.0, -(g2 + 0.8), g2 + 0.8, "y", c)                                                   # room for the pulley sector
    for s in (-1, 1):                                                                                    # boss the lug's tooth seats on at -PRECONE
        a, b = sorted((s * (g2 + 0.2), s * (lo - 0.1)))
        m += wedge(cyl(B / 2 + H.STOP_TOOTH + 0.6, a, b, "y", c) - cyl(B / 2 + 0.3, a - 1, b + 1, "y", c), c, 215.0 + H.PRECONE, 262.0)
    if fairlead:
        post = box([xk - R - 3.6, yc - 1.5, -0.3], [xk - R - 1.6, yc + 1.5, R + 1.2]) - cyl(0.55, xk - R - 4, xk - R - 1, "x", (0, yc, R))
        m += post
    return bores(m, j)


def channels(m, j):
    """Centre-line cord channels (z = 0) for the cords that pass through part j (j = -1: root grip)."""
    for k in range(max(j + 1, 1), 3):
        x0 = RL if j < 0 else KN[j]; x1 = KN[j + 1] if k > j + 1 else KN[j + 1] - H.LEVER[j + 1] - 9.0
        m -= cyl(0.5, x0 - 0.5, x1 + (0.5 if k > j + 1 else 0), "x", (0, H.CORD_Y[k], 0), n=16)
    return m


def phalanx(j):
    x0 = KN[j]; x1 = KN[j + 1] if j < 2 else RT
    m = loft(np.linspace(x0 - 0.6, x1 + (0.6 if j < 2 else 0), 7), foil)
    m -= pad(j, lift=0.8)
    m = end_trim(m, x0, 1, H.FOLD_MAX[j])
    if j < 2: m = end_trim(m, x1, -1, H.FOLD_MAX[j + 1])
    m = distal_end(m, j)
    if j < 2:
        m = proximal_end(m, j + 1); m -= cord_slot(j)
    return channels(m, j)


def root_grip():
    """6061 aluminium: flat eye part between the hub plates (frame F), fork for K1 pitched with the chain.  The pitched part
    starts at x 31 so that no pitched corner comes inside the hub-plate radius when the finger swings SPREAD_MAX."""
    flat = box([14.5, -6, -2.4], [31.6, 6, 2.4]) - cyl(H.LAG_TUBE[0] / 2 + 0.03, -4, 4, c=(RL, 0, 0))
    flat -= cyl(0.7, -4, 4, c=(RL - 3.5, 0, 0))                                             # spread-cord eye in the tail
    W2 = H.W_KN[0] / 2
    fork = M_.batch_hull([box([31.0, -6, -2.6], [32.0, 6, 2.6]), box([34.0, -W2, -3.0], [KN[0] + 0.5, W2, 3.0])])
    m = app(flat, T(rot("x", -H.PITCH))) + fork                                             # chain coordinates
    m = end_trim(m, KN[0], -1, H.FOLD_MAX[0])
    return app(channels(proximal_end(m, 0, fairlead=False), -1), T(rot("x", H.PITCH)))


def lag_tube():
    t = cyl(H.LAG_TUBE[0] / 2, -5.0, 7.5, c=(RL, 0, 0)) - cyl(H.LAG_TUBE[1] / 2, -6, 8, c=(RL, 0, 0))
    for z0, z1 in ((-0.8, 0.8), (5.3, 6.9)): t -= box([RL, -3, z0], [RL + 3.5, 3, z1])   # windows: middle + tip cords; base cord above the plate
    return t


# ------------------------------------------------------------------ hub, cage, followers (pod frame, rotor at spin 0)
FAZ = H.FINGER_AZ
def follower_az():
    out = []
    for c in H.CHANNELS:
        out.append(c["az"] if c["finger"] is None else FAZ[c["finger"]] + H.FOLLOW_DAZ[c["role"]])
    return out


def stop_posts():
    """Two Ø3 posts per finger between the hub plates: lag 0 (flight) and lag SPREAD_MAX."""
    out = []
    for k in range(4):
        s = H.LAG_SIGN[k]
        for lag, y in ((0.0, s * 7.55), (s * H.SPREAD_MAX, -s * 7.55)):
            p = rot("z", FAZ[k]) @ (np.array([RL, 0, 0]) + rot("z", lag) @ np.array([-4.0, y, 0]))
            out.append(cyl(1.5, ZB - 2.5 - 0.3, ZB + 2.5, c=(p[0], p[1], 0)))
    return out


def hub_lower():
    rp = H.HUB["plate_d"] / 2
    m = cyl(rp, Zs["hub_lo"], Zs["hub_lo"] + 2.5) + cyl(7.0, Zs["motor_top"], ZB + 2.5)
    for p in stop_posts(): m += p
    return hub_holes(m)


def hub_upper():
    return hub_holes(cyl(H.HUB["plate_d"] / 2, ZB + 2.5, ZB + 5.0))


def hub_holes(m):
    m -= cyl(2.6, 80, 100)
    for a in FAZ: m -= app(cyl(H.LAG_TUBE[0] / 2 + 0.02, 80, 100, c=(RL, 0, 0)), T(rot("z", a)))
    return m


def cage():
    r0, r1 = H.HUB["cage_r"]; ft = Zs["fin_top"]
    m = cyl(r1, Zs["cage_lo"], Zs["hub_lo"], n=96) - cyl(r0, 50, 90, n=96)
    slot = box([20.5, -1.9, ft - 8 - H.HUB["follow_travel"] - 0.5], [21.7, 1.9, 88]) + box([21.6, -1.1, ft - 8 - H.HUB["follow_travel"] - 0.5], [23.5, 1.1, 88])
    for a in follower_az(): m -= app(slot, T(rot("z", a)))
    pocket = cyl(23.6, ft + 1.5, 88, n=96) - cyl(18.4, 80, 90)
    for a in FAZ:                                           # cords fan out from the lag tube to the three slots
        w = app(half(half(pocket, [np.sin(np.radians(16)), np.cos(np.radians(16)), 0], [0, 0, 0]), [np.sin(np.radians(16)), -np.cos(np.radians(16)), 0], [0, 0, 0]), T(rot("z", a)))
        m -= w
    return m


def follower():
    ft = Zs["fin_top"]
    m = box([20.7, -1.7, ft - 8], [21.5, 1.7, ft]) + box([21.2, -0.9, ft - 8], [23.4, 0.9, ft]) + box([23.1, -2.3, ft - 8], [27.2, 2.3, ft])
    m += box([26.9, -2.3, ft - 2], [H.HUB["fin_r"][1], 2.3, ft])
    return m - cyl(0.55, ft - 9, ft + 1, c=(24.4, 0, 0)) - cyl(1.3, ft - 9, ft - 5, c=(24.4, 0, 0))     # cord hole and knot pocket


def heel():
    return cyl(8.0, ZB + 5.0, Zs["heel"]) - cyl(4.7, ZB + 4, Zs["shaft_tip"] + 0.8)


# ------------------------------------------------------------------ stationary: claw bars, pack frame
def claw_bar(c, az_f):
    """U-channel bar with a gusseted arm that reaches over its follower's fin (az_f = follower azimuth at the park index)."""
    B = H.BAR; r0, r1 = B["r"]; w = B["w"] / 2
    bar = box([r0, -w, B["z0"]], [r1, w, Zs["claw_hi"] + 0.4]) - box([r1 - B["slot"][1], -B["slot"][0] / 2, B["z0"] + 1.4], [r1 + 1, B["slot"][0] / 2, Zs["claw_hi"] - 3])
    d = az_f - c["az"]
    tip = app(box([H.HUB["claw_tip_r"], -2.3, Zs["claw_lo"]], [33.0, 2.3, Zs["claw_hi"] + 0.4]), T(rot("z", d)))
    neck = app(box([32.0, -2.3, Zs["claw_lo"]], [33.5, 2.3, Zs["claw_hi"] + 0.4]), T(rot("z", d)))
    root = box([r0 - 2.6, -w, B["arm_root_z"]], [r0 + 0.5, w, Zs["claw_hi"] + 0.4])                # straight through the sleeve slot, skew starts inboard of it
    arm = M_.batch_hull([box([r0 - 2.6, -w, B["arm_root_z"]], [r0 - 2.0, w, Zs["claw_hi"] + 0.4]), neck]) + root + tip
    return app(bar + arm, T(rot("z", c["az"])))


def pack_frame():
    ro = H.PACK["spool_r_pos"] + 2.5                                    # 51 in rev C, 47 in rev D
    web = cyl(ro - 9, -1, 60, n=96) - cyl(13, -2, 61)                   # skeleton plates: hub disc, rim and one 3 mm spoke per channel
    for c in H.CHANNELS: web -= app(box([10, -1.5, -3], [ro - 6, 1.5, 62]), T(rot("z", c["az"])))
    m = cyl(ro, 0, 2.5, n=96) + cyl(ro, 26.4, 27.4, n=96) + cyl(ro, 51.5, 54, n=96) - web + cyl(10, 2, 52, n=64)
    m -= cyl(7, -1, 60)
    B = H.BAR; r0, r1 = B["r"]; w = B["w"] / 2; sw = B["sleeve_wall"]
    for c in H.CHANNELS:
        zb = 27.4 if c["tier"] == 1 else 51.5
        sl = box([r0 - sw - 0.1, -w - sw - 0.1, zb], [r1 + sw + 0.1, w + sw + 0.1, B["sleeve_top"]])
        cut = box([r0 - 0.1, -w - 0.1, 20], [r1 + 0.1, w + 0.1, 75]) + box([r0 - sw - 1, -w - 0.1, 57.0], [r0, w + 0.1, 75])
        cord = cyl(1.0, 20, 60, c=(H.PACK["spool_r_pos"], 0, 0))
        A = T(rot("z", c["az"]))
        m = m + app(sl, A) - app(cut, A) - app(cord, A)
    return m


def hardware():
    """Bought parts as simple solids, each with (solid, rotating?, material)."""
    out = {}
    for c in H.CHANNELS:
        sv = H.SERVOS["SCS0009" if c["role"] == "tip" else "STS3032"]; hgt = sv["size"][2]; z0 = 3.0 if c["tier"] == 1 else 27.6
        A = T(rot("z", c["az"])); r1 = H.PACK["spool_r_pos"] - 2.0
        out["servo_" + c["name"].replace(" ", "_")] = (app(box([r1 - hgt, -6.05, z0], [r1, 6.05, z0 + 23.2]), A), False, None)
        rs = 3.5 if c["role"] == "spread" else 2.5
        out["spool_" + c["name"].replace(" ", "_")] = (app(cyl(rs + 1.0, r1 + 0.2, r1 + 4.2, "x", (0, 0, z0 + 11.6)), A), False, "pacf")
    out["motor_base"] = (cyl(17, Zs["pack1"], Zs["pack1"] + 8), False, None)
    out["motor_bell"] = (cyl(17, Zs["pack1"] + 8.5, Zs["motor_top"]) + cyl(2.5, Zs["motor_top"], Zs["shaft_tip"]), True, None)
    return out


# ------------------------------------------------------------------ parts and assembly
_P = None
def parts():
    global _P
    if _P is None:
        _P = dict(root_grip=root_grip(), lag_tube=lag_tube(), hub_lower=hub_lower(), hub_upper=hub_upper(), cage=cage(), follower=follower(),
                  heel=heel(), pack_frame=pack_frame())
        for j in range(3): _P[f"phalanx{j + 1}"] = phalanx(j); _P[f"pad{j + 1}"] = pad(j)
        faz = follower_az()
        for c, a in zip(H.CHANNELS, faz): _P["claw_" + c["name"].replace(" ", "_")] = claw_bar(c, a)
    return _P


MAT = dict(root_grip="alu", lag_tube="steel", hub_lower="alu", hub_upper="alu", cage="pacf", follower="pacf", heel="tpu", pack_frame="pacf",
           phalanx1="pacf", phalanx2="pacf", phalanx3="pacf", pad1="tpu", pad2="tpu", pad3="tpu")


def finger_frames(k, lag=0.0, fold=(0, 0, 0)):
    """Pod-frame transforms of finger k: grip (frame F) and the three phalanges (chain frame)."""
    G = T(t=[0, 0, ZB]) @ T(rot("z", FAZ[k])) @ about("z", lag, [RL, 0, 0])
    C = G @ T(rot("x", H.PITCH)); out = [G]
    for j in range(3):
        C = C @ about("y", -fold[j], [KN[j], 0, 0]); out.append(C)
    return out


def assembly(spin=0.0, lag=(0, 0, 0, 0), fold=((0, 0, 0),) * 4, pull=None, hw=True, claw_pull=None):
    """Placed solids: name -> (solid, rotating?).  pull[i] = stroke of channel i (claw bar and follower move down together)."""
    P = parts(); S = T(rot("z", spin)); out = {}; pull = np.zeros(len(H.CHANNELS)) if pull is None else np.asarray(pull, float)
    claw_pull = pull if claw_pull is None else np.asarray(claw_pull, float)
    for n in ("hub_lower", "hub_upper", "cage", "heel"): out[n] = (app(P[n], S), True)
    for k in range(4):
        Fr = finger_frames(k, lag[k], fold[k])
        out[f"grip{k}"] = (app(P["root_grip"], S @ Fr[0]), True)
        out[f"tube{k}"] = (app(P["lag_tube"], S @ T(t=[0, 0, ZB]) @ T(rot("z", FAZ[k]))), True)
        for j in range(3):
            out[f"ph{k}_{j + 1}"] = (app(P[f"phalanx{j + 1}"], S @ Fr[j + 1]), True); out[f"pad{k}_{j + 1}"] = (app(P[f"pad{j + 1}"], S @ Fr[j + 1]), True)
    for i, (c, a) in enumerate(zip(H.CHANNELS, follower_az())):
        out[f"fol{i}"] = (app(P["follower"], S @ T(rot("z", a), [0, 0, -pull[i]])), True)
        out[f"claw{i}"] = (app(P["claw_" + c["name"].replace(" ", "_")], T(t=[0, 0, -claw_pull[i]])), False)
    out["pack_frame"] = (P["pack_frame"], False)
    if hw:
        for n, (m, rotating, _) in hardware().items(): out["hw_" + n] = (app(m, S) if rotating else m, rotating)
    return out


def export(folder="hand3_stl"):
    os.makedirs(folder, exist_ok=True); rep = {}
    for n, m in parts().items():
        t = to_trimesh(m); t.export(f"{folder}/hand3_{n}.stl")
        mat = MAT.get(n, "pacf")
        rep[n] = dict(watertight=bool(t.is_watertight), bodies=len(m.decompose()), vol_mm3=round(float(m.volume()), 1), material=mat,
                      mass_g=round(float(m.volume()) * RHO[mat] / 1000, 2), size=[round(float(x), 1) for x in t.extents])
    return rep


if __name__ == "__main__":
    rep = export(); json.dump(rep, open("hand3_cad_parts.json", "w"), indent=1)
    for n, r in rep.items(): print(f"{n:22s} wt {r['watertight']!s:5s} bodies {r['bodies']}  {r['vol_mm3']:9.1f} mm3  {r['mass_g']:6.2f} g {r['material']:5s} {r['size']}")
