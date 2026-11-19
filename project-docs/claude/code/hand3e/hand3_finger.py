"""hand3 bench finger: the finger solids of the project's claude/code/hand3_cad.py (rev C), unchanged, exported in print
orientation for the first bench print ("one finger on a bench hub with three hand-pulled cords", hand3-design-study.md).
Parts: root grip (printed PA-CF for the bench; aluminium for flight), phalanges 1-3 (PA-CF), pads 1-3 (TPU 95A).
python3 hand3_finger.py -> hand3_finger_stl/*.stl"""
import os, json
import numpy as np
import manifold3d as mf
import trimesh
import hand3_design as H

M_ = mf.Manifold
KN, RT, RL = H.KN, H.R_TIP, H.R_LAG


def cyl(r, a, b, ax="z", c=(0, 0, 0), n=48):
    m = M_.cylinder(b - a, r, r, n).translate([0, 0, a])
    if ax == "x": m = m.rotate([0, 90, 0])
    elif ax == "y": m = m.rotate([-90, 0, 0])
    return m.translate(list(c))


def box(lo, hi):
    lo = np.asarray(lo, float); hi = np.asarray(hi, float)
    return M_.cube(list(np.abs(hi - lo))).translate(list(np.minimum(lo, hi)))


def half(m, n, p0):
    n = np.asarray(n, float); n = n / np.linalg.norm(n)
    return m.trim_by_plane(list(n), float(n @ np.asarray(p0, float)))


def wedge(m, c, a0, a1):
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


def foil(r, n=34):
    xs, yu, yl_ = H.section(r, n); c = H.chord(r)
    yc = (yu + yl_) / 2; ht = np.maximum((yu - yl_) / 2, 0.25)
    zc = float(np.interp(c / 2, xs, yc))
    return np.vstack([np.c_[c / 2 - xs, yc + ht - zc], np.c_[c / 2 - xs, yc - ht - zc][::-1]])


def pad_poly(r, lift=0.0, n=12):
    xs, yu, yl_ = H.section(r, 60); c = H.chord(r); zc = float(np.interp(c / 2, xs, (yu + yl_) / 2))
    xp = np.linspace(H.PAD_X[0] * c, H.PAD_X[1] * c, n); top = np.interp(xp, xs, yu) - zc
    return np.vstack([np.c_[c / 2 - xp, top + lift], np.c_[c / 2 - xp, top - H.PAD_T][::-1]])


def pad_span(j):
    x1 = KN[j + 1] - H.PAD_MARGIN if j < 2 else RT - 2.0
    return KN[j] + H.PAD_MARGIN, x1


def cord_slot(j):
    if j == 2: return None
    R = H.LEVER[j + 1]; xk = KN[j + 1]; y = H.CORD_Y[j + 1]
    return box([xk - R - 11, y - 0.55, -0.5], [xk - R - 3.5, y + 0.55, 8])


def pad(j, lift=0.0):
    a, b = pad_span(j)
    m = loft(np.linspace(a, b, 5), lambda r: pad_poly(r, lift))
    s = cord_slot(j)
    return m - s if (s is not None and lift == 0.0) else m


def end_trim(m, xk, s, fold):
    m = half(m, [s, 0, np.tan(np.radians(0.3))], [xk + 0.02 * s, 0, 0])
    return half(m, [s, 0, -np.tan(np.radians(fold / 2))], [xk + 0.02 * s, 0, 0])


def yl(j):
    g2 = H.GAP[j] / 2
    return g2, g2 + H.LUG[j], H.W_KN[j] / 2


def bores(m, j):
    g2, lo, W2 = yl(j)
    for s in (-1, 1): m -= cyl(H.BUSH_OD[j] / 2 + 0.02, min(s * (g2 - 0.01), s * (W2 + 1)), max(s * (g2 - 0.01), s * (W2 + 1)), "y", (KN[j], 0, 0))
    return m


def distal_end(m, j):
    g2, lo, W2 = yl(j); B = H.BARREL[j]; R = H.LEVER[j]; xk = KN[j]; yc = H.CORD_Y[j]; c = (xk, 0, 0)
    for s in (-1, 1):
        m -= cyl(B / 2 + 0.3, min(s * lo, s * (W2 + 0.3)), max(s * lo, s * (W2 + 0.3)), "y", c)
    for s in (-1, 1):
        a, b = sorted((s * g2, s * lo))
        lug = cyl(B / 2, a, b, "y", c)
        m += M_.batch_hull([lug, box([xk + 6.5, a, -0.8], [xk + 7.5, b, 0.3])])
        m += wedge(cyl(B / 2 + H.STOP_TOOTH, a, b, "y", c) - cyl(B / 2 - 0.6, a - 1, b + 1, "y", c), c, 180.0, 215.0)
    sec = wedge(cyl(R + 0.6, -(g2 + 0.5), g2 + 0.5, "y", c), c, -5.0, 115.0)
    sec -= cyl(R + 1.2, yc - 0.45, yc + 0.45, "y", c) - cyl(R - 0.35, yc - 0.5, yc + 0.5, "y", c)
    m += sec
    m -= cyl(1.7, -(g2 + 0.6), g2 + 0.6, "y", c)
    m -= app(cyl(0.55, 1.0, R + 1.5), about("y", 100.0, c) @ T(t=[xk, yc, 0]))
    return bores(m, j)


def proximal_end(m, j, fairlead=True):
    g2, lo, W2 = yl(j); B = H.BARREL[j]; R = H.LEVER[j]; xk = KN[j]; yc = H.CORD_Y[j]; c = (xk, 0, 0)
    for s in (-1, 1):
        a, b = sorted((s * (W2 - H.CHEEK[j]), s * W2)); a2, b2 = sorted((s * (W2 - H.CHEEK[j] - 2.5), s * (W2 - 0.5)))
        wx = 3.0 if j == 0 else 7.5
        m += M_.batch_hull([cyl(B / 2, a, b, "y", c), box([xk - wx, a2, -0.8], [xk - wx + 1.0, b2, 0.3])])
    m -= cyl(B / 2 + 0.3, -(lo + 0.1), lo + 0.1, "y", c)
    m -= cyl(R + 1.0, -(g2 + 0.8), g2 + 0.8, "y", c)
    for s in (-1, 1):
        a, b = sorted((s * (g2 + 0.2), s * (lo - 0.1)))
        m += wedge(cyl(B / 2 + H.STOP_TOOTH + 0.6, a, b, "y", c) - cyl(B / 2 + 0.3, a - 1, b + 1, "y", c), c, 215.0 + H.PRECONE, 262.0)
    if fairlead:
        post = box([xk - R - 3.6, yc - 1.5, -0.3], [xk - R - 1.6, yc + 1.5, R + 1.2]) - cyl(0.55, xk - R - 4, xk - R - 1, "x", (0, yc, R))
        m += post
    return bores(m, j)


def channels(m, j):
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
    if j == 1 and getattr(H, "COUPLED_TIP", False):
        # rev D: the middle cord does not end at K2; it leaves the K2 sector through its anchor hole (y 0) and joins the tip
        # cord channel (y -0.9) that runs on to the K3 fairlead and sector, where it is anchored
        m -= box([KN[1] + 2.5, H.CORD_Y[2] - 0.5, -1.1], [KN[1] + 9.0, H.CORD_Y[1] + 0.5, 0.45])
    return channels(m, j)


RIM_RELIEF = 0.2                                              # mm between the hub-plate rim and anything of the grip that hangs below its flat


def rim_relief(clear=RIM_RELIEF, n=96):
    """Hub-plate rim relief (rev D: 16 deg pitch).  At full spread the K1 back-stop boss of the pitched fork swings inside
    the lower hub plate's radius below the flat's underside (0.02 mm3 graze).  The cut is the plate's rim grown by `clear`,
    as the grip sees it at SPREAD_MAX (grip frame F: plate axis turned about the lag pin), below the flat only (z < -2.4)."""
    r = (H.HUB["plate_d"] / 2 + clear) / np.cos(np.pi / n)   # polygon round the circle: `clear` at mid-facet too
    out = None
    for s in sorted(set(H.LAG_SIGN)):                         # either swing direction (the same grip on all four fingers)
        c = app(cyl(r, -12.0, -2.4, n=n), about("z", -s * H.SPREAD_MAX, [RL, 0, 0]))
        out = c if out is None else out + c
    return out


def root_grip():
    flat = box([14.5, -6, -2.4], [31.6, 6, 2.4]) - cyl(H.LAG_TUBE[0] / 2 + 0.03, -4, 4, c=(RL, 0, 0))
    flat -= cyl(0.7, -4, 4, c=(RL - 3.5, 0, 0))
    W2 = H.W_KN[0] / 2
    fork = M_.batch_hull([box([31.0, -6, -2.6], [32.0, 6, 2.6]), box([34.0, -W2, -3.0], [KN[0] + 0.5, W2, 3.0])])
    m = app(flat, T(rot("x", -H.PITCH))) + fork
    m = end_trim(m, KN[0], -1, H.FOLD_MAX[0])
    return app(channels(proximal_end(m, 0, fairlead=False), -1), T(rot("x", H.PITCH))) - rim_relief()


def on_bed(m, flip=False):
    """Print orientation: blades on their back face (grip face up), layers along the span; pads flat."""
    if flip: m = m.rotate([180, 0, 0])
    b = np.asarray(m.bounding_box()).reshape(2, 3) if np.asarray(m.bounding_box()).size == 6 else None
    lo = np.array(m.bounding_box()[:3])
    return m.translate([-float(lo[0]), -float(lo[1]), -float(lo[2])])


if __name__ == "__main__":
    os.makedirs("hand3_finger_stl", exist_ok=True); rep = {}
    parts = {"root_grip_benchPACF": root_grip(), "phalanx1": phalanx(0), "phalanx2": phalanx(1), "phalanx3": phalanx(2),
             "pad1_TPU": pad(0), "pad2_TPU": pad(1), "pad3_TPU": pad(2)}
    rho = dict(TPU=1.20)
    for n, m in parts.items():
        m = on_bed(m)
        t = to_trimesh(m); t.export(f"hand3_finger_stl/hand3_{n}.stl")
        rep[n] = dict(watertight=bool(t.is_watertight), bodies=len(m.decompose()), mass_g=round(m.volume() * (1.20 if "TPU" in n else 1.25) / 1000, 2),
                      size_mm=[round(float(x), 1) for x in t.extents])
        print(f"{n:22s}", rep[n])
    json.dump(rep, open("hand3_finger_stl/parts.json", "w"), indent=1)
