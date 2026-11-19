"""Mesh primitives (metres). Each returns (V, F, C) with per-face colours; Scene.add() tags them with a part id."""
import numpy as np


def frame_z(d, ref=None):
    """Rotation matrix whose 3rd column is unit vector d."""
    z = np.asarray(d, float); z = z / np.linalg.norm(z)
    if ref is None:
        ref = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = np.asarray(ref, float) - z * np.dot(ref, z)
    if np.linalg.norm(x) < 1e-9:
        x = np.array([0, 1.0, 0]) - z * z[1]
    x /= np.linalg.norm(x)
    return np.stack([x, np.cross(z, x), z], 1)


def rot(axis, ang):
    a = np.asarray(axis, float); a = a / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def _col(nf, c, c2=None, pattern=None):
    C = np.tile(np.asarray(c, float), (nf, 1))
    if c2 is not None and pattern is not None:
        C[pattern] = c2
    return C


def frustum(p0, p1, r0, r1, c, n=14, caps=True, ref=None, c2=None, stripes=0):
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    d = p1 - p0; L = np.linalg.norm(d)
    if L < 1e-9:
        d = np.array([0, 0, 1.0]); L = 1e-9
    R = frame_z(d, ref)
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.stack([np.cos(th), np.sin(th), np.zeros(n)], 1)
    V = np.concatenate([ring * [r0, r0, 0], ring * [r1, r1, 0] + [0, 0, L]])
    i = np.arange(n); j = (i + 1) % n
    F = [np.stack([i, j, n + j], 1), np.stack([i, n + j, n + i], 1)]
    side = 2 * n
    if caps:
        V = np.concatenate([V, [[0, 0, 0], [0, 0, L]]])
        F += [np.stack([np.full(n, 2 * n), j, i], 1), np.stack([np.full(n, 2 * n + 1), n + i, n + j], 1)]
    F = np.concatenate(F)
    pat = None
    if stripes:
        pat = np.zeros(len(F), bool)
        s = (i * stripes // n) % 2 == 1
        pat[:side] = np.concatenate([s, s])
    return V @ R.T + p0, F, _col(len(F), c, c2, pat)


def sphere(c, r, col, n=16, m=10, R=None, col2=None, checker=0):
    th = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ph = np.linspace(0, np.pi, m + 1)[1:-1]
    P, T = np.meshgrid(ph, th, indexing="ij")
    V = np.stack([np.sin(P) * np.cos(T), np.sin(P) * np.sin(T), np.cos(P)], -1).reshape(-1, 3)
    V = np.concatenate([V, [[0, 0, 1], [0, 0, -1]]])
    F, pat = [], []
    for a in range(m - 2):
        for b in range(n):
            i0 = a * n + b; i1 = a * n + (b + 1) % n; i2 = (a + 1) * n + (b + 1) % n; i3 = (a + 1) * n + b
            F += [[i0, i1, i2], [i0, i2, i3]]
            k = ((a + 1) * checker // m + b * checker // n) % 2 if checker else 0
            pat += [k, k]
    top, bot = len(V) - 2, len(V) - 1
    for b in range(n):
        F.append([top, (b + 1) % n, b]); pat.append(0)
        F.append([bot, (m - 2) * n + b, (m - 2) * n + (b + 1) % n]); pat.append(1 if checker else 0)
    F = np.array(F)
    if R is None:
        R = np.eye(3)
    return (V * r) @ R.T + c, F, _col(len(F), col, col2, np.array(pat, bool) if checker else None)


def torus(c, axis, Rmaj, rmin, col, n=28, m=8, ref=None):
    R = frame_z(axis, ref)
    u = np.linspace(0, 2 * np.pi, n, endpoint=False); v = np.linspace(0, 2 * np.pi, m, endpoint=False)
    U, Vv = np.meshgrid(u, v, indexing="ij")
    X = (Rmaj + rmin * np.cos(Vv)) * np.cos(U); Y = (Rmaj + rmin * np.cos(Vv)) * np.sin(U); Z = rmin * np.sin(Vv)
    V = np.stack([X, Y, Z], -1).reshape(-1, 3)
    i, j = np.meshgrid(np.arange(n), np.arange(m), indexing="ij")
    i, j = i.ravel(), j.ravel()
    a = i * m + j; b = ((i + 1) % n) * m + j; cc = ((i + 1) % n) * m + (j + 1) % m; d = i * m + (j + 1) % m
    F = np.concatenate([np.stack([a, b, cc], 1), np.stack([a, cc, d], 1)])
    return V @ R.T + c, F, _col(len(F), col)


def box(c, R, size, col):
    s = np.asarray(size, float) / 2
    V = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float) * s
    F = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                  [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    return V @ np.asarray(R).T + c, F, _col(12, col)


def tube_path(pts, r, col, n=8):
    """Chain of short cylinders along a polyline (for curved links)."""
    out = []
    for a, b in zip(pts[:-1], pts[1:]):
        out.append(frustum(a, b, r, r, col, n=n, caps=True))
    return merge(out)


def merge(parts):
    Vs, Fs, Cs, off = [], [], [], 0
    for V, F, C in parts:
        Vs.append(V); Fs.append(F + off); Cs.append(C); off += len(V)
    return np.concatenate(Vs), np.concatenate(Fs), np.concatenate(Cs)


class Scene:
    def __init__(self):
        self.prims = []
        self._id = 10

    def add(self, mesh, same=False):
        if not same:
            self._id += 1
        V, F, C = mesh
        self.prims.append((V, F, C, self._id))
