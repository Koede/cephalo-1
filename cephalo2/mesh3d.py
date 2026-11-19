"""LOCAL STAND-IN for claude/code/mesh3d.py: only tube / disc / box as triangle arrays (k, 3, 3), which is all that
xlclaw.meshes() needs for the motor body. Not uploaded: the project's mesh3d.py is the real one."""
import numpy as np


def _frame(d):
    z = np.asarray(d, float); z = z / np.linalg.norm(z)
    ref = np.array([1.0, 0, 0]) if abs(z[0]) < 0.9 else np.array([0, 1.0, 0])
    x = ref - z * (ref @ z); x /= np.linalg.norm(x)
    return x, np.cross(z, x), z


def tube(P, r, n=16):
    P = np.asarray(P, float); x, y, z = _frame(P[1] - P[0])
    a = np.linspace(0, 2 * np.pi, n + 1)
    ring = np.cos(a)[:, None] * x * r + np.sin(a)[:, None] * y * r
    A, B = P[0] + ring, P[1] + ring
    return np.array([t for i in range(n) for t in ([A[i], A[i + 1], B[i + 1]], [A[i], B[i + 1], B[i]])])


def disc(c, nrm, r, n=16):
    c = np.asarray(c, float); x, y, z = _frame(nrm)
    a = np.linspace(0, 2 * np.pi, n + 1)
    ring = c + np.cos(a)[:, None] * x * r + np.sin(a)[:, None] * y * r
    return np.array([[c, ring[i], ring[i + 1]] for i in range(n)])


def box(c, R, half):
    R = np.asarray(R, float); h = np.asarray(half, float)
    V = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float) * h
    V = V @ R.T + np.asarray(c, float)
    F = [[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1], [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]]
    return V[np.array(F)]
