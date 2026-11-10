"""Headless numba z-buffer rasterizer used for all Octopus Drone renders and videos.

Scene = list of primitives, each (V (n,3) world metres, F (m,3) int, C (m,3) rgb 0..1, pid int).
Camera: look-at perspective. Output: uint8 RGB image (H, W, 3).
Features: two-sided flat Lambert (key + fill + rim), planar ground shadow, depth/ID outlines, distance fog, 2x supersampling.
"""
import numpy as np
from numba import njit

GROUND_ID = 1


@njit(cache=True)
def _raster(sx, sy, sz, F, fc, fid, W, H, col, dep, ids):
    for t in range(F.shape[0]):
        a, b, c = F[t, 0], F[t, 1], F[t, 2]
        z0, z1, z2 = sz[a], sz[b], sz[c]
        if z0 <= 0.02 or z1 <= 0.02 or z2 <= 0.02:
            continue
        x0, y0, x1, y1, x2, y2 = sx[a], sy[a], sx[b], sy[b], sx[c], sy[c]
        area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        if abs(area) < 1e-9:
            continue
        xmin = max(int(min(x0, x1, x2)), 0); xmax = min(int(max(x0, x1, x2)) + 1, W - 1)
        ymin = max(int(min(y0, y1, y2)), 0); ymax = min(int(max(y0, y1, y2)) + 1, H - 1)
        if xmin > xmax or ymin > ymax:
            continue
        iz0, iz1, iz2 = 1.0 / z0, 1.0 / z1, 1.0 / z2
        inv = 1.0 / area
        for py in range(ymin, ymax + 1):
            fy = py + 0.5
            for px in range(xmin, xmax + 1):
                fx = px + 0.5
                w0 = ((x1 - fx) * (y2 - fy) - (x2 - fx) * (y1 - fy)) * inv
                w1 = ((x2 - fx) * (y0 - fy) - (x0 - fx) * (y2 - fy)) * inv
                w2 = 1.0 - w0 - w1
                if w0 < 0 or w1 < 0 or w2 < 0:
                    continue
                d = 1.0 / (w0 * iz0 + w1 * iz1 + w2 * iz2)
                if d < dep[py, px]:
                    dep[py, px] = d
                    ids[py, px] = fid[t]
                    col[py, px, 0] = fc[t, 0]; col[py, px, 1] = fc[t, 1]; col[py, px, 2] = fc[t, 2]


@njit(cache=True)
def _mask(sx, sy, sz, F, W, H, m):
    for t in range(F.shape[0]):
        a, b, c = F[t, 0], F[t, 1], F[t, 2]
        if sz[a] <= 0.02 or sz[b] <= 0.02 or sz[c] <= 0.02:
            continue
        x0, y0, x1, y1, x2, y2 = sx[a], sy[a], sx[b], sy[b], sx[c], sy[c]
        area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
        if abs(area) < 1e-9:
            continue
        xmin = max(int(min(x0, x1, x2)), 0); xmax = min(int(max(x0, x1, x2)) + 1, W - 1)
        ymin = max(int(min(y0, y1, y2)), 0); ymax = min(int(max(y0, y1, y2)) + 1, H - 1)
        inv = 1.0 / area
        for py in range(ymin, ymax + 1):
            fy = py + 0.5
            for px in range(xmin, xmax + 1):
                fx = px + 0.5
                w0 = ((x1 - fx) * (y2 - fy) - (x2 - fx) * (y1 - fy)) * inv
                w1 = ((x2 - fx) * (y0 - fy) - (x0 - fx) * (y2 - fy)) * inv
                if w0 < 0 or w1 < 0 or w0 + w1 > 1:
                    continue
                m[py, px] = 1


class Camera:
    def __init__(self, eye, target, fov_deg=40.0, up=(0, 0, 1)):
        self.eye = np.asarray(eye, float)
        f = np.asarray(target, float) - self.eye
        f /= np.linalg.norm(f)
        r = np.cross(f, up); r /= np.linalg.norm(r)
        u = np.cross(r, f)
        self.R = np.stack([r, u, f])  # rows: right, up, forward
        self.f = 1.0 / np.tan(np.radians(fov_deg) / 2)

    def project(self, P, W, H):
        q = (P - self.eye) @ self.R.T
        z = q[:, 2]
        zs = np.where(np.abs(z) < 1e-6, 1e-6, z)
        s = self.f * (H / 2)
        return W / 2 + q[:, 0] / zs * s, H / 2 - q[:, 1] / zs * s, z


LIGHT = np.array([-0.35, -0.25, -1.0]); LIGHT /= np.linalg.norm(LIGHT)   # direction light travels
FILL = np.array([0.6, 0.4, -0.3]); FILL /= np.linalg.norm(FILL)
SKY_TOP = np.array([0.80, 0.86, 0.92]); SKY_BOT = np.array([0.95, 0.96, 0.97])


def ground_mesh(cx=0.0, cy=0.0, half=6.0, tile=0.25):
    n = int(2 * half / tile)
    xs = np.linspace(cx - half, cx + half, n + 1); ys = np.linspace(cy - half, cy + half, n + 1)
    X, Y = np.meshgrid(xs, ys, indexing="ij")
    V = np.stack([X.ravel(), Y.ravel(), np.zeros(X.size)], 1)
    i, j = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    i, j = i.ravel(), j.ravel()
    a = i * (n + 1) + j; b = a + (n + 1); c = b + 1; d = a + 1
    F = np.concatenate([np.stack([a, b, c], 1), np.stack([a, c, d], 1)])
    chk = ((np.floor((xs[i] + 1e-6 + 100) / tile) + np.floor((ys[j] + 100) / tile)) % 2).astype(bool)
    chk = np.concatenate([chk, chk])
    C = np.where(chk[:, None], [0.86, 0.85, 0.82], [0.80, 0.79, 0.76])
    return V, F, C


def render(prims, cam, W, H, ss=2, ground=(0.0, 0.0), shadow=True, fog=(3.0, 9.0)):
    Ws, Hs = W * ss, H * ss
    gV, gF, gC = ground_mesh(*ground)
    Vs, Fs, Cs, Is = [gV], [gF], [gC], [np.full(len(gF), GROUND_ID)]
    off = len(gV)
    objV, objF = [], []
    o2 = 0
    for V, F, C, pid in prims:
        Vs.append(V); Fs.append(F + off); Cs.append(C); Is.append(np.full(len(F), pid))
        objV.append(V); objF.append(F + o2)
        off += len(V); o2 += len(V)
    V = np.concatenate(Vs); F = np.concatenate(Fs).astype(np.int64); C = np.concatenate(Cs); I = np.concatenate(Is).astype(np.int64)
    # shading
    e1 = V[F[:, 1]] - V[F[:, 0]]; e2 = V[F[:, 2]] - V[F[:, 0]]
    n = np.cross(e1, e2); n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    cen = V[F].mean(1)
    view = cen - cam.eye
    flip = np.einsum("ij,ij->i", n, view) > 0
    n[flip] *= -1
    key = np.clip(-n @ LIGHT, 0, 1); fill = np.clip(-n @ FILL, 0, 1)
    rim = np.clip(1 - np.abs(np.einsum("ij,ij->i", n, view / np.linalg.norm(view, axis=1, keepdims=True))), 0, 1) ** 3
    shade = 0.38 + 0.62 * key + 0.18 * fill + 0.15 * rim
    isg = I == GROUND_ID
    shade[isg] = 1.0
    fc = np.clip(C * shade[:, None], 0, 1)
    sx, sy, sz = cam.project(V, Ws, Hs)
    col = np.zeros((Hs, Ws, 3)); dep = np.full((Hs, Ws), 1e9); ids = np.zeros((Hs, Ws), np.int64)
    _raster(sx, sy, sz, F, fc, I, Ws, Hs, col, dep, ids)
    # sky
    sky = dep >= 1e8
    t = np.linspace(0, 1, Hs)[:, None, None]
    skyimg = SKY_TOP * (1 - t) + SKY_BOT * t
    col = np.where(sky[..., None], np.broadcast_to(skyimg, col.shape), col)
    # shadow
    if shadow and objV:
        P = np.concatenate(objV); Fo = np.concatenate(objF).astype(np.int64)
        k = np.clip(P[:, 2], 0, None) / -LIGHT[2]
        Ps = P + k[:, None] * LIGHT
        Ps[:, 2] = 0.0
        qx, qy, qz = cam.project(Ps, Ws, Hs)
        m = np.zeros((Hs, Ws), np.uint8)
        _mask(qx, qy, qz, Fo, Ws, Hs, m)
        gm = (m == 1) & (ids == GROUND_ID)
        col[gm] *= 0.62
    # outlines
    d = dep.copy(); d[sky] = 1e3
    edge = np.zeros((Hs, Ws), bool)
    for ax in (0, 1):
        dd = np.abs(np.diff(d, axis=ax)); di = np.diff(ids, axis=ax) != 0
        rel = dd / np.minimum(np.take(d, range(d.shape[ax] - 1), axis=ax), np.take(d, range(1, d.shape[ax]), axis=ax))
        e = (di & (ids.take(range(ids.shape[ax] - 1), axis=ax) != GROUND_ID) | (rel > 0.04)) & ~(
            (ids.take(range(ids.shape[ax] - 1), axis=ax) == GROUND_ID) & (ids.take(range(1, ids.shape[ax]), axis=ax) == GROUND_ID))
        if ax == 0:
            edge[:-1] |= e
        else:
            edge[:, :-1] |= e
    col[edge] *= 0.25
    # fog
    fz = np.clip((d - fog[0]) / (fog[1] - fog[0]), 0, 1)[..., None] * 0.85
    fz[sky] = 0
    col = col * (1 - fz) + SKY_BOT * fz
    img = col.reshape(H, ss, W, ss, 3).mean((1, 3))
    return (np.clip(img, 0, 1) * 255).astype(np.uint8)
