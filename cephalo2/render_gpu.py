"""CUDA (numba.cuda) versions of the two rasterizer kernels in render.py, used when CEPH_GPU=1.

One thread per triangle.  Depth test without races: each pixel holds a uint64 key = (depth in micrometres << 32) | tri,
updated with atomic min, so the nearest triangle wins regardless of thread order; colour and id are looked up from the
winning triangle afterwards on the host.  Output is identical to the CPU path (ties broken by triangle index).
"""
import numpy as np
from numba import cuda

KEY_EMPTY = np.uint64(0xFFFFFFFFFFFFFFFF)


@cuda.jit
def _raster_k(sx, sy, sz, F, W, H, zbuf):
    t = cuda.grid(1)
    if t >= F.shape[0]:
        return
    a = F[t, 0]; b = F[t, 1]; c = F[t, 2]
    z0 = sz[a]; z1 = sz[b]; z2 = sz[c]
    if z0 <= 0.02 or z1 <= 0.02 or z2 <= 0.02:
        return
    x0 = sx[a]; y0 = sy[a]; x1 = sx[b]; y1 = sy[b]; x2 = sx[c]; y2 = sy[c]
    area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    if abs(area) < 1e-9:
        return
    xmin = max(int(min(x0, x1, x2)), 0); xmax = min(int(max(x0, x1, x2)) + 1, W - 1)
    ymin = max(int(min(y0, y1, y2)), 0); ymax = min(int(max(y0, y1, y2)) + 1, H - 1)
    if xmin > xmax or ymin > ymax:
        return
    iz0 = 1.0 / z0; iz1 = 1.0 / z1; iz2 = 1.0 / z2
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
            key = (np.uint64(d * 1e6) << np.uint64(32)) | np.uint64(t)
            cuda.atomic.min(zbuf, (py, px), key)


@cuda.jit
def _mask_k(sx, sy, sz, F, W, H, m):
    t = cuda.grid(1)
    if t >= F.shape[0]:
        return
    a = F[t, 0]; b = F[t, 1]; c = F[t, 2]
    if sz[a] <= 0.02 or sz[b] <= 0.02 or sz[c] <= 0.02:
        return
    x0 = sx[a]; y0 = sy[a]; x1 = sx[b]; y1 = sy[b]; x2 = sx[c]; y2 = sy[c]
    area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0)
    if abs(area) < 1e-9:
        return
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


def raster(sx, sy, sz, F, fc, fid, W, H):
    """GPU replacement for render._raster: returns col (H,W,3), dep (H,W), ids (H,W)."""
    zb = cuda.to_device(np.full((H, W), KEY_EMPTY, np.uint64))
    n = F.shape[0]; tpb = 128
    _raster_k[(n + tpb - 1) // tpb, tpb](*[cuda.to_device(np.ascontiguousarray(x)) for x in (sx, sy, sz, F)], W, H, zb)
    key = zb.copy_to_host()
    hit = key != KEY_EMPTY
    tri = (key & np.uint64(0xFFFFFFFF)).astype(np.int64)
    dep = np.where(hit, (key >> np.uint64(32)).astype(np.float64) * 1e-6, 1e9)
    col = np.where(hit[..., None], fc[np.where(hit, tri, 0)], 0.0)
    ids = np.where(hit, fid[np.where(hit, tri, 0)], 0)
    return col, dep, ids


def mask(sx, sy, sz, F, W, H):
    m = cuda.to_device(np.zeros((H, W), np.uint8))
    n = F.shape[0]; tpb = 128
    _mask_k[(n + tpb - 1) // tpb, tpb](*[cuda.to_device(np.ascontiguousarray(x)) for x in (sx, sy, sz, F)], W, H, m)
    return m.copy_to_host()
