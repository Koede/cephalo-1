"""Overlap check of the whole coupler + hand3 motion sim, by range math on occupancy grids (no mesh booleans), 2026-11-19.

Each rigid part is turned once into a grid of cells (CELL mm) in its own frame: a cell is solid when its centre lies inside
the part's cross-section at that height (manifold slice + point-in-polygon).  The grid is then eroded by one cell, so faces
that only touch (seats, sliding fits) do not count.  For every sampled frame and every pair of parts whose world-space
ranges (axis-aligned boxes) overlap, the solid cell centres of one part are mapped into the other's frame and looked up in
its eroded grid.  A pair is reported when cells overlap; depth ~ CELL x cells along the worst line.
Output: coupler_sim_overlap.json"""
import json, base64, sys
import numpy as np
from matplotlib.path import Path
import coupler_hand_sim as SIM

CELL = 0.5
STEP = int(sys.argv[1]) if len(sys.argv) > 1 else 10            # every STEP-th frame


def grid(name, part):
    v = np.frombuffer(base64.b64decode(part["v"]), np.float32).reshape(-1, 3).astype(float)
    t = np.frombuffer(base64.b64decode(part["t"]), np.uint32).reshape(-1, 3)
    lo = v.min(0) - CELL; hi = v.max(0) + CELL
    xs = np.arange(lo[0], hi[0], CELL) + CELL / 2; ys = np.arange(lo[1], hi[1], CELL) + CELL / 2; zs = np.arange(lo[2], hi[2], CELL) + CELL / 2
    occ = np.zeros((len(xs), len(ys), len(zs)), bool)
    X, Y = np.meshgrid(xs, ys, indexing="ij"); pts = np.c_[X.ravel(), Y.ravel()]
    tri = v[t]
    for k, z in enumerate(zs):
        # cross-section at z: segments where triangles cross the plane, joined by an even-odd ray count per cell centre
        a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
        za, zb, zc = a[:, 2] - z, b[:, 2] - z, c[:, 2] - z
        segs = []
        for (p, q, zp, zq) in ((a, b, za, zb), (b, c, zb, zc), (c, a, zc, za)):
            m = (zp > 0) != (zq > 0)
            s = zp[m] / (zp[m] - zq[m]); segs.append(p[m] + (q[m] - p[m]) * s[:, None])
        # each crossing triangle contributes 2 points; rebuild per triangle
        cross = ((za > 0) != (zb > 0)).astype(int) + ((zb > 0) != (zc > 0)) + ((zc > 0) != (za > 0))
        if not cross.any():
            continue
        P2 = []
        for (p, q, zp, zq) in ((a, b, za, zb), (b, c, zb, zc), (c, a, zc, za)):
            m = (zp > 0) != (zq > 0); s = np.where(m, zp / np.where(m, zp - zq, 1), 0)
            P2.append(np.where(m[:, None], p + (q - p) * s[:, None], np.nan))
        P2 = np.stack(P2, 1)                                  # (ntri, 3 edges, 3)
        ok = cross == 2
        e = P2[ok]; e = e[~np.isnan(e[:, :, 0])].reshape(-1, 2, 3)[:, :, :2]
        if not len(e):
            continue
        # even-odd: count crossings of a +x ray from each cell centre
        x0, y0 = e[:, 0, 0], e[:, 0, 1]; x1, y1 = e[:, 1, 0], e[:, 1, 1]
        inside = np.zeros(len(pts), bool)
        for i0 in range(0, len(pts), 20000):
            P = pts[i0:i0 + 20000]
            py = P[:, 1:2]; px = P[:, 0:1]
            cond = (y0[None] > py) != (y1[None] > py)
            xi = x0[None] + (py - y0[None]) * (x1 - x0)[None] / np.where(y1 - y0 == 0, 1e-12, y1 - y0)[None]
            inside[i0:i0 + 20000] = (np.sum(cond & (xi > px), 1) % 2) == 1
        occ[:, :, k] = inside.reshape(len(xs), len(ys))
    er = occ.copy()                                            # erode one cell (6-neighbourhood)
    er[1:] &= occ[:-1]; er[:-1] &= occ[1:]; er[:, 1:] &= occ[:, :-1]; er[:, :-1] &= occ[:, 1:]; er[:, :, 1:] &= occ[:, :, :-1]; er[:, :, :-1] &= occ[:, :, 1:]
    er[0] = er[-1] = False; er[:, 0] = er[:, -1] = False; er[:, :, 0] = er[:, :, -1] = False
    I = np.argwhere(occ)
    centres = np.c_[xs[I[:, 0]], ys[I[:, 1]], zs[I[:, 2]]]
    return dict(lo=lo, n=np.array(occ.shape), er=er, pts=centres, box=(v.min(0), v.max(0)))


def world_box(g, A):
    lo, hi = g["box"]; c = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    w = c @ A[:3, :3].T + A[:3, 3]; return w.min(0), w.max(0)


def main():
    parts, poses = SIM.build(); names = list(parts)
    G = {n: grid(n, parts[n]) for n in names}
    print("grids:", sum(len(g["pts"]) for g in G.values()), "solid cells", flush=True)
    ts, spins = SIM.spin_angles()
    frames = list(range(0, len(ts), STEP)) + [int(t * SIM.FPS) for t in (5.0, 7.0, 12.0, 18.0, 21.0, 22.0)]
    hits = {}
    for fi in sorted(set(frames)):
        s = SIM.state(ts[fi]); A = {n: poses[n](s, spins[fi]) for n in names}
        boxes = {n: world_box(G[n], A[n]) for n in names}
        for i, a in enumerate(names):
            for b in names[i + 1:]:
                la, ha = boxes[a]; lb, hb = boxes[b]
                if np.any(la > hb) or np.any(lb > ha):
                    continue                                   # world ranges apart
                if np.allclose(A[a], A[b]) and parts[a]["g"] == parts[b]["g"] and a.split(" ")[0] == b.split(" ")[0] == "finger":
                    pass
                # cells of a mapped into b's frame
                Mab = np.linalg.inv(A[b]) @ A[a]
                p = G[a]["pts"] @ Mab[:3, :3].T + Mab[:3, 3]
                idx = np.floor((p - G[b]["lo"]) / CELL).astype(int)
                okm = np.all((idx >= 0) & (idx < G[b]["n"]), 1)
                if not okm.any():
                    continue
                ii = idx[okm]; inb = G[b]["er"][ii[:, 0], ii[:, 1], ii[:, 2]]
                n_in = int(inb.sum())
                if n_in:
                    key = (a, b); vol = n_in * CELL ** 3
                    if key not in hits or vol > hits[key]["vol_mm3"]:
                        hits[key] = dict(a=a, b=b, t=round(float(ts[fi]), 2), vol_mm3=round(vol, 2), cells=n_in)
        print(f"t {ts[fi]:5.1f}: pairs overlapping so far {len(hits)}", flush=True)
    out = sorted(hits.values(), key=lambda h: -h["vol_mm3"])
    json.dump(dict(cell_mm=CELL, frames=len(set(frames)), overlaps=out), open("coupler_sim_overlap.json", "w"), indent=1)
    for h in out[:60]:
        print(f"{h['vol_mm3']:9.2f} mm3  t {h['t']:5.1f}  {h['a']}  x  {h['b']}")


if __name__ == "__main__":
    main()
