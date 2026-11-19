"""Where can the motor phase leads run through hand3 hub E?  Free-space search on a 1 mm grid (range math), 2026-11-19.

Obstacles, in the connector frame, from the motion sim's parts (coupler_hand_sim.build):
  * static parts (hand base, hub E frame, servos, spools, idlers, rods, tie ring, collar swept through its turn ...);
  * rotating parts swept through a full turn (rotor, cage, rings, fingers at every sampled pose) -> occupied (r, z) cells;
  * pull rings and yokes swept over their whole stroke.
The hand base is solid except a 3.8 mm groove layer in its bottom face (grooves can be cut there).  Free space must keep
a bundle of the three 14 AWG leads (dia 7.5) clear by 0.3 mm: Euclidean distance transform >= 4.05 mm.
Start: under each bullet in the groove layer.  Goal: beside the motor base (BrotherHobby SE 2816, base at hand3 z 58..66,
r 17), where its leads leave the can.  Dijkstra on the 26-neighbour grid.  Writes coupler_lead_route.json + .png."""
import json, heapq, base64
import numpy as np
from scipy import ndimage
import coupler_hand_sim as SIM
import coupler_sim_overlap as OV

import os
P = SIM.P; CELL = 1.0; BUNDLE_R = 3.77 + 0.3; SINGLE_R = float(os.environ.get("LEAD_CLR", 2.05))
LO = np.array([-50.0, -50.0, -82.0]); HI = np.array([50.0, 50.0, 2.0])
N = np.ceil((HI - LO) / CELL).astype(int)


def splat(occ, pts):
    idx = np.floor((pts - LO) / CELL).astype(int); ok = np.all((idx >= 0) & (idx < N), 1); idx = idx[ok]
    occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True


def main():
    parts, poses = SIM.build(); names = list(parts)
    G = {n: OV.grid(n, parts[n]) for n in names}
    occ = np.zeros(N, bool); frm = np.zeros(N, bool); rz = {}                      # swept (r, z) cells of rotating parts
    states = [SIM.state(t) for t in (12.0, 16.0, 18.0, 19.5, 21.0, 22.0)]
    for n in names:
        g = parts[n]["g"]
        if g == "arm" or n.startswith(("motor phase leads", "male bullets")):
            continue
        for s in (states if g in ("rotor", "hand3") else states[:1]):
            for th in ((0.0,) if n != "locking collar" else (-60.0, -30.0, 0.0)):
                s2 = dict(s, th=th, dz=0.0)
                A = poses[n](s2, 0.0); p = G[n]["pts"] @ A[:3, :3].T + A[:3, 3]
                if g == "rotor" or n == "locking collar":
                    r = np.hypot(p[:, 0], p[:, 1]); key = np.c_[np.floor(r / CELL), np.floor((p[:, 2] - LO[2]) / CELL)].astype(int)
                    for a, b in np.unique(key, axis=0): rz.setdefault(int(b), set()).add(int(a))
                else:
                    if n == "hand base":
                        p = p[~((p[:, 2] < P["z_flange"][0] + P["groove_h"]) & (np.hypot(p[:, 0], p[:, 1]) < P["R_flange"] - 1.0))]
                    if n == "hub E frame":
                        splat(frm, p); continue
                    splat(occ, p)
    xs = LO[0] + (np.arange(N[0]) + 0.5) * CELL; ys = LO[1] + (np.arange(N[1]) + 0.5) * CELL
    R2 = np.hypot(*np.meshgrid(xs, ys, indexing="ij"))
    for kz, rs in rz.items():
        if 0 <= kz < N[2]:
            ri = np.floor(R2 / CELL).astype(int); occ[:, :, kz] |= np.isin(ri, list(rs))
    # keep the hand base's bullet bores, the 3 inserts and the male bullets as obstacles in the groove layer
    for az in P["hand_screw_az"]:
        c = np.array(SIM.HB.at(P["hand_screw_r"], az)); m = np.hypot(*(np.stack(np.meshgrid(xs, ys, indexing="ij"), -1) - c).transpose(2, 0, 1)) < 2.0
        zz = np.arange(N[2]); zc = LO[2] + (zz + 0.5) * CELL; occ[:, :, (zc > P["z_flange"][0] - 3.5) & (zc < P["z_flange"][0] + 6)] |= m[:, :, None]
    hard = occ & ~frm
    dist = ndimage.distance_transform_edt(~hard) * CELL          # clearance to anything that is not the hub E frame
    zc = LO[2] + (np.arange(N[2]) + 0.5) * CELL; fz0 = P["z_flange"][0]
    # goal: the motor's leads drop out of its base into the frame's top block centre hole (hand3 z 52..57, r < 10)
    Zg = (zc < fz0 - 52.0) & (zc > fz0 - 57.0)
    goal = (dist >= SINGLE_R) & Zg[None, None, :] & (R2 < 10.0)[:, :, None]
    print("goal cells:", int(goal.sum()), flush=True)
    res = {}
    for az in P["az_bullet"]:
        sxy = np.array(SIM.HB.at(P["r_bullet"], az)); s0 = np.floor((np.r_[sxy, fz0 + 1.9] - LO) / CELL).astype(int)
        path = dijkstra(dist >= SINGLE_R, dist, tuple(s0), goal, frm)
        if path is None:
            print("phase", az, ": NO ROUTE"); res[f"{az:.0f}"] = None; continue
        pp = np.array([[LO[i] + (c[i] + 0.5) * CELL for i in range(3)] for c in path]); res[f"{az:.0f}"] = pp.tolist()
        cut = sum(frm[c] for c in path); L = np.sum(np.linalg.norm(np.diff(pp, axis=0), axis=1))
        print(f"phase {az:.0f}: {L:.0f} mm, through the hub E frame for {cut} mm; min clearance {min(dist[c] for c in path):.1f}", flush=True)
        for c, q in zip(path[::4], pp[::4]):
            print(f"   h3 z {fz0 - q[2]:5.1f}  r {np.hypot(q[0], q[1]):5.1f}  az(conn) {np.degrees(np.arctan2(q[1], q[0])):6.1f}  {'FRAME' if frm[c] else ''}")
    json.dump(dict(cell=CELL, routes=res), open("coupler_lead_route.json", "w"))


def dijkstra(ok, dist, s, goal, frm=None):
    nb = [(i, j, k) for i in (-1, 0, 1) for j in (-1, 0, 1) for k in (-1, 0, 1) if (i, j, k) != (0, 0, 0)]
    w = {d: np.sqrt(d[0] ** 2 + d[1] ** 2 + d[2] ** 2) for d in nb}
    if not ok[s]:
        # step off the start inside the groove layer to the nearest ok cell
        pass
    D = {s: 0.0}; prev = {}; h = [(0.0, s)]; seen = set()
    while h:
        d, u = heapq.heappop(h)
        if u in seen: continue
        seen.add(u)
        if goal[u]:
            path = [u]
            while path[-1] in prev: path.append(prev[path[-1]])
            return path[::-1]
        for dd in nb:
            v = (u[0] + dd[0], u[1] + dd[1], u[2] + dd[2])
            if min(v) < 0 or v[0] >= ok.shape[0] or v[1] >= ok.shape[1] or v[2] >= ok.shape[2] or not ok[v] and dist[v] < 1.0: continue
            if not ok[v]: continue
            nd = d + w[dd] + (2.0 if dist[v] < BUNDLE_R + 1 else 0.0) + (6.0 * w[dd] if frm is not None and frm[v] else 0.0)   # roomy cells; cutting frame costs
            if nd < D.get(v, 1e18): D[v] = nd; prev[v] = u; heapq.heappush(h, (nd, v))
    return None


if __name__ == "__main__":
    main()
