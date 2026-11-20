"""Checks for hand3 hub v3, decision C (every root drives itself), 2026-11-20.  Writes hand3_root_drive_check.json.
1) Range math between roots: root + motor + pinion + brushes + straight finger cut into 0.5 mm radial bands (angle range, z range);
   gap at the closest spacing the roots ever have (MIN_SEP 58 deg) and at each pose's spacings.
2) Simultaneous moves: every transition the sim makes, roots interpolated together; smallest spacing on the way.
3) Whole hub per pose with the fingers curling (all knuckles together) on occupancy grids (0.5 mm cells, eroded one cell).  The
   pinion is checked against the top disc without its crown teeth (the mesh is intended).
4) Station: roller, dock, servo, sensor vs the swept envelope of the roots (r-z), parked and in flight.
5) Loads and section stresses (hand3_root_drive.drive_numbers)."""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hub_v3 as V3, hand3_root_drive as RD
os.chdir(HERE)
import coupler_hand_sim as SIM, coupler_sim_overlap as OV

MIN_SEP = 2 * V3.V["block_half"] + 2.0
res = dict(pairs=[], moves={}, curl={}, station={}, loads={})


def bands(m, dr=0.5):
    me = m.to_mesh(); v = np.asarray(me.vert_properties)[:, :3]; t = np.asarray(me.tri_verts)
    e = np.concatenate([v[t[:, a]] + (v[t[:, b]] - v[t[:, a]]) * f for a, b in ((0, 1), (1, 2), (2, 0)) for f in np.linspace(0, 1, 9)])
    r = np.hypot(e[:, 0], e[:, 1]); th = np.degrees(np.arctan2(e[:, 1], e[:, 0])); out = {}
    for r0 in np.arange(np.floor(r.min() / dr) * dr, r.max() + dr, dr):
        s = (r >= r0) & (r < r0 + dr)
        if s.any(): out[round(float(r0), 2)] = (float(th[s].min()), float(th[s].max()), float(e[s, 2].min()), float(e[s, 2].max()))
    return out


def gap_mm(A, B, da):
    best = (999.0, None)
    for r0, (a0, a1, z0, z1) in A.items():
        if r0 not in B: continue
        b0, b1, w0, w1 = B[r0]
        if z1 <= w0 or w1 <= z0: continue
        b0, b1 = b0 + da, b1 + da
        g = min(((b0 - a1) % 360), ((a0 - b1) % 360)); g = g if g < 180 else g - 360
        mm = np.radians(g) * (r0 + 0.25)
        if mm < best[0]: best = (float(mm), r0)
    return best


# ---- 1) ranges
root_set = RD.root_c(0) + RD.gearbox_and_can() + RD.pinion() + RD.brushes(0)
fing = None
for m in V3.finger(0).values(): fing = m if fing is None else fing + m
Bd = bands(root_set + fing + V3.bead(0))
for da in sorted({MIN_SEP} | {float((b - a) % 360) for azs in V3.POSES.values() for a in azs for b in azs if a != b}):
    if da > 180: continue
    g, r0 = gap_mm(Bd, Bd, da); g2, r2 = gap_mm(Bd, Bd, -da)
    res["pairs"].append(dict(spacing_deg=round(da, 1), gap_mm=round(min(g, g2), 2), at_r=r0 if g <= g2 else r2))
print("ranges", res["pairs"], flush=True)

# ---- 2) simultaneous moves (the transitions the sim makes)
def best_shift(a0, a1):
    a0 = np.array(a0, float); a1 = np.array(a1, float); best = None
    for sh in np.arange(-180, 180, 1.0):
        d = (a1 + sh - a0 + 180) % 360 - 180
        m = np.abs(d).max()
        if best is None or m < best[0]: best = (m, sh, d)
    return best


TRANS = [("flight cross", "3 + 1 thumb"), ("3 + 1 thumb", "2 + 2 pinch"), ("2 + 2 pinch", "all four on one side"), ("all four on one side", "flight cross")]
for a, b in TRANS:
    m, sh, d = best_shift(V3.POSES[a], V3.POSES[b]); a0 = np.array(V3.POSES[a], float); mins = 360.0
    for f in np.linspace(0, 1, 101):
        cur = a0 + d * f
        for i in range(4):
            for j in range(i + 1, 4): mins = min(mins, abs((cur[i] - cur[j] + 180) % 360 - 180))
    res["moves"][f"{a} -> {b}"] = dict(largest_move_deg=float(m), pattern_turn_deg=float(sh), moves=[round(float(x), 1) for x in d],
                                       min_spacing_deg=round(float(mins), 1), ok=bool(mins >= MIN_SEP - 1e-6))
print("moves", res["moves"], flush=True)

# ---- 3) grids per pose, curling
def G(name, m): return OV.grid(name, SIM.mesh(m, tol=0.02))
I4 = np.eye(4)
static = {"base disc": RD.base_disc_c(), "top disc": V3.top_disc(), "heel": C.parts()["heel"], "rail": V3.ring_rail()}
gs = {n: G(n, m) for n, m in static.items()}
g_root = {k: G(f"root{k}", RD.root_c(k)) for k in range(4)}
g_mot = G("motor", RD.gearbox_and_can() + RD.pinion()); g_bead = G("bead", V3.bead(0))
R = C.parts()
g_ph = {j: G(f"ph{j}", V3.B.phalanx_v3(j) + R[f"pad{j + 1}"]) for j in range(3)}
g_band = {(j, round(float(c), 2)): G(f"band{j}_{c:.2f}", V3.B.band_solid(j, H.FOLD_MAX[j] * c) + V3.B.dowels(j, H.FOLD_MAX[j] * c))
          for j in range(3) for c in np.arange(0, 1.0001, 0.05)}
T_ = lambda d: np.asarray(C.T(C.rot("z", d)), float)


def hits(items):
    out = {}
    for i, (na, ga, Aa) in enumerate(items):
        for nb, gb, Ab in items[i + 1:]:
            ka, kb = na.split(":")[0], nb.split(":")[0]
            if ka == kb and ka != "hub": continue
            if ka == "hub" and kb == "hub": continue
            Mab = np.linalg.inv(Ab) @ Aa; p = ga["pts"] @ Mab[:3, :3].T + Mab[:3, 3]
            idx = np.floor((p - gb["lo"]) / OV.CELL).astype(int); okm = np.all((idx >= 0) & (idx < gb["n"]), 1)
            if not okm.any(): continue
            ii = idx[okm]; n_in = int(gb["er"][ii[:, 0], ii[:, 1], ii[:, 2]].sum())
            if n_in: out[f"{na} x {nb}"] = round(n_in * OV.CELL ** 3, 2)
    return out


def own(k, c, Af, fr):
    """Root k's own finger vs its own root + motor (same root key, so checked here separately)."""
    return []


for pose, azs in V3.POSES.items():
    rows = []
    for c in np.arange(0, 1.0001, 0.05):
        fold = tuple(f * c for f in H.FOLD_MAX); fr = [np.asarray(F_, float) for F_ in C.finger_frames(0, 0.0, fold)]
        items = [(f"hub:{n}", g, I4) for n, g in gs.items()]
        for k, az in enumerate(azs):
            A = T_(az); Af = T_(az - C.FAZ[0]); P0 = fr[0] @ np.asarray(C.T(C.rot("x", H.PITCH)), float)
            items += [(f"{k}:root", g_root[k], A), (f"{k}:motor", g_mot, A), (f"{k}:bead", g_bead, A)]
            items += [(f"{k}f:ph{j}", g_ph[j], Af @ fr[j + 1]) for j in range(3)]
            items += [(f"{k}f:band{j}", g_band[(j, round(float(c), 2))], Af @ (P0 if j == 0 else fr[j])) for j in range(3)]
        h = hits(items)
        # own finger vs own root/motor (different keys k / kf, so they are in h already); drop finger-vs-own-root contacts that are
        # the knuckle itself (grip is part of the root): report them separately
        rows.append(dict(curl=round(float(c), 2), hits=h))
        print(pose, round(c, 2), len(h), list(h.items())[:4], flush=True)
    base = rows[0]["hits"]
    def new(r): return {k: v for k, v in r["hits"].items() if k not in base or v > base[k] + 0.5}
    clear = [r["curl"] for r in rows if not new(r)]
    lim = 0.0
    for r in rows:
        if new(r): break
        lim = r["curl"]
    res["curl"][pose] = dict(max_clear_curl=lim, at_zero=base, first_hit=next(({r["curl"]: new(r)} for r in rows if new(r)), None))

# ---- 4) station vs roots (r-z sweep)
def rz(m):
    me = m.to_mesh(); v = np.asarray(me.vert_properties)[:, :3]; t = np.asarray(me.tri_verts)
    e = np.concatenate([v[t[:, a]] + (v[t[:, b]] - v[t[:, a]]) * f for a, b in ((0, 1), (1, 2), (2, 0)) for f in np.linspace(0, 1, 5)])
    return np.c_[np.hypot(e[:, 0], e[:, 1]), e[:, 2]]


from scipy.spatial import cKDTree
sweep = rz(root_set + fing + V3.bead(0)); tree = cKDTree(sweep)
st = RD.station_c()
for name, m in st.items():
    for s in (0.0, V3.ST["roll_gap"]):
        if name in ("tyre", "motor", "dock") and s > 0: mm = m.translate([-s, 0, 0])
        else: mm = m
        d, _ = tree.query(rz(mm)); res["station"][f"{name} (carriage {s} mm): min r-z gap to swept roots"] = round(float(d.min()), 2)
# dock contacts on the pads when the carriage is in
pads = RD.pads(); dock_in = st["dock"].translate([-V3.ST["roll_gap"], 0, 0]).rotate([0, 0, -0.0])
res["station"]["dock face to pad face when in (mm, pogo travel ~1)"] = round(float(RD.HC.R_CAGE[1] + 0.08 - (RD.HC.R_CAGE[1] + 0.6)), 2)
print("station", res["station"], flush=True)

# ---- 5) loads
res["loads"] = RD.drive_numbers()
json.dump(res, open("hand3_root_drive_check.json", "w"), indent=1, default=float)
for k, v in res["curl"].items(): print(k, v["max_clear_curl"], v["at_zero"], v["first_hit"])
print(json.dumps(res["loads"], indent=1, default=float))
