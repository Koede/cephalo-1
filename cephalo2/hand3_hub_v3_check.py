"""Checks for hand3 hub v3 (2026-11-19).  1) Range math: every rotating part of a root cut into 0.5 mm radial bands, each band an
(angle range, z range); two roots collide only where r bands, z ranges AND angle ranges all overlap.  2) Fits and travel by
interval arithmetic.  3) Whole assembly per pose and finger curl on occupancy grids (0.5 mm cells, eroded one cell: touching
faces do not count), as in coupler_sim_overlap.  Writes hand3_hub_v3_check.json"""
import os, sys, json
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hubc as HC, hand3_hub_v3 as V3
os.chdir(HERE)
import coupler_hand_sim as SIM, coupler_sim_overlap as OV
V = V3.V; res = dict(bands={}, pairs=[], fits={}, curl={})


def bands(m, dr=0.5):
    """r band -> (theta0, theta1, z0, z1) of a part built at az 0 (theta in deg).  Vertices of a fine mesh plus edge samples."""
    me = m.to_mesh(); v = np.asarray(me.vert_properties)[:, :3]; t = np.asarray(me.tri_verts)
    e = np.concatenate([v[t[:, a]] + (v[t[:, b]] - v[t[:, a]]) * f for a, b in ((0, 1), (1, 2), (2, 0)) for f in np.linspace(0, 1, 9)])
    r = np.hypot(e[:, 0], e[:, 1]); th = np.degrees(np.arctan2(e[:, 1], e[:, 0])); out = {}
    for r0 in np.arange(np.floor(r.min() / dr) * dr, r.max() + dr, dr):
        s = (r >= r0) & (r < r0 + dr)
        if s.any(): out[round(float(r0), 2)] = (float(th[s].min()), float(th[s].max()), float(e[s, 2].min()), float(e[s, 2].max()))
    return out


def gap_deg(A, B, da):
    """Smallest angular gap between part A at 0 and part B turned by da, over bands where r and z overlap (negative = overlap)."""
    best = (999.0, None)
    for r0, (a0, a1, z0, z1) in A.items():
        if r0 not in B: continue
        b0, b1, w0, w1 = B[r0]
        if z1 <= w0 or w1 <= z0: continue
        b0, b1 = b0 + da, b1 + da
        g = min(((b0 - a1) % 360), ((a0 - b1) % 360))
        g = g if g < 180 else g - 360
        mm = np.radians(g) * (r0 + 0.25)
        if mm < best[0]: best = (float(mm), r0)
    return best


R = C.parts()
blk = V3.root_block(0); pins = V3.latch_pin(0, 1) + V3.latch_pin(0, -1)
straight = V3.finger(0)
fing = None
for m in straight.values(): fing = m if fing is None else fing + m
B = {"root block + grip": bands(blk), "latch pins": bands(pins), "finger (straight)": bands(fing), "bead": bands(V3.bead(0))}
res["bands"] = {k: dict(r=(min(b), max(b)), half_angle_at_r20=b.get(20.0)) for k, b in B.items()}
for pose, azs in V3.POSES.items():
    for i in range(4):
        for j in range(i + 1, 4):
            da = (azs[j] - azs[i]) % 360
            for na in B:
                for nb in B:
                    if nb < na: continue
                    g, r0 = gap_deg(B[na], B[nb], da); g2, r2 = gap_deg(B[nb], B[na], (-da) % 360)
                    g, r0 = min((g, r0), (g2, r2))
                    if g < 3.0: res["pairs"].append(dict(pose=pose, roots=(i, j), a=na, b=nb, gap_mm=round(g, 2), at_r=r0, ok=bool(g > 0)))
# --- fits and travel (interval arithmetic) ---
zt = V["z0"] + V["disc_t"]; Zl = V3.Z_LEG0; za0, za1 = V3.Z_ARM; rail = V["rail"]; bd = V["bead"]; zr = V3.Z_RING_TOP
lip_in = HC.R_LIP_OUT[0] + 0.6                                                     # yoke lip inner edge moved out (decision B)
F = {
 "pin leg in the lip slot (each side)": (V["slot_w"] - V["wire_d"]) / 2,
 "foot catch under the lip when spinning (radial)": (V3.PIN_X - 0.25 + V["float_"] + 1.0 + V["foot"]) - max(V3.PIN_X - 0.25 + 1.0 + V["foot"] + 0.15, V3.PIN_X - 0.25 + V["float_"] + 1.05),
 "foot passes the lip slot at the inner position (radial)": max(V3.PIN_X - 0.25 + 1.0 + V["foot"] + 0.15, V3.PIN_X - 0.25 + V["float_"] + 1.05) - (V3.PIN_X - 0.25 + 1.0 + V["foot"]),
 "lifted leg bottom above the disc top (axial)": Zl + V["lift"] - zt,
 "block left under the arm slot (axial)": za0 - V3.Z_GAP[0],
 "lifted arm still inside its slot, below the block top": V3.Z_GAP[1] - (za1 + V["lift"]),
 "lifted tab below the top disc top (z 101)": 101.0 - (za1 + V["lift"]),
 "pin wire to the grip side (lateral)": V3.PIN_LAT - V["wire_d"] / 2 - 6.0,
 "block wall outside the pin slot (tangential)": np.radians(V["block_half"] - V["pin_daz"]) * V["pin_r"] - V["slot_w"] / 2,
 "web between lip holes (closest pair 14 deg)": np.radians(14.0) * V["pin_r"] - V["slot_w"] - 0.3,
 "bead bottom above the ring hangers (z 87)": zr + bd["dz"][0] - HC.Z_OUT[1],
 "bead top below the base disc (z 91)": V["z0"] - (zr + bd["dz"][1]),
 "bead inner edge outside the cage (r 21.5)": bd["r"][0] - HC.R_CAGE[1],
 "bead outer edge inside the yoke lip (moved to r 27.7)": lip_in - bd["r"][1],
 "cord to the disc rim (r 25.5)": bd["eye_r"] - V["cord_r"] - V["disc_r"],
 "cord to the yoke lip": lip_in - bd["eye_r"] - V["cord_r"],
 "yoke lip still over the ring flange (r 30)": HC.R_FL_OUT - lip_in,
 "bead inner edge to the inner pull ring band (r 23.9; ring removed in B)": bd["r"][0] - HC.R_IN[1],
}
res["fits"] = {k: round(float(v), 3) for k, v in F.items()}
# --- stroke with one cord over K1..K3 (decision B) ---
k1 = HC.LEVER[0] * np.radians(H.FOLD_MAX[0]); k23 = HC.LEVER[1] * np.radians(H.FOLD_MAX[1]) + HC.LEVER[2] * np.radians(H.FOLD_MAX[2])
res["stroke"] = dict(K1=round(k1, 2), K2_K3=round(k23, 2), stretch=HC.STRETCH, one_cord=round(k1 + k23 + HC.STRETCH, 2),
                     today_base_ring=round(HC.TRAVEL_B, 2), today_curl_ring=round(HC.TRAVEL_C, 2))
# --- grids: whole hub per pose, fingers curled in steps ---
def G(name, m): return OV.grid(name, SIM.mesh(m, tol=0.02))
I4 = np.eye(4)
static = {"base disc": V3.base_disc(), "top disc": V3.top_disc(), "heel": R["heel"]}
gs = {n: G(n, m) for n, m in static.items()}
g_blk = G("block", blk); g_pin = G("pin", pins); g_bead = G("bead", V3.bead(0)); g_rail = G("rail", V3.ring_rail())
g_ph = {j: G(f"ph{j}", V3.B.phalanx_v3(j) + R[f"pad{j + 1}"]) for j in range(3)}
def Rz(d): return np.asarray(T_(d))
def T_(d): return C.T(C.rot("z", d))
def hits(items):
    """items: list of (name, grid, 4x4).  Returns {(a, b): mm3} for pairs of different roots or a root vs the hub."""
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
g_band = {(j, round(float(c), 2)): G(f"band{j}_{c:.2f}", V3.B.band_solid(j, H.FOLD_MAX[j] * c) + V3.B.dowels(j, H.FOLD_MAX[j] * c))
          for j in range(3) for c in np.arange(0, 1.0001, 0.05)}
for pose, azs in V3.POSES.items():
    rows = []
    for c in np.arange(0, 1.0001, 0.05):
        fold = tuple(f * c for f in H.FOLD_MAX); fr = [np.asarray(F_, float) for F_ in C.finger_frames(0, 0.0, fold)]
        items = [(f"hub:{n}", g, I4) for n, g in gs.items()] + [("hub:rail", g_rail, I4)]
        for k, az in enumerate(azs):
            A = np.asarray(T_(az), float)
            items += [(f"{k}:block", g_blk, A), (f"{k}:pins", g_pin, A), (f"{k}:bead", g_bead, A)]
            Af = np.asarray(T_(az - C.FAZ[0]), float)
            items += [(f"{k}:ph{j}", g_ph[j], Af @ fr[j + 1]) for j in range(3)]
            items += [(f"{k}:band{j}", g_band[(j, round(float(c), 2))], Af @ (fr[0] @ np.asarray(C.T(C.rot("x", H.PITCH)), float) if j == 0 else fr[j])) for j in range(3)]
        h = hits(items); rows.append(dict(curl=round(float(c), 2), hits=h))
        print(pose, round(c, 2), len(h), list(h.items())[:4], flush=True)
    noboxes = [r for r in rows if not any("box" in k for k in r["hits"])]
    free = [r["curl"] for r in rows if not r["hits"]]; free_nb = [r["curl"] for r in rows if not any("box" not in k for k in r["hits"])]
    res["curl"][pose] = dict(max_clear_curl=(max(free) if free and free[0] == 0 else None), 
                             at_zero=rows[0]["hits"], first_hit={r["curl"]: r["hits"] for r in rows if r["hits"]} and next(iter([{r["curl"]: r["hits"]} for r in rows if r["hits"]]), None))
json.dump(res, open("hand3_hub_v3_check.json", "w"), indent=1, default=float)
print(json.dumps(res["fits"], indent=1)); print(res["stroke"])
for p in res["pairs"]: print(p) if not p["ok"] else None
for k, v in res["curl"].items(): print(k, v["max_clear_curl"], v["at_zero"])
