"""hand3 hub v3: where do the knuckle return springs go?  Options study, 2026-11-20.

Problem: hub E's spring boxes sat on the lag tube; their extensor cords ran down the tube's bore.  In v3 the lag tube is gone and the
top disc (r <= 25.5, z 99..101) covers the old column at r 20, so the cords can't reach the boxes.  The boxes' post on the root block
(r 26.2..30, z 98..102.7) also sits right where the single flexor cord has to rise from the block to K1's sector (z 97 -> 102).

Requirements (hand3_springs targets, est. where marked):
  return torque at fold 0: K1 0.020, K2 0.004, K3 0.016..0.020 N m (enough to open an empty finger on the ground, no spin);
  at full fold <= 40 % of what the flexor can give (31.9 N rated cord x lever 6 / 4 / 3 mm): 0.077 / 0.051 / 0.038 N m;
  works at 12,500 rpm (1309 rad/s) with the finger straight; stock parts; must not cross the flexor path; printable.
Side views below: y = 0 slices of the v3 CAD (root block + phalanges), finger frame pitched 16 deg, to scale."""
import os, sys, json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
import hand3_hub_v3 as V3, hand3_cad as C, hand3_design as H, hand3_springs as SP

HERE = os.path.dirname(os.path.abspath(__file__))
W = 1309.0                                                   # rad/s at 12,500 rpm
LIMIT = [0.40 * 31.9 * H.LEVER[j] * 1e-3 for j in range(3)]
TGT = [0.020, 0.004, 0.018]


def sl(m):
    """y = 0 section of a manifold -> list of (x, z) polygons."""
    return [np.c_[p[:, 0], -p[:, 1]] for p in (np.asarray(q) for q in m.rotate([90, 0, 0]).slice(0.0).to_polygons())]


def oring(ID, cs, mat, tau0, h0, h1, fold):
    """Dorsal O-ring band across a knuckle (option D): hooks h0 below the pin axis, band rides over the barrel (arm h1) when folded.
    Returns strain and torque at fold 0 and full fold.  Two strands, E from hand3_springs (silicone 2, NBR 5.5 MPa)."""
    EA2 = 2 * SP.E_MOD[mat] * np.pi * cs * cs / 4; Lf = np.pi * (ID + cs) / 2
    F0 = tau0 / (h0 * 1e-3); e0 = F0 / EA2; e1 = e0 + h1 * np.radians(fold) / Lf
    span0 = Lf * (1 + e0) - np.pi * (0.6 + cs / 2)                 # hook-to-hook (hook pin r 0.6)
    return dict(ID=ID, cs=cs, mat=mat, e0=round(e0, 2), e1=round(e1, 2), span0=round(span0, 1), tau0=tau0, tau1=round(e1 * EA2 * h1 * 1e-3, 4),
                mass_g=round(np.pi ** 2 * (ID + cs) * cs * cs / 4 * 1.3e-3, 3))


def numbers():
    out = {}
    # current boxes (for mass), from the CAD
    R = C.parts(); box_g = sum(R[n].volume() for n in ("spring_tray1_R", "spring_tray2_R", "spring_lid_R")) * 1.25e-3
    out["box_g_per_finger"] = round(box_g, 2)
    out["box_pull_N_at_r22"] = round(box_g * 1e-3 * 0.022 * W ** 2, 1)
    out["hubE_rings"] = {k: dict(ID=SP.ring_pick(j)["ID"], cs=SP.ring_pick(j)["cs"], mat=SP.ring_pick(j)["mat"],
                                 tau0=round(SP.ring_pick(j)["tau0"], 3), tau_max=round(SP.ring_pick(j)["tau_max"], 4))
                         for j, k in enumerate(("K1", "K2", "K3"))}
    # option D: dorsal O-rings, hook height h0 = barrel radius + 0.5, band arm over the barrel h1 = barrel r + cs/2
    D = {}
    for j in range(3):
        rb = H.BARREL[j] / 2; lo, hi = SP.WINDOW[f"K{j + 1}"]; best = None
        for tau in np.arange(lo, hi + 1e-9, 0.001):
            for ID, cs, mat in SP.STOCK:
                r = oring(ID, cs, mat, round(tau, 3), rb + 0.5, rb + cs / 2, H.FOLD_MAX[j])
                if r["e1"] > 1.0 or r["tau1"] > LIMIT[j] or r["e0"] < 0.1 or not 9.0 <= r["span0"] <= 20.0: continue
                sc = abs(tau - TGT[j]) * 1e3 + r["tau1"] / LIMIT[j] + 0.3 * (mat == "NBR")
                if best is None or sc < best[0]: best = (sc, r)
        D[f"K{j + 1}"] = best[1]
        D[f"K{j + 1}"]["limit"] = round(LIMIT[j], 3); D[f"K{j + 1}"]["ok"] = bool(D[f"K{j + 1}"]["tau1"] <= LIMIT[j] and D[f"K{j + 1}"]["e1"] <= 1.0)
        D[f"K{j + 1}"]["pull_N_at_r"] = round(D[f"K{j + 1}"]["mass_g"] * 1e-3 * (H.KN[j] + 4) * 1e-3 * W ** 2, 1)
    out["D"] = D
    # option C: McMaster music-wire torsion springs, 90 deg series (catalogue: OD / shaft / wire in inch, max torque in-lbf)
    mc = dict(K1=(0.204, 0.13, 0.023, 0.28), K2=(0.204, 0.13, 0.023, 0.28), K3=(0.124, 0.08, 0.014, 0.072))
    Cn = {}
    for j, (k, (od, sh, wd, tq)) in enumerate(mc.items()):
        tmax = tq * 0.113; rate = tmax / 90.0; pre = TGT[j] / rate
        Cn[k] = dict(OD_mm=round(od * 25.4, 2), shaft_mm=round(sh * 25.4, 2), max_Nm=round(tmax, 4), preload_deg=round(pre, 0),
                     total_deg=round(pre + H.FOLD_MAX[j], 0), ok=bool(pre + H.FOLD_MAX[j] <= 90.0),
                     room_mm=round(H.LUG[j], 1), pin_mm=H.PIN_D[j])
    out["C"] = Cn
    # option B: lanes in the block between the discs (6 mm tall, one level), width between the latch-pin leg bores
    lane_w = [2 * (SP.BOX["pin_r"] + SP.ring_pick(j)["cs"] + 0.35) for j in range(3)]
    free_y = 2 * (V3.PIN_LAT - V3.V["slot_w"] / 2 - 0.8)
    out["B"] = dict(lane_w=[round(w, 1) for w in lane_w], need=round(sum(lane_w) + 2 * 0.8, 1), free=round(free_y, 1),
                    levels=int((V3.Z_GAP[1] - V3.Z_GAP[0]) // (SP.BOX["floor"] + SP.BOX["room"])),
                    radial_room=round(SP.X_SLOT - V3.V["block_r"][0], 1), lane_len_needed=max(L["d_end"] for L in SP.LANE.values()))
    # option F: one extensor cord over all saddles: torque ratio = saddle radii
    rs = [SP.SAD[j]["r_s"] for j in range(3)]
    out["F"] = dict(saddle_r=rs, ratio=[round(r / rs[0], 2) for r in rs], wanted=[round(t / TGT[0], 2) for t in TGT])
    return out


OPTS = [("A", "Keep the boxes, move\nthem out on the post", "#888"), ("B", "O-ring lanes inside\nthe root block", "#888"),
        ("C", "Stock torsion springs\non the knuckle pins", "#888"), ("D", "One O-ring band across\neach knuckle's back", "#2a7"),
        ("E", "Printed TPU strap\nalong the finger back", "#888"), ("F", "One extensor cord over\nall 3 saddles, 1 spring", "#c33"),
        ("G", "No return springs\n(spin opens the hand)", "#c33"), ("now", "Hub E boxes as placed\n(cords blocked)", "#c33")]


def figure(N):
    blk = V3.root_block(0); fing = V3.finger(0, (0, 0, 0))
    secs = sl(blk) + sum((sl(m) for n, m in fing.items() if n.startswith("phalanx")), [])
    pads = sum((sl(m) for n, m in fing.items() if n.startswith("pad")), [])
    disc = sl(V3.top_disc()) + sl(V3.base_disc())
    fig = plt.figure(figsize=(22, 10.5))
    for i, (key, title, col) in enumerate(OPTS):
        ax = fig.add_axes([0.02 + (i % 4) * 0.245, 0.69 - (i // 4) * 0.25, 0.235, 0.2])
        for p in disc: ax.add_patch(Polygon(p, fc="#ddd", ec="#999", lw=0.5))
        for p in secs: ax.add_patch(Polygon(p, fc="#c9d6e8", ec="#4a6a9a", lw=0.5))
        for p in pads: ax.add_patch(Polygon(p, fc="#f2c46d", ec="none"))
        zb = H.Z["blade"]
        # flexor (single cord): block hole at r 26.3 -> over K1 sector (lever 6) -> K2 -> K3
        th = np.radians(H.PITCH); zf = lambda h: zb + h * np.cos(th)
        fx = [26.3, 30.0, H.KN[0], H.KN[1] - 4, H.KN[1], H.KN[2] - 3, H.KN[2]]
        fz = [97.0, zf(6.0), zf(6.0), zf(2.2), zf(4.0), zf(1.6), zf(3.0)]
        ax.plot(fx, fz, color="#d33", lw=1.3, ls="-" if key != "A" else "--")
        ax.plot([26.3, 26.3], [86.0, 97.0], color="#d33", lw=1.3)
        if key in ("now", "A"):
            P = V3.V["post"]; ax.add_patch(Rectangle((P[0], zb + 2.0), P[1] - P[0], P[3] - zb - 2.0, fc="#b79", ec="k", lw=0.5))
            R = C.parts()
            for n in ("spring_tray1_R", "spring_tray2_R", "spring_lid_R"):
                for p in sl(R[n]): ax.add_patch(Polygon(p + ([8.0, 0] if key == "A" else [0, 0]), fc="#e9b", ec="k", lw=0.4))
            if key == "now":
                ax.plot([20, 20], [96, 101.5], color="#c33", lw=2); ax.text(20.5, 100.5, "blocked by\ntop disc", fontsize=7, color="#c33")
            else:
                ax.plot([28.5, 28.5], [96, 103], color="#a3a", lw=1.2); ax.text(36, 108, "column r 28.5 crosses\nflexor rise (dashed)", fontsize=7)
        if key == "B":
            for k, x0 in enumerate((10, 10, 10)):
                ax.add_patch(Rectangle((V3.V["block_r"][0] + 0.5, 93.8 + k * 1.6), 13.5, 1.3, fc="#e9b", ec="k", lw=0.4, alpha=0.7))
            ax.text(10, 87, f"need {N['B']['need']} mm wide, {N['B']['free']} free;\n1 level of 6 mm", fontsize=7)
        if key == "C":
            for j in range(3):
                od = N["C"][f"K{j + 1}"]["OD_mm"]; ax.add_patch(Circle((H.KN[j], zb), od / 2, fc="none", ec="#a3a", lw=1.5))
            ax.text(40, 86, "90 deg springs: preload + fold\n> 90 deg rating (K1, K3)", fontsize=7)
        if key == "D":
            for j in range(3):
                rb = H.BARREL[j] / 2 + 0.5; hs = N["D"][f"K{j + 1}"]["span0"] / 2; a = H.KN[j] - hs; b = H.KN[j] + hs
                if j == 0: a = 27.5; b = a + 2 * hs                  # clear of the tackle strands (r 26.3) and the disc rim (25.5)
                ax.plot([a, b], [zb - rb, zb - rb], color="#2a7", lw=2.5); ax.plot([a, b], [zb - rb - 0.3, zb - rb - 0.3], color="#2a7", lw=1)
                for x in (a, b): ax.add_patch(Rectangle((x - 0.6, zb - rb - 0.9), 1.2, 1.4, fc="#2a7", ec="k", lw=0.3))
            ax.text(40, 86, "hooks on the back (-z); no cords,\nsaddles, boxes or post", fontsize=7)
        if key == "E":
            ax.plot([30, 112], [zb - 4.6, zb - 3.4], color="#a3a", lw=3); ax.text(40, 86, "TPU 95A strap, screwed to each\nphalanx; stiffness/creep est.", fontsize=7)
        if key == "F":
            ax.plot([22, 112], [zb - 2.5, zb - 2.5], color="#c33", lw=1.2)
            ax.text(40, 86, f"torque ratio = saddle radii {N['F']['ratio']}\nwanted {N['F']['wanted']}", fontsize=7, color="#c33")
        if key == "G":
            ax.text(40, 86, "on the ground nothing opens\nthe fingers (cords only pull)", fontsize=7, color="#c33")
        ax.set_xlim(6, 116); ax.set_ylim(84, 113); ax.set_aspect("equal"); ax.tick_params(labelsize=6)
        ax.set_title(f"{key}: {title}", fontsize=10, color=col if col != "#888" else "k", fontweight="bold" if col == "#2a7" else "normal")
    D = N["D"]; Cc = N["C"]
    rows = [["", "A boxes outboard", "B lanes in block", "C torsion springs", "D dorsal O-rings", "E TPU strap", "F one cord", "G none"],
            ["return torque 0 / full fold\nK1, K2, K3 (N m)", "hub E values\n.020/.056 .004/.021 .016/.034", "same as A", "0 preload possible\n(90 deg rating)",
             " ".join(f"{D[k]['tau0']:.3f}/{D[k]['tau1']:.3f}" for k in ("K1", "K2", "K3")), "est. only", "K2 gets 1.4x K1", "0"],
            ["limit at full fold (40 % flexor)", "", "", "", " ".join(f"{D[k]['limit']:.3f}" for k in ("K1", "K2", "K3")), "", "", ""],
            ["parts to buy", "12 O-rings (as now)", "12 O-rings", "12 McMaster 9271K\ntorsion springs", " / ".join(f"{D[k]['ID']:g}x{D[k]['cs']:g} {D[k]['mat'][:3]}" for k in D) + "\n12 stock O-rings", "none", "4 O-rings", "none"],
            ["removed", "nothing", "boxes, lids, post", "boxes, post, saddles,\nextensor cords", "boxes, lids, post,\nsaddles, ext. cords", "boxes, post, saddles", "boxes", "all springs"],
            ["finger change", "none", "none", "pockets in every lug", "hooks on the back", "screw bosses", "none", "none"],
            ["mass per finger", f"+{N['box_g_per_finger']} g box", "~0", "+0.3 g springs (est.)", f"+{sum(D[k]['mass_g'] for k in D):.2f} g rings",
             "+0.4 g (est.)", "-", "-"],
            ["spin load", f"box {N['box_pull_N_at_r22']} N on post", "in block", "on pins", " ".join(f"{D[k]['pull_N_at_r']}" for k in D) + " N\non hooks",
             "strap pulls on screws", "", ""],
            ["main risk", "column crosses flexor;\nbox 1.6 mm from next", f"{N['B']['need']} mm of lanes in\n{N['B']['free']} mm: doesn't fit",
             "preload + fold exceeds\n90 deg rating; K3: no room", "exposed bands: snag, wear;\nsilicone tears if nicked", "creep and fatigue\nunknown", "ratio wrong: KILL", "hand can't open: KILL"]]
    ax = fig.add_axes([0.02, 0.01, 0.96, 0.40]); ax.axis("off")
    t = ax.table(cellText=rows, loc="center", cellLoc="center"); t.auto_set_font_size(False); t.set_fontsize(8); t.scale(1, 2.6)
    for r in range(len(rows)):
        t[r, 4].set_facecolor("#dff3e6")
        for c in (6, 7): t[r, c].set_facecolor("#f6dada")
    fig.suptitle("hand3 hub v3 - knuckle return springs: hub E's boxes can't be reached (top disc) and their post blocks the flexor.  "
                 "Side views at y = 0, to scale (mm).  Red: single flexor cord.  Recommended: D", fontsize=13)
    fig.savefig(os.path.join(HERE, "hand3_return_spring_study.png"), dpi=110)


if __name__ == "__main__":
    N = numbers()
    json.dump(N, open(os.path.join(HERE, "hand3_return_spring_study.json"), "w"), indent=1)
    print(json.dumps(N, indent=1)[:3000])
    figure(N)
