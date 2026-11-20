"""hand3 hub v3 C: give the hand room to grab - options study, 2026-11-20.

Measure: balls of diameter D (10..100 mm) set on the palm on the hub axis (they rest on whatever is highest under them: heel,
motor towers, top disc); the fingers (flight-cross pose) curl until they touch the ball.  A ball counts as held when the touch
comes at or above its centre and before the empty-hand curl limit.  To scale, real CAD (hand3_root_drive, hand3_hub_v3).
Writes hand3_grasp_room_study.json/.png."""
import os, sys, json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Circle, Rectangle
from scipy.spatial import cKDTree
import hand3_root_drive as RD, hand3_hub_v3 as V3, hand3_design as H, hand3_cad as C
from hand3_finger import cyl

HERE = os.path.dirname(os.path.abspath(__file__))
POSE = V3.POSES["flight cross"]
W = 1309.0


def pts(m, n=6):
    me = m.to_mesh(); v = np.asarray(me.vert_properties)[:, :3]; t = np.asarray(me.tri_verts)
    return np.concatenate([v[t[:, a]] + (v[t[:, b]] - v[t[:, a]]) * f for a, b in ((0, 1), (1, 2), (2, 0)) for f in np.linspace(0, 1, n)])


# E: Pololu 2359 sub-micro gearmotor 6 x 21 standing on the root at r 30, y -5 (clear of the flexor and the anchor knot)
MICRO = dict(r=3.0, L=21.0, x=30.0, y=-5.0, z0=101.6, m=1.3)


def micro():
    return cyl(MICRO["r"], MICRO["z0"], MICRO["z0"] + MICRO["L"], c=(MICRO["x"], MICRO["y"], 0), n=24)


TOP = pts(RD.top_disc_c()); HEEL = pts(C.parts()["heel"], 5)
ROOTS = np.concatenate([pts(RD.root_c(k, a)) for k, a in enumerate(POSE)])
TOW = np.concatenate([pts(RD.gearbox_and_can().rotate([0, 0, a])) for a in POSE])
MIC = np.concatenate([pts(micro().rotate([0, 0, a])) for a in POSE])
FP = {round(float(c), 2): np.concatenate([pts(m, 4) for a in POSE for n, m in V3.finger(a, tuple(c * x for x in H.FOLD_MAX), bands=False).items()])
      for c in np.arange(0, 1.0001, 0.01)}

OPTS = {
    "A": dict(title="A: now (worm N20 towers to z 135)", lim=0.70, tow="N20", heel=None, root_top=None),
    "B": dict(title="B: A + low heel (thin nut, z 105)", lim=0.70, tow="N20", heel=105.0, root_top=None),
    "E": dict(title="E: 6 mm micro gearmotor per root\n(+ low heel, plain PA-CF root)", lim=0.75, tow="micro", heel=105.0, root_top=101.3),
    "G": dict(title="G: no towers in the palm (reference:\none station, or towers below)", lim=0.75, tow=None, heel=105.0, root_top=101.3),
}


def hub(o):
    h = HEEL if o["heel"] is None else HEEL[HEEL[:, 2] <= o["heel"]]
    r = ROOTS if o["root_top"] is None else ROOTS[ROOTS[:, 2] <= o["root_top"]]
    P = [TOP, h, r]
    if o["tow"] == "N20": P.append(TOW)
    if o["tow"] == "micro": P.append(MIC)
    return np.concatenate(P)


def grasp(o, D):
    R = D / 2; hp = hub(o); r = np.hypot(hp[:, 0], hp[:, 1]); s = r < R
    zc = float((hp[s, 2] + np.sqrt(R * R - r[s] ** 2)).max() + 0.2)
    for c in sorted(FP):
        if c > o["lim"] + 1e-9: return dict(D=D, ok=False, zc=round(zc, 1), why="fingers can't reach it")
        d = np.linalg.norm(FP[c] - [0, 0, zc], axis=1); k = d < R
        if k.any():
            zt = float(FP[c][k, 2].max())
            return dict(D=D, ok=bool(zt >= zc), c=c, touch_above_centre=round(zt - zc, 1), zc=round(zc, 1))
    return dict(D=D, ok=False, zc=round(zc, 1))


def numbers():
    out = {}
    for k, o in OPTS.items():
        rows = [grasp(o, D) for D in range(10, 121, 5)]; ok = [r["D"] for r in rows if r["ok"]]
        out[k] = dict(held_D_mm=(min(ok), max(ok)) if ok else None, rows=rows, palm_top=float(hub(o)[:, 2][np.hypot(*hub(o)[:, :2].T) < 34].max()))
    # E loads/mass (est.)
    m_set = MICRO["m"] + 0.3 + 0.5
    out["E_loads"] = dict(micro="Pololu 2359 700:1 sub-micro plastic planetary, 6 x 21 mm, 1.3 g, 6 V: ~90 rpm free, ~85 mN m stall",
                          set_g=m_set, pull_N=round(m_set * 1e-3 * MICRO["x"] * 1e-3 * W ** 2),
                          pinion="m0.5 z10 brass, 2 mm bore (est.) on a gear rim of the top disc (pitch r ~27, disc grown to r 27.5)",
                          root_speed_deg_s=round(np.degrees(70 / 60 * 2 * np.pi * 2.5 / 27.0), 1), push_N=round(85e-3 / 2.5e-3),
                          flight_hold="spin presses the root's ribs on the disc grooves (~1,200 N): friction ~300 N vs ~6 N needed (est. mu 0.25)",
                          ground_hold="docked while parked: motors powered and held; 700:1 plastic planetary resists back-driving (est.)",
                          mass_vs_A_g=round(-4 * (13.0 - m_set) - 4 * (14.93 - 3.5), 1))
    return out


def figure(N):
    fig = plt.figure(figsize=(21, 11))
    for i, (k, o) in enumerate(OPTS.items()):
        ax = fig.add_axes([0.02 + i * 0.245, 0.42, 0.235, 0.5]); ax.set_aspect("equal"); ax.set_xlim(-75, 75); ax.set_ylim(80, 200)
        hp = hub(o); sl = np.abs(hp[:, 1]) < 1.5
        ax.scatter(hp[sl, 0], hp[sl, 2], s=0.3, c="#556")
        c = o["lim"]; fp = FP[round(c, 2)]; sf = np.abs(fp[:, 1]) < 1.5
        ax.scatter(fp[sf, 0], fp[sf, 2], s=0.3, c="#c33")
        if o["tow"] == "N20":
            for a in (0, 180):
                m = RD.gearbox_and_can().rotate([0, 0, a]); b = np.asarray(m.bounding_box()); ax.add_patch(Rectangle((b[0], b[2]), b[3] - b[0], b[5] - b[2], fc="#8a8f98", ec="k", lw=0.5))
        if o["tow"] == "micro":
            for a in (0, 180):
                x = MICRO["x"] * np.cos(np.radians(a)); ax.add_patch(Rectangle((x - 3, MICRO["z0"]), 6, MICRO["L"], fc="#8a8f98", ec="k", lw=0.5))
        held = N[k]["held_D_mm"]
        if held:
            r = next(rr for rr in N[k]["rows"] if rr["D"] == held[1]); ax.add_patch(Circle((0, r["zc"]), held[1] / 2, fc="#2a7", alpha=0.25, ec="#2a7"))
        ax.set_title(f"{o['title']}\nheld balls: {'Ø%d-%d mm' % held if held else 'none'}", fontsize=10.5, color="#2a7" if k == "E" else "k",
                     fontweight="bold" if k == "E" else "normal")
        ax.tick_params(labelsize=7); ax.plot([40, 60], [195, 195], "k", lw=2); ax.text(50, 197, "20 mm", ha="center", fontsize=7)
    L = N["E_loads"]
    rows = [["", "A now", "B low heel", "C motors hang below", "D N20 lying flat", "E 6 mm micro gearmotor", "G no towers (ref.)"],
            ["balls held (cross)", "Ø%d-%d" % tuple(N["A"]["held_D_mm"]), "Ø%d-%d" % tuple(N["B"]["held_D_mm"]), "as G", "-",
             "Ø%d-%d" % tuple(N["E"]["held_D_mm"]), "Ø%d-%d" % tuple(N["G"]["held_D_mm"])],
            ["curl limit (empty hand)", "70 %", "70 %", "75 %", "-", "75 % (est.)", "75 %"],
            ["each root still moves on its own", "yes", "yes", "yes", "yes", "yes", "only with one station"],
            ["spin pull of the motor set", "625 N", "625 N", "~730 N", "~710 N", f"{L['pull_N']} N", "-"],
            ["root material", "AlSi10Mg", "AlSi10Mg", "AlSi10Mg", "AlSi10Mg", "PA-CF again", "PA-CF"],
            ["mass vs now", "0", "-1 g", "+~10 g (drive moved out)", "-", f"{L['mass_vs_A_g']:.0f} g", "-"],
            ["kill check", "-", "-", "KILL: the pull ring must key to the rotor\nthrough the towers' annulus", "KILL: 33 mm motor in a 26 mm\nchord at 58 deg spacing",
             "est.: holds by friction in flight,\npowered while docked", "fails 'all at once'"]]
    ax = fig.add_axes([0.02, 0.01, 0.96, 0.36]); ax.axis("off")
    t = ax.table(cellText=rows, loc="center", cellLoc="center"); t.auto_set_font_size(False); t.set_fontsize(8.5); t.scale(1, 2.5)
    for r in range(len(rows)):
        t[r, 5].set_facecolor("#dff3e6")
        for cc in (3, 4): t[r, cc].set_facecolor("#f6dada")
    fig.suptitle("hand3 hub v3 C - room to grab: balls set on the palm, fingers curl to touch (section y = 0, cross pose, to scale mm).  "
                 "Green ball = largest held.  Recommended: E", fontsize=12.5)
    fig.savefig(os.path.join(HERE, "hand3_grasp_room_study.png"), dpi=100)


if __name__ == "__main__":
    N = numbers(); json.dump(N, open(os.path.join(HERE, "hand3_grasp_room_study.json"), "w"), indent=1, default=float)
    for k in OPTS: print(k, N[k]["held_D_mm"], N[k]["palm_top"])
    print(N["E_loads"]); figure(N)
