"""Options study: how motor power gets from the coupler bullets through hand3 hub E to the SE 2816 motor.

Uses the free-space grid of coupler_lead_route.py (/tmp/route_occ.npz; rerun that script if it is missing) and the real
cord runs of hand3e/hand3_hubc.py.  The first route search left the moving cords out, so this study adds them.
Writes hubE_lead_options_study.png and hubE_lead_options_study.json.
"""
import sys, json, textwrap
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
sys.path.insert(0, "hand3e")
import hand3_hubc as C
import coupler_hand_sim as SIM

P = SIM.P; fz0 = P["z_flange"][0]
occ = np.load("/tmp/route_occ.npz")["occ"]; LO = np.array([-50.0, -50.0, -82.0])
kz = lambda zh: int(np.floor(fz0 - zh - LO[2]))            # hand3 height -> grid layer (the hand sits end for end)
SEC = occ[:, :, kz(30.5)].T                                  # plan section, world x right, y up
ELEV = np.array([[occ[50, j, kz(zh + 0.5)] for j in range(100)] for zh in range(-8, 72)])   # y-z section at x = 0

# --- loads --------------------------------------------------------------------------------------------------------
I_HOVER, I_MAX = 36.0, 54.4                                  # A per phase (est., from the motor-load monitor)
R = {12: 5.21, 14: 8.29, 16: 13.2, 18: 20.9, 20: 33.3}       # mOhm/m, copper at 20 C
OD = {14: 3.5, 16: 3.0, 18: 2.3, 20: 1.9}                    # silicone wire outer diameter, mm
R_STRIP = 1.724e-8 / (0.5e-3 * 6.35e-3) * 1e3                # K&S 0.5 x 6.35 mm copper strip, mOhm/m
L_RUN, L_STUB = 0.08, 0.02                                   # m: bullet -> top of the servo stack, and the motor's own 18 AWG stub
def loss(r_run, i=I_HOVER, stub=True):
    return 3 * i * i * 1e-3 * (r_run * L_RUN + (R[18] if stub else r_run) * L_STUB)

# --- moving cords (hand frame -> world: x flips) ------------------------------------------------------------------
cords = [(np.array(p) * [-1, 1, 1], np.array(q) * [-1, 1, 1])
         for pb, pc in ((0.0, 0.0), (C.TRAVEL_B, C.TRAVEL_C)) for p, q in C.cord_runs(pb, pc).values()]
def cord_gap(x, y, rad, z0=2.0, z1=51.0):
    """Smallest gap between a vertical conductor of radius rad at (x, y) and any cord."""
    s = np.linspace(0, 1, 300)[:, None]; Q = np.array([x, y, z0]) + np.array([0, 0, z1 - z0]) * s
    best = 1e9
    for p, q in cords:
        Pp = p + (q - p) * s
        best = min(best, np.sqrt(((Pp[:, None, :] - Q[None, :, :]) ** 2).sum(-1)).min())
    return best - rad - C.CORD_R
def part_gap(x0, x1, y0, y1, z0=2, z1=51):
    """Free distance (whole grid cells) around a footprint over the height of the servo stack."""
    for g in range(0, 12):
        i0, i1 = int(np.floor(x0 - g - LO[0])), int(np.ceil(x1 + g - LO[0])); j0, j1 = int(np.floor(y0 - g - LO[1])), int(np.ceil(y1 + g - LO[1]))
        blk = occ[i0:i1, j0:j1, :][:, :, [kz(z + 0.5) for z in range(z0, z1)]].copy()
        blk[:, (np.arange(j0, j1) + LO[1] + 0.5 > -18) & (np.arange(j0, j1) + LO[1] + 0.5 < 18), :] = False   # the servo face it lies on
        if blk.any(): return g
    return 12

YB = -18.0                                                   # back face of the servo stack (no spools, no cords)
def bundle(awg, n=3, y_face=YB, x0=0.0, side=-1):
    d = OD[awg]; return [(x0 + (i - (n - 1) / 2) * d, y_face + side * d / 2, d / 2) for i in range(n)]

opts = []
def add(key, title, conductor, changes, w, joints, mass, clr, field, note, draw, tint=False):
    opts.append(dict(key=key, title=title, conductor=conductor, changes=changes, loss_W=w, joints=joints, mass=mass,
                     clearance=clr, field=field, note=note, draw=draw, tint=tint))

bA, bC, bE = bundle(18), bundle(14), bundle(16)
gA = min(cord_gap(x, y, r) for x, y, r in bA); gC = min(cord_gap(x, y, r) for x, y, r in bC)
pC = part_gap(-5.25, 5.25, -21.5, -18.0); pA = part_gap(-3.5, 3.5, -20.3, -18.0)
dD = (0.0, 18.0 + OD[14] / 2, OD[14] / 2); gD = cord_gap(*dD)
gG = cord_gap(-1.0, 23.5, OD[18] / 2)

add("A", "Motor's own 18 AWG leads, uncut, down the back face", "SE 2816 leads as supplied (18 AWG, 25 cm), trimmed to length; no splice",
    "hand-base grooves exit at the back edge; lead groove in the top block", loss(R[18]), "3 (at the bullets)", "0 g",
    f"parts {pA}+ mm, cords {gA:.0f} mm", "bundled: ~0.05 mT", "runs hot: 27 W/m per lead at hover", ("wires", bA))
add("B", "Copper strip busbars taped to the back face", "3 x K&S 0.5 x 6.35 mm copper strip (3.2 mm2), Kapton-wrapped; motor stubs soldered on",
    "as A, plus a 22 mm wide flat on the top block", loss(R_STRIP), "9 (bullets, strips, stubs)", "+4 g (est.)",
    "parts 5+ mm, cords 20+ mm", "flat, side by side: ~0.1 mT", "lowest loss; most hand work (cut, bend, insulate)", ("strips", [(-7.2, YB - 0.7), (0, YB - 0.7), (7.2, YB - 0.7)]))
add("C", "14 AWG silicone down the back face, joined to short motor stubs", "same 14 AWG as the arm side and the B.6 bullet grooves; motor leads cut to 20 mm",
    "as A (grooves stay 14 AWG size)", loss(R[14]), "6 (bullets + 3 inline)", "+3 g (est.)",
    f"parts {pC}+ mm, cords {gC:.0f} mm", "bundled: ~0.06 mT", "joints sit on the back face under the cage", ("wires", bC), True)
add("D", "16 AWG down the back face (lighter C)", "16 AWG silicone, joined to 20 mm motor stubs", "as A", loss(R[16]), "6", "+1.5 g (est.)",
    f"parts {pC}+ mm, cords {gC:.0f} mm", "bundled: ~0.06 mT", "middle ground", ("wires", bE))
add("E", "Split: one lead up the front face, two up the back", "14 AWG; front lead between the servo face and the cords",
    "as A on both faces; shortest hand-base runs", loss(R[14]), "6", "+3 g (est.)",
    f"front lead {gD:.1f} mm from a moving cord", "unbalanced: ~0.4 mT", "tight to moving cords; stray field at the servo encoders",
    ("wires", bundle(14, 2) + [dD]))
add("F", "C inside a printed cover rail", "14 AWG in a clip-on PA-CF channel on the back face", "as C, plus the rail (2 clips on the frame)",
    loss(R[14]), "6", "+6 g (est.)", "enclosed", "bundled: ~0.06 mT", "protects the leads in a crash; one more printed part", ("rail", bC))
add("G", "Earlier idea: 18 AWG down both side corridors", "motor leads, one at the front (y 23.5), two at the back",
    "2 tunnels in the top block + plate notches", loss(R[18]), "3", "0 g",
    f"front lead {gG:.1f} mm: rubs two moving cords", "unbalanced: ~0.4 mT", "rejected: the first search left the cords out",
    ("wires", [(-1.0, 23.5, 1.15), (-8.5, -23.5, 1.15), (8.5, -23.5, 1.15)]))
add("H", "Move the servos to open a centre channel", "14 AWG up the middle; servo stack split by 10 mm",
    "frame, spools, cords and idlers all move", loss(R[14]), "6", "+8 g (est.)", "3 mm (est.)", "bundled", "not needed: the back face is already empty; hand 5 mm wider",
    ("centre", None))

# --- figure -------------------------------------------------------------------------------------------------------
fig = plt.figure(figsize=(20, 15.5), facecolor="white")
fig.suptitle("Motor power from the coupler bullets through hand3 hub E to the motor: 8 options", fontsize=17, fontweight="bold", y=0.992)
fig.text(0.5, 0.966, f"Must carry 3 phases at {I_HOVER:.0f} A hover / {I_MAX:.1f} A max (est.) over about 0.1 m, clear the moving cords, spools and the rotating cage, "
         "use stock parts, and leave the hand outline alone.   Plan sections at hand3 z 30, to scale (mm): grey = parts, blue = moving cords, red = conductors.",
         ha="center", fontsize=10.5)
RED, BLUE = "#d62728", "#1f77b4"
def plan(ax, o):
    ax.set_aspect("equal"); ax.imshow(SEC, origin="lower", extent=[LO[0], -LO[0], LO[1], -LO[1]], cmap="Greys", vmin=0, vmax=1.8)
    for p, q in cords: ax.plot([p[0], q[0]], [p[1], q[1]], color=BLUE, lw=1.6, alpha=0.8, solid_capstyle="round")
    for xs in (12.0, -12.0): ax.add_patch(Rectangle((xs - 5.5, 18), 11, 6, fill=False, ec=BLUE, lw=1))
    ax.set_xlim(-38, 38); ax.set_ylim(-34, 34); ax.set_xticks([]); ax.set_yticks([])
    ax.set_title("\n".join(textwrap.wrap(f"{o['key']}  {o['title']}", 40)), fontsize=11.5, fontweight="bold", color="#1a7f37" if o["tint"] else "black")
    kind, d = o["draw"]
    if kind in ("wires", "rail"):
        if kind == "rail": ax.add_patch(Rectangle((-7.2, YB - 5.3), 14.4, 5.3, fill=False, ec="#444", lw=2))
        for x, y, r in d: ax.add_patch(Circle((x, y), r, color=RED))
    if kind == "strips":
        for x, y in d: ax.add_patch(Rectangle((x - 3.2, y), 6.35, 0.7, color=RED))
    if kind == "centre":
        for s in (-1, 1): ax.add_patch(Rectangle((-22.6, s * 5 if s > 0 else -22.5), 45.2, 17.5, fill=False, ec="#444", lw=2, ls="--"))
        for a in (0, 120, 240): ax.add_patch(Circle((2.2 * np.cos(np.radians(a)), 2.2 * np.sin(np.radians(a))), 1.75, color=RED))
    ax.text(0, 0, "servo stack", ha="center", va="center", color="white", fontsize=9)
    ax.text(0, 29.5, "front: spools + cords", ha="center", color=BLUE, fontsize=8.5); ax.text(0, -31.5, "back: empty", ha="center", color="#555", fontsize=8.5)
    ax.text(-37, -33, f"{o['loss_W']:.1f} W", fontsize=12, fontweight="bold", color=RED, va="bottom")

W4, H4 = 0.19, 0.265
for i, o in enumerate(opts):
    plan(fig.add_axes([0.008 + (i % 4) * 0.197, 0.672 - (i // 4) * 0.305, W4, H4]), o)

# side view shared by the back-face options
ax = fig.add_axes([0.80, 0.385, 0.195, 0.55]); ax.set_aspect("equal")
ax.imshow(ELEV, origin="lower", extent=[-50, 50, -8, 72], cmap="Greys", vmin=0, vmax=1.8)
for p, q in cords: ax.plot([p[1], q[1]], [p[2], q[2]], color=BLUE, lw=1.4, alpha=0.8)
route = np.array([[-9, -5.5], [-16.5, -3.5], [-19.75, 1.0], [-19.75, 44.0]]); stub = np.array([[-19.75, 44.0], [-19.2, 51.0], [-18.9, 57.5], [-18.2, 60.5]])
ax.plot(route[:, 0], route[:, 1], color=RED, lw=3.2, solid_capstyle="round"); ax.plot(stub[:, 0], stub[:, 1], color=RED, lw=1.8)
ax.plot([-19.75], [44.0], "o", color="black", ms=6)
for txt, xy, xt in (("inline joint", (-19.75, 44), (-47, 40)), ("14 AWG", (-19.75, 22), (-47, 20)), ("18 AWG stub", (-19.0, 54), (-48, 50.5)),
                    ("from the bullets", (-13, -4.5), (-47, -6.5))):
    ax.annotate(txt, xy, xt, fontsize=8.5, arrowprops=dict(arrowstyle="-", lw=0.7), va="center")
for txt, x, y, c in (("servo 1", 0, 14, "white"), ("servo 2", 0, 39, "white"), ("top block", 0, 54.2, "white"), ("motor", 0, 63, "white"),
                     ("swept\ncage", 34, 63, "black"), ("hand base", 30, -6.5, "black")):
    ax.text(x, y, txt, ha="center", va="center", color=c, fontsize=8.5)
ax.set_xlim(-50, 46); ax.set_ylim(-8, 72); ax.set_xticks([]); ax.set_yticks([])
ax.set_title("Side view through the middle (C shown)\nback face on the left", fontsize=11.5, fontweight="bold")

rows = [("", "Conductor", "Geometry changes", "Loss at hover\n(3 phases)", "Solder joints", "Mass", "Clearance (grid 1 mm, cords exact)", "Stray field at servo encoder (est.)", "Note")]
rows += [(f"{o['key']}  {o['title']}", o["conductor"], o["changes"], f"{o['loss_W']:.1f} W  ({loss_at_max:.0f} W at max)", o["joints"], o["mass"], o["clearance"], o["field"], o["note"])
         for o in opts for loss_at_max in [o["loss_W"] * (I_MAX / I_HOVER) ** 2]]
tax = fig.add_axes([0.008, 0.006, 0.984, 0.345]); tax.axis("off")
W = [0.155, 0.185, 0.145, 0.085, 0.07, 0.055, 0.105, 0.085, 0.115]; X = np.cumsum([0] + W[:-1]); wrap = [29, 36, 28, 15, 13, 10, 20, 16, 22]; n = len(rows)
for i, row in enumerate(rows):
    y = 1 - (i + 1) / n
    tint = i > 0 and opts[i - 1]["tint"]
    for j, cc in enumerate(row):
        tax.add_patch(Rectangle((X[j], y), W[j], 1 / n, facecolor="#dff3e3" if tint else ("#eeeeee" if i == 0 else "white"), edgecolor="#bbb", transform=tax.transAxes))
        tax.text(X[j] + 0.003, y + 0.5 / n, "\n".join(textwrap.wrap(cc, wrap[j])) if "\n" not in cc else cc, transform=tax.transAxes, va="center",
                 fontsize=9, fontweight="bold" if (i == 0 or j == 0) else "normal")
fig.text(0.008, 0.358, "Recommended: C (green).  Also looked at and ruled out: leads over the top of the motor (the rotating cage and pull rings enclose it) and 20 AWG (12.9 W).",
         fontsize=10.5, fontweight="bold")
fig.savefig("hubE_lead_options_study.png", dpi=85)
out = {o["key"]: {k: v for k, v in o.items() if k != "draw"} for o in opts}
out["_checks"] = dict(cord_gap_back_18=gA, cord_gap_back_14=gC, cord_gap_front_14=gD, cord_gap_corridor_18=gG, part_gap_back_14=pC, part_gap_back_18=pA,
                      I_hover=I_HOVER, I_max=I_MAX, L_run=L_RUN, L_stub=L_STUB)
json.dump(out, open("hubE_lead_options_study.json", "w"), indent=1, default=float)
for o in opts: print(o["key"], f"{o['loss_W']:.1f} W", "|", o["clearance"])
