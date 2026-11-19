"""Figure for hand3 hub v3: plan views of the four poses, a section through one root, the ring-stroke decision, check results.
Reads hand3_hub_v3_check.json.  Writes hand3_hub_v3.png"""
import os, sys, json, textwrap
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hubc as HC, hand3_hub_v3 as V3
os.chdir(HERE)
CK = json.load(open("hand3_hub_v3_check.json")); HUB = json.load(open("hand3e/hand3_hubc.json")); ST = json.load(open("hand3_indep_fingers_study.json"))["common"]
COL = dict(disc="#c5cad1", block=["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"], pin="#d62728", bead="#111111", box="#e6c229", finger="#3a3a40", rail="#e07b39")
def draw(ax, m, fc, ec="black", lw=0.5, alpha=1.0, z=1):
    for poly in m.project().to_polygons():
        p = np.asarray(poly)
        if len(p) > 2: ax.add_patch(Polygon(p, closed=True, fc=fc, ec=ec, lw=lw, alpha=alpha, zorder=z))
fig = plt.figure(figsize=(20, 13.0), facecolor="white")
fig.suptitle("hand3 hub v3 (CAD): finger roots that travel round the hub, each locked by two axial latch pins", fontsize=17, fontweight="bold", y=0.995)
fig.text(0.5, 0.952, "Top views from above, to scale (mm), fingers straight.  Grey = discs, colours = root blocks (finger fixed in its block), red = latch pins, "
         "yellow = return-spring boxes (all same hand), black = beads on the ring rail (orange).", ha="center", fontsize=10.5)
for i, (pose, azs) in enumerate(V3.POSES.items()):
    ax = fig.add_axes([0.005 + i * 0.165, 0.50, 0.16, 0.42]); ax.set_aspect("equal"); ax.set_xlim(-62, 62); ax.set_ylim(-62, 62); ax.set_xticks([]); ax.set_yticks([])
    draw(ax, V3.ring_rail(), COL["rail"], z=1); draw(ax, V3.base_disc(), COL["disc"], z=2)
    for k, az in enumerate(azs):
        for n, m in V3.finger(az).items(): draw(ax, m, COL["finger"] if "phalanx" in n else "#c23b22", z=3)
        draw(ax, V3.root_block(az), COL["block"][k], z=4); draw(ax, V3.bead(az), COL["bead"], z=4)
    draw(ax, V3.top_disc(), COL["disc"], alpha=0.55, z=5)
    for k, az in enumerate(azs):
        draw(ax, V3.spring_box(az, k), COL["box"], alpha=0.9, z=6)
        for s in (-1, 1): draw(ax, V3.latch_pin(az, s), COL["pin"], z=7)
    draw(ax, C.parts()["heel"], "#2f2f2f", z=8)
    c = CK["curl"][pose]
    ax.set_title(f"{pose}\nempty-hand curl clear to {int(round(100 * c['max_clear_curl']))} %", fontsize=11.5, fontweight="bold")
    ax.plot([35, 55], [-58, -58], color="black", lw=2); ax.text(45, -56, "20 mm", ha="center", fontsize=8)
# section through root 0 (az 0) in the x-z plane
ax = fig.add_axes([0.665, 0.47, 0.33, 0.46]); ax.set_aspect("equal")
def sec(m, fc, alpha=1.0, z=1, ec="black"):
    s = m.rotate([-90, 0, 0])                     # slicing at height 0 cuts the plane y = 0; section coordinates (x, z)
    for poly in s.slice(0.0).to_polygons():
        p = np.asarray(poly)
        if len(p) > 2: ax.add_patch(Polygon(p, closed=True, fc=fc, ec=ec, lw=0.5, alpha=alpha, zorder=z))
def sec_off(m, y, fc, z=1):
    s = m.translate([0, -y, 0]).rotate([-90, 0, 0])
    for poly in s.slice(0.0).to_polygons():
        p = np.asarray(poly)
        if len(p) > 2: ax.add_patch(Polygon(p, closed=True, fc=fc, ec="black", lw=0.5, zorder=z))
HP = HC._parts(); hw = HC.hardware()
sec(HP["cage"][0], "#c9cfd6"); sec(HP["ring_out"][0], "#e07b39", 0.85); sec(V3.ring_rail(), "#e07b39"); sec(hw["motor_base"], "#555a63")
sec(V3.base_disc(), COL["disc"], z=2); sec(V3.top_disc(), COL["disc"], z=2); sec(V3.root_block(0), COL["block"][0], 0.8, z=3); sec(V3.bead(0), COL["bead"], z=4)
sec_off(V3.latch_pin(0, 1), V3.PIN_LAT, COL["pin"], z=5); sec(V3.spring_box(0, 0), COL["box"], 0.9, z=3)
sec(C.parts()["heel"], "#2f2f2f", z=3)
lip = HC.R_LIP_OUT[0] + 0.6
ax.add_patch(Rectangle((lip, HC.Z_LIP_OUT[0]), HC.R_LIP_OUT[1] - lip, 2.0, fc="#f0a030", ec="black", zorder=3))
ax.plot([V3.V["bead"]["eye_r"]] * 2, [V3.Z_RING_TOP + V3.V["bead"]["dz"][1], 94.0], color=COL["pin"], lw=1.5, zorder=6)
for txt, xy, xt in (("top disc", (12, 100.5), (2, 117)), ("root block (finger fixed in it)", (14, 96), (2, 121)), ("latch pin (its own plane, 7.5 mm off the cut)", (24, 95.9), (30, 119)),
                    ("base disc: lip + pocket for the pin's foot", (20, 92), (30, 82)), ("bead on the ring rail; curl cord up into the root", (25.4, 88.6), (30, 78)),
                    ("base pull ring", (27, 85), (34, 74)), ("yoke lip (static), moved out 0.6 mm", (28.5, 88), (36, 88)), ("spring box on the root's post", (25, 104), (30, 112))):
    ax.annotate(txt, xy, xt, fontsize=8.5, arrowprops=dict(arrowstyle="-", lw=0.6), va="center")
ax.set_xlim(0, 62); ax.set_ylim(70, 124); ax.set_xticks([]); ax.set_yticks([]); ax.set_title("Section through one root (radius across, height up)", fontsize=11.5, fontweight="bold")
# stroke decision
S = CK["stroke"]; need, hold = ST["base_cord_peak_need_N"], ST["base_cord_hold_need_N"]; Fs, Fr = HUB["base"]["F_stall_finger"], HUB["base"]["F_rated_finger"]; eta = 0.8
opts = [("S1", "2:1 tackle at the bead + both servos pull the one ring", f"{S['one_cord'] / 2:.1f}", f"{Fs * eta:.0f} / {Fr * eta:.0f}", f"{Fs * eta / need:.2f} / {Fr * eta / hold:.2f}", "2", "-8 (inner ring gone)", "inner yokes re-hooked onto the base ring; eyelet on each bead", True),
        ("S2", "one servo, 1:1, ring travels the full stroke", f"{S['one_cord']:.1f}", f"{Fs:.0f} / {Fr:.0f}", f"{Fs / need:.2f} / {Fr / hold:.2f}", "1", "-63", "yoke legs, idlers and cord routes of the drive re-laid out (yoke eye would hit an idler)", False),
        ("S3", "two rings again (K1 / K2+K3), a rail on each", f"{S['K1'] + 2:.1f} / {S['K2_K3'] + 2:.1f}", "104 / 35", f"{Fs / need:.2f} / {Fr / hold:.2f}", "2", "0", "the inner ring's cords would have to cross the outer ring and the disc: no path found", False),
        ("S4", "one servo, keep today's 13.7 mm ring stroke, curl less", "13.7", f"{Fs:.0f} / {Fr:.0f}", f"{Fs / need:.2f} / {Fr / hold:.2f}", "1", "-63", f"fingers reach {100 * (13.72 - 2) / (S['one_cord'] - 2):.0f} % of full fold", False),
        ("S5", "2:1 tackle, one servo", f"{S['one_cord'] / 2:.1f}", f"{Fs * eta / 2:.0f} / {Fr * eta / 2:.0f}", f"{Fs * eta / 2 / need:.2f} / {Fr * eta / 2 / hold:.2f}", "1", "-63", "too weak: below the grip need", False)]
tax = fig.add_axes([0.005, 0.135, 0.99, 0.28]); tax.axis("off")
tax.text(0, 1.04, f"Decision needed: one cord over K1 to K3 needs {S['one_cord']:.1f} mm of cord travel (K1 {S['K1']:.1f} + K2/K3 {S['K2_K3']:.1f} + stretch {S['stretch']:.0f}); today's rings move {S['today_base_ring']:.1f} and {S['today_curl_ring']:.1f} mm. "
         f"Grip need per finger {need:.0f} N peak / {hold:.0f} N hold (est.).", transform=tax.transAxes, fontsize=11, fontweight="bold")
hdr = ("", "Option", "Ring stroke (mm)", "Cord force per finger, stall / rated (N)", "Margin peak / hold", "Servos", "Mass vs hub E (g, est.)", "What changes")
rows = [hdr] + [o[:-1] for o in opts]; Wc = [0.03, 0.22, 0.08, 0.13, 0.09, 0.05, 0.1, 0.30]; X = np.cumsum([0] + Wc[:-1]); n = len(rows)
for i, r_ in enumerate(rows):
    y = 1 - (i + 1) / n
    for j, c in enumerate(r_):
        tint = i and opts[i - 1][-1]
        tax.add_patch(Rectangle((X[j], y), Wc[j], 1 / n, fc="#dff3e3" if tint else ("#eeeeee" if i == 0 else "white"), ec="#bbb", transform=tax.transAxes))
        tax.text(X[j] + 0.003, y + 0.5 / n, "\n".join(textwrap.wrap(str(c), [6, 48, 16, 26, 18, 8, 20, 70][j])), transform=tax.transAxes, va="center", fontsize=9.3, fontweight="bold" if (i == 0 or j <= 1) else "normal")
# checks summary
F = CK["fits"]; worst = sorted(F.items(), key=lambda kv: kv[1])[:8]
pairs = [p for p in CK["pairs"] if p["ok"]]; mn = min(pairs, key=lambda p: p["gap_mm"]) if pairs else None
bad = [p for p in CK["pairs"] if not p["ok"]]
lines = [f"Range math between roots (all 4 poses): {'no overlaps' if not bad else str(len(bad)) + ' overlaps'}; tightest {mn['gap_mm']} mm ({mn['a']} vs {mn['b']}, r {mn['at_r']}).",
         "Tightest fits (mm): " + "; ".join(f"{k} {v}" for k, v in worst) + ".",
         "Empty-hand curl: " + "; ".join(f"{p} {int(round(100 * v['max_clear_curl']))} %" for p, v in CK["curl"].items()) + " (then opposite or neighbouring fingertips touch; hub E with spread: 70 %).",
         "Not yet modelled: the parked fork + roller station and its sensor, the spring box's foot on the post, cord channels inside the grip, the drive changes of the stroke option."]
fig.text(0.005, 0.115, "\n".join("\n".join(textwrap.wrap(l, 205)) for l in lines), fontsize=10, va="top")
fig.savefig("hand3_hub_v3.png", dpi=85)
print("ok")
