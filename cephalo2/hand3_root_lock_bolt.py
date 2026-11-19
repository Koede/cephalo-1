"""hand3 option F, root lock v3 "rail block + two axial latch pins" (2026-11-19): to-scale layout + interval checks.
Change found while laying it out: with roots that travel, the old sideways swing (lag) of each finger is dropped - the root is a
fixed BLOCK.  Reason: a swinging root grip sweeps the space the pins need (hand3_root_lock_bolt.json: keep_swing).
Block: PA-CF wedge between a base disc and a top disc, two ribs top and bottom riding in two ring grooves of each disc (they
carry the finger's pull, both ways).  Lock: two L-shaped dia 2 music-wire pins, one each side of the neck, axial legs down
through the block into holes in the base disc; their arms lie in slots in the block and stick out past the rim as lift tabs.
Each leg ends in a small outward foot; the spin throws the pin 0.5 mm outward so the foot catches under the disc.
Estimates unless read from hand3_design / hand3_hubc.  Writes hand3_root_lock_bolt.png / .json"""
import os, sys, json, textwrap
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Wedge, Polygon
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_hubc as C
os.chdir(HERE)
ST = json.load(open("hand3_indep_fingers_study.json"))["common"]
RPM = 12500.0; W2 = (RPM * 2 * np.pi / 60) ** 2; M_F, R_COM, F_CF = ST["finger_g"] * 1e-3, ST["finger_com_r"], ST["finger_cf_N"]
D = dict(disc_r=H.HUB["plate_d"] / 2, disc_t=2.0, gap=6.0, z0=H.Z["hub_lo"], boss_r=7.0,
         block_r=(9.0, 24.3), block_half=27.0, neck_half=5.0, rib_h=1.0, ribs=((13.5, 15.5), (22.3, 24.0)), groove_clr=0.1,
         pin_r=20.0, pin_daz=21.0, wire_d=2.0, hole=(2.1, 2.6), lift=2.3, foot=0.4, float_=0.5, slot_w=2.2, tab_r=28.5, step=30.0,
         detent_N=3.0, T_motor=0.62, wire_g_per_mm=np.pi * 1.0 * 7.85e-3)
z0, t, g = D["z0"], D["disc_t"], D["gap"]; stack = 2 * t + g
lat = D["pin_r"] * np.sin(np.radians(D["pin_daz"]))                       # sideways distance of each pin from the root's centre line
holes = sorted((k * D["step"] + s * D["pin_daz"]) % 360 for k in range(int(360 / D["step"])) for s in (-1, 1))
dh = np.diff(holes + [holes[0] + 360]).min(); web = np.radians(dh) * D["pin_r"] - D["hole"][0]
edge_wall = np.radians(D["block_half"] - D["pin_daz"]) * D["pin_r"] - D["hole"][0] / 2
rib_len = [np.radians(2 * D["block_half"]) * r1 for r0, r1 in D["ribs"]]
rib_area = (sum(rib_len) * 2 - 2 * D["slot_w"]) * D["rib_h"]             # top + bottom, minus the two arm slots through the top outer rib
neck_area = 2 * D["neck_half"] * g
L_pin = 6.5 + (D["tab_r"] - D["pin_r"]) ; m_pin = L_pin * D["wire_g_per_mm"]
cf_pin = m_pin * 1e-3 * W2 * 0.024
side = D["T_motor"] / 4 / (D["pin_r"] * 1e-3)
tip_side = 20.0; lock_N = tip_side * 0.114 / (D["pin_r"] * 1e-3)
def imb(az): v = sum(np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]) for a in az); return M_F * W2 * R_COM * 1e-3 * np.linalg.norm(v)
POSES = {"flight cross": (0, 90, 180, 270), "2 + 2 pinch": (30, 150, 210, 330), "3 + 1 thumb": (30, 90, 150, 270), "all four on one side": (0, 60, 120, 180)}
def used(az): return sorted((a + s * D["pin_daz"]) % 360 for a in az for s in (-1, 1))
clash = {k: len(set(used(v))) < 8 for k, v in POSES.items()}
foot_z = z0 - 0.8; ring_top = C.Z_OUT[1]
chk = [
 ("stack height", f"{t:g} + {g:g} + {t:g} = {stack:g} mm", f"same as today ({H.Z['hub_hi'] - H.Z['hub_lo']:g}): shaft and nut unchanged", abs(stack - (H.Z["hub_hi"] - H.Z["hub_lo"])) < 1e-6),
 ("closest two roots", f"block spans {2 * D['block_half']:.0f} deg", f"60 deg apart: {60 - 2 * D['block_half']:.0f} deg ({np.radians(60 - 2 * D['block_half']) * D['block_r'][0]:.1f} mm at the inner end) between blocks", 2 * D["block_half"] < 60),
 ("holes in the base disc", f"{len(holes)} holes, dia {D['hole'][0]} x {D['hole'][1]} slots, on r {D['pin_r']:g}", f"closest pair {dh:.0f} deg: web {web:.1f} mm", web > 1.5),
 ("no two roots want the same hole", "all four poses", "ok" if not any(clash.values()) else "CLASH " + str(clash), not any(clash.values())),
 ("pins either side of the neck", f"{lat:.2f} mm off the centre line", f"neck half-width {D['neck_half']:g}: {lat - D['neck_half'] - D['wire_d'] / 2:.2f} mm between wire and neck; {edge_wall:.2f} mm wall to the block's edge", lat - D["neck_half"] - D["wire_d"] / 2 > 0.5 and edge_wall > 1.0),
 ("finger pull on the ribs", f"{F_CF:.0f} N on {rib_area:.0f} mm2", f"{F_CF / rib_area:.1f} MPa bearing (printed PA-CF)", F_CF / rib_area < 20),
 ("finger pull through the neck", f"{F_CF:.0f} N on {neck_area:.0f} mm2", f"{F_CF / neck_area:.1f} MPa tension", F_CF / neck_area < 25),
 ("pins, full motor torque", f"{side:.1f} N per root", f"{side / 2 / (np.pi * D['wire_d'] ** 2 / 4):.1f} MPa shear per pin", True),
 ("pins, 20 N sideways on a fingertip (parked)", f"{lock_N:.0f} N at the pins", f"{lock_N / 2 / (np.pi * D['wire_d'] ** 2 / 4):.0f} MPa shear, {lock_N / 2 / (D['wire_d'] * t):.0f} MPa on the hole edge", lock_N / 2 / (D["wire_d"] * t) < 30),
 ("keeper while spinning", f"pin {m_pin:.2f} g thrown outward with {cf_pin:.1f} N", f"foot {D['foot']} mm catches under the disc (float {D['float_']} mm): cannot lift", cf_pin > 2 and D["float_"] > D["foot"]),
 ("keeper at rest", f"printed snap nib, {D['detent_N']:.0f} N", f"x{D['detent_N'] / (m_pin * 1e-3 * 9.81 * 5):.0f} over 5 g of shake", True),
 ("pin lift inside the block", f"lift {D['lift']} mm, wire {D['wire_d']} mm", f"slot depth needed {D['lift'] + D['wire_d']:.1f} of {g:g} mm: {g - D['lift'] - D['wire_d']:.1f} mm of block left under the arm", g - D["lift"] - D["wire_d"] > 1.0),
 ("pin feet under the disc", f"r {D['pin_r'] - 1:g} to {D['pin_r'] + 1 + D['float_'] + D['foot']:.1f}, z {foot_z:.1f} to {z0:g}", f"nearest spinning part below is the curl ring (top z {ring_top:g}): {foot_z - ring_top:.1f} mm lower", foot_z - ring_top > 1.0),
 ("lift tabs vs the yoke lips (do not spin)", f"tabs r {D['disc_r']:g} to {D['tab_r']:g}, z {z0 + t + g - D['lift'] - D['wire_d']:.1f} and up", f"lips r {C.R_LIP_OUT[0]} to {C.R_LIP_IN[1]} top z {C.Z_LIP_OUT[1]:g}: {z0 + t + g - D['lift'] - D['wire_d'] - 1.0 - C.Z_LIP_OUT[1]:.1f} mm above them", z0 + t + g - D["lift"] - D["wire_d"] - 1.0 - C.Z_LIP_OUT[1] > 1.0),
 ("shake from 0.05 mm hole play", f"{0.05 / D['pin_r'] * R_COM:.2f} mm at the finger's centre of mass", f"{M_F * W2 * 0.05 / D['pin_r'] * R_COM * 1e-3:.1f} N", True),
] + [(f"imbalance if spun in '{k}'", "", f"{imb(v):.0f} N" + ("" if imb(v) < 5 else ": motor blocked"), imb(v) < 5) for k, v in POSES.items()] + [
 ("cord path from the curl ring (r 22 to 24) to the finger", "ring sits under the disc (r 25.5)", "no straight path yet: needs its own layout", False)]
keep_swing = "with the swinging root grip, pins clear of its sweep need a shoe 56 to 68 deg wide (numeric sweep of the CAD grip), so roots could not sit 60 deg apart, and the lift tabs cannot cross the grip"
json.dump(dict(layout=D, derived=dict(lateral=lat, holes_deg=holes, web=web, rib_area=rib_area, pin_g=m_pin, pin_cf_N=cf_pin), keep_swing=keep_swing,
               checks=[dict(what=a, value=b, result=c, ok=bool(d)) for a, b, c, d in chk], poses=POSES), open("hand3_root_lock_bolt.json", "w"), indent=1, default=float)

fig = plt.figure(figsize=(20, 15), facecolor="white")
fig.suptitle("hand3 travelling roots, v3: root block on two rails, locked by two axial latch pins beside the neck (to scale, mm)", fontsize=17, fontweight="bold", y=0.992)
fig.text(0.5, 0.966, "Grey = fixed to the shaft.  Colours = the four root blocks (the finger is fixed in its block: the old sideways swing is dropped).  Red = latch pins (bent dia 2 music wire).  Orange = parked fork, does not spin.", ha="center", fontsize=10.5)
COL = ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"]; RED = "#d62728"
def plan(ax, az, title, lifted=None):
    ax.set_aspect("equal"); ax.set_xlim(-50, 50); ax.set_ylim(-44, 44); ax.set_xticks([]); ax.set_yticks([])
    ax.add_patch(Circle((0, 0), D["disc_r"], fc="#e6e8eb", ec="#555"))
    for r0, r1 in D["ribs"]: ax.add_patch(Wedge((0, 0), r1 + 0.1, 0, 360, width=r1 - r0 + 0.2, fc="#c5cad1", ec="none"))
    for a in holes: ax.add_patch(Circle((D["pin_r"] * np.cos(np.radians(a)), D["pin_r"] * np.sin(np.radians(a))), 1.05, fc="white", ec="#555", lw=0.6))
    for i, a in enumerate(az):
        u = np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]); n = np.array([-u[1], u[0]])
        ax.add_patch(Polygon([u * 24 + n * 5, u * 30 + n * 9, u * 50 + n * 8, u * 50 - n * 8, u * 30 - n * 9, u * 24 - n * 5], fc=COL[i], ec="black", lw=0.5, alpha=0.3))
        ax.add_patch(Wedge((0, 0), D["block_r"][1], a - D["block_half"], a + D["block_half"], width=D["block_r"][1] - D["block_r"][0], fc=COL[i], ec="black", lw=0.8, alpha=0.8))
        for s in (-1, 1):
            b = np.radians(a + s * D["pin_daz"]); p = D["pin_r"] * np.array([np.cos(b), np.sin(b)]); q = p + u * (D["tab_r"] - D["pin_r"] * np.cos(np.radians(D["pin_daz"])))
            ax.plot([p[0], q[0]], [p[1], q[1]], color=RED, lw=2.4, solid_capstyle="round"); ax.add_patch(Circle(p, 1.0, fc="white" if lifted == i else RED, ec=RED, lw=1.2))
        if lifted == i:
            for s in (-1, 1): ax.add_patch(Polygon([u * 26.3 + n * s * 5.6, u * 40 + n * s * 5.6, u * 40 + n * s * 9, u * 26.3 + n * s * 9], fc="#f0a030", ec="black", lw=0.7))
            ax.add_patch(Polygon([u * 36 + n * 9, u * 40 + n * 9, u * 40 - n * 9, u * 36 - n * 9], fc="#f0a030", ec="black", lw=0.7))
    ax.add_patch(Circle((0, 0), D["boss_r"], fc="#b9bec5", ec="#555")); ax.add_patch(Circle((0, 0), 2.5, fc="#444"))
    ax.plot([28, 48], [-41, -41], color="black", lw=2); ax.text(38, -39.5, "20 mm", ha="center", fontsize=8); ax.set_title(title, fontsize=11.5, fontweight="bold")
n_ = list(POSES)
plan(fig.add_axes([0.008, 0.535, 0.235, 0.4]), POSES[n_[0]], "Flight cross: all 8 pins down")
plan(fig.add_axes([0.25, 0.535, 0.235, 0.4]), POSES[n_[2]], "3 + 1 thumb grasp: fork lifts one pin pair, hub turns under it", lifted=3)
plan(fig.add_axes([0.008, 0.1, 0.235, 0.4]), POSES[n_[1]], "2 + 2 pinch (replaces the old sideways swing)")
plan(fig.add_axes([0.25, 0.1, 0.235, 0.4]), POSES[n_[3]], "All four on one side (60 deg apart)")
def R(ax, a0, a1, b0, b1, **kw): ax.add_patch(Rectangle((a0, b0), a1 - a0, b1 - b0, **kw))
# section 1: along the root (radius - height)
ax = fig.add_axes([0.495, 0.70, 0.5, 0.235]); ax.set_aspect("equal"); zt = z0 + t + g
R(ax, 0, 17, 83, 89, fc="#555a63", ec="black"); ax.text(8.5, 86, "motor bell", color="white", ha="center", va="center", fontsize=8.5); R(ax, 0, 2.5, 89, H.Z["shaft_tip"], fc="#333", ec="none")
R(ax, C.R_IN[0], C.R_IN[1], 83, 87, fc="#39a0e0", ec="black", alpha=.8); R(ax, C.R_OUT[0], C.R_OUT[1], 84, 87, fc="#e07b39", ec="black", alpha=.8)
R(ax, C.R_LIP_OUT[0], C.R_LIP_OUT[1], *C.Z_LIP_OUT, fc="#cfcfc8", ec="black")
R(ax, 2.5, D["disc_r"], z0, z0 + t, fc="#b9bec5", ec="black"); R(ax, 2.5, D["disc_r"], zt, zt + t, fc="#b9bec5", ec="black"); R(ax, 2.5, D["boss_r"], z0 + t, zt, fc="#b9bec5", ec="black")
R(ax, D["block_r"][0], D["block_r"][1], z0 + t, zt, fc=COL[0], ec="black", alpha=.85); R(ax, D["block_r"][1], 52, z0 + t + 0.5, zt - 0.5, fc=COL[0], ec="black", alpha=.3)
for r0, r1 in D["ribs"]:
    R(ax, r0, r1, z0 + t - D["rib_h"], z0 + t, fc=COL[0], ec="black"); R(ax, r0, r1, zt, zt + D["rib_h"], fc=COL[0], ec="black")
za = zt - D["lift"] - D["wire_d"]
R(ax, D["pin_r"] - 1, D["pin_r"] + 1, z0 - 0.8, za + D["wire_d"], fc=RED, ec="black", lw=0.6); R(ax, D["pin_r"] - 1, D["tab_r"], za, za + D["wire_d"], fc=RED, ec="black", lw=0.6); R(ax, D["pin_r"] + 1, D["pin_r"] + 1.4, z0 - 0.8, z0 - 0.1, fc=RED, ec="black", lw=0.6)
R(ax, 26.3, 40, za - 1.4, za - 0.1, fc="#f0a030", ec="black"); R(ax, 37, 38.5, 83, za - 1.4, fc="#f0a030", ec="black")
ax.annotate("", (31, za + 5.2), (31, za + 2.2), arrowprops=dict(arrowstyle="->", lw=1.6)); ax.text(32, za + 4.3, f"lift {D['lift']}", fontsize=8.5)
for txt, xy, xt in (("ribs in ring grooves carry the 826 N pull", (23, zt + 0.5), (27, 106)), ("root block", (11, 96), (-3, 106)), ("latch pin: leg, arm, lift tab", (24, za + 1), (42, 101.5)),
                    ("foot catches under the disc when spinning", (21.2, z0 - 0.5), (42, 86)), ("fork (parked only)", (39, za - 0.7), (42, 90.5)), ("yoke lip (does not spin)", (29, 88), (42, 83.2)), ("pull rings", (23, 85), (-3, 80.5))):
    ax.annotate(txt, xy, xt, fontsize=8.5, arrowprops=dict(arrowstyle="-", lw=0.7), va="center")
ax.set_xlim(-4, 74); ax.set_ylim(79, 108); ax.set_xticks([]); ax.set_yticks([]); ax.set_title("Section along the root, through one pin (radius across, height up)", fontsize=11.5, fontweight="bold")
# section 2: across the root at the pin circle (sideways - height)
ax = fig.add_axes([0.495, 0.50, 0.24, 0.165]); ax.set_aspect("equal")
w = D["pin_r"] * np.sin(np.radians(D["block_half"]))
R(ax, -14, 14, z0, z0 + t, fc="#b9bec5", ec="black"); R(ax, -14, 14, zt, zt + t, fc="#b9bec5", ec="black"); R(ax, -w, w, z0 + t, zt, fc=COL[0], ec="black", alpha=.85)
ax.plot([-D["neck_half"], -D["neck_half"]], [z0 + t, zt], color="black", lw=0.8, ls=":"); ax.plot([D["neck_half"], D["neck_half"]], [z0 + t, zt], color="black", lw=0.8, ls=":")
for s in (-1, 1):
    R(ax, s * lat - 1.1, s * lat + 1.1, za, zt, fc="white", ec="black", lw=0.6); R(ax, s * lat - 1, s * lat + 1, z0 - 0.8, za + D["wire_d"], fc=RED, ec="black", lw=0.6)
ax.text(0, z0 + t + 3, "neck", ha="center", fontsize=8.5); ax.text(0, zt + 2.6, f"two pins, {lat:.1f} mm either side of the middle", ha="center", fontsize=8.5)
ax.set_xlim(-15, 15); ax.set_ylim(z0 - 2, zt + 4); ax.set_xticks([]); ax.set_yticks([]); ax.set_title("Section across the root at the pins", fontsize=11, fontweight="bold")
fig.text(0.745, 0.585, "\n".join(textwrap.wrap("Why the sideways swing goes: " + keep_swing + ". Moving the roots gives the same 2 + 2 pose and more.", 62)), fontsize=9.5, va="center")
tax = fig.add_axes([0.495, 0.03, 0.5, 0.455]); tax.axis("off"); rows = [("Check", "Value", "Result", "")] + [(a, b, c, "ok" if dd else "OPEN") for a, b, c, dd in chk]
Wc = [0.29, 0.25, 0.39, 0.07]; X = np.cumsum([0] + Wc[:-1]); n = len(rows)
for i, r_ in enumerate(rows):
    y = 1 - (i + 1) / n
    for j, c in enumerate(r_):
        bad = i and r_[3] == "OPEN"
        tax.add_patch(Rectangle((X[j], y), Wc[j], 1 / n, fc="#eeeeee" if i == 0 else ("#fde8e8" if bad else "white"), ec="#bbb", transform=tax.transAxes))
        tax.text(X[j] + 0.004, y + 0.5 / n, "\n".join(textwrap.wrap(c, [34, 30, 50, 6][j])), transform=tax.transAxes, va="center", fontsize=8.3, fontweight="bold" if (i == 0 or j == 0) else "normal")
fig.text(0.008, 0.04, "\n".join(textwrap.wrap("To move one root: park it at the fork -> fork nudges both tabs 0.5 mm inward (feet clear) and lifts them 2.3 mm -> a roller turns the hub in 30 deg steps under the held block -> fork lowers, pins snap into the new holes -> fork backs off.", 120)),
         fontsize=10, fontweight="bold")
fig.savefig("hand3_root_lock_bolt.png", dpi=85)
for a, b, c, dd in chk: print("ok  " if dd else "OPEN", a, "|", b, "|", c)
