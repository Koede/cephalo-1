"""hand3 option F, root lock as the user described it (2026-11-19): AXIAL pins, two per root, one each side of the root's
centre line.  To-scale layout + first checks (interval math and hand formulas; estimates marked est.).
Each root sits on a SHOE (two arc leaves above and below the root grip) that slides round the hub between a base disc and a
top disc.  The finger's outward pull goes from the shoe's outer edges into a rim lip on both discs; the two pins only stop
the shoe moving round.  Writes hand3_root_lock_axial.png / .json"""
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
L = dict(step=15.0, r_pin=17.7, d_pin=3.0, d_hole=3.05, pin_daz=15.0, pin_len=9.0, r_lag=H.R_LAG, d_lag=H.LAG_TUBE[0], shoe_r=(13.0, 24.5), shoe_half=23.0, leaf_t=1.5,
         disc_t=2.5, grip_t=H.HUB["grip_t"], lip_r=(24.5, 25.5), lip_h=1.5, tab_r=28.5, lift=2.9, spring_N=3.0, T_motor=0.62, mu=0.15, staple_g=1.3, shoe_g=2.2)
nh = int(360 / L["step"]); pitch = 2 * np.pi * L["r_pin"] / nh
px = L["r_pin"] * np.cos(np.radians(L["pin_daz"])) - L["r_lag"]; py = L["r_pin"] * np.sin(np.radians(L["pin_daz"]))
min_gap_deg = 2 * L["shoe_half"] + 0.5 / L["shoe_r"][1] * 57.3; min_step = np.ceil(min_gap_deg / L["step"]) * L["step"]
lip_area = np.radians(2 * L["shoe_half"]) * L["lip_r"][0] * L["leaf_t"] * 2
side = L["T_motor"] / 4 / (L["r_pin"] * 1e-3); stack = 2 * L["disc_t"] + 2 * L["leaf_t"] + L["grip_t"]
def imb(az):                                                    # rotor imbalance force for root azimuths az (fingers straight out)
    v = sum(np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]) for a in az); return M_F * W2 * R_COM * 1e-3 * np.linalg.norm(v)
POSES = {"flight cross": (45, 135, 225, 315), "2 + 2 pinch": (60, 120, 240, 300), "3 + 1 thumb": (30, 90, 150, 270), "all four on one side": (0, 60, 120, 180)}
chk = [
 ("hole pitch at r 17.7, 24 holes", f"{pitch:.2f} mm", f"web between holes {pitch - L['d_hole']:.2f} mm", pitch - L["d_hole"] > 1.5),
 ("lock pin to the swing pin", f"{np.hypot(px, py):.2f} mm apart", f"gap {np.hypot(px, py) - L['d_lag'] / 2 - L['d_pin'] / 2:.2f} mm", np.hypot(px, py) - L["d_lag"] / 2 - L["d_pin"] / 2 > 1.0),
 ("closest two roots can sit", f"shoe spans {2 * L['shoe_half']:.0f} deg", f"{min_step:.0f} deg apart (on the 15 deg grid)", True),
 ("finger pull into the rim lips", f"{F_CF:.0f} N on {lip_area:.0f} mm2", f"{F_CF / lip_area:.0f} MPa bearing (printed PA-CF: fine)", F_CF / lip_area < 30),
 ("side load on the pin pair, full motor torque", f"{side:.1f} N", f"{side / 2 / (np.pi * 2.25):.2f} MPa shear per pin", True),
 ("pins at rest", f"spring {L['spring_N']:.0f} N", f"vs {L['staple_g'] * 1e-3 * 9.81 * 5:.2f} N at 5 g of shake: x{L['spring_N'] / (L['staple_g'] * 1e-3 * 9.81 * 5):.0f}", True),
 ("pins while spinning", f"side load x friction {L['mu']}", f"+{side * L['mu']:.1f} N; pull is on the lips, so the pins are NOT jammed: the spring is the keeper", False),
 ("shake from 0.05 mm hole play", f"{0.05 / L['r_pin'] * R_COM:.2f} mm at the finger's centre of mass", f"{M_F * W2 * 0.05 / L['r_pin'] * R_COM * 1e-3:.1f} N", True),
 ("hub stack height", f"{stack:.1f} mm (today {H.Z['hub_hi'] - H.Z['hub_lo']:.1f})", f"top at z {H.Z['hub_lo'] + stack:.1f}, shaft tip z {H.Z['shaft_tip']:.1f}: {H.Z['shaft_tip'] - H.Z['hub_lo'] - stack:.1f} mm left for the nut", H.Z["shaft_tip"] - H.Z["hub_lo"] - stack > 4.0),
 ("added spinning mass (est.)", f"4 x ({L['shoe_g']} + {L['staple_g']}) g", f"{4 * (L['shoe_g'] + L['staple_g']):.0f} g, four identical sets: balanced", True),
] + [(f"imbalance if spun in '{k}'", "", f"{imb(v):.0f} N", imb(v) < 5) for k, v in POSES.items()]
json.dump(dict(layout=L, checks=[dict(what=a, value=b, result=c, ok=bool(d)) for a, b, c, d in chk], poses=POSES), open("hand3_root_lock_axial.json", "w"), indent=1, default=float)

fig = plt.figure(figsize=(20, 14), facecolor="white")
fig.suptitle("hand3 travelling roots: axial lock, two pins per root, one each side of its centre line (to scale, mm)", fontsize=17, fontweight="bold", y=0.99)
fig.text(0.5, 0.962, "Grey = fixed to the shaft (base disc with 24 holes and a rim lip).  Colours = the four shoes that carry the roots.  Red = lock pins (dia 3 steel dowels).  Orange = parked fork, does not spin.", ha="center", fontsize=10.5)
COL = ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"]; RED = "#d62728"
def plan(ax, az, title, lifted=None):
    ax.set_aspect("equal"); ax.set_xlim(-50, 50); ax.set_ylim(-44, 44); ax.set_xticks([]); ax.set_yticks([])
    ax.add_patch(Circle((0, 0), L["lip_r"][1], fc="#e6e8eb", ec="#555")); ax.add_patch(Wedge((0, 0), L["lip_r"][1], 0, 360, width=1.0, fc="#9aa1aa", ec="none"))
    off = az[0] % 15
    for a in np.arange(off - 15, 360, 15) + 0.0: ax.add_patch(Circle((L["r_pin"] * np.cos(np.radians(a)), L["r_pin"] * np.sin(np.radians(a))), L["d_hole"] / 2, fc="white", ec="#555", lw=0.6))
    for i, a in enumerate(az):
        u = np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]); n = np.array([-u[1], u[0]])
        ax.add_patch(Polygon([u * 19 + n * 9, u * 50 + n * 8, u * 50 - n * 8, u * 19 - n * 9], fc=COL[i], ec="black", lw=0.5, alpha=0.25))
        ax.add_patch(Wedge((0, 0), L["shoe_r"][1], a - L["shoe_half"], a + L["shoe_half"], width=L["shoe_r"][1] - L["shoe_r"][0], fc=COL[i], ec="black", lw=0.8, alpha=0.8))
        ax.add_patch(Circle(u * L["r_lag"], L["d_lag"] / 2, fc="white", ec="black", lw=0.9))
        for s in (-1, 1):
            b = np.radians(a + s * L["pin_daz"]); ax.add_patch(Circle((L["r_pin"] * np.cos(b), L["r_pin"] * np.sin(b)), 1.5, fc="white" if lifted == i else RED, ec=RED, lw=1.4))
        ax.add_patch(Polygon([u * 24.5 + n * 2.5, u * L["tab_r"] + n * 2.5, u * L["tab_r"] - n * 2.5, u * 24.5 - n * 2.5], fc="#333", ec="black", lw=0.5))
        if lifted == i:
            ax.add_patch(Polygon([u * 26.5 + n * 5, u * 40 + n * 5, u * 40 - n * 5, u * 26.5 - n * 5, u * 26.5 - n * 3, u * 30 - n * 3, u * 30 + n * 3, u * 26.5 + n * 3], fc="#f0a030", ec="black", lw=0.7))
    ax.add_patch(Circle((0, 0), 7.0, fc="#b9bec5", ec="#555")); ax.add_patch(Circle((0, 0), 2.5, fc="#444"))
    ax.plot([28, 48], [-41, -41], color="black", lw=2); ax.text(38, -39.5, "20 mm", ha="center", fontsize=8)
    ax.set_title(title, fontsize=11.5, fontweight="bold")
names = list(POSES)
plan(fig.add_axes([0.008, 0.52, 0.235, 0.41]), POSES[names[0]], "Flight cross: all 8 pins down (the only pose allowed to spin)")
plan(fig.add_axes([0.25, 0.52, 0.235, 0.41]), POSES[names[2]], "3 + 1 thumb grasp: fork lifts one pin pair, hub turns under it", lifted=3)
plan(fig.add_axes([0.008, 0.07, 0.235, 0.41]), POSES[names[1]], "2 + 2 pinch")
plan(fig.add_axes([0.25, 0.07, 0.235, 0.41]), POSES[names[3]], "All four on one side (roots 60 deg apart)")
# side section through one pin, r-z to scale
ax = fig.add_axes([0.495, 0.52, 0.5, 0.41]); ax.set_aspect("equal"); z = H.Z["hub_lo"]; d, lt, g = L["disc_t"], L["leaf_t"], L["grip_t"]
def R(r0, r1, a, b, **kw): ax.add_patch(Rectangle((r0, a), r1 - r0, b - a, **kw))
R(0, 17, 80, 89, fc="#555a63", ec="black"); ax.text(8.5, 84.5, "motor bell", color="white", ha="center", va="center", fontsize=8.5); R(0, 2.5, 89, H.Z["shaft_tip"], fc="#333", ec="none")
R(21.9, 23.9, 80, 87, fc="#39a0e0", ec="black", alpha=.8); R(24.3, 26.3, 84, 87, fc="#e07b39", ec="black", alpha=.8)
R(2.5, 25.5, z, z + d, fc="#b9bec5", ec="black"); R(24.5, 25.5, z + d, z + d + L["lip_h"], fc="#9aa1aa", ec="black")
zt = z + d + 2 * lt + g; R(2.5, 25.5, zt, zt + d, fc="#b9bec5", ec="black"); R(24.5, 25.5, zt - L["lip_h"], zt, fc="#9aa1aa", ec="black"); R(2.5, 7, z + d, zt, fc="#b9bec5", ec="black")
R(13, 24.5, z + d, z + d + lt, fc=COL[0], ec="black", alpha=.85); R(13, 24.5, zt - lt, zt, fc=COL[0], ec="black", alpha=.85)
R(L["r_lag"] - 2.5, L["r_lag"] + 2.5, z + d, zt, fc="white", ec="black"); R(22.6, 50, z + d + lt + 0.3, zt - lt - 0.3, fc=COL[0], ec="black", alpha=.25)
R(L["r_pin"] - 1.5, L["r_pin"] + 1.5, z + 0.2, z + d + lt + 1.6, fc=RED, ec="black")
R(L["r_pin"] - 1.5, 28.5, z + d + lt + 0.2, z + d + lt + 1.6, fc="#333", ec="black")
R(27.0, 40, z + d + lt - 1.4, z + d + lt + 0.1, fc="#f0a030", ec="black"); R(37, 38.5, 82, z + d + lt - 1.4, fc="#f0a030", ec="black"); R(36.3, 40.3, 80, 82, fc="#cfcfc8", ec="black")
ax.annotate("", (33, z + d + lt + 4.5), (33, z + d + lt + 0.3), arrowprops=dict(arrowstyle="->", lw=1.6)); ax.text(34, z + d + lt + 3.2, f"lift {L['lift']} mm", fontsize=8.5)
for txt, xy, xt in (("base disc: 24 holes on r 17.7", (10, z + 0.6), (-3, 77.5)), ("rim lips take the finger's pull (826 N)", (25, z + d + 0.7), (29, 107.5)), ("shoe leaves", (15, zt - 0.7), (8, 110.5)),
                    ("swing pin", (20, zt - 2), (16, 108.3)), ("lock pin, drawn in the section plane (really 15 deg to each side)", (L["r_pin"], z + 3.5), (-3, 113)), ("bridge + tab joining the two pins", (27, z + d + lt + 1.4), (41, 100.5)),
                    ("fork on the tie ring: lifts the tab, holds the shoe", (39, z + d + lt - 0.7), (41, 90.5)), ("pull rings", (25, 86), (41, 85)), ("finger root", (45, 97.5), (45, 104.5)), ("top disc", (10, zt + 1.2), (-3, 107.5))):
    ax.annotate(txt, xy, xt, fontsize=8.5, arrowprops=dict(arrowstyle="-", lw=0.7), va="center")
ax.set_xlim(-4, 72); ax.set_ylim(76, 115); ax.set_xticks([]); ax.set_yticks([]); ax.set_title("Side section, to scale (radius across, height up)", fontsize=11.5, fontweight="bold")
tax = fig.add_axes([0.495, 0.03, 0.5, 0.455]); tax.axis("off"); rows = [("Check", "Value", "Result", "")] + [(a, b, c, "ok" if dd else "OPEN") for a, b, c, dd in chk]
Wc = [0.33, 0.24, 0.36, 0.07]; X = np.cumsum([0] + Wc[:-1]); n = len(rows)
for i, r_ in enumerate(rows):
    y = 1 - (i + 1) / n
    for j, c in enumerate(r_):
        bad = i and r_[3] == "OPEN"
        tax.add_patch(Rectangle((X[j], y), Wc[j], 1 / n, fc="#eeeeee" if i == 0 else ("#fde8e8" if bad else "white"), ec="#bbb", transform=tax.transAxes))
        tax.text(X[j] + 0.004, y + 0.5 / n, "\n".join(textwrap.wrap(c, [36, 27, 44, 6][j])), transform=tax.transAxes, va="center", fontsize=8.8, fontweight="bold" if (i == 0 or j == 0) else "normal")
fig.text(0.008, 0.02, "Sequence to move one root: rotor parked with that root's tab in the fork -> fork lifts the pin pair 2.9 mm -> a roller turns the hub in 15 deg steps under the held shoe -> fork lowers, pins drop into the new holes -> fork backs off.",
         fontsize=10.3, fontweight="bold")
fig.savefig("hand3_root_lock_axial.png", dpi=85)
for a, b, c, dd in chk: print("ok  " if dd else "OPEN", a, "|", b, "|", c)
