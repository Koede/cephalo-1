"""hand3 option F (roots travel round the hub): layout study of the root lock, 2026-11-19.
Rule (user): joints stay steady under rotation -> nothing on the rotor is held by a servo or by friction while it spins.
User's idea: little holes that let pins slide out through them and couple the parts crosswise.
Numbers are first estimates unless read from hand3_design / hand3_hubc.  Writes hand3_root_lock_study.png / .json"""
import os, sys, json, textwrap
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Wedge, Polygon
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_hubc as C
os.chdir(HERE)
ST = json.load(open("hand3_indep_fingers_study.json"))["common"]
RPM = 12500.0; W2 = (RPM * 2 * np.pi / 60) ** 2
M_F, R_COM, F_CF = ST["finger_g"] * 1e-3, ST["finger_com_r"] * 1e-3, ST["finger_cf_N"]
RL, RP = H.R_LAG, H.HUB["plate_d"] / 2
T_MOTOR = 0.62                                    # N m, from the options study
STEEL = 7.85e-3
def pin(d, L): return np.pi * d * d / 4 * L * STEEL                      # g
def min_r(step, d, web): return 360 / step * (d + web) / (2 * np.pi)      # smallest hole-circle radius
def row(key, name, r_lock, d, L, step, radial, cf_seats, release, grow, verdict, note, tint=False):
    m = pin(d, L); cf = m * 1e-3 * W2 * r_lock * 1e-3 if radial else 0.0
    ft = T_MOTOR / 4 / (r_lock * 1e-3)                                    # N per root from full motor torque
    tau = ft / (2 * np.pi * d * d / 4)                                    # double shear, MPa
    play = 0.05 / r_lock * R_COM * 1e3                                    # mm at the finger's centre of mass for 0.05 mm hole clearance
    imb = M_F * W2 * play * 1e-3
    rmin = min_r(step, d, 1.5)
    return dict(key=key, name=name, r_lock=r_lock, pin=f"dia {d:g} x {L:g} steel dowel, {m:.2f} g", step=step, r_min=round(rmin, 1), fits=bool(r_lock >= rmin - 1e-6),
                seat_N=round(cf, 1), cf_seats=cf_seats, side_N=round(ft, 1), shear_MPa=round(tau, 2), imbalance_N=round(imb, 1), release=release, grow=grow,
                verdict=verdict, note=note, tint=tint)
opts = [
 row("L1", "Radial pins out through a rim wall with holes (your idea)", 24.0, 3.0, 10.0, 15.0, True, "yes: harder with speed",
     "pin tips stand proud of the rim; a parked pusher on the tie ring presses one in (2 N spring)", "hub about +5 mm tall (est.), dia unchanged",
     "recommended", "tip proud of the wall = 'locked' signal for a sensor", True),
 row("L2", "Radial pins from the centre spindle into the carrier bores", 10.5, 3.0, 8.0, 30.0, True, "yes, but half the force",
     "no access: the pins are buried under the blades at r 10", "4 carrier rings 5 mm each: hub +10 mm (est.)",
     "works only in 30 deg steps", "15 deg holes do not fit at r 10.5 (need r 17.2)"),
 row("L3", "Axial pins (parallel to the shaft) through the plates", 16.0, 3.0, 12.0, 15.0, False, "no: spring only",
     "lift plate from below", "hub +3 mm (est.)", "fails the rule", "would also carry the 826 N pull, but nothing but a spring keeps it in"),
 row("L4", "Face teeth (crown ring) pressed together by a spring", 18.0, 3.0, 0.0, 15.0, False, "no: spring + friction",
     "lift each carrier 1.5 mm", "hub +4 mm (est.)", "fails the rule", "teeth cam apart under side load unless the spring wins"),
 row("L5", "L1 with dia 2 pins (no hub growth)", 24.0, 2.0, 8.0, 15.0, True, "yes, weaker",
     "as L1", "none (rim 3.6 mm tall fits above the pull rings)", "fallback", "pin 0.2 g: seat force low"),
 row("L6", "L1 plus a V-seat under each root", 24.0, 3.0, 10.0, 15.0, True, "yes, twice over",
     "as L1", "as L1", "later upgrade", "the finger's own 826 N pull wedges the root into a V: takes out hole play"),
]
out = dict(rule="nothing on the rotor is held by a servo or friction while it spins", rpm=RPM, finger_cf_N=F_CF, motor_torque_Nm=T_MOTOR,
           min_hole_circle_r=dict(step15_d3=round(min_r(15, 3, 1.5), 1), step15_d2=round(min_r(15, 2, 1.5), 1), step30_d3=round(min_r(30, 3, 1.5), 1)),
           accidental_full_pull=dict(shear_d3_MPa=round(F_CF / (2 * np.pi * 2.25), 1), bearing_d3_in_3mm_wall_MPa=round(F_CF / 9, 1)),
           hub=dict(plate_r=RP, r_lag=RL, pull_ring_top_z=C.Z_OUT[1], hub_lo_z=H.Z["hub_lo"], blade_z=H.Z["blade"], shaft_tip_z=H.Z["shaft_tip"]), options=opts)
json.dump(out, open("hand3_root_lock_study.json", "w"), indent=1, default=float)

fig = plt.figure(figsize=(20, 14.5), facecolor="white")
fig.suptitle("hand3, roots that travel round the hub: how is each root locked so it stays steady while spinning?", fontsize=17, fontweight="bold", y=0.99)
fig.text(0.5, 0.962, f"Rule: nothing on the rotor is held by a servo or friction while it spins.   At {RPM:.0f} rpm each finger pulls {F_CF:.0f} N outward; full motor torque puts "
         f"{T_MOTOR / 4 / 0.024:.1f} N sideways on each root at r 24.   Plan views to scale (mm).  Grey = fixed to the shaft, colours = the four root carriers, red = lock pins.", ha="center", fontsize=10.5)
COL = ["#2f6fbf", "#c0392b", "#16a085", "#8e44ad"]; RED = "#d62728"
def plan(ax, o, az, title_col="black"):
    ax.set_aspect("equal"); ax.set_xlim(-46, 46); ax.set_ylim(-40, 40); ax.set_xticks([]); ax.set_yticks([])
    ax.add_patch(Circle((0, 0), RP, fc="#e6e8eb", ec="#777"))
    k = o["key"]; rl = o["r_lock"]
    if k in ("L1", "L5", "L6"):
        ax.add_patch(Wedge((0, 0), RP, 0, 360, width=3.0, fc="#b9bec5", ec="#555"))
        for a in np.arange(0, 360, 15): ax.add_patch(Circle((rl * np.cos(np.radians(a)), rl * np.sin(np.radians(a))), 1.5 if k != "L5" else 1.0, fc="white", ec="#555", lw=0.6))
    if k == "L2":
        ax.add_patch(Circle((0, 0), 9.0, fc="#b9bec5", ec="#555"))
        for a in np.arange(0, 360, 30): ax.add_patch(Circle((rl * np.cos(np.radians(a)), rl * np.sin(np.radians(a))), 1.5, fc="white", ec="#555", lw=0.6))
    if k == "L3":
        for a in np.arange(0, 360, 15): ax.add_patch(Circle((rl * np.cos(np.radians(a)), rl * np.sin(np.radians(a))), 1.5, fc="white", ec="#555", lw=0.6))
    if k == "L4":
        for a in np.arange(0, 360, 15): ax.add_patch(Wedge((0, 0), rl + 2, a, a + 7.5, width=4, fc="#b9bec5", ec="none"))
    for i, a in enumerate(az):
        u = np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]); n = np.array([-u[1], u[0]])
        ax.add_patch(Polygon([n * 5, u * 21 + n * 5, u * 21 - n * 5, -n * 5], fc=COL[i], ec="black", lw=0.6, alpha=0.75))          # carrier arm
        ax.add_patch(Polygon([u * 19 + n * 9, u * 46 + n * 8, u * 46 - n * 8, u * 19 - n * 9], fc=COL[i], ec="black", lw=0.6, alpha=0.35))   # finger root
        ax.add_patch(Circle(u * RL, 2.5, fc="white", ec="black", lw=0.8))
        if k in ("L1", "L5", "L6"): ax.plot(*np.c_[u * 17.5, u * 27.0], color=RED, lw=3.2 if k != "L5" else 2.2, solid_capstyle="butt")
        if k == "L2": ax.plot(*np.c_[u * 5, u * 12.5], color=RED, lw=3.2, solid_capstyle="butt")
        if k == "L3": ax.add_patch(Circle(u * rl, 1.5, fc=RED, ec="black", lw=0.5))
        if k == "L6": ax.add_patch(Polygon([u * 14 + n * 3.5, u * 17, u * 14 - n * 3.5], fc="#222"))
    ax.add_patch(Circle((0, 0), 2.5, fc="#444"))
    ax.plot([24, 44], [-37, -37], color="black", lw=2); ax.text(34, -35.5, "20 mm", ha="center", fontsize=7.5)
    ax.set_title("\n".join(textwrap.wrap(f"{k}  {o['name']}", 44)), fontsize=11, fontweight="bold", color=title_col)
POSE = [(45, 135, 225, 315), (45, 135, 225, 315), (45, 135, 225, 315), (45, 135, 225, 315), (60, 120, 270, 195), (0, 45, 90, 270)]
for i, o in enumerate(opts):
    ax = fig.add_axes([0.008 + (i % 3) * 0.215, 0.665 - (i // 3) * 0.3, 0.205, 0.255]); plan(ax, o, POSE[i], "#1a7f37" if o["tint"] else "black")
    ax.text(-45, 36.5, ["flight cross", "flight cross", "flight cross", "flight cross", "2 + 2 pinch", "3 + 1 thumb grasp"][i], fontsize=8.5, va="top")
# side section of L1, to scale in r-z
ax = fig.add_axes([0.665, 0.385, 0.33, 0.55]); ax.set_aspect("equal"); z0 = 91.0
def R(r0, r1, a, b, **kw): ax.add_patch(Rectangle((r0, a), r1 - r0, b - a, **kw))
R(0, 17, 66, 89, fc="#555a63", ec="black"); ax.text(8.5, 78, "motor bell\n(spins)", color="white", ha="center", fontsize=8.5)
R(0, 2.5, 89, 112, fc="#333", ec="none")
R(21.9, 23.9, C.Z_IN[0], 87, fc="#39a0e0", ec="black", alpha=0.8); R(24.3, 26.3, 84, 87, fc="#e07b39", ec="black", alpha=0.8); ax.text(30, 85.5, "pull rings", fontsize=8.5, va="center")
R(19.5, 21.5, 66, 84.5, fc="#c9cfd6", ec="black"); ax.text(20.5, 75, "cage", rotation=90, ha="center", va="center", fontsize=8)
R(2.5, RP, 89.5, 91, fc="#b9bec5", ec="black"); R(RP - 3, RP, 91, 96, fc="#b9bec5", ec="black")                 # base disc + rim wall
R(2.5, 7, 91, 108, fc="#b9bec5", ec="black")
R(7, 21.5, 91.3, 93.3, fc=COL[0], ec="black", alpha=0.8); R(7, 21.5, 101.2, 103.2, fc=COL[0], ec="black", alpha=0.8)   # one carrier: two leaves
R(16.5, 22.3, 91.3, 96, fc=COL[0], ec="black", alpha=0.8)                                                           # pin block
R(17.5, 27.0, 92.2, 95.2, fc=RED, ec="black"); ax.plot([15.5, 17.5], [93.7, 93.7], color="black", lw=1, ls=":")
R(RL - 2.5, RL + 2.5, 96.2, 104.5, fc="white", ec="black"); R(19, 46, 96.6, 100.9, fc=COL[0], ec="black", alpha=0.35)
R(33, 44, 92.4, 95.0, fc="#f0a030", ec="black"); ax.annotate("", (27.4, 93.7), (33, 93.7), arrowprops=dict(arrowstyle="->", lw=1.5))
R(36.3, 40.3, 80, 82, fc="#cfcfc8", ec="black"); R(37, 38.5, 82, 92.4, fc="#f0a030", ec="black")
for txt, xy, xt in (("rim wall with 24 holes", (24, 96), (27, 107.5)), ("lock pin dia 3, thrown outward by the spin", (22, 95.2), (9, 115.5)), ("carrier leaves (one of four pairs)", (12, 103.2), (9, 112)),
                    ("finger root on its swing pin", (36, 100.9), (30, 104.5)), ("parked pusher (does not spin):\npresses the pin in to release", (40, 92.4), (30, 88.5)),
                    ("tie ring", (40, 81), (42, 76)), ("base disc on the shaft", (12, 89.6), (26, 71))):
    ax.annotate(txt, xy, xt, fontsize=8.5, arrowprops=dict(arrowstyle="-", lw=0.7), va="center")
ax.set_xlim(-2, 58); ax.set_ylim(64, 118); ax.set_xticks([]); ax.set_yticks([])
ax.set_title("L1 side section, to scale (radius across, height up; one carrier shown)", fontsize=11.5, fontweight="bold", color="#1a7f37")
hdr = ("", "Lock pin", "Smallest step that fits (dia + 1.5 web)", "Does the spin seat it?", f"Seat force at {RPM:.0f} rpm", "Side load / pin shear at full torque", "Shake from 0.05 mm hole play", "Release when parked", "Hub change", "Verdict")
rows = [hdr] + [(f"{o['key']}  {o['name']}", o["pin"] if o["key"] != "L4" else "none (teeth)", f"{o['step']:g} deg: needs r {o['r_min']} (has {o['r_lock']:g})" + ("" if o["fits"] else " NO"),
                 o["cf_seats"], f"{o['seat_N']} N", f"{o['side_N']} N / {o['shear_MPa']} MPa", f"{o['imbalance_N']} N", o["release"], o["grow"], o["verdict"] + ": " + o["note"]) for o in opts]
tax = fig.add_axes([0.008, 0.006, 0.984, 0.325]); tax.axis("off"); Wc = [0.14, 0.095, 0.085, 0.075, 0.06, 0.08, 0.065, 0.135, 0.105, 0.16]; X = np.cumsum([0] + Wc[:-1]); wr = [27, 18, 16, 14, 11, 15, 12, 26, 20, 31]; n = len(rows)
for i, r_ in enumerate(rows):
    y = 1 - (i + 1) / n
    for j, c in enumerate(r_):
        tax.add_patch(Rectangle((X[j], y), Wc[j], 1 / n, fc="#dff3e3" if (i and opts[i - 1]["tint"]) else ("#eeeeee" if i == 0 else "white"), ec="#bbb", transform=tax.transAxes))
        tax.text(X[j] + 0.003, y + 0.5 / n, "\n".join(textwrap.wrap(c, wr[j])), transform=tax.transAxes, va="center", fontsize=8.8, fontweight="bold" if (i == 0 or j == 0) else "normal")
fig.text(0.008, 0.338, f"Recommended: L1.  If the whole {F_CF:.0f} N finger pull ever landed on one dia 3 pin: shear {out['accidental_full_pull']['shear_d3_MPa']} MPa (fine for steel), "
         f"hole-edge bearing {out['accidental_full_pull']['bearing_d3_in_3mm_wall_MPa']} MPa in a 3 mm wall (too much for printed PA-CF: steel bushings or an aluminium rim).", fontsize=10.3, fontweight="bold")
fig.savefig("hand3_root_lock_study.png", dpi=85)
for o in opts: print(o["key"], o["seat_N"], "N seat |", o["side_N"], "N side |", o["imbalance_N"], "N shake | r_min", o["r_min"], o["fits"])
print(out["accidental_full_pull"], out["min_hole_circle_r"])
