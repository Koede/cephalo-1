"""hand3 hub v3: the parked station that moves a finger root (options study, 2026-11-19).
The station must, with the rotor parked: (1) turn the rotor so a chosen root faces it, (2) free that root's two latch pins
(0.5 mm inward, then 2.1 mm up), (3) hold that root and its bead while the hub turns 30 deg steps under it, (4) drop the pins,
(5) read all eight pins down and the four root positions before spin-up.  Nothing of it may be in the rotor's path in flight.
Access: r-z section through the station azimuth from the CAD, plus the clearance numbers.  Part data from maker pages; loads est.
Writes hand3_station_study.png / .json"""
import os, sys, json, textwrap
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle, FancyArrowPatch
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_cad as C, hand3_hubc as HC, hand3_hub_v3 as V3
os.chdir(HERE)
V = V3.V; HUB = json.load(open("hand3e/hand3_hubc.json"))
AZ = 90.0                                                                     # station azimuth: clear of all four yoke lips
lips = [(HC.HALVES[k][0] - HC.LIP_HALF, HC.HALVES[k][0] + HC.LIP_HALF) for k in HC.HALVES]
lip_gap = min(min(abs((AZ - a0 + 180) % 360 - 180), abs((AZ - a1 + 180) % 360 - 180)) for a0, a1 in lips)
# --- loads ---
T_ROT = 0.05                                                                  # N m to turn the parked rotor: SE 2816 cogging 0.03 (est.) + drag
R_CAGE = HC.R_CAGE[1]; ROLL_D = 8.0; MU = 0.6
F_t = T_ROT / (R_CAGE * 1e-3); F_n = F_t / MU
N20 = dict(name="Pololu 298:1 Micro Metal Gearmotor HPCB 6V (#3069)", stall_kgcm=3.4, rpm=110, stall_A=1.5, g=10.5)
roll_T = F_t * ROLL_D / 2 * 1e-3; n20_T = N20["stall_kgcm"] * 0.0981
rotor_rpm = N20["rpm"] * ROLL_D / (2 * R_CAGE); t30 = 30 / 360 / rotor_rpm * 60
SG = dict(name="TowerPro SG92R micro servo (Adafruit #169)", stall_kgcm=2.5, g=9.0)
F_pins = 2 * 3.0 + 2.0                                                        # two pin springs + friction (est.)
sg_T = SG["stall_kgcm"] * 0.0981; horn = 10.0; sg_F = sg_T / (horn * 1e-3)
QTR = dict(name="Pololu QTR-1A reflectance sensor (#2458)", range_mm=(3, 6), g=0.2)
SMA = dict(name="Muscle Wire 0.005 in (SparkFun COM-11899)", pull_N=0.49 * 4.448, mA=200)
n_sma = int(np.ceil(F_pins / SMA["pull_N"]))
# ring flange under S1 (both servos on the base ring): bending at the band, per lip
spool_F = HUB["servo"]["stall"] / (HUB["spool_r"] * 1e-3) if "spool_r" in HUB else HUB["servo"]["stall"] / 4e-3
ring_F = 2 * HUB["base"]["F_stall_finger"] * 4                               # both servos, stall, at the ring (1:1 before the tackle)
per_lip = ring_F / 4; arm = (HC.R_LIP_OUT[0] + 0.6 + HC.R_FL_OUT) / 2 - HC.R_OUT[1]; L_lip = np.radians(2 * HC.LIP_HALF) * (HC.R_OUT[1] + arm)
def flange_MPa(t): return per_lip * arm / (L_lip * t * t / 6)
flange = dict(ring_force_stall_N=round(ring_F), per_lip_N=round(per_lip), arm_mm=round(arm, 2), MPa_t2=round(flange_MPa(2.0), 1), MPa_t3=round(flange_MPa(3.0), 1),
              MPa_t2_with_slots=round(flange_MPa(2.0) / 0.5, 1))
# --- free space at the station: every part sliced in the plane of the station azimuth, rasterised in (r, z) at 0.1 mm ---
from matplotlib.path import Path
def section_polys(m):
    s = m.rotate([0, 0, -AZ]).rotate([-90, 0, 0]); return [np.asarray(p) for p in s.slice(0.0).to_polygons() if len(p) > 2]
def occupancy(dz_ring=0.0):
    Pp = HC._parts(); hw_ = HC.hardware(); parts = [Pp["cage"][0], (Pp["ring_out"][0] + V3.ring_rail()).translate([0, 0, -dz_ring]), Pp["tie_ring"][0], V3.base_disc(), V3.top_disc(),
             V3.root_block(AZ), V3.bead(AZ).translate([0, 0, -dz_ring]), V3.spring_box(AZ, 0)] + list(V3.finger(AZ).values()) + [V3.latch_pin(AZ, s) for s in (-1, 1)]
    rr = np.arange(20.0, 50.0, 0.1) + 0.05; zz = np.arange(80.0, 100.0, 0.1) + 0.05; Rg, Zg = np.meshgrid(rr, zz, indexing="ij"); pts = np.c_[Rg.ravel(), Zg.ravel()]
    occ = np.zeros(len(pts), bool)
    for m in parts:
        inside = np.zeros(len(pts), bool)
        for p in section_polys(m): inside ^= Path(p).contains_points(pts)           # even-odd within a part: its holes stay free
        occ |= inside                                                                # parts overlap only where they touch
    return rr, zz, occ.reshape(len(rr), len(zz))
def blade_band(rr, zz, occ, r0, r1, z_lo=82.6):
    """Largest z interval free for every r in [r0, r1] (a blade sliding radially in to r0)."""
    sel = (rr >= r0) & (rr <= r1); free = ~occ[sel].any(axis=0) & (zz > z_lo)
    best, cur = (0, None), None
    for i, f in enumerate(free):
        if f: cur = i if cur is None else cur
        if (not f or i == len(free) - 1) and cur is not None:
            j = i if f else i - 1
            if zz[j] - zz[cur] > best[0]: best = (zz[j] - zz[cur] + 0.1, (zz[cur] - 0.05, zz[j] + 0.05))
            cur = None
    return best
RR, ZZ, OCC = occupancy()
band_w, band_z = blade_band(RR, ZZ, OCC, V["bead"]["r"][1] + 0.2, 46.0)
band = dict(r_in=V["bead"]["r"][1] + 0.2, z=(round(band_z[0], 2), round(band_z[1], 2)), height=round(band_w, 2))

opts = [
 dict(key="A", name="Slide in the free band under the root + a rocker on each root",
      how="A static blade slides in radially at the station, under the finger and above the bead. It pushes the two rocker tips of the root standing there: the rocker pulls both pins 0.5 mm in and 2.1 mm up, and the blade's notch holds the root and its bead while the roller turns the hub.",
      rotor="4 rockers on the roots, est. 0.4 g each",
      static=f"SG92R servo on the slide ({sg_F:.0f} N at a 10 mm horn vs {F_pins:.0f} N needed), N20 roller on the cage, QTR-1A",
      access=f"free band z {band['z'][0]:.1f} to {band['z'][1]:.1f} ({band['height']:.1f} mm) from r {band['r_in']:.1f} outward, under the finger, above the ring flange",
      mass="+22 g static, +1.6 g rotor (est.)", risk="new rocker in a crowded root block", verdict="recommended", tint=True),
 dict(key="B", name="Prongs rise from below through slots in the ring flange",
      how="Two prongs climb at r 27.5 through slots in the spinning flange (lined up when parked), ramp the tabs inward, lift them and hold the root.",
      rotor="24 to 36 slots in the pull-ring flange", static="SG92R on a vertical slide, N20 roller, QTR-1A",
      access="free column at the station from z 60 to the tabs (yokes, idlers, cords all elsewhere)",
      mass="+22 g static (est.)", risk=f"flange bending at stall {flange['MPa_t2']} MPa now, {flange['MPa_t2_with_slots']} MPa with slots (PA-CF)", verdict="rejected (flange)"),
 dict(key="C", name="Contacts touch the parked root; shape-memory wire lifts the pins",
      how="Pogo pins on the static side touch two pads on the root at the station; a Flexinol / Muscle Wire loop in the root block pulls the pins in and up. Hold by a fork on the bead.",
      rotor=f"{n_sma} x 0.005 in wires per root ({SMA['pull_N']:.1f} N each at {SMA['mA']} mA), pads, est. 0.6 g per root", static="pogo block + fork servo, N20 roller, QTR-1A",
      access="pads must sit in the same band as A's rocker tips", mass="+18 g static, +2.4 g rotor (est.)",
      risk="needs the same access as A plus wiring on every root; wire fatigue", verdict="no better than A"),
 dict(key="D", name="Another Cephalo hand moves the roots",
      how="A neighbouring arm's hand grips the root, presses the tabs and turns it; the rotor is held by its own pins on the other roots.",
      rotor="tabs stay as they are", static="none on this hand", access="from outside, at the tabs and fingers", mass="0 g",
      risk="needs a second hand within reach and fine control; slow", verdict="fallback / field repair"),
 dict(key="E", name="Set the roots by hand between flights",
      how="The user lifts the two tabs with a fingernail and slides the root; the pose is fixed for the next flight.",
      rotor="none", static="none", access="by hand", mass="0 g", risk="no pose change during a mission", verdict="day-one option"),
 dict(key="F", name="Flight motor turns the hub, static fork holds the root",
      how="The ESC creeps the rotor round while a fork holds the lifted root.",
      rotor="none", static="fork + lifter (same access problem as A)", access="as A",
      mass="+12 g", risk="sensorless ESC cannot creep 30 deg reliably at standstill", verdict="rejected (ESC)"),
]
common = dict(roller=dict(part=N20["name"], contact=f"rubber tyre dia {ROLL_D:g} on the cage outside (r {R_CAGE}), z 62 to 78, now free with the inner ring gone",
                          needed_Nm_at_rotor=T_ROT, tangential_N=round(F_t, 2), press_N=round(F_n, 1), roller_Nm=round(roll_T, 4), N20_stall_Nm=round(n20_T, 3),
                          margin_at_25pct_of_stall=round(0.25 * n20_T / roll_T, 1), rotor_rpm=round(rotor_rpm, 1), seconds_per_30deg=round(t30, 2)),
              servo=dict(part=SG["name"], force_at_10mm_N=round(sg_F, 1), need_N=F_pins), sensor=dict(part=QTR["name"], range_mm=QTR["range_mm"]),
              station_az=AZ, gap_to_nearest_lip_deg=round(lip_gap, 1), service_band=band, flange_S1=flange)
json.dump(dict(common=common, options=opts), open("hand3_station_study.json", "w"), indent=1, default=float)

fig = plt.figure(figsize=(20, 14.5), facecolor="white")
fig.suptitle("hand3 hub v3: the parked station that moves a finger root (options; section at the station azimuth, to scale, mm)", fontsize=16, fontweight="bold", y=0.995)
fig.text(0.5, 0.962, "Needs: turn the rotor to the root, free its two pins (0.5 mm in, 2.1 mm up), hold the root + bead while the hub turns 30 deg, drop the pins, check all 8 pins before spin-up.  "
         "Nothing may be in the rotor's path in flight.  Common to A to C: a roller on the cage + one reflectance sensor.", ha="center", fontsize=10.3)
def sec(ax, m, fc, alpha=1.0, z=1):
    s = m.rotate([0, 0, -AZ]).rotate([-90, 0, 0])
    for poly in s.slice(0.0).to_polygons():
        p = np.asarray(poly)
        if len(p) > 2: ax.add_patch(Polygon(p, closed=True, fc=fc, ec="black", lw=0.4, alpha=alpha, zorder=z))
P = HC._parts(); hw = HC.hardware()
def base(ax, dz_ring=0.0):
    sec(ax, P["cage"][0], "#c9cfd6"); sec(ax, (P["ring_out"][0] + V3.ring_rail()).translate([0, 0, -dz_ring]), "#e07b39", 0.9); sec(ax, hw["motor_base"], "#555a63")
    sec(ax, HC.frame(), "#e9e9e4"); sec(ax, P["tie_ring"][0], "#cfcfc8"); sec(ax, V3.base_disc(), "#b8bec6", z=2); sec(ax, V3.top_disc(), "#b8bec6", z=2)
    sec(ax, V3.root_block(AZ), "#2f6fbf", 0.8, z=3); sec(ax, V3.bead(AZ).translate([0, 0, -dz_ring]), "#111", z=4)
    for n, m in V3.finger(AZ).items(): sec(ax, m, "#2b2b30" if "phalanx" in n else "#c23b22", z=3)
    sec(ax, V3.spring_box(AZ, 0), "#e6c229", 0.9, z=3); sec(ax, C.parts()["heel"], "#2f2f2f")
    bell = C.cyl(17, H.Z["pack1"] + 8.5, H.Z["motor_top"]); sec(ax, bell, "#3a3f47")
    ax.set_xlim(0, 52); ax.set_ylim(56, 112); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])
ax = fig.add_axes([0.005, 0.40, 0.32, 0.54]); base(ax)
ax.imshow(np.where(OCC, np.nan, 1.0).T, origin="lower", extent=[RR[0] - 0.05, RR[-1] + 0.05, ZZ[0] - 0.05, ZZ[-1] + 0.05], cmap="Greens", vmin=0, vmax=2.2, alpha=0.55, zorder=5, interpolation="nearest")
ax.add_patch(Rectangle((band["r_in"], band["z"][0]), 50 - band["r_in"], band["z"][1] - band["z"][0], fill=False, ec="#1a7f37", lw=2, zorder=6))
ax.add_patch(Rectangle((22.0, 60.0), 2.2, 18.0, fc="#7fd37f", alpha=0.6, ec="green", lw=1, zorder=6))
for txt, xy, xt in ((f"blade band z {band['z'][0]:.1f} to {band['z'][1]:.1f} ({band['height']:.1f} mm), from r {band['r_in']:.1f} out", (40, sum(band["z"]) / 2), (12, 106)),
                    ("cage outside, free for a roller (z 60 to 78)", (23, 70), (27, 64)), ("pull ring flange (spins)", (28, 85), (32, 79.5)),
                    ("root at the station", (12, 96), (2, 110)), ("tie ring (static)", (38, 81), (40, 75.5))):
    ax.annotate(txt, xy, xt, fontsize=8.5, arrowprops=dict(arrowstyle="-", lw=0.6), va="center", zorder=7)
ax.set_title(f"Access at the station (az {AZ:.0f}, {lip_gap:.0f} deg from the nearest yoke lip): green = free", fontsize=11, fontweight="bold")
def arrow(ax, a, b, c="#d62728"): ax.add_patch(FancyArrowPatch(a, b, arrowstyle="-|>", mutation_scale=12, lw=2, color=c, zorder=8))
for i, o in enumerate(opts):
    ax = fig.add_axes([0.335 + (i % 3) * 0.222, 0.67 - (i // 3) * 0.275, 0.215, 0.255])
    if o["key"] in ("A", "C", "F"):
        base(ax); ax.set_ylim(78, 106); ax.set_xlim(10, 52)
        z0b, z1b = band["z"]
        if o["key"] == "A":
            ax.add_patch(Rectangle((band["r_in"] + 0.4, z0b + 0.2), 50 - band["r_in"], min(2.4, z1b - z0b - 0.4), fc="#f0a030", ec="black", zorder=7)); arrow(ax, (47, z1b + 2.0), (36, z1b + 2.0))
            ax.add_patch(Polygon([(band["r_in"], z0b + 0.6), (band["r_in"] + 0.8, z0b + 0.6), (band["r_in"] + 0.8, 95.0), (band["r_in"], 95.0)], fc="#d62728", ec="black", zorder=7))
            ax.text(29, 81, "rocker (red) beside the root, 7.5 mm\noff this plane: the blade pushes its tip", fontsize=8, color="#d62728")
        if o["key"] == "C":
            ax.add_patch(Rectangle((band["r_in"] + 0.4, z0b + 0.3), 20, 1.0, fc="#b8bcc4", ec="black", zorder=7)); ax.text(30, 81, "pogo block in the same band;\nwires + pads on every root", fontsize=8.5, color="#d62728")
        if o["key"] == "F": ax.text(14, 82, "sensorless ESC cannot\ncreep the rotor 30 deg", fontsize=9, color="#d62728")
    elif o["key"] == "B":
        base(ax); ax.set_ylim(56, 104); ax.set_xlim(10, 52)
        ax.add_patch(Rectangle((27.0, 62), 1.5, 32.5, fc="#f0a030", ec="black", zorder=7)); arrow(ax, (31, 70), (31, 80))
        ax.text(31, 62, f"through flange slots:\n{flange['MPa_t2_with_slots']} MPa at stall", fontsize=8.5, color="#d62728")
    else:
        ax.axis("off"); ax.text(0.5, 0.5, "no station on the hand", ha="center", va="center", fontsize=12, color="#555", transform=ax.transAxes)
    ax.set_title("\n".join(textwrap.wrap(f"{o['key']}  {o['name']}", 46)), fontsize=10.5, fontweight="bold", color="#1a7f37" if o.get("tint") else "black")
hdr = ("", "How it works", "On the rotor", "Static parts", "Access", "Mass (est.)", "Main risk", "Verdict")
rows = [hdr] + [(f"{o['key']}  {o['name']}", o["how"], o["rotor"], o["static"], o["access"], o["mass"], o["risk"], o["verdict"]) for o in opts]
tax = fig.add_axes([0.005, 0.045, 0.99, 0.31]); tax.axis("off"); Wc = [0.13, 0.25, 0.11, 0.13, 0.12, 0.07, 0.12, 0.07]; X = np.cumsum([0] + Wc[:-1]); wr = [26, 58, 24, 28, 26, 14, 26, 13]; n = len(rows)
for i, r_ in enumerate(rows):
    y = 1 - (i + 1) / n
    for j, c in enumerate(r_):
        tax.add_patch(Rectangle((X[j], y), Wc[j], 1 / n, fc="#dff3e3" if (i and opts[i - 1].get("tint")) else ("#eeeeee" if i == 0 else "white"), ec="#bbb", transform=tax.transAxes))
        tax.text(X[j] + 0.003, y + 0.5 / n, "\n".join(textwrap.wrap(c, wr[j])), transform=tax.transAxes, va="center", fontsize=8.2, fontweight="bold" if (i == 0 or j == 0) else "normal")
cm = common["roller"]
fig.text(0.005, 0.005, "\n".join(textwrap.wrap(
    f"Roller (A to C): {N20['name']}, {N20['stall_kgcm']} kg cm stall, {N20['rpm']} rpm; tyre dia {ROLL_D:g} on the cage needs {cm['tangential_N']} N ({cm['press_N']} N press at mu {MU}): "
    f"margin {cm['margin_at_25pct_of_stall']} at a quarter of stall, 30 deg in {cm['seconds_per_30deg']} s.  Sensor: {QTR['name']}, best at 3 mm.  "
    f"Side finding for S1: both servos on one ring double the flange load: {flange['MPa_t2']} MPa bending at stall with the 2 mm flange, {flange['MPa_t3']} MPa at 3 mm (rated: a third).", 230)),
    fontsize=9.5, fontweight="bold")
fig.savefig("hand3_station_study.png", dpi=85)
print(json.dumps(common, indent=1, default=float))
