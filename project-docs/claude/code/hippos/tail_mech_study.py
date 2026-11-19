"""Hippos-1 tail mechanism: design study (joint layout, length, reach, servo loads, interface parts).
Hand calculations + kinematic sampling, no CAD.  python3 tail_mech_study.py -> tail_mech.json, sheet_tail_mech.png
Frame: x forward, z up, y left (horse frame, 0.75 scale).  Tail root = SFF seat on the rump, connector axis pointing
straight back.  Servos: STS3250 (1.57 / 4.90 N m rated / stall, 74.5 g), STS3215 (0.98 / 2.94, 55 g), both 45.2 x 24.7 x 35 mm.
"""
import json, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

G = 9.81
SV = {"STS3215": (0.98, 2.94, 0.055), "STS3250": (1.57, 4.90, 0.0745)}
SERVOS = ["STS3250", "STS3250", "STS3215", "STS3215"]
ROOT = np.array([-0.310, 0.0, 0.450])        # SFF seat on the rump (chassis_v3 tail root, 0.75 scale, est.)
HIP_R = np.array([-0.150, 0.0, 0.378])       # rear hip axis (0.75 scale)
OFF = 0.060                                  # seat -> first joint axis (tail adapter + root bracket)
M_ADAPT = 0.0106 + 0.0331 + 0.016            # SFF base + tail adapter + bought (kg), rides on the tail
M_TIPSOCK = 0.009 + 0.004                    # standard SFF socket at the tail tip + pins/board (kg)
LAYOUTS = {"A  P-P-P-P (all pitch)": "PPPP", "B  Y-P-P-P (root yaw, then pitch)": "YPPP", "C  P-Y-P-Y (octopus-arm style)": "PYPY"}
LENGTHS = {"300 mm (80/70/60/90)": np.array([0.080, 0.070, 0.060, 0.090]), "400 mm (100/95/85/120)": np.array([0.100, 0.095, 0.085, 0.120])}
LIM = {"P": np.radians(110), "Y": np.radians(90)}
M_LINK_PER_M = 0.30                          # printed link + wiring, kg per metre (est.: 25 g per 80 mm link)

def rot(axis, a):
    c, s = np.cos(a), np.sin(a)
    if axis == "y": return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    if axis == "z": return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])

def fk(layout, links, q, body_pitch=0.0):
    """Joint and tip positions.  The tail starts pointing straight back (-x).  P = pitch about y, Y = yaw about the local z."""
    Rb = rot("y", -body_pitch)                                   # body pitched nose-up about the rear hip
    R = np.eye(3); p = np.array([-OFF, 0, 0.0]); pts = [np.zeros(3), p.copy()]
    for kind, L, a in zip(layout, links, q):
        R = R @ (rot("y", a) if kind == "P" else rot("z", a))
        p = p + R @ np.array([-L, 0, 0]); pts.append(p.copy())
    pts = np.array(pts)
    root = HIP_R + Rb @ (ROOT - HIP_R)
    return root + pts @ Rb.T, R

def joint_torques(layout, links, q, payload, tip_force=np.zeros(3), tip_mass=0.06, body_pitch=0.0):
    """Static torque about each joint axis (N m): gravity of links, servos, tip mass + payload, plus a tip force."""
    pts, _ = fk(layout, links, q, body_pitch)
    Rb = rot("y", -body_pitch)
    axes = []; R = np.eye(3)
    for kind, a in zip(layout, q):
        R = R @ (rot("y", a) if kind == "P" else rot("z", a))
        axes.append(Rb @ (R @ (np.array([0, 1.0, 0]) if kind == "P" else np.array([0, 0, 1.0]))))
    loads = []                                                   # (point, force)
    for k in range(4):
        mid = 0.5 * (pts[k + 1] + pts[k + 2]); loads.append((mid, np.array([0, 0, -M_LINK_PER_M * links[k] * G])))
        if k < 3: loads.append((pts[k + 2], np.array([0, 0, -SV[SERVOS[k + 1]][2] * G])))
    loads.append((pts[-1], np.array([0, 0, -(tip_mass + payload) * G]) + tip_force))
    tq = []
    for j in range(4):
        pj = pts[j + 1]; t = sum(np.cross(p - pj, f) @ axes[j] for p, f in loads[2 * j:])
        tq.append(abs(t))
    # moment through the SFF seat (bending, about the seat point)
    Mseat = np.linalg.norm(sum(np.cross(p - pts[0], f) for p, f in loads) + np.cross(pts[1] - pts[0], [0, 0, -(SV[SERVOS[0]][2] + M_ADAPT) * G]))
    return np.array(tq), Mseat

def tasks(layout):
    """Joint angles per task.  Tail starts straight back."""
    P = [i for i, k in enumerate(layout) if k == "P"]; Y = [i for i, k in enumerate(layout) if k == "Y"]
    q0 = np.zeros(4)
    out = {"held straight back, level, 0.3 kg": (q0, 0.3)}
    qs = q0.copy()
    if layout[0] == "Y": qs[0] = np.radians(90)
    elif "Y" in layout: qs[Y[0]] = np.radians(90)
    out["held out to the side, 0.3 kg"] = (qs if "Y" in layout else None, 0.3)
    qc = q0.copy()
    for i in P: qc[i] = np.radians(-50)                               # curled up over the rump
    out["curled up over the croup (rest)"] = (qc, 0.0)
    return out

def reach_cloud(layout, links, n=6000, seed=1, body_pitch=0.0):
    rng = np.random.default_rng(seed); pts = []
    for _ in range(n):
        q = np.array([rng.uniform(-LIM[k], LIM[k]) for k in layout])
        p, _ = fk(layout, links, q, body_pitch); pts.append(p[-1])
    return np.array(pts)

res = {}
for ln, links in LENGTHS.items():
    for name, lay in LAYOUTS.items():
        r = {}
        for tn, (q, pl) in tasks(lay).items():
            if q is None: r[tn] = None; continue
            tq, Ms = joint_torques(lay, links, q, pl)
            r[tn] = dict(frac_rated=[t / SV[s][0] for t, s in zip(tq, SERVOS)], M_seat=Ms)
        cl = reach_cloud(lay, links)
        r["reach"] = dict(lowest_tip_z_stand=float(cl[:, 2].min()), max_side_y=float(np.abs(cl[:, 1]).max()))
        res[(ln, name)] = r

# ground reach vs body pose: lowest tip height (B layout), stand / rear-up / crouch
poses = {"standing": 0.0, "rear-up 30 deg": np.radians(30), "rear-up 45 deg": np.radians(45)}
ground = {}
for ln, links in LENGTHS.items():
    for pn, bp in poses.items():
        cl = reach_cloud("YPPP", links, 4000, 2, bp)
        ground[(ln, pn)] = float(cl[:, 2].min())
crouch_drop = 0.180                          # hips lowered 180 mm in a crouch (kneel at 0.75 scale, est. from legs_v2)
for ln, links in LENGTHS.items():
    ground[(ln, "crouched 180 mm")] = ground[(ln, "standing")] - crouch_drop

# kickstand: tail straight down from the root as a column on its hard stops (0 servo torque), loads into the seat
def kick(links, rake_deg, body_pitch, frac=0.10, M_total=30.8):
    q = np.zeros(4); q[1] = np.radians(-(90 - rake_deg))         # B: j1 pitches the whole column down, raked back by rake_deg
    pts, _ = fk("YPPP", links, q, body_pitch)
    F = np.array([0, 0, frac * M_total * G])
    return float(np.linalg.norm(np.cross(pts[-1] - pts[0], F))), float(pts[-1][2])

kicks = {}
for ln, links in LENGTHS.items():
    for pn, bp in (("rear-up 45 deg", np.radians(45)), ("crouched", 0.0)):
        M, ztip = kick(links, 15, bp)
        if pn == "crouched": ztip -= crouch_drop
        kicks[(ln, pn)] = dict(M_seat=M, tip_z=ztip)

out = dict(layouts={f"{k[0]} | {k[1]}": v for k, v in res.items()}, ground={f"{k[0]} | {k[1]}": v for k, v in ground.items()},
           kick={f"{k[0]} | {k[1]}": v for k, v in kicks.items()})
json.dump(out, open("tail_mech.json", "w"), indent=1, default=float)
for k, v in res.items():
    s = " | ".join(f"{tn.split(',')[0]}: " + ("n/a" if d is None else f"max {max(d['frac_rated'])*100:.0f}% rated, seat {d['M_seat']:.2f}")
                   for tn, d in v.items() if tn != "reach")
    print(f"{k[0]:22s} {k[1]:36s} {s} | lowest tip {v['reach']['lowest_tip_z_stand']*1e3:.0f} mm, side {v['reach']['max_side_y']*1e3:.0f}")
for k, v in ground.items(): print("ground", k, f"{v*1e3:.0f} mm")
for k, v in kicks.items(): print("kick", k, {a: round(b, 3) for a, b in v.items()})

# ---------------------------------------------------------------- sheet
fig = plt.figure(figsize=(16, 9.4), dpi=110)
fig.suptitle("Hippos-1 tail mechanism: joint layout, length, reach, loads", x=0.02, ha="left", fontsize=17, fontweight="bold")
cols = {"P": "#2f6db3", "Y": "#d4802a"}
demo_q = {"PPPP": np.radians([20, -30, -40, -30]), "YPPP": np.radians([35, 20, -40, -40]), "PYPY": np.radians([20, 35, -40, 30])}
for i, (name, lay) in enumerate(LAYOUTS.items()):
    ax = fig.add_axes([0.01 + i * 0.165, 0.52, 0.165, 0.36], projection="3d")
    pts, _ = fk(lay, LENGTHS["300 mm (80/70/60/90)"], demo_q[lay])
    pts = pts - pts[0]
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], "-", color="#666", lw=4)
    for k, kind in enumerate(lay):
        ax.scatter(*pts[k + 1], s=90, color=cols[kind], depthshade=False)
    ax.scatter(*pts[-1], s=60, color="#2e9e5b", marker="s", depthshade=False)
    ax.scatter(0, 0, 0, s=120, color="#e0a030", marker="s", depthshade=False)
    ax.set_xlim(-0.36, 0.04); ax.set_ylim(-0.2, 0.2); ax.set_zlim(-0.2, 0.2); ax.set_box_aspect((1, 1, 1))
    ax.view_init(18, -60); ax.set_xticklabels([]); ax.set_yticklabels([]); ax.set_zticklabels([])
    ax.set_title(name, fontsize=11, fontweight="bold", loc="left")
fig.text(0.02, 0.50, "blue = pitch joint, orange = yaw joint, gold = SFF seat on the rump, green = SFF socket at the tip", fontsize=10, color="#555")

# load table
ax = fig.add_axes([0.51, 0.53, 0.47, 0.36]); ax.axis("off")
ax.set_title("Worst servo load (% of rated) and bending through the rump seat", loc="left", fontsize=12, fontweight="bold")
rows = []
for ln in LENGTHS:
    for name in LAYOUTS:
        r = res[(ln, name)]
        def cell(t):
            d = r[t]
            return "can't" if d is None else f"{max(d['frac_rated'])*100:.0f} % / {d['M_seat']:.2f}"
        rows.append([name.split()[0] + " " + ln.split()[0] + " mm", cell("held straight back, level, 0.3 kg"), cell("held out to the side, 0.3 kg"),
                     f"{r['reach']['max_side_y']*1e3:.0f} mm"])
tb = ax.table(cellText=rows, colLabels=["layout, length", "back, 0.3 kg: % / N m", "to the side, 0.3 kg", "side reach"], loc="upper left", cellLoc="left")
tb.auto_set_font_size(False); tb.set_fontsize(10); tb.scale(1, 1.45)
for (r_, c_), cellx in tb.get_celld().items():
    if r_ > 0 and c_ in (1, 2):
        txt = cellx.get_text().get_text()
        if txt.endswith("can't"): cellx.set_facecolor("#eeeeee")
        elif txt and int(txt.split()[0]) > 100: cellx.set_facecolor("#f6d0cc")
        elif txt: cellx.set_facecolor("#d6efdd")

# reach to the ground
ax = fig.add_axes([0.05, 0.08, 0.28, 0.34]); ax.set_title("Lowest the tail tip can reach (B layout)", loc="left", fontsize=12, fontweight="bold")
pn_list = list(poses) + ["crouched 180 mm"]; xx = np.arange(len(pn_list))
for j, ln in enumerate(LENGTHS):
    ax.bar(xx + (j - 0.5) * 0.36, [ground[(ln, p)] * 1e3 for p in pn_list], 0.34, label=ln, color=["#9bb7dd", "#2f6db3"][j])
ax.axhline(0, color="k"); ax.set_xticks(xx); ax.set_xticklabels([p.replace(" deg", "°").replace("crouched 180 mm", "crouched") for p in pn_list], fontsize=10); ax.set_ylabel("tip height above ground (mm)")
ax.legend(fontsize=9, loc="upper right"); ax.text(-0.4, -40, "below 0 = the tail can plant on the ground", fontsize=9, color="#555")

fig.text(0.37, 0.40, "Recommendation: B (Y-P-P-P).  Length is your pick.", fontsize=14, fontweight="bold")
b3 = res[("300 mm (80/70/60/90)", "B  Y-P-P-P (root yaw, then pitch)")]; b4 = res[("400 mm (100/95/85/120)", "B  Y-P-P-P (root yaw, then pitch)")]
a3 = res[("300 mm (80/70/60/90)", "A  P-P-P-P (all pitch)")]; a4 = res[("400 mm (100/95/85/120)", "A  P-P-P-P (all pitch)")]
W3 = lambda r: max(r["held straight back, level, 0.3 kg"]["frac_rated"]) * 100
k4r = kicks[("400 mm (100/95/85/120)", "rear-up 45 deg")]; k4c = kicks[("400 mm (100/95/85/120)", "crouched")]
g = lambda ln, p: ground[(ln, p)] * 1e3
L3, L4 = "300 mm (80/70/60/90)", "400 mm (100/95/85/120)"
lines = [
    f"B: the root yaw sees no gravity, so the first pitch sits one link out: worst servo holding",
    f"0.3 kg back or to the side {W3(b3):.0f} % of rated (300) / {W3(b4):.0f} % (400).  A and C load the first pitch",
    f"at the root ({W3(a3):.0f} / {W3(a4):.0f} %), and A can't reach sideways.",
    f"The rump is 450 mm up.  300 mm never plants: {g(L3,'standing'):.0f} mm short standing, {g(L3,'rear-up 45 deg'):.0f} mm",
    f"at a 45 deg rear-up, {g(L3,'crouched 180 mm'):.0f} mm crouched.  So 300 = a hand and a balancer, no kickstand.",
    f"400 mm plants in a 30 deg rear-up ({-g(L4,'rear-up 30 deg'):.0f} mm to spare), 45 deg ({-g(L4,'rear-up 45 deg'):.0f}), crouched ({-g(L4,'crouched 180 mm'):.0f}).",
    f"Kickstand through the seat: {k4r['M_seat']:.1f} N m reared at 45 deg; crouched it must plant within",
    f"about 100 mm of the seat (straight down from the first pitch: {k4c['M_seat']:.1f} N m, lugs x1.4).",
    f"400 costs about 35 g and {b4['held straight back, level, 0.3 kg']['M_seat']:.2f} N m on the seat holding 0.3 kg (lugs x4).",
    "",
    "Interface parts (CAD after your pick):",
    "1  root bracket: U-cradle on the adapter flange (4 x M3), STS3250 yaw axis vertical,",
    "   idler shaft in a 6800 bearing in the lower cheek;",
    "2  tail tip: standard SFF arm socket (46 x 32 plate) on the last link, axis along it;",
    "3  rump pad: Ø64 face, 4 M3 inserts at r 29.5, wire hole, in the rump end ring.",
]
for i, s_ in enumerate(lines): fig.text(0.37, 0.37 - i * 0.0235, s_, fontsize=10.5)
fig.savefig("sheet_tail_mech.png")
