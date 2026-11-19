"""Hippos-1 tail on the SFF connector (the 32 mm push-and-twist hand connector, `hand-connector.md`): design study.
Hand calculations only, no CAD.  python3 tail_sff_study.py -> tail_sff.json, sheet_tail_sff.png
Tail model = centaur_equo_v2_sizing.py (4 joints 80/70/60/90 mm, 2 x STS3250 root, 2 x STS3215), Centauri-1 30.8 kg.
"""
import json, numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch

G = 9.81
M_DOCK, M_SOLO = 30.8, 18.5
links = np.array([0.080, 0.070, 0.060, 0.090])
servos = ["STS3250", "STS3250", "STS3215", "STS3215"]
# rated N m, stall N m, mass kg, rated A, stall A (12 V).  STS3250 from Feetech listings; STS3215 from the project (2.7 A stall)
SV = {"STS3215": (0.98, 2.94, 0.055, 0.9, 2.7), "STS3250": (1.57, 4.90, 0.0745, 1.4, 4.2)}
m_link, m_grip = 0.025, 0.060
OFF = 0.030          # option A: socket face 30 mm ahead of the root joint axis (hand base 9.7 + servo half-length)

def tail(th, payload, tipF=np.zeros(2), m_tip=m_grip):
    P = [np.zeros(2)]
    for L, a in zip(links, th): P.append(P[-1] + L * np.array([-np.cos(a), np.sin(a)]))
    P = np.array(P)
    # point masses: links at mids, servo j+1 at joint j+1, tip mass + payload at the tip
    pts = [(0.5 * (P[k] + P[k + 1]), m_link) for k in range(4)] + [(P[k], SV[servos[k]][2]) for k in range(1, 4)] + [(P[4], m_tip + payload)]
    def moment(about, include_root_servo=False):
        t = 0.0
        for p, m in pts: t += m * G * (p[0] - about[0])
        t += tipF[1] * (P[4][0] - about[0]) - tipF[0] * (P[4][1] - about[1])
        if include_root_servo: t += SV[servos[0]][2] * G * (P[0][0] - about[0])
        return abs(t)
    Fz = sum(m for _, m in pts) * G - tipF[1]
    return P, moment, Fz

W = M_DOCK * G
poses = {
    "held out level, 0.3 kg": (np.zeros(4), 0.3, np.zeros(2)),
    "held out level, empty": (np.zeros(4), 0.0, np.zeros(2)),
    "curled up (rest)": (np.radians([40, 90, 140, 200]), 0.0, np.zeros(2)),
    "kickstand straight, 10 % of weight": (np.radians([-90] * 4), 0.0, np.array([0, 0.10 * W])),
    "kickstand raked 15 deg, 10 % of weight": (np.radians([-75] * 4), 0.0, np.array([0, 0.10 * W])),
}
res = {}
for n, (th, pl, f) in poses.items():
    P, mom, Fz = tail(th, pl, f)
    conn_A = mom(np.array([OFF, 0.0]), include_root_servo=True)       # socket on the rump, whole tail detaches
    conn_B = mom(P[1])                                                 # socket after joint 0 (root servo stays on the body)
    res[n] = dict(M_A=conn_A, M_B=conn_B, Fz=Fz, j0=mom(P[0]), j0_frac_rated=mom(P[0]) / SV["STS3250"][0])

# SFF capacity, scaled from hand-connector.md's statics: worst lug = 2M/(3 r) + pull/3, r = 12 mm.
# At the design case (100 N + 1 N m) the hand doc gives 94.8 N worst lug, factor 5.8 (layer shear) and 6.0 (lip bending).
r_lug = 0.012
F_lug_cap = 94.8 * min(5.8, 6.0, 6.2, 7.3)          # N per lug at factor 1 (weakest feature: lug root layer shear)
def lug_factor(M, pull=0.0): return min(99.0, F_lug_cap / max(1e-9, 2 * M / (3 * r_lug) + max(pull, 0.0) / 3))
T_latch_cap = 1.0 * 3.4                               # N m about the axis, latch tooth bending between layers (factor 3.4 at 1 N m)
O_ring = (20.0, 50.0); r_or = 0.0135
M_gap = [3 * F * r_or / 2 / 1.5 for F in O_ring]       # rough moment at which the O-ring preload is used up on one side (est.)
for n in res:
    res[n]["lug_factor_A"] = lug_factor(res[n]["M_A"], 0.0)
    res[n]["lug_factor_B"] = lug_factor(res[n]["M_B"], 0.0)
# worst dynamic case: the root servo at stall reacting through the socket (A only), e.g. tail swung hard into a stop
M_stall = SV["STS3250"][1]
res_dyn = dict(M_A=M_stall, lug_factor_A=lug_factor(M_stall))

# current through the one 12 V pin (Mill-Max 0906, 6.5 A, 5.2 A derated)
I_pin, I_pin_der = 6.5, 5.2
I_stall = sum(SV[s][4] for s in servos)
I_rated = sum(SV[s][3] for s in servos)
# proposal: STS protection current per servo so the sum stays under the derated pin
I_prot = {"j0 STS3250": 1.8, "j1 STS3250": 1.6, "j2 STS3215": 0.8, "j3 STS3215": 0.8}
def tq_at(I, s): Rt, St, m, Ir, Is = SV[s]; I0 = 0.28 if s == "STS3250" else 0.18; return St * (I - I0) / (Is - I0)
prot_tq = {k: tq_at(v, k.split()[1]) for k, v in I_prot.items()}
need = res["held out level, 0.3 kg"]["j0"]

L_tail = links.sum()
gap, r_face = 0.2e-3, 0.014
th_plain = gap / r_face
c_sk, L_sk = 0.10e-3, 0.035          # skirt: 0.10 mm radial slip fit, end 35 mm from the cone seat
th_skirt = c_sk / L_sk
wob = dict(plain_deg=np.degrees(th_plain), plain_tip_mm=th_plain * L_tail * 1e3, skirt_deg=np.degrees(th_skirt), skirt_tip_mm=th_skirt * L_tail * 1e3)
mass_add = dict(socket=9.3, base=10.6, pins_boards_oring=5.0)
out = dict(poses=res, dyn=res_dyn, F_lug_cap=F_lug_cap, T_latch_cap=T_latch_cap, M_gap_range=M_gap,
           current=dict(I_stall_all=I_stall, I_rated_all=I_rated, I_pin=I_pin, I_pin_derated=I_pin_der, I_prot=I_prot,
                        I_prot_sum=sum(I_prot.values()), prot_torque=prot_tq, need_j0_held_out=need),
           mass_add_g=sum(mass_add.values()), wobble=wob)
json.dump(out, open("tail_sff.json", "w"), indent=1, default=float)

print(f"{'pose':42s} {'M at socket A':>14s} {'lug x':>6s} {'M at B':>8s} {'lug x':>6s} {'root j0':>8s}")
for n, r in res.items():
    print(f"{n:42s} {r['M_A']:10.2f} N m {r['lug_factor_A']:6.1f} {r['M_B']:8.2f} {r['lug_factor_B']:6.1f} {r['j0_frac_rated']*100:6.0f} %")
print(f"root servo at stall through socket A: {M_stall:.2f} N m, lug x{res_dyn['lug_factor_A']:.1f}")
print(f"O-ring preload used up (face starts to rock) at about {M_gap[0]:.2f}-{M_gap[1]:.2f} N m (est.)")
print(f"wobble: plain SFF +-{wob['plain_deg']:.2f} deg = +-{wob['plain_tip_mm']:.1f} mm at the tip; with skirt +-{wob['skirt_deg']:.2f} deg = +-{wob['skirt_tip_mm']:.1f} mm")
print(f"current: all stalled {I_stall:.1f} A, all rated {I_rated:.1f} A, pin {I_pin} A ({I_pin_der} derated); protection sum {sum(I_prot.values()):.1f} A")
print("  protection torques:", {k: round(v, 2) for k, v in prot_tq.items()}, f"need j0 {need:.2f} N m held out")

# ---------------------------------------------------------------- sheet
fig = plt.figure(figsize=(16, 9), dpi=110)
fig.suptitle("Hippos-1 tail on the SFF connector: where to cut, loads, current", x=0.02, ha="left", fontsize=17, fontweight="bold")
C = dict(body="#c9ced6", tail="#2f6db3", sock="#e0a030", servo="#444", hand="#2e9e5b")

def draw(ax, mode, title):
    ax.set_title(title, loc="left", fontsize=13, fontweight="bold")
    ax.add_patch(FancyBboxPatch((-0.20, -0.06), 0.20, 0.12, boxstyle="round,pad=0.01,rounding_size=0.04", fc=C["body"], ec="#888"))
    ax.text(-0.17, 0.0, "rump", fontsize=11, va="center")
    th = np.radians([-20, -10, 10, 30]); P = [np.array([0.0, 0.0])]
    base = np.array([0.03 if mode == "A" else 0.0, 0.0])
    P = [base]
    for L, a in zip(links, th): P.append(P[-1] + L * np.array([np.cos(a), np.sin(a)]))
    P = np.array(P)
    det = slice(0, 5) if mode == "A" else slice(1, 5)
    ax.plot(P[:, 0], P[:, 1], "-", color="#bbb", lw=7)
    ax.plot(P[det, 0], P[det, 1], "-", color=C["tail"], lw=7, solid_capstyle="round")
    for k in range(4): ax.add_patch(Rectangle(P[k] - [0.012, 0.009], 0.024, 0.018, color=C["servo"]))
    ax.add_patch(Circle(P[4], 0.014, color=C["hand"])); ax.text(P[4][0] + 0.02, P[4][1], "SFF socket\nat the tip:\nany small hand", fontsize=10, color=C["hand"], va="center")
    sx = 0.0 if mode == "A" else P[1][0] - 0.02
    sy = 0.0 if mode == "A" else P[1][1]
    ax.add_patch(Rectangle((sx, sy - 0.022), 0.010, 0.044, color=C["sock"]))
    ax.annotate("SFF: push, twist 60 deg", (sx + 0.005, sy + 0.024), (sx - 0.05, sy + 0.09), fontsize=11, color=C["sock"],
                arrowprops=dict(arrowstyle="->", color=C["sock"]))
    ax.text(0.10, -0.13, "blue = comes off", fontsize=10, color=C["tail"])
    ax.set_xlim(-0.21, 0.40); ax.set_ylim(-0.16, 0.20); ax.set_aspect("equal"); ax.axis("off")

draw(fig.add_axes([0.0, 0.50, 0.30, 0.40]), "A", "A  Socket on the rump: whole tail comes off")
draw(fig.add_axes([0.0, 0.06, 0.30, 0.40]), "B", "B  Socket after the root joint: stump stays")

ax = fig.add_axes([0.47, 0.12, 0.24, 0.70]); ax.set_title("Bending through the socket (N m)", loc="left", fontsize=13, fontweight="bold")
names = list(res) + ["root servo at stall (A)"]
MA = [res[n]["M_A"] for n in res] + [M_stall]; MB = [res[n]["M_B"] for n in res] + [np.nan]
y = np.arange(len(names))
ax.barh(y + 0.18, MA, 0.34, color=C["tail"], label="A rump")
ax.barh(y - 0.18, MB, 0.34, color="#9bb7dd", label="B after root")
cap = F_lug_cap * 3 * r_lug / 2
ax.axvline(cap / 2, color="#c0392b", ls="--"); ax.text(cap / 2 - 0.1, 2.6, ha="right", s=None) if False else ax.text(cap / 2 - 0.08, 2.5, f"lugs at x2\n({cap/2:.1f} N m)", color="#c0392b", fontsize=10, ha="right")
ax.axvspan(M_gap[0], M_gap[1], color="#e0a030", alpha=0.25); ax.text(M_gap[1] + 0.1, 2.35, "O-ring preload used up:\nface may rock (bench check)", fontsize=9, color="#a06a10")
ax.set_yticks(y); ax.set_yticklabels(names, fontsize=10); ax.invert_yaxis(); ax.legend(loc="upper right", fontsize=10)
for i, v in enumerate(MA): ax.text(v + 0.05, i + 0.18, f"{v:.2f}  (x{lug_factor(v):.1f})", fontsize=9, va="center")
ax.set_xlim(0, 6.5)

ax = fig.add_axes([0.74, 0.50, 0.24, 0.36]); ax.set_title("Current through the one 12 V pin (A)", loc="left", fontsize=13, fontweight="bold")
ax.bar(["all 4 rated", "all 4 stalled", "protection\nlimits (sum)"], [I_rated, I_stall, sum(I_prot.values())], color=["#7a7a7a", "#c0392b", C["hand"]])
ax.axhline(I_pin_der, color="k", ls="--"); ax.text(2.45, I_pin_der + 0.2, f"pin {I_pin_der} A derated", ha="right", fontsize=10)
ax.set_ylim(0, 15)

fig.text(0.74, 0.43, "Recommendation: A", fontsize=14, fontweight="bold")
lines = [
    "Socket on the rump; the whole tail comes off.",
    f"Worst static bending {max(r['M_A'] for r in res.values()):.1f} N m (raked kickstand): lugs x{min(r['lug_factor_A'] for r in res.values()):.1f}.",
    f"Root servo stalled into a stop: {M_stall:.1f} N m, lugs x{res_dyn['lug_factor_A']:.1f}.",
    "Strength passes.  Stiffness does not:",
    f"above ~0.5 N m the plug rocks on the O-ring:",
    f"+-{wob['plain_deg']:.2f} deg = +-{wob['plain_tip_mm']:.0f} mm at the tail tip, clunks on reversal.",
    f"Fix (tail only, standard untouched): a 44 mm skirt",
    f"on the tail base in a rump sleeve: +-{wob['skirt_tip_mm']:.1f} mm.",
    f"Current: 4 stalled {I_stall:.1f} A > {I_pin_der} A pin.  STS protection",
    f"current {sum(I_prot.values()):.1f} A total + 5 A PTC; j0 still {prot_tq['j0 STS3250']:.2f} N m (needs {need:.2f}).",
    "SFF socket at the tail tip too: small hands swap there.",
]
for i, s in enumerate(lines): fig.text(0.74, 0.40 - i * 0.031, s, fontsize=10.5)
fig.savefig("sheet_tail_sff.png")
