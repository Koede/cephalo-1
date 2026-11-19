"""Coupler rev B.2: design study for the axial preload (anti-rattle) element (2026-11-18).  No CAD change: sketches on
the real section of handmount_b.py, numbers by hand formula (estimates).  python3 coupler_preload_study.py ->
coupler_preload_study.png, coupler_preload_study.json

Problem: a locked hand has 0.1 mm axial play (lug between the nose-ring face and the collar lip) and nothing pushes it
one way; only the 5 spring pins push the hand out, about 2.9 N (Mill-Max 0906: 60 g each at the 0.71 mm working stroke).
Every load reversal along the tip axis (arm swings, wheel bumps, rotor spool-up / -down) knocks the hand across the gap.

Options (all keep the rev B.2 lugs, lips, ramp and bullets):
  0  nothing: spring pins only
  A  face O-ring in a groove in the NOSE-RING face, pressing on the hand base's flange top inside the bullets
     (one ring per arm, every hand benefits; also a dust seal round the spring pins).  As rev A did on the hand side.
  B  thin wave spring (steel) under the three lugs, between the lug tops and the nose ring; nose ring recessed for it
  C  printed flexure lips: each collar lip segment is a cantilever with 0.3 mm interference on the lug
  D  TPU 95A pads glued on the lip tops (0.6 mm, 0.2 mm interference)
"""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, Polygon
import manifold3d as mf
import handmount_b as HB

P = HB.P
G = 9.81
M_HAND = dict(hand3=0.3964, wheel=0.289 + 0.012)     # kg (hand3-final.md; wheelhand.json total + hand-side screws, est.)
POGO = 5 * 0.060 * G                                  # N, spring pins at working stroke
MU = 0.30                                             # PA-CF on PA-CF, dry (est.)
R_LIP = (P["lip_r0"] + P["lug_r1"]) / 2               # mm, mean radius of the lug-on-lip contact
TAN_RAMP = P["ramp"] / (R_LIP * np.radians(P["ramp_deg"]))
GAP = P["ax_clr"]                                     # 0.1 mm play at lock
TOL = 0.10                                            # mm, print / stack tolerance on the squeeze or interference (est.)

# ---- A: face O-ring in the nose ring (arm side).  Free annulus on the nose-ring face: bore r 10.6 .. female-bullet mouth
# r 15.4 - 2.6 = 12.8 (screw counterbores start at 13.9).  Groove 11.0 .. 12.4 (0.4 mm walls each side), O-ring 22 x 1.0.
# Groove depth is picked so the NBR ring gives about F_TARGET at lock (see pick_depth()).
ORING = dict(ID=22.0, cs=1.0, groove=(11.0, 12.4), depth=0.70, mats=dict(silicone_50A=2.0, NBR_70A=5.5))   # E in MPa (est.)
F_TARGET = 35.0


def lindley(E, d, eps):
    """Face-compressed O-ring, force per unit length (N/mm), Lindley's formula (eps = squeeze / cs)."""
    eps = np.clip(eps, 0, None)
    return E * d * (1.25 * eps ** 1.5 + 50 * eps ** 6)


def oring_force(E, squeeze):
    rc = (ORING["ID"] + ORING["cs"]) / 2
    return lindley(E, ORING["cs"], squeeze / ORING["cs"]) * 2 * np.pi * rc


def pick_depth():
    best = min(np.arange(0.60, 0.851, 0.01), key=lambda d: abs(oring_force(ORING["mats"]["NBR_70A"], ORING["cs"] - d - GAP) - F_TARGET))
    ORING["depth"] = round(float(best), 2)


def option_A():
    pick_depth()
    sq = ORING["cs"] - ORING["depth"] - GAP                       # squeeze at lock (hand pushed out onto the lips)
    out = dict(squeeze_mm=round(sq, 2), squeeze_pct=round(100 * sq / ORING["cs"]), squeeze_full_in_pct=round(100 * (sq + GAP) / ORING["cs"]))
    for m, E in ORING["mats"].items():
        out[m] = dict(F=round(float(oring_force(E, sq)), 1), F_lo=round(float(oring_force(E, sq - TOL)), 1), F_hi=round(float(oring_force(E, sq + TOL)), 1))
    free = ORING["cs"] - ORING["depth"]                           # hand rests on the free ring before the collar turns
    out["lug_vs_ramp_capture_mm"] = round(P["ramp"] - (free - GAP), 2)   # > 0: the ramp's low end still catches the lug
    out["groove_wall_to_bore_mm"] = round(ORING["groove"][0] - P["r_bore"], 2)
    out["groove_wall_to_bullet_mm"] = round(P["r_bullet"] - (P["pin_d"] / 2 + 0.6) - ORING["groove"][1], 2)
    return out


# ---- B: wave spring under the lugs (single-turn, crest-to-crest style), r 20.4 .. 22.4, working height 0.5 mm
WAVE = dict(ID=40.8, OD=44.8, t=0.45, waves=4, h_free=0.9, h_work=0.5, E=200e3)


def option_B():
    # single-turn wave spring rate (Smalley design formula): k = E b t^3 N^4 / (2.4 D^3) * (ID/OD); est.
    b = (WAVE["OD"] - WAVE["ID"]) / 2; Dm = (WAVE["OD"] + WAVE["ID"]) / 2; N = WAVE["waves"]
    k = WAVE["E"] * b * WAVE["t"] ** 3 * N ** 4 / (2.4 * Dm ** 3) * (WAVE["ID"] / WAVE["OD"])
    defl = WAVE["h_free"] - WAVE["h_work"]
    stress = 3 * np.pi * k * defl * Dm / (4 * b * WAVE["t"] ** 2 * N ** 2)     # MPa, Smalley single-turn formula
    return dict(rate_N_per_mm=round(k, 1), stress_MPa=round(stress), F=round(k * defl, 1), F_lo=round(k * (defl - TOL), 1), F_hi=round(k * (defl + TOL), 1),
                nose_ring_recess_mm=WAVE["h_work"], source="custom size (no catalogue part found at 40.8 x 44.8; est.)")


# ---- C: printed flexure lip (cantilever, PA-CF), interference 0.3 mm
def option_C(interf=0.30, L=6.0, t=1.2, w=12.0, E=3500.0):
    k = 3 * E * (w * t ** 3 / 12) / L ** 3                        # N/mm per lip segment, cantilever tip
    F = 3 * k * interf
    s = 6 * (k * interf) * L / (w * t ** 2)
    return dict(F=round(F, 1), F_lo=round(3 * k * (interf - TOL), 1), F_hi=round(3 * k * (interf + TOL), 1), root_stress_MPa=round(s, 1),
                creep="PA-CF under a steady 40-50 % of its yield: relaxes noticeably over weeks (est.)")


# ---- D: TPU pads on the lip tops, 0.6 mm thick, 0.2 mm interference
def option_D(t=0.6, interf=0.01, E=30.0):
    A = 3 * np.radians(120 - P["gap_w"]) * R_LIP * (P["lug_r1"] - P["lip_r0"])   # mm^2, pad area under the lugs
    k = E * A / t
    return dict(F=round(k * interf, 1), F_lo=round(k * max(interf - TOL, 0), 1), F_hi=round(k * (interf + TOL), 1), rate_N_per_mm=round(k),
                wear="pads slide 60 deg under the lugs on every lock, under load")


def torques(F):
    """Collar torque to draw the hand in over the ramp against F, and friction torque that then holds the collar (N m)."""
    r = R_LIP * 1e-3
    lock = F * r * (TAN_RAMP + MU) / (1 - MU * TAN_RAMP) + 0.08          # + the ball detent click (0.08 N m)
    hold = F * r * MU                                                       # on the flat seat
    return round(lock, 2), round(hold, 3)


def summary():
    A = option_A(); B = option_B(); Cc = option_C(); D = option_D()
    rows = {"0 spring pins only": dict(F=round(POGO, 1), F_lo=round(POGO, 1), F_hi=round(POGO, 1), parts=0, mass_g=0.0, per="-",
                                       creep="none", note="already there")}
    rows["A O-ring in the nose ring (NBR 70A)"] = dict(**{k: A["NBR_70A"][k] for k in ("F", "F_lo", "F_hi")}, parts=1, mass_g=0.1, per="arm",
                                                      creep="NBR compression set ~15-25 %/yr (est.); replace on service", note="O-ring 22 x 1.0, off the shelf; seals the pin well")
    rows["A' same, silicone 50A"] = dict(**{k: A["silicone_50A"][k] for k in ("F", "F_lo", "F_hi")}, parts=1, mass_g=0.1, per="arm", creep="low", note="softer: less tolerance-sensitive, less preload")
    rows["B wave spring under the lugs"] = dict(F=B["F"], F_lo=B["F_lo"], F_hi=B["F_hi"], parts=1, mass_g=0.6, per="arm", creep="none (steel)", note=B["source"])
    rows["C printed flexure lips"] = dict(F=Cc["F"], F_lo=Cc["F_lo"], F_hi=Cc["F_hi"], parts=0, mass_g=0.0, per="-", creep=Cc["creep"], note=f"root stress {Cc['root_stress_MPa']} MPa")
    rows["D TPU pads on the lips"] = dict(F=D["F"], F_lo=D["F_lo"], F_hi=D["F_hi"], parts=3, mass_g=0.1, per="arm", creep="TPU set + wear", note=D["wear"] + f"; {D['rate_N_per_mm']} N/mm: 0.01 mm sets the force")
    for k, r in rows.items():
        F = r["F"] + (0 if k.startswith("0") else POGO)
        r["F_total"] = round(F, 1)
        r["rattle_free_g"] = {h: round(F / (m * G), 1) for h, m in M_HAND.items()}
        r["rattle_free_g_lo"] = {h: round((r["F_lo"] + (0 if k.startswith("0") else POGO)) / (m * G), 1) for h, m in M_HAND.items()}
        r["lock_torque_Nm"], r["hold_torque_Nm"] = torques(F)
    return dict(rows=rows, A_detail=A, B_detail=B, C_detail=Cc, D_detail=D, pogo_N=round(POGO, 2), tan_ramp=round(TAN_RAMP, 3), mu=MU, tol_mm=TOL)


# ------------------------------------------------------------------ figure
def section(m, flip=True):
    """Polygons of the solid cut by the plane y = 0 (x, z)."""
    cs = m.rotate([90, 0, 0]).slice(0.0)
    out = []
    for poly in cs.to_polygons():
        p = np.asarray(poly); out.append(np.c_[p[:, 0], -p[:, 1]] if flip else p)
    return out


COL = dict(socket="#2b2d31", nose="#4b5058", collar="#d2542d", hand="#6d7f95", bullet="#c9a227", new="#1a7f37", ring="#111")


def draw_section(ax, opt):
    parts = [(HB.arm_socket(), COL["socket"]), (HB.nose_ring(), COL["nose"]), (HB.collar(0), COL["collar"]),
             (HB.hand_base().translate([0, 0, -GAP if opt in ("A", "B", "C", "D") else 0.0]), COL["hand"])]
    for m, c in parts:
        for p in section(m):
            ax.add_patch(Polygon(p, closed=True, fc=c, ec="white", lw=0.4, alpha=0.95))
    z0, z1 = P["lug_z"]
    if opt == "A":
        r0, r1 = ORING["groove"]; rc = (r0 + r1) / 2; d = ORING["depth"]
        for s in (-1, 1):
            ax.add_patch(Rectangle((s * rc - (r1 - r0) / 2 if s > 0 else -r1, 0), r1 - r0, d, fc="white", ec=COL["new"], lw=0.8))
            ax.add_patch(Circle((s * rc, d - ORING["cs"] / 2 + 0.15), ORING["cs"] / 2, fc=COL["new"], ec=COL["ring"], lw=0.6))
        ax.annotate("O-ring 22 x 1.0 in a\nnose-ring face groove", (rc, 0.3), (9, -7.5), fontsize=8.5, color=COL["new"], arrowprops=dict(arrowstyle="-", color=COL["new"], lw=0.7))
    if opt == "B":
        for s in (-1, 1):
            xs = np.linspace(P["lug_r"][0], P["lug_r"][1], 30)
            ax.plot(s * xs, 0.25 + 0.2 * np.sin(np.linspace(0, 3 * np.pi, 30)), color=COL["new"], lw=1.6)
        ax.annotate("wave spring under\nthe lugs (recess 0.5)", (21.4, 0.3), (12, 7), fontsize=8.5, color=COL["new"], arrowprops=dict(arrowstyle="-", color=COL["new"], lw=0.7))
    if opt == "C":
        for s in (-1, 1):
            ax.plot([s * 23.0, s * 25.5], [P["lip_z"][1] - 0.6] * 2, color="white", lw=2.2)
        ax.annotate("slot makes each lip a\ncantilever, 0.3 interference", (24, P["lip_z"][1] - 0.6), (9, -7.5), fontsize=8.5, color=COL["new"], arrowprops=dict(arrowstyle="-", color=COL["new"], lw=0.7))
    if opt == "D":
        for s in (-1, 1):
            ax.add_patch(Rectangle((s * P["lip_r0"] if s > 0 else -P["lug_r1"], P["lip_z"][1] - 0.3), P["lug_r1"] - P["lip_r0"], 0.5, fc=COL["new"], ec="none"))
        ax.annotate("TPU pad on each lip top", (21.6, P["lip_z"][1]), (9, -7.5), fontsize=8.5, color=COL["new"], arrowprops=dict(arrowstyle="-", color=COL["new"], lw=0.7))
    if opt == "0":
        ax.annotate("0.1 mm play: lug between the\nnose-ring face and the lip", (21.4, -1.3), (9, -7.5), fontsize=8.5, color="#b42318", arrowprops=dict(arrowstyle="-", color="#b42318", lw=0.7))
    ax.set_xlim(8.5, 28.5); ax.set_ylim(-9, 11); ax.set_aspect("equal"); ax.axis("off")
    for zz, lab in ((0.0, "z 0 mouth"), (P["lip_z"][1], "lip top")):
        pass
    ax.text(27.8, 10.2, "arm", fontsize=8, ha="right", color="#4a5058"); ax.text(27.8, -8.7, "hand", fontsize=8, ha="right", color="#4a5058")


def main():
    S = summary(); rows = S["rows"]
    fig = plt.figure(figsize=(16, 9.6), facecolor="white")
    fig.text(0.015, 0.965, "Coupler rev B.2: what holds a locked hand still?  Preload / anti-rattle options", fontsize=16, fontweight="bold")
    fig.text(0.015, 0.935, "Real section of handmount_b.py through lug 0 (arm up, hand down).  Forces are hand formulas (estimates); "
             f"±{TOL} mm print error shown as the bar's range.  Rattle-free = the hand's own inertia (g) before it crosses the play.", fontsize=10, color="#4a5058")
    keys = ["0", "A", "B", "C", "D"]
    titles = ["0  spring pins only", "A  O-ring in the nose-ring face", "B  wave spring under the lugs", "C  printed flexure lips", "D  TPU pads on the lips"]
    for i, (k, t) in enumerate(zip(keys, titles)):
        ax = fig.add_axes([0.005 + 0.198 * i, 0.47, 0.195, 0.43]); draw_section(ax, k); ax.set_title(t, fontsize=11, fontweight="bold", loc="left")
    # forces
    ax = fig.add_axes([0.15, 0.07, 0.27, 0.34])
    names = list(rows); y = np.arange(len(names))[::-1]
    for yi, n in zip(y, names):
        r = rows[n]; base = 0 if n.startswith("0") else S["pogo_N"]
        if r["F_hi"] + base > 150: ax.text(146, yi + 0.3, "off scale ->", fontsize=8, ha="right")
        ax.barh(yi, min(r["F_total"], 150), color="#1a7f37" if n.startswith("A ") else "#8c959f", height=0.55)
        ax.plot([r["F_lo"] + base, min(r["F_hi"] + base, 150)], [yi, yi], color="black", lw=2)
        ax.text(min(max(r["F_hi"] + base, r["F_total"]) + 2, 152), yi, f"{r['F_total']:.0f} N  |  hand3 {r['rattle_free_g']['hand3']:.0f} g ({r['rattle_free_g_lo']['hand3']:.0f} g low)", va="center", fontsize=9)
    ax.set_yticks(y); ax.set_yticklabels(names, fontsize=9.5); ax.set_xlabel("axial preload on the lugs, N (incl. 2.9 N of spring pins)")
    ax.axvline(M_HAND["hand3"] * G * 3, color="#b42318", ls="--", lw=1); ax.text(M_HAND["hand3"] * G * 3 + 0.5, y[-1] - 0.5, "3 g of hand3", color="#b42318", fontsize=8.5)
    ax.set_xlim(0, 150)
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    # table of trade-offs
    A = S["A_detail"]
    lines = ["Recommendation: A, NBR 70A O-ring 22 x 1.0 in the nose ring (one per arm, every hand gets it)",
             f"  squeeze {A['squeeze_mm']} mm ({A['squeeze_pct']} %) at lock, {A['squeeze_full_in_pct']} % with the hand pushed fully in; groove 11.0-12.4 x {ORING['depth']} deep",
             f"  {rows['A O-ring in the nose ring (NBR 70A)']['F_total']:.0f} N preload: a locked hand3 stays put to {rows['A O-ring in the nose ring (NBR 70A)']['rattle_free_g']['hand3']:.0f} g; collar lock torque "
             f"{rows['A O-ring in the nose ring (NBR 70A)']['lock_torque_Nm']} N m (fingers), friction then holds it {rows['A O-ring in the nose ring (NBR 70A)']['hold_torque_Nm']} N m",
             f"  ramp still catches the lug: {A['lug_vs_ramp_capture_mm']} mm margin; groove walls {A['groove_wall_to_bore_mm']} mm (bore) / {A['groove_wall_to_bullet_mm']} mm (bullet mouth)",
             "  + off the shelf, 0.1 g, nothing on the hands changes, seals the spring-pin well, proven on rev A",
             "  - rubber takes a set over time (replace on service); ±0.1 mm print error moves the force a lot (9-77 N): print the nose ring, measure the gap, reprint at the right groove depth (4 g, 0.05 mm steps)",
             "",
             f"B wave spring (0.45 steel, 4 waves): {rows['B wave spring under the lugs']['F_total']:.0f} N, hardly changes with print error, no set; but custom size (none found off the shelf), 0.5 mm recess",
             "C flexure lips: no parts, but PA-CF creeps at that stress and the lip gets weaker (the lip is a stress hot spot)",
             f"D TPU pads: {S['D_detail']['rate_N_per_mm']} N/mm stiff, so 0.1 mm print error = 0 or {S['D_detail']['F_hi']:.0f} N; and they slide 60 deg under load",
             "0 spring pins alone hold the hand3 only to under 1 g: it will rattle"]
    import textwrap
    wrapped = []
    for l in lines:
        ind = len(l) - len(l.lstrip())
        wrapped += textwrap.wrap(l, 86, subsequent_indent=" " * (ind + 2)) or [""]
    fig.text(0.575, 0.42, "\n".join(wrapped), fontsize=9.3, va="top", family="DejaVu Sans", linespacing=1.45)
    fig.savefig("coupler_preload_study.png", dpi=110)
    json.dump(S, open("coupler_preload_study.json", "w"), indent=1, default=float)
    for k, r in rows.items(): print(f"{k:40s}", {kk: r[kk] for kk in ("F_total", "F_lo", "F_hi", "rattle_free_g", "lock_torque_Nm", "hold_torque_Nm")})
    print("A", S["A_detail"]); print("B", S["B_detail"]); print("C", S["C_detail"]); print("D", S["D_detail"])


if __name__ == "__main__":
    main()
