"""Coupler face rethink: should the motor-phase bullets stay?  Visual design study (2026-11-19).
Face layouts to scale (hand side, looking into the hand's face) for four ways of getting the motor phases and the signals
across the joint, and a comparison of what each one costs.  Numbers marked est. are estimates; part data from datasheets:
Castle 4 mm bullet 75 A; Mill-Max 0860-0-15-20-82-14-11-0 spring pin 7.3 A, 120 gf mid-stroke, Ø2.36 pad;
Amass MR60 3-pole 3.5 mm motor plug (60 A class).  Current to carry: hand3 rotor 36 A hover, 55 A peak, 6S (25.2 V)."""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Wedge, Rectangle, FancyBboxPatch
import textwrap
import handmount_b as H

P = H.P
OUT = "coupler_face_study2.png"
C = dict(flange="#d9d9d9", lug="#9e9e9e", spigot="#bdbdbd", bullet="#c47a2c", pin="#1f6fb2", ring="#e0a33a",
         sig="#7fb3e0", screw="#555555", collar="#8e6bbf", mr="#d4a017", pad="#e0a33a")


def at(r, az):
    a = np.radians(az); return r * np.cos(a), r * np.sin(a)


def base(ax, title, lugs_w, collar):
    ax.set_aspect("equal"); ax.set_xlim(-31, 31); ax.set_ylim(-31, 31); ax.axis("off")
    ax.set_title(title, fontsize=12, fontweight="bold", loc="left")
    if collar:
        ax.add_patch(Circle((0, 0), P["R_collar"], color=C["collar"], alpha=0.25, lw=0))
        ax.add_patch(Circle((0, 0), P["collar_cb"], color="white", lw=0))
    for az in (0, 120, 240):
        ax.add_patch(Wedge((0, 0), P["lug_r1"], az - lugs_w / 2, az + lugs_w / 2, color=C["lug"], lw=0))
    ax.add_patch(Circle((0, 0), P["R_flange"], color=C["flange"], lw=0))
    ax.add_patch(Circle((0, 0), P["r_spigot"], color=C["spigot"], lw=0))


def signal_pins(ax, rings):
    for r in P["pin_r"]:
        if rings and r > 0:
            ax.add_patch(Circle((0, 0), r + 0.45, color=C["sig"], fill=False, lw=2.2))
        x, y = at(r, 0); ax.add_patch(Circle((x, y), 0.55, color=C["pin"], lw=0))


fig = plt.figure(figsize=(20, 13.5), facecolor="white")
fig.suptitle("Cephalo-2 coupler face, rethink 2: stock parts only (no custom PCBs).  Hand face, to scale (mm), looking into the hand",
             fontsize=16, fontweight="bold", y=0.985)
axs = [fig.add_axes([0.01 + i * 0.247, 0.47, 0.235, 0.46]) for i in range(4)]

# A: today (B.5)
ax = axs[0]; base(ax, "A  today (B.5): 3 bullets + collar", P["lug_w"], True)
for az in P["az_bullet"]:
    x, y = at(P["r_bullet"], az); ax.add_patch(Circle((x, y), P["fem_bore"] / 2, color=C["bullet"], lw=0))
for r, az in P["screws"]:
    x, y = at(r, az); ax.add_patch(Circle((x, y), 3.1, color=C["screw"], fill=False, lw=1.5, ls="--"))
signal_pins(ax, False)
ax.text(0, -30, "phases: 3 x Castle 4 mm bullets (75 A), 8.5 mm deep\nsignals: 5 pogo pins in the spigot\nlocks: collar turns 60 deg, hand pushed straight",
        ha="center", va="bottom", fontsize=9.5)

# B2: spring pins onto Mill-Max target studs, hand twists (no collar)
ax = axs[1]; base(ax, "B2  pins on target studs, hand twists (no collar)", 50, False)
for k, az0 in enumerate((90, 210, 330)):
    for j_, (r, da) in enumerate(((13.0, -9), (13.0, 9), (16.0, -7.5), (16.0, 7.5), (19.0, -6.3), (19.0, 6.3))):
        x, y = at(r, az0 + da); ax.add_patch(Circle((x, y), 1.6, color=C["pad"], lw=0)); ax.add_patch(Circle((x, y), 0.64, color=C["pin"], lw=0))
signal_pins(ax, True)
ax.text(0, -30, "phases: 6 Mill-Max 0860 pins (7.3 A) per phase land on\n6 gold target studs (Mill-Max 1974, press-fit into the print)\nlocks: hand turns 60 deg; the ramp lifts the pins clear\nuntil the last 15 deg, then sets them on the studs",
        ha="center", va="bottom", fontsize=9.2)

# C2: same pins + studs, straight push, collar kept
ax = axs[2]; base(ax, "C2  pins on target studs, straight push + collar", P["lug_w"], True)
for k, az0 in enumerate((90, 210, 330)):
    for j_, (r, da) in enumerate(((13.0, -9), (13.0, 9), (16.0, -7.5), (16.0, 7.5), (19.0, -6.3), (19.0, 6.3))):
        x, y = at(r, az0 + da); ax.add_patch(Circle((x, y), 1.6, color=C["pad"], lw=0)); ax.add_patch(Circle((x, y), 0.64, color=C["pin"], lw=0))
signal_pins(ax, False)
ax.text(0, -30, "phases: same 18 pins on 18 studs\nno 60 N push, nothing to line up to 0.1 mm\nlocks: collar as today",
        ha="center", va="bottom", fontsize=9.5)

# F: motor plug on a lead, plugged by hand; bayonet twists (no collar)
ax = axs[3]; base(ax, "F  MR60 plug on a lead + twist bayonet", 50, False)
ax.add_patch(FancyBboxPatch((21.5, -6.5), 7.5, 13, boxstyle="round,pad=0.3", color=C["mr"], lw=0))
for dy in (-4.4, 0, 4.4):
    ax.add_patch(Circle((25.2, dy), 1.75, color="white", lw=0)); ax.add_patch(Circle((25.2, dy), 1.2, color=C["bullet"], lw=0))
ax.plot([20.6, 21.6], [0, 0], color="k", lw=3)
signal_pins(ax, True)
ax.text(0, -30, "phases: Amass MR60 (3 x 3.5 mm, 60 A class) on a 6 cm lead,\nplugged by hand into a pocket on the hand's side\nlocks: the hand itself turns (rev A style), no collar;\nface carries only the 5 signal pins",
        ha="center", va="bottom", fontsize=9.2)

# comparison table
rows = [
    ("", "A  bullets + collar (today)", "B2  pins on studs, hand twists", "C2  pins on studs + collar", "F  plug on a lead + twist"),
    ("Phase current", "75 A per bullet", "6 x 7.3 = 44 A per phase cont.; 55 A short peaks (est.)", "same as B2", "60 A class (Amass MR60)"),
    ("Parts to buy (all stock)", "3 Castle 4 mm bullet pairs", "18 Mill-Max 0860 pins + 18 Mill-Max 1974 studs (Digi-Key / Mouser)", "same as B2", "1 MR60 pair (any RC shop)"),
    ("Insert / lock effort", "~60 N push, then turn the collar", "push, turn the hand 60 deg; ramp takes the ~27 N of pin springs", "~27 N push, turn the collar", "turn the hand, then plug the MR60 by hand"),
    ("Alignment", "3 rigid pins + spigot: over-constrained, ±0.1 mm", "studs Ø3.2 under Ø1.3 plungers: ±0.9 mm", "±0.9 mm", "plug on a free lead: none"),
    ("Collar", "yes, 13.4 g + detent", "no (lugs lock on the socket's lips)", "yes", "no"),
    ("Anti-rattle preload", "O-ring + shims", "the 23 pin springs (~27 N)", "pin springs", "O-ring (as today)"),
    ("Coupler mass, arm + hand (est.)", "~48 g", "~34-38 g", "~45 g", "~32-35 g"),
    ("Blind / robot can swap alone", "yes (if something turns the collar)", "yes (if something turns the hand)", "yes", "no: a person plugs the lead"),
    ("Main risk", "heavy, stiff to push, collar is the weak link", "18 small contacts share the current: one bad pin heats the rest; bench test", "keeps the collar", "loose lead flapping near the rotor; extra step"),
]
tax = fig.add_axes([0.02, 0.02, 0.96, 0.42]); tax.axis("off")
widths = [0.16, 0.21, 0.23, 0.2, 0.2]; xs = np.cumsum([0] + widths[:-1])
nr = len(rows)
for i, row in enumerate(rows):
    y = 1 - (i + 1) / (nr + 0.2)
    for j, cell in enumerate(row):
        bold = i == 0 or j == 0
        col = "#eef6ee" if (j == 2 and i > 0) else "white"
        tax.add_patch(Rectangle((xs[j], y), widths[j], 1 / (nr + 0.2), facecolor=col, edgecolor="#bbbbbb", lw=0.8, transform=tax.transAxes))
        tax.text(xs[j] + 0.005, y + 0.5 / (nr + 0.2), "\n".join(textwrap.wrap(cell, 44 if j else 30)), transform=tax.transAxes,
                 fontsize=10, va="center", fontweight="bold" if bold else "normal")
fig.text(0.5, 0.445, "Masses and forces marked est. are first estimates. Every part listed is a stock catalogue part; printed parts are PA-CF as today.",
         ha="center", fontsize=10.5, style="italic")
fig.savefig(OUT, dpi=90)
print("wrote", OUT)
