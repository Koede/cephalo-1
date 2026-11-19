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
OUT = "coupler_face_study.png"
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
fig.suptitle("Cephalo-2 coupler face: should the motor-phase bullets stay?  Hand face, to scale (mm), looking into the hand",
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

# B: ring contacts, hand twists itself (no collar)
ax = axs[1]; base(ax, "B  contact rings, hand twists (no collar)", 50, False)
rings = [(11.2, 13.6), (14.2, 16.6), (17.2, 19.6)]
for (a, b) in rings:
    ax.add_patch(Wedge((0, 0), b, 0, 360, width=b - a, color=C["ring"], lw=0))
for k, (a, b) in enumerate(rings):
    rm = (a + b) / 2
    for j in range(6):
        x, y = at(rm, 15 + k * 20 + j * 60); ax.add_patch(Circle((x, y), 1.18, color=C["pin"], lw=0))
signal_pins(ax, True)
ax.text(0, -30, "phases: 3 gold rings on the hand's face PCB,\n6 x Mill-Max 0860 (7.3 A) per ring in the arm = 44 A\nlocks: the hand turns 60 deg; pins wipe round the rings",
        ha="center", va="bottom", fontsize=9.5)

# C: pads, straight push, keep collar
ax = axs[2]; base(ax, "C  contact pads, straight push + collar", P["lug_w"], True)
for k, az0 in enumerate((90, 210, 330)):
    x, y = at(15.3, az0); ax.add_patch(Rectangle((x - 3.6, y - 3.6), 7.2, 7.2, angle=0, color=C["pad"], lw=0))
    for dx in (-1.9, 0, 1.9):
        for dy in (-1.4, 1.4):
            ax.add_patch(Circle((x + dx, y + dy), 0.75, color=C["pin"], lw=0))
signal_pins(ax, False)
ax.text(0, -30, "phases: 3 gold pads, 6 spring pins each\nno insertion force, no barrels to line up\nlocks: collar as today (hand cannot turn on pads)",
        ha="center", va="bottom", fontsize=9.5)

# D: one moulded 3-pole plug (MR60), straight push + collar
ax = axs[3]; base(ax, "D  one Amass MR60 plug, straight push + collar", P["lug_w"], True)
ax.add_patch(FancyBboxPatch((-7.7, 11.5), 15.4, 6.5, boxstyle="round,pad=0.3", color=C["mr"], lw=0))
for dx in (-4.4, 0, 4.4):
    ax.add_patch(Circle((dx, 14.75), 1.75, color="white", lw=0)); ax.add_patch(Circle((dx, 14.75), 1.2, color=C["bullet"], lw=0))
signal_pins(ax, False)
ax.text(0, -30, "phases: one off-the-shelf 3-pole plug on a floating mount\n(3.5 mm pins, 60 A class), off centre\nlocks: collar as today",
        ha="center", va="bottom", fontsize=9.5)

# comparison table
rows = [
    ("", "A  bullets + collar (today)", "B  rings, hand twists", "C  pads + collar", "D  MR60 plug + collar"),
    ("Phase current", "75 A per bullet", "6 x 7.3 = 44 A per ring cont.; 55 A peak short (est.)", "44 A per pad, same pins", "60 A class (Amass)"),
    ("Push-in force", "~60 N, all by hand (est.)", "~25 N of pin springs, taken by the twist ramp", "~25 N, straight push", "~30-40 N (est.)"),
    ("Alignment", "3 rigid pins + spigot: over-constrained, needs ±0.1 mm", "none: rings work at any angle", "none: pads 7 mm", "one plug must float"),
    ("Collar", "yes, 13.4 g + detent", "no: lugs lock on the socket's own lips (rev A style)", "yes", "yes"),
    ("Anti-rattle preload", "O-ring + shims", "the 23 pin springs (~25 N) do it", "pin springs", "O-ring"),
    ("Coupler mass, per arm + hand (est.)", "~48 g (41.7 g of PA-CF parts + bullets)", "~34-38 g (no collar; lips move onto the socket; 2 PCBs + 23 pins)", "~45 g", "~48 g"),
    ("Coupler length (axial)", "bullets 8.5 mm deep set the socket", "~3 mm shorter (no barrels)", "~3 mm shorter", "plug 15 mm deep"),
    ("Wear / life", "bullets: thousands of mates", "pins slide 60 deg on the rings each change: hard gold needed", "pins rated 1M cycles", "3000 mates"),
    ("Main risk", "known; heavy, stiff to push", "contact resistance of sliding pins; needs bench test", "keeps the collar's mass", "off-centre, bulky"),
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
fig.text(0.5, 0.445, "Masses and forces marked est. are first estimates; B's contact resistance under a 60 deg wipe is the one thing to bench-test first.",
         ha="center", fontsize=10.5, style="italic")
fig.savefig(OUT, dpi=90)
print("wrote", OUT)
