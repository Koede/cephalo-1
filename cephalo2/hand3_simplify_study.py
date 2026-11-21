"""hand3 hub: simplification study (2026-11-20).  User on F3: "this seems very convoluted".

Why F3 got busy: the fingers are the rotor blades (12,500 rpm), so anything a finger needs must either ride the spinning hub or
reach it from the static side only when parked.  F3 does BOTH: curl from 8 static drivers that dock from below (keys, lift ring),
rotation from 4 motors on the roots (powered through a static track ring + wipers), and a two-stage lift to sequence them.
Options below cut that down.  Half sections (r-z) to scale, rotor-side parts blue, static parts orange, actuators red.
Numbers -> hand3_simplify_study.json.  est. = estimate."""
import os, sys, json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import hand3_rotation_f3 as F, hand3_multidigit as MD

HERE = os.path.dirname(os.path.abspath(__file__))
W = 1309.0
def spin(m, r): return m * 1e-3 * r * 1e-3 * W ** 2

N20 = 9.5; MICRO = 1.3
OPTS = {
 "S0": dict(t="F3 as built", rot="own motor per root, any angle", curl="own winch per finger (8 static drivers)",
            act_rotor=4, act_static=8 + 1, rotor_g=4 * (MICRO + 0.25), static_g=8 * 9.5 + 9 + 4 * 1.09 + 2.7,
            contacts="5-track ring + 8 wipers", lift="2-stage", mech_parts=4 + 4 + 8 + 8 + 1 + 1 + 4 + 4 + 8 + 4 + 4,
            risk="many interfaces; keys must find the slots", verdict="reference"),
 "S1": dict(t="Every motor rides its root\n(no static drivers)", rot="own motor per root, any angle", curl="own motor on each winch screw",
            act_rotor=8, act_static=1, rotor_g=4 * (MICRO + 0.25 + N20 + 0.4), static_g=9 + 4 * 1.09 + 3.5,
            contacts="9-track ring + 16 wipers", lift="1 stage (tracks only)", mech_parts=4 + 4 + 4 + 4 + 4 + 1 + 1 + 4 + 16,
            risk=f"+{4 * N20:.0f} g spinning at r ~36: {spin(N20 + 0.4, 36):.0f} N more per leg (now ~200 N)", verdict="KILL as is: leg needs ~3x section"),
 "S2": dict(t="S1 + one board on the rotor,\n3-pin dock at park", rot="own motor per root, any angle", curl="own motor on each winch screw",
            act_rotor=8, act_static=1, rotor_g=4 * (MICRO + 0.25 + N20 + 0.4) + 4.0, static_g=9 + 1.0,
            contacts="3 pogo pins at one spot + 4 flex loops in the hub", lift="1 stage (dock only)", mech_parts=4 + 4 + 4 + 4 + 4 + 1 + 1 + 4 + 1,
            risk="electronics and flex loops at 12,500 rpm (3,600 g at r 20)", verdict="fewest parts, riskiest"),
 "S3": dict(t="Cam sleeve: one screw\nmoves all roots to presets", rot="presets only (cross / 3+1 / 2+2 / one side), all at once",
            curl="own winch per finger (8 static drivers)", act_rotor=0, act_static=9 + 1, rotor_g=6.0, static_g=9 * 9.5 + 9,
            contacts="none (keys only)", lift="1 stage (keys)", mech_parts=1 + 1 + 4 + 9 + 9 + 1 + 4 + 4 + 4,
            risk="not free angles; slot wear; order of presets fixed", verdict="simplest rotation, but presets"),
 "S4": dict(t="Cam sleeve + v3 single\npull-ring curl", rot="presets only, all at once", curl="all fingers together (2 STS3215)",
            act_rotor=0, act_static=1 + 2 + 1, rotor_g=6.0 + 12.0, static_g=9.5 + 2 * 21 + 9,
            contacts="none", lift="1 stage (sleeve key)", mech_parts=1 + 1 + 4 + 1 + 1 + 1 + 4 + 8 + 1 + 7 + 2,
            risk="gives up independent curl and free angles", verdict="simplest overall"),
 "S5": dict(t="S1 but rotation only\n(v3 pull-ring curl back)", rot="own motor per root, any angle", curl="all fingers together (2 STS3215)",
            act_rotor=4, act_static=2 + 1, rotor_g=4 * (MICRO + 0.25), static_g=2 * 21 + 9 + 3,
            contacts="KILL: yokes fill the space under the rim (see rotation study)", lift="-", mech_parts=0,
            risk="no room for tracks / legs under the v3 yokes", verdict="KILL"),
}


def draw(ax, k):
    def R(x0, z0, x1, z1, c, a=1.0, lab=None):
        ax.add_patch(Rectangle((x0, z0), x1 - x0, z1 - z0, fc=c, ec="k", lw=0.4, alpha=a))
        if lab: ax.text((x0 + x1) / 2, (z0 + z1) / 2, lab, ha="center", va="center", fontsize=5.5)
    BL, OR, RD = "#9db7d9", "#f2c08a", "#d9534f"
    R(0, 91, 25.5, 93, BL); R(0, 99, 25.5, 101, BL); R(9, 93, 36, 99, BL, lab="root"); R(36, 93.5, 60, 98.5, "#555")
    R(19.5, 59.5, 21.5, 91, BL); R(0, 58.5, 17, 89, "#666", 0.5); R(0, 0, 38, 58, OR, 0.5, "frame")
    if k in ("S0", "S1", "S2", "S3"):
        R(23, 66, 30.8, 93, BL, 0.7, "leg"); R(27.6, 67.5, 28.8, 92, "#888")
    if k in ("S0",):
        R(22.9, 66.8, 28.9, 87.8, RD, lab="rot\nmotor"); R(34.6, 62, 40.6, 63.5, OR, lab=""); ax.text(41, 62, "tracks", fontsize=5.5)
        R(26.7, 30, 29.7, 63.5, RD, 0.8); ax.text(30, 45, "8 curl\ndrivers", fontsize=5.5); R(21.5, 22.5, 35.5, 24.5, OR)
    if k in ("S1", "S2"):
        R(22.9, 66.8, 28.9, 87.8, RD, lab="rot"); R(31, 56, 41, 68, RD, lab="curl N20\n(on leg)")
        if k == "S1": R(42, 60, 48, 61.5, OR); ax.text(42, 57.5, "9 tracks", fontsize=5.5)
        else: R(22, 60.5, 30, 64, "#2a7", lab="board"); R(36, 52, 39, 58, OR, lab="pogo")
    if k in ("S3", "S4"):
        R(21.6, 66, 22.9, 90, "#2a7", lab=""); ax.text(10, 75, "cam\nsleeve", fontsize=5.5, color="#2a7")
        R(17.5, 30, 20.5, 59, RD, 0.8); ax.text(4, 40, "sleeve\ndriver", fontsize=5.5)
    if k == "S3":
        R(26.7, 30, 29.7, 63.5, RD, 0.8); ax.text(30, 45, "8 curl\ndrivers", fontsize=5.5); R(21.5, 22.5, 35.5, 24.5, OR)
    if k in ("S4", "S5"):
        R(24.3, 78, 30, 89, BL, 0.8, "pull\nring"); R(27.7, 69, 40.8, 89, OR, 0.6, "yokes"); R(30, 5, 38, 40, RD, 0.8, "2x\nSTS3215")
    if k == "S5":
        ax.text(30, 100, "no room", color="#c33", fontsize=8)
    ax.set_xlim(0, 62); ax.set_ylim(0, 104); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])


def figure():
    fig = plt.figure(figsize=(24, 13))
    ks = list(OPTS)
    for i, k in enumerate(ks):
        ax = fig.add_axes([0.005 + i * 0.166, 0.5, 0.16, 0.42]); draw(ax, k)
        col = "#2a7" if k == "S3" else ("#c33" if k in ("S1", "S5") else "k")
        ax.set_title(f"{k}: {OPTS[k]['t']}", fontsize=10, color=col, fontweight="bold" if k == "S3" else "normal")
    rows = [[""] + ks]
    for lbl, f in [("finger rotation", lambda o: o["rot"]), ("finger curl", lambda o: o["curl"]),
                   ("actuators (rotor / static)", lambda o: f"{o['act_rotor']} / {o['act_static']}"),
                   ("power onto the rotor", lambda o: o["contacts"]), ("lift", lambda o: o["lift"]),
                   ("moving mechanical parts (approx.)", lambda o: str(o["mech_parts"]) if o["mech_parts"] else "-"),
                   ("added rotor mass / static mass", lambda o: f"{o['rotor_g']:.0f} g / {o['static_g']:.0f} g (est.)"),
                   ("main risk", lambda o: o["risk"]), ("verdict", lambda o: o["verdict"])]:
        rows.append([lbl] + [f(OPTS[k]).replace(", ", ",\n", 1) if len(f(OPTS[k])) > 30 else f(OPTS[k]) for k in ks])
    ax = fig.add_axes([0.005, 0.01, 0.99, 0.47]); ax.axis("off")
    t = ax.table(cellText=rows, loc="center", cellLoc="center"); t.auto_set_font_size(False); t.set_fontsize(8.3); t.scale(1, 3.0)
    for r in range(len(rows)):
        t[r, 4].set_facecolor("#dff3e6"); t[r, 6].set_facecolor("#f6dada"); t[r, 2].set_facecolor("#f6dada")
    fig.suptitle("hand3 hub - simpler ways to get the same hand.  Half sections r-z, to scale (mm): blue = spins with the rotor, "
                 "orange = static, red = actuators.  Recommended: S3 if 4 grip layouts are enough; otherwise F3 stays", fontsize=12.5)
    fig.savefig(os.path.join(HERE, "hand3_simplify_study.png"), dpi=100)


if __name__ == "__main__":
    json.dump(OPTS, open(os.path.join(HERE, "hand3_simplify_study.json"), "w"), indent=1)
    figure(); print("ok")
