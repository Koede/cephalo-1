"""hand3 hub v3: multi-digit control - every finger curls on its own, all at the same time, actuators under the hub, palm free
(user, 2026-11-20: "need multidigit control simultaneously"; earlier: "multiple stations underneath").  Options study.

Today all four flexor cords end on one base ring that both STS3215s pull, so the fingers curl together.
Needs per finger: 22.2 mm of cord, 55 N at stall margin (hand3_hubc: needed 54.9 N base / 49.9 N curl per finger);
roots must still travel round the hub (beads, station) and the rotor must spin with nothing static touching it.
Side views in the r-z plane at one root (to scale, mm, from hand3_hub_v3 / hand3_hubc numbers).  "est." marks estimates."""
import os, sys, json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import hand3_hub_v3 as V3, hand3_hubc as HC, hand3_design as H

HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
STS = dict(m=55.0, stall_Nm=2.94); SG92R = dict(m=9.0, stall_Nm=2.5 * 0.0981)
N20_30 = dict(name="Pololu 30:1 Micro Metal Gearmotor HPCB 6V", m=9.5, stall_Nm=0.6 * 0.0981 * 1.0, rpm=1000)       # est. from Pololu table
F_FINGER = 55.0; CORD = V3.CORD_STROKE


def numbers():
    o = {}
    # B/C: one ring per finger, 2:1 tackle -> ring force F/2 per finger; a servo per ring on the 4 mm spool
    F_ring = F_FINGER / 2
    o["ring_per_finger"] = dict(force_N=F_ring, STS3215_N=round(STS["stall_Nm"] / 4e-3), SG92R_N=round(SG92R["stall_Nm"] / 4e-3),
                                ring_stroke_mm=V3.RING_STROKE)
    o["B_stack_height_mm"] = 4 * (5.0 + V3.RING_STROKE)                           # flange+rail ~5 (est.) + stroke, per ring
    o["B_room_mm"] = round(H.Z["hub_lo"] - (V3.CAGE_TRACK[1] + 0.5), 1)            # from the cage track top to the base disc
    o["C_rail_r"] = [26.5, 29.5, 32.5, 35.5]
    o["C_blade_band"] = "station blade slides in at z 86..89.7 from r 26.9 out: rails at r >= 26.5 sit in it"
    # D: worm winch in each root + drivers under the parked roots
    drum_r = 2.5; worm_ratio = 20; eta = 0.4
    T_wheel = F_FINGER * drum_r * 1e-3; T_in = T_wheel / worm_ratio / eta
    cord_speed = N20_30["rpm"] / worm_ratio / 60 * 2 * np.pi * drum_r
    o["D"] = dict(drum_r=drum_r, worm_ratio=worm_ratio, T_in_Nm=round(T_in, 4), N20_30_stall_Nm=round(N20_30["stall_Nm"], 3),
                  cord_mm_s=round(cord_speed, 1), curl_s=round(CORD / cord_speed, 1),
                  positions=[0, 60, 90, 120, 180, 240, 270, 300],
                  covers="cross (0/90/180/270), 3+1 (0/60/120/240 = 30/90/150/270 turned), 2+2 (0/120/180/300), one side (0/60/120/180)",
                  removed=["base ring + rail", "4 yoke halves", "8 carbon rods", "tie ring", "7 MR63 idlers", "2 STS3215 + spools", "beads, 2:1 tackles"],
                  removed_g=round(2 * 55 + 5.0 + 7.1 + 4.8 + 2.3 + 2.1 + 0.6 + 0.1, 1),
                  added=["8 x Pololu 30:1 micro metal gearmotor (drivers)", "lift ring + 1 SG92R (engages all drivers)", "4 root winches: brass m0.5 worm + printed wheel + drum (est. 2.5 g)"],
                  added_g=round(8 * 9.5 + 9.0 + 3.0 + 4 * 2.5, 1))
    o["D"]["net_g"] = round(o["D"]["added_g"] - o["D"]["removed_g"], 1)
    return o


def draw_common(ax):
    ax.add_patch(Rectangle((0, V3.V["z0"]), V3.V["disc_r"], 2, fc="#ccc", ec="#888"))                       # base disc
    ax.add_patch(Rectangle((0, V3.Z_TOP[0]), V3.V["disc_r"], 2, fc="#ccc", ec="#888"))                       # top disc
    ax.add_patch(Rectangle((V3.V["block_r"][0], V3.Z_GAP[0]), V3.V["block_r"][1] - V3.V["block_r"][0], 6, fc="#7fa7d9", ec="#335"))
    ax.add_patch(Rectangle((24.3, H.Z["blade"] - 2.4), 12, 4.8, fc="#7fa7d9", ec="#335"))                       # grip
    ax.add_patch(Rectangle((36, H.Z["blade"] - 3), 30, 6, fc="#2b2b30", ec="none"))                              # phalanx 1
    ax.add_patch(Rectangle((0, H.Z["pack1"] + 8.5), 17, H.Z["motor_top"] - H.Z["pack1"] - 8.5, fc="#3a3f47", ec="none", alpha=0.6))   # motor bell
    ax.add_patch(Rectangle((HC.R_CAGE[0], V3.CAGE_TRACK[0]), HC.R_CAGE[1] - HC.R_CAGE[0], H.Z["hub_lo"] - V3.CAGE_TRACK[0], fc="#c9cfd6", ec="#888"))
    ax.add_patch(Rectangle((0, 0), 38, H.Z["pack1"], fc="#e9e9e4", ec="#999"))                                    # frame + servo pack
    ax.text(2, 28, "frame + servos", fontsize=7)


def figure(N):
    opts = [("A", "Now: one ring, both servos\n(all fingers together)", "#c33"),
            ("B", "4 pull rings stacked,\none servo each", "k"),
            ("C", "4 pull rings nested,\none servo each", "#c33"),
            ("D", "Winch in every root +\ndrivers underneath", "#2a7"),
            ("E", "2 rings (opposite pairs)", "k"),
            ("F", "4 push rods up a\nhollow motor shaft", "#c33")]
    fig = plt.figure(figsize=(22, 11.5))
    for i, (k, t, col) in enumerate(opts):
        ax = fig.add_axes([0.01 + i * 0.165, 0.47, 0.16, 0.43]); ax.set_aspect("equal"); ax.set_xlim(-2, 66); ax.set_ylim(-2, 112); ax.tick_params(labelsize=6)
        draw_common(ax)
        if k in ("A", "E"):
            ax.add_patch(Rectangle((19, 78), 11, 11, fc="#e07b39", ec="k", lw=0.5)); ax.add_patch(Rectangle((24.6, 86), 1.4, 4, fc="#111"))
            if k == "E": ax.add_patch(Rectangle((19, 64), 11, 11, fc="#e07b39", ec="k", lw=0.5, alpha=0.7))
            ax.plot([26.3, 26.3], [87, 96], "r", lw=1)
        if k == "B":
            for j in range(4): ax.add_patch(Rectangle((22, 89 - (j + 1) * 16.1), 8, 5, fc="#e07b39", ec="k", lw=0.5))
            ax.plot([26.3, 26.3], [25, 96], "r", lw=1)
            ax.text(31, 40, f"stack {N['B_stack_height_mm']:.0f} mm\nvs {N['B_room_mm']} mm free", fontsize=7, color="#c33")
        if k == "C":
            for r in N["C_rail_r"]: ax.add_patch(Rectangle((r - 1, 78), 2.4, 11, fc="#e07b39", ec="k", lw=0.5)); ax.plot([r, r], [89, 96], "r", lw=0.8)
            ax.add_patch(Rectangle((26.9, 86), 21, 3.7, fc="none", ec="#c33", lw=1.5, ls="--")); ax.text(30, 76, "station blade band", fontsize=7, color="#c33")
        if k == "D":
            ax.add_patch(Rectangle((25.8, 84), 5, 9, fc="#2a7", ec="k", lw=0.5)); ax.text(32, 87, "worm winch\n(self-locking)", fontsize=7)
            ax.add_patch(Rectangle((26, 60), 10, 22, fc="#8a8f98", ec="k", lw=0.5)); ax.text(37, 68, "driver N20\n(static, 8 of them)", fontsize=7)
            ax.add_patch(Rectangle((24, 58), 20, 1.5, fc="#f0a030", ec="k", lw=0.5)); ax.text(37, 55, "lift ring (1 SG92R)", fontsize=7)
            ax.plot([28.3, 28.3], [82, 84], "k", lw=2)
        if k == "F":
            ax.add_patch(Rectangle((0, 0), 2.5, 104, fc="#c33", alpha=0.4)); ax.text(3, 50, "SE 2816 shaft is solid dia 5", fontsize=7, color="#c33")
        ax.set_title(f"{k}: {t}", fontsize=10, color=col, fontweight="bold" if col == "#2a7" else "normal")
        ax.plot([45, 65], [107, 107], "k", lw=2); ax.text(55, 109, "20 mm", ha="center", fontsize=7)
    D = N["D"]; R = N["ring_per_finger"]
    rows = [["", "A now", "B 4 rings stacked", "C 4 rings nested", "D winch per root + drivers", "E 2 rings (pairs)", "F push rods in shaft"],
            ["each finger on its own, all at once", "no (all together)", "yes", "yes", "yes, while parked on a driver", "pairs only", "yes"],
            ["actuators", "2 STS3215 under", "4 servos under (SG92R gives\n%d N vs %.1f N needed)" % (R["SG92R_N"], R["force_N"]), "4 servos under",
             "8 Pololu 30:1 N20 drivers under\n+ 1 SG92R lift ring", "2 servos", "4 servos under"],
            ["palm", "free", "free", "free", "free", "free", "free"],
            ["roots travel", "yes (station)", "yes", "rails block the blade", "yes; blade band moved under\nthe winches (est.)", "yes", "spiders follow roots"],
            ["poses a finger can curl in", "all", "all", "all", "drivers at 0/60/90/120/180/240/270/300:\nall four poses", "all", "all"],
            ["curl speed (22 mm cord)", "~1 s", "~1 s", "~1 s", f"~{D['curl_s']} s (est.)", "~1 s", "~1 s"],
            ["mass change", "0", "+2 servos, bigger drive (est. +30 g)", "+2 servos (est. +30 g)", f"{D['net_g']:+.0f} g (est.)", "0", "-"],
            ["kill check", "fails the ask", f"stack {N['B_stack_height_mm']:.0f} mm in {N['B_room_mm']} mm:\nframe + cage redone", "KILL: rails at r 26.5-35.5\nfill the station blade band",
             "8 drivers must sit within\n+-1 deg of each parked root", "fails the ask", "KILL: no hollow-shaft motor\nof this size (SE 2816 solid shaft)"]]
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.42]); ax.axis("off")
    t = ax.table(cellText=rows, loc="center", cellLoc="center"); t.auto_set_font_size(False); t.set_fontsize(8.2); t.scale(1, 2.55)
    for r in range(len(rows)):
        t[r, 4].set_facecolor("#dff3e6")
        for c in (3, 6): t[r, c].set_facecolor("#f6dada")
    fig.suptitle("hand3 - multi-digit control: every finger curls on its own, at the same time, actuators under the hub, palm free.  "
                 "Side views at one root, to scale (mm).  Recommended: D", fontsize=12.5)
    fig.savefig(os.path.join(HERE, "hand3_multidigit_study.png"), dpi=100)


if __name__ == "__main__":
    N = numbers(); json.dump(N, open(os.path.join(HERE, "hand3_multidigit_study.json"), "w"), indent=1, default=float)
    print(json.dumps(N, indent=1, default=float)); figure(N)
