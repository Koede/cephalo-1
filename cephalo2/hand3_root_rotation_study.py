"""hand3 hub: each finger ROTATES round the hub on its own, all four at the same time, actuators under the hub, palm free.
Options study, 2026-11-20 (user: "individual control of the fingers rotating on the hub, not an overhaul on the grippers").

What sits under the hub rim today decides everything:
  - v3 curl drive (single pull ring): rotor-side pull ring r 18.3-30 z 78-89 + 4 STATIC yokes r 27.7-40.8 z 69-89, 36 deg each,
    + rods / tie ring r 36-40 below z 82.  The yokes stay still while the rotor spins, so nothing rotor-side may pass r 27.7-40.8
    at z 69-89, and nothing static may reach a root except through the gaps between the yokes.
  - D build (per-finger winch curl, built today): no pull ring, no yokes; each root already has a leg down to z 66 at r 23-29.6;
    8 static drivers below z 65.
Top views to scale (mm), flight-cross pose, rotor parked.  Numbers -> hand3_root_rotation_study.json.  "est." = estimate."""
import os, sys, json, itertools
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle, Rectangle, Polygon
import hand3_hub_v3 as V3, hand3_hubc as HC, hand3_multidigit as MD

HERE = os.path.dirname(os.path.abspath(__file__))
W = 1309.0
POSES = {"cross": (0, 90, 180, 270), "2+2": (30, 150, 210, 330), "3+1": (30, 90, 150, 270), "one side": (0, 60, 120, 180)}
MICRO = dict(name="Pololu 2359 700:1 sub-micro plastic planetary gearmotor 6D x 21L", m=1.3, rpm=90, stall_Nm=12 * 0.00706,
             gearbox_limit_Nm=250e-5 * 9.81, A_stall=0.40)                     # pololu.com/product/2359 (250 g cm gearbox limit)
PINION_R = 1.8                                                                   # m0.3 z12 brass pinion, 2 mm bore (est.)
NEED_N = 3.0                                                                     # push to slide a root on its ribs when parked (est.)
GRASP = {"A": (30, 110), "B": (30, 60), "C": (30, 95), "D": (30, 110), "F": (30, 110)}    # hand3_grasp_room_study (G = no towers)
STATION_G = 9.0 + 9.5 + 6.0                                                      # SG92R + roller N20 + blade/tyre/sensor (est.)
YOKE_AZ = [(16, 52), (128, 164), (194, 230), (310, 346)]                         # V3.yoke() extents
BLADE_WIN = [(0, 5), (65, 115), (175, 180), (240, 300)]                          # hand3_multi_station_study: blade clears the drive


def spin_N(m_g, r_mm):
    return m_g * 1e-3 * r_mm * 1e-3 * W ** 2


def shift_moves(a0, a1):
    """Simultaneous move: the whole pattern may turn, pick the shift with the smallest largest move."""
    best = (1e9, 0)
    for sh in range(360):
        d = [((b + sh - a + 180) % 360) - 180 for a, b in zip(a0, a1)]
        best = min(best, (max(abs(x) for x in d), sh))
    return best[0]


def travelling_stations():
    """Option D: 4 static stations, each carrying one root; they can only reach a root inside a blade window.  A pose is reachable
    in one simultaneous move if some turn of the pattern puts every root in a window, one station per window (stations cannot
    cross the yokes)."""
    def ok(p):
        for sh in range(0, 360):
            az = sorted((a + sh) % 360 for a in p)
            wins = []
            for a in az:
                w = [i for i, (lo, hi) in enumerate(BLADE_WIN) if lo <= a <= hi]
                if not w: break
                wins.append(w[0])
            else:
                if len(set(wins)) == 4: return sh
        return None
    return {k: ok(p) for k, p in POSES.items()}


def numbers():
    o = {}
    T = min(MICRO["gearbox_limit_Nm"], 0.25 * MICRO["stall_Nm"])                 # continuous rule of thumb vs gearbox limit
    push = T / (PINION_R * 1e-3)
    o["micro"] = dict(name=MICRO["name"], usable_Nm=round(T, 4), push_N=round(push, 1), need_N=NEED_N, margin=round(push / NEED_N, 1))
    # B as built (C decision): numbers from hand3_root_drive_check / BOM
    o["B"] = dict(mass_g=+109, move_s=1.2, spin_N=624)
    # C: micro on the root at r 30 standing up (grasp study E), pinion on a gear rim of the top disc (pitch r ~27)
    gear_r = 27.0; deg_s = np.degrees(MICRO["rpm"] * 0.75 / 60 * 2 * np.pi * PINION_R / gear_r)
    mv = shift_moves(POSES["cross"], POSES["2+2"])
    o["C"] = dict(deg_s=round(deg_s, 1), cross_to_2p2_s=round(mv / deg_s, 1), spin_N=round(spin_N(2.1, 30)),
                  mass_g=round(4 * (1.3 + 0.3 + 0.5) + 6.0 - STATION_G, 1), height_above_palm=21 - 5)
    # D: travelling stations
    reach = travelling_stations()
    o["D"] = dict(one_move=reach, mass_g=round(4 * (1.3 + 3.0 + 1.0) + 10.0 + 15.0 - STATION_G, 1),
                  note="with one station per gap between the yokes only the cross pose fits; 2+2, 3+1 and one side need re-grabs")
    # E: micro hanging under each root, v3 curl drive kept
    o["E"] = dict(leg_r=(23.0, 29.6), leg_z=(66, 91), yoke_r=(27.7, 40.8), yoke_z=(69, 89), pull_ring_r=(18.3, 30.0), pull_ring_z=(78, 89),
                  verdict="leg passes through the rotor-side pull ring and sweeps through the static yokes in flight")
    # F: D build kept + micro on each winch leg, pinion on a gear flange round the cage
    mot = np.array([26.0, 6.6]); rm = float(np.hypot(*mot)); gear_pitch = rm - PINION_R
    deg_sF = np.degrees(MICRO["rpm"] * 0.75 / 60 * 2 * np.pi * PINION_R / gear_pitch)
    o["F"] = dict(motor_xy=mot.tolist(), motor_r=round(rm, 1), gear_pitch_r=round(gear_pitch, 1), pinion_z=(66.0, 68.0),
                  key_top_retracted=MD.KEY["top_retracted"], pinion_over_keys_mm=66.0 - MD.KEY["top_retracted"],
                  deg_s=round(deg_sF, 1), cross_to_2p2_s=round(mv / deg_sF, 1),
                  leg_extra_spin_N=round(spin_N(2.1, rm)), leg_now_N=111,
                  mass_g=round(4 * (1.3 + 0.3 + 0.5) + 6.0 - STATION_G, 1),
                  power="5 rotor-side copper tracks under the cage flange (common + 1 per motor), phosphor-bronze wiper on each leg; "
                        "5 spring pogo pins on the lift ring touch the tracks only while parked (est.)")
    # G: micro lying low beside each grip, v3 curl drive kept
    o["G"] = dict(room_mm=91.0 - 89.0, pinion_d_min=2 * PINION_R + 0.6, verdict="2 mm between the pull-ring rail top (z 89) and the base disc (z 91)")
    o["A"] = dict(cross_to_2p2_s=15.6, mass_g=0)
    o["move_deg_cross_to_2p2"] = mv
    return o


def hub(ax, yokes=True, ring=True, legs=False, azs=POSES["cross"]):
    ax.add_patch(Circle((0, 0), 25.5, fc="#ddd", ec="#888", lw=0.6))
    if ring: ax.add_patch(Wedge((0, 0), 30.0, 0, 360, width=11.7, fc="#f3c9a0", ec="none", alpha=0.6))
    if yokes:
        for a0, a1 in YOKE_AZ: ax.add_patch(Wedge((0, 0), 40.8, a0, a1, width=13.1, fc="#c33", ec="k", lw=0.4, alpha=0.55))
    for a in azs:
        u = np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]); n = np.array([-u[1], u[0]])
        p = [9 * u - 4 * n, 24.3 * u - 11 * n, 36 * u - 6 * n, 60 * u - 10 * n, 60 * u + 10 * n, 36 * u + 6 * n, 24.3 * u + 11 * n, 9 * u + 4 * n]
        ax.add_patch(Polygon(p, fc="#7fa7d9", ec="#335", lw=0.5))
        if legs: ax.add_patch(Polygon([23 * u - 3.3 * n, 29.6 * u - 3.3 * n, 29.6 * u + 3.3 * n, 23 * u + 3.3 * n], fc="#556", ec="k", lw=0.4))


def micro_on(ax, a, x, y, col="#2a7"):
    u = np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]); n = np.array([-u[1], u[0]])
    ax.add_patch(Circle(tuple(x * u + y * n), 3.0, fc=col, ec="k", lw=0.5))


def figure(N):
    opts = [("A", "Keep v3: one station,\none root at a time", "k"),
            ("B", "As built (C): worm tower\non each root (palm)", "k"),
            ("C", "Micro motor standing on\neach root, v3 curl drive", "k"),
            ("D", "4 travelling stations under\nthe hub, v3 curl drive", "#c33"),
            ("E", "Micro motor hanging under\neach root, v3 curl drive", "#c33"),
            ("F", "Micro motor on each root's\nwinch leg (D curl kept)", "#2a7"),
            ("G", "Micro motor low beside\neach grip, v3 curl drive", "#c33")]
    fig = plt.figure(figsize=(25, 12.5))
    for i, (k, t, col) in enumerate(opts):
        ax = fig.add_axes([0.005 + i * 0.142, 0.5, 0.138, 0.4]); ax.set_aspect("equal"); ax.set_xlim(-66, 66); ax.set_ylim(-66, 66)
        ax.set_xticks([]); ax.set_yticks([])
        hub(ax, yokes=k != "F", ring=k != "F", legs=k == "F")
        if k == "A":
            ax.add_patch(Rectangle((-5.5, 27), 11, 22, fc="#e07b39", ec="k", lw=0.5)); ax.text(0, 52, "station", ha="center", fontsize=7)
        if k == "B":
            for a in POSES["cross"]: micro_on(ax, a, 17, 0, "#8a8f98")
            ax.text(0, -62, "33 mm worm towers stand in the palm", ha="center", fontsize=7)
        if k == "C":
            for a in POSES["cross"]: micro_on(ax, a, 30, -5)
            ax.text(0, -62, "6 x 21 mm motors stand 16 mm over the palm at r 30", ha="center", fontsize=7)
        if k == "D":
            for a in POSES["cross"]:
                u = np.array([np.cos(np.radians(a)), np.sin(np.radians(a))])
                ax.add_patch(Circle(tuple(48 * u), 4, fc="#e07b39", ec="k", lw=0.5))
            ax.add_patch(Wedge((0, 0), 56, 0, 360, width=10, fc="none", ec="#e07b39", lw=1))
            for lo, hi in BLADE_WIN: ax.add_patch(Wedge((0, 0), 44, lo, hi, width=3, fc="#2a7", ec="none"))
            ax.text(0, -62, "reach only through green windows (between yokes)", ha="center", fontsize=7)
        if k == "E":
            for a in POSES["cross"]:
                u = np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]); n = np.array([-u[1], u[0]])
                ax.add_patch(Polygon([23 * u - 3.3 * n, 29.6 * u - 3.3 * n, 29.6 * u + 3.3 * n, 23 * u + 3.3 * n], fc="#556", ec="k", lw=0.4))
                micro_on(ax, a, 26, 6.6)
            ax.text(0, -62, "legs cut the pull ring + sweep the yokes", ha="center", fontsize=7, color="#c33")
        if k == "F":
            ax.add_patch(Wedge((0, 0), N["F"]["gear_pitch_r"] + 0.6, 0, 360, width=4.0, fc="#f0a030", ec="k", lw=0.4, alpha=0.7))
            for a in POSES["cross"]: micro_on(ax, a, *N["F"]["motor_xy"])
            for a in MD.DRIVER_AZ:
                u = np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]); ax.plot(*(28.2 * u), "s", ms=3, color="#444")
            ax.text(0, -62, "gear flange on the cage (orange), z 66-68, under the base disc", ha="center", fontsize=7)
        if k == "G":
            for a in POSES["cross"]: micro_on(ax, a, 30, -15)
            ax.text(0, -62, "pinion needs 4.2 mm, only 2 mm under the disc", ha="center", fontsize=7, color="#c33")
        ax.set_title(f"{k}: {t}", fontsize=10, color=col, fontweight="bold" if col == "#2a7" else "normal")
        ax.plot([40, 60], [60, 60], "k", lw=2); ax.text(50, 62, "20 mm", ha="center", fontsize=7)
    m = N["micro"]; C, D, F = N["C"], N["D"], N["F"]
    reach = ", ".join(f"{p}: {'yes' if v is not None else 'no'}" for p, v in D["one_move"].items())
    rows = [["", "A keep v3", "B as built (C)", "C micro on root", "D travelling stations", "E micro hanging", "F micro on winch leg", "G micro low beside grip"],
            ["each root on its own,\nall at once", "no (one by one)", "yes", "yes", "only some poses\n(" + reach + ")", "-", "yes", "-"],
            ["actuators", "1 station under", "4 worm N20 on roots", "4 x Pololu 2359 on roots", "4 stations under + lift ring", "4 x 2359 under roots", "4 x Pololu 2359 under the base disc\n(+ D's 8 curl drivers)", "4 x 2359 at the rim"],
            ["palm (balls held, mm)", "free (30-110)", "towers (30-60)", "low towers (30-95)", "free (30-110)", "free", "free (30-110)", "free"],
            ["finger curl drive", "v3 pull ring\n(all fingers together)", "v3 pull ring", "v3 pull ring\n(all together)", "v3 pull ring", "v3 pull ring", "D winches (each finger\non its own) - as built today", "v3 pull ring"],
            ["cross -> 2+2", f"{N['A']['cross_to_2p2_s']} s", f"{N['B']['move_s']} s", f"{C['cross_to_2p2_s']} s ({C['deg_s']} deg/s)", "needs a re-grab", "-", f"{F['cross_to_2p2_s']} s ({F['deg_s']} deg/s)", "-"],
            ["drive force", "-", "-", f"{m['push_N']} N vs ~{m['need_N']} N (est.)", "-", "-", f"{m['push_N']} N vs ~{m['need_N']} N (est.)", "-"],
            ["mass vs its base design", "0", f"+{N['B']['mass_g']} g", f"{C['mass_g']:+.0f} g (est., station gone)", f"{D['mass_g']:+.0f} g (est.)", "-", f"{F['mass_g']:+.0f} g (est., station gone)", "-"],
            ["kill check", "fails the ask", "palm blocked", "motors stand 16 mm proud\nof the palm at r 30", "KILL: yokes leave 4 windows;\n2+2 and 3+1 can't be reached in one move",
             "KILL: leg cuts the pull ring and\nsweeps the static yokes in flight", f"pinion {F['pinion_over_keys_mm']:.1f} mm over the retracted\nkeys; leg {F['leg_now_N']}+{F['leg_extra_spin_N']} N spin: SF 2.06 -> ~1.1\nunless its foot bears on the flange (est.)",
             "KILL: 2 mm under the disc,\npinion needs 4.2"]]
    ax = fig.add_axes([0.005, 0.01, 0.99, 0.46]); ax.axis("off")
    t = ax.table(cellText=rows, loc="center", cellLoc="center"); t.auto_set_font_size(False); t.set_fontsize(8.2); t.scale(1, 2.75)
    for r in range(len(rows)):
        t[r, 6].set_facecolor("#dff3e6")
        for c in (4, 5, 7): t[r, c].set_facecolor("#f6dada")
    fig.suptitle("hand3 - every finger ROTATES round the hub on its own, all at once, actuators under the hub, palm free.  Top views, "
                 "flight cross, to scale (mm).  Red = static curl yokes (v3).  Recommended: F (keeps today's per-finger curl); C if the v3 pull ring must stay",
                 fontsize=12)
    fig.savefig(os.path.join(HERE, "hand3_root_rotation_study.png"), dpi=100)


if __name__ == "__main__":
    N = numbers(); json.dump(N, open(os.path.join(HERE, "hand3_root_rotation_study.json"), "w"), indent=1, default=str)
    print(json.dumps(N, indent=1, default=str)); figure(N)
