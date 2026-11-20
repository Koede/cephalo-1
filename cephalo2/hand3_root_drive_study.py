"""hand3 hub v3: move every finger root on its own and at the same time (not one by one through the station).  Options study,
2026-11-20.  Top views to scale (mm), rotor parked.  Numbers -> hand3_root_drive_study.json.

Requirements:
  - each of the 4 roots moves round the hub by its own amount, all at once; order kept (roots can't pass), MIN_SEP 58 deg;
  - in flight (12,500 rpm) each root held against its spin load (finger 826 N radial, carried by the disc ribs as now) and the
    tangential load from spin-up (rotor torque 0.62 N m / 4 at r ~25 -> ~6 N, est.);
  - no slip ring at 12,500 rpm: anything electric on the rotor gets power only while parked;
  - stock parts (Adafruit fair game); every gram counts (arm servo is over its rating, hand3-mass-study);
  - today: cross -> 2+2 takes 4 single moves x 3.9 s = 15.6 s; 3+1 -> 2+2 3 moves = 11.7 s (hand3_v3_sim)."""
import os, sys, json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle, Rectangle, Polygon, FancyArrowPatch
import hand3_hub_v3 as V3

HERE = os.path.dirname(os.path.abspath(__file__))
W = 1309.0                              # rad/s
G = 9.81
DISC_R = V3.V["disc_r"]; BR = V3.V["block_r"]; BH = V3.V["block_half"]
POSES = V3.POSES
# --- bought parts (datasheets / shop pages)
WORM = dict(name="N20 worm gearmotor 1220WG-N20VA (E-S Motor, RobotShop) / CHF-GW12T-N20VA (Chihai)", m=13.0, gearbox=(18, 12, 10),
            L=33.0, rpm=94, stall_kgcm=2.4, stall_A=0.65, self_lock=True)                  # 13 g: Chihai page, "approx."
SOL = dict(name="Adafruit 2776 mini push-pull solenoid 5 V", m=12.6, throw=3.0, force_N=0.08 * G, A=1.1)
SG92R = dict(m=9.0); N20 = dict(m=9.5)
DOCK = "Adafruit 5469 magnetic connector, 8 pogo contacts, 2.2 mm pitch"


def spin_pull(m_g, r_mm):
    return m_g * 1e-3 * r_mm * 1e-3 * W ** 2


def move_time_simultaneous(a0, a1, deg_s):
    """All roots at once; the whole pattern may turn freely, so pick the shift that minimises the largest move."""
    a0 = np.array(a0, float); a1 = np.array(a1, float); best = 1e9
    for sh in np.arange(0, 360, 1.0):
        d = (a1 + sh - a0 + 180) % 360 - 180
        best = min(best, np.abs(d).max())
    return best / deg_s, best


def numbers():
    out = {}
    # C: pinion m0.5 z12 (d 6) on a crown ring at r 24, worm output 94 rpm
    v = WORM["rpm"] / 60 * np.pi * 6.0; degs = np.degrees(v / 24.0)
    t1, m1 = move_time_simultaneous(POSES["flight cross"], POSES["2 + 2 pinch"], degs)
    t2, m2 = move_time_simultaneous(POSES["3 + 1 thumb"], POSES["2 + 2 pinch"], degs)
    F_hold = WORM["stall_kgcm"] * G * 1e-2 / 3e-3
    out["C"] = dict(root_speed_deg_s=round(degs, 1), cross_to_2p2_s=round(t1 + 1.0, 1), largest_move_deg=m1, thumb_to_2p2_s=round(t2 + 1.0, 1),
                    pull_N_per_motor=round(spin_pull(WORM["m"], 28.0)), root_total_N=round(826 + spin_pull(WORM["m"], 28.0)),
                    worm_hold_N=round(F_hold), spinup_N=6, added_rotor_g=4 * WORM["m"] + 3.0 + 2.0, removed_g=round(8 * 0.19 + 8 * 0.04 + 9.5 + 1.0 + 2.0, 1),
                    tower_drag_Nm=round(4 * 0.5 * 1.2 * (W * 0.028) ** 2 * 1.0 * (12 * 33e-6) * 0.028, 3))
    out["C"]["net_g"] = round(out["C"]["added_rotor_g"] - out["C"]["removed_g"], 1)
    # D: one hold ring + roller, a solenoid per root lifts its pins into the static hold ring; all move the same way
    t1d, _ = move_time_simultaneous(POSES["flight cross"], POSES["2 + 2 pinch"], 60.0)
    out["D"] = dict(pull_N_per_solenoid=round(spin_pull(SOL["m"], 22.0)), added_rotor_g=4 * SOL["m"] + 5.0, solenoid_force_N=round(SOL["force_N"], 2),
                    needed_lift_N="pins + crank spring ~1 N (est.)", cross_to_2p2_s=round(t1d + 2.0, 1))
    # E: four nested gear rings under the hub (one per root), static N20s
    out["E"] = dict(ring_g_each="~5 (est.)", rings_r=(32, 36, 40, 44), static_g=4 * N20["m"] + SG92R["m"] + 3.0, rotor_g=20 + 4,
                    clash="ring stack z 76..90 is where the pull ring (z 83..86), yoke lips and station live")
    # F: motor inside a taller root block between the discs
    out["F"] = dict(radial_room_mm=round(BR[1] - BR[0], 1), worm_L=WORM["L"], gap_now=6.0, gap_needed=WORM["gearbox"][1] + 1.0,
                    pull_N_per_motor=round(spin_pull(WORM["m"], 16.0)))
    out["B"] = dict(cross_to_2p2_s=round(2 * 3.9, 1), added_g=SG92R["m"] + 2.0 + 0.3)
    out["A"] = dict(cross_to_2p2_s=15.6, thumb_to_2p2_s=11.7)
    return out


OPTS = [("A", "Keep: one station,\none root at a time", "k"), ("B", "Two stations (180 deg):\ntwo roots at a time", "k"),
        ("C", "Worm gearmotor on each root,\npinion on a crown ring", "#2a7"), ("D", "One drive, per-root solenoid\nselects 'hold' or 'ride'", "k"),
        ("E", "Four drive rings under the hub,\nstatic motors", "#c33"), ("F", "Motor inside a taller\nroot block", "#c33")]


def hub(ax, azs, hl=None):
    ax.add_patch(Circle((0, 0), DISC_R, fc="#e6e6e6", ec="#999", lw=0.6))
    for k, a in enumerate(azs):
        ax.add_patch(Wedge((0, 0), BR[1], a - BH, a + BH, width=BR[1] - BR[0], fc="#7fa7d9" if k != hl else "#f0a030", ec="#335", lw=0.5))
        c, s = np.cos(np.radians(a)), np.sin(np.radians(a))
        ax.add_patch(Polygon([[c * 24 - s * 5, s * 24 + c * 5], [c * 70 - s * 5, s * 70 + c * 5], [c * 70 + s * 5, s * 70 - c * 5], [c * 24 + s * 5, s * 24 - c * 5]],
                             fc="#2b2b30", ec="none", alpha=0.85))


def figure(N):
    fig = plt.figure(figsize=(22, 11.5))
    azs = POSES["flight cross"]
    for i, (key, title, col) in enumerate(OPTS):
        ax = fig.add_axes([0.01 + i * 0.165, 0.50, 0.16, 0.40]); ax.set_aspect("equal"); ax.set_xlim(-62, 62); ax.set_ylim(-62, 62)
        ax.set_xticks([]); ax.set_yticks([])
        hub(ax, azs, hl=1 if key in ("A", "B") else None)
        st = lambda a, c="#f0a030": ax.add_patch(Wedge((0, 0), 48, a - 11, a + 11, width=48 - 30, fc=c, ec="k", lw=0.5, alpha=0.6))
        if key == "A":
            st(90); ax.text(0, -58, "station at 90 deg; each root: align, engage,\nindex, release = 3.9 s; others wait", ha="center", fontsize=7)
        if key == "B":
            st(90); st(270); ax.text(0, -58, "second blade + SG92R at 270;\nonly opposite roots move together", ha="center", fontsize=7)
        if key == "C":
            ax.add_patch(Circle((0, 0), 24.0, fc="none", ec="#2a7", lw=2.2, ls=(0, (1, 1))))
            for a in azs:
                c, s = np.cos(np.radians(a)), np.sin(np.radians(a))
                p = np.array([c * 28 - s * 6, s * 28 + c * 6])
                ax.add_patch(Polygon([p + R2(a) @ q for q in ([-5, -6], [5, -6], [5, 6], [-5, 6])], fc="#2a7", ec="k", lw=0.5))
            ax.add_patch(Rectangle((-6, 30), 12, 6, fc="#e9b", ec="k", lw=0.5)); ax.text(0, 38, "dock (8 pogo)", ha="center", fontsize=6.5)
            ax.text(0, -58, "worm motor 12 x 10 x 33 standing on each root's\nouter post; tracks under the top disc", ha="center", fontsize=7)
        if key == "D":
            ax.add_patch(Circle((0, 0), 21.0, fc="none", ec="#a3a", lw=3, alpha=0.6)); st(90, "#ccc")
            for a in azs:
                c, s = np.cos(np.radians(a)), np.sin(np.radians(a))
                ax.add_patch(Circle((c * 22 + s * 0, s * 22), 2.2, fc="#a3a", ec="k", lw=0.5))
            ax.text(0, -58, "static hold ring (purple) over the top disc;\nroller turns the discs under the held roots", ha="center", fontsize=7)
        if key == "E":
            for r in (32, 36, 40, 44): ax.add_patch(Circle((0, 0), r, fc="none", ec="#c33", lw=1.5))
            ax.text(0, -58, "4 nested rings + hangers round the disc rim;\nclash with pull ring, yokes, station", ha="center", fontsize=7, color="#c33")
        if key == "F":
            for a in azs:
                c, s = np.cos(np.radians(a)), np.sin(np.radians(a))
                p = np.array([c * 16.5, s * 16.5]); ax.add_patch(Polygon([p + R2(a) @ q for q in ([-16.5, -6], [16.5, -6], [16.5, 6], [-16.5, 6])], fc="none", ec="#c33", lw=1.5))
            ax.text(0, -58, f"33 mm motor vs {N['F']['radial_room_mm']} mm of block;\ngap 6 -> 13 mm: discs, pins, cranks all redone", ha="center", fontsize=7, color="#c33")
        ax.set_title(f"{key}: {title}", fontsize=10.5, color=col, fontweight="bold" if col == "#2a7" else "normal")
        ax.plot([35, 55], [55, 55], "k", lw=2); ax.text(45, 57, "20 mm", ha="center", fontsize=7)
    C, D, E, F, B = N["C"], N["D"], N["E"], N["F"], N["B"]
    rows = [["", "A keep", "B two stations", "C worm motor per root", "D solenoid select", "E 4 drive rings", "F motor in block"],
            ["independent + simultaneous", "no", "pairs only", "yes, any direction", "yes (all one way;\npattern turns freely)", "yes", "yes"],
            ["cross -> 2+2", "15.6 s", f"{B['cross_to_2p2_s']} s", f"~{C['cross_to_2p2_s']} s ({C['largest_move_deg']:.0f} deg at {C['root_speed_deg_s']:.0f} deg/s)",
             f"~{D['cross_to_2p2_s']} s", "~2 s (est.)", "~2 s (est.)"],
            ["flight lock", "latch pins", "latch pins", f"self-locking worm,\nholds {C['worm_hold_N']} N vs ~6 N", "latch pins", "latch pins", "worm"],
            ["mass change", "0", f"+{B['added_g']:.0f} g", f"+{C['added_rotor_g']:.0f} g rotor, -{C['removed_g']} g\n(pins, cranks, blade, roller)",
             f"+{D['added_rotor_g']:.0f} g rotor\n+ hold ring", f"+{E['static_g']:.0f} g static\n+{E['rotor_g']} g rings", "+52 g, hub redone"],
            ["spin load added per root", "0", "0", f"{C['pull_N_per_motor']} N (root {C['root_total_N']} N)", f"{D['pull_N_per_solenoid']} N", "rings: hoop stress", f"{F['pull_N_per_motor']} N"],
            ["power on the rotor", "none", "none", "8 copper tracks + brushes,\ndock while parked", "8 tracks + dock", "none", "tracks + dock"],
            ["parts to buy", "-", "SG92R", "4 worm N20, Adafruit 5469 dock,\n2 DRV8833, copper tape", "4 Adafruit 2776, dock,\ncopper tape", "4 N20 + SG92R", "4 worm N20 ..."],
            ["main risk", "fails the ask", "fails the ask", "N20 at ~4,900 g (bench\nspin test); +5 % rotor drag (est.)", "solenoid 0.8 N, 1.1 A each;\nheavier than C",
             "doesn't fit", "doesn't fit as is"]]
    ax = fig.add_axes([0.01, 0.01, 0.98, 0.44]); ax.axis("off")
    t = ax.table(cellText=rows, loc="center", cellLoc="center"); t.auto_set_font_size(False); t.set_fontsize(8.2); t.scale(1, 2.75)
    for r in range(len(rows)):
        t[r, 3].set_facecolor("#dff3e6")
        for c in (5, 6): t[r, c].set_facecolor("#f6dada")
    fig.suptitle("hand3 hub v3 - move every finger root on its own and at the same time.  Top views at the hub, to scale (mm).  Recommended: C",
                 fontsize=13)
    fig.savefig(os.path.join(HERE, "hand3_root_drive_study.png"), dpi=105)


def R2(a):
    c, s = np.cos(np.radians(a)), np.sin(np.radians(a)); return np.array([[c, -s], [s, c]])


if __name__ == "__main__":
    N = numbers(); json.dump(N, open(os.path.join(HERE, "hand3_root_drive_study.json"), "w"), indent=1, default=str)
    print(json.dumps(N, indent=1, default=str)); figure(N)
