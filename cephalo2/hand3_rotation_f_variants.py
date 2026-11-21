"""hand3 hub, decision F (micro gearmotor on each root's winch leg): placement and implementation variants, 2026-11-20.
User: "explore minor placement tweaks and implementation tweaks to see if there is a better implementation".

Fixed by F: D's per-finger winch curl stays; each root gets a Pololu 2359 (700:1, 6 x 21 mm, 1.3 g) beside its leg; roots turn
round the hub on their own, all at once; static drivers + lift ring below; palm free.  Variants change WHERE the pinion meshes,
the leg section, the gearbox, how power gets onto the moving roots, and how a root finds its parking spot (the 8 curl drivers
need each root within +-1 deg).  Side views x-z in the root frame (x radial, z up), to scale (mm).  "est." = estimate."""
import os, sys, json
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "hand3e"))
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle, Polygon
import hand3_multidigit as MD, hand3_hub_v3 as V3, hand3_design as H

HERE = os.path.dirname(os.path.abspath(__file__))
W = 1309.0
POSES = {"cross": (0, 90, 180, 270), "2+2": (30, 150, 210, 330), "3+1": (30, 90, 150, 270), "one side": (0, 60, 120, 180)}
G700 = dict(name="Pololu 2359 700:1", rpm=90, stall=12 * 0.00706, A=0.40)
G136 = dict(name="Pololu 2358 136:1", rpm=500, stall=8 * 0.00706, A=0.40)        # pololu.com 136:1: 500 rpm, 8 oz-in at 6 V
GB_LIMIT = 250e-5 * 9.81                                                          # 250 g cm instantaneous gearbox limit (Pololu)
MOTOR = dict(d=6.0, L=21.0, m=1.3, shaft=2.7)
PIN = dict(m=0.3, z=12, w=2.0, g=0.3)                                              # brass m0.3 z12, 2 mm bore (est.)
NEED_N = 3.0                                                                       # est.
Z_ROOT = H.Z["blade"] - 2.0                                                        # leg root (into the grip)
LEG = MD.LEG; KEY = MD.KEY
CLAMP_G = 0.5


def spin(m_g, r):
    return m_g * 1e-3 * r * 1e-3 * W ** 2


def worst_move():
    best = {}
    for a, pa in POSES.items():
        for b, pb in POSES.items():
            if a >= b: continue
            m = 1e9
            for sh in range(360):
                d = [abs(((y + sh - x + 180) % 360) - 180) for x, y in zip(sorted(pa), sorted(pb))]
                m = min(m, max(d))
            best[f"{a} -> {b}"] = m
    return best


MOVES = worst_move()
WORST = max(MOVES.values())

VARIANTS = {
    "F1": dict(title="F as proposed: shaft down,\ngear flange on the cage", mesh="cage flange", motor_y=6.6, x_out=29.6, gear=G700,
               power="track ring on the lift ring (springs)", park="ripple count only", tidy=False),
    "F2": dict(title="Shaft UP: pinion on a gear\nskirt under the base disc", mesh="disc skirt", motor_y=6.6, x_out=29.6, gear=G700,
               power="track ring on the lift ring (springs)", park="ripple count only", tidy=False),
    "F3": dict(title="F2 + deeper leg + parking\nV-notch + pins/cranks gone", mesh="disc skirt", motor_y=6.6, x_out=30.8, gear=G700,
               power="track ring on the lift ring (springs)", park="ripple count + V-notch on the key", tidy=True),
    "F4": dict(title="F3 with the 136:1 gearbox\n(5x faster)", mesh="disc skirt", motor_y=6.6, x_out=30.8, gear=G136,
               power="track ring on the lift ring (springs)", park="ripple count + V-notch on the key", tidy=True),
    "F5": dict(title="F3, power by rotor tracks\non the cage + pogo block", mesh="disc skirt", motor_y=6.6, x_out=30.8, gear=G700,
               power="5 rotor tracks + static pogo block", park="ripple count + V-notch on the key", tidy=True),
    "F6": dict(title="F3, motor tucked in\n(x 24.1, r 25)", mesh="disc skirt", motor_y=6.6, x_out=30.8, gear=G700, tuck=True,
               power="track ring on the lift ring (springs)", park="ripple count + V-notch on the key", tidy=True),
}


def geom(v):
    """Motor axis, gear pitch radius, z bands for one variant."""
    tuck = v.get("tuck", False)
    rp = PIN["m"] * PIN["z"] / 2
    if tuck:
        x = 24.1
    else:
        x = 25.87
    r_ax = float(np.hypot(x, v["motor_y"])); r_gear = r_ax - rp
    if v["mesh"] == "cage flange":
        pin_z = (66.0, 68.0); body = (68.0 + 1.5, 68.0 + 1.5 + MOTOR["L"])
    else:
        pin_z = (88.5, 90.5); body = (pin_z[0] - 1.5 - MOTOR["L"], pin_z[0] - 1.5)
    return dict(x=x, r_ax=r_ax, r_gear=r_gear, rp=rp, pin_z=pin_z, body=body)


def numbers():
    out = {"moves_deg": MOVES, "worst_move_deg": WORST}
    S0 = 2 * LEG["y"] * (LEG["x_out"] - LEG["x_rim"]) ** 2 / 6
    d = MD.numbers(); F_leg = d["leg_pull_N"]; M_leg = F_leg * (Z_ROOT - 78.0)
    for k, v in VARIANTS.items():
        g = geom(v); gr = v["gear"]
        T = min(GB_LIMIT, 0.25 * gr["stall"]); push = T / (g["rp"] * 1e-3)
        deg_s = np.degrees(0.75 * gr["rpm"] / 60 * 2 * np.pi * g["rp"] / g["r_gear"])
        F_mot = spin(MOTOR["m"] + 0.3 + CLAMP_G, g["r_ax"]); zc = sum(g["body"]) / 2
        S = 2 * LEG["y"] * (v["x_out"] - LEG["x_rim"]) ** 2 / 6
        sig = (M_leg + F_mot * (Z_ROOT - zc)) / S
        key_top_stage1 = 63.5 + 1.5                                               # keys lowered 1.5, tracks touch at lift 1.5
        lowest = min(g["pin_z"][0], g["body"][0], LEG["z0"])
        o = dict(motor_axis_r=round(g["r_ax"], 1), gear_pitch_r=round(g["r_gear"], 2), gear_teeth=int(round(2 * g["r_gear"] / PIN["m"])),
                 pinion_z=g["pin_z"], motor_z=tuple(round(z, 1) for z in g["body"]),
                 clear_over_keys_mm=round(lowest - key_top_stage1, 1),
                 push_N=round(push, 1), push_margin=round(push / NEED_N, 1), root_deg_s=round(deg_s, 1), worst_move_s=round(WORST / deg_s, 2),
                 motor_spin_N=round(F_mot), leg_MPa=round(sig), leg_SF=round(230 / sig, 2))
        o["new_rotor_part"] = "gear flange round the cage, z 66-68" if v["mesh"] == "cage flange" else "gear skirt on the base disc's underside, z 88.5-91 (same printed part)"
        if v["mesh"] == "cage flange":
            o["flange_vs_leg"] = "leg bottom (x 23-25.6, z 66-69) must be cut back to x 25.7 to clear the flange teeth"
        if v["power"].startswith("5 rotor"):
            o["power_parts"] = "5 copper bands on the cage (z 70-80) + 5 bands z 60-65 joined through the cage, pogo block that must retract radially (est.)"
        else:
            o["power_parts"] = "printed ring r 34.6-40.5 with 5 copper-tape tracks, 3 springs on the lift ring, frame stop; 2 phosphor-bronze wipers per root"
        o["mass_g"] = round(4 * (MOTOR["m"] + 0.3 + CLAMP_G + 0.2) + (4.0 if not v["power"].startswith("5 rotor") else 5.5)
                            - (24.5 if True else 0) - (8 * 0.15 + 8 * 0.05 if v["tidy"] else 0) + (0.6 if v["x_out"] > 30 else 0), 1)
        if gr is G136:
            o["flight_hold"] = "136:1 back-drives easier; relies on rib friction under spin (est. 300 N vs 6 N)"
        else:
            o["flight_hold"] = "700:1 + rib friction under spin (est.)"
        out[k] = o
    out["electrics"] = dict(driver="Adafruit 2448 TB6612 (4 outputs, 15 V): each own-track 0 / 12 V, common track held at 6 V -> +-6 V per motor",
                            current_limit_A=round(0.40 * GB_LIMIT / G700["stall"], 2),
                            note="stall torque (85 mN m) is 3.5x the gearbox limit (24.5 mN m): limit each motor to ~0.11 A or it strips")
    out["park"] = "V-notch on each leg foot + chamfered driver key: key rising into the notch centres the root +-1.5 mm (+-3 deg) -> ripple count only needs +-3 deg"
    return out


def draw(ax, k, v, N):
    g = geom(v); o = N[k]
    ax.add_patch(Rectangle((0, 91), 25.5, 2, fc="#ccc", ec="#888"))                       # base disc
    ax.add_patch(Rectangle((0, 99), 25.5, 2, fc="#ccc", ec="#888"))                       # top disc
    ax.add_patch(Rectangle((9, 93), 15.3, 6, fc="#7fa7d9", ec="#335", lw=0.5))
    ax.add_patch(Rectangle((24.3, 93.6), 11.7, 4.8, fc="#7fa7d9", ec="#335", lw=0.5))
    ax.add_patch(Rectangle((31.8, 93), 25, 6, fc="#2b2b30", ec="none"))
    ax.add_patch(Rectangle((19.5, 59.5), 2, 31.5, fc="#c9cfd6", ec="#888", lw=0.5))         # cage
    ax.add_patch(Rectangle((0, 58.5), 17, 30.5, fc="#3a3f47", ec="none", alpha=0.5))        # motor bell
    xo = v["x_out"]
    ax.add_patch(Polygon([(25.8, Z_ROOT), (xo, Z_ROOT), (xo, LEG["z0"]), (23.0 if v["mesh"] != "cage flange" else 25.7, LEG["z0"]),
                          (23.0 if v["mesh"] != "cage flange" else 25.7, 87.5), (25.8, 87.5)], fc="#8a6f4e", ec="k", lw=0.5, alpha=0.85))
    ax.add_patch(Rectangle((24.4, 81), 5.2, 5, fc="#d9b26f", ec="k", lw=0.4))                # slider
    ax.plot([28.2, 28.2], [67.5, 92], color="#555", lw=1.5)                                  # screw
    ax.add_patch(Rectangle((g["x"] - 3, g["body"][0]), 6, MOTOR["L"], fc="#2a7", ec="k", lw=0.5, alpha=0.8))
    ax.add_patch(Rectangle((g["x"] - g["rp"] - 0.3, g["pin_z"][0]), 2 * (g["rp"] + 0.3), 2, fc="#b08d2b", ec="k", lw=0.4))
    rg = g["r_gear"]
    if v["mesh"] == "cage flange":
        ax.add_patch(Rectangle((21.5, 66), rg + 0.3 - 21.5, 2, fc="#f0a030", ec="k", lw=0.4))
    else:
        ax.add_patch(Rectangle((rg - 3.0, 88.5), 3.3, 2.5, fc="#f0a030", ec="k", lw=0.4))
    kt = 63.5
    ax.add_patch(Rectangle((26.7, kt - 30), 3.0, 30, fc="#bbb", ec="#777", lw=0.4)); ax.text(30.2, 50, "curl driver\n(static)", fontsize=6)
    if v["power"].startswith("5 rotor"):
        for j in range(5): ax.add_patch(Rectangle((21.5, 70 + 2 * j), 0.4, 1.2, fc="#c87533"))
        ax.add_patch(Rectangle((22.2, 60), 4, 5, fc="#e07b39", ec="k", lw=0.4)); ax.text(14, 62, "pogo", fontsize=6)
    else:
        ax.add_patch(Rectangle((34.6, 62.0), 5.9, 1.5, fc="#c87533", ec="k", lw=0.4)); ax.text(34.6, 59.5, "track ring", fontsize=6)
        ax.add_patch(Rectangle((xo, 66), 40.5 - xo, 0.8, fc="#8a6f4e", ec="k", lw=0.3))
        for xx in (35.0, 37.0): ax.plot([xx, xx], [66, 63.6], color="#c87533", lw=1)
    if v["tidy"]:
        ax.add_patch(Polygon([(27.2, 66), (28.2, 64.6), (29.2, 66)], fc="w", ec="k", lw=0.4))
    ax.set_xlim(-1, 58); ax.set_ylim(30, 104); ax.set_aspect("equal"); ax.tick_params(labelsize=6)
    ax.plot([40, 50], [101, 101], "k", lw=2); ax.text(45, 102, "10 mm", ha="center", fontsize=6)


def figure(N):
    fig = plt.figure(figsize=(24, 13))
    keys = list(VARIANTS)
    for i, k in enumerate(keys):
        v = VARIANTS[k]; ax = fig.add_axes([0.005 + i * 0.166, 0.47, 0.16, 0.44]); draw(ax, k, v, N)
        col = "#2a7" if k == "F3" else "k"
        ax.set_title(f"{k}: {v['title']}", fontsize=10, color=col, fontweight="bold" if k == "F3" else "normal")
    rows = [[""] + [f"{k}" for k in keys]]
    def row(lbl, f): rows.append([lbl] + [f(k, N[k], VARIANTS[k]) for k in keys])
    row("pinion meshes", lambda k, o, v: o["new_rotor_part"].replace(" (same printed part)", "\n(same printed part)").replace(", z", "\nz"))
    row("lowest rotor part over\nthe keys (rotation stage)", lambda k, o, v: f"{o['clear_over_keys_mm']} mm")
    row("worst pose change", lambda k, o, v: f"{o['worst_move_s']} s ({o['root_deg_s']} deg/s)")
    row("drive force", lambda k, o, v: f"{o['push_N']} N ({o['push_margin']}x est.)")
    row("leg: spin load / SF\n(AlSi10Mg 230 MPa)", lambda k, o, v: f"111 + {o['motor_spin_N']} N, {o['leg_MPa']} MPa\nSF {o['leg_SF']}")
    row("power onto moving roots", lambda k, o, v: v["power"])
    row("parking (+-1 deg for keys)", lambda k, o, v: v["park"])
    row("flight hold", lambda k, o, v: o["flight_hold"].replace("; ", "\n"))
    row("mass vs D build (est.)", lambda k, o, v: f"{o['mass_g']:+.1f} g")
    row("verdict", lambda k, o, v: {"F1": "leg SF too low; flange forces\nthe leg foot to be cut back",
                                    "F2": "leg SF too low",
                                    "F3": "RECOMMENDED",
                                    "F4": "fast, but weaker hold;\ncould swap in later (same fit)",
                                    "F5": "more parts, pogo needs\na radial retract",
                                    "F6": "motor 0.5 mm from the spinning\ncage: too tight for a 2 s gain"}[k])
    ax = fig.add_axes([0.005, 0.01, 0.99, 0.43]); ax.axis("off")
    t = ax.table(cellText=rows, loc="center", cellLoc="center"); t.auto_set_font_size(False); t.set_fontsize(8.2); t.scale(1, 2.65)
    for r in range(len(rows)): t[r, 3].set_facecolor("#dff3e6")
    fig.suptitle("hand3 F - placement / implementation variants of the micro motor on each winch leg.  Side views in the root frame, "
                 f"to scale (mm).  Worst pose change = {WORST} deg.  Recommended: F3", fontsize=12.5)
    fig.savefig(os.path.join(HERE, "hand3_rotation_f_variants.png"), dpi=100)


if __name__ == "__main__":
    N = numbers()
    for k, v in VARIANTS.items():                                                  # F6 cage clearance (kill check)
        g = geom(v); N[k]["motor_to_cage_mm"] = round(g["r_ax"] - 3.0 - 21.5 - 0.0, 2) if v.get("tuck") else round(g["r_ax"] - 3.0 - 21.5, 2)
    json.dump(N, open(os.path.join(HERE, "hand3_rotation_f_variants.json"), "w"), indent=1, default=str)
    print(json.dumps(N, indent=1, default=str)); figure(N)
