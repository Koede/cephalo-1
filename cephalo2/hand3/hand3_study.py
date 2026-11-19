"""Three-phalanx hand-prop: visual design study sheets (2026-11-13). Sketches only, no CAD.
python3 hand3_study.py -> hand3_study_1..8.png   (numbers from hand3_loads.json)"""
import json, numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle, FancyBboxPatch, Polygon, Arc, FancyArrowPatch

L = json.load(open("hand3_loads.json"))
INK = "#1f2328"; MUTE = "#6b7280"; BLADE = "#3b4252"; PAD = "#e8912d"; PICK = "#1a7f37"; NO = "#b42318"
TC = ["#d1242f", "#1f6feb", "#8250df"]          # base / middle / tip tendon
W_IN, ROW_IN, DPI = 9.0, 3.9, 110
PH = [36.0, 32.0, 29.0]                         # phalanx lengths, mm
TH = [4.4, 3.9, 3.4]                            # thickness


def rot(a):
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def chain(ang):
    """Frames of the three phalanges for joint angles (deg, + = curl toward the grip side = up in the sketch)."""
    p = np.array([0.0, 0.0]); a = 0.0; fr = []
    for k in range(3):
        a += np.radians(ang[k]); R = rot(a)
        fr.append((p.copy(), R)); p = p + R @ np.array([PH[k], 0.0])
    return fr


def finger(ax, ang, tendons=(), pads=True, hubw=12, lw=1.0, joints=True, alpha=1.0):
    """Side view. tendons: list of (colour, [offset in each phalanx at start, end] x3 or None, lw)."""
    fr = chain(ang)
    ax.add_patch(Rectangle((-hubw, -5), hubw, 10, fc="#9aa3ad", ec=INK, lw=lw, alpha=alpha))
    for k, (p, R) in enumerate(fr):
        t = TH[k] / 2; g = 1.6
        poly = np.array([[g, -t], [PH[k] - g, -t], [PH[k] - g, t], [g, t]])
        ax.add_patch(Polygon(poly @ R.T + p, closed=True, fc=BLADE, ec=INK, lw=lw, alpha=alpha, joinstyle="round"))
        if pads:
            pd = np.array([[5, t], [PH[k] - 5, t], [PH[k] - 5, t + 1.5], [5, t + 1.5]])
            ax.add_patch(Polygon(pd @ R.T + p, closed=True, fc=PAD, ec=INK, lw=0.6, alpha=alpha))
        if joints:
            ax.add_patch(Circle(p, 2.3, fc="white", ec=INK, lw=lw, zorder=5, alpha=alpha))
    for col, pts, w in tendons:                      # pts: [(phalanx index or -1 for the hub, x along it, y toward the grip side)]
        P = []
        for k, x, y in pts:
            P.append(np.array([x, y]) if k < 0 else fr[k][0] + fr[k][1] @ np.array([x, y]))
        P = np.array(P); ax.plot(P[:, 0], P[:, 1], color=col, lw=w, zorder=6, solid_capstyle="round", solid_joinstyle="round")
    return fr


def sheet(title, sub, rows, fn, note=None):
    import textwrap
    hs = []
    for r in rows:                                   # row height from its text
        nl = sum(len(textwrap.wrap(t, 46)) for _, t in r["lines"])
        hs.append(max(2.55, 0.75 + 0.215 * nl + 0.07 * len(r["lines"]) + 0.2))
    Htot = 1.25 + sum(hs) + (0.5 if note else 0.1)
    fig = plt.figure(figsize=(W_IN, Htot), dpi=DPI)
    H = fig.get_figheight()
    fig.text(0.035, 1 - 0.42 / H, title, fontsize=19, fontweight="bold", color=INK, va="top")
    for i, sl in enumerate(textwrap.wrap(sub, 100)[:2]):
        fig.text(0.035, 1 - (0.84 + 0.2 * i) / H, sl, fontsize=10.5, color=MUTE, va="top")
    axes = []; y1 = H - 1.3
    for i, r in enumerate(rows):
        y0 = y1 - hs[i] + 0.15
        pick = r.get("pick"); rej = r.get("rej")
        fig.patches.append(FancyBboxPatch((0.02, y0 / H), 0.96, (y1 - y0) / H, boxstyle="round,pad=0,rounding_size=0.012", transform=fig.transFigure,
                                          fc="#f3fbf5" if pick else "white", ec=PICK if pick else "#d0d7de", lw=2.4 if pick else 1.0, zorder=0))
        ax = fig.add_axes([0.035, (y0 + 0.1) / H, 0.47, (y1 - y0 - 0.2) / H]); ax.set_aspect("equal"); ax.axis("off"); ax.set_zorder(5); ax.patch.set_alpha(0); axes.append(ax)
        tx = 0.525; ty = (y1 - 0.2) / H
        fig.text(tx, ty, r["name"], fontsize=13.5, fontweight="bold", color=INK, va="top")
        if pick: fig.text(0.965, ty, "PICK", fontsize=11, fontweight="bold", color="white", va="top", ha="right", bbox=dict(boxstyle="round,pad=0.3", fc=PICK, ec="none"))
        if rej: fig.text(0.965, ty, "no", fontsize=11, fontweight="bold", color="white", va="top", ha="right", bbox=dict(boxstyle="round,pad=0.3", fc=NO, ec="none"))
        yy = y1 - 0.62
        for kind, txt in r["lines"]:
            col = {"+": PICK, "-": NO, "n": INK, "#": "#0b5cad"}[kind]
            mark = {"+": "+", "-": "–", "n": "·", "#": "#"}[kind]
            fig.text(tx, yy / H, mark, fontsize=11.5, color=col, va="top", fontweight="bold")
            for wl in textwrap.wrap(txt, 46):
                fig.text(tx + 0.022, yy / H, wl, fontsize=10.6, color=INK, va="top"); yy -= 0.215
            yy -= 0.07
        y1 = y1 - hs[i]
    if note: fig.text(0.035, 0.22 / H, note, fontsize=10, color=MUTE, va="bottom")
    return fig, axes


def lim(ax, x0, x1, y0, y1):
    ax.set_xlim(x0, x1); ax.set_ylim(y0, y1)


def fit(ax, pad=6):
    ax.relim(); ax.autoscale_view(); x0, x1 = ax.get_xlim(); y0, y1 = ax.get_ylim()
    ax.set_xlim(x0 - pad, x1 + pad); ax.set_ylim(y0 - pad, y1 + pad)


G = 3.4
def cords_A(d=0.7):
    """Centre-line routing: each cord stays on the centre line through the joints before its own, then moves out under the pad."""
    return [(TC[0], [(-1, -12, G), (0, 3, G), (0, PH[0] - 5, G)], 2.2),
            (TC[1], [(-1, -12, d), (0, 0, d * 0.3), (0, PH[0] - 14, d), (0, PH[0] - 5, G), (1, 3, G), (1, PH[1] - 5, G)], 2.2),
            (TC[2], [(-1, -12, -d), (0, 0, -d * 0.3), (1, 0, -d * 0.3), (1, PH[1] - 14, -d), (1, PH[1] - 5, G), (2, 3, G), (2, PH[2] - 5, G)], 2.2)]


def cords_side(y, d):
    """All three cords on one side at offset y (stacked by d): coupled."""
    return [(TC[0], [(-1, -12, y), (0, 3, y), (0, PH[0] - 5, y)], 2.2),
            (TC[1], [(-1, -12, y + d), (0, 3, y + d), (0, PH[0] - 3, y + d), (1, 3, y), (1, PH[1] - 5, y)], 2.2),
            (TC[2], [(-1, -12, y + 2 * d), (0, 3, y + 2 * d), (0, PH[0] - 3, y + 2 * d), (1, 3, y + d), (1, PH[1] - 3, y + d), (2, 3, y), (2, PH[2] - 5, y)], 2.2)]


def lab(ax, x, y, s, c=INK, fs=9.5, ha="center", **kw):
    ax.text(x, y, s, color=c, fontsize=fs, ha=ha, va="center", **kw)


# ---------------------------------------------------------------- 1 the finger and its flight loads
def s1():
    K = L["knuckles"]
    rows = [dict(name="Blade = three phalanges", lines=[
                ("n", "10 in rotor (r 127 mm), 4 fingers. Knuckles at r 30, 66 and 98 mm; phalanges 36, 32 and 29 mm."),
                ("n", "Chord 18 to 12 mm, 4.4 to 3.4 mm thick, TPU pad on the grip face of each phalanx."),
                ("#", f"26.5 N needs about {L['rpm_at_26N']:,} rpm (est.); proof speed {L['rpm_proof']:,} rpm, tip {L['tip_speed_proof']} m/s."),
                ("n", "Off-centre fault of the old claw fixed: every cord runs on the blade's centre line.")]),
            dict(name="What each knuckle must carry", lines=[
                ("#", f"Pull at proof speed: base {K[1]['CF_proof_N']} N, middle {K[2]['CF_proof_N']} N, tip {K[3]['CF_proof_N']} N (lag pin {K[0]['CF_proof_N']} N)."),
                ("#", f"Thrust tries to fold them: {K[1]['thrust_moment_Nmm']:.0f} / {K[2]['thrust_moment_Nmm']:.0f} / {K[3]['thrust_moment_Nmm']:.0f} N·mm at full thrust."),
                ("+", "A 2° preconed stop seat at each knuckle lets the spin itself hold the finger straight: about 2.8× the thrust moment, at any rpm."),
                ("-", "A 1 mm wire pin would see 55-130 MPa on the plastic: too much. Pins must be 2-3 mm, in bushings.")]),
            dict(name="What joint-by-joint means", lines=[
                ("n", "Each joint of each finger gets its own cord: 3 cords × 4 fingers = 12, plus 2 for the finger spread."),
                ("n", "Any joint can curl alone (tip hook, base wrap, middle pinch) or together."),
                ("n", "Spinning: cords slack, seats closed, the finger is a rigid blade.")])]
    fig, ax = sheet("1  The finger: a blade made of three phalanges", "Three-phalanx hand-prop design study, sheet 1 of 8. Sketches, not CAD. Loads are estimates.", rows, 1)
    a = ax[0]; finger(a, [0, 0, 0]); lim(a, -16, 104, -22, 22)
    for x, s in ((0, "K1\nr 30"), (36, "K2\nr 66"), (68, "K3\nr 98")): lab(a, x, -12, s)
    lab(a, 97, -9, "tip\nr 127"); lab(a, -6, 11, "hub"); lab(a, 50, 13, "pads (grip face)", PAD)
    a = ax[1]; finger(a, [0, 0, 0], pads=False); lim(a, -16, 104, -24, 24)
    for x, k in ((0, 1), (36, 2), (68, 3)):
        a.add_patch(FancyArrowPatch((x + 3, -8), (x + 22, -8), arrowstyle="-|>", mutation_scale=13, color=NO, lw=2))
        lab(a, x + 12, -14, f"{K[k]['CF_proof_N']} N", NO, 10)
    a.add_patch(FancyArrowPatch((80, 4), (80, 18), arrowstyle="-|>", mutation_scale=13, color="#0b5cad", lw=2)); lab(a, 80, 21.5, "thrust", "#0b5cad")
    a = ax[2]
    for i, (ang, s) in enumerate((([0, 0, 70], "tip only"), ([55, 0, 0], "base only"), ([50, 60, 60], "all three"))):
        fr = chain(ang); off = np.array([i * 62.0, 0])
        for k, (p, R) in enumerate(fr):
            t = TH[k] / 2; poly = np.array([[1.6, -t], [PH[k] - 1.6, -t], [PH[k] - 1.6, t], [1.6, t]]) * 0.55
            a.add_patch(Polygon(poly @ R.T + p * 0.55 + off, closed=True, fc=BLADE, ec=INK, lw=0.8))
            a.add_patch(Circle(p * 0.55 + off, 1.4, fc="white", ec=INK, lw=0.8, zorder=5))
        a.add_patch(Rectangle((off[0] - 6, -3), 6, 6, fc="#9aa3ad", ec=INK, lw=0.8)); lab(a, off[0] + 22, -12, s)
    lim(a, -10, 178, -18, 48)
    fig.savefig("hand3_study_1.png"); plt.close(fig)


# ---------------------------------------------------------------- 2 how a cord picks its joint
def s2():
    rows = [dict(name="A  Centre-line routing", pick=True, lines=[
                ("n", "Each cord runs down the middle of the blade and crosses the knuckles it must not bend exactly on their axis (between two stub pins). Just before its own knuckle it moves out under the pad."),
                ("+", "No lever arm there, so pulling the tip cord curls only the tip. Three cords, three joints, no cross-talk."),
                ("+", "No extra parts: channels printed in the phalanges."),
                ("-", "Each pin becomes two short stubs with a 3 mm gap for the cords.")]),
            dict(name="B  Bowden sheaths", lines=[
                ("n", "Each cord in its own sheath (1 mm PTFE tube) along the back of the blade to its phalanx."),
                ("+", "Decoupled wherever the sheath goes."),
                ("-", "Sheaths on the back spoil the airfoil and want to fly off at 1,000 g; friction grows with every bend.")]),
            dict(name="C  All cords on the grip side", lines=[
                ("n", "Simplest channels: every cord under the pads."),
                ("-", "The tip cord crosses all three joints with a lever arm: pulling it curls everything. Single joints need the other cords paid out in step."),
                ("-", "With latching keys that is three key moves for one joint.")]),
            dict(name="D  One cord and joint brakes", rej=True, lines=[
                ("n", "One cord per finger; a small brake at each knuckle chooses which may move."),
                ("-", "Twelve brakes on the rotor, each needing its own signal. Worse than twelve cords.")])]
    fig, ax = sheet("2  How a cord picks its joint", "Sheet 2 of 8. In every sketch the tip cord (purple) has been pulled. Red = base cord, blue = middle, purple = tip.", rows, 2)
    finger(ax[0], [0, 0, 65], tendons=cords_A())
    finger(ax[1], [0, 0, 65], tendons=cords_side(-3.6, -1.0)); lab(ax[1], 30, -12, "sheaths on the back", MUTE)
    finger(ax[2], [38, 42, 50], tendons=cords_side(G, 1.0))
    fr = finger(ax[3], [0, 0, 65], tendons=[("#444", [(-1, -12, G), (0, 3, G), (0, PH[0] - 3, G), (1, 3, G), (1, PH[1] - 3, G), (2, 3, G), (2, PH[2] - 5, G)], 2.2)])
    for p, R in fr[:2]: ax[3].add_patch(Rectangle(p - [3.2, 3.2], 6.4, 6.4, fc=NO, ec=INK, lw=0.8, zorder=7, alpha=0.85))
    lab(ax[3], 18, -10, "brakes on", NO)
    for a in ax: fit(a)
    fig.savefig("hand3_study_2.png"); plt.close(fig)


# ---------------------------------------------------------------- 3 lever arm
def s3():
    Lv = {int(round(x["r_mm"] * 10)): x for x in L["lever"]}
    def row(nm, r, extra, **kw):
        x = Lv[int(round(r * 10))]
        return dict(name=nm, lines=[("#", f"Lever arm {r:g} mm: {x['torque_Nm']} N·m per joint at 80 N of cord."),
                                    ("#", f"Fingertip squeeze {x['tip_force_N_at_15mm']:.0f} N; whole-finger wrap {x['base_force_N_at_70mm']:.0f} N at the far end."),
                                    ("#", f"One 2 mm key tooth = {x['deg_per_2mm_tooth']:.0f}° of joint; 80° needs {x['travel_80deg_mm']} mm of cord.")] + extra, **kw)
    rows = [row("A  Inside the airfoil", 2.0, [("+", "Clean blade."), ("-", "Too weak, and one tooth is most of the joint's travel.")]),
            row("B  Tunnel under the pad", 4.5, [("+", "The pad is there anyway; the cord is covered, so the object never touches it."),
                                              ("-", "25° per tooth is coarse on its own: needs the elastic link (sheet 5) or finer teeth.")], pick=True),
            row("C  Pulley horn on the grip side", 8.0, [("+", "Strongest and finest."), ("-", "Three 6 mm bumps per blade in the airflow; they also hit the object first.")]),
            dict(name="D  Bowstring (cord leaves the blade)", rej=True, lines=[("n", "Lever arm grows as the finger curls."),
                 ("-", "This is the case you described: the object presses on the cord and the grip changes by itself. Also snags.")])]
    fig, ax = sheet("3  Where the cord pulls: the lever arm", "Sheet 3 of 8. One knuckle, bent 45°. Cord tension limited to 80 N by the key teeth.", rows, 3)
    for i, (r, mode) in enumerate(((2.0, "in"), (4.5, "pad"), (8.0, "horn"), (9.0, "bow"))):
        a = ax[i]; ang = np.radians(45); R = rot(ang)
        for (p, Rm, t) in ((np.array([-30.0, 0]), np.eye(2), 2.2), (np.array([0.0, 0]), R, 2.0)):
            poly = np.array([[2, -t], [30, -t], [30, t], [2, t]]) if Rm is R else np.array([[0, -t], [28, -t], [28, t], [0, t]])
            a.add_patch(Polygon(poly @ Rm.T + p, closed=True, fc=BLADE, ec=INK, lw=1))
            if mode in ("pad", "bow", "horn"):
                pd = np.array([[4, t], [26, t], [26, t + 2.6], [4, t + 2.6]]); a.add_patch(Polygon(pd @ Rm.T + p, closed=True, fc=PAD, ec=INK, lw=0.6))
        a.add_patch(Circle((0, 0), 2.6, fc="white", ec=INK, lw=1, zorder=5))
        if mode == "horn":
            th = np.linspace(np.radians(60), np.radians(170), 20); arc = np.c_[np.cos(th), np.sin(th)] * 8.0
            a.add_patch(Polygon(np.vstack([[0, 0], arc]), closed=True, fc="#9aa3ad", ec=INK, lw=0.8, zorder=4))
        if mode == "bow":
            p0 = np.array([-22.0, 4.8]); p1 = R @ np.array([22.0, 4.6]); a.plot([p0[0], p1[0]], [p0[1], p1[1]], color=TC[0], lw=2.4, zorder=6)
            a.add_patch(Circle((-2, 17), 7, fc="#d8dee4", ec=INK, lw=0.8)); lab(a, -2, 17, "object", fs=8)
        else:
            th = np.linspace(np.pi / 2, np.pi / 2 + ang, 14); arc = np.c_[np.cos(th), np.sin(th)] * r
            pts = np.vstack([[-26.0, r], arc, R @ np.array([22.0, r])])
            a.plot(pts[:, 0], pts[:, 1], color=TC[0], lw=2.6, zorder=8, solid_capstyle="round")
            a.plot([0, 0], [0, r], color="white" if r < 3 else INK, lw=1.2, zorder=9); lab(a, -6, -6.5, f"lever arm {r:g} mm", fs=9)
        fit(a, 4)
    fig.savefig("hand3_study_3.png"); plt.close(fig)


# ---------------------------------------------------------------- 4 knuckle type
def s4():
    K = L["knuckles"]
    rows = [dict(name="A  Stub pins in bushings", pick=True, lines=[
                ("n", "Fork and tongue. Two short hardened dowel pins (ISO 8734) in brass bushings, 3 mm gap between them for the cords. Flat stop faces with 2° precone."),
                ("#", f"Ø3 stubs at the base and middle ({K[1]['CF_proof_N']} and {K[2]['CF_proof_N']} N), Ø2 at the tip: 26-34 MPa on the bushing seats, pin shear under 170 MPa."),
                ("+", "Exact axis (the centre-line routing needs it), no slop in blade pitch, parts off the shelf."),
                ("-", "A hinge barrel about 7 mm across at each knuckle; 6 pins and 6 bushings per finger.")]),
            dict(name="B  Rolling knuckle on ligaments", lines=[
                ("n", "Two printed cylinders roll on each other, held by crossed Dyneema cords that also carry the spin load (as the scale fingers do now)."),
                ("+", "No pins, no bearing stress in plastic; 1.5 mm Dyneema holds about 2 kN."),
                ("-", "The axis moves as it rolls, so no cord can cross 'on the axis': joints couple. Cord creep opens the seats over time.")]),
            dict(name="C  Steel tape spring (as the Snap-Digit)", rej=True, lines=[
                ("n", "A strip of tape-measure steel is the hinge."),
                ("-", "It snaps: straight or folded, nothing in between. That is the on/off feel you want to get rid of."),
                ("-", "The strip fills the centre line where the cords must run.")]),
            dict(name="D  Printed TPU flexure", lines=[
                ("n", "A living hinge printed in one piece with a ligament cord through it."),
                ("+", "One part per finger, springs back by itself."),
                ("-", "Soft in twist: the blade sections can flutter in pitch. Axis not defined. TPU creeps under 400-1,000 N.")])]
    fig, ax = sheet("4  The knuckle", "Sheet 4 of 8. It must carry the spin load, lock straight in flight, and let three cords through.", rows, 4)
    a = ax[0]                                              # plan view of fork + tongue
    a.add_patch(Rectangle((-34, -9), 34, 18, fc=BLADE, ec=INK)); a.add_patch(Rectangle((-8, -4), 9, 8, fc="white", ec="white"))
    a.add_patch(Rectangle((-7, -3.6), 41, 7.2, fc="#5b6475", ec=INK))
    for sgn in (-1, 1):
        a.add_patch(Rectangle((-2.2, sgn * 1.5 if sgn > 0 else -8.5), 4.4, 7.0, fc="#c9a227", ec=INK, lw=0.8, zorder=4))
        a.add_patch(Rectangle((-1.2, sgn * 1.5 if sgn > 0 else -9.0), 2.4, 7.5, fc="#d8dee4", ec=INK, lw=0.8, zorder=5))
    for k, dy in enumerate((-0.9, 0, 0.9)): a.plot([-34, 34], [dy, dy], color=TC[k], lw=1.8, zorder=6)
    lab(a, 0, -12.5, "stub pins + bushings"); lab(a, 0, 12.5, "3 cords through the gap"); lab(a, -22, 0, "fork", "white", zorder=9); lab(a, 22, 5.8, "tongue", MUTE)
    lim(a, -38, 38, -16, 16)
    a = ax[1]
    a.add_patch(Rectangle((-34, -3), 28, 6, fc=BLADE, ec=INK)); a.add_patch(Rectangle((6, -3), 28, 6, fc=BLADE, ec=INK))
    a.add_patch(Circle((-6, 0), 6, fc="#5b6475", ec=INK)); a.add_patch(Circle((6, 0), 6, fc="#5b6475", ec=INK))
    th = np.linspace(-np.pi / 2, np.pi / 2, 30)
    a.plot(np.r_[-6 + 6 * np.cos(th[::-1] + np.pi) * 0 - 0, 0], np.r_[th * 0, 0], lw=0)
    a.plot([-12, -6, 6, 12], [5.5, 6.2, -6.2, -5.5], color="#1a7f37", lw=2.2); a.plot([-12, -6, 6, 12], [-5.5, -6.2, 6.2, 5.5], color="#1a7f37", lw=2.2)
    lab(a, 0, -12.5, "crossed ligament cords"); lim(a, -38, 38, -16, 16)
    a = ax[2]
    R = rot(np.radians(50)); a.add_patch(Rectangle((-34, -2.5), 28, 5, fc=BLADE, ec=INK))
    poly = np.array([[8, -2.5], [34, -2.5], [34, 2.5], [8, 2.5]]); a.add_patch(Polygon(poly @ R.T, closed=True, fc=BLADE, ec=INK))
    tt = np.linspace(0, 1, 20); p0 = np.array([-10.0, 3]); p1 = R @ np.array([12.0, 3]); c0 = p0 + [9, 0]; c1 = p1 - R @ np.array([9.0, 0])
    bz = ((1 - tt) ** 3)[:, None] * p0 + (3 * (1 - tt) ** 2 * tt)[:, None] * c0 + (3 * (1 - tt) * tt ** 2)[:, None] * c1 + (tt ** 3)[:, None] * p1
    a.plot(bz[:, 0], bz[:, 1], color="#c9a227", lw=3.2); lab(a, -4, -9, "steel tape: snaps"); lim(a, -38, 38, -14, 30)
    a = ax[3]
    a.add_patch(Rectangle((-34, -3), 30, 6, fc=BLADE, ec=INK)); a.add_patch(Rectangle((4, -3), 30, 6, fc=BLADE, ec=INK))
    a.add_patch(Rectangle((-4, -1.2), 8, 2.4, fc="#7fb3d5", ec=INK)); a.plot([-34, 34], [0, 0], color="#1a7f37", lw=1.6); lab(a, 0, -8, "TPU web + ligament"); lim(a, -38, 38, -16, 16)
    fig.savefig("hand3_study_4.png"); plt.close(fig)


# ---------------------------------------------------------------- 5 where the give lives
def s5():
    E = L["elastic"]
    rows = [dict(name="A  No elastic (cord straight to the key)", lines=[
                ("#", f"0.6 mm Dyneema, 150 mm: about {E['dyneema_0p6_150mm']:.0f} N/mm."),
                ("-", "One 2 mm tooth would add about 190 N: the finger is either loose or locked solid. That is the binary grip again."),
                ("n", "The TPU pads give only about 0.3 mm.")]),
            dict(name="B  Silicone cord in line", rej=True, lines=[
                ("#", f"3 mm silicone cord, 15 mm long: {E['silicone_3mm_15mm']} N/mm."),
                ("-", "Far too soft: 40 N of grip would need 56 mm of stretch. It would only work at a few newtons."),
                ("n", "This is the option I suggested by voice. The numbers say no.")]),
            dict(name="C  Printed TPU link in the key", pick=True, lines=[
                ("#", f"TPU 95A dog-bone, 2 × 2 mm, 16 mm long: about {E['tpu95_2x2_16mm']} N/mm (est., E 30 MPa)."),
                ("+", "Each 2 mm tooth adds about 15 N: five grip levels up to 75 N, and between teeth the finger gives and conforms."),
                ("+", "The link sits in the hub, not on the grip face. One printed part per cord, 0.1 g."),
                ("-", "TPU creeps: a grip held for minutes relaxes. Needs a bench curve.")]),
            dict(name="D  Steel extension spring", lines=[
                ("#", f"Ø4 mm, 0.6 mm wire, 10 coils: {E['steel_spring_4od_0p6wire_10coils']} N/mm, but {E['steel_spring_stress_at_40N_MPa']} MPa at 40 N."),
                ("-", "A spring small enough for the hub yields at about 20 N; one strong enough is Ø6 × 25 mm and 1.5 g, twelve times, spinning."),
                ("+", "No creep. Keep as the fallback if TPU relaxes too much.")])]
    fig, ax = sheet("5  Where the give lives", "Sheet 5 of 8. The cord from a finger joint meets its key in the hub. What sits between them decides how the grip feels.", rows, 5)
    for i, kind in enumerate(("none", "sil", "tpu", "spring")):
        a = ax[i]; a.plot([-36, -12], [0, 0], color=TC[1], lw=2.2); lab(a, -26, 4, "cord from\nthe finger", fs=8.5)
        a.add_patch(Rectangle((14, -7), 12, 14, fc="#9aa3ad", ec=INK)); lab(a, 20, 0, "key", fs=9)
        for y in (-5, -2.5, 0, 2.5, 5): a.plot([26, 28.5], [y, y - 1.2], color=INK, lw=1.2)
        a.add_patch(Rectangle((28.5, -9), 3, 18, fc="#d8dee4", ec=INK)); lab(a, 30, -12.5, "rack", fs=8.5)
        if kind == "none": a.plot([-12, 14], [0, 0], color=TC[1], lw=2.2)
        if kind == "sil": a.plot([-12, 14], [0, 0], color="#f4a6b8", lw=7, solid_capstyle="round"); lab(a, 1, -6, "silicone", fs=8.5)
        if kind == "tpu":
            a.add_patch(Polygon([[-12, -2.5], [-7, -2.5], [-5, -1], [7, -1], [9, -2.5], [14, -2.5], [14, 2.5], [9, 2.5], [7, 1], [-5, 1], [-7, 2.5], [-12, 2.5]], closed=True, fc="#7fb3d5", ec=INK))
            lab(a, 1, -6, "TPU link", fs=8.5)
        if kind == "spring":
            xs = np.linspace(-12, 14, 80); a.plot(xs, 2.6 * np.sin((xs + 12) * 2.4), color="#555", lw=1.6); lab(a, 1, -6.5, "steel spring", fs=8.5)
        lim(a, -40, 36, -16, 14)
    fig.savefig("hand3_study_5.png"); plt.close(fig)


# ---------------------------------------------------------------- 6 selector
def hub_top(a, nkeys=14, shoes=(0,), screw=False):
    a.add_patch(Circle((0, 0), 34, fc="#eef1f4", ec=INK, lw=1)); a.add_patch(Circle((0, 0), 18.5, fc="#30363d", ec=INK)); lab(a, 0, 0, "motor", "white", fs=9)
    az = []
    for f in range(4):
        for j in range(3): az.append((45 + 90 * f + (j - 1) * 21, TC[j]))
    az += [(0, "#57606a"), (180, "#57606a")]
    for ang, col in az[:nkeys]:
        t = np.radians(ang); c = np.array([np.cos(t), np.sin(t)]) * 26.5
        if screw: a.add_patch(Circle(c, 3.6, fc=col, ec=INK, lw=0.7)); a.add_patch(Circle(c, 1.2, fc="white", ec=INK, lw=0.5))
        else:
            Rm = rot(t); poly = np.array([[-3.2, -3.4], [3.2, -3.4], [3.2, 3.4], [-3.2, 3.4]]); a.add_patch(Polygon(poly @ Rm.T + c, closed=True, fc=col, ec=INK, lw=0.7))
    for s in shoes:
        t = np.radians(s); c = np.array([np.cos(t), np.sin(t)]) * 38.5; Rm = rot(t)
        poly = np.array([[-3.5, -4.5], [5, -4.5], [5, 4.5], [-3.5, 4.5]]); a.add_patch(Polygon(poly @ Rm.T + c, closed=True, fc="#f0b72f", ec=INK, lw=1, zorder=6))
    for f in range(4):
        t = np.radians(45 + 90 * f); a.plot([34 * np.cos(t), 62 * np.cos(t)], [34 * np.sin(t), 62 * np.sin(t)], color=BLADE, lw=7, solid_capstyle="round", zorder=1)
    lim(a, -64, 64, -50, 50)


def s6():
    rows = [dict(name="A  14 latching keys, one pusher", lines=[
                ("n", "The key hub that already exists, widened: 12 cord keys + 2 spread keys round the motor. The motor turns the wanted key under one pusher."),
                ("+", "Every joint of every finger on its own. Nothing powered on the rotor. One small actuator."),
                ("-", "One key at a time: a full grasp is 14 moves, about 20 s (est. from the 6-key hub: 9 s).")]),
            dict(name="B  14 keys, two pushers", pick=True, lines=[
                ("n", "Same hub, plus a second pusher: a ring with four notches 90° apart: it sets the same joint on all four fingers in one move."),
                ("+", "Full grasp in 3 moves + 2 spread moves, about 7 s. Single fingers still possible (peace sign, pointing) with the single pusher."),
                ("+", "With the TPU links the four fingers settle on the object unevenly: the hand conforms."),
                ("-", "Steps, not a continuous squeeze: five force levels per joint. Narrower keys needed to fit 14 (to be proven in CAD).")]),
            dict(name="C  Screw keys, one gearmotor", lines=[
                ("n", "Each key is a nut on a screw post with a star wheel; the motor indexes a post to a stationary driver."),
                ("+", "Continuous and self-locking; the driver's current measures grip force."),
                ("-", "Still one at a time (about 20 s). The driver meshing with a wheel that arrives sideways is unproven.")]),
            dict(name="D  Three sliding rings, three servos", lines=[
                ("n", "One ring per joint level (all bases, all middles, all tips), each on its own bearing below the motor, each pulled by an STS3032."),
                ("+", "Live, continuous, force-controlled squeeze in about 0.3 s."),
                ("-", "No single-finger moves. About +120 g and 50 mm taller per hand; a fourth ring for the spread.")]),
            dict(name="E  Twelve micro servos on the rotor", rej=True, lines=[
                ("-", "Each 2 g servo would weigh about 10 kg at proof speed; power through a slip ring; balance. No.")])]
    fig, ax = sheet("6  The selector: how 14 cords get pulled", "Sheet 6 of 8. Nothing powered can ride on the spinning rotor. Top views of the hub (red / blue / purple = base / middle / tip keys, grey = spread).", rows, 6)
    hub_top(ax[0], shoes=(24,)); lab(ax[0], 50, 30, "pusher", "#9a6700", fs=9)
    hub_top(ax[1], shoes=(24,)); a = ax[1]
    a.add_patch(Circle((0, 0), 41, fc="none", ec="#f0b72f", lw=5, zorder=3, alpha=0.9))
    for f in range(4):
        t = np.radians(45 + 90 * f - 21); a.add_patch(Circle((41 * np.cos(t), 41 * np.sin(t)), 3.4, fc="#f0b72f", ec=INK, lw=0.8, zorder=7))
    lab(a, 0, -47, "ring pusher: 4 notches", "#9a6700", fs=9)
    hub_top(ax[2], shoes=(), screw=True); ax[2].add_patch(Circle((40, 0), 5.5, fc="#f0b72f", ec=INK, zorder=6)); lab(ax[2], 52, 9, "driver", "#9a6700", fs=9)
    a = ax[3]                                              # side view of three rings under the motor
    a.add_patch(Rectangle((-17, 10), 34, 22, fc="#30363d", ec=INK)); lab(a, 0, 21, "motor", "white", fs=9)
    a.add_patch(Rectangle((-13, -44), 26, 54, fc="#d8dee4", ec=INK)); lab(a, 0, -38, "arm tip", fs=8.5)
    a.plot([-60, 60], [36, 36], color=BLADE, lw=6, solid_capstyle="round")
    for k, y in enumerate((0, -13, -26)):
        a.add_patch(Rectangle((-24, y - 3), 11, 6, fc=TC[k], ec=INK)); a.add_patch(Rectangle((13, y - 3), 11, 6, fc=TC[k], ec=INK))
        a.plot([-22 - 2 * k, -22 - 2 * k], [y + 3, 34], color=TC[k], lw=1.6); a.plot([22 + 2 * k, 22 + 2 * k], [y + 3, 34], color=TC[k], lw=1.6)
        a.add_patch(Rectangle((30, y - 4), 12, 8, fc="#f0b72f", ec=INK)); a.plot([24, 30], [y, y], color=INK, lw=1.2)
    lab(a, 50, -13, "3 servos", "#9a6700", fs=9, ha="left"); lim(a, -64, 72, -48, 44)
    a = ax[4]; hub_top(a, nkeys=0, shoes=())
    for f in range(4):
        for j in range(3):
            t = np.radians(45 + 90 * f + (j - 1) * 24); c = np.array([np.cos(t), np.sin(t)]) * 27; Rm = rot(t)
            poly = np.array([[-3.5, -4.5], [3.5, -4.5], [3.5, 4.5], [-3.5, 4.5]]); a.add_patch(Polygon(poly @ Rm.T + c, closed=True, fc="#f0b72f", ec=INK, lw=0.7))
    fig.savefig("hand3_study_6.png"); plt.close(fig)


# ---------------------------------------------------------------- 7 spread
def rotor_top(a, angs, col=BLADE):
    a.add_patch(Circle((0, 0), 9, fc="#9aa3ad", ec=INK))
    for b, t in zip((45, 135, 225, 315), angs):
        r0 = np.array([np.cos(np.radians(b)), np.sin(np.radians(b))]) * 8.0; d = np.array([np.cos(np.radians(t)), np.sin(np.radians(t))])
        a.plot([r0[0], r0[0] + d[0] * 44], [r0[1], r0[1] + d[1] * 44], color=col, lw=6, solid_capstyle="round"); a.add_patch(Circle(r0, 2.0, fc="white", ec=INK, zorder=5))
    lim(a, -60, 60, -54, 54)


def s7():
    rows = [dict(name="A  Lag pins + two spread keys", pick=True, lines=[
                ("n", "Each finger swings on a vertical pin at r 20 mm. Two keys pull the pairs from the flight cross to the hand position, two against two."),
                ("+", "This is the 'rotate on the base to be closer to one finger than the other' you asked for. Already designed and fit-checked."),
                ("+", "All three cords of a finger go down through its lag tube (Ø3 bore), on the swing axis, so spreading does not pull them."),
                ("-", "The lag pin carries 1,070 N at proof speed: Ø4 tube in a two-plate hub, as in the folding-prop pivot.")]),
            dict(name="B  Fixed cross (no spread)", lines=[
                ("+", "Simplest and stiffest hub; 12 keys instead of 14."),
                ("-", "Fingers 90° apart cannot oppose each other round a handle; grip only works on things wider than the hub.")]),
            dict(name="C  Each finger spread on its own", lines=[
                ("n", "Four spread keys instead of two."),
                ("+", "Three-against-one and other odd grips."),
                ("-", "16 keys round the motor do not fit one tier. Can be added later: the hub only gains two keys.")])]
    fig, ax = sheet("7  Finger spread on the hub", "Sheet 7 of 8. Top views of the rotor: flight position in grey, hand position in dark.", rows, 7)
    rotor_top(ax[0], (45, 135, 225, 315), "#c8ced6"); rotor_top(ax[0], (45 - 49, 135 + 49 - 0, 225 - 33 - 16, 315 + 33 + 16))
    lab(ax[0], 0, -47, "pairs close two against two", fs=9)
    rotor_top(ax[1], (45, 135, 225, 315))
    rotor_top(ax[2], (45, 135, 225, 315), "#c8ced6"); rotor_top(ax[2], (20, 160, 200, 270))
    fig.savefig("hand3_study_7.png"); plt.close(fig)


# ---------------------------------------------------------------- 8 the recommended design
def s8():
    P = L["power"]
    rows = [dict(name="Recommended hand", pick=True, lines=[
                ("n", "Finger: three PA-CF phalanges, TPU pads, cords on the centre line, each moving out under the pad just before its own joint (2A + 3B)."),
                ("n", "Knuckles: stub dowel pins in brass bushings, 2° preconed stop seats (4A)."),
                ("n", "Give: a printed TPU link between every cord and its key (5C)."),
                ("n", "Hub: 14 latching keys, single pusher + four-notch ring pusher, motor as the wrist (6B). Lag pins and spread keys kept (7A).")]),
            dict(name="Bought parts", lines=[
                ("#", f"Motor: 10 in needs about {L['rpm_at_26N']:,} rpm and {L['torque_at_26N_Nm']} N·m at 26.5 N (est.). BrotherHobby Avenger 3110 730KV (76 g, 6S, tested by the maker on a 10×5×3). Alternative: T-Motor 2810 KV900 (65 g)."),
                ("n", "Pins: ISO 8734 hardened dowels Ø3 × 6 (16), Ø2 × 5 (8); brass bushings 4 × 3 and 3 × 2; lag pivots Ø4 × 0.5 stainless tube."),
                ("n", "Cords: 0.6 mm Dyneema braid (12 + 4). Pushers: two N20 lead-screw gearmotors or STS3032 servos (to choose in CAD).")]),
            dict(name="Printing", lines=[
                ("n", "Phalanges flat on the back face, layers along the span, so the spin load never pulls layers apart. Channels print as open grooves closed by the pad."),
                ("n", "PA-CF for phalanges, cage and keys (2 mm teeth want a 0.25 mm nozzle); TPU 95A for pads and links."),
                ("n", "First print: ONE finger on a bench hub with three hand-pulled cords.")]),
            dict(name="What is still unknown", lines=[
                ("-", "Whether 14 keys fit round a Ø37 mm motor with usable gaps (first thing the CAD must answer)."),
                ("-", f"Thrust of thick untwisted finger blades: hover power guessed at {P['13.0N']['fm050_W']} W per rotor, no better than the XL claw."),
                ("-", "A drone motor cannot hold an angle: the wrist needs an encoder or a detent (open since the first key hub)."),
                ("-", "10 in discs on neighbouring arms are about 20 mm apart: which arms carry this hand is still to decide.")])]
    fig, ax = sheet("8  The design I would build", "Sheet 8 of 8. Nothing is drawn in CAD until you agree or change these picks.", rows, 8)
    finger(ax[0], [35, 45, 55], tendons=cords_A()); fit(ax[0])
    hub_top(ax[1], shoes=(24,)); ax[1].add_patch(Circle((0, 0), 41, fc="none", ec="#f0b72f", lw=5, zorder=3, alpha=0.9))
    a = ax[2]; finger(a, [0, 0, 0]); lim(a, -16, 104, -26, 22)
    for y in (-14, -16.5, -19): a.plot([-10, 100], [y, y], color=MUTE, lw=0.8)
    lab(a, 45, -23, "layers run along the span", MUTE, 9)
    a = ax[3]; rotor_top(a, (45, 135, 225, 315)); a.add_patch(Circle((0, 0), 52, fc="none", ec=MUTE, lw=1, ls="--")); lab(a, 0, -47, "Ø254 mm disc", MUTE, 9)
    fig.savefig("hand3_study_8.png"); plt.close(fig)


if __name__ == "__main__":
    for f in (s1, s2, s3, s4, s5, s6, s7, s8): f()
    print("done")
