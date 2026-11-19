"""Three-phalanx hand-prop: the selected design on one sheet (2026-11-13, after Forest's choices). python3 hand3_selected.py"""
import numpy as np
import hand3_study as S
from hand3_study import *

L = S.L; K = L["knuckles"]
rows = [dict(name="Finger: three joints, three cords", pick=True, lines=[
            ("n", "Three PA-CF phalanges (36 / 32 / 29 mm) with TPU pads on a 10 in rotor."),
            ("n", "Cords run on the blade's centre line. Each moves out under the pad just before its own joint, so it bends that joint only."),
            ("#", "Pad tunnel, 4.5 mm lever arm: 0.36 N·m per joint, about 24 N fingertip squeeze at 80 N of cord.")]),
        dict(name="Each finger on its own", pick=True, lines=[
            ("n", "Your choice: individual control. Every joint of every finger has its own key: 12 curl keys."),
            ("n", "So one finger can hook with its tip while another wraps from the base and two stay straight.")]),
        dict(name="Knuckle: stub pins", pick=True, lines=[
            ("n", "Fork and tongue. Two hardened dowel stubs in brass bushings: Ø3 at the base and middle, Ø2 at the tip."),
            ("#", f"Carries {K[1]['CF_proof_N']} / {K[2]['CF_proof_N']} / {K[3]['CF_proof_N']} N at proof speed; 2° preconed stop seats lock the finger straight in flight."),
            ("n", "The 3 mm gap between the stubs is where the cords cross the axis.")]),
        dict(name="Give: a TPU link per cord", pick=True, lines=[
            ("n", "A printed TPU 95A dog-bone (2 × 2 × 16 mm) between each cord and its key, inside the hub."),
            ("#", "About 7.5 N/mm: each 2 mm key tooth adds about 15 N. Five grip levels per joint, and the finger gives between them.")]),
        dict(name="Hub: 14 latching keys, one pusher", pick=True, lines=[
            ("n", "12 curl keys + 2 spread keys round the motor. The motor turns the wanted key under the pusher, the pusher sets its tooth, the key latches."),
            ("n", "Nothing powered on the rotor. One joint moves at a time (about 1.5 s each)."),
            ("n", "Optional, dashed: a four-notch ring pusher as a shortcut for a plain grab. Leave it out and nothing else changes.")]),
        dict(name="Spread: fingers swing on the base", pick=True, lines=[
            ("n", "Each finger on a vertical lag pin at r 20 mm; two spread keys bring the pairs from the flight cross to two against two."),
            ("n", "A finger's three cords go down through its lag tube, on the swing axis.")]),
        dict(name="Motor and parts", lines=[
            ("#", f"About {L['rpm_at_26N']:,} rpm and {L['torque_at_26N_Nm']} N·m for 26.5 N (est.): BrotherHobby Avenger 3110 730KV, 76 g, 6S."),
            ("n", "ISO 8734 dowels Ø3 × 6 and Ø2 × 5, brass bushings, 0.6 mm Dyneema, Ø4 × 0.5 stainless lag tubes."),
            ("n", "PA-CF phalanges printed flat, layers along the span; TPU 95A pads and links.")])]
fig, ax = sheet("Selected design: three-joint hand-prop", "What will be drawn in CAD. Sketches, not CAD; loads are estimates.", rows, 9)
finger(ax[0], [30, 40, 50], tendons=cords_A()); fit(ax[0])
a = ax[1]                                                       # four fingers, four different poses (side views, small)
for i, (ang, s) in enumerate((([0, 0, 75], "tip hook"), ([60, 0, 0], "base wrap"), ([0, 0, 0], "straight"), ([35, 45, 50], "full curl"))):
    fr = chain(ang); off = np.array([(i % 2) * 84.0, -(i // 2) * 60.0]); sc = 0.55
    a.add_patch(Rectangle((off[0] - 6, off[1] - 3), 6, 6, fc="#9aa3ad", ec=INK, lw=0.8))
    for k, (p, R) in enumerate(fr):
        t = TH[k] / 2; poly = np.array([[1.6, -t], [PH[k] - 1.6, -t], [PH[k] - 1.6, t], [1.6, t]]) * sc
        a.add_patch(Polygon(poly @ R.T + p * sc + off, closed=True, fc=BLADE, ec=INK, lw=0.8)); a.add_patch(Circle(p * sc + off, 1.4, fc="white", ec=INK, lw=0.8, zorder=5))
    lab(a, off[0] + 22, off[1] - 11, f"finger {i + 1}: {s}", fs=8.5)
lim(a, -14, 150, -80, 44)
a = ax[2]
a.add_patch(Rectangle((-34, -9), 34, 18, fc=BLADE, ec=INK)); a.add_patch(Rectangle((-8, -4), 9, 8, fc="white", ec="white")); a.add_patch(Rectangle((-7, -3.6), 41, 7.2, fc="#5b6475", ec=INK))
for sgn in (-1, 1):
    a.add_patch(Rectangle((-2.2, 1.5 if sgn > 0 else -8.5), 4.4, 7.0, fc="#c9a227", ec=INK, lw=0.8, zorder=4)); a.add_patch(Rectangle((-1.2, 1.5 if sgn > 0 else -9.0), 2.4, 7.5, fc="#d8dee4", ec=INK, lw=0.8, zorder=5))
for k, dy in enumerate((-0.9, 0, 0.9)): a.plot([-34, 34], [dy, dy], color=TC[k], lw=1.8, zorder=6)
lab(a, 0, -12.5, "stub pins + bushings"); lab(a, 0, 12.5, "cords through the gap"); lim(a, -38, 38, -16, 16)
a = ax[3]
a.plot([-36, -12], [0, 0], color=TC[1], lw=2.2); lab(a, -26, 4, "cord", fs=8.5)
a.add_patch(Polygon([[-12, -2.5], [-7, -2.5], [-5, -1], [7, -1], [9, -2.5], [14, -2.5], [14, 2.5], [9, 2.5], [7, 1], [-5, 1], [-7, 2.5], [-12, 2.5]], closed=True, fc="#7fb3d5", ec=INK)); lab(a, 1, -6, "TPU link", fs=8.5)
a.add_patch(Rectangle((14, -7), 12, 14, fc="#9aa3ad", ec=INK)); lab(a, 20, 0, "key", fs=9)
for y in (-5, -2.5, 0, 2.5, 5): a.plot([26, 28.5], [y, y - 1.2], color=INK, lw=1.2)
a.add_patch(Rectangle((28.5, -9), 3, 18, fc="#d8dee4", ec=INK)); lab(a, 30, -12.5, "rack", fs=8.5); lim(a, -40, 36, -16, 12)
hub_top(ax[4], shoes=(24,)); ax[4].add_patch(Circle((0, 0), 41, fc="none", ec="#f0b72f", lw=2, ls="--", zorder=3)); lab(ax[4], 52, 30, "pusher", "#9a6700", fs=9)
rotor_top(ax[5], (45, 135, 225, 315), "#c8ced6"); rotor_top(ax[5], (45 - 49, 135 + 49, 225 - 49, 315 + 49))
a = ax[6]; rotor_top(a, (45, 135, 225, 315)); a.add_patch(Circle((0, 0), 52, fc="none", ec=MUTE, lw=1, ls="--")); a.add_patch(Circle((0, 0), 6.5, fc="#30363d", ec=INK, zorder=6)); lab(a, 0, -47, "Ø254 mm disc", MUTE, 9)
fig.savefig("hand3_selected.png"); print("ok")
