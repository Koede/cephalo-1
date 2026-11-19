"""Three-phalanx hand-prop: the selector decision again, for fingers that move at the same time (2026-11-13).
python3 hand3_selector2.py -> hand3_selector2.png.  Masses and times are estimates."""
import numpy as np
import hand3_study as S
from hand3_study import *

SV = "#f0b72f"
def dock_side(a, n_lv=3, tilt=False):
    """Side view: parked rotor, cage followers round the motor, stationary servos below pushing rods up."""
    a.plot([-62, 62], [38, 38], color=BLADE, lw=6, solid_capstyle="round"); lab(a, 44, 44, "rotor, parked", MUTE, 8.5)
    a.add_patch(Rectangle((-17, 10), 34, 24, fc="#30363d", ec=INK)); lab(a, 0, 22, "motor", "white", fs=9)
    a.add_patch(Rectangle((-13, -46), 26, 56, fc="#d8dee4", ec=INK)); lab(a, 0, -40, "arm tip", fs=8.5)
    for sgn in (-1, 1):
        for k in range(n_lv):
            x = sgn * (22 + 7 * k)
            a.plot([x, x], [16 - 3 * k, 36], color=TC[k], lw=1.6)                                   # cord up to the finger
            a.add_patch(Rectangle((x - 2.6, 8 - 3 * k), 5.2, 9, fc=TC[k], ec=INK, lw=0.8, zorder=4))  # follower on the cage
            a.plot([x, x], [-6 - 9 * k, 8 - 3 * k], color=INK, lw=1.4)                               # push rod
            a.add_patch(Rectangle((x - 3.2 if sgn > 0 else x - 3.2, -15 - 9 * k), 6.4, 9, fc=SV, ec=INK, lw=0.8))
    a.plot([-46, 46], [5.5, 5.5], color=MUTE, lw=0.8, ls="--"); lab(a, 53, 5.5, "gap in\nflight", MUTE, 7.5)
    lim(a, -66, 70, -50, 48)

rows = [dict(name="A  Park and dock: one servo per joint", pick=True, lines=[
            ("n", "The hand only works with the rotor stopped, so the rotor parks at a fixed angle and 12 stationary servos each push one joint's follower through a rod. Two more do the spread."),
            ("+", "Every joint of every finger moves on its own, all at the same time, continuously: no steps. A full grasp takes about 0.5 s."),
            ("+", "The servos report their load, so the grip force is measured and set. The latching teeth disappear (no 0.25 mm nozzle)."),
            ("-", "14 × Feetech STS3032 (20 g, 55 N at a 6 mm spool) ≈ 335 g per hand at the arm tip, against about 35 g for the key hub. The arm servos must be re-checked."),
            ("-", "The rotor needs a parking detent, and the rods must clear the spinning cage in flight.")]),
        dict(name="B  Park and dock, two servos per finger", lines=[
            ("n", "Same idea with 8 servos: the base joint has its own, the middle and tip share one cord (as in a human finger). Two more for the spread."),
            ("+", "About 250 g. Fingers still independent and simultaneous."),
            ("-", "No tip-only or middle-only curl: not joint by joint.")]),
        dict(name="C  Three swashplates, nine servos", lines=[
            ("n", "One tilting ring per joint level, as on a helicopter: height and two tilts."),
            ("+", "Simultaneous and continuous, about 260 g."),
            ("-", "Not individual: a ring can favour one side, but moving one finger alone drags its two neighbours half as far.")]),
        dict(name="D  One finger at a time, three pushers", lines=[
            ("n", "The motor turns a finger's three keys under three pushers; that finger's joints move together, then the next finger."),
            ("+", "About 95 g. A full grasp in about 4 s."),
            ("-", "Fingers still take turns.")]),
        dict(name="E  Key hub, one pusher", rej=True, lines=[
            ("-", "One joint at a time, about 20 s for a full grasp. This is what you turned down.")])]
fig, ax = sheet("6 again: fingers that move at the same time", "Independent and simultaneous means one actuator per joint. Nothing powered can ride on the rotor, so the actuators sit on the arm tip.", rows, 10)
dock_side(ax[0]); lab(ax[0], -56, -24, "servos", "#9a6700", 9)
dock_side(ax[1], n_lv=2)
a = ax[2]
a.plot([-62, 62], [38, 38], color=BLADE, lw=6, solid_capstyle="round"); a.add_patch(Rectangle((-17, 10), 34, 24, fc="#30363d", ec=INK)); lab(a, 0, 22, "motor", "white", fs=9)
a.add_patch(Rectangle((-13, -46), 26, 56, fc="#d8dee4", ec=INK))
for k, (y, tl) in enumerate(((2, 9), (-13, -6), (-28, 4))):
    R = rot(np.radians(tl)); p = np.array([[-36, -2.5], [36, -2.5], [36, 2.5], [-36, 2.5]]) @ R.T + [0, y]
    a.add_patch(Polygon(p, closed=True, fc=TC[k], ec=INK, lw=0.8, alpha=0.9))
    for x in (-44, 44): a.add_patch(Rectangle((x - 3, y - 5), 6, 9, fc=SV, ec=INK, lw=0.8))
lim(a, -66, 70, -50, 48)
a = ax[3]; hub_top(a, shoes=())
for j in range(3):
    t = np.radians(45 + (j - 1) * 21); c = np.array([np.cos(t), np.sin(t)]) * 38.5; Rm = rot(t)
    poly = np.array([[-3.5, -3.6], [5, -3.6], [5, 3.6], [-3.5, 3.6]]); a.add_patch(Polygon(poly @ Rm.T + c, closed=True, fc=SV, ec=INK, lw=1, zorder=6))
hub_top(ax[4], shoes=(24,))
fig.savefig("hand3_selector2.png"); print("ok")
