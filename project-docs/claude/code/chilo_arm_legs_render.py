"""Render: Cephalo-1 docked on Chilo-1's face, arms 0/7 and 1/6 planted as the front legs (solved poses from
arm_leg_solve.py, 70 % joint margin, reach <= 80 %), arms 2-5 up as hands; no centipede head legs.
python3 chilo_arm_legs_render.py -> arm_legs.png"""
import json
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from geom import frustum, sphere, merge, Scene, torus, box
from render import Camera, render
import chilo_face_dock as FD, chilo_dock_designs as DD, chilo_arm_legs as A
from chilo_build_concepts import FONT, FOOT, SERVO, HUBC, ACC

ARM = (0.79, 0.40, 0.12); LEGARM = (0.95, 0.62, 0.27); Yv = np.array([0, 1.0, 0])


def centipede(sc):
    ch = FD.chain([0, 0, 0])
    for i, (p0, p1, d) in enumerate(ch):
        FD.seg(sc, p0, p1, FD.RAD[i])
        if i < 5: sc.add(sphere(p0 - d * FD.GAP / 2, 0.04, (0.72, 0.74, 0.77), n=12, m=8))
        if i == 0: continue                                   # no head legs: the octopus arms are the front legs
        c = 0.5 * (p0 + p1)
        for s in (1, -1):
            FD.leg(sc, c + s * Yv * FD.RAD[i] * 0.9, s, np.array([c[0], s * (FD.RAD[i] + 0.32), 0.02]), (i % 2 == 0) == (s > 0))
    p0, p1, d = ch[0]
    sc.add(torus(p1 + d * 0.004, d, 0.109, 0.009, HUBC, n=48, m=10))
    return p1 + d * 0.012


def octo_body(sc, p):
    tmp = Scene(); DD.octopus(tmp, 0.0, 0.0, bl=0.0)
    R = np.column_stack([[0, 0, -1.0], [0, 1.0, 0], [1.0, 0, 0]])
    for V, F, C, pid in tmp.prims[:-8]:
        sc.add((V @ R.T + p, F, C))


def draw_arm(sc, root, P, spig, tip, col, foot):
    pts = [root] + list(P) + [spig]
    parts = [frustum(a, b, 0.017, 0.016, col, n=10) for a, b in zip(pts[:-1], pts[1:])]
    for k, p in enumerate(P):
        parts.append(sphere(p, 0.024 if k == 0 else 0.020, SERVO, n=10, m=6))
    parts.append(frustum(spig, tip, 0.030, 0.020, (0.25, 0.26, 0.28), n=14))
    parts.append(sphere(tip, 0.022, FOOT if foot else (0.25, 0.26, 0.28), n=10, m=6))
    sc.add(merge(parts))


def build(sc, sol):
    plate = centipede(sc); octo_body(sc, plate)
    for i in range(8):
        root, R0 = A.arm_frame(i, plate)
        if min(i, 7 - i) in sol:
            v = sol[min(i, 7 - i)]
            off = plate - np.array([0.315 + 0.012, 0, A.HC])            # poses were solved with the plate at x 0.327
            P = np.array(v["joints"]) + off; spig = np.array(v["spigot"]) + off; tip = np.array(v["foot"]) + off
            root0, _ = A.arm_frame(min(i, 7 - i), plate)
            if i >= 4:                                         # mirror the solved side
                M = np.diag([1, -1, 1.0]); P = P @ M; spig = spig @ M; tip = tip @ M
            draw_arm(sc, root, P, spig, tip, LEGARM, True)
        else:                                                  # hands: curled up and forward, mid-range joints
            q = np.radians([-30, 0, -45, 0, -35, 0, -20, 0] if i in (3, 4) else [-20, 0, -30, 0, -40, 0, -30, 0])
            P, Ax, spig, tip = A.fk(q, root, R0)
            draw_arm(sc, root, P, spig, tip, ARM, False)


def panel(img, sol, x, y):
    dr = ImageDraw.Draw(img); f = ImageFont.truetype(FONT % "", 18); fb = ImageFont.truetype(FONT % "-Bold", 20)
    dr.rectangle([x - 12, y - 12, x + 860, y + 230], fill=(255, 255, 255), outline=(200, 205, 210), width=2)
    dr.text((x, y), "Planted arms (real Cephalo-1 arm, hand3 folded as the foot)", font=fb, fill=(25, 30, 35))
    rows = [("", "reach", "joints", "load *", "with prop lift **")]
    for k, name in ((1, "arms 1 / 6 (side)"), (0, "arms 0 / 7 (low)")):
        a, b = sol[k], sol_l[k]
        rows.append((name, f"{a['reach'] * 100:.0f} %", f"{a['joint_range'] * 100:.0f} %", f"{a['x_rated']:.2f} x rated", f"{b['x_rated']:.2f} x rated"))
    for r_, row in enumerate(rows):
        for c, t in enumerate(row):
            dr.text((x + [0, 230, 330, 480, 680][c], y + 36 + 28 * r_), t, font=fb if r_ == 0 or c == 0 else f, fill=(25, 30, 35))
    notes = ["reach = foot distance / full arm length (limit 80 %); joint range = worst joint / its limit (limit 70 %)",
             "* 31.4 N on each side arm, 20.9 N on each low arm = 105 N of the octopus's 126 N; the other 21 N goes through the hub",
             "** two free hands spinning as props lift 40 N (est., hand3 thrust unmeasured): 20.9 N side, 14.3 N low, 15 N via hub",
             "Arms 2/5 cannot reach the floor inside the limits: they stay hands with 3/4."]
    for k, t in enumerate(notes):
        dr.text((x, y + 128 + 22 * k), t, font=ImageFont.truetype(FONT % "", 14), fill=(70, 78, 86))


if __name__ == "__main__":
    sol = {0: json.load(open("arm_leg_0_21.json")), 1: json.load(open("arm_leg_1_31.json"))}
    sol_l = {0: json.load(open("arm_leg_0_14.json")), 1: json.load(open("arm_leg_1_21.json"))}
    W, H = 1920, 1080
    sc = Scene(); build(sc, sol)
    tgt = np.array([0.30, 0, 0.30]); az, el = np.radians(-38), np.radians(16)
    eye = tgt + 2.6 * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
    big = Image.fromarray(render(sc.prims, Camera(eye, tgt, 34), W, H, ss=2, fog=(8, 18)))
    tgt2 = np.array([0.80, 0, 0.25]); az2, el2 = np.radians(-4), np.radians(8)
    eye2 = tgt2 + 1.6 * np.array([np.cos(az2) * np.cos(el2), np.sin(az2) * np.cos(el2), np.sin(el2)])
    small = Image.fromarray(render(sc.prims, Camera(eye2, tgt2, 40), 640, 420, ss=2, fog=(6, 14)))
    big.paste(small, (W - 640 - 30, 30))
    dr = ImageDraw.Draw(big)
    dr.rectangle([W - 670, 30, W - 30, 450], outline=(40, 45, 50), width=3)
    dr.text((W - 660, 38), "FRONT", font=ImageFont.truetype(FONT % "-Bold", 20), fill=(25, 30, 35))
    dr.text((30, 24), "Octopus arms as the front legs", font=ImageFont.truetype(FONT % "-Bold", 44), fill=(25, 30, 35))
    dr.text((32, 80), "Cephalo-1 docked on Chilo-1's face.  Arms 0/7 and 1/6 planted, arms 2-5 up as hands; no centipede head legs.",
            font=ImageFont.truetype(FONT % "", 22), fill=(70, 78, 86))
    panel(big, sol, 42, H - 262)
    big.save("arm_legs.png"); print("arm_legs.png")
