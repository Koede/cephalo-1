"""Chilo-1 deep-body designs: the body is deeper (260 mm) than Cephalo-1's fully extended blades (224 mm), so the octopus
sits ON the body and its blades are INSIDE it.  Dock section drawn with the near wall cut away.  True-size sketch,
no CAD, no sim.   python3 chilo_deep.py -> deep_grid.png"""
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from geom import frustum, sphere, merge, Scene, torus, box
from render import Camera, render
import chilo_dock_designs as DD
from chilo_build_concepts import B, cyl, PRINT, PRINT2, CARBON, SERVO, STEEL, ACC, PACK, LEGC, FOOT, X, Y, Z, FONT, HUBC

HC, D, W = 0.40, 0.26, 0.30          # body centre height, depth, width
TOP, BOT = HC + D / 2, HC - D / 2
E3 = np.eye(3)


def hull(sc, x0, x1, w=W, d=D, col=PRINT):
    B(sc, ((x0 + x1) / 2, 0, HC + (D - d) / 2 * 0 ), (abs(x1 - x0), w, d), col)


def legs(sc, x, i, w=W):
    for s in (1, -1):
        fly = (i % 2 == 0) == (s > 0)
        hip = np.array([x, s * w / 2, HC - 0.04]); k0 = hip + s * Y * 0.07
        knee = k0 + s * Y * 0.16 + Z * 0.13; foot = np.array([x, s * (w / 2 + 0.32), 0.02])
        B(sc, hip, (0.06, 0.05, 0.07), SERVO)
        sc.add(merge([frustum(hip, k0, 0.018, 0.018, PRINT2, n=10), sphere(k0, 0.028, SERVO, n=10, m=6),
                      frustum(k0, knee, 0.016, 0.014, LEGC, n=10), sphere(knee, 0.024, SERVO, n=10, m=6),
                      frustum(knee, foot, 0.014, 0.009, ACC if fly else LEGC, n=10), sphere(foot, 0.02, FOOT, n=10, m=6)]))


def joint(sc, x):
    sc.add(merge([sphere((x, 0, HC), 0.045, STEEL, n=14, m=8), box(np.array([x, 0, HC + 0.06]), E3, (0.05, 0.07, 0.03), ACC)]))


def dock(sc, x, L=0.34, w=W, round_=False):
    """Dock section of the body, near wall (-y) cut away; collar flush with the top; octopus seated, blades inside."""
    t = 0.006
    B(sc, (x, 0, BOT + t / 2), (L, w, t), PRINT)                                   # floor
    B(sc, (x, w / 2 - t / 2, HC), (L, t, D), PRINT)                                # far wall
    for e in (1, -1):
        B(sc, (x + e * (L / 2 - t / 2), 0, HC), (t, w, D), PRINT2)                 # bulkheads
    B(sc, (x, w / 4, TOP - t / 2), (L, w / 2, t), PRINT)                           # far half of the deck
    for zz in (TOP - 0.004, BOT + 0.004):
        cyl(sc, (x - L / 2, -w / 2 + 0.004, zz), (x + L / 2, -w / 2 + 0.004, zz), 0.005, CARBON, n=8)   # near-edge rails
    sc.add(merge([torus((x, 0, TOP + 0.002), Z, 0.109, 0.009, HUBC, n=56, m=10)]))
    for k in range(4):
        a = np.pi / 4 + np.pi / 2 * k; er = np.array([np.cos(a), np.sin(a), 0]); et = np.array([-np.sin(a), np.cos(a), 0])
        R = np.column_stack([er, et, Z]); c = np.array([x, 0, 0]) + er * 0.079
        sc.add(box(c + Z * (TOP - 0.115), R, (0.003, 0.046, 0.230), ACC))           # scabbard back plate (orange)
    DD.octopus(sc, x, TOP + 0.010, bl=0.209)


# ---------------------------------------------------------------- layouts (x forward)
def dA(sc):
    dock(sc, 0.0); legs(sc, 0.10, 2); legs(sc, -0.10, 3)
    for i, x0 in ((0, 0.51), (1, 0.21), (4, -0.47), (5, -0.77)):
        hull(sc, x0, x0 + 0.26); legs(sc, x0 + 0.13, i)
    for x in (0.19, 0.49, -0.19, -0.49): joint(sc, x)
    return -0.0


def dB(sc):
    dock(sc, 0.0)
    hull(sc, 0.17, 0.80); hull(sc, -0.17, -0.80)
    for i, x in enumerate((0.70, 0.42, 0.11, -0.11, -0.42, -0.70)): legs(sc, x, i)
    return 0.0


def dC(sc):
    dock(sc, 0.0, L=0.36); legs(sc, 0.11, 0); legs(sc, -0.11, 1)
    for k in range(4):
        x0 = -0.22 - 0.30 * k - 0.26; w = W - 0.03 * k
        B(sc, (x0 + 0.13, 0, HC), (0.26, w, D - 0.03 * k), PRINT); legs(sc, x0 + 0.13, k + 2, w)
        joint(sc, x0 + 0.28)
    return -0.55


def dD(sc):
    dock(sc, 0.0)
    hull(sc, 0.17, 0.50); hull(sc, -0.17, -0.50)
    for i, x in ((1, 0.38), (2, 0.11), (3, -0.11), (4, -0.38)): legs(sc, x, i)
    hull(sc, 0.54, 0.80); legs(sc, 0.67, 0); hull(sc, -0.54, -0.80); legs(sc, -0.67, 5)
    joint(sc, 0.52); joint(sc, -0.52)
    return 0.0


def dE(sc):
    dock(sc, 0.0, L=0.32)
    sc.add(merge([torus((0, 0, BOT - 0.012), Z, 0.15, 0.014, ACC, n=56, m=10), torus((0, 0, BOT - 0.03), Z, 0.15, 0.012, STEEL, n=56, m=10)]))
    for sx in (1, -1):
        hull(sc, sx * 0.20, sx * 0.86, d=D)
        B(sc, (sx * 0.10, 0, BOT - 0.03), (0.22, 0.10, 0.02), CARBON)
        for k in range(3):
            legs(sc, sx * (0.32 + 0.23 * k), (2 - k) if sx > 0 else (3 + k))
    return 0.0


def dF(sc):
    dock(sc, 0.0); legs(sc, 0.10, 2); legs(sc, -0.10, 3)
    for i, x0 in ((0, 0.55), (1, 0.25), (4, -0.51), (5, -0.81)):
        hull(sc, x0, x0 + 0.26); legs(sc, x0 + 0.13, i)
    for x in (0.21, 0.53, -0.21, -0.53):
        cyl(sc, (x - 0.035, 0, HC), (x + 0.035, 0, HC), 0.07, ACC, n=32)
    return 0.0


DES = [(dA, "A  Saddle thorax", "Dock section with 4 legs in the middle; two segments each end on spine joints (4 joints)."),
       (dB, "B  Rigid hull", "One rigid hull, 12 legs, no spine joints.  Strongest; turns by stepping."),
       (dC, "C  Centaur", "Octopus is the head on a 4-leg chest; tapering 4-segment tail on spine joints."),
       (dD, "D  Rigid core + moving ends", "Rigid 4-pair core around the dock; head and tail segments jointed (2 joints)."),
       (dE, "E  Articulated at the dock", "Front and rear hulls pivot on a ring (orange) under the dock section; one steering servo."),
       (dF, "F  Modular pods", "Separate pods on bayonet couplers (orange); the dock pod takes any slot.")]


def still(fn, title, line, Wp=960, Hp=600):
    sc = Scene(); cx = fn(sc)
    tgt = np.array([cx, 0, 0.40]); az, el = np.radians(-62), np.radians(17)
    eye = tgt + 2.45 * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
    img = Image.fromarray(render(sc.prims, Camera(eye, tgt, 36), Wp, Hp, ss=2, ground=(cx, 0), fog=(7, 16)))
    dr = ImageDraw.Draw(img)
    dr.rectangle([0, 0, Wp, 78], fill=(255, 255, 255))
    dr.text((20, 8), title, font=ImageFont.truetype(FONT % "-Bold", 30), fill=(25, 30, 35))
    dr.text((22, 48), line, font=ImageFont.truetype(FONT % "", 17), fill=(70, 78, 86))
    dr.rectangle([0, 0, Wp - 1, Hp - 1], outline=(200, 205, 210), width=2)
    return img


if __name__ == "__main__":
    G = Image.new("RGB", (1920, 1870), "white")
    ImageDraw.Draw(G).text((20, 20), "Chilo-1 deep body: 260 mm deep, 300 mm wide, about 1.6-1.7 m long.  Octopus seated, blades fully down (224 mm) INSIDE the body; near wall cut away.",
                           font=ImageFont.truetype(FONT % "-Bold", 21), fill=(25, 30, 35))
    for k, (fn, t, l) in enumerate(DES):
        G.paste(still(fn, t, l), ((k % 2) * 960, 70 + (k // 2) * 600))
    G.save("deep_grid.png"); print("deep_grid.png")
