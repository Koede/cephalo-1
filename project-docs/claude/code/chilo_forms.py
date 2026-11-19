"""Chilo-1 deep-body FORMS (not boxes): lofted shells around the same dock, layout A (saddle thorax) as the stand-in.
Octopus seated, blades fully down inside; near half of the dock section cut away.  Sketch only.
python3 chilo_forms.py -> forms_grid.png"""
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from geom import frustum, sphere, merge, Scene, torus, box
from render import Camera, render
import chilo_dock_designs as DD
from chilo_build_concepts import PRINT, PRINT2, CARBON, SERVO, STEEL, ACC, LEGC, FOOT, Y, Z, FONT, HUBC

TOP = 0.53
SHELL = (0.80, 0.82, 0.85); INNER = (0.62, 0.65, 0.69)
SEGS = [(-0.19, 0.19, True), (0.22, 0.48, False), (0.51, 0.77, False), (-0.48, -0.22, False), (-0.77, -0.51, False)]
LEGX = [(0.64, 0), (0.35, 1), (0.10, 2), (-0.10, 3), (-0.35, 4), (-0.64, 5)]


def loft(xs, prof, col, th0=0.0, th1=2 * np.pi, nth=44, cap=True, n=2.6):
    th = np.linspace(th0, th1, nth, endpoint=(th1 - th0) < 2 * np.pi - 1e-6); full = (th1 - th0) > 2 * np.pi - 1e-6
    V = []
    for x in xs:
        a, b, cz = prof(x); c, s = np.cos(th), np.sin(th)
        V.append(np.stack([np.full(len(th), x), a * np.sign(c) * np.abs(c) ** (2 / n), np.minimum(cz + b * np.sign(s) * np.abs(s) ** (2 / n), TOP)], 1))
    m, k = len(xs), len(th); V = np.concatenate(V); F = []
    for i in range(m - 1):
        for j in range(k if full else k - 1):
            a0 = i * k + j; a1 = i * k + (j + 1) % k
            F += [[a0, a1, a1 + k], [a0, a1 + k, a0 + k]]
    if cap and full:
        for i in (0, m - 1):
            V = np.vstack([V, V[i * k:(i + 1) * k].mean(0)])
            F += [[len(V) - 1, i * k + j, i * k + (j + 1) % k] for j in range(k)]
    F = np.array(F)
    return V, F, np.tile(np.asarray(col, float), (len(F), 1))


def legs(sc, x, i, a, zh):
    for s in (1, -1):
        fly = (i % 2 == 0) == (s > 0)
        hip = np.array([x, s * a, zh]); k0 = hip + s * Y * 0.07
        knee = k0 + s * Y * 0.16 + Z * 0.13; foot = np.array([x, s * (a + 0.33), 0.02])
        sc.add(merge([sphere(hip, 0.034, SERVO, n=12, m=8), frustum(hip, k0, 0.018, 0.018, PRINT2, n=10), sphere(k0, 0.028, SERVO, n=10, m=6),
                      frustum(k0, knee, 0.016, 0.014, LEGC, n=10), sphere(knee, 0.024, SERVO, n=10, m=6),
                      frustum(knee, foot, 0.014, 0.009, ACC if fly else LEGC, n=10), sphere(foot, 0.02, FOOT, n=10, m=6)]))


def dock_parts(sc):
    sc.add(torus((0, 0, TOP + 0.002), Z, 0.109, 0.009, HUBC, n=56, m=10))
    for k in range(4):
        a = np.pi / 4 + np.pi / 2 * k; er = np.array([np.cos(a), np.sin(a), 0]); et = np.array([-np.sin(a), np.cos(a), 0])
        sc.add(box(np.array([0, 0, TOP - 0.115]) + er * 0.079, np.column_stack([er, et, Z]), (0.003, 0.046, 0.230), ACC))
    DD.octopus(sc, 0.0, TOP + 0.010, bl=0.209)


def joints(sc, zfun):
    for x in (0.205, 0.495, -0.205, -0.495):
        sc.add(sphere((x, 0, zfun(x)), 0.04, STEEL, n=14, m=8))


def shell(sc, prof, bulge=0.0, cut=True):
    for x0, x1, isdock in SEGS:
        def p(x, x0=x0, x1=x1):
            a, b, cz = prof(x); u = (x - x0) / (x1 - x0); g = 1 - bulge + bulge * np.sin(np.pi * u) ** 0.6
            return a * g, b * g, cz
        xs = np.linspace(x0, x1, 22)
        if isdock and cut:
            sc.add(loft(xs, p, INNER, -np.pi / 2, np.pi / 2))                       # far half only: cutaway
        else:
            sc.add(loft(xs, p, SHELL))
    for x, i in LEGX:
        a, b, cz = prof(x); legs(sc, x, i, a * 0.97, cz - 0.25 * b)


def f1(sc):       # barrel segments, deep all along
    t = lambda x: 1 - 0.22 * np.clip((abs(x) - 0.45) / 0.32, 0, 1)
    prof = lambda x: (0.155 * t(x), 0.135 * t(x), TOP - 0.135)
    shell(sc, prof, bulge=0.12); joints(sc, lambda x: TOP - 0.135)


def f2(sc):       # wasp thorax: deep only at the dock, slender ends, level back
    def prof(x):
        g = np.exp(-(x / 0.27) ** 2)
        b = 0.062 + 0.073 * g
        return 0.095 + 0.065 * g, b, TOP - b
    shell(sc, prof, bulge=0.05); joints(sc, lambda x: TOP - 0.06)


def f3(sc):       # open rib cage: spine tube + hoops, same outline as f2
    def prof(x):
        g = np.exp(-(x / 0.27) ** 2); b = 0.062 + 0.073 * g
        return 0.095 + 0.065 * g, b, TOP - b
    for x0, x1, isdock in SEGS:
        sc.add(frustum((x0, 0, TOP - 0.012), (x1, 0, TOP - 0.012), 0.012, 0.012, CARBON, n=14))
        for s in (1, -1):
            pts = [np.array([x, s * prof(x)[0] * 0.75, prof(x)[2] - prof(x)[1] * 0.78]) for x in np.linspace(x0, x1, 8)]
            sc.add(merge([frustum(p, q, 0.006, 0.006, CARBON, n=8) for p, q in zip(pts[:-1], pts[1:])]))
        for x in np.linspace(x0 + 0.015, x1 - 0.015, 7 if isdock else 5):
            a, b, cz = prof(x); th = np.linspace(0, 2 * np.pi, 33)
            P = [np.array([x, a * np.sign(np.cos(t)) * abs(np.cos(t)) ** 0.77, min(cz + b * np.sign(np.sin(t)) * abs(np.sin(t)) ** 0.77, TOP)]) for t in th]
            sc.add(merge([frustum(p, q, 0.005, 0.005, SHELL, n=6) for p, q in zip(P[:-1], P[1:])]))
    for x, i in LEGX:
        a, b, cz = prof(x); legs(sc, x, i, a * 0.97, cz - 0.25 * b)
    joints(sc, lambda x: TOP - 0.02)


def f4(sc):       # mirror dome: a bowl under the octopus (its mantle mirrored), slender tube segments
    R = 0.19
    bowl = lambda x: (0.165 * np.sqrt(max(1e-4, 1 - (x / R) ** 2)), 0.265 * np.sqrt(max(1e-4, 1 - (x / R) ** 2)), TOP)
    sc.add(loft(np.linspace(-R + 0.002, R - 0.002, 30), bowl, INNER, -np.pi / 2, np.pi / 2, n=2.0))
    prof = lambda x: (0.085, 0.065, TOP - 0.075)
    for x0, x1, isdock in SEGS[1:]:
        def p(x, x0=x0, x1=x1):
            u = (x - x0) / (x1 - x0); g = 0.9 + 0.1 * np.sin(np.pi * u) ** 0.6
            return 0.085 * g, 0.065 * g, TOP - 0.075
        sc.add(loft(np.linspace(x0, x1, 16), p, SHELL))
    for x, i in LEGX:
        if abs(x) < 0.2:
            a = bowl(x)[0]; legs(sc, x, i, a * 0.95, TOP - 0.07)
        else:
            legs(sc, x, i, 0.083, TOP - 0.09)
    joints(sc, lambda x: TOP - 0.075)


FORMS = [(f1, "1  Barrel segments", "Rounded barrel sections, deep along the whole body; most room inside for packs and actuators."),
         (f2, "2  Wasp thorax", "Deep only where the blades are: the belly swells under the dock and the ends stay slender and light."),
         (f3, "3  Open rib cage", "Same outline as 2 as carbon spine + printed hoops, no skin (like the horse body's ribs); lightest."),
         (f4, "4  Mirror dome", "A bowl under the octopus mirrors its mantle, so the pair read as one sphere; slim tube segments.")]


def still(fn, title, line, Wp=960, Hp=600):
    sc = Scene(); fn(sc); dock_parts(sc)
    tgt = np.array([0.0, 0, 0.42]); az, el = np.radians(-62), np.radians(15)
    eye = tgt + 2.45 * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
    img = Image.fromarray(render(sc.prims, Camera(eye, tgt, 36), Wp, Hp, ss=2, fog=(7, 16)))
    dr = ImageDraw.Draw(img)
    dr.rectangle([0, 0, Wp, 78], fill=(255, 255, 255))
    dr.text((20, 8), title, font=ImageFont.truetype(FONT % "-Bold", 30), fill=(25, 30, 35))
    dr.text((22, 48), line, font=ImageFont.truetype(FONT % "", 17), fill=(70, 78, 86))
    dr.rectangle([0, 0, Wp - 1, Hp - 1], outline=(200, 205, 210), width=2)
    return img


if __name__ == "__main__":
    G = Image.new("RGB", (1920, 1200), "white")
    for k, (fn, t, l) in enumerate(FORMS):
        G.paste(still(fn, t, l), ((k % 2) * 960, (k // 2) * 600))
    G.save("forms_grid.png"); print("forms_grid.png")
