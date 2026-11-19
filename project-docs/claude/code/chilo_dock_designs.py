"""Chilo-1 body designs built AROUND the octopus dock (constraint: Cephalo-1 must couple to it).
Whole-robot sketches with Cephalo-1 hovering over the hub, blades down, about to dock.  No CAD, no sim; masses est.
python3 chilo_dock_designs.py -> dock_<k>.png, dock_grid.png"""
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from geom import frustum, sphere, merge, Scene, torus, box
from render import Camera, render
import chilo_build_concepts as BC
from chilo_build_concepts import B, cyl, dock_hub, PRINT, PRINT2, CARBON, SERVO, STEEL, ACC, PACK, LEGC, FOOT, X, Y, Z, FONT

H = 0.17
OCT, OCT2 = (0.91, 0.51, 0.23), (0.79, 0.40, 0.12)


def leg(sc, x, yhip, s, flyer=False, z=H, reach=0.15):
    hip = np.array([x, s * yhip, z]); k0 = hip + s * Y * 0.045
    knee = k0 + s * Y * 0.085 + Z * 0.07; foot = np.array([x, s * (yhip + reach), 0.012])
    B(sc, hip - s * Y * 0.012, (0.0247, 0.0452, 0.035), SERVO)
    sc.add(merge([frustum(hip + s * Y * 0.01, k0, 0.011, 0.011, PRINT2, n=10), sphere(k0, 0.016, SERVO, n=10, m=6),
                  frustum(k0, knee, 0.009, 0.008, LEGC, n=10), sphere(knee, 0.014, SERVO, n=10, m=6),
                  frustum(knee, foot, 0.008, 0.005, ACC if flyer else LEGC, n=10), sphere(foot, 0.012, FOOT, n=10, m=6)]))


def pair(sc, x, yhip, i, z=H):
    for s in (1, -1):
        leg(sc, x, yhip, s, flyer=((i % 2 == 0) == (s > 0)), z=z)


def seg(sc, x, i, w=0.112):
    B(sc, (x, 0, H), (0.084, w, 0.05), PRINT); pair(sc, x, w / 2 + 0.004, i)


def joint(sc, x, z=H):
    sc.add(merge([sphere((x, 0, z), 0.017, STEEL, n=12, m=8), box(np.array([x, 0, z + 0.022]), np.eye(3), (0.03, 0.03, 0.012), ACC)]))


def octopus(sc, x, zc):
    """Cephalo-1 stand-in: plate underside at zc; 4 Scabbard blades hanging down at the socket azimuths."""
    b = np.array([x, 0, zc])
    parts = [frustum(b, b + Z * 0.006, 0.125, 0.125, (0.70, 0.73, 0.77), n=40)]
    ph = np.linspace(0, np.pi / 2, 8)
    for p0, p1 in zip(ph[:-1], ph[1:]):
        parts.append(frustum(b + Z * (0.006 + 0.15 * np.sin(p0)), b + Z * (0.006 + 0.15 * np.sin(p1)), 0.125 * np.cos(p0), 0.125 * np.cos(p1), OCT, n=40, caps=False))
    parts.append(frustum(b + Z * 0.10, b + Z * 0.224, 0.09, 0.09, OCT2, n=32))
    sc.add(merge(parts))
    sc.add(frustum(b - Z * 0.034, b - Z * 0.003, 0.057, 0.057, SERVO, n=24))
    for k in range(4):
        a = np.pi / 4 + np.pi / 2 * k; er = np.array([np.cos(a), np.sin(a), 0]); et = np.array([-np.sin(a), np.cos(a), 0])
        sc.add(box(b + er * 0.070 - Z * 0.075, np.column_stack([er, et, Z]), (0.013, 0.037, 0.120), (0.08, 0.09, 0.10)))
    for i in range(8):
        a = np.radians(22.5 + 45 * i); er = np.array([np.cos(a), np.sin(a), 0]); r = b + 0.125 * er + 0.02 * Z
        pts = [r, r + 0.13 * er + 0.04 * Z, r + 0.19 * er + 0.15 * Z, r + 0.17 * er + 0.27 * Z]
        sc.add(merge([frustum(p, q, 0.016, 0.014, OCT2, n=10) for p, q in zip(pts[:-1], pts[1:])] + [sphere(p, 0.016, OCT2, n=8, m=6) for p in pts[1:]]))


# ------------------------------------------------------------------ designs (return hub x, collar top z)
def d1(sc):
    """Saddle thorax: round 2-plate carbon thorax = the dock; 2-segment trains front and rear."""
    for zz in (H + 0.026, H - 0.026):
        sc.add(frustum((0, 0, zz - 0.0015), (0, 0, zz + 0.0015), 0.128, 0.128, CARBON, n=48))
    for a in np.radians([0, 60, 120, 180, 240, 300]):
        cyl(sc, (0.115 * np.cos(a), 0.115 * np.sin(a), H - 0.025), (0.115 * np.cos(a), 0.115 * np.sin(a), H + 0.025), 0.004, STEEL, n=8)
    for sx in (1, -1):
        B(sc, (sx * 0.05, 0.0, H), (0.035, 0.075, 0.03), PACK)
    pair(sc, 0.06, 0.105, 2); pair(sc, -0.06, 0.105, 3)
    for i, x in ((0, 0.335), (1, 0.215), (4, -0.215), (5, -0.335)):
        seg(sc, x, i)
    for x in (0.15, 0.275, -0.15, -0.275):
        joint(sc, x)
    zt = H + 0.0275 + 0.030
    dock_hub(sc, 0.0, zt - 0.105, "none")
    return 0.0, zt


def d2(sc):
    """Twin-rail low-rider: two carbon rails outside the socket circle, the hub hangs between them."""
    for s in (1, -1):
        cyl(sc, (0.36, s * 0.100, H), (-0.36, s * 0.100, H), 0.009, CARBON, n=16)
    for i in range(6):
        x = 0.30 - 0.12 * i
        B(sc, (x, 0, H), (0.030, 0.236, 0.026), PRINT) if abs(x) > 0.09 else [B(sc, (x, s * 0.105, H), (0.030, 0.03, 0.026), PRINT) for s in (1, -1)]
        pair(sc, x, 0.122, i)
        if abs(x) > 0.09: B(sc, (x, 0, H - 0.03), (0.075, 0.035, 0.03), PACK)
    zt = H + 0.045
    dock_hub(sc, 0.0, zt - 0.105, "none")
    for s in (1, -1):
        for dx in (0.045, -0.045):
            B(sc, (dx, s * 0.100, H + 0.012), (0.03, 0.028, 0.016), ACC)
    return 0.0, zt


def d3(sc):
    """Centaur: the octopus is the head on a 4-leg chest; a 4-segment tail follows on spine joints."""
    B(sc, (0, 0, H), (0.20, 0.19, 0.055), PRINT)
    for sx in (1, -1): B(sc, (sx * 0.05, 0, H), (0.035, 0.075, 0.03), PACK)
    pair(sc, 0.06, 0.099, 0); pair(sc, -0.06, 0.099, 1)
    for k, x in enumerate((-0.20, -0.32, -0.44, -0.56)):
        seg(sc, x, k + 2, w=0.112 - 0.008 * k)
    for x in (-0.13, -0.26, -0.38, -0.50):
        joint(sc, x)
    zt = H + 0.0275 + 0.097
    dock_hub(sc, 0.0, zt - 0.105, "saddle")
    return 0.0, zt


def d4(sc):
    """Rigid core + moving ends: 4 leg pairs on a rigid deck with the dock sunk into it; head and tail on joints."""
    for zz in (H + 0.026, H - 0.026):
        B(sc, (0, 0, zz), (0.46, 0.20, 0.003), CARBON)
    for x in (0.2, 0.07, -0.07, -0.2):
        for s in (1, -1): cyl(sc, (x, s * 0.092, H - 0.025), (x, s * 0.092, H + 0.025), 0.004, STEEL, n=8)
    for x in (0.18, -0.18): B(sc, (x, 0, H), (0.075, 0.035, 0.03), PACK)
    for i, x in ((1, 0.18), (2, 0.06), (3, -0.06), (4, -0.18)):
        pair(sc, x, 0.104, i)
    seg(sc, 0.33, 0); seg(sc, -0.33, 5); joint(sc, 0.26); joint(sc, -0.26)
    zt = H + 0.0275 + 0.030
    dock_hub(sc, 0.0, zt - 0.105, "none")
    return 0.0, zt


def d5(sc):
    """Articulated at the dock: front and rear 3-pair frames pivot on a ring bearing concentric with the hub."""
    for sx in (1, -1):
        for s in (1, -1):
            cyl(sc, (sx * 0.06, s * 0.03, H), (sx * 0.37, s * 0.05, H), 0.008, CARBON, n=14)
        for k in range(3):
            x = sx * (0.13 + 0.11 * k); i = (2 - k) if sx > 0 else (3 + k)
            B(sc, (x, 0, H), (0.030, 0.13, 0.026), PRINT); pair(sc, x, 0.069, i)
            B(sc, (x, 0, H - 0.03), (0.075, 0.035, 0.03), PACK)
    sc.add(merge([torus((0, 0, H + 0.004), Z, 0.070, 0.008, ACC, n=48, m=10), torus((0, 0, H - 0.008), Z, 0.070, 0.008, STEEL, n=48, m=10),
                  frustum((0, 0, H - 0.014), (0, 0, H + 0.016), 0.058, 0.058, PRINT2, n=40)]))
    BC.servo(sc, (0.10, 0.0, H + 0.03), Z, X)
    zt = H + 0.016 + 0.097
    dock_hub(sc, 0.0, zt - 0.105, "saddle")
    return 0.0, zt


def d6(sc):
    """Modular pods: identical leg pods on bayonet spine couplers; a wide DOCK POD takes any slot (or two)."""
    xs = [0.36, 0.22, -0.22, -0.36]
    for i, x in zip((0, 1, 4, 5), xs):
        seg(sc, x, i)
    for x in (0.29, -0.29, 0.155, -0.155):
        B(sc, (x, 0, H), (0.04, 0.045, 0.045), SERVO)
        for e in (1, -1): cyl(sc, (x + e * 0.02, 0, H), (x + e * 0.028, 0, H), 0.016, ACC, n=24)
    sc.add(frustum((0, 0, H - 0.03), (0, 0, H + 0.03), 0.128, 0.128, PRINT, n=8))
    pair(sc, 0.055, 0.108, 2); pair(sc, -0.055, 0.108, 3)
    zt = H + 0.03 + 0.030
    dock_hub(sc, 0.0, zt - 0.105, "none")
    return 0.0, zt


DESIGNS = [
    (d1, "A  Saddle thorax", ["Round two-plate carbon thorax IS the dock: sockets pass down between the plates.",
                              "4 legs on the thorax, 2-segment trains front and rear (4 spine joints, 44 servos).",
                              "Keeps the centipede wave; the thorax's 4 legs take most of the 12.8 kg."]),
    (d2, "B  Twin-rail low-rider", ["Two carbon rails run outside the socket circle; the hub hangs between them,",
                                    "so the octopus rides 60 mm lower.  No spine joints (36 servos), all 12 legs share",
                                    "the load.  Strongest and simplest; turns by stepping only."]),
    (d3, "C  Centaur: octopus as the head", ["4-leg chest module carries the dock up front; a 4-segment tail follows on",
                                             "spine joints (44 servos).  Octopus arms reach forward to work.  The chest's",
                                             "4 legs carry nearly all 12.8 kg: they need bigger servos."]),
    (d4, "D  Rigid core + moving ends", ["Dock sunk into a rigid 4-pair carbon deck; only head and tail segments are",
                                         "jointed (2 joints, 40 servos).  8 legs carry the octopus, no joint is loaded;",
                                         "the head can still lift and steer."]),
    (d5, "E  Articulated at the dock", ["Front and rear 3-pair frames pivot on one ring bearing concentric with the",
                                        "hub (orange), steered by one servo (37 servos).  The octopus sits on the pivot,",
                                        "so its weight splits evenly front/rear whatever the bend."]),
    (d6, "F  Modular pods + dock pod", ["Identical leg pods on bayonet couplers (orange, 5 pogo pins); a wide dock pod",
                                        "with its own 4 legs drops into any slot.  Add pods, or a second dock pod for",
                                        "two octopuses.  Heaviest joints; pins pass octopus power into the bus."]),
]


def still(fn, title, lines, W=960, H_=640):
    sc = Scene(); hx, zt = fn(sc)
    octopus(sc, hx, zt + 0.14)
    tgt = np.array([hx - 0.05 if fn is d3 else 0.0, 0, 0.31])
    if fn is d3: tgt[0] = -0.22
    az, el = np.radians(-52), np.radians(17)
    eye = tgt + 2.0 * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
    img = Image.fromarray(render(sc.prims, Camera(eye, tgt, 36), W, H_, ss=2, ground=(tgt[0], 0), fog=(4, 10)))
    dr = ImageDraw.Draw(img)
    dr.rectangle([0, 0, W, 124], fill=(255, 255, 255))
    dr.text((20, 10), title, font=ImageFont.truetype(FONT % "-Bold", 30), fill=(25, 30, 35))
    for i, s in enumerate(lines):
        dr.text((22, 50 + 23 * i), s, font=ImageFont.truetype(FONT % "", 17), fill=(70, 78, 86))
    dr.rectangle([0, 0, W - 1, H_ - 1], outline=(200, 205, 210), width=2)
    return img


if __name__ == "__main__":
    G = Image.new("RGB", (1920, 1920), "white")
    for k, (fn, t, l) in enumerate(DESIGNS):
        im = still(fn, t, l); im.save(f"dock_{k + 1}.png"); G.paste(im, ((k % 2) * 960, (k // 2) * 640))
    G.save("dock_grid.png"); print("dock_grid.png")
