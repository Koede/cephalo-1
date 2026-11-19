"""Chilo-1 dock-centred designs scaled up 1.8x so Cephalo-1's Scabbard blades can extend to their FULL stroke
(tips 224 mm below the plate) inside the body.  Sketch: the centipede geometry is scaled as a whole (so its actuators
are drawn 1.8x too: real ones must be re-chosen); hub, blades and octopus are true size.
python3 chilo_dock_scaled.py -> dock_scaled_grid.png"""
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from geom import frustum, merge, Scene, torus, box
from render import Camera, render
import chilo_dock_designs as DD
from chilo_build_concepts import HUBC, HUBD, SOCK, LATCH, Z, FONT

S = 1.8
DEPTH = 0.226            # socket depth below the collar top: blade tip at 224 mm when fully extended
DD.dock_hub = lambda *a, **k: None


def deep_hub(sc, x, zt):
    sc.add(merge([frustum((x, 0, zt - 0.030), (x, 0, zt - 0.014), 0.112, 0.118, HUBC, n=56), torus((x, 0, zt - 0.007), Z, 0.109, 0.008, HUBC, n=56, m=10),
                  frustum((x, 0, zt - 0.032), (x, 0, zt - 0.030), 0.098, 0.098, (0.75, 0.77, 0.80), n=48),
                  frustum((x, 0, zt - 0.040), (x, 0, zt - 0.033), 0.0475, 0.0475, HUBD, n=36),
                  frustum((x, 0, zt - DEPTH - 0.006), (x, 0, zt - DEPTH), 0.088, 0.088, HUBD, n=48)]))
    for k in range(4):
        a = np.pi / 4 + np.pi / 2 * k; er = np.array([np.cos(a), np.sin(a), 0]); et = np.array([-np.sin(a), np.cos(a), 0])
        R = np.column_stack([er, et, Z]); c = np.array([x, 0, 0]) + er * 0.0698
        sc.add(box(c + Z * (zt - 0.014 - (DEPTH - 0.014) / 2), R, (0.017, 0.047, DEPTH - 0.014), (0.93, 0.49, 0.13)))     # scabbard (orange)
        sc.add(box(c + Z * (zt - 0.0295), R, (0.0096, 0.036, 0.004), SOCK), same=True)
        sc.add(box(np.array([x, 0, 0]) + er * 0.083 + Z * (zt - 0.042), R, (0.010, 0.014, 0.010), LATCH))
    for k in range(4):
        a = np.pi / 2 * k; p = np.array([x + 0.026 * np.cos(a), 0.026 * np.sin(a), zt - 0.033])
        sc.add(frustum(p, p + Z * 0.007, 0.009, 0.002, HUBC, n=12))


NOTES = {"A": ["Thorax grows to a 460 mm disc, 90 mm deep; the four scabbards (orange) hang 150 mm below it", "between the thorax legs.  4 spine joints, wave gait kept."],
         "B": ["Rails 360 mm apart, hub between them; scabbards hang clear below the rails.", "No spine joints; lowest ride for the octopus."],
         "C": ["Chest module 360 x 340 x 100 mm with the scabbards through it; tail of 4 segments.", "Octopus is the head; chest legs carry almost all of it."],
         "D": ["Rigid deck 830 x 360 mm; scabbards pass through the deck between its 4 leg pairs.", "Head and tail segments still jointed."],
         "E": ["Pivot ring grows to 250 mm so the four scabbards pass down through its bore:", "the octopus's blades become the pivot's king-pin."],
         "F": ["Dock pod 460 mm across, 110 mm deep, scabbards below it; leg pods 150 x 200 mm.", "Pods still swap on bayonet couplers."]}


def still(fn, title, W=960, H_=640):
    sc = Scene(); hx, zt = fn(sc)
    sc.prims = [(V * S, F, C, pid) for V, F, C, pid in sc.prims]
    sc._id = max(p[3] for p in sc.prims) + 1
    ztS = DD.H * S + (0.105 if fn in (DD.d3, DD.d5) else 0.075)
    if fn is DD.d2: ztS = DD.H * S + 0.06
    deep_hub(sc, hx * S, ztS)
    DD.octopus(sc, hx * S, ztS + 0.27, bl=0.209)
    tgt = np.array([-0.40 if fn is DD.d3 else 0.0, 0, 0.42])
    az, el = np.radians(-52), np.radians(15)
    eye = tgt + 3.1 * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
    img = Image.fromarray(render(sc.prims, Camera(eye, tgt, 36), W, H_, ss=2, ground=(tgt[0], 0), fog=(6, 14)))
    dr = ImageDraw.Draw(img)
    dr.rectangle([0, 0, W, 100], fill=(255, 255, 255))
    dr.text((20, 10), title, font=ImageFont.truetype(FONT % "-Bold", 30), fill=(25, 30, 35))
    for i, s in enumerate(NOTES[title[0]]):
        dr.text((22, 50 + 23 * i), s, font=ImageFont.truetype(FONT % "", 17), fill=(70, 78, 86))
    dr.rectangle([0, 0, W - 1, H_ - 1], outline=(200, 205, 210), width=2)
    return img


if __name__ == "__main__":
    G = Image.new("RGB", (1920, 1920 + 70), "white")
    ImageDraw.Draw(G).text((20, 16), "Chilo-1 at 1.8x: 1.3 m long, body 306 mm off the floor.  Octopus drawn true size, blades fully extended (224 mm); orange = full-depth scabbards.",
                           font=ImageFont.truetype(FONT % "-Bold", 22), fill=(25, 30, 35))
    for k, (fn, t, l) in enumerate(DD.DESIGNS):
        G.paste(still(fn, t), ((k % 2) * 960, 70 + (k // 2) * 640))
    G.save("dock_scaled_grid.png"); print("dock_scaled_grid.png")
