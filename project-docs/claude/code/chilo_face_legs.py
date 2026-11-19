"""Chilo-1 + Cephalo-1 coupled: four of the octopus's arms plant as the centipede's front legs, four stay as hands.
Posed sketch, true size, no sim.   python3 chilo_face_legs.py -> face_legs.png"""
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from geom import frustum, sphere, merge, Scene
from render import Camera, render
import chilo_dock_designs as DD, chilo_face_dock as FD
from chilo_build_concepts import FONT, FOOT

OCT2 = DD.OCT2; LEGARM = (0.93, 0.60, 0.25)
Yv = np.array([0, 1.0, 0])


def arm(sc, pts, col, foot=False):
    parts = [frustum(p, q, 0.016, 0.014, col, n=10) for p, q in zip(pts[:-1], pts[1:])] + [sphere(p, 0.016, col, n=8, m=6) for p in pts[1:-1]]
    parts.append(sphere(pts[-1], 0.022, FOOT if foot else col, n=10, m=6))
    sc.add(merge(parts))


def build(sc):
    FD.robot(sc, [0, 0, 0])
    p0, p1, d = FD.chain([0, 0, 0])[0]
    p = p1 + d * 0.012
    tmp = Scene(); DD.octopus(tmp, 0.0, 0.0, bl=0.0)
    R = np.column_stack([[d[2], 0, -d[0]], [0, 1, 0], d])
    for V, F, C, pid in tmp.prims[:-8]:
        sc.add((V @ R.T + p, F, C))
    xl = np.array([d[2], 0, -d[0]])                       # "down" for a horizontal body
    for i in range(8):
        a = np.radians(22.5 + 45 * i); e = np.cos(a) * xl + np.sin(a) * Yv; s = np.sign(np.sin(a))
        root = p + 0.125 * e + 0.02 * d
        if i in (0, 7):                                   # lower pair: inner front legs
            foot = np.array([p[0] + 0.06, s * 0.25, 0.022]); knee = root + 0.10 * e + s * Yv * 0.10 + d * 0.05
            arm(sc, [root, knee, 0.5 * (knee + foot) + s * Yv * 0.05, foot], LEGARM, foot=True)
        elif i in (1, 6):                                 # side pair: outer front legs
            foot = np.array([p[0] + 0.17, s * 0.39, 0.022]); knee = root + 0.17 * e + d * 0.08 + np.array([0, 0, 0.10])
            arm(sc, [root, knee, 0.5 * (knee + foot) + s * Yv * 0.04, foot], LEGARM, foot=True)
        else:                                             # upper four: free hands, reaching forward
            arm(sc, [root, root + 0.13 * e + 0.05 * d, root + 0.20 * e + 0.17 * d, root + 0.15 * e + 0.30 * d], OCT2)


def view(az, el, dist, tgt, title, line, Wp=960, Hp=640):
    sc = Scene(); build(sc)
    tgt = np.array(tgt); az, el = np.radians(az), np.radians(el)
    eye = tgt + dist * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
    img = Image.fromarray(render(sc.prims, Camera(eye, tgt, 36), Wp, Hp, ss=2, fog=(8, 18)))
    dr = ImageDraw.Draw(img)
    dr.rectangle([0, 0, Wp, 78], fill=(255, 255, 255))
    dr.text((20, 8), title, font=ImageFont.truetype(FONT % "-Bold", 30), fill=(25, 30, 35))
    dr.text((22, 48), line, font=ImageFont.truetype(FONT % "", 16), fill=(70, 78, 86))
    dr.rectangle([0, 0, Wp - 1, Hp - 1], outline=(200, 205, 210), width=2)
    return img


if __name__ == "__main__":
    G = Image.new("RGB", (1920, 640), "white")
    G.paste(view(-68, 12, 3.6, (0.05, 0, 0.35), "5  Arms become legs", "Four octopus arms (light orange) plant as two extra front leg pairs: 16 legs.  Four stay up as hands."), (0, 0))
    G.paste(view(-22, 14, 1.9, (0.85, 0, 0.30), "Front view", "The planted arms carry the octopus's own weight, so it no longer hangs off the face."), (960, 0))
    G.save("face_legs.png"); print("face_legs.png")
