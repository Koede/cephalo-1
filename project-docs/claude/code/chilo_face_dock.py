"""Chilo-1 face dock: the hub is on the centipede's face.  It rears up so the face points up, Cephalo-1 lowers its
blades into the head segment (along the body axis), then the centipede stands back down with the octopus as its head.
Four-step sketch, true size, no sim.   python3 chilo_face_dock.py -> face_dock.png"""
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from geom import frustum, sphere, merge, Scene, torus
from render import Camera, render
import chilo_dock_designs as DD
from chilo_build_concepts import PRINT2, SERVO, STEEL, ACC, LEGC, FOOT, FONT, HUBC

SHELL = (0.80, 0.82, 0.85)
HC = 0.38
LEN = [0.28, 0.20, 0.20, 0.20, 0.20, 0.20]; RAD = [0.125, 0.112, 0.10, 0.095, 0.085, 0.075]; GAP = 0.035
Yv = np.array([0, 1.0, 0])


def chain(ang):
    """ang = pitch (deg above horizontal) of segments 0, 1, 2; segments 3-5 lie flat.  Returns (rear, front, dir) per segment."""
    out = [None] * 6; p = np.array([0.0, 0, HC])
    for i in (2, 1, 0):
        a = np.radians(ang[i]); d = np.array([np.cos(a), 0, np.sin(a)])
        p0 = p + d * GAP; p1 = p0 + d * LEN[i]; out[i] = (p0, p1, d); p = p1
    p = np.array([0.0, 0, HC])
    for i in (3, 4, 5):
        d = np.array([1.0, 0, 0]); p1 = p.copy(); p0 = p1 - d * LEN[i]; out[i] = (p0, p1, d); p = p0 - d * GAP
    return out


def seg(sc, p0, p1, r):
    us = np.linspace(0, 1, 9); rr = [r * (0.86 + 0.14 * np.sin(np.pi * u) ** 0.6) for u in us]
    P = [p0 + (p1 - p0) * u for u in us]
    sc.add(merge([frustum(P[k], P[k + 1], rr[k], rr[k + 1], SHELL, n=28, caps=(k in (0, 7))) for k in range(8)]))


def leg(sc, hip, s, foot, fly):
    k0 = hip + s * Yv * 0.06
    knee = k0 + s * Yv * 0.15 + np.array([0, 0, 0.12]) if foot[2] < 0.05 else 0.5 * (k0 + foot) + s * Yv * 0.10
    sc.add(merge([sphere(hip, 0.03, SERVO, n=10, m=6), frustum(hip, k0, 0.016, 0.016, PRINT2, n=10), sphere(k0, 0.025, SERVO, n=10, m=6),
                  frustum(k0, knee, 0.015, 0.013, LEGC, n=10), sphere(knee, 0.022, SERVO, n=10, m=6),
                  frustum(knee, foot, 0.013, 0.009, ACC if fly else LEGC, n=10), sphere(foot, 0.018, FOOT, n=10, m=6)]))


def octo(sc, p, d, bl):
    tmp = Scene(); DD.octopus(tmp, 0.0, 0.0, bl=bl)
    R = np.column_stack([[d[2], 0, -d[0]], [0, 1, 0], d])
    for V, F, C, pid in tmp.prims:
        sc.add((V @ R.T + p, F, C))


def robot(sc, ang, octo_gap=None, bl=0.209):
    ch = chain(ang)
    for i, (p0, p1, d) in enumerate(ch):
        seg(sc, p0, p1, RAD[i])
        if i < 5: sc.add(sphere(p0 - d * GAP / 2 if i < 3 else p0 - d * GAP / 2, 0.04, STEEL, n=12, m=8))
        c = 0.5 * (p0 + p1) if i else p0 + d * 0.07          # head's legs sit at its rear so the face stays clear
        for s in (1, -1):
            hip = c + s * Yv * RAD[i] * 0.9
            if ang[i] if i < 3 else 0:
                if ang[i] <= 50: foot = np.array([c[0] + 0.10, s * (RAD[i] + 0.30), 0.02])       # planted as props
                else: foot = hip + s * Yv * 0.12 - d * 0.30                                       # tucked along the body
            else:
                foot = np.array([c[0], s * (RAD[i] + 0.32), 0.02])
            leg(sc, hip, s, foot, (i % 2 == 0) == (s > 0))
    p0, p1, d = ch[0]
    sc.add(merge([torus(p1 + d * 0.004, d, 0.109, 0.009, HUBC, n=48, m=10), frustum(p1, p1 + d * 0.003, 0.098, 0.098, (0.2, 0.21, 0.23), n=40)]))
    for k in range(4):
        a = np.pi / 4 + np.pi / 2 * k; e = np.array([d[2], 0, -d[0]]) * np.cos(a) + Yv * np.sin(a)
        sc.add(sphere(p1 + d * 0.004 + e * 0.07, 0.012, ACC, n=8, m=6))
    if octo_gap is not None:
        octo(sc, p1 + d * (0.012 + octo_gap), d, bl)


def step1(sc):
    robot(sc, [0, 0, 0]); DD.octopus(sc, 0.95, 0.85, bl=0.0)


def step2(sc):
    robot(sc, [90, 90, 45], octo_gap=0.30)


def step3(sc):
    robot(sc, [90, 90, 45], octo_gap=0.0)


def step4(sc):
    robot(sc, [0, 0, 0], octo_gap=0.0)


STEPS = [(step1, "1  Walk up", "Hub on the face (grey ring, orange socket mouths).  Octopus hovers nearby."),
         (step2, "2  Rear up", "Front two segments go vertical, the third at 45 deg with its legs planted.  Octopus lines up above, blades down."),
         (step3, "3  Couple", "Blades slide 224 mm down into the head segment along its axis; latch pins lock.  Octopus still holds its own weight."),
         (step4, "4  Stand back down", "The front lowers onto all 12 legs.  The octopus is now the head, arms facing forward.")]


def still(fn, title, line, Wp=960, Hp=640):
    sc = Scene(); fn(sc)
    tgt = np.array([0.0, 0, 0.72]); az, el = np.radians(-68), np.radians(10)
    eye = tgt + 4.5 * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
    img = Image.fromarray(render(sc.prims, Camera(eye, tgt, 36), Wp, Hp, ss=2, fog=(8, 18)))
    dr = ImageDraw.Draw(img)
    dr.rectangle([0, 0, Wp, 78], fill=(255, 255, 255))
    dr.text((20, 8), title, font=ImageFont.truetype(FONT % "-Bold", 30), fill=(25, 30, 35))
    dr.text((22, 48), line, font=ImageFont.truetype(FONT % "", 16), fill=(70, 78, 86))
    dr.rectangle([0, 0, Wp - 1, Hp - 1], outline=(200, 205, 210), width=2)
    return img


if __name__ == "__main__":
    G = Image.new("RGB", (1920, 1280), "white")
    for k, (fn, t, l) in enumerate(STEPS):
        G.paste(still(fn, t, l), ((k % 2) * 960, (k // 2) * 640))
    G.save("face_dock.png"); print("face_dock.png")
