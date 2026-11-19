"""Renders of the hand-prop solids (hand3_cad.py) with the project's rasterizer (render.py, one level up).
python3 hand3_cad_render.py -> hand3_cad_1..4.png.  The model is drawn 10 x life size so the renderer's ground and fog fit."""
import sys; sys.path.insert(0, "..")
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import render as RD
import hand3_cad as C
import hand3_design as H

COL = dict(alu=[0.74, 0.76, 0.79], steel=[0.60, 0.62, 0.66], pacf=[0.17, 0.18, 0.20], tpu=[0.92, 0.55, 0.14], servo=[0.10, 0.22, 0.55], motor=[0.12, 0.12, 0.14],
           frame=[0.30, 0.32, 0.36], claw=[0.95, 0.72, 0.18], follower=[0.80, 0.22, 0.20], cage=[0.45, 0.47, 0.52])
S = 0.01                                                     # mm -> scene units


def colour(n):
    if n.startswith(("grip", "hub_")): return COL["alu"]
    if n.startswith("tube"): return COL["steel"]
    if n.startswith(("pad", "heel")): return COL["tpu"]
    if n.startswith("hw_servo"): return COL["servo"]
    if n.startswith("hw_motor"): return COL["motor"]
    if n.startswith("hw_spool"): return COL["alu"]
    if n.startswith("claw"): return COL["claw"]
    if n.startswith("fol"): return COL["follower"]
    if n == "cage": return COL["cage"]
    if n == "pack_frame": return COL["frame"]
    return COL["pacf"]


def prims(asm, skip=()):
    out = []; pid = 20
    for n, (m, _) in asm.items():
        if skip and n.startswith(skip): continue
        me = m.to_mesh(); V = np.asarray(me.vert_properties)[:, :3] * S; F = np.asarray(me.tri_verts).astype(np.int64)
        pid += 1; out.append((V, F, np.tile(np.asarray(colour(n), float), (len(F), 1)), pid))
    return out


def shot(asm, eye, target, name, title, fov=30.0, skip=(), W=1280, Hh=960):
    cam = RD.Camera(np.asarray(eye, float) * S, np.asarray(target, float) * S, fov)
    img = Image.fromarray(RD.render(prims(asm, skip), cam, W, Hh, ss=2, fog=(30.0, 90.0)))
    d = ImageDraw.Draw(img); d.rectangle([0, 0, W, 50], fill=(255, 255, 255)); d.text((16, 12), title, fill=(20, 20, 20), font=ImageFont.load_default(size=22))
    img.save(name); print(name)


if __name__ == "__main__":
    flat = C.assembly()
    shot(flat, [330, -300, 290], [0, 0, 55], "hand3_cad_1.png", "Flight position: four fingers on the rotor, 14 claw bars and the servo pack (CAD solids)")
    lag = [s * H.SPREAD_MAX for s in H.LAG_SIGN]
    grasp = C.assembly(lag=lag, fold=((55, 60, 50),) * 4, pull=[9.0] * 14)
    shot(grasp, [300, -330, 260], [0, 0, 80], "hand3_cad_2.png", "Spread 47 deg and curled 55 / 60 / 50 deg, claw bars pulled 9 mm")
    one = {n: v for n, v in C.assembly(fold=((0, 0, 0), (30, 45, 40), (0, 0, 0), (0, 0, 0))).items() if n.startswith(("grip1", "tube1", "ph1_", "pad1_"))}
    shot(one, [42, 142, 175], [-50, 50, 108], "hand3_cad_3.png", "One finger curled 30 / 45 / 40 deg: aluminium root grip, pulley sectors, fairleads, pads", fov=30)
    shot(flat, [150, -120, 150], [20, 5, 78], "hand3_cad_4.png", "Under the hub: cage, followers with fins (red), claw bars (yellow) in their guide sleeves", fov=30,
         skip=("ph", "pad", "hub_upper", "heel"))
