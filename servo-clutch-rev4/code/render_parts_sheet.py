"""Parts sheet: every printed part of the adopted clutch + winch set, laid out flat with labels."""
import numpy as np, trimesh, json
from PIL import Image, ImageDraw, ImageFont
from geom import Scene
from render import render, Camera

rep = json.load(open("stl_clutch/adopted_parts.json"))
names = [k for k in rep if not k.startswith("_")]
cols = {"cam": [0.95, 0.55, 0.15], "rocker": [0.2, 0.5, 0.85], "winch": [0.3, 0.7, 0.45]}
S = Scene(); pos = {}
x = 0.0
for i, nm in enumerate(names):
    tm = trimesh.load(f"stl_clutch/{nm}.stl")
    V = np.asarray(tm.vertices) / 1000.0
    V -= [V[:, 0].min(), V[:, 1].mean(), V[:, 2].min()]
    row = 0 if "STS3215" in nm else (1 if "STS3250" in nm else 2)
    col = sum(1 for n in names[:i] if (("STS3215" in n) == ("STS3215" in nm)) and (("STS3250" in n) == ("STS3250" in nm)))
    off = np.array([col * 0.085, -row * 0.07, 0.0])
    c = cols["cam"] if "cam" in nm else cols["rocker"] if "rocker" in nm else cols["winch"]
    S.add((V + off, np.asarray(tm.faces), np.tile(c, (len(tm.faces), 1))))
    pos[nm] = off + [0.0, -0.033, 0.0]
cam = Camera(np.array([0.11, -0.30, 0.30]), np.array([0.11, -0.065, 0.0]), fov_deg=34)
img = render(S.prims, cam, 1400, 900, ss=2, ground=(0.1, -0.07), fog=(5, 9))
im = Image.fromarray(img); d = ImageDraw.Draw(im)
f = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26)
g = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 17)
d.text((28, 22), "Cephalo-1 clutch set, per arm (PETG; masses at 4 walls + 40 % gyroid, est.)", font=f, fill=(30, 30, 30))
y = 62
for nm in names:
    r = rep[nm]
    d.text((28, y), f"{nm}  x{r['qty_per_arm']}   {r['extents_mm'][0]:.0f} x {r['extents_mm'][1]:.0f} x {r['extents_mm'][2]:.0f} mm   {r['mass_g']} g each",
           font=g, fill=(40, 40, 40)); y += 22
im.save("clutch_parts_sheet.png"); print("ok")
