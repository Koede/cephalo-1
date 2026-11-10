"""Hero render of the STS3215 detent-cam clutch assembly (mm -> m for the renderer)."""
import numpy as np, trimesh
from PIL import Image, ImageDraw, ImageFont
from geom import Scene, frustum, box, tube_path, rot
from render import render, Camera
import clutch_parts as CP

S = Scene(); s = 0.001
disk, p = CP.cam_disk(2.9); rk = CP.rocker(p)
def add_tm(man, R, t, col):
    tm = CP.to_trimesh(man); V = (np.asarray(tm.vertices) @ R.T + t) * s
    S.add((V + [0, 0, 0.05], np.asarray(tm.faces), np.tile(col, (len(tm.faces), 1))))
I = np.eye(3)
add_tm(disk, I, [0, 0, 0], [0.95, 0.55, 0.15])
lr, li = p["l_roller"], p["l_idler"]
rx = 14.0 + 6.5
Rr = rot([0, 0, 1], np.pi / 2)                      # rocker long axis along +y, roller end at the cam
add_tm(rk, Rr, [rx, lr, 7.0], [0.2, 0.5, 0.85])
Z0 = 0.05
def cyl(c, r, h, col, z=0):
    S.add(frustum(np.array([c[0], c[1], z]) * s + [0, 0, Z0], np.array([c[0], c[1], z + h]) * s + [0, 0, Z0], r * s, r * s, col, n=40))
cyl((rx, 0), 6.5, 5, [0.75, 0.77, 0.8], 0.5)         # 624ZZ roller, in the cam plane
cyl((rx, 0), 2.0, 16, [0.3, 0.3, 0.32], 0.0)         # M4 roller axle
cyl((rx, lr + li), 5.0, 4, [0.75, 0.77, 0.8], 15.2)  # 623ZZ idler on top of the rocker
cyl((rx, lr), 2.8, 22, [0.3, 0.3, 0.32], -6)         # pivot bolt into the link
cyl((0, 0), 10.0, 3, [0.25, 0.25, 0.28], -3)         # servo horn
S.add(box(np.array([0, -10, -21.5]) * s + [0, 0, Z0], np.eye(3), np.array([24.7, 45.2, 35.0]) * s, [0.2, 0.21, 0.23]))  # STS3215
# tension line: comes down the arm (along -x), wraps a quarter turn under the idler, continues along -y... drawn simple
idl = np.array([rx + 5.2, lr + li, 5.0])
pts = np.array([[rx + 5.2, lr + li + 60, 5.0], idl + [0, 3, 0], idl + [0, 0, 0], idl + [0.5, -3, 0], [rx + 60, lr + li - 5.2 - 0.1, 5.0]])
zl = 17.2
pts = np.array([[rx + 5.2, lr + li + 60, zl], [rx + 5.2, lr + li, zl], [rx + 60, lr + li - 5.2, zl]])
S.add(tube_path(pts * s + [0, 0, Z0], 0.0008, [0.9, 0.1, 0.1], n=8))
cam = Camera(np.array([0.09, -0.07, 0.16]), np.array([0.012, 0.02, Z0]), fov_deg=38)
img = render(S.prims, cam, 1280, 900, ss=2, ground=(0, 0), fog=(3, 9))
im = Image.fromarray(img); d = ImageDraw.Draw(im)
f = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 26); g = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 19)
d.text((30, 24), "Detent-cam clutch for an STS3215 joint (j3-j5, j7)", font=f, fill=(30, 30, 30))
lines = ["orange: cam disk on the servo horn (V-cam, 10 mm/rad rise, +-40 deg, then stop)",
         f"blue: rocker on M3 pivot, 624ZZ roller at {lr:.0f} mm, 623ZZ idler at {li:.0f} mm",
         "red: the arm's tension line, wrapped 90 deg under the idler, tensioned by the root winch",
         f"at 60 N line: roller force {p['P']:.0f} N, slips at {p['M']:.1f} N m (0.9 x stall), springs back"]
for i, l in enumerate(lines):
    d.text((30, 64 + 26 * i), l, font=g, fill=(40, 40, 40))
im.save("clutch_hero.png")
print("ok")
