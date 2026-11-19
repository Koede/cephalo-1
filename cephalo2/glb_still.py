import sys, numpy as np, trimesh
from render import Camera, render
from PIL import Image
prims = []
for k, pose in enumerate(("stand", "flight")):
    s = trimesh.load(f"cephalo2_{pose}.glb")
    for n, g in s.dump(concatenate=False) if False else [(None, g) for g in s.dump()]:
        c = np.array(g.visual.material.baseColorFactor[:3], float)
        c = c / 255 if c.max() > 1 else c
        V = np.asarray(g.vertices) + [0, 0, 0.30]
        prims.append(((V, np.asarray(g.faces), np.tile(c, (len(g.faces), 1)), 11 + len(prims)), pose))
for pose, eye in (("stand", (0.75, -0.9, 0.75)), ("flight", (0.75, -0.9, 0.95))):
    P = [p for p, ps in prims if ps == pose]
    img = render(P, Camera(eye, (0, 0, 0.36), 40), 960, 720, ss=1)
    Image.fromarray(img).save(f"c2_{pose}.png")
