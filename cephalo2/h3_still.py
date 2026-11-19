"""Close-up stills of hand3 on the robot arm tip: flight (fingers = rotor) and grasp."""
import os, sys, numpy as np
os.environ.update(CEPH_PROP="xl", CEPH_PACKS="2", CEPH_MONITOR="0")
from PIL import Image
import cephalo4 as C, xlpod4; xlpod4.install()
import hand3_draw as H3
from render import Camera, render
from geom import Scene
S = Scene()
Rt = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]]).T @ np.eye(3)      # pod axis = world z
Rt = np.stack([[0, 0, 1], [1, 0, 0], [0, 1, 0]], 1)
H3.draw(S, np.array([0, 0, 0.5]), Rt, 0.3, fan_on=True)
lag, fold = H3.grasp_pose()
H3.draw(S, np.array([0.45, 0, 0.5]), Rt, 0.0, lag, fold)
cam = Camera(np.array([0.22, -0.75, 0.95]), np.array([0.22, 0, 0.56]), fov_deg=36)
Image.fromarray(render(S.prims, cam, 1600, 900, ss=2, ground=(0, 0), fog=(4.5, 13.0))).save("h3_still.png")
