"""Chilo-1 rev 2 walk video (octopus as the head, arm legs + style-A body legs) from chilo2_walk*.npz.  1920x1080, 30 fps.
python3 chilo2_video.py still <t> <tag>  |  chunk <i> <n> <tag>  |  join <n> <tag>      tag = '' | _assist | _assist4"""
import os, sys, json, subprocess
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
from geom import frustum, sphere, merge, Scene, torus
from render import Camera, render
import chilo2 as C, chilo2_walk as W, chilo_arm_legs as A, chilo_arm_legs_render as AR, chilo_face_dock as FD, chilo_dock_designs as DD
from chilo_build_concepts import FONT, FOOT, SERVO, HUBC

W_, H_ = 1920, 1080
LEGC = (0.95, 0.62, 0.27); SHELL = (0.80, 0.82, 0.85); STEEL = (0.72, 0.74, 0.77)
INK, DIM, GRN, ORG, RED = (28, 34, 40), (95, 105, 115), (40, 150, 90), (225, 130, 30), (200, 60, 40)
GROUPS = [("arm j0-j1 (STS3250)", lambda n: n.startswith("arm") and n[-1] in "01"),
          ("arm j2-j7 (STS3215)", lambda n: n.startswith("arm") and n[-1] not in "01"),
          ("leg yaw (STS3215)", lambda n: n[0] in "LR" and n.endswith("_q0")),
          ("leg p1-p2 (STS3250)", lambda n: n[0] in "LR" and n[-1] in "12"),
          ("leg p3-p4 (STS3215)", lambda n: n[0] in "LR" and n[-1] in "34"),
          ("spine yaw (STS3215)", lambda n: n.startswith("sy")), ("spine pitch (STS3250)", lambda n: n.startswith("sp"))]


def font(sz, bold=False):
    return ImageFont.truetype(FONT % ("-Bold" if bold else ""), sz)


class Ctx:
    def __init__(self, tag):
        self.tag = tag
        self.m = C.build(); self.d = mujoco.MjData(self.m)
        z = np.load(f"chilo2_walk{tag}.npz")
        self.q, self.t, self.load, self.down, self.itot, self.com = z["qpos"], z["t"], z["load"], z["down"], z["itot"], z["com"]
        self.names = [str(n) for n in z["names"]]
        self.met = json.load(open(f"chilo2_walk{tag}.json"))
        self.gix = [[k for k, n in enumerate(self.names) if f(n)] for _, f in GROUPS]
        pad = np.pad(self.com, ((20, 20), (0, 0)), mode="edge"); k = np.ones(41) / 41
        self.cs = np.stack([np.convolve(pad[:, j], k, mode="valid") for j in range(3)], 1)
        # octopus body + hands in the head frame (head origin at SEG_X[0], 0, HC at t = 0)
        h0 = np.array([C.SEG_X[0], 0, C.HC]); self.static = []
        tmp = Scene(); AR.octo_body(tmp, C.PLATE)
        for V, F, Cc, pid in tmp.prims: self.static.append((V - h0, F, Cc))
        tmp = Scene()
        for i in C.ARM_HANDS:
            root, R0 = A.arm_frame(i, C.PLATE)
            q = np.radians([-30, 0, -45, 0, -35, 0, -20, 0] if i in (3, 4) else [-20, 0, -30, 0, -40, 0, -30, 0])
            P, Ax, spig, tip = A.fk(q, root, R0); AR.draw_arm(tmp, root, P, spig, tip, AR.ARM, False)
        for V, F, Cc, pid in tmp.prims: self.static.append((V - h0, F, Cc))
        self.static.append(tuple(list(torus(np.array([C.FACE_X + 0.004, 0, C.HC]) - h0, np.array([1.0, 0, 0]), 0.109, 0.009, HUBC, n=48, m=10))))
        self.legs = [f"{'L' if s > 0 else 'R'}{seg}" for seg in range(1, 6) for s in (1, -1)]
        self.N = len(self.t)

    def scene(self, f):
        m, d = self.m, self.d
        d.qpos[:] = self.q[f]; mujoco.mj_kinematics(m, d)
        sc = Scene()
        hb = m.body("seg0").id; Rh = d.xmat[hb].reshape(3, 3); ph = d.xpos[hb]
        for V, F, Cc in self.static:
            sc.add((V @ Rh.T + ph, F, Cc))
        for i in range(6):
            b = m.body(f"seg{i}").id; R = d.xmat[b].reshape(3, 3); p = d.xpos[b]; x = R[:, 0]
            FD.seg(sc, p - x * C.SEG_L[i] / 2, p + x * C.SEG_L[i] / 2, C.SEG_R[i])
            if i: sc.add(sphere(p + x * (C.PIVOT_X[i] - C.SEG_X[i]), 0.04, STEEL, n=12, m=8))
        for n in self.legs:
            pts = [d.xpos[m.body(f"{n}_hip").id]] + [d.xpos[m.body(f"{n}_l{k}").id] for k in range(1, 5)] + [d.site_xpos[m.site(f"{n}_ft").id]]
            parts = [frustum(a, b, 0.017, 0.016, LEGC, n=10) for a, b in zip(pts[:-1], pts[1:])]
            for k, p in enumerate(pts[:-1]):
                parts.append(sphere(p, 0.026 if k else 0.03, SERVO, n=10, m=6))
                if 0 < k < 4: parts.append(sphere(0.5 * (p + pts[k + 1]), 0.02, SERVO, n=8, m=6))
            parts.append(sphere(pts[-1], 0.016, FOOT, n=10, m=6))
            sc.add(merge(parts))
        for i in C.ARM_LEGS:
            root = d.xpos[m.body(f"arm{i}_root").id]
            P = np.array([d.xpos[m.body(f"arm{i}_j{k}").id] for k in range(8)])
            x7 = d.xmat[m.body(f"arm{i}_j7").id].reshape(3, 3)[:, 0]
            spig = P[7] + x7 * (A.SPIGOT - A.S[7]); tip = d.site_xpos[m.site(f"arm{i}_ft").id]
            AR.draw_arm(sc, root, P, spig, tip, AR.LEGARM, True)
        return sc

    def frame(self, f):
        t = float(self.t[f]); u = f / (self.N - 1)
        tgt = self.cs[f] * [1, 1, 0] + [0.10, 0, 0.26]
        az = np.radians(-52 + 30 * u); el = np.radians(17)
        eye = tgt + 2.9 * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
        img = Image.fromarray(render(self.scene(f).prims, Camera(eye, tgt, 34), W_, H_, ss=2, ground=(tgt[0], 0), fog=(8, 18)))
        dr = ImageDraw.Draw(img)
        dr.text((40, 26), "CHILO-1 + CEPHALO-1", font=font(50, True), fill=INK)
        dr.text((40, 86), "Octopus docked as the head: arms 1/6 + 0/7 are the front legs, style-A body legs behind", font=font(24), fill=DIM)
        dr.text((40, 124), W.phase_name(t), font=font(32, True), fill=ORG)
        o = self.met; g = o["gait"]
        lines = [f"t = {t:4.1f} s     walked {self.com[f, 0] - self.com[0, 0]:+.2f} m",
                 f"{o['mass_kg']:.1f} kg total (octopus 12.8, centipede {o['mass_kg'] - 12.81:.1f}, est.)",
                 f"cycle {g['T']} s, stride {g['stride_mm']:.0f} mm, {g['speed_cm_s']} cm/s",
                 f"free hands as props: {o['prop_lift_N']:.0f} N lift (est.)" if o["prop_lift_N"] else "no prop lift",
                 f"feet on the floor: {int(self.down[f].sum())} of 14 (arm feet {int(self.down[f][:4].sum())} of 4)",
                 f"arm joints held to <= 70 % of their range"]
        for i, s in enumerate(lines):
            dr.text((40, 176 + 34 * i), s, font=font(23), fill=INK)
        yb = 400
        dr.text((40, yb), "SERVO LOAD NOW (worst in group, x stall)", font=font(20, True), fill=INK)
        for r, ((nm, _), ix) in enumerate(zip(GROUPS, self.gix)):
            v = float(self.load[f][ix].max()); yy = yb + 44 + 32 * r
            dr.text((40, yy), nm, font=font(19), fill=INK)
            dr.rectangle([300, yy + 4, 500, yy + 22], outline=DIM, width=2)
            col = GRN if v < 1 / 3 else (ORG if v < 0.8 else RED)
            dr.rectangle([300, yy + 4, 300 + int(200 * min(v, 1)), yy + 22], fill=col)
            dr.line([300 + 200 / 3, yy + 2, 300 + 200 / 3, yy + 24], fill=INK, width=1)
            dr.text((512, yy), f"{v:.2f}", font=font(19), fill=INK)
        dr.text((300, yb + 40 + 32 * 7), "| = rated (1/3 stall)", font=font(16), fill=DIM)
        yc = yb + 40 + 32 * 8 + 10
        dr.text((40, yc), f"12 V servo bus: {float(self.itot[f]):.0f} A  ({float(self.itot[f]) * 12:.0f} W, est.)", font=font(23, True), fill=INK)
        dr.text((40, H_ - 56), "MuJoCo, 92 stall-limited STS servos, open-loop gait + arm-foot load balancer.  Masses, hand thrust and currents are estimates; nothing printed yet.",
                font=font(19), fill=DIM)
        return np.asarray(img)


def enc(path):
    return subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W_}x{H_}", "-r", "30", "-i", "-",
                             "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", path], stdin=subprocess.PIPE)


if __name__ == "__main__":
    cmd = sys.argv[1]
    if cmd == "still":
        tag = sys.argv[3] if len(sys.argv) > 3 else ""
        cx = Ctx(tag); f = min(int(round(float(sys.argv[2]) * 30)), cx.N - 1)
        Image.fromarray(cx.frame(f)).save(f"chilo2_still_{f:04d}.png"); print(f"chilo2_still_{f:04d}.png")
    elif cmd == "chunk":
        i, n = int(sys.argv[2]), int(sys.argv[3]); tag = sys.argv[4] if len(sys.argv) > 4 else ""
        cx = Ctx(tag); fr = np.array_split(np.arange(cx.N), n)[i]
        p = enc(f"c2_{i}.part.mp4")
        for f in fr: p.stdin.write(cx.frame(f).tobytes())
        p.stdin.close()
        if p.wait() == 0: os.replace(f"c2_{i}.part.mp4", f"c2_{i}.mp4")
    elif cmd == "join":
        n = int(sys.argv[2]); tag = sys.argv[3] if len(sys.argv) > 3 else ""
        open("c2_list.txt", "w").write("".join(f"file 'c2_{i}.mp4'\n" for i in range(n)))
        out = f"chilo1_octopus_head_walk{tag}.mp4"
        subprocess.check_call(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "c2_list.txt", "-c", "copy", out]); print(out)
