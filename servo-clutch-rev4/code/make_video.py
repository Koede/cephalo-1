"""Render side-by-side comparison videos (left Ball Drive, right Omni-Wrist).
usage: python3 make_video.py <walk|arms> [start end out.mp4]   (frames [start, end))
       python3 make_video.py <walk|arms> still <frame> out.png
Conventions: 1280x720, 30 fps, libx264 crf 18 preset veryfast yuv420p, 2x supersampling."""
import sys, subprocess, numpy as np
from PIL import Image, ImageDraw, ImageFont
from geom import Scene, sphere
from render import render, Camera
from robot import arm_fk, arm_mesh, body_mesh, pod_mesh
from wrists import BallDrive, OmniWrist

W2, H = 640, 720
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
FONTB = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
f_title = ImageFont.truetype(FONTB, 24); f_sub = ImageFont.truetype(FONT, 14); f_cap = ImageFont.truetype(FONT, 17)
f_stat = ImageFont.truetype(FONTB, 16)
Z = np.array([0, 0, 1.0])


class Variant:
    def __init__(self, cls, video):
        self.cls = cls
        self.w = cls()
        self.st = dict(np.load(f"states_{video}_{cls.__name__}.npz"))
        self.trail = []
        self.sweep = False

    def scene(self, f):
        st = self.st; S = Scene()
        body = st["body"][f]; t = f / 30
        body_mesh(S, body, edf_ang=st["edf"][f] * t * 40)
        errs, nclamp = [], 0
        self.sweep = False
        roll = float(st["roll"][f]) if "roll" in st else 0.0
        for i in range(8):
            frames, (R, p) = arm_fk(i, st["q"][f][i], body)
            arm_mesh(S, frames, (R, p), self.w.accent)
            m, u, stalk, rref, clamped = self.w.build(p, R[:, 0], R[:, 2], st["u"][f][i], roll_rate=roll if i == 0 else 0.0) \
                if self.cls is BallDrive else self.w.build(p, R[:, 0], R[:, 2], st["u"][f][i])
            S.add(m)
            prop_on = bool(st["prop"][f][i])
            ang = (t * 47.0 if prop_on else 0.0) + i
            pod_mesh(S, stalk, u, rref, ang, st["fold"][f][i], clamped)
            sweep = bool(st["trail"][f]) if "trail" in st else False
            if sweep and i == 0:
                self.trail.append((stalk + u * 0.05, clamped))
                errs = [np.degrees(np.arccos(np.clip(np.dot(u, st["u"][f][0]), -1, 1)))]; nclamp = int(clamped); self.sweep = True
            elif sweep:
                pass
            elif prop_on and st["flight"][f]:
                errs.append(np.degrees(np.arccos(np.clip(np.dot(u, st["u"][f][i]), -1, 1))))
                nclamp += clamped
        for k, (pt, c) in enumerate(self.trail[-400:]):
            S.add(sphere(pt, 0.0035, [0.9, 0.15, 0.1] if c else self.w.accent, n=6, m=4), same=k > 0)
        return S, errs, nclamp


def frame(vs, f):
    halves, stats = [], []
    for v in vs:
        S, errs, nc = v.scene(f)
        st = v.st
        cam = Camera(st["cam"][f], st["tgt"][f], fov_deg=38)
        img = render(S.prims, cam, W2, H, ss=2, ground=(st["body"][f][0], 0))
        halves.append(img); stats.append((errs, nc))
    im = Image.fromarray(np.concatenate(halves, 1)).convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0)); do = ImageDraw.Draw(ov)
    for k in range(2):
        do.rounded_rectangle([(k * W2 + 8, 8), (k * W2 + W2 - 8, 96)], 10, fill=(255, 255, 255, 200))
    im = Image.alpha_composite(im, ov).convert("RGB")
    d = ImageDraw.Draw(im)
    d.line([(W2, 0), (W2, H)], fill=(60, 60, 60), width=2)
    for k, v in enumerate(vs):
        x0 = k * W2 + 18
        d.text((x0, 14), v.w.name, font=f_title, fill=tuple(int(c * 255) for c in v.w.accent))
        d.text((x0, 44), v.w.sub, font=f_sub, fill=(40, 40, 40))
        errs, nc = stats[k]
        if errs:
            e = max(errs)
            col = (200, 30, 20) if e > 1 else (30, 120, 40)
            msg = (f"swept fan off command: {e:3.0f}°   {'AT LIMIT' if nc else 'reaching'}" if v.sweep else
                   f"fans off vertical: worst {e:3.0f}°   wrists at limit: {nc}/{len(errs)}")
            d.text((x0, 68), msg, font=f_stat, fill=col)
    cap = str(vs[0].st["cap"][f])
    tw = d.textlength(cap, font=f_cap)
    d.rectangle([(0, H - 40), (2 * W2, H)], fill=(25, 27, 30))
    d.text(((2 * W2 - tw) / 2, H - 31), cap, font=f_cap, fill=(240, 240, 240))
    return im


def main():
    video = sys.argv[1]
    vs = [Variant(BallDrive, video), Variant(OmniWrist, video)]
    N = len(vs[0].st["body"])
    if sys.argv[2] == "still":
        target = int(sys.argv[3])
        for f in range(target):           # step wrist state / trail up to the frame (no render)
            for v in vs:
                v.scene(f)
        frame(vs, target).save(sys.argv[4]); return
    a, b, out = int(sys.argv[2]), min(int(sys.argv[3]), N), sys.argv[4]
    for f in range(a):
        for v in vs:
            v.scene(f)
    tmp = out[:-4] + ".part.mp4"
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{2 * W2}x{H}",
                          "-r", "30", "-i", "-", "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", tmp],
                         stdin=subprocess.PIPE)
    for f in range(a, b):
        p.stdin.write(np.asarray(frame(vs, f)).tobytes())
    p.stdin.close()
    if p.wait() == 0:
        import os; os.replace(tmp, out)


if __name__ == "__main__":
    main()
