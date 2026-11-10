"""3x2 grid comparison videos for the second-round wrists (wrists2.py).
usage: python3 make_grid.py <hit|range> <start> <end> <out.mp4>
       python3 make_grid.py <hit|range> still <frame> <out.png>
Output 1920x1080 (six 640x540 tiles), 30 fps, libx264 crf 18 veryfast yuv420p, 2x supersampling.
The hit responses are SCRIPTED from estimated stiffness / retention (not a physics simulation)."""
import sys, os, subprocess, numpy as np
from PIL import Image, ImageDraw, ImageFont
from geom import Scene, sphere, frustum, merge, rot
from render import render, Camera
from robot import arm_fk, arm_mesh, body_mesh, pod_mesh
from choreo import FLIGHT, ss
import wrists2 as W

TW, TH, COLS, ROWS = 640, 540, 3, 2
FPS = 30
Z = np.array([0, 0, 1.0])
F = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"; FB = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
f_t = ImageFont.truetype(FB, 20); f_s = ImageFont.truetype(F, 12); f_st = ImageFont.truetype(FB, 14); f_c = ImageFont.truetype(F, 20)
H_HOVER = 0.599
A0 = np.radians(22.5)
D_HIT = np.array([-np.sin(A0), np.cos(A0), 0.0])             # side hit on arm 0's fan, tangential
HIT, YANK, T_HIT = 1.5, 6.5, 10.0

# scripted response per wrist: (side-hit peak deg, ring freq Hz, damping), yank travel m
RESP = {"BallCaptive": (22, 3.0, 0.35, 0.002), "Yoke": (9, 7.0, 0.30, 0.0015), "Knuckle": (12, 6.0, 0.30, 0.002),
        "Flex": (40, 2.5, 0.12, 0.009)}


def ring(tau, A, f, z):
    if tau < 0:
        return 0.0
    w = 2 * np.pi * f
    return np.radians(A) * np.exp(-z * w * tau) * np.sin(w * np.sqrt(1 - z * z) * tau) / 0.85


def pulse(tau, dur=0.4):
    return np.sin(np.pi * tau / dur) if 0 <= tau < dur else 0.0


class Detach:
    """Ball knocked off its seat: flies out, hangs on its tether/wires, optionally reels back."""
    def __init__(self, t0, d, reel_at=None):
        self.t0, self.d, self.reel_at = t0, d / np.linalg.norm(d), reel_at

    def state(self, t, p, c0):
        tau = t - self.t0
        if tau < 0:
            return None
        def path(tau):
            peak = self.d * 0.05 + Z * 0.015
            rest = (p - Z * 0.12) - c0
            if tau < 0.15:
                return peak * ss(tau / 0.15), np.pi * ss(tau / 0.5)
            s = ss((tau - 0.15) / 0.6)
            sw = self.d * 0.035 * np.exp(-(tau - 0.15) / 1.5) * np.sin(2 * np.pi * (tau - 0.15) / 0.75) * s
            return peak * (1 - s) + rest * s + sw, np.pi * ss(tau / 0.5)
        if self.reel_at is not None and t >= self.reel_at:
            sep0, a0 = path(self.reel_at - self.t0)
            r = ss((t - self.reel_at) / 1.0)
            if r >= 1:
                return None
            sep, ang = sep0 * (1 - r), a0 * (1 - r)
        else:
            sep, ang = path(tau)
        ax = np.cross(Z, self.d); ax /= np.linalg.norm(ax)
        return sep, rot(ax, ang) @ Z, (self.reel_at is not None and t >= self.reel_at)


def hit_script(cls, t, p, c0):
    """returns defl rotvec, sep, u_force, status text"""
    name = cls.__name__
    ax = np.cross(Z, D_HIT); ax /= np.linalg.norm(ax)
    if name in RESP:
        A, f, z, yank = RESP[name]
        th = ring(t - HIT, A, f, z)
        if name == "BallCaptive" and t > HIT:          # wheels slip, then drive the ball back
            th += np.radians(14) * ss((t - HIT) / 0.15) * (1 - ss((t - HIT - 0.4) / 1.0))
        sep = Z * yank * pulse(t - YANK)
        st = "seated"
        if abs(np.degrees(th)) > 1:
            st = f"knocked {abs(np.degrees(th)):.0f}° off, recovering"
        if pulse(t - YANK) > 0.05:
            st = f"yanked: moves {yank * 1000 * pulse(t - YANK):.0f} mm, holds"
        return ax * th, sep, None, st
    if name == "BallMagnetic":
        for ev in (Detach(HIT, D_HIT, HIT + 1.8), Detach(YANK, Z + 0.3 * D_HIT, YANK + 1.8)):
            s = ev.state(t, p, c0)
            if s is not None:
                return None, s[0], s[1], ("reeling back in, magnet re-seats" if s[2] else "broke away, hanging on tether")
        return None, None, None, "seated"
    if name == "BallOpen":
        ev = Detach(0.6, Z * 0.2 + D_HIT * 0.05)
        s = ev.state(t, p, c0)
        if s is None:
            return None, None, None, "seated (for now)"
        sep, uf = s[0], s[1]
        if t > HIT:     # the hit swings the dangling pod
            sep = sep + D_HIT * 0.04 * np.exp(-(t - HIT) / 1.2) * np.sin(2 * np.pi * (t - HIT) / 0.75)
        return None, sep, uf, "fan pull alone lifted the ball out; hanging on its wires"


def arrow(a, b, col):
    d = b - a; L = np.linalg.norm(d); u = d / L
    return merge([frustum(a, b - u * 0.03, 0.006, 0.006, col, n=10), frustum(b - u * 0.03, b, 0.016, 0.0005, col, n=12)])


class Tile:
    def __init__(self, cls, video):
        self.cls, self.video = cls, video
        self.ws = [cls() for _ in range(8)]; self.w = self.ws[0]
        if video == "range":
            self.st = dict(np.load("states_arms_BallDrive.npz"))
        self.trail = []

    def scene(self, f):
        t = f / FPS; S = Scene()
        if self.video == "hit":
            body = np.array([0, 0, H_HOVER + 0.004 * np.sin(2 * np.pi * t / 1.7)])
            Q = np.tile(FLIGHT, (8, 1)); U = np.tile(Z, (8, 1)); roll = 0.0; sweep = False
        else:
            st = self.st
            body, Q, U = st["body"][f], st["q"][f], st["u"][f]
            roll, sweep = float(st["roll"][f]), bool(st["trail"][f])
        body_mesh(S, body, edf_ang=t * 40)
        errs, ncl, status = [], 0, ""
        for i in range(8):
            frames, (R, p) = arm_fk(i, Q[i], body)
            arm_mesh(S, frames, (R, p), self.w.accent)
            kw = {}
            if self.video == "hit" and i == 0:
                c0 = p + R[:, 0] * getattr(self.ws[0], "OFFSET", 0.0)
                defl, sep, uf, status = hit_script(self.cls, t, p, c0)
                kw = dict(defl=defl, sep=sep, u_force=uf)
            m, u, stalk, rref, cl = self.ws[i].build(p, R[:, 0], R[:, 2], U[i], roll_rate=roll if i == 0 else 0.0, **kw)
            S.add(m)
            off = kw.get("sep") is not None and np.linalg.norm(kw["sep"]) > 0.003 and kw.get("u_force") is not None
            pod_mesh(S, stalk, u, rref, (0.3 if off else t * 47.0) + i, 0.0, cl)
            if self.video == "hit" and i == 0:
                self.pod0 = stalk
            if sweep and i == 0:
                self.trail.append((stalk + u * 0.05, cl))
                errs = [np.degrees(np.arccos(np.clip(np.dot(u, U[0]), -1, 1)))]; ncl = int(cl)
            elif not sweep:
                errs.append(np.degrees(np.arccos(np.clip(np.dot(u, U[i]), -1, 1)))); ncl += cl
        for k, (pt, c) in enumerate(self.trail[-400:]):
            S.add(sphere(pt, 0.0035, [0.9, 0.15, 0.1] if c else self.w.accent, n=6, m=4), same=k > 0)
        if self.video == "hit":
            _, (R0, p0) = arm_fk(0, FLIGHT, body)
            seat = p0 + Z * 0.09
            if HIT - 0.45 < t < HIT + 0.1:
                s = ss((t - HIT + 0.45) / 0.45)
                S.add(arrow(seat - D_HIT * (0.22 - 0.16 * s), seat - D_HIT * (0.08 - 0.06 * s), [0.85, 0.1, 0.1]))
            if YANK - 0.3 < t < YANK + 0.35:
                S.add(arrow(seat + Z * 0.06, seat + Z * 0.16, [0.85, 0.1, 0.1]))
            cam_t = p0 + R0[:, 0] * 0.04 + Z * 0.03
            az, el, dist = A0 - np.radians(55), 0.28, 0.46
            eye = cam_t + dist * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
            cap = ("Side hit on the fan pod (1.5 s), then a hard yank straight up (6.5 s).  "
                   "Responses are scripted from estimated stiffness and retention, not simulated.")
        else:
            eye, cam_t = self.st["cam"][f], self.st["tgt"][f]
            if sweep:
                eye = cam_t + (eye - cam_t) * 0.8
            cap = str(self.st["cap"][f])
        self.status = status
        return S, errs, ncl, Camera(eye, cam_t, fov_deg=40), cap, body


def frame(tiles, f):
    imgs, info = [], []
    for tl in tiles:
        S, errs, ncl, cam, cap, body = tl.scene(f)
        imgs.append(render(S.prims, cam, TW, TH, ss=2, ground=(body[0], body[1])))
        info.append((errs, ncl, tl.status, cap))
    rows = [np.concatenate(imgs[r * COLS:(r + 1) * COLS], 1) for r in range(ROWS)]
    im = Image.fromarray(np.concatenate(rows, 0)).convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0)); do = ImageDraw.Draw(ov)
    for k in range(len(tiles)):
        x, y = (k % COLS) * TW, (k // COLS) * TH
        do.rounded_rectangle([(x + 6, y + 6), (x + TW - 6, y + 92)], 9, fill=(255, 255, 255, 205))
    im = Image.alpha_composite(im, ov).convert("RGB"); d = ImageDraw.Draw(im)
    for k, tl in enumerate(tiles):
        x, y = (k % COLS) * TW + 14, (k // COLS) * TH + 10
        d.text((x, y), tl.w.name, font=f_t, fill=tuple(int(c * 255) for c in tl.w.accent))
        d.text((x, y + 26), tl.w.sub, font=f_s, fill=(40, 40, 40))
        errs, ncl, status, cap = info[k]
        if tl.video == "hit":
            bad = any(s in status for s in ("lifted", "broke", "reeling"))
            d.text((x, y + 43), tl.w.retention, font=f_s, fill=(70, 70, 70))
            d.text((x, y + 64), status, font=f_st, fill=(200, 30, 20) if bad else (30, 120, 40))
        elif errs:
            e = max(errs)
            msg = (f"swept fan off command: {e:3.0f}°" if tl.trail and len(errs) == 1 else
                   f"fans off target: worst {e:3.0f}°   at limit {ncl}/{len(errs)}")
            d.text((x, y + 50), msg, font=f_st, fill=(200, 30, 20) if e > 1 else (30, 120, 40))
    for c in range(1, COLS):
        d.line([(c * TW, 0), (c * TW, ROWS * TH)], fill=(60, 60, 60), width=2)
    d.line([(0, TH), (COLS * TW, TH)], fill=(60, 60, 60), width=2)
    out = Image.new("RGB", (COLS * TW, ROWS * TH + 0), (25, 27, 30)); out.paste(im, (0, 0))
    d = ImageDraw.Draw(out)
    cap = info[0][3]
    d.rectangle([(0, ROWS * TH - 40), (COLS * TW, ROWS * TH)], fill=(25, 27, 30))
    tw = d.textlength(cap, font=f_c)
    d.text(((COLS * TW - tw) / 2, ROWS * TH - 31), cap, font=f_c, fill=(240, 240, 240))
    return out


def main():
    video = sys.argv[1]
    tiles = [Tile(c, video) for c in W.ALL]
    N = int(T_HIT * FPS) if video == "hit" else len(tiles[0].st["body"])
    if sys.argv[2] == "still":
        tgt = int(sys.argv[3])
        for f in range(tgt):
            for tl in tiles:
                tl.scene(f)
        frame(tiles, tgt).save(sys.argv[4]); return
    a, b, out = int(sys.argv[2]), min(int(sys.argv[3]), N), sys.argv[4]
    for f in range(a):
        for tl in tiles:
            tl.scene(f)
    tmp = out[:-4] + ".part.mp4"
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{COLS * TW}x{ROWS * TH}",
                          "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", tmp],
                         stdin=subprocess.PIPE)
    for f in range(a, b):
        p.stdin.write(np.asarray(frame(tiles, f)).tobytes())
    p.stdin.close()
    if p.wait() == 0:
        os.replace(tmp, out)


if __name__ == "__main__":
    main()
