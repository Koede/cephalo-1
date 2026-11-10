"""2x2 video of the MuJoCo hit test (0.2 kg object at 4 m/s into the prop):
columns = knuckle | magnetic ball (Fc 30 N);  rows = rev 3 arm (STS3032 at j5) | j5 upgraded to STS3215.
usage: python3 make_hitsim_video.py <start> <end> <out.mp4>   |   still <frame> <out.png>
1920x1080, 30 fps; the impact window is shown at 1/5 speed."""
import sys, os, subprocess, numpy as np
from PIL import Image, ImageDraw, ImageFont
import sim_wrist as S
from geom import Scene, sphere, rot
from render import render, Camera
from robot import arm_fk, arm_mesh, body_mesh, pod_mesh
import wrists2 as W

TW, TH = 960, 540
F = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"; FB = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
f_t = ImageFont.truetype(FB, 22); f_s = ImageFont.truetype(F, 15); f_c = ImageFont.truetype(F, 20); f_b = ImageFont.truetype(FB, 17)
V_HIT = 4.0
T_HIT = 0.5
BODY = np.array([0, 0, S.BODY_Z])


def run(var, upgrade):
    S.STALL = [4.9] * 3 + [2.9] * 2 + ([2.9] if upgrade else [0.44]) + [0.44] * 3
    S.SERVO_M = [0.076] * 3 + [0.055] * 2 + ([0.055] if upgrade else [0.020]) + [0.020] * 3
    s = S.Sim(var, Fc=30.0, imp_v=V_HIT, t_hit=T_HIT).run(3.2, rec_dt=0.001)
    return s.log


# output timeline: real time, then the impact at 1/5 speed, then real time
def timeline():
    ts, t = [], 0.30
    while t < 3.2:
        ts.append(t)
        t += (1 / 30) * (0.2 if 0.46 <= t < 0.80 else 1.0)
    return ts


class FixedKnuckle(W.Knuckle):
    def set(self, roll, pitch):
        self._rp = (roll, pitch)

    def track(self, Rt, u_cmd, dt):
        self.roll, self.pitch = self._rp
        Rr = Rt @ rot([0, 0, 1], self.roll)
        return Rr @ np.array([np.sin(self.pitch), 0, np.cos(self.pitch)]), Rr, False


TILES = [("knuckle", False), ("magnet", False), ("knuckle", True), ("magnet", True)]


def scene(var, snap, loads, t):
    Sc = Scene()
    body_mesh(Sc, BODY, edf_ang=t * 40)
    for i in range(8):
        q = snap["q"] if i == 0 else S.FLIGHT
        frames, (R, p) = arm_fk(i, q, BODY)
        acc = W.Knuckle.accent if var == "knuckle" else W.BallMagnetic.accent
        arm_mesh(Sc, frames, (R, p), acc)
        on = loads["fan_on"] > 0.5 if i == 0 else True
        if var == "knuckle":
            w = FixedKnuckle()
            w.set(snap["roll"] if i == 0 else 0.0, (snap["h1"] + snap["h2"]) if i == 0 else 0.0)
            m, u, stalk, rref, cl = w.build(p, R[:, 0], R[:, 2], R[:, 0])
        else:
            w = W.BallMagnetic()
            if i == 0:
                c0 = p + R[:, 0] * w.OFFSET
                m, u, stalk, rref, cl = w.build(p, R[:, 0], R[:, 2], R[:, 0], sep=snap["ball_p"] - c0, u_force=snap["ball_R"][:, 0])
            else:
                m, u, stalk, rref, cl = w.build(p, R[:, 0], R[:, 2], R[:, 0])
        Sc.add(m)
        pod_mesh(Sc, stalk, u, rref, (t * 47.0 if on else 0.4) + i, 0.0, False)
        if i == 0:
            tipp = p
    Sc.add(sphere(snap["imp"], 0.02, [0.85, 0.15, 0.1], n=14, m=10))
    return Sc, tipp


def frame(logs, ts, k):
    t = ts[k]
    imgs, info = [], []
    for (var, up), log in zip(TILES, logs):
        idx = min(int(round(t / 0.001)), len(log) - 1)
        tt, loads, snap = log[idx]
        Sc, tipp = scene(var, snap, loads, t)
        _, (R0, p0) = arm_fk(0, S.FLIGHT, BODY)
        tgt = 0.55 * p0 + 0.45 * (BODY + [0, 0, 0.08]) + np.array([0, 0, 0.06])
        az, el, dist = np.radians(22.5 + 15), 0.14, 1.2
        eye = tgt + dist * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
        imgs.append(render(Sc.prims, Camera(eye, tgt, fov_deg=40), TW, TH, ss=2, ground=(0, 0)))
        pk = max(l["impact_N"] for _, l, _ in log[:idx + 1])
        dev = np.degrees(np.abs(snap["q"] - S.FLIGHT))
        info.append((var, up, loads, pk, dev))
    im = Image.fromarray(np.concatenate([np.concatenate(imgs[:2], 1), np.concatenate(imgs[2:], 1)], 0)).convert("RGBA")
    ov = Image.new("RGBA", im.size, (0, 0, 0, 0)); do = ImageDraw.Draw(ov)
    for j in range(4):
        x, y = (j % 2) * TW, (j // 2) * TH
        do.rounded_rectangle([(x + 8, y + 8), (x + TW - 8, y + 100)], 10, fill=(255, 255, 255, 210))
    im = Image.alpha_composite(im, ov).convert("RGB"); d = ImageDraw.Draw(im)
    for j, (var, up, loads, pk, dev) in enumerate(info):
        x, y = (j % 2) * TW + 18, (j // 2) * TH + 14
        acc = W.Knuckle.accent if var == "knuckle" else W.BallMagnetic.accent
        name = ("GEARED KNUCKLE + ROLL" if var == "knuckle" else "MAGNETIC BALL DRIVE (30 N)")
        d.text((x, y), name, font=f_t, fill=tuple(int(c * 255) for c in acc))
        d.text((x, y + 28), "arm as built: j5 = STS3032 (0.44 N·m)" if not up else "j5 upgraded to STS3215 (2.9 N·m, +35 g)",
               font=f_s, fill=(40, 40, 40))
        folded = dev[5] > 60 and t > 1.5
        msg = (f"impact peak {pk:5.0f} N   fan tilt {loads['tilt_deg']:4.0f}°   j5 {dev[5]:3.0f}°  j7 {dev[7]:3.0f}° off"
               + ("" if var == "knuckle" else f"   gap {loads['gap_mm']:.0f} mm"))
        d.text((x, y + 50), msg, font=f_s, fill=(40, 40, 40))
        status = ("FOLDED: j5 can't lift the arm back (0.5 N·m needed > 0.44 stall)" if folded else
                  ("fan cut (pod > 25° off vertical)" if loads["fan_on"] < 0.5 else "fan on"))
        d.text((x, y + 70), status, font=f_b, fill=(200, 30, 20) if (folded or loads["fan_on"] < 0.5) else (30, 120, 40))
    d.line([(TW, 0), (TW, 2 * TH)], fill=(60, 60, 60), width=2); d.line([(0, TH), (2 * TW, TH)], fill=(60, 60, 60), width=2)
    slow = 0.46 <= t < 0.80
    cap = (f"MuJoCo: 0.2 kg object at {V_HIT:.0f} m/s into the prop    t = {t:4.2f} s" + ("    ▶ 1/5 speed" if slow else ""))
    d.rectangle([(0, 2 * TH - 40), (2 * TW, 2 * TH)], fill=(25, 27, 30))
    tw = d.textlength(cap, font=f_c)
    d.text(((2 * TW - tw) / 2, 2 * TH - 31), cap, font=f_c, fill=(255, 220, 120) if slow else (240, 240, 240))
    return im


def main():
    logs = [run(v, u) for v, u in TILES]
    ts = timeline()
    if sys.argv[1] == "still":
        frame(logs, ts, int(sys.argv[2])).save(sys.argv[3]); return
    a, b, out = int(sys.argv[1]), min(int(sys.argv[2]), len(ts)), sys.argv[3]
    tmp = out[:-4] + ".part.mp4"
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{2 * TW}x{2 * TH}",
                          "-r", "30", "-i", "-", "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", tmp],
                         stdin=subprocess.PIPE)
    for k in range(a, b):
        p.stdin.write(np.asarray(frame(logs, ts, k)).tobytes())
    p.stdin.close()
    if p.wait() == 0:
        os.replace(tmp, out)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "count":
        print(len(timeline()))
    else:
        main()
