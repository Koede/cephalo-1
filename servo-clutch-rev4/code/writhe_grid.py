"""3x3 grid video: one tentacle per joint type, all given the same writhing command.
usage: python3 writhe_grid.py still <frame> out.png
       python3 writhe_grid.py <start> <end> out.mp4        (frames [start, end))
Kinematic animation, not physics.  Red dots = commanded backbone shape.  HUD: tip error vs command,
joints at limit, self-collision (non-adjacent parts closer than their radii).
"""
import sys, os, subprocess, numpy as np
from PIL import Image, ImageDraw, ImageFont
from geom import Scene, sphere, frustum
from render import render, Camera
import importlib
from tentacles import L_ARM, Tr, Rt, H, Y, Z, GREY, WHITE
VARIANTS = importlib.import_module(os.environ.get('GRID_MODULE', 'tentacles')).VARIANTS

FPS, T = 30, 18.0
N = int(T * FPS)
CW, CH, BAR = 640, 360, 56
SF = np.linspace(0, L_ARM, 480, endpoint=False) + L_ARM / 960
DS = SF[1] - SF[0]
BASE = H(p=[0, 0, 0.62])
FONT = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"; FONTB = "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"
f_t = ImageFont.truetype(FONTB, 17); f_s = ImageFont.truetype(FONT, 12); f_h = ImageFont.truetype(FONTB, 13)
f_c = ImageFont.truetype(FONT, 20)

#            t     K    A    f    v    tw   w    bias
KEYS = np.array([
    [0.0,  0.0, 0.8, 1.0, 0.4, 0.0, 0.3, 1.0],
    [1.5,  3.0, 0.8, 1.0, 0.4, 0.0, 0.3, 1.0],
    [5.0,  4.5, 0.9, 1.3, 0.5, 0.3, 0.5, 1.0],
    [8.0,  7.5, 0.15, 0.5, 0.2, 0.0, 0.2, 2.5],
    [10.5, 8.5, 0.15, 0.5, 0.2, 0.0, 0.6, 2.5],
    [13.5, 6.0, 0.3, 0.8, 0.3, 1.6, 0.9, 1.2],
    [16.0, 4.0, 1.0, 1.6, 0.9, 0.2, 0.5, 1.0],
    [18.0, 0.5, 0.8, 1.0, 0.4, 0.0, 0.3, 1.0]])
CAPS = [(0, "Gentle travelling wave"), (4.0, "Bigger wave, bending plane turning"), (7.0, "Tip coil: curl the last third tightly"),
        (11.5, "Corkscrew: bending plane twists along the arm"), (15.0, "Whip: fast short wave"), (17.0, "Settle")]


def params(t):
    i = np.searchsorted(KEYS[:, 0], t, side="right") - 1
    i = min(max(i, 0), len(KEYS) - 2)
    a, b = KEYS[i], KEYS[i + 1]
    u = np.clip((t - a[0]) / (b[0] - a[0]), 0, 1); u = 0.5 - 0.5 * np.cos(np.pi * u)
    return a[1:] + (b[1:] - a[1:]) * u


def command(f):
    """kappa_pitch, kappa_yaw (rad/m) along the arm at frame f (phases integrated from t=0, so chunks agree)."""
    ts = np.arange(f + 1) / FPS
    P = np.array([params(t) for t in ts])
    wave_ph = np.sum(P[:-1, 3]) / FPS if f else 0.0
    plane_ph = np.sum(P[:-1, 5]) / FPS if f else 0.0
    K, A, fr, v, tw, w, bias = P[-1]
    x = SF / L_ARM
    b = (bias + 1) * x ** bias
    k = K / L_ARM * b * (1 + A * np.sin(2 * np.pi * (fr * x - wave_ph))) / (1 + 0.0)
    psi = plane_ph + 2 * np.pi * tw * x
    return k * np.cos(psi), k * np.sin(psi), ts[-1]


def ideal(kp, ky):
    M = BASE.copy(); pts = [M[:3, 3].copy()]
    for a, b in zip(kp, ky):
        M = M @ Rt(Y, a * DS) @ Rt(Z, b * DS) @ Tr(DS)
        pts.append(M[:3, 3].copy())
    return np.array(pts)


VS = [V(SF) for V in VARIANTS]
for _v in VS:
    if hasattr(_v, 'summary'):
        _v.sub = _v.summary()
SAMP = np.linspace(0.02, L_ARM - 0.005, 24)


def cam_at(t):
    az = np.radians(-72 + 30 * t / T); el = np.radians(17)
    tgt = np.array([0.16, 0.0, 0.67])
    return Camera(tgt + 0.80 * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)]), tgt, fov_deg=40)


def frame(f):
    kp, ky, t = command(f)
    ref = ideal(kp, ky)
    cam = cam_at(t)
    img = Image.new("RGB", (3 * CW, 3 * CH + BAR), (25, 27, 30))
    hud = []
    for vi, var in enumerate(VS):
        q, at = var.map(kp, ky)
        frames, tip = var.fk(q, BASE)
        S = Scene()
        S.add(frustum([-0.03, 0, 0.62], [-0.004, 0, 0.62], 0.075, 0.075, GREY, n=36))      # mantle wall stub
        for m in var.meshes(frames):
            S.add(m)
        for k, p in enumerate(ref[::20]):
            S.add(sphere(p, 0.0028, [0.92, 0.12, 0.10], n=6, m=4), same=k > 0)
        im = render(S.prims, cam, CW, CH, ss=2, ground=(0.2, 0), shadow=False)
        err = np.linalg.norm(tip[:3, 3] - ref[-1]) * 1000
        k_ = np.clip(np.searchsorted(var.edges, SAMP, side="right") - 1, 0, var.n)
        Fm = np.array(frames + [tip])[k_]
        act = Fm[:, :3, 3] + Fm[:, :3, 0] * (SAMP - var.edges[k_])[:, None]
        shape = np.mean(np.linalg.norm(act - ref[np.round(SAMP / DS).astype(int)], axis=1)) * 1000
        clear = var.self_hit(frames)
        img.paste(Image.fromarray(im), ((vi % 3) * CW, (vi // 3) * CH))
        hud.append((var, shape, err, int(at.sum()), var.n, clear))
    ov = Image.new("RGBA", img.size, (0, 0, 0, 0)); do = ImageDraw.Draw(ov)
    for vi in range(9):
        x0, y0 = (vi % 3) * CW, (vi // 3) * CH
        do.rounded_rectangle([(x0 + 6, y0 + 6), (x0 + CW - 6, y0 + 62)], 8, fill=(255, 255, 255, 205))
    img = Image.alpha_composite(img.convert("RGBA"), ov).convert("RGB")
    d = ImageDraw.Draw(img)
    for vi, (var, shape, err, nat, n, clear) in enumerate(hud):
        x0, y0 = (vi % 3) * CW + 14, (vi // 3) * CH + 10
        d.text((x0, y0), var.name, font=f_t, fill=(30, 30, 30))
        d.text((x0, y0 + 21), var.sub, font=f_s, fill=(70, 70, 70))
        col = (30, 130, 50) if shape < 10 else ((200, 120, 0) if shape < 30 else (200, 30, 20))
        msg = f"shape off {shape:3.0f} mm avg, tip {err:3.0f} mm   at limit {nat}/{n}"
        d.text((x0, y0 + 37), msg, font=f_h, fill=col)
        if clear < 0:
            d.text((x0 + 500, y0 + 37), "SELF-HIT", font=f_h, fill=(200, 30, 20))
    for vi in range(1, 3):
        d.line([(vi * CW, 0), (vi * CW, 3 * CH)], fill=(40, 40, 40), width=2)
        d.line([(0, vi * CH), (3 * CW, vi * CH)], fill=(40, 40, 40), width=2)
    cap = [c for tt, c in CAPS if t >= tt][-1]
    cap = f"{cap}   ·   same command to all nine, red dots = commanded shape   ·   kinematic, limits estimated   ·   t {t:4.1f} s"
    tw_ = d.textlength(cap, font=f_c)
    d.text(((3 * CW - tw_) / 2, 3 * CH + 16), cap, font=f_c, fill=(235, 235, 235))
    return img, hud


def main():
    if sys.argv[1] == "still":
        frame(int(sys.argv[2]))[0].save(sys.argv[3]); return
    a, b, out = int(sys.argv[1]), min(int(sys.argv[2]), N), sys.argv[3]
    tmp = out[:-4] + ".part.mp4"
    p = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{3 * CW}x{3 * CH + BAR}",
                          "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", tmp],
                         stdin=subprocess.PIPE)
    log = []
    for f in range(a, b):
        im, hud = frame(f)
        p.stdin.write(np.asarray(im).tobytes())
        log.append([f] + [x for h in hud for x in (h[1], h[2], h[3], h[5])])
    p.stdin.close()
    np.save(out[:-4] + "_log.npy", np.array(log))
    if p.wait() == 0:
        os.replace(tmp, out)


if __name__ == "__main__":
    main()
