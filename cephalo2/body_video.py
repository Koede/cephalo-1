"""Video of a body4 session: the robot driven by intents, with its senses on screen (1920x1080, 30 fps).
python3 body_video.py still <t> [npz] ; chunk <i> <n> [npz] ; join <n> [npz]"""
import os, sys, subprocess, numpy as np
from PIL import Image, ImageDraw
import cephalo4 as C
from render import Camera, render
import viz4 as V
import states4 as ST

NPZ = sys.argv[-1] if sys.argv[-1].endswith(".npz") else "body4_mid_day.npz"
Z = np.load(NPZ)
CASE = NPZ.split("_")[1]
FMAX = {"mid": 12.6, "pess": 10.1, "opt": 15.1}.get(CASE, 12.6)
W, H = V.W_IMG, V.H_IMG
STEPS = ["Rest (lying on the EDF bumper)", "Get up", "Breathe", "Walk 18 cm forward, turn 30 deg", "Wave arm 1",
         "Breathe", "Take off to 0.8 m", "Fly north 1.5 m, climb 0.3 m", "Fly east 1.5 m, descend 0.3 m, turn 90 deg",
         "Hover", "Land (feel for the floor)", "Breathe", "Lie down", "Rest"]
OUT = os.path.basename(NPZ).replace("body4_", "cephalo1_body_").replace(".npz", ".mp4")

t = Z["t"]; N = len(t)
# smoothed camera target and flight blend
com = Z["com"].copy()
k = np.ones(25) / 25
sm = np.stack([np.convolve(np.pad(com[:, a], 12, mode="edge"), k, "valid") for a in range(3)], 1)
air_s = np.convolve(np.pad(Z["air"], 30, mode="edge"), np.ones(61) / 61, "valid")
fan = Z["fan"]
spin = np.cumsum(np.sqrt(np.clip(fan, 0, None)) * np.gradient(t)[:, None], 0) * 9.0


def frame(j):
    st = ST.state(Z["qpos"][j])
    fold = np.radians(60) * (1 - np.clip(fan[j] / 2.5, 0, 1))
    S = V.scene_from_state(st, t[j], spin[j], fold, fan[j], Z["edf"][j])
    if "step" in NPZ:
        from geom import box
        S.add(box(np.array([0, 1.6, 0.06]), np.eye(3), np.array([0.5, 0.5, 0.06]) * 2, (0.75, 0.70, 0.60)))
    a = air_s[j]
    az = np.radians(215 + 3.0 * t[j])
    dist = 2.0 + 0.7 * a
    tgt = np.array([sm[j, 0], sm[j, 1], max(sm[j, 2] + 0.05, 0.30)])
    cam = Camera(tgt + np.array([np.cos(az) * dist, np.sin(az) * dist, 0.85 - 0.25 * a]), tgt, fov_deg=42)
    img = render(S.prims, cam, W, H, ss=2, ground=(st[0][0], st[0][1]), fog=(4.5, 13.0))
    return hud(img, j)


def hud(img, j):
    im = Image.fromarray(img); d = ImageDraw.Draw(im, "RGBA")
    d.rectangle([0, 0, W, 64], fill=(20, 22, 26, 200))
    d.text((24, 12), "CEPHALO-1 REV 4  |  one body, driven by intents: skills, reflexes, a spinal layer  (body4.py)", font=V.F_B, fill=(240, 240, 240))
    # intent list
    x0, y0 = 20, 80
    d.rectangle([x0, y0, x0 + 560, y0 + 46 + 30 * len(STEPS)], fill=(255, 255, 255, 205), outline=(60, 60, 60, 255))
    d.text((x0 + 16, y0 + 10), "INTENTS", font=V.F_M, fill=(30, 30, 30))
    cur = int(Z["sidx"][j])
    for n, s in enumerate(STEPS):
        yy = y0 + 44 + 30 * n
        col = (180, 30, 30) if n == cur else ((120, 120, 120) if n < cur else (40, 40, 40))
        d.text((x0 + 16, yy), ("> " if n == cur else "   ") + s, font=V.F_S if n != cur else V.F_M, fill=col)
    # current action + senses
    y1 = y0 + 60 + 30 * len(STEPS)
    d.rectangle([x0, y1, x0 + 560, y1 + 300], fill=(255, 255, 255, 205), outline=(60, 60, 60, 255))
    d.text((x0 + 16, y1 + 10), str(Z["label"][j])[:44], font=V.F_M, fill=(15, 15, 15))
    rows = [("t", f"{t[j]:5.1f} s"), ("Claw-prop max (est.)", f"{FMAX:.1f} N"), ("EDF", f"{Z['edf'][j]:.0f} N"),
            ("Claw-props running", f"{int((fan[j] > 1).sum())}  (max {fan[j].max():.1f} N)"),
            ("Ground -> flight control", f"{Z['air'][j] * 100:3.0f} %"), ("Load governor", f"{Z['rate'][j]:.2f}"),
            ("Body tilt / heading", f"{Z['tilt'][j]:.1f} deg / {Z['yaw'][j]:.0f} deg"), ("Winch line", f"{Z['T'][j]:.0f} N")]
    for n, (a, b) in enumerate(rows):
        d.text((x0 + 16, y1 + 50 + 30 * n), a, font=V.F_S, fill=(70, 70, 70)); d.text((x0 + 300, y1 + 50 + 30 * n), b, font=V.F_S, fill=(15, 15, 15))
    # per-arm senses
    xa, ya = W - 520, 80
    d.rectangle([xa, ya, W - 20, ya + 330], fill=(255, 255, 255, 205), outline=(60, 60, 60, 255))
    d.text((xa + 14, ya + 8), "Per arm:  servo load (x stall)   foot force   claw-prop", font=V.F_S, fill=(30, 30, 30))
    for i in range(8):
        yy = ya + 44 + i * 34
        d.text((xa + 14, yy), f"arm {i}", font=V.F_S, fill=(40, 40, 40))
        v = min(Z["servo_arm"][j][i], 1.0)
        col = (40, 160, 70) if v < 0.7 else ((230, 160, 30) if v < 0.95 else (210, 40, 40))
        d.rectangle([xa + 80, yy + 3, xa + 230, yy + 21], outline=(120, 120, 120)); d.rectangle([xa + 80, yy + 3, xa + 80 + int(150 * v), yy + 21], fill=col)
        f = min(Z["foot"][j][i] / 40.0, 1.0)
        d.rectangle([xa + 245, yy + 3, xa + 365, yy + 21], outline=(120, 120, 120)); d.rectangle([xa + 245, yy + 3, xa + 245 + int(120 * f), yy + 21], fill=(60, 110, 200))
        p = min(fan[j][i] / FMAX, 1.0)
        d.rectangle([xa + 380, yy + 3, xa + 480, yy + 21], outline=(120, 120, 120)); d.rectangle([xa + 380, yy + 3, xa + 380 + int(100 * p), yy + 21], fill=(230, 130, 30))
    d.rectangle([0, H - 40, W, H], fill=(20, 22, 26, 190))
    d.text((24, H - 32), "MuJoCo full-robot simulation, real time. Masses, claw-prop thrust, rotor inertia and clutch parameters are estimates. "
           "Foot force = sim contact (real robot: servo current + tip IMU + ToF).", font=V.F_S, fill=(230, 230, 230))
    return np.asarray(im)


def chunk(ci, n):
    idx = np.array_split(np.arange(N), n)[ci]
    out = f"bchunk_{ci:02d}.part.mp4"
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", "30", "-i", "-",
                           "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
    for j in idx:
        ff.stdin.write(frame(j).tobytes())
    ff.stdin.close()
    if ff.wait() == 0: os.replace(out, f"bchunk_{ci:02d}.mp4")


if __name__ == "__main__":
    if sys.argv[1] == "still":
        j = int(np.argmin(np.abs(t - float(sys.argv[2])))); Image.fromarray(frame(j)).save(f"bstill_{sys.argv[2]}.png")
    elif sys.argv[1] == "chunk":
        chunk(int(sys.argv[2]), int(sys.argv[3]))
    else:
        n = int(sys.argv[2])
        open("bchunks.txt", "w").write("".join(f"file 'bchunk_{i:02d}.mp4'\n" for i in range(n)))
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "bchunks.txt", "-c", "copy", OUT], check=True)
