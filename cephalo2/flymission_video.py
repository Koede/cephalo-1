"""1920x1080 video of the organic flight mission (flymission4.py): tentacle steering, climb/descend, grab, carry home.
Red pods = arms currently leaning to steer (claw axis > 10 deg from vertical).  Minimap top right.
python flymission_video.py still <t> | chunk <i> <n> | join <n>
"""
import os, sys, subprocess, numpy as np
from PIL import Image, ImageDraw
import cephalo4 as C
from render import Camera, render
from geom import torus, sphere, frustum, Scene
import viz4 as V
import states4 as ST
import flymission4 as FM

CASE = os.environ.get("CEPH_CASE", "mid")
Z = np.load(f"flymission4_{CASE}.npz")
T = Z["t"]
W, H = 1920, 1080
FPS = 30
_k = 15
POS_S = np.array([np.convolve(np.pad(Z["pos"][:, c], _k, mode="edge"), np.ones(2 * _k + 1) / (2 * _k + 1), "valid") for c in range(3)]).T
YEL = [0.95, 0.80, 0.20]


def ffmpeg():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        return "ffmpeg"


def blend(t, a, b, r=0.8):
    return float(FM.mj5((t - a) / r) * (1 - FM.mj5((t - b) / r)))


def camera(j):
    t = T[j]; p = POS_S[j]
    tgt = np.array([p[0], p[1], p[2] + 0.10])
    az = np.radians(215 + 3.0 * t); dist, hz = 2.8, 0.85
    g = blend(t, FM.T_FANOFF - 0.3, FM.T_CARRY + 1.2)          # grab close-up
    tgt = (1 - g) * tgt + g * np.array([(p[0] + FM.OBJ[0]) / 2, (p[1] + FM.OBJ[1]) / 2, FM.Z_OBJ + 0.25])
    dist = (1 - g) * dist + g * 1.75; hz = (1 - g) * hz + g * 0.30
    az = (1 - g) * az + g * np.radians(FM.np.degrees(FM.AZ) - 65)
    rgt = np.array([-np.sin(az), np.cos(az), 0.0])
    tgt = tgt - rgt * (0.22 + 0.10 * g)              # push the robot right of the info panel
    return Camera(tgt + np.array([np.cos(az) * dist, np.sin(az) * dist, hz]), tgt, fov_deg=44)


def scene(j):
    st = ST.state(Z["qpos"][j]); p, R, q, c = st[:4]
    fold, spin = V.fold_spin(None, Z, j)
    fold[FM.GA] = Z["fold_ga"][j]
    S = Scene()
    V.body_mesh(S, p, R, edf_ang=T[j] * 40.0)
    steer = (Z["dax"][j] > 10) & (Z["gate"][j] > 0.5)
    for i in range(8):
        V.arm_mesh(S, i, p, R, q[i], c[i], spin[i], fold[i], Z["fan"][j][i] > 1.0, warn=bool(steer[i]))
    S.add(sphere(Z["obj"][j], FM.OBJ_R, YEL, n=18, m=12))
    o = FM.OBJ
    S.add(frustum(np.array([o[0], o[1], 0.0]), np.array([o[0], o[1], o[2] - FM.OBJ_R]), 0.012, 0.012, V.GREY, n=14))
    S.add(frustum(np.array([o[0], o[1], o[2] - FM.OBJ_R - 0.004]), np.array([o[0], o[1], o[2] - FM.OBJ_R + 0.002]), 0.04, 0.04, V.DARK, n=20))
    S.add(torus(np.array([0, 0, 0.004]), np.array([0, 0, 1.0]), 0.25, 0.012, [0.2, 0.55, 0.3], n=40, m=6))     # home pad
    return S, steer


def minimap(d, j):
    x0, y0, sz = W - 330, 84, 300
    d.rectangle([x0, y0, x0 + sz, y0 + sz + 26], fill=(255, 255, 255, 215), outline=(60, 60, 60, 255))
    lo, hi = -0.8, 3.3
    f = lambda x, y: (x0 + 10 + (x - lo) / (hi - lo) * (sz - 20), y0 + sz - 10 - (y - lo) / (hi - lo) * (sz - 20))
    pts = [(0.0, 0.0)] + [tuple(s[2][:2]) for s in FM.SEGS]
    d.line([f(*p) for p in pts], fill=(170, 170, 170), width=2)
    d.ellipse([f(0, 0)[0] - 9, f(0, 0)[1] - 9, f(0, 0)[0] + 9, f(0, 0)[1] + 9], outline=(40, 140, 70), width=3)
    ox, oy = f(FM.OBJ[0], FM.OBJ[1]); d.rectangle([ox - 5, oy - 5, ox + 5, oy + 5], outline=(120, 120, 120), width=2)
    tr = [f(*Z["pos"][k][:2]) for k in range(0, j + 1, 3)]
    if len(tr) > 1:
        d.line(tr, fill=(40, 90, 200), width=3)
    px, py = f(*Z["pos"][j][:2])
    d.ellipse([px - 7, py - 7, px + 7, py + 7], fill=(30, 30, 30))
    a = Z["adem"][j]; n = np.linalg.norm(a)
    if n > 0.15:
        e = np.array([a[0], -a[1]]) / n * min(18 + 12 * n, 50)
        d.line([(px, py), (px + e[0], py + e[1])], fill=(210, 40, 30), width=4)
    if Z["latched"][j] > 0.5:
        d.ellipse([px + 5, py + 5, px + 13, py + 13], fill=(230, 190, 40))
    d.text((x0 + sz - 30, y0 + 6), "N", font=V.F_B, fill=(30, 30, 30))
    d.line([(x0 + sz - 22, y0 + 60), (x0 + sz - 22, y0 + 36)], fill=(30, 30, 30), width=3)
    d.text((x0 + 10, y0 + sz + 4), "top view: blue = flown, red = push", font=V.font(15), fill=(60, 60, 60))


def frame(j):
    S, steer = scene(j)
    cam = camera(j)
    img = Image.fromarray(render(S.prims, cam, W, H, ss=2, ground=(POS_S[j][0], POS_S[j][1]), fog=(5.0, 14.0)))
    d = ImageDraw.Draw(img, "RGBA")
    t = T[j]
    d.rectangle([0, 0, W, 64], fill=(20, 22, 26, 200))
    d.text((24, 12), "CEPHALO-1 REV 4  |  organic flight: tentacle steering, climb / descend, grab in flight", font=V.F_B, fill=(240, 240, 240))
    d.rectangle([20, 80, 760, 470], fill=(255, 255, 255, 205), outline=(60, 60, 60, 255))
    d.text((36, 92), FM.label(t), font=V.font(30, True), fill=(20, 20, 20))
    v = Z["vel"][j]
    steer_ids = [i for i in range(8) if steer[i]]
    rows = [("t", f"{t:5.2f} s"), ("Speed / climb rate", f"{np.linalg.norm(v[:2]):4.2f} m/s  /  {v[2]:+4.2f} m/s"),
            ("Altitude (plate)", f"{Z['pos'][j][2]:4.2f} m"), ("Body tilt", f"{Z['tilt'][j]:4.1f} deg"),
            ("Steering arms (red pods)", " ".join(str(i) for i in steer_ids) if steer_ids else "-"),
            ("Max claw-prop lean", f"{np.delete(Z['dax'][j], FM.GA).max():4.1f} deg"),
            ("Claw-props running", f"{int((Z['fan'][j] > 1).sum())}   EDF {Z['edf'][j]:.0f} N"),
            ("Payload (0.30 kg ball)", "held by arm 2" if Z["latched"][j] > 0.5 else "-"),
            ("Worst servo load", f"{Z['servo'][j]:.2f} x stall")]
    y = 140
    for k, val in rows:
        d.text((36, y), k, font=V.F_M, fill=(70, 70, 70)); d.text((400, y), val, font=V.F_M, fill=(15, 15, 15)); y += 36
    minimap(d, j)
    d.rectangle([0, H - 40, W, H], fill=(20, 22, 26, 190))
    d.text((24, H - 32), "MuJoCo full-robot simulation, mid thrust. Masses, thrust, drag (CdA 0.10 m^2) and the object are estimates; "
           "grasp = spring tie once the closing fingers surround the ball (finger contact not simulated). Real time.",
           font=V.F_S, fill=(230, 230, 230))
    return np.asarray(img)


def chunk(ci, n):
    idx = np.array_split(np.arange(len(T)), n)[ci]
    out = f"mchunk_{ci:02d}.part.mp4"
    ff = subprocess.Popen([ffmpeg(), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
                           "-i", "-", "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
    for j in idx:
        ff.stdin.write(frame(j).tobytes())
    ff.stdin.close()
    if ff.wait() == 0:
        os.replace(out, f"mchunk_{ci:02d}.mp4")
    print("chunk", ci, "done", flush=True)


if __name__ == "__main__":
    if sys.argv[1] == "still":
        j = int(np.argmin(np.abs(T - float(sys.argv[2])))); Image.fromarray(frame(j)).save(f"mstill_{sys.argv[2]}.png")
    elif sys.argv[1] == "chunk":
        chunk(int(sys.argv[2]), int(sys.argv[3]))
    else:
        n = int(sys.argv[2])
        open("mchunks.txt", "w").write("".join(f"file 'mchunk_{i:02d}.mp4'\n" for i in range(n)))
        subprocess.run([ffmpeg(), "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "mchunks.txt", "-c", "copy",
                        "cephalo1_organic_flight_grab.mp4"], check=True)
