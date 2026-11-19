"""1280x720 snippet videos of the in-flight chores (chores4.py: shovel, dishes) and of lying down to rest (rest4.py).
python3 chores_video.py <shovel|dishes|rest> still <t> | render <t0> <t1> <name>   -> cephalo1_<name>.mp4
Red pods = arms leaning to steer.  Tools are drawn from their simulated poses.
"""
import os, sys, subprocess, numpy as np, mujoco
from PIL import Image, ImageDraw
SCN = sys.argv[1]
os.environ["CEPH_SCN"] = SCN
import cephalo4 as C
from render import Camera, render
from geom import torus, sphere, frustum, box, Scene
import viz4 as V
if SCN == "rest":
    import rest4 as RS
    Z = np.load("rest4.npz"); LAB = RS.label
else:
    import chores4 as CH
    Z = np.load(f"chores4_{SCN}_{os.environ.get('CEPH_CASE', 'mid')}.npz"); LAB = CH.label
import states4 as ST
T = Z["t"]; W, H = 1280, 720
_k = 12
POS_S = np.array([np.convolve(np.pad(Z["pos"][:, c], _k, mode="edge"), np.ones(2 * _k + 1) / (2 * _k + 1), "valid") for c in range(3)]).T
WOOD = [0.72, 0.55, 0.36]; SAND = [0.88, 0.80, 0.58]; STEEL = [0.72, 0.74, 0.77]; DARK = [0.22, 0.23, 0.25]; WHITE = [0.93, 0.93, 0.91]
COUNTER = [0.80, 0.82, 0.84]; YEL = [0.95, 0.80, 0.20]; GREEN = [0.25, 0.60, 0.35]
I3 = np.eye(3)


def ffmpeg():
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        return "ffmpeg"


def obj_pose(j, k, n):
    q = Z["qpos"][j][-7 * (n - k):][:7]
    R = np.zeros(9); mujoco.mju_quat2Mat(R, q[3:7]); return q[:3].copy(), R.reshape(3, 3)


def world(S, j):
    if SCN == "shovel":
        p, R = obj_pose(j, 0, 1); x = R[:, 0]
        S.add(frustum(p - x * 0.50, p + x * 0.09, 0.014, 0.014, WOOD, n=12))
        S.add(sphere(p - x * 0.50, 0.024, DARK, n=10, m=8))
        S.add(box(p + x * 0.19, R, [0.22, 0.16, 0.005], STEEL))
        c = CH.SAND_C
        S.add(box(np.array([c[0], c[1], 0.14]), I3, [0.74, 0.74, 0.28], WOOD))
        S.add(box(np.array([c[0], c[1], 0.29]), I3, [0.68, 0.68, 0.02], SAND))
        if Z["sand"][j] > 0.5 and T[j] < CH.T_SAND_OFF - 0.4:
            S.add(sphere(p + x * 0.20 + R[:, 2] * 0.025, 0.05, SAND, n=12, m=8))
        if T[j] > CH.T_SAND_OFF - 0.3:
            g = np.array([CH.H4[0], CH.H4[1], 0.0]) + CH.BF(0.50, 0, 0)
            S.add(sphere(np.array([g[0], g[1], 0.0]), 0.07, SAND, n=14, m=8))
    elif SCN == "dishes":
        E, TVv = CH.E, CH.TV; Rc = np.stack([E, TVv, CH.UZ], 1)
        for x0, x1, zt in ((-0.62, -0.221, 0.90), (0.221, 0.62, 0.90), (-0.22, 0.22, 0.80)):     # front of the basin left open (cutaway)
            S.add(box(CH.BF((x0 + x1) / 2, 0, zt / 2), Rc, [x1 - x0, 0.56, zt], COUNTER))
        S.add(box(CH.BF(0, 0.26, 0.45), Rc, [0.44, 0.04, 0.90], COUNTER))
        S.add(frustum(CH.BF(0, 0.25, 0.90), CH.BF(0, 0.25, 1.06), 0.012, 0.012, STEEL, n=10))           # tap
        S.add(frustum(CH.BF(0, 0.25, 1.06), CH.BF(0, 0.13, 1.06), 0.010, 0.010, STEEL, n=10))
        S.add(box(CH.BF(-0.27, 0, 0.912), Rc, [0.26, 0.34, 0.012], [0.35, 0.36, 0.38]))                               # drying rack
        S.add(box(CH.BF(0.34, 0, 0.915), Rc, [0.05, 0.08, 0.03], DARK))                                 # brush rest
        p, R = obj_pose(j, 0, 2); x = R[:, 0]; c = p - x * 0.03
        S.add(frustum(c - R[:, 2] * 0.02, c + R[:, 2] * 0.02, 0.085, 0.10, DARK, n=28))
        S.add(frustum(c + R[:, 2] * 0.012, c + R[:, 2] * 0.021, 0.088, 0.092, [0.45, 0.46, 0.48], n=28))
        S.add(frustum(c + x * 0.10 + R[:, 2] * 0.02, c + x * 0.30 + R[:, 2] * 0.02, 0.011, 0.011, DARK, n=10))
        p, R = obj_pose(j, 1, 2); x = R[:, 0]
        S.add(frustum(p - x * 0.16, p + x * 0.09, 0.009, 0.009, GREEN, n=10))
        S.add(box(p + x * 0.10 - R[:, 2] * 0.012, R, [0.05, 0.035, 0.024], GREEN))
        S.add(box(p + x * 0.10 - R[:, 2] * 0.044, R, [0.045, 0.03, 0.04], WHITE))


def scene(j):
    st = ST.state(Z["qpos"][j]); p, R, q, c = st[:4]
    fold, spin = V.fold_spin(None, Z, j)
    steer = np.zeros(8, bool)
    if SCN != "rest":
        fold[CH.HA], fold[CH.HB] = Z["fold"][j]
        steer = (Z["dax"][j] > 10) & (Z["gate"][j] > 0.5)
    S = Scene()
    V.body_mesh(S, p, R, edf_ang=T[j] * 40.0 * (Z["edf"][j] > 1))
    for i in range(8):
        V.arm_mesh(S, i, p, R, q[i], c[i], spin[i], fold[i], Z["fan"][j][i] > 1.0, warn=bool(steer[i]))
    world(S, j)
    return S


def camera(j):
    t = T[j]; p = POS_S[j]
    if SCN == "rest":
        a = np.radians(215 + 5 * t); tgt = np.array([p[0], p[1], 0.12])
        return Camera(tgt + np.array([np.cos(a) * 1.9, np.sin(a) * 1.9, 0.75]), tgt, fov_deg=44)
    a = CH.AZ - np.radians(78) + np.radians(14) * np.sin(0.25 * t)
    dz, dist, hz = (-0.22, 2.0, 0.42) if SCN == "shovel" else (-0.22, 1.75, 0.80)
    tgt = np.array([p[0], p[1], max(p[2] + dz, 0.25)])
    rgt = np.array([-np.sin(a), np.cos(a), 0.0]); tgt = tgt - rgt * 0.12
    return Camera(tgt + np.array([np.cos(a) * dist, np.sin(a) * dist, hz]), tgt, fov_deg=46)


def frame(j):
    S = scene(j); t = T[j]
    img = Image.fromarray(render(S.prims, camera(j), W, H, ss=2, ground=(POS_S[j][0], POS_S[j][1]), fog=(5.0, 14.0)))
    d = ImageDraw.Draw(img, "RGBA")
    title = {"shovel": "in-flight chores: pick up a shovel, hand it over, dig", "dishes": "in-flight chores: washing up at the sink",
             "rest": "lie down to rest"}[SCN]
    d.rectangle([0, 0, W, 44], fill=(20, 22, 26, 200))
    d.text((16, 9), "CEPHALO-1 REV 4  |  " + title, font=V.font(20, True), fill=(240, 240, 240))
    d.rectangle([12, 54, 520, 262 if SCN != "rest" else 190], fill=(255, 255, 255, 210), outline=(60, 60, 60, 255))
    d.text((24, 60), LAB(t)[:44], font=V.font(19, True), fill=(20, 20, 20))
    if SCN == "rest":
        rows = [("t", f"{t:5.2f} s"), ("Plate height", f"{Z['pos'][j][2] * 1000:4.0f} mm"), ("Winch line", f"{Z['T'][j]:.0f} N"),
                ("Worst servo load", f"{Z['servo'][j]:.2f} x stall")]
    else:
        held = Z["held"][j]; g = Z["grip"][j]
        rows = [("t", f"{t:5.2f} s"), ("Altitude / body tilt", f"{Z['pos'][j][2]:4.2f} m / {Z['tilt'][j]:3.1f} deg"),
                ("Claw-props running", f"{int((Z['fan'][j] > 1).sum())} of 8  (max {Z['fan'][j].max():4.1f} N)"),
                ("Hands holding", ("arm 2 " if held[0] else "") + ("arm 6" if held[1] else "") or "-"),
                ("Payload", f"{Z['payload'][j]:.2f} kg"),
                ("Grip load (max of hands)", f"{max(g[0], g[2]):4.1f} N, {max(g[1], g[3]):.2f} N m"),
                ("Worst servo load", f"{Z['servo'][j]:.2f} x stall")]
    y = 90
    for k, v in rows:
        d.text((24, y), k, font=V.F_S, fill=(70, 70, 70)); d.text((270, y), v, font=V.F_S, fill=(15, 15, 15)); y += 24
    d.rectangle([0, H - 30, W, H], fill=(20, 22, 26, 190))
    foot = ("MuJoCo full-robot simulation. Masses and limits are estimates." if SCN == "rest" else
            "MuJoCo full-robot simulation, mid thrust. Masses, thrust, tools and sand are estimates; grasps are spring ties "
            "(finger and bristle contact not simulated). Real time.")
    d.text((12, H - 24), foot, font=V.font(13), fill=(230, 230, 230))
    return np.asarray(img)


if __name__ == "__main__":
    if sys.argv[2] == "still":
        j = int(np.argmin(np.abs(T - float(sys.argv[3])))); Image.fromarray(frame(j)).save(f"cstill_{SCN}_{sys.argv[3]}.png")
    else:
        t0, t1, name = float(sys.argv[3]), float(sys.argv[4]), sys.argv[5]
        idx = np.where((T >= t0 - 1e-6) & (T < t1))[0]
        out = f"cephalo1_{name}.part.mp4"
        ff = subprocess.Popen([ffmpeg(), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", "30",
                               "-i", "-", "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
        for j in idx:
            ff.stdin.write(frame(j).tobytes())
        ff.stdin.close()
        if ff.wait() == 0:
            os.replace(out, f"cephalo1_{name}.mp4")
        print(name, "done", len(idx), "frames", flush=True)
