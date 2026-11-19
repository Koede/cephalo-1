"""Side-by-side take-off comparison of two take-off versions (default v6 vs v7), both from the full-robot MuJoCo runs.
CEPH_LEFT / CEPH_RIGHT pick the versions (v5, v6, v7).
python3 takeoff_video.py chunk <i> <n> ; python3 takeoff_video.py join <n> ; python3 takeoff_video.py still <tau>"""
import os, sys, subprocess, importlib.util, numpy as np
from PIL import Image, ImageDraw
import cephalo4 as C
from render import Camera, render
import viz4 as V


def load_sim(tag):
    os.environ["CEPH_TAKEOFF"] = tag
    spec = importlib.util.spec_from_file_location(f"sim4_{tag}", "sim4.py")
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    return m


LEFT, RIGHT = os.environ.get("CEPH_LEFT", "v6"), os.environ.get("CEPH_RIGHT", "v7")
FILE = {"v5": "sim4_mid_v5.npz", "v6": "sim4_mid_v6.npz", "v7": "sim4_mid.npz"}
SIMS = {t: load_sim(t) for t in (LEFT, RIGHT)}
os.environ["CEPH_TAKEOFF"] = "v7"
import states4 as ST
DATA = {t: np.load(FILE[t]) for t in (LEFT, RIGHT)}
TAU0 = -0.5
TAU1 = max(SIMS[t].T_LEG0[1] - SIMS[t].T_TO for t in (LEFT, RIGHT)) + 1.5
W, H = 640, 720
NAMES = {"v5": "v5: stepwise", "v6": "v6: fluid wave", "v7": "v7: organic + fast"}
LABEL = {LEFT: "BEFORE " + NAMES[LEFT], RIGHT: "NEW " + NAMES[RIGHT]}


def half(tag, tau):
    SM, Z = SIMS[tag], DATA[tag]
    t_abs = SM.T_TO + tau
    ok = np.r_[True, np.diff(Z["t"]) > 0.02] | (np.abs(Z["t"] - t_abs) < 0.02)
    idx = np.where(ok)[0]; j = idx[np.argmin(np.abs(Z["t"][idx] - t_abs))]
    st = ST.state(Z["qpos"][j]); fold, spin = V.fold_spin(SM, Z, j)
    S = V.scene_from_state(st, Z["t"][j], spin, fold, Z["fan"][j], Z["edf"][j])
    p = st[0]; tgt = np.array([p[0], p[1], 0.45])
    cam = Camera(tgt + np.array([np.cos(np.radians(235)) * 2.3, np.sin(np.radians(235)) * 2.3, 0.75]), tgt, fov_deg=44)
    img = Image.fromarray(render(S.prims, cam, W, H, ss=2, ground=(p[0], p[1]), fog=(4.0, 12.0)))
    d = ImageDraw.Draw(img, "RGBA")
    d.rectangle([0, 0, W, 132], fill=(255, 255, 255, 215))
    d.text((16, 10), LABEL[tag], font=V.F_B, fill=(20, 20, 20))
    done = t_abs >= SM.T_LEG0[1]
    mode = SM.mode(t_abs) if t_abs >= SM.T_TO else "STAND"
    d.text((16, 48), mode[:40], font=V.F_S, fill=(40, 40, 40))
    d.text((16, 74), f"t = {tau:5.2f} s   tilt {Z['tilt'][j]:4.1f} deg   claw-props on {int((Z['fan'][j] > 1).sum())}",
           font=V.F_S, fill=(40, 40, 40))
    d.text((16, 100), f"all legs up at {SM.T_LEG0[1] - SM.T_TO:.1f} s" + ("   DONE" if done else ""), font=V.F_S,
           fill=(20, 130, 50) if done else (90, 90, 90))
    return np.asarray(img)


def frame(tau):
    a, b = half(LEFT, tau), half(RIGHT, tau)
    im = np.concatenate([a, b], 1); im[:, W - 1:W + 1] = 30
    pil = Image.fromarray(im); d = ImageDraw.Draw(pil, "RGBA")
    d.rectangle([0, H - 30, 2 * W, H], fill=(20, 22, 26, 200))
    d.text((12, H - 25), "Same robot, same MuJoCo model, mid thrust (12.6 N). Masses and thrust are estimates. Real time.",
           font=V.F_S, fill=(230, 230, 230))
    return np.asarray(pil)


def chunk(ci, n):
    taus = np.arange(TAU0, TAU1, 1 / 30)
    idx = np.array_split(np.arange(len(taus)), n)[ci]
    out = f"tchunk_{ci:02d}.part.mp4"
    ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{2 * W}x{H}", "-r", "30",
                           "-i", "-", "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
    for k in idx:
        ff.stdin.write(frame(taus[k]).tobytes())
    ff.stdin.close()
    if ff.wait() == 0:
        os.replace(out, f"tchunk_{ci:02d}.mp4")


if __name__ == "__main__":
    if sys.argv[1] == "still":
        Image.fromarray(frame(float(sys.argv[2]))).save(f"tstill_{sys.argv[2]}.png")
    elif sys.argv[1] == "chunk":
        chunk(int(sys.argv[2]), int(sys.argv[3]))
    else:
        n = int(sys.argv[2])
        open("tchunks.txt", "w").write("".join(f"file 'tchunk_{i:02d}.mp4'\n" for i in range(n)))
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "tchunks.txt", "-c", "copy",
                        f"cephalo1_takeoff_{LEFT}_vs_{RIGHT}.mp4"], check=True)
