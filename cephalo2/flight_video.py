"""Side-by-side 1280x720 video: current fan-only flight control (left) vs organic tentacle correction (right),
both from flight4.py runs (full-robot MuJoCo, mid thrust).  Same fixed world camera on both sides, so drift and spin
are visible against the floor; a red ring on the floor marks the position target.
python flight_video.py still <t> | chunk <i> <n> | join <n>      (CEPH_GPU=1 renders on the GPU)
"""
import os, sys, subprocess, numpy as np
from PIL import Image, ImageDraw
import cephalo4 as C
from render import Camera, render
from geom import torus
import viz4 as V
import states4 as ST
import flight4 as FL

CASE = os.environ.get("CEPH_CASE", "mid")
TAGS = ("baseline", "organic")
DATA = {m: np.load(f"flight4_{m}_{CASE}.npz") for m in TAGS}
W, H = 640, 720
LABEL = {"baseline": ("CURRENT: fans only", "body tilts to move, no yaw control"),
         "organic": ("NEW: organic tentacle correction", "arms curl to aim the claw-props, body level")}
EYE_AZ = np.radians(232)


def ffmpeg():
    if os.environ.get("FFMPEG"):
        return os.environ["FFMPEG"]
    try:
        import imageio_ffmpeg
        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError:
        return "ffmpeg"


def event(t):
    if FL.T_GUST - 0.3 <= t < FL.T_GUST + FL.GUST_DUR + 0.4:
        return f"GUST: {FL.F_GUST:.0f} N side push on the body", (200, 40, 30)
    if FL.T_YAW - 0.3 <= t < FL.T_YAW + 1.5:
        return f"SWIRL: {FL.M_YAW:.0f} N m twist on the claw-props, 0.25 s", (200, 40, 30)
    if FL.T_HIT - 0.3 <= t < FL.T_HIT + 1.2:
        return "HIT: 0.2 kg ball at 6 m/s into arm 2's claw-prop", (200, 40, 30)
    if FL.T_MOVE <= t < FL.T_MOVE + FL.MOVE_DUR + 1.0:
        return f"MOVE: target slides {FL.MOVE_Y * 100:.0f} cm sideways in {FL.MOVE_DUR:.0f} s", (30, 90, 170)
    return "HOVER", (60, 60, 60)


def arrow(d, cam, p0, p1, col):
    xs, ys, zs = cam.project(np.array([p0, p1]), W, H)
    if (zs <= 0.05).any():
        return
    d.line([(xs[0], ys[0]), (xs[1], ys[1])], fill=col, width=6)
    v = np.array([xs[1] - xs[0], ys[1] - ys[0]]); v /= np.linalg.norm(v) + 1e-9; nrm = np.array([-v[1], v[0]])
    tip = np.array([xs[1], ys[1]])
    d.polygon([tuple(tip + 14 * v), tuple(tip - 6 * v + 11 * nrm), tuple(tip - 6 * v - 11 * nrm)], fill=col)


def half(tag, t):
    Z = DATA[tag]
    j = int(np.argmin(np.abs(Z["t"] - t)))
    st = ST.state(Z["qpos"][j]); fold, spin = V.fold_spin(None, Z, j)
    S = V.scene_from_state(st, Z["t"][j], spin, fold, Z["fan"][j], Z["edf"][j], imp=st[4])
    ref = Z["ref"][j]
    S.add(torus(np.array([ref[0], ref[1], 0.004]), np.array([0, 0, 1.0]), 0.09, 0.008, V.RED, n=32, m=6))
    tgt = np.array([0.0, FL.MOVE_Y / 2, 0.72])
    cam = Camera(tgt + np.array([np.cos(EYE_AZ) * 2.45, np.sin(EYE_AZ) * 2.45, 0.55]), tgt, fov_deg=50)
    img = Image.fromarray(render(S.prims, cam, W, H, ss=2, ground=(0.0, FL.MOVE_Y / 2), fog=(4.5, 12.0)))
    d = ImageDraw.Draw(img, "RGBA")
    if Z["gust"][j] > 0.5:
        p = st[0] + np.array([0, 0, 0.08])
        arrow(d, cam, p - np.array([0.75, 0, 0]), p - np.array([0.30, 0, 0]), (210, 40, 30, 230))
    d.rectangle([0, 0, W, 186], fill=(255, 255, 255, 220))
    t1, t2 = LABEL[tag]
    d.text((16, 8), t1, font=V.F_B, fill=(20, 20, 20))
    d.text((16, 42), t2, font=V.F_S, fill=(70, 70, 70))
    ev, ec = event(t)
    d.text((16, 70), ev, font=V.F_S, fill=ec)
    rows = [("off target", f"{Z['exy'][j]:5.1f} cm"), ("body tilt", f"{Z['tilt'][j]:4.1f} deg"),
            ("heading error", f"{abs(Z['yaw'][j]):5.1f} deg"), ("claw-prop axis tilt (max)", f"{Z['dax'][j].max():4.1f} deg")]
    for k, (a, b) in enumerate(rows):
        y = 98 + 22 * k
        d.text((16, y), a, font=V.F_S, fill=(70, 70, 70)); d.text((260, y), b, font=V.F_S, fill=(15, 15, 15))
    d.text((420, 98), f"t = {t:5.2f} s", font=V.F_S, fill=(40, 40, 40))
    return np.asarray(img)


def frame(t):
    a, b = half("baseline", t), half("organic", t)
    im = np.concatenate([a, b], 1); im[:, W - 1:W + 1] = 30
    pil = Image.fromarray(im); d = ImageDraw.Draw(pil, "RGBA")
    d.rectangle([0, H - 30, 2 * W, H], fill=(20, 22, 26, 200))
    d.text((12, H - 25), "Same robot, same MuJoCo model, same disturbances, mid thrust. Masses, thrust and gust are estimates. "
           "Red ring = position target. Real time.", font=V.font(15), fill=(230, 230, 230))
    return np.asarray(pil)


def chunk(ci, n):
    ts = np.arange(0, FL.T_END - 0.02, 1 / 30)
    idx = np.array_split(np.arange(len(ts)), n)[ci]
    out = f"fchunk_{ci:02d}.part.mp4"
    ff = subprocess.Popen([ffmpeg(), "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{2 * W}x{H}", "-r", "30",
                           "-i", "-", "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
    for k in idx:
        ff.stdin.write(frame(ts[k]).tobytes())
    ff.stdin.close()
    if ff.wait() == 0:
        os.replace(out, f"fchunk_{ci:02d}.mp4")
    print("chunk", ci, "done", len(idx), "frames", flush=True)


if __name__ == "__main__":
    if sys.argv[1] == "still":
        Image.fromarray(frame(float(sys.argv[2]))).save(f"fstill_{sys.argv[2]}.png")
    elif sys.argv[1] == "chunk":
        chunk(int(sys.argv[2]), int(sys.argv[3]))
    else:
        n = int(sys.argv[2])
        open("fchunks.txt", "w").write("".join(f"file 'fchunk_{i:02d}.mp4'\n" for i in range(n)))
        subprocess.run([ffmpeg(), "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "fchunks.txt", "-c", "copy",
                        f"cephalo1_organic_correction.mp4"], check=True)
