"""Video of the flip run (python3 flip_video.py still <t> | chunk <i> <n> | join <n>)  ->  cephalo1_flip_walk_inverted.mp4"""
import os, sys, subprocess, json, numpy as np
os.environ.setdefault("CEPH_PROP", "xl"); os.environ.setdefault("CEPH_PACKS", "1"); os.environ.setdefault("CEPH_J1", "STS3250")
from PIL import Image, ImageDraw
import cephalo4 as C
import power4 as PW
C.CLAW_R = PW.PROPS["xl"]["R"]                    # draw the 7.5 in claw-props
from render import Camera, render
import viz4 as V
import states4 as ST

NPZ = os.environ.get("FLIP_NPZ", "flip4.npz")
Z = np.load(NPZ); J = json.load(open(NPZ.replace(".npz", ".json"))) if os.path.exists(NPZ.replace(".npz", ".json")) else None
W, H = 1920, 1080
N = len(Z["t"])
OUT = "cephalo1_flip_walk_inverted.mp4"


def cam_target(j):
    # smoothed CoM track (camera does not shake with the body)
    k = np.arange(max(0, j - 20), min(N, j + 21))
    p = Z["com"][k].mean(0)
    return np.array([p[0], p[1], max(p[2] - 0.08, 0.25)])


def frame(j):
    t = Z["t"][j]
    st = ST.state(Z["qpos"][j]); fan = Z["fan"][j]
    fold = np.where(fan > 1, 0.0, np.radians(60))
    spin = np.cumsum(np.sqrt(np.clip(Z["fan"][:j + 1], 0, None)), 0)[-1] * 9.0 / 30
    S = V.scene_from_state(st, t, spin, fold, fan, Z["edf"][j])
    tgt = cam_target(j)
    az = np.radians(235 + 3.0 * t)
    dist = 1.9 + 0.6 * float(np.clip((tgt[2] - 0.3) / 0.7, 0, 1))
    cam = Camera(tgt + np.array([np.cos(az) * dist, np.sin(az) * dist, 0.3 * dist]), tgt, fov_deg=44)
    img = render(S.prims, cam, W, H, ss=2, ground=(tgt[0], tgt[1]), fog=(5.0, 14.0))
    im = Image.fromarray(img); d = ImageDraw.Draw(im, "RGBA")
    d.rectangle([0, 0, W, 64], fill=(20, 22, 26, 205))
    d.text((24, 14), "CEPHALO-1  |  mid-air flip, inverted landing, walking upside down", font=V.F_B, fill=(240, 240, 240))
    d.rectangle([20, 80, 760, 420], fill=(255, 255, 255, 205), outline=(60, 60, 60, 255))
    y = 92
    d.text((36, y), str(Z["label"][j]), font=V.F_M, fill=(20, 20, 20)); y += 44
    up = Z["up"][j]
    rows = [f"t {t:5.1f} s     body turned {up:5.1f} deg from upright",
            f"battery 6S: {Z['V'][j]:4.1f} V  {Z['I'][j]:4.0f} A    claw-prop ceiling {Z['fmax'][j]:4.1f} N",
            "claw-props (N): " + " ".join(f"{f:4.1f}" if f > 0.5 else "  - " for f in fan) + f"   EDF {Z['edf'][j]:4.1f}",
            "worst servo per arm: " + " ".join(f"{v:.2f}" for v in Z["servo_arm"][j]),
            "feet on the floor: " + " ".join(str(i) for i in range(8) if Z["foot"][j][i] > 3.0)]
    for r in rows:
        d.text((36, y), r, font=V.F_S, fill=(30, 30, 30)); y += 34
    d.rectangle([0, H - 40, W, H], fill=(20, 22, 26, 200))
    d.text((14, H - 32), "MuJoCo full-robot sim: gravity, battery sag (1 x 6S 5000 mAh), prop reaction torque, servo stall limits, clutches. "
           "7.5 in claw-props (proposed), STS3250 at the root yaw j1. Real time.", font=V.F_S, fill=(230, 230, 230))
    return np.asarray(im)


if __name__ == "__main__":
    if sys.argv[1] == "still":
        j = int(np.argmin(np.abs(Z["t"] - float(sys.argv[2])))); Image.fromarray(frame(j)).save(f"fstill_{sys.argv[2]}.png")
    elif sys.argv[1] == "chunk":
        ci, n = int(sys.argv[2]), int(sys.argv[3]); idx = np.array_split(np.arange(N), n)[ci]
        out = f"fchunk_{ci:02d}.part.mp4"
        ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", "30", "-i", "-",
                               "-c:v", "libx264", "-crf", "20", "-preset", "veryfast", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
        for j in idx: ff.stdin.write(frame(j).tobytes())
        ff.stdin.close()
        if ff.wait() == 0: os.replace(out, f"fchunk_{ci:02d}.mp4")
    else:
        n = int(sys.argv[2])
        open("fchunks.txt", "w").write("".join(f"file 'fchunk_{i:02d}.mp4'\n" for i in range(n)))
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "fchunks.txt", "-c", "copy", OUT], check=True)
