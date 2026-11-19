"""Video of a quad-mode episode (python3 quad_video.py still <t> <npz> | chunk <i> <n> <npz> | join <n> <npz>).
Left: before training (ansatz), right: after; or a single run full-width when one npz is given."""
import os, sys, subprocess, json, numpy as np
os.environ.setdefault("CEPH_PROP", "xl"); os.environ.setdefault("CEPH_PACKS", "2")
from PIL import Image, ImageDraw
import cephalo4 as C
import power4 as PW
C.CLAW_R = PW.PROPS["xl"]["R"]                    # draw the 7.5 in claw-props
import quad4r as Q4
import hand3_draw as H3
H3.install((2, 6))
from render import Camera, render
from geom import box, frustum, sphere
import viz4 as V
import states4 as ST

NPZ = [a for a in sys.argv if a.endswith(".npz")]
Zs = [np.load(f) for f in NPZ]
J = [json.load(open(f.replace(".npz", ".json"))) for f in NPZ]
Q = Q4.Quad()                                       # model indices, sand box position
TQ = Q.tq
W, H = (960, 1080) if len(Zs) == 2 else (1920, 1080)
N = min(len(z["t"]) for z in Zs)
WOOD, STEEL, PAN, SAND = (0.62, 0.45, 0.28), (0.55, 0.57, 0.60), (0.20, 0.20, 0.22), (0.86, 0.76, 0.52)


def tool_pose(qpos, n):
    a = TQ[n]; R = np.zeros(9)
    import mujoco
    mujoco.mju_quat2Mat(R, qpos[a + 3:a + 7]); return qpos[a:a + 3], R.reshape(3, 3)


def scene(Z, j):
    st = ST.state(Z["qpos"][j]); fan = Z["fan"][j]
    if "fold" in Z.files:
        for n, i in enumerate(Q4.WORK): H3.STATE[i] = dict(lag=Z["lag"][j][n], fold=Z["fold"][j][n])
    fold = np.where(fan > 1, 0.0, np.radians(60))
    spin = np.cumsum(np.sqrt(np.clip(Z["fan"][:j + 1], 0, None)), 0)[-1] * 9.0 / 30
    S = V.scene_from_state(st, Z["t"][j], spin, fold, fan, Z["edf"][j])
    if "shovel" in TQ:
      p, R = tool_pose(Z["qpos"][j], "shovel")
      a = p - R[:, 0] * Q4.S_LO / 2; b = p + R[:, 0] * (Q4.S_TIP - Q4.S_LO / 2 - 0.12)
      a0 = a - R[:, 0] * 0.06
      for k in range(16): S.add(frustum(a0 + (b - a0) * k / 16, a0 + (b - a0) * (k + 1) / 16, 0.014, 0.014, WOOD, caps=k in (0, 15)), same=k > 0)
      S.add(box(b + R[:, 0] * 0.06, R, [0.13, 0.11, 0.004], STEEL))
      if Z["scoop"][j]: S.add(sphere(b + R[:, 0] * 0.07 + np.array([0, 0, 0.02]), 0.04, SAND))
    if "pan" in TQ:
      p, R = tool_pose(Z["qpos"][j], "pan")
      S.add(frustum(p - R[:, 2] * 0.015, p + R[:, 2] * 0.015, 0.09, 0.10, PAN))
      S.add(frustum(p - R[:, 0] * 0.09, p - R[:, 0] * 0.11, 0.01, 0.01, PAN))
      p, R = tool_pose(Z["qpos"][j], "brush")
      S.add(box(p - np.array([0, 0, 0.02]), R, [0.06, 0.04, 0.03], (0.85, 0.75, 0.2)))
    if "shovel" in TQ: S.add(box(np.array([Q.sand_xy[0], Q.sand_xy[1], Q.sand_z / 2]), np.eye(3), [0.5, 0.5, Q.sand_z], SAND))
    return S, st


def panel(Z, k, j):
    S, st = scene(Z, j)
    p = Z["com"][j]; tgt = np.array([p[0], p[1], p[2] - 0.15])
    az = np.radians(200 + 2.0 * Z["t"][j])
    cam = Camera(tgt + np.array([np.cos(az) * 2.2, np.sin(az) * 2.2, 0.55]), tgt, fov_deg=44)
    img = render(S.prims, cam, W, H, ss=2, ground=(p[0], p[1]), fog=(4.5, 13.0))
    if "fold" in Z.files and len(Zs) == 1:                 # close-up of the lower hand's grasp
        Rt, pt = C.arm_fk(Q4.WORK[-1], st[2][Q4.WORK[-1]], st[0], st[1], clutch=st[3][Q4.WORK[-1]])[1]
        c = pt + Rt[:, 0] * 0.115
        if "shovel" in TQ:                                 # look along the shaft, from between the hands
            ex = tool_pose(Z["qpos"][j], "shovel")[1][:, 0]; sd = np.cross(ex, [0, 0, 1.0]); sd /= np.linalg.norm(sd)
            e = c - 0.24 * ex + 0.07 * sd + 0.05 * np.array([0, 0, 1.0])
        else:
            e = c + 0.30 * (0.55 * Rt[:, 0] + 0.75 * Rt[:, 1] + 0.35 * np.array([0, 0, 1.0]))
        ins = render(S.prims, Camera(e, c, fov_deg=40), 560, 420, ss=2, ground=(p[0], p[1]), fog=(4.5, 13.0))
        img = img.copy(); img[310:730, W - 580:W - 20] = ins; img[306:310, W - 584:W - 16] = 30; img[730:734, W - 584:W - 16] = 30
        img[306:734, W - 584:W - 580] = 30; img[306:734, W - 20:W - 16] = 30
    im = Image.fromarray(img); d = ImageDraw.Draw(im, "RGBA")
    if "fold" in Z.files and len(Zs) == 1:
        d.rectangle([W - 584, 734, W - 16, 770], fill=(255, 255, 255, 215))
        d.text((W - 574, 740), "lower hand, seen along the shaft (knuckle angles from the sim)", font=V.F_S, fill=(20, 20, 20))
    r = J[k]["result"]
    title = ("BEFORE training (hand ansatz)" if k == 0 else "AFTER training") if len(Zs) == 2 else "HAND WORK MODE (6 fly, 2 hands work)"
    d.rectangle([0, 0, W, 300], fill=(255, 255, 255, 205))
    d.text((18, 12), title, font=V.F_B, fill=(20, 20, 20))
    d.text((18, 52), "Flying: arms " + " ".join(map(str, Q4.FLY)) + " (7.5 in claw-props) + EDF", font=V.F_S, fill=(40, 40, 40))
    d.text((18, 76), "Working: hand3 on arms 2 + 6, rotors parked, fingers wrapped round the " + ("shovel shaft" if Q4.TASK == "shovel" else Q4.TASK) + " (contact + friction only, no ties)", font=V.F_S, fill=(40, 40, 40))
    exy = np.linalg.norm(Z["com"][j][:2] - Z["xyref"][j]) * 100
    rows = [f"t {Z['t'][j]:5.1f} s   position error {exy:4.1f} cm   tilt {Z['tilt'][j]:4.1f} deg   yaw {Z['yaw'][j]:5.1f} deg",
            f"battery 2 x 6S: {Z['V'][j]:4.1f} V  {Z['I'][j]:4.0f} A   claw-prop ceiling {Z['fmax'][j]:4.1f} N",
            "claw-props: " + "  ".join(f"{Z['fan'][j][i]:4.1f}" for i in Q4.FLY) + f" N   EDF {Z['edf'][j]:4.1f} N",
            "worst servo per arm: " + " ".join(f"{v:.2f}" for v in Z["servo_arm"][j]),
            f"whole run: error mean {r['exy_mean']:.1f} / max {r['exy_max']:.1f} cm, tilt max {r['tilt_max']:.1f} deg, servo max {r['servo']:.2f}"]
    for n, s in enumerate(rows):
        d.text((18, 108 + 36 * n), s, font=V.F_S, fill=(30, 30, 30))
    return np.asarray(im)


def frame(j):
    ims = [panel(Z, k, j) for k, Z in enumerate(Zs)]
    im = np.concatenate(ims, 1)
    if len(ims) == 2: im[:, W - 2:W + 2] = 30
    pil = Image.fromarray(im); d = ImageDraw.Draw(pil, "RGBA")
    d.rectangle([0, H - 36, im.shape[1], H], fill=(20, 22, 26, 200))
    d.text((14, H - 30), "MuJoCo full-robot sim: gravity, battery sag, prop torque. Shovel held only by finger contact and friction (knuckle "
           "servos within rated torque). Sand forces scripted. Masses and thrust are estimates. Real time.",
           font=V.F_S, fill=(230, 230, 230))
    return np.asarray(pil)


if __name__ == "__main__":
    if sys.argv[1] == "still":
        j = int(np.argmin(np.abs(Zs[0]["t"] - float(sys.argv[2])))); Image.fromarray(frame(j)).save(f"qstill_{sys.argv[2]}.png")
    elif sys.argv[1] == "chunk":
        ci, n = int(sys.argv[2]), int(sys.argv[3]); idx = np.array_split(np.arange(N), n)[ci]
        out = f"qchunk_{ci:02d}.part.mp4"; WW = W * len(Zs)
        ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{WW}x{H}", "-r", "30", "-i", "-",
                               "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
        for j in idx: ff.stdin.write(frame(j).tobytes())
        ff.stdin.close()
        if ff.wait() == 0: os.replace(out, f"qchunk_{ci:02d}.mp4")
    else:
        n = int(sys.argv[2])
        open("qchunks.txt", "w").write("".join(f"file 'qchunk_{i:02d}.mp4'\n" for i in range(n)))
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "qchunks.txt", "-c", "copy",
                        os.environ.get("QV_OUT", "cephalo1_quad_mode_trained.mp4")], check=True)
