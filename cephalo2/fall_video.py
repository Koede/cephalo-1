"""Video of the fall-and-recover runs (python3 fall_video.py still <case> <t> | chunk <i> <n> | join <n>)
-> cephalo1_falls_reinforced.mp4 (FV_OUT to rename): a 2.5 s card with rideover_cams.png (rideover_parts.py), then the
cases in order: cut, hit, inv, drift (fall4_<case>.npz/.json)."""
import os, sys, subprocess, json, numpy as np
os.environ.setdefault("CEPH_PROP", "xl"); os.environ.setdefault("CEPH_PACKS", "1"); os.environ.setdefault("CEPH_J1", "STS3250")
if os.environ.get("FALL_HW", "reinforced") == "reinforced": os.environ["CEPH_J1CLUTCH"] = "0.9"   # same joint layout as the sim: states4 builds its own model
from PIL import Image, ImageDraw
import cephalo4 as C
import power4 as PW
import xlpod4; xlpod4.install()
from render import Camera, render
import viz4 as V
import states4 as ST

ALL = ["cut", "hit", "inv", "drift"]
CASES = os.environ.get("FV_CASES", ",".join(ALL)).split(",")     # FV_CASES=hit,inv FV_INTRO=0 FV_OUT=... for a subset
W, H = 1600, 900
T_SKIP = 0.5                       # start each case 0.5 s before the fault
OUT = os.environ.get("FV_OUT", "cephalo1_falls_reinforced.mp4")
INTRO = int(os.environ.get("FV_INTRO", 75))   # frames of the cam card before the falls (2.5 s)
SRC = {c: os.environ.get(f"FV_FILE_{c}", f"fall4_{c}") for c in CASES}     # FV_FILE_hit=fall4_hit_ts30_0 shows another run of that case
Z = {c: np.load(SRC[c] + ".npz") for c in CASES}
J = {c: json.load(open(SRC[c] + ".json")) for c in CASES}
for c in CASES: assert Z[c]["qpos"].shape[1] == ST._M.nq, "fall_video: joint layout differs from the sim that wrote fall4_%s.npz" % c
FAULT = {"cut": 1.0, "hit": 1.0, "inv": 1.0, "drift": 1.6}
TITLE = {"cut": "1 / 4   Motor cut-out at 1.5 m, upright",
         "hit": "2 / 4   Side blow at 1.5 m (36 N m for 0.25 s), ESC failsafe at 45 deg tilt",
         "inv": "3 / 4   Motor cut-out at 1.2 m while upside down",
         "drift": "4 / 4   Motor cut-out flying sideways at 2.5 m/s, 1.0 m up"}
TITLE = {c: f"{CASES.index(c) + 1} / {len(CASES)}" + os.environ.get(f"FV_TITLE_{c}", TITLE[c][5:]) for c in CASES}
SEQ = [("intro", 0, n) for n in range(INTRO)]   # (case, frame index, frames since case start)
for c in CASES:
    t = Z[c]["t"]; idx = np.where(t >= FAULT[c] - T_SKIP)[0]
    for n, j in enumerate(idx): SEQ.append((c, j, n))
N = len(SEQ)


def cam_target(z, j):
    k = np.arange(max(0, j - 15), min(len(z["t"]), j + 16)); p = z["com"][k].mean(0)
    return np.array([p[0], p[1], max(p[2] - 0.05, 0.22)])


_CARD = None
def intro_card():
    global _CARD
    if _CARD is None:
        im = Image.new("RGB", (W, H), (245, 245, 242)); d = ImageDraw.Draw(im)
        d.text((40, 30), "CEPHALO-1  |  reinforced for falls", font=V.F_B, fill=(30, 30, 30))
        lines = ["Ride-over clutch cams (a V every 90 deg, 60 deg at the root): past the crest the roller drops into the next V,",
                 "so a slammed link can never hit a stop that feeds its momentum into the servo gears.",
                 "+ a clutch at j1, TPU pads on the link stops, a Hall switch per clutch, and a re-index move after the fall."]
        for k, l in enumerate(lines): d.text((40, 80 + 30 * k), l, font=V.F_S, fill=(50, 50, 50))
        fig = Image.open("rideover_cams.png").convert("RGB"); fig = fig.resize((W - 80, int(fig.height * (W - 80) / fig.width)))
        im.paste(fig, (40, 190)); _CARD = np.asarray(im)
    return _CARD


def frame(s):
    c, j, n = SEQ[s]
    if c == "intro": return intro_card()
    z = Z[c]; t = z["t"][j]
    st = ST.state(z["qpos"][j]); fan = z["fan"][j]
    fold = np.where(fan > 1, 0.0, np.radians(60))
    spin = np.cumsum(np.sqrt(np.clip(z["fan"][:j + 1], 0, None)), 0)[-1] * 9.0 / 30
    S = V.scene_from_state(st, t, spin, fold, fan, z["edf"][j])
    tgt = cam_target(z, j)
    az = np.radians(235 + 4.0 * t + 50 * ALL.index(c))
    dist = 1.9 + 0.7 * float(np.clip((tgt[2] - 0.25) / 0.8, 0, 1))
    cam = Camera(tgt + np.array([np.cos(az) * dist, np.sin(az) * dist, 0.35 * dist]), tgt, fov_deg=46)
    img = render(S.prims, cam, W, H, ss=2, ground=(tgt[0], tgt[1]), fog=(5.0, 14.0))
    im = Image.fromarray(img); d = ImageDraw.Draw(im, "RGBA")
    d.rectangle([0, 0, W, 58], fill=(20, 22, 26, 205))
    d.text((20, 12), "CEPHALO-1  |  " + TITLE[c], font=V.F_B, fill=(240, 240, 240))
    d.rectangle([16, 72, 1010, 300], fill=(255, 255, 255, 205), outline=(60, 60, 60, 255))
    y = 82
    d.text((30, y), str(z["label"][j]), font=V.F_M, fill=(20, 20, 20)); y += 40
    jj = J[c]
    rows = [f"t {t - FAULT[c]:+5.2f} s from the fault     speed {z['speed'][j]:4.1f} m/s     tilt {z['up'][j]:5.1f} deg",
            f"impact {jj['v_imp']:.1f} m/s   body {jj['decel']:.0f} g (10 ms avg)   clutch slip now {z['slip_arm'][j].max():3.0f} deg   worst gear so far {np.max(z['gear'][:j + 1]):.2f}",
            "claw-props (N): " + " ".join(f"{f:4.1f}" if f > 0.5 else "  - " for f in fan),
            "worst servo per arm: " + " ".join(f"{v:.2f}" for v in z["servo_arm"][j]),
            f"battery {z['V'][j]:4.1f} V {z['I'][j]:4.0f} A    floor force: body {z['bump'][j]:5.0f} N  feet {z['foot'][j].sum():5.0f} N"]
    for r in rows:
        d.text((30, y), r, font=V.F_S, fill=(30, 30, 30)); y += 32
    if n < 24:                                          # title card (first 0.8 s of each case)
        a = int(230 * min(1.0, (24 - n) / 8))
        d.rectangle([W // 2 - 520, H - 170, W // 2 + 520, H - 80], fill=(20, 22, 26, a))
        d.text((W // 2 - 500, H - 140), TITLE[c], font=V.F_B, fill=(240, 240, 240, a))
    d.rectangle([0, H - 36, W, H], fill=(20, 22, 26, 200))
    d.text((12, H - 30), "MuJoCo sim: servo stall limits, ride-over clutches, TPU stop pads, bumper, dome pad, toe caps (est.), battery sag. Gear 1.00 = strip limit (est.). Real time.", font=V.F_S, fill=(230, 230, 230))
    return np.asarray(im)


if __name__ == "__main__":
    if sys.argv[1] == "still":
        c, t = sys.argv[2], float(sys.argv[3])
        s = 0 if c == "intro" else min((k for k in range(N) if SEQ[k][0] == c), key=lambda k: abs(Z[c]["t"][SEQ[k][1]] - t))
        Image.fromarray(frame(s)).save(f"fallstill_{c}_{t}.png")
    elif sys.argv[1] == "chunk":
        ci, n = int(sys.argv[2]), int(sys.argv[3]); idx = np.array_split(np.arange(N), n)[ci]
        out = f"fvchunk_{ci:02d}.part.mp4"
        ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", "30", "-i", "-",
                               "-c:v", "libx264", "-crf", "20", "-preset", "veryfast", "-pix_fmt", "yuv420p", out], stdin=subprocess.PIPE)
        for s in idx: ff.stdin.write(frame(s).tobytes())
        ff.stdin.close()
        if ff.wait() == 0: os.replace(out, f"fvchunk_{ci:02d}.mp4")
    else:
        n = int(sys.argv[2])
        open("fvchunks.txt", "w").write("".join(f"file 'fvchunk_{i:02d}.mp4'\n" for i in range(n)))
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-f", "concat", "-safe", "0", "-i", "fvchunks.txt", "-c", "copy", OUT], check=True)
