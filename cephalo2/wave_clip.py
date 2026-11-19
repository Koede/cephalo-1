import sys, subprocess, os, numpy as np
sys.argv = ["x", "body4_mid_day.npz"]
import body_video as BV
idx = np.where(BV.Z["sidx"] == 4)[0]
ci, n = int(os.environ["CI"]), int(os.environ["N"])
part = np.array_split(idx, n)[ci]
ff = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{BV.W}x{BV.H}", "-r", "30", "-i", "-",
                       "-c:v", "libx264", "-crf", "18", "-preset", "veryfast", "-pix_fmt", "yuv420p", f"wchunk_{ci}.mp4"], stdin=subprocess.PIPE)
for j in part: ff.stdin.write(BV.frame(j).tobytes())
ff.stdin.close(); ff.wait()
