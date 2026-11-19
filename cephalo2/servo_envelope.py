"""Check every joint motion of a body4 session against the real servo torque-speed line (python3 servo_envelope.py body4_mid_day.npz).
Available speed at a load = no-load speed x (1 - torque / stall) (linear DC-motor line, 12 V datasheet points):
STS3250 (j0) 0.133 s/60 deg = 451 deg/s, 4.9 N m stall; STS3215 (j1-j7) 0.222 s/60 deg = 270 deg/s, 2.94 N m stall.
Demand = max(|commanded speed|, |actual joint speed in the sim|) - the sim servo has a torque limit but no speed limit."""
import sys, json, numpy as np
f = sys.argv[1] if len(sys.argv) > 1 else "body4_mid_day.npz"
Z = np.load(f); t = Z["t"]; sk = Z["skill"]; sid = Z["sidx"]
W0 = np.array([451.0] + [270.0] * 7)
load = np.clip(Z["load"], 0, 1)
need = np.maximum(np.abs(np.degrees(Z["qdcmd"])), np.abs(np.degrees(Z["qdm"])))
avail = W0[None, None, :] * (1 - load)
ratio = need / np.maximum(avail, 1e-6)
ratio[need < 5] = 0                      # holding still: no speed demand
res = {}
for s in np.unique(sid):
    w = sid == s
    r = ratio[w]; k = np.unravel_index(r.argmax(), r.shape)
    res[f"{s:02d} {sk[w][0]}"] = dict(max_ratio=float(r.max()), at=f"t {t[w][k[0]]:.2f} arm {k[1]} j{k[2]}",
                                     need=float(need[w][k]), load=float(load[w][k]), frames_over_0p8=int((r > 0.8).any(axis=(1, 2)).sum()))
for k, v in res.items():
    print(f"{k:12s} worst demand/available {v['max_ratio']:.2f} at {v['at']} ({v['need']:.0f} deg/s at {v['load']:.2f} x stall), frames > 0.8: {v['frames_over_0p8']}")
json.dump(res, open(f.replace(".npz", "_envelope.json"), "w"), indent=1)
