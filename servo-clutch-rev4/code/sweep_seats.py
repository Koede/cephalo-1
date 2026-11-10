"""Rigid arm vs breakaway seats at three winch tensions: side hits, fan run-away pull, side stiffness."""
import json, numpy as np
from sim_seats import SeatSim, MODES

CASES = [("rigid", False, 0.0)] + [(k, True, v) for k, v in MODES.items()]
out = {}


def summarise(s, t0):
    T = np.array([t for t, _, _ in s.log]); L = [l for _, l, _ in s.log]
    servo = np.array([[l[k] for k in range(9)] for l in L])
    sat = servo.max(0)
    qdev = np.array([l["qdev"] for l in L]); tilt = np.array([l["tilt"] for l in L])
    late = np.where((T > t0 + 0.05) & ((qdev > 3) | (tilt > 5)))[0]
    gear = np.array([[l[f"g{k}"] for k in range(9)] for l in L]).max(0)
    return dict(gear_peak=gear.round(2).tolist(), gear_worst=float(gear.max()), servo_peak=sat.round(2).tolist(), worst_servo=int(sat.argmax()), worst_ratio=float(sat.max()),
                sat_joints=[int(k) for k in np.where(sat > 0.99)[0]],
                qdev_max=float(qdev.max()), recover_s=float(T[late[-1]] - t0) if len(late) else 0.0,
                stuck=bool(qdev[-1] > 10), open_max=float(max(l["open_deg"] for l in L)),
                ext_max=float(max(l["ext_mm"] for l in L)), T_max=float(max(l["T"] for l in L)),
                impact=float(max(l["impact_N"] for l in L)))


if __name__ == "__main__":
  pass
for v in (4.0, 6.0, 8.0) if __name__ == "__main__" else ():
    for name, seats, T0 in CASES:
        s = SeatSim(seats, T0, imp_v=v).run(3.0, rec=0.001)
        r = summarise(s, 0.5); out[f"hit{v:.0f}_{name}"] = r
        print(f"hit {v} {name:5s}", {k: (round(x, 2) if isinstance(x, float) else x) for k, x in r.items() if k != "servo_peak"})

for F in (20.0, 40.0) if __name__ == "__main__" else ():
    for name, seats, T0 in CASES:
        s = SeatSim(seats, T0, yank=(0.5, F, 0.3)).run(2.0, rec=0.001)
        r = summarise(s, 0.5); out[f"pull{F:.0f}_{name}"] = r
        print(f"pull {F} {name:5s}", {k: (round(x, 2) if isinstance(x, float) else x) for k, x in r.items() if k != "servo_peak"})

for name, seats, T0 in (CASES if __name__ == "__main__" else []):      # stiffness: steady 3 N sideways on the pod
    base = SeatSim(seats, T0).run(1.0, rec=0.05)
    s = SeatSim(seats, T0, side=np.array([-np.sin(np.radians(22.5)), np.cos(np.radians(22.5)), 0]) * 3.0).run(1.5, rec=0.05)
    p0 = base.d.xpos[base.b_pod]; p1 = s.d.xpos[s.b_pod]
    out[f"stiff_{name}"] = dict(defl_mm=float(1000 * np.linalg.norm(p1 - p0)), open=float(s.log[-1][1]["open_deg"]))
    print("stiff", name, round(1000 * np.linalg.norm(p1 - p0), 1), "mm", "seat open", round(s.log[-1][1]["open_deg"], 2))

if __name__ == "__main__":
    json.dump(out, open("seat_results.json", "w"), indent=1)
