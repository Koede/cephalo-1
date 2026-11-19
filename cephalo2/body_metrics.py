"""Fluidity and load metrics for a body4 session (python3 body_metrics.py body4_mid_day.npz)."""
import sys, json, numpy as np
import states4 as ST

f = sys.argv[1] if len(sys.argv) > 1 else "body4_mid_day.npz"
Z = np.load(f)
t = Z["t"]; sk = Z["skill"]; lab = Z["label"]
dt = np.gradient(t)
qd = np.degrees(Z["qdcmd"]).reshape(len(t), -1)
qdd = np.gradient(qd, t, axis=0)
active = ~np.isin(sk, ["rest"])
moving = (np.abs(qd) > 10).sum(1)
still = (np.abs(qd).max(1) < 2.0) & active & ~np.isin(sk, ["idle"])
# skill boundaries: command velocity change across the switch
bnd = [j for j in range(1, len(t)) if sk[j] != sk[j - 1]]
jump_b = max(float(np.abs(qd[j] - qd[j - 1]).max()) for j in bnd)
jump_all = float(np.abs(np.diff(qd, axis=0)).max())
raw = float(np.degrees(Z["gjump"].max()))
k = np.argmax(Z["gjump"])
air = Z["air"] > 0.99
com = Z["com"]; ref = Z["xyref"]
exy = np.linalg.norm(com[:, :2] - ref, axis=1) * 100
fly = air & np.isin(sk, ["fly", "idle", "take off"])
w, at = ST.check(f)
srv = Z["servo_arm"]
res = dict(
    duration=float(t[-1]), skills=int(len(bnd) + 1),
    joints_moving_mean=float(moving[active & ~np.isin(sk, ["idle"])].mean()), joints_moving_max=int(moving.max()),
    still_time=float((still * dt).sum()),
    vel_jump_at_switch=jump_b, vel_jump_any=jump_all, raw_goal_step_max=raw, raw_goal_step_at=f"{t[k]:.2f} {sk[k]}",
    peak_cmd_speed=float(np.abs(qd).max()), peak_cmd_acc=float(np.abs(qdd).max()),
    servo_max=float(srv.max()), servo_at=f"{t[np.unravel_index(srv.argmax(), srv.shape)[0]]:.2f} arm {np.unravel_index(srv.argmax(), srv.shape)[1]}",
    slip_max=float(Z["slip_arm"].max()), tilt_max=float(Z["tilt"].max()),
    fan_max=float(Z["fan"].max()), sat_time=float(((Z["sat"] > 0) * dt).sum()),
    governor_time=float(((Z["rate"] < 0.99) * dt).sum()), governor_min=float(Z["rate"].min()),
    flight_err_max=float(exy[fly].max()) if fly.any() else 0.0,
    clear_nb=float(w["nb"] * 1000), clear_body=float(w["body"] * 1000), clear_floor=float(w["floor"] * 1000))
for k2, v in res.items():
    print(f"{k2:22s} {v}")
json.dump(res, open(f.replace(".npz", "_metrics.json"), "w"), indent=1)
