"""Torque-speed check of the v2 leg actuators over the gait cycles (est. DC-motor line from the listings)."""
import json, numpy as np, contextlib, io
with contextlib.redirect_stdout(io.StringIO()):
    import legs_v2_study as LS
B = LS.ARCH["B"]; inr = LS.inertia_of("B")
# (no-load rpm at 48 V, Kt N m/Arms, line R ohm): RS06 from its listing (Kt derived from 14.3 Apk at 11 N m, R est.), RS03 from its listing
MOT = {"RobStride 06": (480.0, 1.09, 0.45), "RobStride 03": (195.0, 2.36, 0.39)}
V = 39.6                                     # 12S at 3.3 V/cell, worst case
def avail_rpm(name, T):
    n0, kt, r = MOT[name]; ke = 48.0 / n0
    return n0 * V / 48.0 - (abs(T) / kt) * r / ke
out = {}
for leg, hip in (("front", "RobStride 03"), ("rear", "RobStride 06")):
    for gait in ("walk", "trot", "fast trot"):
        c = LS.m2_cycle(B, leg, gait, inr)
        tau, qd = np.abs(c["tau"]), np.abs(c["qd"])
        res = {}
        for j, (name, ratio, eff) in enumerate(((hip, 1.0, 1.0), ("RobStride 06", LS.KNEE_RATIO, LS.BELT_EFF))):
            T_act = tau[:, j] / ratio / eff; n_act = qd[:, j] * ratio * 30 / np.pi
            margin = np.min([avail_rpm(name, t) / n if n > 1 else 99 for t, n in zip(T_act, n_act)])
            k = np.argmax(n_act); res["hip" if j == 0 else "knee"] = dict(actuator=name, max_rpm=float(n_act.max()), T_at_max=float(T_act[k]),
                                                                       avail_at_max=float(avail_rpm(name, T_act[k])), worst_margin=float(margin))
        out[f"{leg} {gait}"] = res
json.dump(out, open("ts_check.json", "w"), indent=1)
for k, v in out.items():
    print(k.ljust(16), "  ".join(f"{j} {r['actuator'][-2:]}: {r['max_rpm']:.0f} rpm at {r['T_at_max']:.1f} N m, available {r['avail_at_max']:.0f}, worst x{r['worst_margin']:.2f}" for j, r in v.items()))
