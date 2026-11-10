"""Hover pose when the fan may lean off vertical: worst need vs allowed lean (fan at least R_TIP from the robot centre)."""
import sys, os, json, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np, spec_lib as S, poses as PS
A = S.Arm("alt90"); T = float(sys.argv[2]) if len(sys.argv) > 2 else 21.53; out = []
for lean in [float(x) for x in sys.argv[1].split(",")]:
    for rt in (0.60,):
        def load(q, f): return (T * f["tipR"][:, 2], "thrust", None)
        def cons(q, f):
            z = f["tipR"][:, 2]; tip = f["tip"]
            return [], [float(z[2]) - math.cos(math.radians(lean)), float(np.hypot(tip[0], tip[1])) - rt, S.self_clear(f)]
        rng = np.random.default_rng(5); base = np.zeros(S.N); base[0] = 0.5; base[2::2] = 0.13
        hook = np.zeros(S.N); hook[0] = math.radians(45); hook[2] = math.radians(20); hook[4] = math.radians(-10); hook[6] = math.radians(-20); hook[8] = math.radians(-20)
        lean0 = np.zeros(S.N); lean0[0] = math.radians(45); lean0[2] = math.radians(20)
        x0s = [base, hook, lean0] + [rng.uniform(-1, 1, S.N) * PS.LIM for _ in range(int(os.environ.get("NSEED", 45)))]
        try:
            P0 = dict(np.load(os.path.join(S.HERE, "results", "poses_alt90.npz"))); x0s.append(P0["alt90_flight_hover"])
        except Exception: pass
        r = PS.solve(A, load, cons, x0s)
        f = A.fk(r["q"]); z = f["tipR"][:, 2]
        row = dict(lean_allowed_deg=lean, fan_radius_min_m=rt, ok=r["ok"], worst=round(r["worst"], 3), need=[round(float(x), 2) for x in r["need"]],
                   lean_used_deg=round(math.degrees(math.acos(min(1, z[2]))), 1), lift_lost=round(1 - float(z[2]), 3),
                   fan_xy_m=[round(float(x), 3) for x in f["tip"][:2]], q_deg=[round(math.degrees(x), 1) for x in r["q"]])
        out.append(row); print(row, flush=True)
json.dump(out, open(os.path.join(S.HERE, "results", f"hover_tilt_{sys.argv[1].replace(',', '_')}_T{round(T)}.json"), "w"), indent=1)
