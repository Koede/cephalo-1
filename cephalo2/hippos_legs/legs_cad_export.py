#!/usr/bin/env python3
"""Rev B export: motions closed on the CAD kinematics (legs_motion), check results (legs_cad_verify.json,
legs_motion_test.json) and the viewer data (parts in rest coords, joint tree, struts, toggles, per-frame joint values).
No pod covers.    python3 legs_cad_export.py -> viewer/legs_data.js, legs_cad_summary.json"""
import os, json
import numpy as np
import hippos_legs_cad as C
import legs_motion as M
HERE = M.HERE; S = M.S
mot, ev, crouch = M.build()
ver = json.load(open(os.path.join(HERE, "legs_cad_verify.json"))); tst = json.load(open(os.path.join(HERE, "legs_motion_test.json")))
frames = {}; dts = {}
for name, m in mot.items():
    fr = m["frames"]; step = 2 if len(fr) > 1 else 1; dts[name] = m["dt"] * step
    frames[name] = [dict(b=[round(f["b"][0], 1), round(f["b"][1], 1), round(f["b"][2], 2)],
                         l=[[round(l["q0"], 2), round(l["q1"], 2), round(l["fold"], 2), int(l["st"]), round(l["hp"], 2)] for l in f["l"]]) for f in fr[::step]]
mo = tst["motions"]
mn = lambda key: min(mo[k]["distance"][key][0] for k in mo)
summ = dict(rev="B", mass_total=ev["total"], four_legs=M.CADM["four_legs"], acts=M.ACTS, jump_mm=ev["jump_mm"], worst=ev["worst"], fails=ev["fails"],
            limits=M.LIM, lower=M.LO, crouch=crouch, rule_mm=ver["rule_mm"],
            poses_checked={k: ver["result"][k]["poses"] for k in ver["result"]}, tightest={k: ver["result"][k]["tightest"] for k in ver["result"]},
            under_rule={k: ver["result"][k]["under_rule"] for k in ver["result"]},
            in_leg_motion=mn("in_leg"), leg_leg_motion=mn("leg_leg"), leg_leg_twisted_gallop=tst["gallop_twisted_leg_leg"][0],
            neck=min(mn("neck"), tst["stand_twist"]["neck"][0]), octo=min(mn("octo"), tst["stand_twist"]["octo"][0]),
            slip_mm=max(mo[k]["slip_mm"] for k in mo), speed_margin=min(v for k in mo for v in mo[k]["speed_margin"].values()),
            roll={k: mo[k]["distance"]["roll_deg"] for k in mo}, gas_len=[min(mo[k]["mechanism"]["gas"][0] for k in mo), max(mo[k]["mechanism"]["gas"][1] for k in mo)],
            rod_angle=min(mo[k]["mechanism"]["rod_angle"] for k in mo), forces=tst["forces"], carpus_servo=M.CARPUS_SERVO, dt=dts,
            belt=C.FORE.mech["belt"], yaw_centres=C.YAW_C, gas=dict(fore=C.FORE.mech["gas"], hind=C.HIND.mech["gas"]),
            shell_overlaps=sorted({d["kind"] + ": " + d["part"] for d in tst["shell_overlaps"]}))
json.dump(summ, open(os.path.join(HERE, "legs_cad_summary.json"), "w"), indent=1, default=float)

def pack(m):
    V, F = C.mesh_of(m)
    return dict(v=np.round(V, 2).ravel().tolist(), f=F.ravel().tolist())
def end(e):
    return [e[0], e[1], [float(x) for x in e[2]], float(e[3]), e[4]] if e[0] == "tan" else ([e[0], int(e[1]), float(e[2])] if e[0] == "knee" else [e[0], [float(x) for x in e[1]]])
data = dict(rev="B", kinds={}, frames=frames, dt=dts, stand=C.STAND, limits=M.LIM, summary=summ)
for kind, L in (("fore", C.FORE), ("hind", C.HIND)):
    data["kinds"][kind] = dict(pivot=C.LEG[kind]["pivot"].tolist(), delta=C.LEG[kind]["delta"],
                               nodes={n: dict(parent=v[0], o=v[1], axis=v[2], coef=v[3], const=v[4]) for n, v in C.TREE[kind].items()},
                               parts=[dict(name=p["name"], node=p["node"], group=p["group"], col=p["col"], **pack(p["solid"])) for p in L.parts],
                               struts=[dict(name=s["name"], a=end(s["a"]), b=end(s["b"]), r=s["r"], col=s["col"], group=s.get("group", "drive")) for s in L.struts],
                               toggles=[dict(name=t["name"], a=[t["a"][0], t["a"][1]], b=[t["b"][0], t["b"][1]], l=t["l"], r=t["r"], col=t["col"],
                                             y1=sum(t["y1"]) / 2, y2=sum(t["y2"]) / 2, lock_below=t["lock_below"]) for t in L.toggles])
Pg = M.C4.shell_points() * 1e3; nx, na = Pg.shape[:2]
F = []
for i in range(nx - 1):
    for k in range(na - 1):
        a, b, c_, d = i * na + k, i * na + k + 1, (i + 1) * na + k + 1, (i + 1) * na + k
        F += [a, b, c_, a, c_, d]
data["shell"] = dict(v=np.round(Pg.reshape(-1, 3), 1).ravel().tolist(), f=F)
data["octo"] = dict(c=((np.array(M.C3.cc) + np.array(M.C3.n_up) * 0.10) * 1e3).tolist(), r=120.0)
data["ref"] = [S.X_COM * 1e3, 300.0]
os.makedirs(os.path.join(HERE, "viewer"), exist_ok=True)
js = "window.LEGS = " + json.dumps(data, separators=(",", ":"), default=float) + ";"
open(os.path.join(HERE, "viewer", "legs_data.js"), "w").write(js)
print("legs_data.js", round(len(js) / 1e6, 2), "MB"); print(json.dumps({k: summ[k] for k in ("mass_total", "four_legs", "jump_mm", "worst", "crouch", "tightest", "in_leg_motion", "leg_leg_motion", "leg_leg_twisted_gallop", "neck", "octo", "slip_mm", "speed_margin", "roll", "gas_len", "rod_angle", "dt")}, default=float)[:1800])
