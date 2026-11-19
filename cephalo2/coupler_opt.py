"""Coupler rev B.2 optimisation: least mass with factor >= FOS on every hand-calc stress in the 3 g wheel landing.
Sweeps lug width and lug outer radius (lug-on-lip radial overlap kept >= 1.5 mm); for each, thicknesses (lug, lip, wall, step) are raised in 0.1 steps until
their own stress passes; mass comes from the CAD solids.  python3 coupler_opt.py -> coupler_opt.json"""
import json, itertools
import numpy as np
import handmount_b as HB
P = HB.P
FOS = 2.0; TMIN = dict(t_lug=2.4, t_lip=2.4, t_wall=1.6, t_step=2.4)
case = json.load(open("wheelhand_sizing.json"))["connector"]["3 g landing on the wheels (est.)"]
KEY = dict(t_lug=("lug_bending_MPa", "lug_shear_MPa"), t_lip=("lip_bending_MPa",), t_wall=("collar_wall_MPa",), t_step=("collar_step_MPa",))


def size(lug_w, lug_r1):
    P.update(lug_w=lug_w, lug_r1=lug_r1, **TMIN); HB.derive()
    for _ in range(200):
        s = HB.stresses(case["bending_Nm"], case["radial_N"]); ok = True
        for t, ks in KEY.items():
            if max(s[k] for k in ks) > 30.0 / FOS: P[t] = round(P[t] + 0.1, 1); ok = False
        HB.derive()
        if ok: break
    return s


def mass():
    v = dict(socket=HB.arm_socket(), nose=HB.nose_ring(), collar=HB.collar(0), hand_base=HB.hand_base())
    return {k: round(m.volume() / 1000 * 1.15, 2) for k, m in v.items()}


if __name__ == "__main__":
    rows = []
    for lw, r1 in itertools.product(np.arange(36.0, 49.1, 1.0), np.arange(22.4, 23.45, 0.1)):
        s = size(float(lw), round(float(r1), 1)); m = mass()
        rows.append(dict(lug_w=float(lw), lug_r1=round(float(r1), 1), t={k: P[k] for k in TMIN}, mass=m, total=round(sum(m.values()), 2),
                         R_collar=round(P["R_collar"], 2), fos=s["factor_vs_PA_CF_interlayer_30MPa"], bear=s["lug_on_lip_bearing_MPa"]))
    rows.sort(key=lambda r: r["total"])
    for r in rows[:6] + rows[-2:]: print(r)
    json.dump(dict(fos_target=FOS, best=rows[0], top=rows[:10], n=len(rows)), open("coupler_opt.json", "w"), indent=1)
