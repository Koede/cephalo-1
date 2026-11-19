"""Can 4 claw-props + EDF hold Cephalo-1 with shovel + pan in flight, under battery sag? (python3 quad_sizing.py)"""
import numpy as np, power4 as PW, cephalo4 as C

TOOLS = 0.40 + 0.20 + 0.25 + 0.05          # shovel, scoop of sand, pan, dish brush (kg)


def solve(prop, packs, soc, n_fly, payload=0.0, servo_load=0.3):
    p = PW.PROPS[prop]
    m = C.M_ROBOT + 8 * p["dM_pod"] + (packs - 1) * PW.M_PACK + payload
    W = m * C.G
    b = PW.Battery(packs, soc)
    e, f = C.EDF_MAX, (W - C.EDF_MAX) / n_fly
    for _ in range(80):
        P = PW.EDF_K * e ** 1.5 + p["K"] * n_fly * f ** 1.5 + 12 * 64 * (0.05 + 2.7 * servo_load) / 0.9 + 10
        E = b.ocv(); d = E * E - 4 * P * b.R
        V = (E + np.sqrt(max(d, 0))) / 2
        e = min(C.EDF_MAX, PW.EDF_T_REF * (V / 25.2) ** PW.EDF_EXP)
        f = (W - e) / n_fly
    fmax = PW.claw_max(prop, V)
    return dict(m=m, V=V, I=P / V, f=f, fmax=fmax, frac=f / fmax, tw=(e + n_fly * fmax) / W)


if __name__ == "__main__":
    print(f"{'prop':4s} {'packs':5s} {'SoC':4s} | {'mass':>6s} | quad mode + tools: {'V':>5s} {'A':>4s} {'N/prop':>6s} {'of max':>6s} | 8 props T/W | 6 props T/W")
    for prop in ("std", "xl"):
        for packs in (1, 2):
            for soc in (1.0, 0.6, 0.4):
                q = solve(prop, packs, soc, 4, TOOLS); a8 = solve(prop, packs, soc, 8); a6 = solve(prop, packs, soc, 6)
                print(f"{prop:4s} {packs:5d} {soc:4.1f} | {q['m']:6.2f} | {q['V']:23.1f} {q['I']:4.0f} {q['f']:6.1f} {q['frac'] * 100:5.0f}% | "
                      f"{a8['tw']:11.2f} | {a6['tw']:11.2f}")
