"""hand3 numbers the full-robot sims use, read from the design files (hand3/hand3_design.json, hand3/hand3_thrust_cad.json)
so the drivers (hand3_fly.py, quad_h3.py) cannot drift from the design.  Rev D rotor (9 in) with hub C (two rings, two servos) when hand3_hubc.json exists."""
import os, json
_H = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.environ.get("H3_DIR", "hand3e"))
D = json.load(open(os.path.join(_H, "hand3_design.json"))); TC = json.load(open(os.path.join(_H, "hand3_thrust_cad.json")))
try: HUBC = json.load(open(os.path.join(_H, "hand3_hubc.json")))                # hub C (two rings, 2 x STS3215) replaces the rev D drum when present
except Exception: HUBC = None
_M = HUBC if (HUBC and os.environ.get("H3_HUB", "C") == "C") else D
M_HAND = float(os.environ.get("H3_MASS", _M["mass"]["total"] * 1e-3))            # kg
COM = float(os.environ.get("H3_COM_MM", _M["com_from_spigot"])) * 1e-3; BO_H = D["Z"]["blade"] * 1e-3; R_H = D["blade"].get("R_tip", 114.3) * 1e-3
F_ROTOR = (COM - 0.015) / (BO_H - 0.015)                                          # share of the mass lumped at the blade plane (rest: cylinder 0-30 mm)
T = dict(low=TC["low"]["T_max"], high=TC["high"]["T_max"]); T["mid"] = round((T["low"] + T["high"]) / 2, 2); T["none"] = T["mid"]
QT = dict(low=TC["low"]["qT"], high=TC["high"]["qT"]); QT["mid"] = (QT["low"] + QT["high"]) / 2
K_HAND = TC["K_W_per_N15"]; HANDS = (2, 6)
