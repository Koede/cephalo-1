"""Flight battery sizing for Cephalo-1 with the main arm (1 m, 13 joints, 16 N claw fans, central EDF).

Hover power = 8 claw fans + the central EDF + servos and electronics. The EDF is assumed to run at its full 42.2 N
(as in Forest's take-off sequence), so any added weight is carried by the claw fans, whose power scales as thrust^1.5
from the T-Motor 2807 1250KV / 7.5 in bench point (13.2 N at about 270 W, 21.4 N at 551 W). Writes results/battery.json.

  python3 battery.py
"""
import os, json, math
HERE = os.path.dirname(os.path.abspath(__file__))
G = 9.81
ROBOT_KG = 15.08            # main design with the geared root, no battery added (the 3.45 kg body allowance came from the 8.63 kg robot)
EDF_N = 42.2                # [F]
EDF_G_PER_W = 1.45          # [A] Dr. Mad Thrust 90 mm 12-blade 6S: 3.40 kg at 2350 W max (HobbyKing listing); Forest's EDF not known
FAN_N_HOVER_REF, FAN_W_REF = 13.2, 270.0
FAN_RATED, FAN_FULL = 16.0, 21.4
OTHER_W = 150.0             # [A] 104 servos holding the flight pose, flight controller, radios
V_LOADED = 21.0             # V, 6S under load (3.5 V per cell)
WH_PER_KG = 192.0           # Tattu 6S 22000 mAh 25C: 488 Wh, 2544 g (GetFPV listing)
USABLE = 0.80               # [A] fraction of the pack used before landing
PACKS = {                   # name: (Wh, kg, continuous A)
    "Tattu 6S 16000 mAh 15C": (16.0 * 22.2, 1.932, 16.0 * 15),
    "Tattu 6S 22000 mAh 25C": (22.0 * 22.2, 2.544, 22.0 * 25),
    "2 x 6S 10000 mAh 25C in parallel [A: 192 Wh/kg class]": (20.0 * 22.2, 20.0 * 22.2 / WH_PER_KG, 20.0 * 25),
}

def hover(extra_kg, edf_gpw=EDF_G_PER_W):
    W = (ROBOT_KG + extra_kg) * G
    fan = (W - EDF_N) / 8
    p_fan = FAN_W_REF * (fan / FAN_N_HOVER_REF) ** 1.5
    p_edf = EDF_N / G * 1000 / edf_gpw
    P = 8 * p_fan + p_edf + OTHER_W
    return dict(robot_kg=round(ROBOT_KG + extra_kg, 2), fan_hover_N=round(fan, 2), fan_hover_W=round(p_fan), edf_W=round(p_edf), total_W=round(P),
                current_A=round(P / V_LOADED), tw_rated=round((EDF_N + 8 * FAN_RATED) / W, 3), tw_full=round((EDF_N + 8 * FAN_FULL) / W, 3),
                fan_hover_pct_of_full=round(100 * fan / FAN_FULL))

def main():
    out = dict(assumptions=dict(robot_kg_without_battery=ROBOT_KG, edf_N=EDF_N, edf_g_per_W=EDF_G_PER_W, other_W=OTHER_W, v_loaded=V_LOADED,
                                wh_per_kg=WH_PER_KG, usable=USABLE), no_battery=hover(0.0), packs={}, per_kg={}, sensitivity={})
    for nm, (wh, kg, amps) in PACKS.items():
        h = hover(kg); t = wh * USABLE / h["total_W"] * 60
        h.update(pack_Wh=round(wh), pack_kg=kg, pack_cont_A=amps, current_ok=amps >= h["current_A"], hover_min=round(t, 1))
        out["packs"][nm] = h
    for kg in (0.5, 1.0, 1.5, 2.0, 2.5, 3.0):
        h = hover(kg); h["hover_min"] = round(kg * WH_PER_KG * USABLE / h["total_W"] * 60, 2); out["per_kg"][str(kg)] = h
    # sensitivity: how much of the body allowance is already battery, and a more efficient central fan
    for have in (0.0, 0.8, 1.5):
        kg = 1.932; h = hover(kg - have); h["hover_min"] = round(16 * 22.2 * USABLE / h["total_W"] * 60, 1)
        out["sensitivity"][f"16 Ah pack, {have} kg of battery already in the 3.45 kg body allowance"] = h
    for gpw in (1.2, 1.45, 1.8, 4.5):
        h = hover(1.932, gpw); h["hover_min"] = round(16 * 22.2 * USABLE / h["total_W"] * 60, 1)
        out["sensitivity"][f"16 Ah pack, central lift at {gpw} g/W"] = h
    json.dump(out, open(os.path.join(HERE, "..", "results", "battery.json"), "w"), indent=1)
    print(json.dumps(out, indent=1))

if __name__ == "__main__":
    main()
