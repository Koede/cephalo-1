"""Will the B.5 coupler survive?  Datasheet Z (layer-bond) strengths vs the FEA stress bands (2026-11-19).
Stress bands (B.5 FEA, 3 g wheel landing, est.): collar lip-wall fillet 15-20 MPa, hand-base lug-root fillet 15-22 MPa.
Knock-downs: real print vs datasheet bars 0.8 (est.); repeated landings: fatigue 0.5 of static for 1e3-1e4 cycles (est.)."""
import json
MAT = {  # Z tensile strength, MPa: (dry, wet), datasheet typical
    "Bambu PAHT-CF (PA12-CF), TDS V2.0":        (64.0, 64.0 * 0.85),   # wet: Bambu quotes 12-18 % drop after water uptake
    "Bambu PAHT-CF, older TDS value":            (47.0, 47.0 * 0.85),
    "Polymaker Fiberon PA6-CF20":                 (54.0, 25.5),          # wet value from the PolyMide PA6-CF TDS
}
PEAK = {"collar fillet": 20.0, "hand lug root": 22.0}
PRINT, FATIGUE, G = 0.8, 0.5, 3.0
out = {}
for m, (dry, wet) in MAT.items():
    r = {}
    for cond, s in (("dry", dry), ("wet", wet)):
        allow = s * PRINT
        worst = max(PEAK.values())
        r[cond] = dict(allowable_MPa=round(allow, 1), SF_single_3g=round(allow / worst, 2), SF_repeated_3g=round(allow * FATIGUE / worst, 2),
                       breaks_at_g_single=round(G * allow / worst, 1), unlimited_landings_up_to_g=round(G * allow * FATIGUE / worst, 1))
    out[m] = r
json.dump(out, open("coupler_strength_margin.json", "w"), indent=1)
for m, r in out.items():
    print(m); [print("  ", k, v) for k, v in r.items()]
