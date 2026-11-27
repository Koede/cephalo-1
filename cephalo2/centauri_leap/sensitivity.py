#!/usr/bin/env python3
"""Does the optimised leap still land if the estimates are off?  Same commands, changed robot:
body pitch inertia 1.0 / 1.9 kg m2 (est. 1.4), centre of mass 30 mm lower / higher, bus 36 V (sagged) / 48 V (full),
floor grip 0.5 (est. 0.7), body 1.5 kg heavier (Cephalo-2 at 14.81 kg instead of the 12.81 kg the mass budget used).
    python3 sensitivity.py -> sensitivity.json"""
import os, json, subprocess, re
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
CASES = [("baseline", {}, 39.6, None), ("body inertia 1.0", {"CENT_IBODY": "1.0"}, 39.6, None), ("body inertia 1.9", {"CENT_IBODY": "1.9"}, 39.6, None),
         ("CoM 30 mm lower", {"CENT_DZ": "-30"}, 39.6, None), ("CoM 30 mm higher", {"CENT_DZ": "30"}, 39.6, None),
         ("bus 36 V", {}, 36.0, None), ("bus 48 V", {}, 48.0, None), ("grip 0.5", {}, 39.6, ("friction", 0.5)), ("body +1.5 kg", {}, 39.6, ("mass", 1.5))]


def main():
    import leap as L
    x = json.load(open(os.path.join(HERE, "best_params.json")))["x"]; out = []
    for name, env, V, mod in CASES:
        xml = "centauri1_revC.xml"
        if env:
            xml = "sens_%s.xml" % re.sub(r"\W+", "_", name)
            subprocess.run(["python3", "build_model.py"], cwd=HERE, env=dict(os.environ, CENT_XML=xml, **env), check=True, capture_output=True)
        txt = open(os.path.join(HERE, xml)).read()
        if mod and mod[0] == "friction":
            txt = txt.replace('friction="0.7 ', 'friction="%.2f ' % mod[1]); xml = "sens_grip.xml"; open(os.path.join(HERE, xml), "w").write(txt)
        if mod and mod[0] == "mass":
            m0 = float(re.search(r'<body name="torso".*?<inertial pos="[^"]*" mass="([\d.]+)"', txt, re.S).group(1))
            txt = txt.replace('mass="%.5f"' % m0, 'mass="%.5f"' % (m0 + mod[1]), 1); xml = "sens_mass.xml"; open(os.path.join(HERE, xml), "w").write(txt)
        R = L.Rollout(xml, V); o = R.run(np.array(x)); f = o["final"]; mon = o["mon"]
        landed = (mon["bad_contact"] is None and abs(f["pitch"]) < 5 and abs(f["z"]) < 0.04 and abs(f["vx"]) < 0.1 and o["st"]["td"]["fore"] is not None
                  and o["st"]["td"]["hind"] is not None and o["st"]["td"]["fore"] <= o["st"]["td"]["hind"])
        out.append(dict(case=name, leap_mm=round(o["dist"] * 1e3), air_mm=round(o["air_travel"] * 1e3), flight_ms=round(o["flight"] * 1e3), rise_mm=round(o["rise"] * 1e3),
                        lands_and_stops=bool(landed), ext_err_deg=round(mon["ext_err_min"], 1), pitch_max=round(mon["pitch_minmax"][1], 1),
                        pitch_min=round(mon["pitch_minmax"][0], 1), fore_land_N=round(mon["grf_land"]["fore"]), stop_Nm=round(mon["stop_Nm"]), body_floor=mon["bad_contact"]))
        print(out[-1], flush=True)
    json.dump(out, open(os.path.join(HERE, "sensitivity.json"), "w"), indent=1, default=str)


if __name__ == "__main__":
    main()
