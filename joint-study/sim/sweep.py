"""Enumerate arm configurations and evaluate them. Stage A runs once per unique geometry
(family, effective range, arrangement, actuation class, joint count, local servo); stage B fans out
over mechanism variant x drive x landing compliance."""
import math, itertools
import numpy as np
import config as C
import families as F
import metrics as M
import chain as CH

N_GRID = [7, 9, 11, 13, 15, 19, 25]
LOCAL_SERVOS = ["STS3215_12V", "STS3250_12V", "STS3032", "XL330_M288"]
REMOTE_DRIVES = ["STS3215_12V", "STS3032", "XL330_M288"]
ROOT_SERVOS = ["STS3250_12V", "STS3250x2", "HTS35H"]
COMP_FOR = dict(local=["rigid", "tpu_coupler", "overtravel_detent", "magnetic_breakaway"],
                remote=["rigid", "tendon_spring", "overtravel_detent", "magnetic_breakaway"])

def joint_types():
    out = []
    for f in F.FAMILIES:
        if f.get("addon") or f.get("excluded"): continue
        for v in f["variants"]:
            for r in f["ranges"]:
                out.append((f, v, r))
    return out

def arrangements(f):
    return ["omni", "omni_roll4"] if f["bend"] == 2 else ["planar", "alt90", "helix60", "alt90_roll4"]

def kin_jobs():
    """unique stage-A geometries, each with the stage-B combinations that share it"""
    jobs = {}
    for f, v, r in joint_types():
        reff = r * v.get("rng_mult", 1.0)
        for arr in arrangements(f):
            acts = list(f["act"]) + (["graded"] if "local" in f["act"] and not f.get("servo_override") else [])
            for act in acts:
                cls = "remote" if act in ("tendon", "pushpull") else act
                for n in ([0] if act == "graded" else N_GRID):
                    servos = [None]
                    if act == "local": servos = [f["servo_override"]] if f.get("servo_override") else LOCAL_SERVOS
                    if act == "hybrid": servos = ROOT_SERVOS
                    if act == "graded": servos = [(sm, rt, sp) for sm in ("STS3032", "XL330_M288") for rt in ("STS3250_12V", "STS3250x2") for sp in ("short_tip", "long_mid")]
                    if act == "hybrid" and n < 6: continue
                    for sv in servos:
                        key = (f["id"], round(reff, 1), arr, cls, n, sv, v.get("cant"), v.get("pitch_D"))   # variants that change geometry get their own stage A
                        cfg = dict(family=f["id"], variant=v["v"], range_deg=r, arrangement=arr,
                                   actuation="tendon" if cls == "remote" else act, n_nodes=n)
                        if act == "local": cfg["servo"] = sv
                        if act == "hybrid": cfg["root_servo"] = sv
                        if act == "graded": cfg["servo"], cfg["root_servo"], cfg["split"] = sv
                        j = jobs.setdefault(key, dict(cfg=cfg, combos=[]))
                        drives = [None] if act in ("local", "graded") else (["STS3032", "STS3215_12V"] if act == "hybrid" else REMOTE_DRIVES)
                        comps = COMP_FOR["local" if act in ("local", "graded") else "remote"]
                        for d, c in itertools.product(drives, comps):
                            j["combos"].append((v["v"], r, act, d, c))
    return list(jobs.values())

def run_job(job, seed=0):
    cfg = job["cfg"]
    try:
        kin = M.kin_eval(cfg, seed)
    except Exception as e:  # keep the sweep going, record the failure
        return [dict(family=cfg["family"], arrangement=cfg["arrangement"], error=repr(e)[:200])]
    f = CH.FAM[cfg["family"]]
    rows = []
    base = {k: v for k, v in kin.items() if not k.startswith("_")}
    for var_name, r, act, drive, comp in job["combos"]:
        var = next(v for v in f["variants"] if v["v"] == var_name)
        c2 = dict(cfg, variant=var_name, range_deg=r, actuation=act)
        sc = M.score(kin, c2, f, var, comp, F.COMPLIANCE[comp], drive)
        row = dict(family=f["id"], family_name=f["name"], prior_label=f.get("prior", ""), variant=var_name,
                   range_deg=r, arrangement=cfg["arrangement"], actuation=act, n_nodes_req=cfg["n_nodes"],
                   local_servo=cfg.get("servo") or f.get("servo_override") or "", root_servo=cfg.get("root_servo", ""),
                   remote_drive=drive or "", compliance=comp, grade_split=cfg.get("split", ""))
        row.update(base); row.update({k: (float(v) if isinstance(v, (np.floating, float)) else v) for k, v in sc.items()})
        rows.append(row)
    return rows
