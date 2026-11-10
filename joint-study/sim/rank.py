#!/usr/bin/env python3
"""Rank the sweep, check it against the earlier 9-type comparison, run MuJoCo on a shortlist,
draw the plots and export the top arms for the 3D preview.
  python3 rank.py [--workers 12] [--short 40]"""
import os
for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMBA_NUM_THREADS"):
    os.environ.setdefault(_v, "1")
import sys, json, math, argparse
from multiprocessing import Pool
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np, pandas as pd
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
import families as F, metrics as M, chain as CH, dynamics as DY, config as C

RES = os.path.join(HERE, "results")
PRIOR = {  # earlier grid: label -> (controllable joints, mean shape miss mm), and how it maps here
    "base-box rolling vertebrae": (19, 6, dict(family="rolling_1d", actuation="tendon", n_nodes_req=19)),
    "current servo chain": (9, 12, dict(family="servo_chain_bracket", actuation="local", n_nodes_req=9)),
    "geared double hinges": (8, 12, dict(family="gear_rolling", actuation="local", n_nodes_req=9)),
    "servo shells": (9, 12, dict(family="servo_shell", actuation="local", n_nodes_req=9)),
    "cross-axis flexures": (17, 18, dict(family="cross_axis_flexure", actuation="tendon", n_nodes_req=19)),
    "3-section rolling": (8, 18, dict(family="rolling_1d", actuation="section", n_nodes_req=9)),
    "cycloidal modules": (6, 19, dict(family="cycloidal_module", actuation="local", n_nodes_req=7)),
    "one-piece printed rolling": (8, 22, dict(family="rolling_monolithic", actuation="tendon", n_nodes_req=9)),
    "TPU spine": (4, 40, dict(family="tpu_spine", actuation="section", n_nodes_req=7)),
}

def cfg_of(row):
    cfg = dict(family=row.family, variant=row.variant, range_deg=int(row.range_deg), arrangement=row.arrangement,
               actuation=row.actuation, n_nodes=int(row.n_nodes_req))
    if row.actuation == "local": cfg["servo"] = row.local_servo
    if row.actuation in ("hybrid", "graded"): cfg["root_servo"] = row.root_servo
    if row.actuation == "graded":
        cfg["servo"] = row.local_servo
        cfg["split"] = getattr(row, "grade_split", "short_tip") or "short_tip"
    return cfg

def _dyn(row):
    row = pd.Series(row)
    cfg = cfg_of(row); fam = CH.FAM[row.family]; var = next(v for v in fam["variants"] if v["v"] == row.variant)
    kin = M.kin_eval(cfg)
    drive = row.remote_drive if isinstance(row.remote_drive, str) and row.remote_drive else None
    try:
        d = DY.evaluate(kin, cfg, fam, var, row.compliance, F.COMPLIANCE[row.compliance], drive)
    except Exception as e:
        d = dict(dyn_error=repr(e)[:150])
    d["row_id"] = int(row.row_id); return d

def pareto(df, cols_max):
    X = df[cols_max].to_numpy(); keep = np.ones(len(X), bool)
    for i in range(len(X)):
        if not keep[i]: continue
        dom = np.all(X >= X[i], 1) & np.any(X > X[i], 1)
        if dom.any(): keep[i] = False
    return keep

def export_arm(row, path_prefix):
    cfg = cfg_of(row); ch = CH.build_chain(cfg)
    shapes = M.target_shapes(ch); poses = {}
    poses["straight"] = CH.run_fk(ch, M._x0(ch))[0]
    for nm in ("arc_curl", "spiral_curl", "helix_coil", "s_curve", "twisted_bend"):
        q, _, _ = M.fit_shape(ch, shapes[nm]); poses[nm] = CH.run_fk(ch, q)[0]
    qa = M.ik_foot(ch, M.FEET[1], M._x0(ch))[0]; poses["walking_stance"] = CH.run_fk(ch, qa)[0]
    return dict(label=f"{row.family_name} | {row.variant} | {row.arrangement} | {row.actuation} | N={int(row.N)}",
                radii=[float(r) for r in ch.seg_r], node_act=list(ch.node_act),
                poses={k: np.round(v, 5).tolist() for k, v in poses.items()},
                targets={k: np.round(shapes[k], 5).tolist() for k in ("arc_curl", "spiral_curl", "helix_coil", "s_curve", "twisted_bend")})

def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--workers", type=int, default=12); ap.add_argument("--short", type=int, default=40)
    a = ap.parse_args()
    df = pd.read_csv(os.path.join(RES, "sweep.csv")); df["row_id"] = np.arange(len(df))
    df["joint_type"] = df.family + "/" + df.variant + "/" + df.range_deg.astype(str)
    df["arch"] = df.actuation.map(dict(local="servo in every node", tendon="base-box tendons", pushpull="base-box push-pull",
                                         section="3-section tendons", hybrid="root servos + base-box tendons", graded="graded servo chain"))
    summ = dict(configs=len(df), geometries=int(df.groupby(["family", "range_deg", "arrangement", "actuation", "n_nodes_req", "local_servo", "root_servo"]).ngroups) if False else int(df.groupby(["family", "range_deg", "arrangement", "actuation", "n_nodes_req", "local_servo", "root_servo"], dropna=False).ngroups),
                joint_types=int(df.joint_type.nunique()), families=int(df.family.nunique()), feasible=int(df.feasible.sum()))
    # fail reasons
    fails = df.loc[~df.feasible, "fail"].str.split(",").explode().value_counts().to_dict()
    summ["fail_counts"] = fails
    # validation against earlier grid
    val = []
    for lab, (nj, miss_prev, sel) in PRIOR.items():
        sub = df[(df.family == sel["family"]) & (df.actuation == sel["actuation"]) & (df.n_nodes_req == sel["n_nodes_req"])]
        if len(sub) == 0: continue
        best = sub.loc[sub.miss_mean_mm.idxmin()]
        val.append(dict(prior=lab, prior_joints=nj, prior_miss_mm=miss_prev, here_dof=int(best.nv), here_miss_mm=round(float(best.miss_mean_mm), 1),
                        here_arrangement=best.arrangement))
    val = pd.DataFrame(val)
    from scipy.stats import spearmanr
    rho = spearmanr(val.prior_miss_mm, val.here_miss_mm).statistic if len(val) > 2 else float("nan")
    summ["validation_spearman"] = float(rho)
    val.to_csv(os.path.join(RES, "validation_vs_earlier_grid.csv"), index=False)
    # shortlist: best feasible configs, at most 3 per family and 1 per (family, actuation, arrangement)
    feas = df[df.feasible].sort_values("score", ascending=False)
    feas = feas.drop_duplicates(["family", "actuation", "arrangement"])
    short = feas.groupby("family").head(3).head(a.short)
    # the chosen main design always gets a dynamics run, wherever it lands
    md = getattr(C, "MAIN_DESIGN", None)
    if md:
        allf = df[df.feasible].sort_values("score", ascending=False)
        sel = allf
        for k, v in md.items(): sel = sel[sel[k] == v]
        if len(sel) and sel.iloc[0].row_id not in set(short.row_id):
            short = pd.concat([short, sel.head(1)])
    with Pool(a.workers) as pool:
        dyn = pool.map(_dyn, [r.to_dict() for _, r in short.iterrows()])
    dyn = pd.DataFrame(dyn)
    short = short.merge(dyn, on="row_id", how="left")
    ok = short.get("dyn_error").isna() if "dyn_error" in short else np.ones(len(short), bool)
    short["s_dynamics"] = ((np.exp(-short.wave_tip_rms_mm / 50) * np.exp(-short.lift_tip_rms_mm / 30) *
                            (1 - 0.5 * short.wave_sat_frac) * np.minimum(1, 1 / short.step_settle_s.clip(lower=0.05)) *
                            np.exp(-short.sag_mm / 60)) ** 0.2).where(ok, 0.05)
    short["final_score"] = short.score * short.s_dynamics ** 0.5
    short = short.sort_values("final_score", ascending=False)
    short.to_csv(os.path.join(RES, "shortlist_dynamics.csv"), index=False)
    # pareto on the full feasible set
    feas_all = df[df.feasible].copy()
    feas_all["neg_miss"] = -feas_all.miss_mean_mm
    feas_all["pareto"] = pareto(feas_all, ["dof", "neg_miss", "s_build", "s_loads", "s_flight_mass", "s_precision"])
    feas_all[feas_all.pareto].sort_values("score", ascending=False).to_csv(os.path.join(RES, "pareto_front.csv"), index=False)
    summ["pareto_size"] = int(feas_all.pareto.sum())
    # best per architecture and per family
    df.sort_values("score", ascending=False).groupby("arch").head(5).to_csv(os.path.join(RES, "best_per_architecture.csv"), index=False)
    df.sort_values("score", ascending=False).drop_duplicates("family").to_csv(os.path.join(RES, "best_per_family.csv"), index=False)
    plots(df, feas_all, short, val)
    tops = []
    md = getattr(C, "MAIN_DESIGN", None)
    if md:
        sel = short
        for k, v in md.items(): sel = sel[sel[k] == v]
        if len(sel):
            e = export_arm(sel.iloc[0], ""); e["label"] = "Main design | " + getattr(C, "MAIN_LABEL", e["label"]); tops.append(e)
    tops += [export_arm(r, "") for _, r in short.head(3).iterrows()]
    hd = feas_all.sort_values(["dof", "score"], ascending=False)
    if len(hd):
        e = export_arm(hd.iloc[0], ""); e["label"] = "Most DOF (feasible) | " + e["label"]; tops.append(e)
    json.dump(tops, open(os.path.join(RES, "top_arms_geometry.json"), "w"))
    json.dump(summ, open(os.path.join(RES, "summary.json"), "w"), indent=1, default=str)
    print(json.dumps(summ, indent=1, default=str)); print(val.to_string())
    cols = ["family", "variant", "range_deg", "arrangement", "actuation", "N", "local_servo", "root_servo", "remote_drive", "compliance",
            "dof", "miss_mean_mm", "util_walk", "util_flight_nom", "util_land", "tip_err_mm", "total_mass_kg", "thrust_to_weight",
            "n_drives_robot", "cost_usd", "assembly_h", "score", "wave_tip_rms_mm", "lift_tip_rms_mm", "sag_mm", "final_score"]
    print(short[cols].head(15).to_string())

PAL = {"graded servo chain": "#0f8b8d", "servo in every node": "#2a6fdb", "base-box tendons": "#d9822b", "3-section tendons": "#8a63d2",
       "root servos + base-box tendons": "#6b8e23", "base-box push-pull": "#c2405a"}

def plots(df, feas, short, val):
    plt.rcParams.update({"font.size": 9, "axes.spines.top": False, "axes.spines.right": False})
    # 1 DOF vs fluidity
    fig, ax = plt.subplots(figsize=(7.5, 5))
    g = df.sort_values("score").drop_duplicates(["family", "variant", "range_deg", "arrangement", "actuation", "n_nodes_req"], keep="last")
    for arch, c in PAL.items():
        s = g[g.arch == arch]
        ax.scatter(s.dof, s.miss_mean_mm, s=6, c=c, alpha=0.25, lw=0)
        sf = s[s.feasible]; ax.scatter(sf.dof, sf.miss_mean_mm, s=14, c=c, alpha=0.9, lw=0, label=arch)
    for _, r in short.head(5).iterrows():
        ax.scatter(r.dof, r.miss_mean_mm, s=80, facecolors="none", edgecolors="black", lw=1.2)
    ax.set_yscale("log"); ax.set_xlabel("Controlled DOF per arm"); ax.set_ylabel("Mean shape miss over 7 target shapes (mm, log)")
    ax.set_title("DOF vs fluidity: faint = fails a gate, solid = feasible, circles = top 5 after dynamics", fontsize=9, loc="left")
    ax.legend(frameon=False, fontsize=8, loc="upper right"); fig.tight_layout(); fig.savefig(os.path.join(RES, "dof_vs_fluidity.png"), dpi=160); plt.close(fig)
    # 2 top 15 subscores
    sub = ["s_dof", "s_fluidity", "s_loads", "s_precision", "s_flight_mass", "s_build", "s_dynamics"]
    t = short.head(15).iloc[::-1]
    fig, ax = plt.subplots(figsize=(8.5, 6))
    im = ax.imshow(t[sub].to_numpy(), aspect="auto", cmap="viridis", vmin=0, vmax=1)
    ax.set_xticks(range(len(sub)), [s[2:] for s in sub], rotation=30, ha="right")
    ax.set_yticks(range(len(t)), [f"{r.family} {r.arrangement} {r.actuation} N={int(r.N)} {r.compliance}  ({r.final_score:.2f})" for _, r in t.iterrows()], fontsize=7)
    fig.colorbar(im, ax=ax, fraction=0.03); ax.set_title("Top 15 after the MuJoCo check: sub-scores (0-1), final score in brackets", fontsize=9, loc="left")
    fig.tight_layout(); fig.savefig(os.path.join(RES, "top15_subscores.png"), dpi=160); plt.close(fig)
    # 3 family x architecture best score
    piv = df.pivot_table(index="family", columns="arch", values="score", aggfunc="max")
    piv = piv.loc[piv.max(axis=1).sort_values().index]
    fig, ax = plt.subplots(figsize=(7.5, 8))
    im = ax.imshow(piv.to_numpy(), aspect="auto", cmap="magma", vmin=0)
    ax.set_xticks(range(piv.shape[1]), piv.columns, rotation=25, ha="right", fontsize=8); ax.set_yticks(range(piv.shape[0]), piv.index, fontsize=7)
    fig.colorbar(im, ax=ax, fraction=0.03); ax.set_title("Best composite score per joint family and architecture (blank = not applicable)", fontsize=9, loc="left")
    fig.tight_layout(); fig.savefig(os.path.join(RES, "family_by_architecture.png"), dpi=160); plt.close(fig)
    # 4 mass vs DOF
    fig, ax = plt.subplots(figsize=(7.5, 4.5))
    for arch, c in PAL.items():
        s = g[g.arch == arch]; ax.scatter(s.dof, s.total_mass_kg, s=6, c=c, alpha=0.4, lw=0, label=arch)
    ax.axhline(128.6 / 9.81 / 1.15, color="k", lw=0.8, ls="--"); ax.text(1, 128.6 / 9.81 / 1.15 + 0.2, "T/W = 1.15 gate", fontsize=8)
    ax.axhline(8.63, color="grey", lw=0.8, ls=":"); ax.text(1, 8.4, "today 8.63 kg", fontsize=8, color="grey")
    ax.set_ylim(5, 20); ax.set_xlabel("Controlled DOF per arm"); ax.set_ylabel("Robot mass (kg)"); ax.legend(frameon=False, fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(RES, "mass_vs_dof.png"), dpi=160); plt.close(fig)

if __name__ == "__main__":
    main()
