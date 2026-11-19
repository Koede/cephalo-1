"""Summary sheet for the leg study re-run at Equo-1 v2 size (490 mm hips, 32.6 kg).  Reads legs.json from legs_v2_study.py."""
import sys, json
sys.path.insert(0, "/home/claude/octodrone")
import numpy as np, sk3d, matplotlib.pyplot as plt
R = json.load(open("legs.json"))
REC = "R36 RS03 front hips, RS06 elsewhere, 12S"
fig = sk3d.figure(16, 9, "Equo-1 v2 legs re-run at true size (hips 490 mm, 34 kg): torque margins")
cases = ["stand", "walk M1", "trot M1", "trot M2", "get up", "landing 2.5 g, pair", "trot + 0.25 g push", "walk up 10 deg"]
lab = ["stand", "walk", "trot", "trot\n(shaped)", "get up", "land\n2.5 g", "push\n0.25 g", "slope\n10 deg"]
for k, leg in enumerate(("front", "rear")):
    ax = fig.add_axes([0.05 + 0.48 * k, 0.48, 0.43, 0.40]); m = R["margins"][REC][leg]
    x = np.arange(len(cases)); w = 0.13
    for i, (j, c) in enumerate((("hip", "#2f6db3"), ("knee", "#d43a2f"), ("abd", "#2e9e5b"))):
        rm = [min(m[cs][j]["m_rms"], 6) if m[cs][j]["rms"] > 1e-6 else np.nan for cs in cases]
        pk = [min(m[cs][j]["m_peak"], 6) if m[cs][j]["peak"] > 1e-6 else np.nan for cs in cases]
        ax.bar(x + (2 * i - 2.5) * w, rm, w, color=c, label=f"{j} rated")
        ax.bar(x + (2 * i - 1.5) * w, pk, w, color="white", edgecolor=c, hatch="//", label=f"{j} peak")
    ax.axhline(1, color="k", ls="--", lw=1); ax.set_ylim(0, 6); ax.set_xticks(x); ax.set_xticklabels(lab, fontsize=11)
    ax.set_ylabel("margin (actuator / need), capped at 6", fontsize=11)
    ax.set_title(f"{leg.capitalize()} leg: " + ("RS03 hip, RS06 knee + abduction" if leg == "front" else "RS06 hip, knee + abduction"), loc="left", fontsize=13, fontweight="bold")
    if k == 0: ax.legend(fontsize=10, ncol=3, loc="upper right", frameon=False)
ax = fig.add_axes([0.03, 0.06, 0.6, 0.32]); ax.axis("off")
rows = []
for s, wv in R["worst"].items():
    rows.append([s.split("  ")[-1] if "  " in s else s.split(" ", 1)[1], f"{wv['req_rms']:.2f} / {wv['req_peak']:.2f}", f"{wv['all_rms']:.2f} / {wv['all_peak']:.2f}",
                 f"{R['parts'][s]['m_legs']:.1f} kg"])
tb = ax.table(cellText=rows, colLabels=["Actuator set", "Margin, core cases", "Margin, all cases", "Four legs"], loc="upper left",
              cellLoc="left", colLoc="left", colWidths=[0.42, 0.2, 0.2, 0.12])
tb.auto_set_font_size(False); tb.set_fontsize(11); tb.scale(1, 1.7)
for i, s in enumerate(R["worst"]):
    if s == REC:
        for j in range(4): tb[i + 1, j].set_facecolor("#dff3e4"); tb[i + 1, j].get_text().set_fontweight("bold")
    elif R["worst"][s]["all_peak"] < 1: [tb[i + 1, j].set_facecolor("#fbe3e1") for j in range(4)]
g = R["geom"]; b = R["belt"]["cases"]
sk3d.panel_text(fig, 0.66, 0.37, [
    f"Leg: femur {g['L1']*1e3:.0f}, tibia {g['L2']*1e3:.0f}, cannon {R['geom']['LC']*1e3:.0f}, pastern {g['LP']*1e3:.0f} mm",
    f"Knee belt: HTD 5M PU/steel, 25 x {g['belt_teeth']*5} mm, {g['z1']}T : {g['z2']}T",
    f"  teeth x{1104/b['trot M1']['tension']:.2f} trot peak, x{1104/b['landing 2.5 g, pair']['tension']:.2f} at a 2.5 g landing",
    f"  belt tension x{2500/b['landing 2.5 g, pair']['tension']:.1f} at landing (Elatech PU/steel ratings)",
    f"Knee actuator speed: {R['speeds'][REC]['fast trot']['knee_act_rpm']:.0f} rpm needed (fast trot)",
    f"Kneels to hip {R['kneel']['kneel']['hip']*1e3:.0f} mm, belly {R['kneel']['kneel']['belly']*1e3:.0f} mm",
    f"Hip pods {R['module'][REC]['width']*1e3:.0f} mm across, no clashes",
], fs=12)
sk3d.save(fig, "legs_v2_margins.png")
