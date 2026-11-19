"""Diagnostic for coupler_fea.py: solve one load case and plot the hand base's sigma_zz / sigma_1 in a radial section through
the hot spot, plus where the lip contact pressure sits.  python3 coupler_fea_diag.py <maxvol> <force az> <section az> <png>"""
import sys, json
import numpy as np
import coupler_fea as F

P = F.P


def main(maxvol, az, sec, path):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    M = F.build(maxvol); kpen = F.E / 0.25
    f, _ = F.load_vector(M, az)
    u, act, cforce = F.solve(M, f, kpen)
    fig, axs = plt.subplots(2, 2, figsize=(15, 10), facecolor="white")
    fig.suptitle(f"Coupler rev {F.REV} FEA diagnostic: force along az {az:.0f}; radial section at az {sec:.0f} (+-1.5 deg)", fontsize=14, fontweight="bold")
    out = {}
    for row, (key, nm) in enumerate((("H", "hand base"), ("Cl", "collar"))):
        B = M[key]; X, vm, s1, szz = F.stresses(B, u)
        a = np.degrees(np.arctan2(X[:, 1], X[:, 0])) % 360; r = np.hypot(X[:, 0], X[:, 1])
        sel = (np.abs((a - sec + 180) % 360 - 180) < 1.5) & (r > 17)
        for col, (v, lab) in enumerate(((szz, "sigma_zz (layer-normal), MPa"), (s1, "sigma_1, MPa"))):
            ax = axs[row, col]; o = np.argsort(v[sel])
            sc = ax.scatter(r[sel][o], X[sel, 2][o], c=v[sel][o], s=14, cmap="turbo", vmin=-10, vmax=35)
            ax.set_title(f"{nm}: {lab}", fontsize=11); ax.set_xlabel("r mm"); ax.set_ylabel("z mm"); ax.set_aspect("equal")
            fig.colorbar(sc, ax=ax, fraction=0.03)
        hot = np.argsort(szz)[-40:]
        out[nm] = dict(top_szz=[[round(float(r[i]), 2), round(float(a[i]), 1), round(float(X[i, 2]), 2), round(float(szz[i]), 1)] for i in hot[::-1][:15]])
    # lip contact: active c2 slave nodes and their penetration force
    out["contact_force_N"] = cforce
    fig.tight_layout(rect=[0, 0, 1, 0.95]); fig.savefig(path, dpi=100)
    json.dump(out, open(path.replace(".png", ".json"), "w"), indent=1)
    print(json.dumps(out, indent=1)[:3000])


if __name__ == "__main__":
    main(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4])
