"""hand3 v3 (travelling roots, no sideways swing): how do the curl cords get from the spinning pull rings (below the hub) to a
finger whose root can sit at any 30 deg position?  Options study, 2026-11-19.  r-z sections to scale from hand3_hubc /
hand3_root_lock_bolt; clearances are interval math on those numbers; masses and friction are estimates (est.).
Writes hand3_cord_route_study.png / .json"""
import os, sys, json, textwrap
import numpy as np, matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, os.path.join(HERE, "hand3e")); os.chdir(os.path.join(HERE, "hand3e"))
import hand3_design as H, hand3_hubc as C
os.chdir(HERE)
LK = json.load(open("hand3_root_lock_bolt.json"))["layout"]
RPM = 12500.0; W2 = (RPM * 2 * np.pi / 60) ** 2
z0 = LK["z0"]; zt = z0 + LK["disc_t"] + LK["gap"]; rd = LK["disc_r"]
ring_top = C.Z_OUT[1]; lip_r0 = C.R_LIP_OUT[0]; lip_z = C.Z_LIP_OUT
feet_r1 = LK["pin_r"] + 1 + LK["float_"] + LK["foot"]
gap_z = z0 - 0.8 - ring_top                                          # under the disc, above the rings (feet hang to z0 - 0.8)
def bead(m_g, r): return m_g * 1e-3 * W2 * r * 1e-3
opts = [
 dict(key="A", name="Two traveller hoops under the disc rim (keeps K1 and K2+K3 separate)",
      how="Each pull ring carries a steel wire hoop on top; each finger's cord ends in a C-clip bead that slides round its hoop. The cord goes out under the disc rim and up into the finger root.",
      clr=f"outer bead to the static yoke lip: {lip_r0 - (rd + 0.3 + 1.2):.1f} mm; inner cord over the outer bead about 0.6 mm (est.)",
      load=f"bead 0.3 g at r 26: {bead(0.3, 26):.0f} N outward, held by the hoop", parts="2 x 0.8 mm music-wire hoops, 8 brass C-clips (est.)", servos="2 (as today)",
      grip="K1 and K2+K3 separate", risk="tightest; two hoops share 4 mm under the disc", verdict="works, tight"),
 dict(key="B", name="One ring, one hoop: K1 to K3 curl together",
      how="As A with only the outer (base) ring. The inner ring and its servo go; each finger curls all three knuckles from one cord (today's K2+K3 cord path).",
      clr=f"yoke lip inner edge moved out 0.6 mm (r {lip_r0} -> {lip_r0 + 0.6:.1f}, still 2.3 mm over the flange): bead clears it by {lip_r0 + 0.6 - (rd + 0.3 + 1.2):.1f} mm; {gap_z:.1f} mm of height for one hoop (est. 2.0 used)",
      load=f"same bead: {bead(0.3, 26):.0f} N on the hoop", parts="1 hoop, 4 C-clips; minus 1 STS3215 (-55 g)", servos="1 (one STS3215 freed)",
      grip="one curl motion per finger; no separate K1", risk="less grip shaping; finger force sharing must be re-sized", verdict="recommended", tint=True),
 dict(key="C", name="Hanger arm from each root block down past the rim, slotted guide on the ring",
      how="A thin arm hangs from the block outside the disc rim and drags a slider on a rail on the pull ring; the cord runs inside the arm.",
      clr=f"arm at r {rd + 0.2:.1f} to {rd + 1.4:.1f}: {lip_r0 - rd - 1.4:.1f} mm to the yoke lip, hits the base ring top when pulled 13 mm down",
      load="arm 0.6 g at r 26 (est.): 26 N bending it outward", parts="4 printed arms, 2 rails", servos="2",
      grip="K1 and K2+K3 separate", risk="arm must reach 13 mm down through the moving ring: collides", verdict="rejected (fit)"),
 dict(key="D", name="Bowden cables with a slack loop round the hub",
      how="Sheathed cable from a fixed anchor on the ring to the root block; the sheath lies in a channel and takes up the root's travel.",
      clr="needs about 80 mm of loose sheath per finger (half a turn at r 25) in a channel outside the rim: no room under the cage",
      load=f"80 mm of sheath (est. 1.5 g) at r 28: {bead(1.5, 28):.0f} N of fling", parts="4 Bowden cables, channel", servos="2",
      grip="K1 and K2+K3 separate", risk="friction round half a turn (est. 30 to 50 % loss); fling", verdict="rejected"),
 dict(key="E", name="Ball-end cords that drop into 12 cradles on the ring",
      how="Each cord ends in a ball sitting in one of 12 open cradles (every 30 deg). When the root moves, the ball slides out over a ramp and drops into the next.",
      clr="like A (one ring) but the ball must climb out sideways when slack: cradle walls 1.5 mm",
      load=f"ball 0.5 g at r 26: {bead(0.5, 26):.0f} N, seated by the spin", parts="12 cradles per ring, 4 balls", servos="1 or 2",
      grip="as A or B", risk="ball can hop to the wrong cradle; needs slack at each move", verdict="fallback for B"),
 dict(key="F", name="Keep everything on the rotor top: pull plate above the hub",
      how="Rings move above the top disc; pulled through a hollow motor shaft by a static rod.",
      clr="SE 2816 shaft is solid dia 5: no rod through it", load="-", parts="different motor", servos="2", grip="-", risk="motor change", verdict="rejected (motor)"),
]
out = dict(question="cord route from the spinning pull rings to a travelling root", gap_under_disc=gap_z, disc_r=rd, lip_r0=lip_r0, options=opts)
json.dump(out, open("hand3_cord_route_study.json", "w"), indent=1, default=float)

fig = plt.figure(figsize=(20, 14.5), facecolor="white")
fig.suptitle("hand3 v3: how do the curl cords reach a finger whose root can sit anywhere round the hub?  (sections to scale, radius across, height up, mm)", fontsize=16, fontweight="bold", y=0.99)
fig.text(0.5, 0.962, f"The base disc is one piece from the shaft out to r {rd:g}: anything below it that must follow a root has to go round its rim.  Space under the disc above the rings: {gap_z:.1f} mm.  "
         f"Static yoke lip from r {lip_r0}.  Grey = spins, orange = static, red = cord path.", ha="center", fontsize=10.5)
def base(ax):
    def R(r0, r1, a, b, **kw): ax.add_patch(Rectangle((r0, a), r1 - r0, b - a, **kw))
    R(0, 17, 66, 89, fc="#555a63", ec="black"); R(0, 2.5, 89, 104, fc="#333", ec="none")
    R(*C.R_CAGE, H.Z["cage_lo"], H.Z["fin_top"], fc="#c9cfd6", ec="black")
    R(*C.R_IN, C.Z_IN[0], C.Z_IN[1], fc="#39a0e0", ec="black", alpha=.85); R(C.R_IN[1], C.R_FL_IN, *C.Z_FL_IN, fc="#39a0e0", ec="black", alpha=.85)
    R(*C.R_OUT, *C.Z_OUT, fc="#e07b39", ec="black", alpha=.85); R(C.R_OUT[1], C.R_FL_OUT, *C.Z_FL_OUT, fc="#e07b39", ec="black", alpha=.85)
    R(*C.R_LIP_OUT, *C.Z_LIP_OUT, fc="#f0a030", ec="black"); R(*C.R_LIP_IN, *C.Z_LIP_IN, fc="#f0a030", ec="black"); R(*C.R_LEG, 66, C.Z_LIP_OUT[1], fc="#f0a030", ec="black", alpha=.6)
    R(2.5, rd, z0, z0 + 2, fc="#b9bec5", ec="black"); R(2.5, rd, zt, zt + 2, fc="#b9bec5", ec="black")
    R(LK["block_r"][0], LK["block_r"][1], z0 + 2, zt, fc="#2f6fbf", ec="black", alpha=.7); R(LK["block_r"][1], 46, z0 + 2.5, zt - 0.5, fc="#2f6fbf", ec="black", alpha=.25)
    R(LK["pin_r"] - 1, feet_r1, z0 - 0.8, z0 + 4, fc="#d62728", ec="none", alpha=.5)
    ax.set_xlim(8, 46); ax.set_ylim(64, 104); ax.set_xticks([]); ax.set_yticks([]); ax.set_aspect("equal")
def cord(ax, pts, **kw): p = np.array(pts); ax.plot(p[:, 0], p[:, 1], color="#d62728", lw=2.2, **kw)
for i, o in enumerate(opts):
    ax = fig.add_axes([0.008 + (i % 3) * 0.33, 0.63 - (i // 3) * 0.33, 0.32, 0.29]); base(ax)
    k = o["key"]
    if k in ("A", "B", "E"):
        rings = [(C.R_OUT, "#e07b39")] + ([(C.R_IN, "#39a0e0")] if k == "A" else [])
        for j, (rr, cc) in enumerate(rings):
            rh = rd + 0.4 if j == 0 else rr[0] + 1.0; zh = ring_top + 0.7 + (0 if j == 0 else 1.6)
            ax.add_patch(Circle((rh, zh), 0.4 if k != "E" else 0.6, fc="black")); ax.add_patch(Rectangle((rh - 0.7, zh - 0.7), 1.4, 1.4, fc="none", ec=cc, lw=1.5))
            cord(ax, [(rh, zh + 0.7), (rd + 0.6 + 0.5 * j, z0 - 0.4), (rd + 1.0 + 0.5 * j, z0 + 3), (31 + 2 * j, z0 + 3.5), (36 + 2 * j, z0 + 4)])
    if k == "C":
        ax.add_patch(Rectangle((rd + 0.2, ring_top - 13), 1.2, z0 + 3 - (ring_top - 13), fc="#2f6fbf", ec="black", alpha=.6)); cord(ax, [(rd + 0.8, ring_top - 12), (rd + 0.8, z0 + 3), (36, z0 + 4)])
        ax.text(28.5, 72, "arm must reach the\nring all the way\ndown its 13 mm stroke", color="#d62728", fontsize=9)
    if k == "D":
        ax.add_patch(Rectangle((rd + 2, 76), 4, 10, fc="none", ec="#d62728", lw=1.5, ls="--")); cord(ax, [(23, ring_top), (rd + 4, 80), (rd + 4, z0 + 3), (36, z0 + 4)])
        ax.text(33, 70, "80 mm of sheath\nper finger:\nno room", color="#d62728", fontsize=9)
    if k == "F": ax.text(27, 74, "pull rod would run\nthrough the solid\ndia 5 motor shaft", color="#d62728", fontsize=9)
    ax.set_title("\n".join(textwrap.wrap(f"{k}  {o['name']}", 52)), fontsize=11, fontweight="bold", color="#1a7f37" if o.get("tint") else "black")
hdr = ("", "How it works", "Clearance", "Spin load", "Parts", "Servos", "Grip", "Main risk", "Verdict")
rows = [hdr] + [(f"{o['key']}  {o['name']}", o["how"], o["clr"], o["load"], o["parts"], o["servos"], o["grip"], o["risk"], o["verdict"]) for o in opts]
tax = fig.add_axes([0.008, 0.005, 0.984, 0.29]); tax.axis("off"); Wc = [0.13, 0.21, 0.14, 0.09, 0.1, 0.06, 0.09, 0.11, 0.07]; X = np.cumsum([0] + Wc[:-1]); wr = [25, 50, 31, 18, 20, 11, 18, 22, 13]; n = len(rows)
for i, r_ in enumerate(rows):
    y = 1 - (i + 1) / n
    for j, c in enumerate(r_):
        tax.add_patch(Rectangle((X[j], y), Wc[j], 1 / n, fc="#dff3e3" if (i and opts[i - 1].get("tint")) else ("#eeeeee" if i == 0 else "white"), ec="#bbb", transform=tax.transAxes))
        tax.text(X[j] + 0.003, y + 0.5 / n, "\n".join(textwrap.wrap(c, wr[j])), transform=tax.transAxes, va="center", fontsize=8.0, fontweight="bold" if (i == 0 or j == 0) else "normal")
fig.savefig("hand3_cord_route_study.png", dpi=85)
print("gap under disc", round(gap_z, 2), "bead to lip", round(lip_r0 - (rd + 0.3 + 1.2), 2), [o["verdict"] for o in opts])
