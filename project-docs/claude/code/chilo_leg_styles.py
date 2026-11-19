"""Chilo-1 body-leg redesign: 5 leg styles on the full robot (octopus docked, arm-legs as solved), with static
servo loads per style.  Sketch + hand statics, no sim.
Load case (est.): centipede body ~9 kg + 21 N of the octopus through the hub, 7 of 10 legs down -> 16 N per foot.
python3 chilo_leg_styles.py -> leg_styles.png, leg_styles.json"""
import json
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from geom import frustum, sphere, merge, Scene, torus, box, frame_z
from render import Camera, render
import chilo_face_dock as FD, chilo_arm_legs_render as AR
from chilo_build_concepts import FONT, FOOT, SERVO, HUBC

F_FOOT = 16.0
SV = {"STS3215": (0.981, 2.94), "STS3250": (1.63, 4.90), "STS3215+4:1": (3.14, 9.4)}     # rated, stall (N m); geared = printed planetary, eff 0.8 (est.)
LEGC = (0.95, 0.62, 0.27); DARK = (0.22, 0.23, 0.25); STEEL = (0.72, 0.74, 0.77); SPRING = (0.18, 0.62, 0.35)
Yv = np.array([0, 1.0, 0]); Zv = np.array([0, 0, 1.0])


def planar(hip, s, pts_yz):
    """pts_yz: list of (dy outward, z) relative to the hip -> world points in the leg plane (x fixed)."""
    return [hip + s * Yv * dy + Zv * dz for dy, dz in pts_yz]


def torque(joints_yz, foot_yz, F=F_FOOT):
    return [F * abs(foot_yz[0] - j[0]) for j in joints_yz]


# each style: joints (dy, dz) from the hip, foot (dy, dz), servo per pitch joint, extra parts
H0 = FD.HC
STYLES = {
    "A": dict(name="A  Arm-matched tentacle legs", servos=["STS3250", "STS3250", "STS3215", "STS3215"],
              note="Short copy of the octopus arm (4 joints, same 56 mm link cradles, same colour) hanging down and curling in at the foot.",
              joints=[(0.04, 0.0), (0.085, -0.045), (0.115, -0.105), (0.13, -0.17)], foot=(0.14, -H0), armlike=True),
    "B": dict(name="B  True centipede", servos=["STS3250", "STS3215+4:1"],
              note="Femur up and out, long curved tibia raked back to a claw tip, like Scolopendra.  Printed 4:1 planetary at the knee.",
              joints=[(0.04, 0.0), (0.20, 0.13)], foot=(0.34, -H0), curve=True),
    "C": dict(name="C  Knees under (upright)", servos=["STS3250", "STS3215"],
              note="Femur angled down, tibia nearly vertical: the foot sits close under the hip, so the levers are short.",
              joints=[(0.04, 0.0), (0.12, -0.14)], foot=(0.14, -H0)),
    "D": dict(name="D  Geared spider legs", servos=["STS3215+4:1", "STS3215+4:1"],
              note="Spider stance (feet 260 mm out), both pitch joints on printed 4:1 planetaries (STS3215 inside each).",
              joints=[(0.04, 0.0), (0.17, 0.10)], foot=(0.26, -H0)),
    "E": dict(name="E  Spring-assisted", servos=["STS3250", "STS3215+4:1"],
              note="Same shape as B; an extension spring from the body to the femur carries the standing load, 4:1 knee.",
              joints=[(0.04, 0.0), (0.20, 0.13)], foot=(0.34, -H0), curve=True, spring=True),
}


def loads(st):
    tau = torque(st["joints"], st["foot"])
    if st.get("spring"):
        tau[0] = max(0.0, tau[0] - F_FOOT * 0.30 * 0.85)      # spring sized to cancel 85 % of the hip moment at stance (est.)
    rows = []
    for t, sv in zip(tau, st["servos"]):
        r, s = SV[sv]; rows.append(dict(servo=sv, tau_Nm=round(t, 2), x_rated=round(t / r, 2), x_stall=round(t / s, 2)))
    return rows


def draw_leg(sc, hip, s, st, fly, xo=0.0):
    J = planar(hip, s, st["joints"]); foot = planar(hip, s, [st["foot"]])[0]
    foot = foot + np.array([xo, 0, 0.02])
    col = (0.95, 0.62, 0.27) if st.get("armlike") else (0.78, 0.80, 0.83)
    pts = [hip] + J
    parts = [frustum(a, b, 0.017, 0.016, col, n=10) for a, b in zip(pts[:-1], pts[1:])]
    if st.get("curve"):
        k = J[-1]; m1 = k + s * Yv * 0.09 + Zv * -0.05 + np.array([-0.03, 0, 0]); m2 = 0.5 * (m1 + foot) + s * Yv * 0.035 + np.array([-0.04, 0, 0])
        tib = [k, m1, m2, foot]
        parts += [frustum(a, b, r0, r1, col, n=10) for a, b, r0, r1 in zip(tib[:-1], tib[1:], (0.016, 0.013, 0.010), (0.013, 0.010, 0.005))]
        parts += [sphere(p, r, col, n=8, m=6) for p, r in ((m1, 0.013), (m2, 0.010))]
    else:
        parts.append(frustum(J[-1], foot, 0.015, 0.007, col, n=10))
    for k, p in enumerate(J):
        geared = "4:1" in st["servos"][min(k, len(st["servos"]) - 1)] if k < len(st["servos"]) else False
        parts.append(sphere(p, 0.032 if geared else 0.024, SERVO, n=10, m=6))
        if st.get("armlike") and k + 1 < len(J):                                   # banded link like the arm cradles
            parts.append(sphere(0.5 * (p + J[k + 1]), 0.02, SERVO, n=8, m=6))
        if geared:
            parts.append(frustum(p - Yv * 0.0 - np.array([0.03, 0, 0]) * 0, p + np.array([0.0, 0, 0]) + s * np.array([0, 0, 0]), 0.0, 0.0, SERVO))
    parts.append(sphere(hip, 0.028, SERVO, n=10, m=6))
    parts.append(sphere(foot, 0.014, FOOT, n=10, m=6))
    sc.add(merge(parts))
    if st.get("spring"):
        anchor = hip + Zv * 0.07 - s * Yv * 0.02; att = J[0] + 0.45 * (J[1] - J[0])
        n = 14; d = att - anchor; R = frame_z(d)
        coil = [anchor + d * (t / n) + R @ np.array([0.009 * np.cos(t * 2.2), 0.009 * np.sin(t * 2.2), 0]) for t in range(n + 1)]
        sc.add(merge([frustum(a, b, 0.0025, 0.0025, SPRING, n=6) for a, b in zip(coil[:-1], coil[1:])]))


def build(sc, st, sol):
    ch = FD.chain([0, 0, 0])
    for i, (p0, p1, d) in enumerate(ch):
        FD.seg(sc, p0, p1, FD.RAD[i])
        if i < 5: sc.add(sphere(p0 - d * FD.GAP / 2, 0.04, STEEL, n=12, m=8))
        if i == 0: continue
        c = 0.5 * (p0 + p1)
        for s in (1, -1):
            hip = c + s * Yv * FD.RAD[i] * 0.92
            draw_leg(sc, hip, s, st, (i % 2 == 0) == (s > 0), xo=-0.03)
    p0, p1, d = ch[0]
    sc.add(torus(p1 + d * 0.004, d, 0.109, 0.009, HUBC, n=48, m=10))
    plate = p1 + d * 0.012
    AR.octo_body(sc, plate)
    import chilo_arm_legs as A
    for i in range(8):
        root, R0 = A.arm_frame(i, plate)
        if min(i, 7 - i) in sol:
            v = sol[min(i, 7 - i)]; off = plate - np.array([0.315 + 0.012, 0, A.HC])
            P = np.array(v["joints"]) + off; spig = np.array(v["spigot"]) + off; tip = np.array(v["foot"]) + off
            if i >= 4:
                M = np.diag([1, -1, 1.0]); P = P @ M; spig = spig @ M; tip = tip @ M
            AR.draw_arm(sc, root, P, spig, tip, AR.LEGARM, True)
        else:
            q = np.radians([-30, 0, -45, 0, -35, 0, -20, 0] if i in (3, 4) else [-20, 0, -30, 0, -40, 0, -30, 0])
            P, Ax, spig, tip = A.fk(q, root, R0); AR.draw_arm(sc, root, P, spig, tip, AR.ARM, False)


def still(key, sol, Wp=960, Hp=640):
    st = STYLES[key]; sc = Scene(); build(sc, st, sol)
    tgt = np.array([0.30, 0, 0.28]); az, el = np.radians(-40), np.radians(15)
    eye = tgt + 2.75 * np.array([np.cos(az) * np.cos(el), np.sin(az) * np.cos(el), np.sin(el)])
    img = Image.fromarray(render(sc.prims, Camera(eye, tgt, 34), Wp, Hp, ss=2, fog=(8, 18)))
    dr = ImageDraw.Draw(img)
    dr.rectangle([0, 0, Wp, 100], fill=(255, 255, 255))
    dr.text((20, 8), st["name"], font=ImageFont.truetype(FONT % "-Bold", 28), fill=(25, 30, 35))
    dr.text((22, 44), st["note"], font=ImageFont.truetype(FONT % "", 15), fill=(70, 78, 86))
    L = loads(st); worst = max(r["x_rated"] for r in L)
    txt = "   ".join(f"{r['servo']}: {r['x_rated']:.2f}x rated" for r in L)
    dr.text((22, 70), f"Standing at 16 N/foot:  {txt}", font=ImageFont.truetype(FONT % "-Bold", 15),
            fill=(30, 120, 70) if worst <= 1.0 else (190, 60, 30))
    dr.rectangle([0, 0, Wp - 1, Hp - 1], outline=(200, 205, 210), width=2)
    return img, L


if __name__ == "__main__":
    sol = {0: json.load(open("arm_leg_0_21.json")), 1: json.load(open("arm_leg_1_31.json"))}
    G = Image.new("RGB", (1920, 1920), "white"); out = {}
    for k, key in enumerate("ABCDE"):
        im, L = still(key, sol); out[key] = L; G.paste(im, ((k % 2) * 960, (k // 2) * 640))
    dr = ImageDraw.Draw(G); x, y = 990, 1300
    dr.text((x, y), "How to read the loads", font=ImageFont.truetype(FONT % "-Bold", 26), fill=(25, 30, 35))
    lines = ["16 N per foot (est.): centipede ~9 kg + 21 N of the octopus via the hub,",
             "with 7 of the 10 body legs down during the wave gait.",
             "Torque = foot force x sideways distance from the joint to the foot.",
             "Green = every servo at or under its rated (continuous) torque.",
             "",
             "STS3215: 0.98 rated / 2.94 stall N m",
             "STS3250: 1.63 rated / 4.90 stall N m",
             "STS3215 + printed 4:1 planetary: 3.14 / 9.4 N m, +35 g (est.)",
             "Spring (E): steel extension spring, hardware-store part,",
             "  set to carry 85 % of the standing hip load (est.).",
             "",
             "All legs drawn in the same standing pose (no gait).",
             "Over rated while standing: B's hip (2.9x) and D's hip (1.1x)."]
    for i, l in enumerate(lines):
        dr.text((x, y + 44 + 27 * i), l, font=ImageFont.truetype(FONT % "", 19), fill=(55, 62, 70))
    G.save("leg_styles.png"); json.dump(out, open("leg_styles.json", "w"), indent=1)
    for k, v in out.items(): print(k, v)
