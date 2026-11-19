"""Cephalo-2 wheel stance (2026-11-18): four arms on wheel hands (arms 0, 2, 4, 6), four keep hand3 raised.
Axles point along +-y (a car: wheels on the left and right sides), each axle = the arm-tip axis, tyre on the floor.
Finds the stance (plate height, wheel x / y) with the lowest servo load, then checks the load cases.
python3 wheelstance.py -> wheelstance.json, wheelstance_q.json"""
import os, json
os.environ.setdefault("CEPH_PROP", "xl"); os.environ.setdefault("CEPH_PACKS", "2"); os.environ.setdefault("CEPH_MONITOR", "0")
import numpy as np
import cephalo4 as C

G = 9.81
WH = json.load(open("wheelhand.json")); SZ = json.load(open("wheelhand_sizing.json"))
LEAD = WH["lever: socket mouth to tyre centre plane (mm)"] / 1e3          # spigot (socket mouth) to tyre centre plane
R_W = 0.060
M_WHEELHAND = WH["mass (g)"]["total"] / 1e3 + 0.0177 + 0.0193              # + rev B socket + collar on the arm
COM_WH = 0.055                                                             # wheel hand COM from the mouth (est. from the CAD masses)
M_HAND3 = 0.3964
M_ROBOT = 14.81 - 4 * (M_HAND3 - M_WHEELHAND)                               # four hand3 swapped for wheel hands
W = M_ROBOT * G
WHEEL_ARMS = (0, 2, 4, 6); UP_ARMS = (1, 3, 5, 7)
MODE = os.environ.get("WS_MODE", "axial")          # axial: axle = arm-tip axis (wheelhand.py v1); column: axle across the tip (bevel drive), arm stands on it
L_COLUMN = 0.045 + R_W                              # column mode: socket mouth -> axle 45 mm (bevel box, est.), + tyre radius -> contact
P4 = json.load(open("poses4.json"))
Q_UP = np.radians(P4["flight"])                                            # hand3 arms held up (tulip), props stopped


def side(i):
    a = np.radians(22.5 + 45 * i); return 1.0 if np.sin(a) > 0 else -1.0


def contact_targets(X, Y):
    out = {}
    for i in WHEEL_ARMS:
        a = np.radians(22.5 + 45 * i)
        out[i] = np.array([np.sign(np.cos(a)) * X, side(i) * Y, 0.0])
    return out


def solve(h, X, Y, q0=None):
    body_p = np.array([0, 0, h]); qs = {}; err = 0.0
    for i in WHEEL_ARMS:
        if MODE == "axial":
            u = np.array([0, side(i), 0.0]); centre = contact_targets(X, Y)[i] + [0, 0, R_W]; lead = LEAD
        else:
            u = np.array([0, 0, -1.0]); centre = contact_targets(X, Y)[i]; lead = L_COLUMN
        q_nom = np.zeros(8)
        q = C.ik(i, centre, u, q0[i] if q0 else np.zeros(8), q_nom, body_p, lead=lead, w_ax=3.0, w_reg=0.01)
        f, uu = C.tip_point(i, q, body_p, lead=lead)
        err = max(err, np.linalg.norm(f - centre), np.degrees(np.arccos(np.clip(uu @ u, -1, 1))) * 1e-3)
        qs[i] = q
    return qs, err


def loads(qs, h, X, Y, F_extra=np.zeros(3), grav=np.array([0, 0, -G])):
    """Servo load fractions (|tau| / rated) for each wheel arm.  Each wheel takes W/4 vertically (+ F_extra per wheel);
    gravity on the arm masses and on the wheel hand; the pod mass of cephalo4 is replaced by the wheel hand."""
    body_p = np.array([0, 0, h]); M_POD0 = C.M_POD; C.M_POD = 0.0
    out = {}
    for i, q in qs.items():
        segs, (R, p) = C.arm_fk(i, q, body_p)
        u = R[:, 0]
        contact = p + u * LEAD - np.array([0, 0, R_W]) if MODE == "axial" else p + u * L_COLUMN
        Fg = np.array([0, 0, W / 4]) + F_extra
        tau = C.joint_torques(i, q, body_p, np.eye(3), Fg, contact, acc=grav)
        tau += C.joint_torques(i, q, body_p, np.eye(3), M_WHEELHAND * grav, p + u * COM_WH, acc=np.zeros(3))
        out[i] = tau
    C.M_POD = M_POD0
    return out


def clearance(qs, h):
    """Smallest gap of the wheel arms' capsules to the floor and to the body; wheel to its own arm."""
    body_p = np.array([0, 0, h]); g_floor, g_body = 9.0, 9.0
    for i, q in qs.items():
        P, r, (R, p) = C.arm_capsules(i, q, body_p)
        g_floor = min(g_floor, float((P[:, 2] - r).min()))
        g_body = min(g_body, float(C.body_clearance(P, r, body_p).min()) if hasattr(C, "body_clearance") else 9.0)
        # collar / housing to the floor (axle level): handled by the CAD clearance (33 mm)
    return g_floor, g_body


def search():
    best = None
    hs = np.arange(0.16, 0.341, 0.02) if MODE == "axial" else np.arange(0.20, 0.501, 0.025)
    Xs = np.arange(0.10, 0.301, 0.025) if MODE == "axial" else np.arange(0.05, 0.301, 0.025)
    Ys = np.arange(0.30, 0.521, 0.025) if MODE == "axial" else np.arange(0.05, 0.401, 0.025)
    for h in hs:
        for X in Xs:
            for Y in Ys:
                qs, err = solve(h, X, Y)
                if err > 0.003: continue
                L = loads(qs, h, X, Y)
                f = max(np.max(np.abs(t) / C.RATED) for t in L.values())
                gf, gb = clearance(qs, h)
                if gf < (0.010 if MODE == "axial" else L_COLUMN - 0.02): continue
                if best is None or f < best[0]:
                    best = (f, h, X, Y, gf, gb)
    return best


if __name__ == "__main__":
    best = search()
    f, h, X, Y, gf, gb = best
    qs, err = solve(h, X, Y)
    res = dict(stance=dict(plate_height_m=h, wheel_x_m=X, wheel_y_m=Y, ik_error_m=err, arm_floor_gap_m=gf, track_m=2 * Y, wheelbase_m=2 * X),
               robot_mass_kg=M_ROBOT, wheel_hand_kg=M_WHEELHAND)
    cases = {"standing still": dict(),
             "driving: 15 deg slope + 1 m/s^2 (design)": dict(F_extra=np.array([SZ["loads"]["drive_force_design_N"], 0, 0])),
             "full traction (mu 0.8), forward": dict(F_extra=np.array([SZ["loads"]["traction_limit_N"], 0, 0])),
             "full braking": dict(F_extra=np.array([-SZ["loads"]["traction_limit_N"], 0, 0])),
             "0.3 g sideways (turning)": dict(F_extra=np.array([0, 0.3 * W / 4, 0])),
             "0.3 g sideways, other way": dict(F_extra=np.array([0, -0.3 * W / 4, 0])),
             "bump 2 g": dict(F_extra=np.array([0, 0, W / 4]))}
    for name, kw in cases.items():
        L = loads(qs, h, X, Y, **kw)
        frac = {i: (np.abs(t) / C.RATED) for i, t in L.items()}
        worst_arm = max(frac, key=lambda i: frac[i].max()); j = int(np.argmax(frac[worst_arm]))
        res[name] = dict(worst_load_vs_rated=round(float(frac[worst_arm][j]), 2), at=f"arm {worst_arm} joint {j} ({C.SERVO[j]})",
                         vs_stall=round(float(np.abs(L[worst_arm][j]) / C.STALL[j]), 2),
                         per_joint_worst=[round(float(max(frac[i][k] for i in frac)), 2) for k in range(8)])
    res["q_deg"] = {str(i): np.degrees(q).round(1).tolist() for i, q in qs.items()}
    print(json.dumps(res, indent=1))
    json.dump(res, open(f"wheelstance_{MODE}.json", "w"), indent=1)
    json.dump({"h": h, "X": X, "Y": Y, "q": {str(i): q.tolist() for i, q in qs.items()}, "q_up": Q_UP.tolist()}, open(f"wheelstance_q_{MODE}.json", "w"))
