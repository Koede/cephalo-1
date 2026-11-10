"""Moves between the stance poses when walking on the folded fan pod (Forest, Nov 10: "Yes, check the moves").

Builds on pod_stance.py (same arm, foot, parked prop and clearances; hub 250 mm up, 7 of 8 feet down). A step for
one arm, traced pose by pose with each pose solved from the last (joint moves between neighbouring poses penalised so
the path is one the servos can follow):
  stance stroke : the body moves 90 mm over the planted finger, along three headings (sideways, in-out, diagonal)
  unload / lift : the foot load ramps off, the finger rises 45 mm
  swing         : the finger travels back to the front of the stroke, 45 mm up
  set down      : the finger comes down and the load ramps on
  fold / unfold : the arm lifts clear and the pod swings 0 to 88 deg with the prop stopped (any blade angle until the
                  parking collar seats in the last 1.6 deg); pod servo load from the pod's weight and the post spring
Neighbour arms are taken in the nominal stance, 45 deg either side. Statics only: walking is slow, but a 1.5x
dynamic allowance is applied in the summary [A]. Writes results/pod_gait.json.
"""
import os, sys, json, math, time
os.environ.setdefault("JOINT_SCENARIO", "more_thrust")
HERE = os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0, HERE)
import numpy as np
from scipy.optimize import minimize
import config as C, chain as CH, metrics as M, central_prop as CP, pod_stance as PS

H = 0.25; LIFT = 0.045; STROKE = 0.090; DYN = 1.5
W_MOVE = float(os.environ.get("W_MOVE", "0.3"))     # penalty on joint moves between neighbouring poses (rad^-2)
POD_KG = (13.6 + 132 + 26 + 3) / 1000          # moving pod: printed pod, motor, prop, collar and adapter [fan_pod.json]
POD_CG = 38.2e-3                               # pod CG along the fan axis from the hinge
ZP = 0.034; POD_STALL = 0.441; SPRING_NM = 0.115; PARK_FROM = 86.4
DISC_R = PS.BLADE_R

def pod_frame(th):
    t = math.radians(th); return np.array([0, math.sin(t), math.cos(t)])      # fan axis in the tip frame, +th folds to +y

class Gait(PS.Arm):
    def torques_pod(s, q, fw, th):
        """joint torques with the pod at fold angle th (its weight moves with it); foot load fw on the finger"""
        m, eo, ea, R = s.fk(q); cen = 0.5 * (m[:-1] + m[1:]); p = m[-1] + R @ PS.FOOT_OFF; F = np.array([0, 0, fw])
        cg = m[-1] + R @ (np.array([0, 0, ZP]) + POD_CG * pod_frame(th)); fixed = s.tip - POD_KG
        out = np.zeros(13)
        for j in range(13):
            e = s.es[j]; k = s.ch.enode[e]; o = eo[e]; a = ea[e] / np.linalg.norm(ea[e])
            Mv = (np.cross(cen[k:] - o, s.masses[k:, None] * s.g).sum(0) + np.cross(m[-1] - o, fixed * s.g)
                  + np.cross(cg - o, POD_KG * s.g) + np.cross(p - o, F))
            out[j] = a @ Mv
        # pod servo: pod weight about the hinge axis (tip-frame x)
        hinge = m[-1] + R @ np.array([0, 0, ZP]); pod_t = float((R[:, 0]) @ np.cross(cg - hinge, POD_KG * s.g))
        return out, pod_t

def neighbours(A, q_nom):
    m, R, p, bl = A.geom(q_nom)
    return [(PS.rot_z(m, a), PS.rot_z(np.array(bl), a)) for a in (math.pi / 4, -math.pi / 4)]

def geo_cons(A, q, nb, parked=True, th=88.0, foot_free=False):
    """clearances (>= 0 is clear), in m"""
    m, R, p, bl = A.geom(q); ch = A.ch
    pts = PS.arm_pts(m[1:]); rz = np.c_[np.hypot(pts[:, 0], pts[:, 1]), pts[:, 2]]
    c = [CP.disc_gap(rz, PS.DISC_R, PS.DISC_Z) - CP.MARGIN]
    c += list(np.sort(pts[:, 2] - (-H + 0.040))[:4])
    g = 1.0
    for nm, nbl in nb:
        for i in range(1, len(m) - 1):
            for j in range(1, len(nm) - 1):
                g = min(g, CH.seg_dist(m[i], m[i + 1], nm[j], nm[j + 1]) - ch.seg_r[i] - ch.seg_r[j])
            g = min(g, CH.seg_dist(nbl[0], nbl[1], m[i], m[i + 1]) - ch.seg_r[i] - 0.010)
    if parked:
        c += [R[:, 2] @ np.array([0, 0, -1.0]) - math.cos(PS.TILT)]
        c += [b[2] + H - PS.BLADE_MARGIN for b in bl]
        c += [PS.blade_disc_gap(bl)]
        for nm, nbl in nb:
            for i in range(1, len(nm) - 1): g = min(g, CH.seg_dist(bl[0], bl[1], nm[i], nm[i + 1]) - ch.seg_r[i] - 0.010)
            g = min(g, CH.seg_dist(bl[0], bl[1], nbl[0], nbl[1]) - 0.020)
        c += [A.motor(q)[2] + H - (PS.MOTOR_R + 0.005)]
    else:
        c += list(disc_clear(A, q, nb, th))
    c += [g]
    c += [-CH.self_penetration(m, ch.seg_r, 0.9) - CH.hub_penetration(m, ch.seg_r, M.HUB_R_COLL)]   # the arm must not pass through itself
    return np.array(c)

def disc_pts(A, q, th, n=24):
    """rim and spokes of the full prop disc (any blade angle) at fold angle th, world frame"""
    m, eo, ea, R = A.fk(q)
    ax = pod_frame(th); hub = np.array([0, 0, ZP]) + (0.102 - ZP) * ax       # prop plane 102 mm above the tip plate when straight
    u = np.array([1.0, 0, 0]); v = np.cross(ax, u)
    pts = [hub + r * (math.cos(t) * u + math.sin(t) * v) for t in np.linspace(0, 2 * math.pi, n, endpoint=False) for r in (0.06, 0.13, DISC_R)]
    return m, R, np.array([m[-1] + R @ x for x in pts]), m[-1] + R @ hub, R @ ax

def disc_clear(A, q, nb, th):
    m, R, P, hub, ax = disc_pts(A, q, th)
    ground = P[:, 2].min() - (-H) - 0.020
    rz = np.c_[np.hypot(P[:, 0], P[:, 1]), P[:, 2]]
    central = CP.disc_gap(rz, PS.DISC_R, PS.DISC_Z) + CP.ARM_R - 0.010
    def seg_to_disc(a, b, r):     # closest approach of a segment (tube radius r) to the disc, sampled
        best = 1.0
        for t in np.linspace(0, 1, 7):
            x = a + t * (b - a) - hub; h = x @ ax; rad = np.linalg.norm(x - h * ax)
            d = abs(h) if rad <= DISC_R else math.hypot(rad - DISC_R, h)
            best = min(best, d - r)
        return best
    own = min(seg_to_disc(m[i], m[i + 1], A.ch.seg_r[i]) for i in range(1, len(m) - 2)) - 0.010   # last link checked in CAD
    nbg = 1.0
    for nm, nbl in nb:
        for i in range(1, len(nm) - 1): nbg = min(nbg, seg_to_disc(nm[i], nm[i + 1], A.ch.seg_r[i]) - 0.010)
        nbg = min(nbg, seg_to_disc(nbl[0], nbl[1], 0.005) - 0.010)
    return np.array([ground, central, own, nbg])

def track(A, q_prev, caps, nb, foot=None, fw=0.0, parked=True, ths=(88.0,), w=0.02, x0=None):
    """one pose on a path: minimise the worst need/stall plus a penalty on joint moves from q_prev"""
    ch = A.ch
    def need(q):
        return float(np.max([np.abs(A.torques_pod(q, fw, th)[0]) / caps for th in ths]))
    def cn(x): return np.array([x[-1] - need(x[:-1])])
    def cg(x):
        q = x[:-1]
        if parked:
            c = geo_cons(A, q, nb)
            if BETWEEN_CONS:      # the straight servo move from q_prev must stay clear too
                c = np.concatenate([c] + [geo_cons(A, q_prev + s_ * (q - q_prev), nb) for s_ in (0.25, 0.5, 0.75)])
            return c * 10
        return np.concatenate([geo_cons(A, q, nb, parked=False, th=t) for t in ths]) * 10
    cons = [dict(type="ineq", fun=cn), dict(type="ineq", fun=cg)]
    if foot is not None:
        cons.append(dict(type="eq", fun=lambda x: (A.geom(x[:-1])[2] - foot) * 10))
    q0 = q_prev if x0 is None else x0
    xs = np.r_[q0, need(q0)]
    r = minimize(lambda x: x[-1] + w * np.sum((x[:-1] - q_prev) ** 2), xs, method="SLSQP",
                 bounds=list(zip(ch.lo, ch.hi)) + [(0, 5)], constraints=cons, options=dict(maxiter=300, ftol=1e-7))
    q = r.x[:-1]
    geo = cg(r.x) / 10; ok_foot = True if foot is None else bool(np.linalg.norm(A.geom(q)[2] - foot) < 2e-3)
    pen = CH.self_penetration(A.fk(q)[0], ch.seg_r, 0.9) + CH.hub_penetration(A.fk(q)[0], ch.seg_r, M.HUB_R_COLL)
    return q, dict(need=need(q), clear_mm=float(geo.min() * 1e3), ok=bool(geo.min() > -1e-3 and ok_foot and pen < 1e-3))

def row(A, q, caps, fw, th=88.0):
    t, pod = A.torques_pod(q, fw, th); n = np.abs(t) / caps
    return dict(need=[round(float(x), 3) for x in n], pod_servo=round(abs(pod) / POD_STALL, 3))

def between(A, qa, qb, caps, nb, fw, parked, th, n=4):
    """the servos move straight from qa to qb: worst load, least clearance and finger slip on the way"""
    f_a = A.geom(qa)[2]; f_b = A.geom(qb)[2]; worst_i, clear_i, slip = 0.0, 1.0, 0.0
    for s_ in np.linspace(0, 1, n + 2)[1:-1]:
        qi = qa + s_ * (qb - qa); fi = A.geom(qi)[2]
        worst_i = max(worst_i, float((np.abs(A.torques_pod(qi, fw, th)[0]) / caps).max()))
        clear_i = min(clear_i, float(geo_cons(A, qi, nb, parked, th).min()))
        slip = max(slip, float(np.linalg.norm(fi - (f_a + s_ * (f_b - f_a)))))
    return worst_i, clear_i, slip

BETWEEN_CONS = os.environ.get("BETWEEN_CONS", "0") == "1"   # also constrain the poses on the straight move from the last pose
SUB_MAX = int(os.environ.get("SUB_MAX", "4"))     # most extra waypoints the servo controller gets inside one step

def solve_step(A, q, caps, nb, foot, fw, parked, ths, w):
    qn, info = track(A, q, caps, nb, foot, fw, parked, ths, w=w)
    if not info["ok"]:          # retry from a few nudged starts, keep the best feasible
        best = None
        for k in range(4):
            x0 = np.clip(qn + np.random.default_rng(k).normal(0, 0.15, len(q)), A.ch.lo, A.ch.hi)
            q2, i2 = track(A, q, caps, nb, foot, fw, parked, ths, w=w, x0=x0)
            if i2["ok"] and (best is None or i2["need"] < best[1]["need"]): best = (q2, i2)
        if best: qn, info = best
    return qn, info

def path(A, q_start, caps, nb, steps, w=None):
    """steps: list of (label, foot or None, fw, parked, ths). A step whose straight servo move passes through a clash
    (or slips the finger) is split into solved waypoints, up to SUB_MAX, so the controller follows a checked path."""
    q = q_start.copy(); out = []; prev = None
    for lab, foot, fw, parked, ths in steps:
        w = W_MOVE if w is None else w
        qn, info = solve_step(A, q, caps, nb, foot, fw, parked, ths, w)
        bw, bc, bs = between(A, q, qn, caps, nb, fw, parked, ths[-1])
        subs = 0; chain = [qn]
        if (bc < -2e-3 or bs > 3e-3) and prev is not None and foot is not None:
            f_prev, fw_prev = prev
            for k in range(1, SUB_MAX + 1):
                qq, ok_all, cw, cc, cs, ch_ = q, True, 0.0, 1.0, 0.0, []
                for j in range(1, k + 2):
                    s_ = j / (k + 1)
                    qj, ij = solve_step(A, qq, caps, nb, f_prev + s_ * (foot - f_prev), fw_prev + s_ * (fw - fw_prev), parked, ths, w)
                    a, c, sl = between(A, qq, qj, caps, nb, fw, parked, ths[-1])
                    cw, cc, cs = max(cw, a, ij["need"]), min(cc, c, ij["clear_mm"] / 1e3), max(cs, sl)
                    ok_all &= ij["ok"]; ch_.append(qj); qq = qj
                if ok_all and cc > -2e-3 and cs <= 3e-3:
                    qn, info, bw, bc, bs, subs, chain = qq, ij, cw, cc, cs, k, ch_
                    break
        r = dict(label=lab, ok=info["ok"], worst=round(info["need"], 3), clear_mm=round(info["clear_mm"], 1),
                 move_deg=round(float(np.degrees(np.abs(qn - q).max())), 1), q_deg=[round(float(v), 1) for v in np.degrees(qn)])
        r.update(row(A, qn, caps, fw, ths[-1]))
        r.update(between=dict(worst=round(bw, 3), clear_mm=round(bc * 1e3, 1), finger_off_path_mm=round(bs * 1e3, 1)), waypoints_added=subs)
        if subs: r["waypoints_deg"] = [[round(float(v), 1) for v in np.degrees(c)] for c in chain[:-1]]
        out.append(r); q = qn; prev = (foot, fw) if foot is not None else None
        if os.environ.get("VERBOSE"): print("  step", lab, r["ok"], r["worst"], r["clear_mm"], r["between"], "wp", subs, flush=True)
    return q, out

def main():
    t0 = time.time(); np.random.seed(0)
    st = json.load(open(os.path.join(HERE, "..", "results", "pod_stance.json")))
    case = st["cases"]["hub 250 mm above ground"]
    A = Gait(); caps = PS.caps_for(set()); q_nom = np.radians(case["q_deg"][0])
    nb = neighbours(A, q_nom); f0 = A.geom(q_nom)[2]; fw = A.fw
    rad = f0[:2] / np.linalg.norm(f0[:2]); tan = np.array([-rad[1], rad[0]])
    out = dict(assumptions=dict(hub_mm=H * 1e3, lift_mm=LIFT * 1e3, stroke_mm=STROKE * 1e3, foot_N=round(fw, 1), dynamic_allowance=DYN,
                                pod_kg=POD_KG, pod_cg_mm=POD_CG * 1e3, pod_servo_stall=POD_STALL, spring_Nm=SPRING_NM), phases={})
    heads = dict(sideways=tan, in_out=rad, diagonal=(tan + rad) / math.sqrt(2))
    only = sys.argv[1] if len(sys.argv) > 1 else None
    shift = float(sys.argv[2]) / 1000 if len(sys.argv) > 2 else 0.0      # move the stroke's centre outward, m
    stroke = float(sys.argv[3]) / 1000 if len(sys.argv) > 3 else STROKE
    if only: heads = {only: heads[only]}
    f0 = f0 + np.r_[rad * shift, 0.0]
    out["assumptions"].update(w_move=W_MOVE, centre_shift_out_mm=shift * 1e3, stroke_mm=stroke * 1e3)
    n = 9
    for hn, d in heads.items():
        d3 = np.r_[d, 0.0]
        front = f0 + d3 * stroke / 2; back = f0 - d3 * stroke / 2
        q, _ = path(A, q_nom, caps, nb, [("to front", front, fw, True, (88.0,))])
        q_front = q
        steps = [(f"stance {i}/{n-1}", front + (back - front) * i / (n - 1), fw, True, (88.0,)) for i in range(n)]
        steps += [(f"unload {k}", back, fw * k, True, (88.0,)) for k in (0.5, 0.0)]
        up = np.array([0, 0, LIFT])
        steps += [(f"lift {i}/3", back + up * i / 3, 0.0, True, (88.0,)) for i in (1, 2, 3)]
        steps += [(f"swing {i}/{n-1}", back + up + (front - back) * i / (n - 1), 0.0, True, (88.0,)) for i in range(1, n)]
        steps += [(f"set down {i}/3", front + up * (3 - i) / 3, 0.0, True, (88.0,)) for i in (1, 2, 3)]
        steps += [(f"load {k}", front, fw * k, True, (88.0,)) for k in (0.5, 1.0)]
        q, rows = path(A, q_front, caps, nb, steps)
        out["phases"][hn] = rows
        w = max(rows, key=lambda r: r["worst"])
        print(f"{hn}: worst {w['worst']} at {w['label']} | max move {max(r['move_deg'] for r in rows)} deg | all ok {all(r['ok'] for r in rows)}"
              f" | j6 {max(r['need'][6] for r in rows):.2f} j9 {max(r['need'][9] for r in rows):.2f} j11 {max(r['need'][11] for r in rows):.2f} j12 {max(r['need'][12] for r in rows):.2f}"
              f"  t={time.time()-t0:.0f}s", flush=True)
        print(f"   between poses: worst {max(r['between']['worst'] for r in rows):.2f}, least clear {min(r['between']['clear_mm'] for r in rows)} mm,"
              f" finger off path {max(r['between']['finger_off_path_mm'] for r in rows)} mm, waypoints added {[(r['label'], r['waypoints_added']) for r in rows if r['waypoints_added']]}", flush=True)
        bad = [(r["label"], r["worst"], r["clear_mm"]) for r in rows if not r["ok"] or r["worst"] > 1 / DYN]
        if bad: print("   flagged:", bad, flush=True)
    tag = ("" if not only else f"_{only}_{int(shift*1e3)}_{int(stroke*1e3)}") + os.environ.get("TAG", "")
    json.dump(out, open(os.path.join(HERE, "..", "results", f"pod_gait{tag}.json"), "w"), indent=1, default=float)

if __name__ == "__main__":
    main()
