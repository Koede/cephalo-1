"""Compare the landings: v6 (descend, leg 0, leg 4, touch, pairs one after another) vs v7 (organic: one continuous movement)."""
import json, subprocess, sys
res = {}
TAGS = ("v6", "v7")
FILE = {"v6": "sim4_mid_land6.npz", "v7": "sim4_mid.npz"}
for tag in TAGS:
    out = subprocess.run([sys.executable, "-c", f"""
import os; os.environ['CEPH_LANDING']='{tag}'
import numpy as np, json, sim4 as S
pl=S.Plan(); t=pl.t; Q=np.degrees(pl.Q)
t1=S.T_PAIRS_END
sel=(t>=S.T_DESC)&(t<=t1)
v=np.gradient(Q,0.01,axis=0); a=np.gradient(v,0.01,axis=0); j=np.gradient(a,0.01,axis=0)
legs=(np.abs(v).max(2)>10)
nleg=legs[sel].sum(1); idle=float((nleg==0).mean()*(t1-S.T_DESC))
mov=(np.abs(v[sel]).reshape(sel.sum(),-1)>10).sum(1)
Z=np.load('{FILE[tag]}'); tz=Z['t']
w=(tz>=S.T_DESC)&(tz<=S.T_END)&(np.r_[1,np.diff(tz)]>0.02)
tt=tz[w]; z=Z['qpos'][w,2]; tilt=Z['tilt'][w]; fan=Z['fan'][w]; srv=Z['servo_arm'][w]; slip=Z['slip'][w]
vz=np.gradient(z,tt); az=np.gradient(vz,tt); jz=np.gradient(az,tt)
# body pauses: descent speed below 1 cm/s for > 0.3 s between leaving hover and the final stand
low=(np.abs(vz)<0.01)&(tt>S.T_DESC+0.3)&(tt<S.T_TOUCH)
k=np.unravel_index(srv.argmax(),srv.shape)
kt=np.argmin(np.abs(tt-S.T_TOUCH))
print(json.dumps(dict(dur=float(t1-S.T_DESC), total=float(S.T_END-S.T_DESC), idle=idle, legs_mean=float(nleg[nleg>0].mean()), legs_max=int(nleg.max()),
  mov=float(mov[mov>0].mean()), movmax=int(mov.max()), vmax=float(np.abs(v[sel]).max()), amax=float(np.abs(a[sel]).max()),
  hold=float(low.sum()*np.median(np.diff(tt))), vz_touch=float(vz[max(kt-3,0):kt+4].min()), az_max=float(np.abs(az).max()),
  jz_rms=float(np.sqrt((jz**2).mean())), tilt_max=float(tilt.max()), srv=float(srv.max()), srv_t=float(tt[k[0]]), srv_arm=int(k[1]),
  slip=float(slip.max()), fanmax=float(fan.max()), z_end=float(z[-1]), tilt_end=float(tilt[-1]))))
"""], capture_output=True, text=True)
    if not out.stdout.strip(): print(out.stderr[-2000:])
    res[tag] = json.loads(out.stdout.strip().splitlines()[-1])
rows = [("Landing duration (leave hover -> last foot down)", "dur", "s", 1), ("  incl. EDF spool-down + winch tight", "total", "s", 1),
        ("Time with no leg moving", "idle", "s", 1), ("Body hovering still before touch-down", "hold", "s", 1),
        ("Legs moving at once (mean / max)", "legs_mean", "", 1), ("", "legs_max", "", 0),
        ("Joints moving at once (>10 deg/s, mean / max)", "mov", "", 1), ("", "movmax", "", 0),
        ("Peak commanded joint speed", "vmax", "deg/s", 0), ("Peak commanded joint accel", "amax", "deg/s^2", 0),
        ("Sink rate at touch-down", "vz_touch", "m/s", 3), ("Peak body vertical accel", "az_max", "m/s^2", 2),
        ("RMS body vertical jerk", "jz_rms", "m/s^3", 1), ("Max body tilt", "tilt_max", "deg", 1), ("Worst servo load", "srv", "x stall", 2),
        ("Max clutch slip", "slip", "deg", 1), ("Peak claw-prop thrust", "fanmax", "N", 1), ("Final plate height", "z_end", "m", 3),
        ("Final tilt", "tilt_end", "deg", 2)]
print(f"{'':52s}" + "".join(f"{t:>10s}" for t in TAGS))
for nm, k, u, dp in rows:
    print(f"{nm:52s}" + "".join(f"{res[t][k]:10.{dp}f}" for t in TAGS) + f"  {u}")
print("worst servo at", {k: (round(r['srv_t'], 2), r['srv_arm']) for k, r in res.items()})
json.dump(res, open("landing_compare.json", "w"), indent=1)
