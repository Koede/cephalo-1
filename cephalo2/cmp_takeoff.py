"""Compare take-offs v5 (stepwise), v6 (fluid wave) and v7 (organic + fast) from the full-robot sim runs + commanded paths."""
import os, json, numpy as np, subprocess, sys
res = {}
TAGS = ("v5", "v6", "v7")
FILE = {"v5": "sim4_mid_v5.npz", "v6": "sim4_mid_v6.npz", "v7": "sim4_mid.npz"}
for tag in TAGS:
    out = subprocess.run([sys.executable, "-c", f"""
import os; os.environ['CEPH_TAKEOFF']='{tag}'
import numpy as np, json, sim4 as S
pl=S.Plan(); t=pl.t; Q=np.degrees(pl.Q)
sel=(t>=S.T_TO)&(t<=S.T_LEG0[1])
v=np.gradient(Q,0.01,axis=0); a=np.gradient(v,0.01,axis=0); j=np.gradient(a,0.01,axis=0)
sp=np.abs(v).max(2)
stops=0
for i in range(8):
    s=sp[sel][:,i]; pk=s.max()
    for n in range(1,len(s)-1):      # a stop = speed dips below 10 % of peak and the leg then moves again
        if s[n]<s[n-1] and s[n]<=s[n+1] and s[n]<0.1*pk and s[n+1:].max()>0.3*pk: stops+=1
Z=np.load('{FILE[tag]}'); tz=Z['t']
mov=(np.abs(v[sel]).reshape(len(v[sel]),-1)>10).sum(1)
w=(tz>=S.T_TO)&(tz<=S.T_LEG0[1]+0.6)&(np.r_[1,np.diff(tz)]>0.02)
tt=tz[w]; z=Z['qpos'][w,2]; tilt=Z['tilt'][w]; fan=Z['fan'][w]; srv=Z['servo_arm'][w]
dt=np.gradient(tt); vz=np.gradient(z,tt); az=np.gradient(vz,tt); jz=np.gradient(az,tt)
k=np.unravel_index(srv.argmax(),srv.shape)
print(json.dumps(dict(dur=float(S.T_LEG0[1]-S.T_TO), vmax=float(np.abs(v[sel]).max()), amax=float(np.abs(a[sel]).max()),
  jrms=float(np.sqrt((j[sel]**2).mean())), stops=stops, az_max=float(np.abs(az).max()), jz_rms=float(np.sqrt((jz**2).mean())),
  tilt_max=float(tilt.max()), tiltrate=float(np.abs(np.gradient(tilt,tt)).max()), dFdt=float(np.abs(np.gradient(fan,tt,axis=0)).max()),
  srv=float(srv.max()), srv_t=float(tt[k[0]]), srv_arm=int(k[1]), mov=float(mov.mean()))))
"""], capture_output=True, text=True)
    res[tag] = json.loads(out.stdout.strip().splitlines()[-1])
rows = [("Take-off duration (EDF start -> last leg up)", "dur", "s", 1), ("Stop-and-go points inside leg swings", "stops", "", 0),
        ("Peak commanded joint speed", "vmax", "deg/s", 0), ("Peak commanded joint accel", "amax", "deg/s^2", 0),
        ("RMS commanded joint jerk", "jrms", "deg/s^3", 0), ("Peak body vertical accel", "az_max", "m/s^2", 2),
        ("RMS body vertical jerk", "jz_rms", "m/s^3", 1), ("Max body tilt", "tilt_max", "deg", 1), ("Peak tilt rate", "tiltrate", "deg/s", 1),
        ("Peak fan thrust rate", "dFdt", "N/s", 0), ("Worst servo load", "srv", "x stall", 2), ("Joints moving at once (>10 deg/s, mean)", "mov", "", 1)]
print(f"{'':48s}" + "".join(f"{t:>10s}" for t in TAGS))
for nm, k, u, dp in rows:
    print(f"{nm:48s}" + "".join(f"{res[t][k]:10.{dp}f}" for t in TAGS) + f"  {u}")
print("worst servo at", {k: (round(r['srv_t'], 2), r['srv_arm']) for k, r in res.items()})
json.dump(res, open("takeoff_compare.json", "w"), indent=1)
