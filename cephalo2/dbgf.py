import numpy as np, fall4 as FA, mujoco
F=FA.Fall("cut"); d=F.d
rec=[]; orig=F.fall_thrust
def ft(t):
    orig(t)
    if 1.62<t<1.80:
        b,l=F.floor_forces(); rec.append((t, b+l+F.toe_f, d.qvel[2], d.qacc[2] if True else 0))
F.fall_thrust=ft
F.run(1.85)
r=np.array(rec)
print("peak", r[:,1].max().round(0), "at", r[r[:,1].argmax(),0].round(4))
# impulse-averaged force over windows
for w in (0.002,0.005,0.01,0.02):
    n=int(w/ (r[1,0]-r[0,0])); 
    ma=np.convolve(r[:,1],np.ones(n)/n,'valid').max(); print("window",w,"max avg force",ma.round(0), "=", (ma/FA.C.W_ROBOT).round(1),"x W")
i0=np.argmax(r[:,1]>50); print("vz at contact",r[i0,2].round(2)); 
for j in range(i0, min(i0+400,len(r)),20): print(r[j,0].round(4), r[j,1].round(0), r[j,2].round(2))
