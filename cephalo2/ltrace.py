import numpy as np, sim4 as S, sys
Z=np.load(sys.argv[1] if len(sys.argv)>1 else 'sim4_mid.npz'); t=Z['t']; z=Z['qpos'][:,2]
print('T_DESC',S.T_DESC,'T_LAND0',S.T_LAND0,'T_TOUCH',S.T_TOUCH,'PAIRS',S.T_LAND_PAIRS,'PE',S.T_PAIRS_END)
w=(t>=S.T_DESC-0.2)&(t<=S.T_PAIRS_END+0.3)
tt=t[w]; zz=z[w]; vz=np.gradient(zz,tt)
for a in np.arange(S.T_TOUCH-1.0,S.T_PAIRS_END,0.1):
    k=np.argmin(abs(tt-a)); print(f"{tt[k]:6.2f} z {zz[k]:.3f} vz {vz[k]:+.3f} tilt {Z['tilt'][w][k]:.1f} fan {np.round(Z['fan'][w][k],1)} srv {np.round(Z['servo_arm'][w][k],2)}")
