import numpy as np, sys
z=np.load(sys.argv[1] if len(sys.argv)>1 else 'flip4.npz'); st=int(sys.argv[2]) if len(sys.argv)>2 else 9
for j in range(0,len(z['t']),st):
    print(f"{z['t'][j]:5.1f} {z['phase'][j]:8s} up {z['up'][j]:4.0f} att {z['att'][j]:4.0f} z {z['qpos'][j][2]:5.2f} com {z['com'][j].round(2)} fan {z['fan'][j].round(0)} sv {z['servo_arm'][j].round(2)} slip {z['slip_arm'][j].max():4.0f} dome {z['dome'][j]:4.0f} st {z['stance'][j].sum()}")
