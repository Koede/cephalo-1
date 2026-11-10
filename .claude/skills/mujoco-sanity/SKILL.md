---
name: mujoco-sanity
description: Sanity-check the cephalo-1 arm in MuJoCo - loads, joint limits respected, no NaNs, energy drift with damping off, deterministic with a fixed seed. Use when dynamics.py or the chain model changes.
---
# mujoco-sanity

`python3 tools/mujoco_sanity.py [--joints 13 --length 1000 --twist 60 --poses 20 --seconds 1]`

Builds an MJCF chain from the CAD's joint axes and checks: tip position equals `tools/kin.py` FK (error 2e-10 mm), the translational Jacobian equals `Kin.jacobian`, limits are respected (soft, 5 deg tolerance), no NaN over 2000 steps, energy conserved (no damping, no limits, no contacts) and deterministic reruns. It validates the kinematic model, not the servo dynamics. Needs `pip install mujoco`.
