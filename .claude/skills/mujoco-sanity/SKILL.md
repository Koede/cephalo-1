---
name: mujoco-sanity
description: Sanity-check the cephalo-1 arm in MuJoCo - loads, joint limits respected, no NaNs, energy drift with damping off, deterministic with a fixed seed. Use when dynamics.py or the chain model changes.
---
# mujoco-sanity

Install `mujoco>=3.1`. Use `joint-study/sim/dynamics.py` (and `chain.build_chain`) to build the model; step 1000 steps from a bent pose with gravity. Check: no NaN/inf, joint angles within limits, total energy drift < 1% over 1 s with damping 0, identical final state for two runs with the same seed, tip position matches `tools/kin.Kin.tip` at the initial pose within 0.1 mm. Report each check PASS/FAIL.
