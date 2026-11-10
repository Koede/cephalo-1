---
name: collision-check
description: Layered mesh-interference collision check of a cephalo-1 arm - static, each joint alone, joint pairs, random-pose Monte Carlo, mantle port cutters - naming the colliding links and the pose that reproduces it. Use after geometry changes or to probe a specific pose.
---
# collision-check

`python3 tools/collision.py [--level static,sweeps,pairs,mc,mantle|all] [--samples N --seed S] [--range 90] [--step 5] [--length L --joints N --twist T --no-gear]`

- Single pose: `--pose 0:30,1:-20,2:45` (joint:deg, rest 0) checks links within 3 of each other.
- Contact means interference volume > 2 mm^3 (pin_cad's threshold). Monte Carlo output prints `--pose ...` for the worst hit, so it can be replayed and rendered with `/pose-render`.
- Does not yet cover arm-vs-arm or arm-vs-hub beyond the mantle cutters; for those use `joint-study/sim/chain.py` (`pair_penetration`, `hub_penetration`) via `/study-smoke`.
- Exit 1 on any collision. ~3 s for static+sweeps+pairs+30 MC samples.
