---
name: skills-index
description: List all cephalo-1 project skills grouped by purpose with one-line usage. Use when asked what skills or checks exist in this repo.
---
# skills-index

Every skill runs a script in `tools/` (cross-platform; use `python` or `py -3` where `python3` is not defined) or a pytest test. Print the table below, then `ls .claude/skills` to catch skills added later.

- Setup: setup-env, doctor, repo-map (`tools/doctor.py`, `tools/repo_map.py`)
- Workflow: check-all, quick-check, regression-golden (`tools/check_all.py [--quick|--full]`, `tools/golden_diff.py`)
- Regenerate: cad-regen, regen-subassemblies, variant-compare, param-sweep, stale-output-clean
- Geometry checks: cad-fit-check, collision-check, clearance-map, stl-integrity, tolerance-stackup, interference-heatmap, fastener-check, helix-check, symmetry-8arm
- Motion: joint-range, joint-dof, singularity-scan, workspace-reach, fk-ik-playground, pose-render, gait-feasibility
- Loads: mass-budget, joint-torque-check, bearing-load-check, gear-check, thrust-weight, battery-endurance, prop-clearance, fluid-drag
- Study: study-smoke, study-full, study-compare, mujoco-sanity, rebuild-study-page, servo-catalog
- Docs/review: design-doc-sync, design-log-entry, changelog-from-diff, cad-review, print-prep, bom-export, geom-unit-tests

One-shot: `python3 tools/check_all.py`. Procedure-only (no script yet): study-compare, rebuild-study-page, design-log-entry, helix-check (pytest), geom-unit-tests (pytest).
