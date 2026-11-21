---
name: cad-regen
description: Regenerate a cephalo-1 CAD variant (main 1 m/13-joint geared design, 585 mm, 900 mm, twist 90) into a fresh scratch directory in the correct order. Use when CAD parameters or scripts changed and you need new STLs and reports.
---
# cad-regen

`python3 tools/regen.py [--variant main|585|900|twist90] [--out DIR] [--length L --joints N --twist T --root-servos K] [--no-sweep]`

- Order matters: `root_gear.py` calls `pin_cad.main()` itself and must be given `--length` and `--joints`; a plain `pin_cad.py` run gives the non-geared socket.
- Runs from `arm-pin-hinge/` into a **fresh** directory (default `<tempdir>/cephalo/regen/<variant>`, i.e. `/tmp/cephalo/regen/...` on Linux/macOS), so a stale `servo_lid_x13.stl` cannot survive (root_gear renames the lid to x12).
- Refuses to overwrite committed `arm-pin-hinge/out*` unless `--force`. Do not use `--force` unless the user asks to update the committed outputs.
- Main design takes about 5 s. Follow with `/cad-fit-check` and `/golden-diff`.
