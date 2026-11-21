---
name: print-prep
description: Pre-print checklist for cephalo-1 - values to measure on the real parts (the [A] assumptions), STL inventory per arm x8, material and support notes, integrity check. Use before printing a prototype.
---
# print-prep

`python3 tools/print_prep.py [DIR] [--arms 8] [--bed 250]`

Lists every `[A]` parameter in `pin_cad.py` and `root_gear.py` with file:line (measure these first), the printable parts with quantity for the robot, bounding box and mass (about 4.25 kg PETG for 8 arms), flags parts bigger than the bed, and the STL integrity result. Machined 7075 parts (gear, pinion, idler pin) are reference models. Exit 1 if a part exceeds the bed or an STL fails.
