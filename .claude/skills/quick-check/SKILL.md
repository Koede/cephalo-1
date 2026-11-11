---
name: quick-check
description: Fast (under 10 s) pre-commit check for cephalo-1 that does not regenerate CAD - unit tests and the committed-design gates. Use after small edits.
---
# quick-check

`python3 tools/check_all.py --quick` (about 8 s).

It does not regenerate CAD, so after editing `arm-pin-hinge/*.py` run `/check-all` instead.
