---
name: quick-check
description: Fast (under 10 s) pre-commit check for cephalo-1 that does not regenerate CAD - unit tests and the committed-design gates. Use after small edits.
---
# quick-check

```bash
python3 -m pytest -q tests && python3 tools/check_gates.py && python3 tools/collision.py --level static,pairs
```

Report failures with the failing line. If `arm-pin-hinge/*.py` changed, recommend `/check-all` because these checks read the committed outputs, not a fresh build.
