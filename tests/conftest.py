import os, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (os.path.join(ROOT, "tools"), os.path.join(ROOT, "arm-pin-hinge"), os.path.join(ROOT, "spec-sheet")):
    sys.path.insert(0, p)
