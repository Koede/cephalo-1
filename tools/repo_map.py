#!/usr/bin/env python3
"""Generate a table of every python script in the repo: purpose, flags, what it writes, and problems found.

  python3 tools/repo_map.py                  # markdown to stdout
  python3 tools/repo_map.py --write SCRIPTS.md
Purpose = first docstring line; flags = argparse add_argument names; writes = file names written via open(...,'w'),
.export(, json.dump, savefig, to_csv. Problems flagged: no docstring, hard-coded absolute paths, outputs relative to the
current directory rather than the script. Uses ast, no imports of the scripts (safe on any machine).
"""
import argparse, ast, os, re, subprocess, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from common import ROOT


def files():
    r = subprocess.run(["git", "ls-files", "*.py"], cwd=ROOT, capture_output=True, text=True)
    return sorted(r.stdout.split()) if r.returncode == 0 else sorted(os.path.relpath(os.path.join(d, f), ROOT) for d, _, fs in os.walk(ROOT) for f in fs if f.endswith(".py"))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--write", default=None)
    a = ap.parse_args(); rows = ["| script | purpose | flags | writes | problems |", "|---|---|---|---|---|"]
    for rel in files():
        src = open(os.path.join(ROOT, rel), encoding="utf8", errors="ignore").read()
        try: tree = ast.parse(src)
        except SyntaxError: rows.append(f"| `{rel}` | (syntax error) | | | FAIL |"); continue
        doc = (ast.get_docstring(tree) or "").strip().splitlines(); purpose = next((l.strip() for l in doc if l.strip() and not set(l.strip()) <= set("=-")), "")
        flags = sorted({n.args[0].value for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "attr", "") == "add_argument" and n.args and isinstance(n.args[0], ast.Constant)})
        writes = sorted(set(re.findall(r'["\']([\w./{}-]+\.(?:stl|json|csv|png|svg|dxf|html|md))["\']', src)))[:6]
        probs = []
        if not doc: probs.append("no docstring")
        if "/mnt/" in src: probs.append("hard-coded /mnt path")
        if re.search(r'makedirs\(["\']results', src): probs.append("cwd-relative output")
        rows.append(f"| `{rel}` | {purpose[:90]} | {' '.join(flags)[:60]} | {', '.join(writes)[:70]} | {'; '.join(probs)} |")
    out = "\n".join(rows) + "\n"
    if a.write: open(a.write, "w", encoding="utf8").write(out); print(f"wrote {a.write} ({len(rows)-2} scripts)")
    else: print(out)


if __name__ == "__main__":
    main()
