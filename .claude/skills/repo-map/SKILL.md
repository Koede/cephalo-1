---
name: repo-map
description: Produce or refresh a table of every script in cephalo-1 with what it does, how to run it, inputs and outputs, and flag undocumented or broken ones. Use when orienting in the repo or after adding scripts.
---
# repo-map

`python3 tools/repo_map.py [--write SCRIPTS.md]` builds a markdown table (script, purpose, argparse flags, files it writes, problems) from the AST, so it is safe on any machine and never runs the scripts. Problems flagged: no docstring, hard-coded `/mnt/` paths, cwd-relative output dirs. Offer to save it as `SCRIPTS.md` or into the README; do not edit docs without being asked.
