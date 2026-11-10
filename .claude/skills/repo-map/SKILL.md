---
name: repo-map
description: Produce or refresh a table of every script in cephalo-1 with what it does, how to run it, inputs and outputs, and flag undocumented or broken ones. Use when orienting in the repo or after adding scripts.
---
# repo-map

1. List scripts: `git ls-files '*.py' | sort`.
2. For each, read the module docstring and `argparse` flags (`grep -n "add_argument\|^\"\"\"" file`) and what files it writes (`grep -n "open(\|export(\|json.dump"`).
3. Output a markdown table: path, purpose, command, writes, run-from directory.
4. Flag: scripts without a docstring, hard-coded absolute paths (`/mnt/`), cwd-relative outputs, scripts that nothing imports or documents, committed outputs with no generator.
5. Offer to write the table into README.md under "Scripts"; do not edit without being asked.
