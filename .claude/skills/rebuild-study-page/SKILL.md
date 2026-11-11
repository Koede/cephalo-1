---
name: rebuild-study-page
description: Rebuild the cephalo-1 joint-study HTML (build_page.py) from results JSON and check assets and links. Use after results change.
---
# rebuild-study-page

`cd joint-study/sim && python3 build_page.py ../results/cephalo-joint-study.html`; the writeup copy `joint-study/writeup/cephalo-joint-study.html` is committed separately, so diff both. Check `page/viewer.js` exists and that all `src`/`href` targets in the HTML resolve (`grep -o 'src="[^"]*"'`). Open it with the Playwright Chromium if asked for a screenshot (`deviceScaleFactor` 1). `git status` afterwards: tell the user what changed.
