#!/bin/bash
cd /home/claude/octodrone
rm -f fvchunk_*.mp4
for p in "0 1" "2 3"; do set -- $p
  nice python3 fall_video.py chunk $1 4 > fvr$1.log 2>&1 & nice python3 fall_video.py chunk $2 4 > fvr$2.log 2>&1; wait; done
python3 fall_video.py join 4 && ffprobe -v error -show_entries format=duration -of csv=p=0 $FV_OUT
echo RENDERDONE
