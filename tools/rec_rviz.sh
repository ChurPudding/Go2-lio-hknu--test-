#!/bin/bash
NAME=${1:-out}; DUR=${2:-120}
OUT=~/fastlio_ws/videos/${NAME}.mp4
WID=$(wmctrl -l | grep -i rviz | head -1 | awk '{print $1}')
if [ -z "$WID" ]; then echo "RViz 창을 못 찾음"; exit 1; fi
GEO=$(xwininfo -id "$WID" | awk '
  /Absolute upper-left X/{x=$4} /Absolute upper-left Y/{y=$4}
  /Width:/{w=$2} /Height:/{h=$2}
  END{printf "%dx%d+%d+%d", w-(w%2), h-(h%2), x, y}')
echo "녹화 $GEO -> $OUT (${DUR}s)"
ffmpeg -y -f x11grab -framerate 30 -video_size ${GEO%%+*} \
  -i ${DISPLAY}+$(echo $GEO | cut -d+ -f2-) -t "$DUR" \
  -c:v libx264 -preset veryfast -crf 22 -pix_fmt yuv420p "$OUT"
echo "완료 $OUT"
