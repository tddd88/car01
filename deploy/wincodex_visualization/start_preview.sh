#!/usr/bin/env bash
set -e
D=/home/ubuntu/car01/deploy/wincodex_visualization
for n in bridge model_preview map_preview camera_preview debug_preview; do
if [[ -f "$D/$n.pid" ]] && kill -0 "$(cat "$D/$n.pid")" 2>/dev/null; then echo "$n 已运行"; continue; fi
nohup "$D/start_$n.sh" > "$D/$n.log" 2>&1 < /dev/null &
echo $! > "$D/$n.pid"
sleep 1
done
