#!/usr/bin/env bash
set -e
D=/home/ubuntu/car01/deploy/wincodex_visualization
for n in debug_preview camera_preview map_preview model_preview; do
if [[ -f "$D/$n.pid" ]]; then
p=$(cat "$D/$n.pid")
if [[ -r /proc/$p/cmdline ]] && tr '\0' ' ' < /proc/$p/cmdline | grep -q "$D/$n.launch"; then kill -INT "$p"; fi
fi
done
