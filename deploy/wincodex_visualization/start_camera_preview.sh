#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /home/ubuntu/car01/devel/setup.bash
if fuser /dev/video0 >/dev/null 2>&1; then echo '相机已占用，拒绝启动预览。' >&2; exit 1; fi
exec roslaunch /home/ubuntu/car01/deploy/wincodex_visualization/camera_preview.launch
