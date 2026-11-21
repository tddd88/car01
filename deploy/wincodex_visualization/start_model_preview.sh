#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /home/ubuntu/car01/devel/setup.bash
if rostopic list | grep -Eq '^/tf(_static)?$'; then echo '已存在TF，拒绝启动模型预览以避免竞争。' >&2; exit 1; fi
exec roslaunch /home/ubuntu/car01/deploy/wincodex_visualization/model_preview.launch
