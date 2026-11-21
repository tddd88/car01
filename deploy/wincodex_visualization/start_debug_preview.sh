#!/usr/bin/env bash
set -e
source /opt/ros/noetic/setup.bash
source /home/ubuntu/car01/devel/setup.bash
exec roslaunch /home/ubuntu/car01/deploy/wincodex_visualization/debug_preview.launch
