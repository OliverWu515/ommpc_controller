#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
cd /catkin_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
