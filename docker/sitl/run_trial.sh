#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

exec /catkin_ws/src/ommpc_controller/test/run_sitl_trial.sh "$@"
