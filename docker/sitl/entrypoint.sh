#!/usr/bin/env bash
set -e

source /opt/ros/noetic/setup.bash
if [[ -f /catkin_ws/devel/setup.bash ]]; then
  source /catkin_ws/devel/setup.bash
fi
source /opt/PX4-Autopilot/Tools/setup_gazebo.bash \
  /opt/PX4-Autopilot /opt/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:/opt/PX4-Autopilot:/opt/PX4-Autopilot/Tools/sitl_gazebo"

exec "$@"
