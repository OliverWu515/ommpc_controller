#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/noetic/setup.bash
if [[ -f /catkin_ws/devel/setup.bash ]]; then
  source /catkin_ws/devel/setup.bash
fi
source /opt/PX4-Autopilot/Tools/setup_gazebo.bash \
  /opt/PX4-Autopilot /opt/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH="${ROS_PACKAGE_PATH}:/opt/PX4-Autopilot:/opt/PX4-Autopilot/Tools/sitl_gazebo"
set -u

exec roslaunch px4 mavros_posix_sitl.launch gui:=false interactive:=false
