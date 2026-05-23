#ifndef OMMPC_ROS_MAVROS_COMMAND_INTERFACE_HPP
#define OMMPC_ROS_MAVROS_COMMAND_INTERFACE_HPP

#include <string>

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <ros/ros.h>

#include "ommpc_core/types.hpp"

namespace ommpc_ros
{

class MavrosCommandInterface
{
public:
  void init(ros::NodeHandle &nh);

  void publishAttitudeTarget(const ommpc_core::ControlCommand &output) const;

  bool armDisarm(bool arm, std::string *error_message = nullptr);
  bool setMode(const std::string &mode, std::string *error_message = nullptr);

private:
  ros::Publisher cmd_pub_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceClient arming_client_;
};

} // namespace ommpc_ros

#endif
