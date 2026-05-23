#include "ommpc_ros/mavros_command_interface.hpp"

namespace ommpc_ros
{

static constexpr double kMaxOutputNormalizedThrust = 0.9;
static constexpr double kMinOutputNormalizedThrust = 0.04;

void MavrosCommandInterface::init(ros::NodeHandle &nh)
{
  cmd_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
  set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
}

void MavrosCommandInterface::publishAttitudeTarget(
    const ommpc_core::ControlCommand &output) const
{
  mavros_msgs::AttitudeTarget cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.body_rate.x = output.bodyrates(0);
  cmd.body_rate.y = output.bodyrates(1);
  cmd.body_rate.z = output.bodyrates(2);
  if (output.thrust >= kMaxOutputNormalizedThrust)
  {
    cmd.thrust = kMaxOutputNormalizedThrust;
  }
  else if (output.thrust <= kMinOutputNormalizedThrust)
  {
    cmd.thrust = kMinOutputNormalizedThrust;
  }
  else
  {
    cmd.thrust = output.thrust;
  }
  cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  cmd_pub_.publish(cmd);
}

bool MavrosCommandInterface::armDisarm(bool arm, std::string *error_message)
{
  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = arm;
  if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
  {
    return true;
  }

  if (error_message != nullptr)
  {
    *error_message = arm ? "ARM rejected by PX4!" : "DISARM rejected by PX4!";
  }
  return false;
}

bool MavrosCommandInterface::setMode(const std::string &mode, std::string *error_message)
{
  mavros_msgs::SetMode mode_cmd;
  mode_cmd.request.custom_mode = mode;
  if (set_mode_client_.call(mode_cmd) && mode_cmd.response.mode_sent)
  {
    return true;
  }

  if (error_message != nullptr)
  {
    *error_message = "Failed to set PX4 mode to " + mode;
  }
  return false;
}

} // namespace ommpc_ros
