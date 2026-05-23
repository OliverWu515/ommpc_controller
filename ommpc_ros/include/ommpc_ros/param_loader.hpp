#ifndef OMMPC_ROS_PARAM_LOADER_HPP
#define OMMPC_ROS_PARAM_LOADER_HPP

#include <string>

#include <ros/ros.h>

#include "ommpc_core/types.hpp"

namespace ommpc_ros
{

class ParameterLoader
{
public:
  static void load(ros::NodeHandle &nh, ommpc_core::ParameterSet &param)
  {
    param.mpc.horizon_steps = ommpc_core::kDefaultHorizonSteps;

    bool enu_frame, vel_in_body;
    readEssentialParam(nh, "vel_in_body", vel_in_body);
    readEssentialParam(nh, "enu_frame", enu_frame);
    if (enu_frame)
    {
        param.controller.world_frame = ommpc_core::WorldFrame::kEnu;
    }
    else
    {
        param.controller.world_frame = ommpc_core::WorldFrame::kNed;
    }
    if (vel_in_body)
    {
        param.controller.velocity_frame = ommpc_core::VelocityFrame::kBody;
    }
    else    {
        param.controller.velocity_frame = ommpc_core::VelocityFrame::kWorld;
    }
    readEssentialParam(nh, "takeoff_height", param.controller.takeoff_height);
    readEssentialParam(nh, "takeoff_land_speed", param.controller.takeoff_land_speed);
    readEssentialParam(nh, "ref_txt/enable", param.controller.text_reference.enable);
    readEssentialParam(nh, "ref_txt/time_step", param.controller.text_reference.time_step);
    readEssentialParam(nh, "ref_txt/ref_filename", param.controller.text_reference.ref_filename);
    readEssentialParam(
        nh, "hover_percentage", param.controller.thrust_normalization.hover_percentage);
    readEssentialParam(nh, "MPC_params/Q_pos_xy", param.mpc.Q_pos_xy);
    readEssentialParam(nh, "MPC_params/Q_pos_z", param.mpc.Q_pos_z);
    readEssentialParam(nh, "MPC_params/Q_attitude_rp", param.mpc.Q_attitude_rp);
    readEssentialParam(nh, "MPC_params/Q_attitude_yaw", param.mpc.Q_attitude_yaw);
    readEssentialParam(nh, "MPC_params/Q_velocity", param.mpc.Q_velocity);
    readEssentialParam(nh, "MPC_params/R_thrust", param.mpc.R_thrust);
    readEssentialParam(nh, "MPC_params/R_pitchroll", param.mpc.R_pitchroll);
    readEssentialParam(nh, "MPC_params/R_yaw", param.mpc.R_yaw);
    readEssentialParam(nh, "MPC_params/min_thrust", param.mpc.min_thrust);
    readEssentialParam(nh, "MPC_params/max_thrust", param.mpc.max_thrust);
    readEssentialParam(nh, "MPC_params/max_bodyrate_xy", param.mpc.max_bodyrate_xy);
    readEssentialParam(nh, "MPC_params/max_bodyrate_z", param.mpc.max_bodyrate_z);
    readEssentialParam(
        nh, "MPC_params/state_cost_exponential", param.mpc.state_cost_exponential);
    readEssentialParam(
        nh, "MPC_params/input_cost_exponential", param.mpc.input_cost_exponential);
    readEssentialParam(nh, "MPC_params/step_T", param.mpc.step_T);
    readEssentialParam(nh, "use_fix_yaw", param.controller.use_fix_yaw);
    readEssentialParam(
        nh, "use_trajectory_ending_pos", param.controller.use_trajectory_ending_pos);
  }

private:
  template <typename T>
  static void readEssentialParam(ros::NodeHandle &nh, const std::string &name, T &val)
  {
    if (!nh.getParam(name, val))
    {
      ROS_ERROR_STREAM("Read param_: " << name << " failed.");
      ROS_BREAK();
    }
  }
};

} // namespace ommpc_ros

#endif
