#ifndef OMMPC_CORE_TYPES_HPP
#define OMMPC_CORE_TYPES_HPP

#include <Eigen/Eigen>

#include <cstddef>
#include <string>
#include <vector>

namespace ommpc_core
{

static constexpr int kDefaultHorizonSteps = 20;  // N steps
static constexpr int kErrorStateDim = 9;  // dimension of error state (δp, δv, δR)
static constexpr int kStateDim = 10;  // dimension of state (pos quat vel)
static constexpr int kInputDim = 4;   // dimension of control input (thrust omg)

enum class WorldFrame
{
  kEnu,
  kNed
};

enum class VelocityFrame
{
  kWorld,
  kBody
};

struct ControllerState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d p = Eigen::Vector3d::Zero();
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  double timestamp_s = 0.0;
};

struct ImuSample
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
  Eigen::Vector3d w = Eigen::Vector3d::Zero();
  Eigen::Vector3d a = Eigen::Vector3d::Zero();
  double timestamp_s = 0.0;
};

struct ReferencePoint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double time_from_start_s = 0.0;
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct ReferenceTrajectory
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  double sample_dt_s = 0.0;
  std::vector<ReferencePoint, Eigen::aligned_allocator<ReferencePoint>> points;

  void clear()
  {
    points.clear();
  }

  void resize(std::size_t size)
  {
    points.resize(size);
  }

  std::size_t size() const
  {
    return points.size();
  }

  bool empty() const
  {
    return points.empty();
  }

  ReferencePoint &operator[](std::size_t idx)
  {
    return points[idx];
  }

  const ReferencePoint &operator[](std::size_t idx) const
  {
    return points[idx];
  }
};

struct ControlCommand
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Eigen::Vector3d bodyrates = Eigen::Vector3d::Zero();
  double thrust = 0.0;
  double collective_acceleration = 0.0;
};

struct MPCParams
{
  int horizon_steps = kDefaultHorizonSteps;
  double step_T = 0.01;

  double Q_pos_xy = 0.0;
  double Q_pos_z = 0.0;
  double Q_velocity = 0.0;
  double Q_attitude_rp = 0.0;
  double Q_attitude_yaw = 0.0;

  double R_thrust = 0.0;
  double R_pitchroll = 0.0;
  double R_yaw = 0.0;

  double state_cost_exponential = 0.0;
  double input_cost_exponential = 0.0;

  double max_bodyrate_xy = 0.0;
  double max_bodyrate_z = 0.0;
  double min_thrust = 0.0;
  double max_thrust = 0.0;
};

struct TextReferenceConfig
{
  bool enable = false;
  double time_step = 0.01;
  std::string ref_filename;
};

struct ThrustNormalizationParams
{
  double hover_percentage = 0.25;
  bool enable_estimation = true;
};

struct DragCompensationParams
{
  bool enable = false;
  double mass = 1.0;  // kg
  // D = diag(d_h, d_h, d_v), in kg/s (N/(m/s)).
  double horizontal_coefficient = 0.0;
  double vertical_coefficient = 0.0;
  // Speed scaling: sigma(||v||) = 1 + c_p sqrt(||v||^2 + epsilon).
  // c_p is in s/m and is used by both flatness and the MPC drag Jacobians.
  double speed_coefficient = 0.0;
};

struct ControllerConfig
{
  bool use_fix_yaw = true;
  bool use_trajectory_ending_pos = true;

  WorldFrame world_frame = WorldFrame::kEnu;
  VelocityFrame velocity_frame = VelocityFrame::kBody;

  TextReferenceConfig text_reference;
  ThrustNormalizationParams thrust_normalization;

  double takeoff_land_speed = 0.0;
  double takeoff_height = 0.0;
};

struct ParameterSet
{
  MPCParams mpc;
  ControllerConfig controller;
  DragCompensationParams drag_compensation;
};

} // namespace ommpc_core

#endif
