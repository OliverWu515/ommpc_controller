#ifndef OMMPC_CORE_CONTROLLER_CORE_HPP
#define OMMPC_CORE_CONTROLLER_CORE_HPP

#include "ommpc_core/mpc_wrapper.hpp"
#include "ommpc_core/polynomial_trajectory.h"
#include "ommpc_core/so3_math.h"

#include <Eigen/Sparse>

#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace ommpc_core
{

class OMMPCControllerCore
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OMMPCControllerCore();

  void init(const ParameterSet &param);

  bool execMPC(const ControllerState &odom, double now_s, ControlCommand &u);

  bool setHoverReference(const Eigen::Vector4d &quad_pose);
  bool setTextReference(const ReferenceTrajectory &reference,
                        const ControllerState &odom,
                        double start_yaw);
  bool setTrajectoryReference(const Trajectory &traj,
                              double tstart,
                              double start_yaw,
                              const Trajectory &yaw_traj,
                              const ControllerState &odom);

  void estimateThrustModel(const Eigen::Vector3d &est_a, double now_s);

  double getTimingMs() const;
  const std::string &getLastError() const;

private:
  static constexpr int kMaxStoredThrustSamples = 100;
  static constexpr double kThrustForgettingFactor = 0.998;
  static constexpr double kMinHoverPercentage = 0.1;
  static constexpr double kMaxHoverPercentage = 0.8;
  static constexpr double kMinEstimationDelayS = 0.035;
  static constexpr double kMaxEstimationDelayS = 0.045;
  static constexpr double kAlmostZeroValueThreshold = 1e-3;
  static constexpr double kMinNormalizedCollectiveThrust = 0.1;

  const double gravity_ = 9.81;

  // MPC param wrapper
  MpcWrapper mpc_wrapper_;

  // params
  ParameterSet param_;

  // for MPC timing
  double timing_feedback_ = 0.0;

  // calculate yaw
  double last_yaw_ = 0.0;
  double last_yaw_dot_ = 0.0;

  // thrust estimation
  double thr2acc_ = 0.0;
  double P_ = 1e6;
  std::queue<std::pair<double, double>> timed_thrust_;

  std::string last_error_;

  // MPC matrices and bounds
  std::vector<Eigen::SparseMatrix<double>> Fx_;
  std::vector<Eigen::SparseMatrix<double>> Fu_;
  std::vector<Eigen::VectorXd> u_lb_;
  std::vector<Eigen::VectorXd> u_ub_;

  // cache finite-difference acceleration/jerk for txt references
  Eigen::Vector3d last_text_des_acc_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_text_acc_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_text_des_jerk_ = Eigen::Vector3d::Zero();

  // Helper functions for calculating yaw and yawdot
  double angleDiff(double a, double b) const;
  void calculateYaw(Eigen::Vector3d vel, double dt, double &yaw, double &yawdot);

  // core functions for MPC setup
  void setStateMatricesAndBounds(int i,
                                 const Eigen::Quaterniond &q,
                                 const Eigen::Vector3d &omg,
                                 double t_step,
                                 double thracc);
  // reference computation
  void computeFlatInputwithHopfFibration(const Eigen::Vector3d &thr_acc,
                                         const Eigen::Vector3d &jer,
                                         double yaw,
                                         double yawd,
                                         const Eigen::Quaterniond &att_est,
                                         Eigen::Quaterniond &att,
                                         Eigen::Vector3d &omg) const;
   
  void setError(const std::string &error_message);
  void clearError();
};

} // namespace ommpc_core

#endif
