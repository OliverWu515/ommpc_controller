#include "ommpc_core/controller_core.hpp"

#include <algorithm>
#include <cmath>
#include <ctime>

namespace ommpc_core
{

OMMPCControllerCore::OMMPCControllerCore() = default;

void OMMPCControllerCore::init(const ParameterSet &param)
{
  param_ = param;
  clearError();

  specific_drag_matrix_.setZero();
  if (param_.drag_compensation.enable)
  {
    const double horizontal_specific_drag =
        param_.drag_compensation.horizontal_coefficient / param_.drag_compensation.mass;
    const double vertical_specific_drag =
        param_.drag_compensation.vertical_coefficient / param_.drag_compensation.mass;
    specific_drag_matrix_.diagonal() << horizontal_specific_drag,
        horizontal_specific_drag,
        vertical_specific_drag;
  }

  // Thrust-accel mapping initial value
  thr2acc_ = gravity_ / param_.controller.thrust_normalization.hover_percentage;
  P_ = 1e6;

  // Timing / yaw cache
  timing_feedback_ = 0.0;
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;
  yaw_reference_initialized_ = false;
  last_text_des_acc_.setZero();
  last_text_acc_.setZero();
  last_text_des_jerk_.setZero();

  // set gains
  Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> Q =
      (Eigen::Matrix<double, kErrorStateDim, 1>()
           << param_.mpc.Q_pos_xy, param_.mpc.Q_pos_xy, param_.mpc.Q_pos_z,
         param_.mpc.Q_velocity, param_.mpc.Q_velocity, param_.mpc.Q_velocity,
         param_.mpc.Q_attitude_rp, param_.mpc.Q_attitude_rp,
         param_.mpc.Q_attitude_yaw)
          .finished()
          .asDiagonal();
  Eigen::Matrix<double, kInputDim, kInputDim> R =
      (Eigen::Matrix<double, kInputDim, 1>()
           << param_.mpc.R_thrust, param_.mpc.R_pitchroll,
         param_.mpc.R_pitchroll, param_.mpc.R_yaw)
          .finished()
          .asDiagonal();

  mpc_wrapper_.buildHessianMatrix(
      Q, R, param_.mpc.state_cost_exponential, param_.mpc.input_cost_exponential);

  // MPC reference state/input matrices and bounds
  Fx_.resize(kDefaultHorizonSteps);
  Fu_.resize(kDefaultHorizonSteps);
  u_lb_.resize(kDefaultHorizonSteps);
  u_ub_.resize(kDefaultHorizonSteps);
  for (int i = 0; i < kDefaultHorizonSteps; ++i)
  {
    u_lb_[i].resize(kInputDim);
    u_ub_[i].resize(kInputDim);
  }
}

bool OMMPCControllerCore::execMPC(const ControllerState &odom, double now_s, ControlCommand &u)
{
  clearError();

  const std::clock_t start = std::clock();

  // 1. set init error state of OMMPC
  Eigen::VectorXd x_des_start(kStateDim);
  Eigen::VectorXd u_des_start(kInputDim);
  mpc_wrapper_.getDesiredStart(x_des_start, u_des_start);
  if (x_des_start.size() != kStateDim || u_des_start.size() != kInputDim)
  {
    setError("Reference start state is not initialized.");
    return false;
  }

  auto rot_q = odom.q;
  rot_q.normalize();
  const Eigen::Quaterniond est_q = rot_q;
  const Eigen::Quaterniond des_q(x_des_start(3), x_des_start(4),
                                 x_des_start(5), x_des_start(6));
  const Eigen::Vector3d err_q =
      SO3::log(est_q.toRotationMatrix().transpose() * des_q.toRotationMatrix());
  const Eigen::Vector3d err_p = x_des_start.head<3>() - odom.p;
  const Eigen::Vector3d err_v = x_des_start.tail<3>() - odom.v;
  Eigen::VectorXd delta_x_init(kErrorStateDim);
  delta_x_init << err_p, err_v, err_q;
  mpc_wrapper_.setInitValue(delta_x_init);

  // 2. solve MPC optimization problem
  Solution solution;
  if (!mpc_wrapper_.solve(solution))
  {
    setError("OSQP failed to solve the MPC problem.");
    return false;
  }

  // 3. take the first predicted control as output
  u.bodyrates(0) = u_des_start(1) - solution.delta_u[0](1);
  u.bodyrates(1) = u_des_start(2) - solution.delta_u[0](2);
  u.bodyrates(2) = u_des_start(3) - solution.delta_u[0](3);

  // thrustacc -> normalized thrust signal
  const double thrustacc = u_des_start(0) - solution.delta_u[0](0);
  u.thrust = thrustacc / thr2acc_;
  u.collective_acceleration = thrustacc;

  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::make_pair(now_s, u.thrust));
  while (static_cast<int>(timed_thrust_.size()) > kMaxStoredThrustSamples)
  {
    timed_thrust_.pop();
  }

  const std::clock_t end = std::clock();
  timing_feedback_ = 0.9 * timing_feedback_ +
      0.1 * static_cast<double>(end - start) / CLOCKS_PER_SEC;
  return true;
}

bool OMMPCControllerCore::getDesiredState(ControllerState &state) const
{
  Eigen::VectorXd x_des(kStateDim);
  Eigen::VectorXd u_des(kInputDim);
  mpc_wrapper_.getDesiredStart(x_des, u_des);
  if (x_des.size() != kStateDim)
  {
    return false;
  }

  state.p = x_des.head<3>();
  state.q = Eigen::Quaterniond(x_des(3), x_des(4), x_des(5), x_des(6));
  state.q.normalize();
  state.v = x_des.tail<3>();
  return true;
}

// Case 1: HOVER reference
bool OMMPCControllerCore::setHoverReference(const Eigen::Vector4d &quad_pose)
{
  clearError();

  // Eigen::Vector4d(px, py, pz, yaw)
  const double yaw = quad_pose(3);
  last_yaw_ = yaw;
  last_yaw_dot_ = 0.0;
  yaw_reference_initialized_ = true;
  const Eigen::Vector3d des_acc_in_world(0.0, 0.0, gravity_);
  const double thracc = gravity_;

  const Eigen::Quaterniond identity_q(1.0, 0.0, 0.0, 0.0);
  Eigen::Quaterniond q;
  Eigen::Vector3d omg;
  // des_acc, des_jerk, des_yaw, des_yawdot, des_q (when fail to calculate proper q),
  // out_q, out_omg
  computeFlatInputwithHopfFibration(des_acc_in_world, Eigen::Vector3d::Zero(),
                                   yaw, 0.0, identity_q, q, omg);

  const double t_step = param_.mpc.step_T;
  for (int i = 0; i < kDefaultHorizonSteps; ++i)
  {
    setStateMatricesAndBounds(i, q, Eigen::Vector3d::Zero(), omg, t_step, thracc);
  }
  mpc_wrapper_.buildConstraintMatrix(Fx_, Fu_);
  mpc_wrapper_.buildConstraintVectors(u_lb_, u_ub_);

  Eigen::VectorXd x_des_start(kStateDim);
  Eigen::VectorXd u_des_start(kInputDim);
  x_des_start << quad_pose.head<3>(), q.w(), q.x(), q.y(), q.z(), Eigen::Vector3d::Zero();
  u_des_start << gravity_, omg;
  mpc_wrapper_.setDesiredStart(x_des_start, u_des_start);
  return true;
}

// Case 2: TXT reference
bool OMMPCControllerCore::setTextReference(const ReferenceTrajectory &reference,
                                           const ControllerState &odom,
                                           double start_yaw)
{
  clearError();
  if (reference.size() != static_cast<std::size_t>(kDefaultHorizonSteps + 1))
  {
    setError("Text reference horizon size does not match MPC horizon.");
    return false;
  }

  const double t_step = param_.mpc.step_T;
  Eigen::Quaterniond last_q = odom.q;
  if (!yaw_reference_initialized_)
  {
    last_yaw_ = start_yaw;
    last_yaw_dot_ = 0.0;
    yaw_reference_initialized_ = true;
  }
  double predicted_yaw = last_yaw_;
  double predicted_yaw_dot = last_yaw_dot_;

  for (int i = 0; i < kDefaultHorizonSteps; ++i)
  {
    const auto &ref_point = reference.points.at(i);
    const Eigen::Vector3d quad_velocity = ref_point.velocity;

    Eigen::Vector3d quad_acc;
    Eigen::Vector3d quad_jerk;
    if (i == 0)
    {
      quad_acc = last_text_des_acc_;
      last_text_acc_ = quad_acc;
      quad_jerk = last_text_des_jerk_;
    }
    else
    {
      quad_acc = (reference.points[i].velocity - reference.points[i - 1].velocity) / t_step;
      quad_jerk = (quad_acc - last_text_acc_) / t_step;
      last_text_acc_ = quad_acc;
      if (i == 1)
      {
        last_text_des_acc_ = quad_acc;
        last_text_des_jerk_ = quad_jerk;
      }
    }

    double yaw = 0.0;
    double yaw_dot = 0.0;
    // Compute the Hopf fiber angle that aligns body x with the trajectory tangent.
    if (!param_.controller.use_fix_yaw)
    {
      calculateYaw(quad_velocity, quad_acc, t_step,
                   predicted_yaw, predicted_yaw_dot);
      yaw = predicted_yaw;
      yaw_dot = predicted_yaw_dot;
      if (i == 0)
      {
        last_yaw_ = yaw;
        last_yaw_dot_ = yaw_dot;
      }
    }
    // read yaw from reference
    // {
    //   yaw = ref_point.yaw;
    //   yaw_dot = ref_point.yaw_dot;
    // }

    Eigen::Quaterniond q;
    Eigen::Vector3d omg;
    double thracc = 0.0;
    // if there's significant discontinuous in the txt traj (especially at the end of it),
    // don't use jerk!
    computeDragCompensatedInput(quad_velocity, quad_acc, Eigen::Vector3d::Zero(), yaw, yaw_dot, last_q, q, omg, thracc);

    q.normalize();
    last_q = q;
    if (i == 0)
    {
      Eigen::VectorXd x_des_start(kStateDim);
      Eigen::VectorXd u_des_start(kInputDim);
      x_des_start << ref_point.position, q.w(), q.x(), q.y(), q.z(), ref_point.velocity;
      u_des_start << thracc, omg;
      mpc_wrapper_.setDesiredStart(x_des_start, u_des_start);
    }

    setStateMatricesAndBounds(i, q, quad_velocity, omg, t_step, thracc);
  }

  mpc_wrapper_.buildConstraintMatrix(Fx_, Fu_);
  mpc_wrapper_.buildConstraintVectors(u_lb_, u_ub_);
  return true;
}

// Case 3: TRAJ reference
// TODO: set proper yaw
bool OMMPCControllerCore::setTrajectoryReference(const Trajectory &traj,
                                                 double tstart,
                                                 double start_yaw,
                                                 const Trajectory &yaw_traj,
                                                 const ControllerState &odom)
{
  clearError();
  (void)yaw_traj;  // TODO: use yaw traj when it's ready

  if (traj.getPieceNum() <= 0)
  {
    setError("Polynomial trajectory is empty.");
    return false;
  }

  const double t_step = param_.mpc.step_T;
  const double t_all = traj.getTotalDuration() - 1.0e-3;
  double t = tstart;

  Eigen::Quaterniond last_quat = odom.q;
  if (!yaw_reference_initialized_)
  {
    last_yaw_ = start_yaw;
    last_yaw_dot_ = 0.0;
    yaw_reference_initialized_ = true;
  }
  double predicted_yaw = last_yaw_;
  double predicted_yaw_dot = last_yaw_dot_;

  for (int i = 0; i < kDefaultHorizonSteps; ++i)
  {
    Eigen::Vector3d pos_quad;
    Eigen::Vector3d vel_quad;
    Eigen::Vector3d acc_quad;
    Eigen::Vector3d jerk_quad;

    Eigen::MatrixXd pvajs;
    if (t > t_all)
    {
      // if t is larger than the total time, use the last point
      // TODO: more than one traj
      t = t_all;
      pvajs = traj.getPVAJSC(t);
      pos_quad = pvajs.col(0);
      vel_quad = Eigen::Vector3d::Zero();
      acc_quad = Eigen::Vector3d::Zero();
      jerk_quad = Eigen::Vector3d::Zero();
    }
    else
    {
      pvajs = traj.getPVAJSC(t);
      pos_quad = pvajs.col(0);
      vel_quad = pvajs.col(1);
      acc_quad = pvajs.col(2);
      jerk_quad = pvajs.col(3);
    }

    double yaw = 0.0;
    double yaw_dot = 0.0;
    if (!param_.controller.use_fix_yaw)
    {
      calculateYaw(vel_quad, acc_quad, t_step,
                   predicted_yaw, predicted_yaw_dot);
      yaw = predicted_yaw;
      yaw_dot = predicted_yaw_dot;
      if (i == 0)
      {
        last_yaw_ = yaw;
        last_yaw_dot_ = yaw_dot;
      }
    }
    // TODO: use yaw traj

    Eigen::Quaterniond quat;
    Eigen::Vector3d omg;
    double thracc = 0.0;
    computeDragCompensatedInput(vel_quad, acc_quad, jerk_quad, yaw, yaw_dot, last_quat, quat, omg, thracc);

    last_quat = quat;
    if (i == 0)
    {
      Eigen::VectorXd x_des_start(kStateDim);
      Eigen::VectorXd u_des_start(kInputDim);
      x_des_start << pos_quad, last_quat.w(), last_quat.x(), last_quat.y(), last_quat.z(), vel_quad;
      u_des_start << thracc, omg;
      mpc_wrapper_.setDesiredStart(x_des_start, u_des_start);
    }

    setStateMatricesAndBounds(i, last_quat, vel_quad, omg, t_step, thracc);
    t += t_step;
  }

  mpc_wrapper_.buildConstraintMatrix(Fx_, Fu_);
  mpc_wrapper_.buildConstraintVectors(u_lb_, u_ub_);
  return true;
}

void OMMPCControllerCore::estimateThrustModel(const Eigen::Vector3d &est_a, double now_s)
{
  if (!param_.controller.thrust_normalization.enable_estimation)
  {
    return;
  }

  while (!timed_thrust_.empty())
  {
    // Choose data before 35~45ms ago
    const auto t_t = timed_thrust_.front();
    const double time_passed = now_s - t_t.first;
    if (time_passed > kMaxEstimationDelayS)
    {
      timed_thrust_.pop();
      continue;
    }
    if (time_passed < kMinEstimationDelayS)
    {
      return;
    }

    const double thr = t_t.second;
    timed_thrust_.pop();

    /***********************************************************/
    /* Recursive least squares algorithm with vanishing memory */
    /***********************************************************/

    /***********************************/
    /* Model: est_a(2) = thr2acc * thr */
    /***********************************/
    const double gamma = 1.0 / (kThrustForgettingFactor + thr * P_ * thr);
    const double K = gamma * P_ * thr;
    thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
    P_ = (1.0 - K * thr) * P_ / kThrustForgettingFactor;

    const double hover_percentage = gravity_ / thr2acc_;
    if (hover_percentage > kMaxHoverPercentage || hover_percentage < kMinHoverPercentage)
    {
      thr2acc_ = hover_percentage > kMaxHoverPercentage ? gravity_ / kMaxHoverPercentage : thr2acc_;
      thr2acc_ = hover_percentage < kMinHoverPercentage ? gravity_ / kMinHoverPercentage : thr2acc_;
    }
  }
}

double OMMPCControllerCore::getTimingMs() const
{
  return timing_feedback_ * 1000.0;
}

const std::string &OMMPCControllerCore::getLastError() const
{
  return last_error_;
}

double OMMPCControllerCore::angleDiff(double a, double b) const
{
  double d1 = a - b;
  double d2 = 2.0 * M_PI - std::fabs(d1);
  if (d1 > 0.0)
  {
    d2 *= -1.0;
  }
  return std::fabs(d1) < std::fabs(d2) ? d1 : d2;
}

void OMMPCControllerCore::calculateYaw(const Eigen::Vector3d &velocity,
                                       const Eigen::Vector3d &acceleration,
                                       double dt,
                                       double &yaw,
                                       double &yawdot) const
{
  const auto fiber_yaw = [this](const Eigen::Vector3d &vel,
                                const Eigen::Vector3d &acc,
                                double &result)
  {
    constexpr double kMinHorizontalHeadingSpeed = 1.0e-3;
    const Eigen::Vector2d horizontal_velocity = vel.head<2>();
    const double drag_scale =
        1.0 + param_.drag_compensation.speed_coefficient *
                  std::sqrt(vel.squaredNorm() + kDragSpeedSquaredSmoothing);
    const Eigen::Vector3d unnormalized_body_z =
        acc + Eigen::Vector3d(0.0, 0.0, gravity_) +
        specific_drag_matrix_(0, 0) * drag_scale * vel;
    if (horizontal_velocity.norm() <= kMinHorizontalHeadingSpeed ||
        unnormalized_body_z.norm() <= kMinNormalizedCollectiveThrust)
    {
      return false;
    }

    const Eigen::Vector3d body_z = unnormalized_body_z.normalized();
    if (1.0 + body_z.z() >= kAlmostZeroValueThreshold)
    {
      const double norm = std::sqrt(2.0 * (1.0 + body_z.z()));
      const Eigen::Quaterniond tilt_quaternion(
          (1.0 + body_z.z()) / norm,
          -body_z.y() / norm,
          body_z.x() / norm,
          0.0);
      const Eigen::Matrix2d horizontal_tilt =
          tilt_quaternion.toRotationMatrix().topLeftCorner<2, 2>();
      if (std::fabs(horizontal_tilt.determinant()) >= kAlmostZeroValueThreshold)
      {
        const Eigen::Vector2d fiber_heading =
            horizontal_tilt.inverse() * horizontal_velocity.normalized();
        result = std::atan2(fiber_heading.y(), fiber_heading.x());
        return true;
      }
    }
    return false;
  };

  double target_yaw = yaw;
  const bool target_is_valid = fiber_yaw(velocity, acceleration, target_yaw);
  const double yaw_error = target_is_valid ? angleDiff(target_yaw, yaw) : 0.0;
  const double yaw_dot_limit = param_.controller.yaw_reference.max_rate > 0.0
                                   ? param_.controller.yaw_reference.max_rate
                                   : param_.mpc.max_bodyrate_z * 0.90;
  const double yaw_acceleration_limit =
      param_.controller.yaw_reference.max_acceleration > 0.0
          ? param_.controller.yaw_reference.max_acceleration
          : param_.mpc.max_bodyrate_z * 4.0;

  double target_yaw_dot = 0.0;
  if (std::fabs(yaw_error) > kAlmostZeroValueThreshold)
  {
    const double braking_yaw_dot =
        std::sqrt(2.0 * yaw_acceleration_limit * std::fabs(yaw_error));
    target_yaw_dot = std::copysign(
        std::min(yaw_dot_limit, braking_yaw_dot), yaw_error);
  }

  const double yaw_dot_step = yaw_acceleration_limit * dt;
  const double next_yaw_dot = yawdot + std::max(
      -yaw_dot_step, std::min(yaw_dot_step, target_yaw_dot - yawdot));
  double yaw_step = 0.5 * (yawdot + next_yaw_dot) * dt;
  if (yaw_error * yaw_step > 0.0 &&
      std::fabs(yaw_step) > std::fabs(yaw_error))
  {
    yaw_step = yaw_error;
    yawdot = 0.0;
  }
  else
  {
    yawdot = next_yaw_dot;
  }

  yaw += yaw_step;
  if (yaw > M_PI)
  {
    yaw -= 2.0 * M_PI;
  }
  else if (yaw < -M_PI)
  {
    yaw += 2.0 * M_PI;
  }
}

void OMMPCControllerCore::setStateMatricesAndBounds(int i,
                                                    const Eigen::Quaterniond &q,
                                                    const Eigen::Vector3d &velocity,
                                                    const Eigen::Vector3d &omg,
                                                    double t_step,
                                                    double thracc)
{
  Fx_[i] = Eigen::SparseMatrix<double>(kErrorStateDim, kErrorStateDim);
  Fu_[i] = Eigen::SparseMatrix<double>(kErrorStateDim, kInputDim);

  // Fx
  std::vector<Eigen::Triplet<double>> triplet_list;
  triplet_list.reserve(33);

  // (0,0), (0,3)
  for (int k = 0; k < 3; ++k)
  {
    triplet_list.emplace_back(k, k, 1.0);
    triplet_list.emplace_back(k, 3 + k, t_step);
  }

  const Eigen::Matrix3d rotation = q.toRotationMatrix();

  const double speed =
      std::sqrt(velocity.squaredNorm() + kDragSpeedSquaredSmoothing);
  const double drag_speed_scale =
      1.0 + param_.drag_compensation.speed_coefficient * speed;
  const Eigen::Matrix3d scaled_velocity_jacobian =
      drag_speed_scale * Eigen::Matrix3d::Identity() +
      param_.drag_compensation.speed_coefficient / speed * velocity *
          velocity.transpose();

  // (3,3): I - dt * R D R^T d[(1 + c_p |v|)v]/dv
  const Eigen::Matrix3d mat_3_3 =
      Eigen::Matrix3d::Identity() -
      t_step * rotation * specific_drag_matrix_ * rotation.transpose() *
          scaled_velocity_jacobian;
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      triplet_list.emplace_back(3 + row, 3 + col, mat_3_3(row, col));
    }
  }

  // (6,6)
  const Eigen::Matrix3d exp_mat = SO3::exp(-omg * t_step);
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      triplet_list.emplace_back(6 + row, 6 + col, exp_mat(row, col));
    }
  }

  // (3,6): thrust and drag attitude Jacobians from OMMPC_derivation.pdf, page 6.
  const Eigen::Vector3d body_velocity = rotation.transpose() * velocity;
  const Eigen::Matrix3d drag_attitude_jacobian =
      SO3::hat(specific_drag_matrix_ * body_velocity) -
      specific_drag_matrix_ * SO3::hat(body_velocity);
  const Eigen::Matrix3d mat_3_6 =
      t_step * rotation *
      (-thracc * SO3::hat(Eigen::Vector3d::UnitZ()) +
       drag_speed_scale * drag_attitude_jacobian);
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      triplet_list.emplace_back(3 + row, 6 + col, mat_3_6(row, col));
    }
  }

  Fx_[i].setFromTriplets(triplet_list.begin(), triplet_list.end());
  Fx_[i].makeCompressed();

  // Fu
  triplet_list.clear();
  triplet_list.reserve(12);

  // (3,0)
  const Eigen::Vector3d vec_3_0 = t_step * rotation * Eigen::Vector3d::UnitZ();
  for (int k = 0; k < 3; ++k)
  {
    triplet_list.emplace_back(3 + k, 0, vec_3_0(k));
  }

  // (6,1)
  const Eigen::Matrix3d mat_6_1 = SO3::leftJacobian(omg * t_step).transpose() * t_step;
  for (int row = 0; row < 3; ++row)
  {
    for (int col = 0; col < 3; ++col)
    {
      triplet_list.emplace_back(6 + row, 1 + col, mat_6_1(row, col));
    }
  }

  Fu_[i].setFromTriplets(triplet_list.begin(), triplet_list.end());
  Fu_[i].makeCompressed();

  u_ub_[i] << thracc - param_.mpc.min_thrust,
      param_.mpc.max_bodyrate_xy + omg(0), param_.mpc.max_bodyrate_xy + omg(1),
      param_.mpc.max_bodyrate_z + omg(2);
  u_lb_[i] << -(param_.mpc.max_thrust - thracc),
      omg(0) - param_.mpc.max_bodyrate_xy, omg(1) - param_.mpc.max_bodyrate_xy,
      omg(2) - param_.mpc.max_bodyrate_z;
}

void OMMPCControllerCore::computeDragCompensatedInput(
    const Eigen::Vector3d &velocity, const Eigen::Vector3d &acceleration,
    const Eigen::Vector3d &jerk, double yaw, double yawd,
    const Eigen::Quaterniond &att_est, Eigen::Quaterniond &att,
    Eigen::Vector3d &omg, double &thracc) const
{
  const Eigen::Quaterniond normalized_attitude_estimate = att_est.normalized();
  const Eigen::Vector3d gravity_compensated_acceleration =
      acceleration + Eigen::Vector3d(0.0, 0.0, gravity_);

  if (!param_.drag_compensation.enable)
  {
    computeFlatInputwithHopfFibration(gravity_compensated_acceleration, jerk,
                                     yaw, yawd, normalized_attitude_estimate, att, omg);
    att.normalize();
    thracc = gravity_compensated_acceleration.dot(
        normalized_attitude_estimate.toRotationMatrix() * Eigen::Vector3d::UnitZ());
    return;
  }

  // GCOPTER thesis, Section 4.2, with
  //   sigma(||v||) = 1 + c_p sqrt(||v||^2 + epsilon):
  //   z_b = normalize(a_d + g e_z + (d_h / m) sigma(||v||) v_d).
  // Horizontal symmetry makes z_b independent of yaw and gives this closed form.
  const double horizontal_specific_drag = specific_drag_matrix_(0, 0);
  const double vertical_specific_drag = specific_drag_matrix_(2, 2);
  const double smoothed_speed =
      std::sqrt(velocity.squaredNorm() + kDragSpeedSquaredSmoothing);
  const double drag_scale =
      1.0 + param_.drag_compensation.speed_coefficient * smoothed_speed;
  const Eigen::Vector3d scaled_velocity = drag_scale * velocity;
  const Eigen::Vector3d scaled_velocity_derivative =
      drag_scale * acceleration +
      param_.drag_compensation.speed_coefficient * velocity.dot(acceleration) /
          smoothed_speed * velocity;
  const Eigen::Vector3d unnormalized_body_z =
      gravity_compensated_acceleration + horizontal_specific_drag * scaled_velocity;
  const Eigen::Vector3d unnormalized_body_z_derivative =
      jerk + horizontal_specific_drag * scaled_velocity_derivative;

  computeFlatInputwithHopfFibration(unnormalized_body_z, unnormalized_body_z_derivative,
                                   yaw, yawd, normalized_attitude_estimate, att, omg);
  att.normalize();

  // a_T = z_b^T (a_d + g e_z + (d_v / m) sigma(||v||) v_d).
  const Eigen::Vector3d body_z =
      att.toRotationMatrix() * Eigen::Vector3d::UnitZ();
  thracc = body_z.dot(gravity_compensated_acceleration +
                         vertical_specific_drag * scaled_velocity);
}

void OMMPCControllerCore::computeFlatInputwithHopfFibration(const Eigen::Vector3d &thr_acc,
                                                            const Eigen::Vector3d &jer,
                                                            double yaw,
                                                            double yawd,
                                                            const Eigen::Quaterniond &att_est,
                                                            Eigen::Quaterniond &att,
                                                            Eigen::Vector3d &omg) const
{
  // use previous angular velocity when approaching Hopf singularity
  static Eigen::Vector3d omg_old = Eigen::Vector3d::Zero();

  const Eigen::Vector3d abc = thr_acc.normalized();
  const double a = abc(0);
  const double b = abc(1);
  const double c = abc(2);
  const Eigen::Vector3d abc_dot =
      (thr_acc.dot(thr_acc) * Eigen::Matrix3d::Identity() - thr_acc * thr_acc.transpose()) /
      thr_acc.norm() / thr_acc.squaredNorm() * jer;
  const double a_dot = abc_dot(0);
  const double b_dot = abc_dot(1);
  const double c_dot = abc_dot(2);

  if(1.0 + c < kAlmostZeroValueThreshold)
  {
    // cout << "Near singularity, body near -g!" << endl;
    omg = omg_old;
    att = att_est;
  }
  else if(thr_acc.norm() < kMinNormalizedCollectiveThrust)
  {
    // cout << "Near singularity, thrust is almost 0!" << endl;
    omg.setConstant(0.0);
    att = att_est;
  }
  else
  {
    const double norm = std::sqrt(2.0 * (1.0 + c));
    const Eigen::Quaterniond q((1.0 + c) / norm, -b / norm, a / norm, 0.0);
    const Eigen::Quaterniond q_yaw(std::cos(yaw / 2.0), 0.0, 0.0, std::sin(yaw / 2.0));
    att = q * q_yaw;
    const double syaw = std::sin(yaw);
    const double cyaw = std::cos(yaw);
    omg(0) = syaw * a_dot - cyaw * b_dot - (a * syaw - b * cyaw) * c_dot / (c + 1.0);
    omg(1) = cyaw * a_dot + syaw * b_dot - (a * cyaw + b * syaw) * c_dot / (c + 1.0);
    omg(2) = (b * a_dot - a * b_dot) / (1.0 + c) + yawd;
  }
  omg_old = omg;
}

void OMMPCControllerCore::setError(const std::string &error_message)
{
  last_error_ = error_message;
}

void OMMPCControllerCore::clearError()
{
  last_error_.clear();
}

} // namespace ommpc_core
