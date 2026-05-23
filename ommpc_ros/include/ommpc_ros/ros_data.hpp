#ifndef OMMPC_ROS_ROS_DATA_HPP
#define OMMPC_ROS_ROS_DATA_HPP

#include <deque>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <traj_utils/PolyTraj.h>

#include "ommpc_core/types.hpp"
#include "ommpc_core/polynomial_trajectory.h"

namespace ommpc_ros
{

struct OdomData : public ommpc_core::ControllerState
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  nav_msgs::Odometry msg;
  ros::Time rcv_stamp;
  bool recv_new_msg = false;

  void feed(nav_msgs::OdometryConstPtr pMsg, bool enu_frame, bool vel_in_body)
  {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    recv_new_msg = true;
    timestamp_s = rcv_stamp.toSec();

    p(0) = msg.pose.pose.position.x;
    p(1) = msg.pose.pose.position.y;
    p(2) = msg.pose.pose.position.z;

    v(0) = msg.twist.twist.linear.x;
    v(1) = msg.twist.twist.linear.y;
    v(2) = msg.twist.twist.linear.z;

    q.w() = msg.pose.pose.orientation.w;
    q.x() = msg.pose.pose.orientation.x;
    q.y() = msg.pose.pose.orientation.y;
    q.z() = msg.pose.pose.orientation.z;

    w(0) = msg.twist.twist.angular.x;
    w(1) = msg.twist.twist.angular.y;
    w(2) = msg.twist.twist.angular.z;

    if (!enu_frame)
    {
      Eigen::Matrix3d R_mid;
      R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
      const Eigen::Quaterniond q_mid(R_mid);

      p = q_mid.toRotationMatrix() * p;
      v = q_mid.toRotationMatrix() * v;
      q = q_mid * q * q_mid;
      q.normalize();
      w = q_mid.toRotationMatrix() * w;
    }

    if (vel_in_body)
    {
      v = q.toRotationMatrix() * v;
    }
  }
};

struct ImuData : public ommpc_core::ImuSample
{
  sensor_msgs::Imu msg;
  ros::Time rcv_stamp;
  bool recv_new_msg = false;

  void feed(sensor_msgs::ImuConstPtr pMsg, bool enu_frame)
  {
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    recv_new_msg = true;
    timestamp_s = rcv_stamp.toSec();

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    if (!enu_frame)
    {
      Eigen::Matrix3d R_mid;
      R_mid << 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0;
      const Eigen::Quaterniond q_mid(R_mid);

      a = q_mid.toRotationMatrix() * a;
      q = q_mid * q * q_mid;
      q.normalize();
      w = q_mid.toRotationMatrix() * w;
    }
  }
};

struct OneTrajectoryData
{
  ros::Time traj_start_time{0};
  ros::Time traj_end_time{0};
  Trajectory traj;
  Trajectory yaw_traj;
};

class TrajectoryData
{
public:
  ros::Time total_traj_start_time{0};
  ros::Time total_traj_end_time{0};
  int exec_traj = 0;
  std::deque<OneTrajectoryData> traj_queue;

  void adjust_end_time()
  {
    if (traj_queue.size() < 2)
    {
      return;
    }
    for (auto it = traj_queue.begin(); it != (traj_queue.end() - 1); ++it)
    {
      it->traj_end_time = (it + 1)->traj_start_time;
    }
  }

  // from ego planner v2
  void feedFromTrajUtils(traj_utils::PolyTrajConstPtr pMsg)
  {
    const traj_utils::PolyTraj &traj = *pMsg;

    if (traj.order < 3)
    {
      exec_traj = -1;
      return;
    }

    ROS_WARN("[MPCCtrl] Loading the trajectory.");
    if (traj.traj_id < 1)
    {
      ROS_ERROR("[MPCCtrl] The trajectory_id must start from 1");
      return;
    }

    OneTrajectoryData traj_data;
    traj_data.traj_start_time = pMsg->start_time;
    double t_total = 0.0;

    for (int i = 0; i < static_cast<int>(traj.duration.size()); ++i)
    {
      const int num_dim = 3;
      const int num_order = traj.order;
      t_total += traj.duration[i];

      Eigen::MatrixXd piece_coef(num_dim, num_order + 1);
      for (int j = 0; j <= num_order; ++j)
      {
        piece_coef(0, j) = traj.coef_x[i * (num_order + 1) + j];
        piece_coef(1, j) = traj.coef_y[i * (num_order + 1) + j];
        piece_coef(2, j) = traj.coef_z[i * (num_order + 1) + j];
      }
      traj_data.traj.emplace_back(traj.duration[i], piece_coef);
    }

    traj_data.traj_end_time = traj_data.traj_start_time + ros::Duration(t_total);

    if (ros::Time::now() < traj_data.traj_start_time)
    {
      while (!traj_queue.empty() && traj_queue.back().traj_start_time > traj_data.traj_start_time)
      {
        traj_queue.pop_back();
      }
      traj_queue.push_back(traj_data);
    }
    else
    {
      while (!traj_queue.empty() && traj_queue.front().traj_start_time < traj_data.traj_start_time)
      {
        traj_queue.pop_front();
      }
      traj_queue.push_front(traj_data);
    }

    total_traj_end_time = traj_queue.back().traj_end_time;
    total_traj_start_time = traj_queue.front().traj_start_time;
    exec_traj = 1;
  }
};

} // namespace ommpc_ros

#endif
