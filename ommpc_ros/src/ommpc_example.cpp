#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <dynamic_reconfigure/server.h>

#include "ommpc_core/controller_core.hpp"
#include "ommpc_ros/fsm_changeConfig.h"
#include "ommpc_ros/ros_data.hpp"
#include "ommpc_ros/mavros_command_interface.hpp"
#include "ommpc_ros/param_loader.hpp"
#include "ommpc_ros/text_reference_loader.hpp"

#include <algorithm>

static constexpr int nstep = ommpc_core::kDefaultHorizonSteps;
static constexpr int nx = ommpc_core::kErrorStateDim;
static constexpr int nstate = ommpc_core::kStateDim;
static constexpr int nu = ommpc_core::kInputDim;

using Controller_Output_t = ommpc_core::ControlCommand;
using Parameter_t = ommpc_core::ParameterSet;
using MpcController = ommpc_core::OMMPCControllerCore;

using Odom_Data_t = ommpc_ros::OdomData;
using Imu_Data_t = ommpc_ros::ImuData;
using oneTraj_Data_t = ommpc_ros::OneTrajectoryData;
using Trajectory_Data_t = ommpc_ros::TrajectoryData;

enum Exec_Traj_State_t
{
    HOVER = 10,     // execute the hover trajectory
    POLY_TRAJ = 11, // execute the polynomial trajectory
    POINTS = 12,    // execute the point trajectory (txt)
    TAKEOFF = 13,   // execute the takeoff trajectory
    LAND = 14       // execute the land trajectory
};

class OMMPC_EXAMPLE{
private:
    ros::Subscriber odom_sub_, imu_sub_, state_sub_, mpc_traj_sub_;
    ros::Publisher reference_pub_;
    ros::Timer exec_timer_;
    mavros_msgs::State state_;
    Odom_Data_t odom_data_;
    Imu_Data_t imu_data_;
    Exec_Traj_State_t exec_traj_state_;
    Parameter_t param_;
    Trajectory_Data_t trajectory_data_;
    MpcController ommpc_controller_;
    ommpc_ros::MavrosCommandInterface mavros_interface_;
    ommpc_ros::TextReferenceLoader text_reference_loader_;
    
    bool is_command_mode_ = false;
    bool takeoff_enabled_ = false, last_takeoff_enabled_ = false, takeoff_trigger_ = false;
    bool land_enabled_ = false, last_land_enabled_ = false, land_trigger_ = false;
    bool has_takeoff_ = false, has_land_ = true; // default to landed, so has_land_ = true
    double start_takeoff_land_time;
    Eigen::Vector4d hover_pose_;
    int consecutive_failures_ = 0;

    int line_cnt_ = 0, number_of_steps_ = 0;
    ommpc_core::ReferenceTrajectory text_reference_trajectory_;
    ommpc_core::ReferenceTrajectory active_reference_;

    dynamic_reconfigure::Server<ommpc_ros::fsm_changeConfig> dynamic_reconfig_server_;
    dynamic_reconfigure::Server<ommpc_ros::fsm_changeConfig>::CallbackType dynamic_reconfig_cb_type_;

    bool useEnuFrame() const
    {
        return param_.controller.world_frame == ommpc_core::WorldFrame::kEnu;
    }

    bool velocityInBodyFrame() const
    {
        return param_.controller.velocity_frame == ommpc_core::VelocityFrame::kBody;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
        odom_data_.feed(msg, useEnuFrame(), velocityInBodyFrame());
    }

    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg){
        imu_data_.feed(msg, useEnuFrame());
    }

    void stateCallback(const mavros_msgs::State::ConstPtr &msg){
        state_ = *msg;
    }

    void dynamicReconfigureCallback(ommpc_ros::fsm_changeConfig &config, uint32_t level){
        last_takeoff_enabled_ = takeoff_enabled_;
        last_land_enabled_ = land_enabled_;
        land_enabled_ = config.land_enabled;
        is_command_mode_ = config.command_or_hover;
        takeoff_enabled_ = config.takeoff_enabled;
        if (last_takeoff_enabled_ == false && takeoff_enabled_ == true)
        {
            takeoff_trigger_ = true;
        }
        if (last_land_enabled_ == false && land_enabled_ == true)
        {
            land_trigger_ = true;
        }
    }

    void setHoverWithOdom()
    {
        hover_pose_.head<3>() = odom_data_.p;
        hover_pose_(3) = getYawFromQuaternion(odom_data_.q);
    }

    double getYawFromQuaternion(const Eigen::Quaterniond& q) {
        return atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    }

    void setReferencePoint(const int idx,
                           const Eigen::Vector3d &position,
                           const Eigen::Vector3d &velocity,
                           const double yaw,
                           const double yaw_rate = 0.0)
    {
        auto &ref_point = active_reference_.points.at(idx);
        ref_point.time_from_start_s = idx * param_.mpc.step_T;
        ref_point.position = position;
        ref_point.velocity = velocity;
        ref_point.acceleration.setZero();
        ref_point.jerk.setZero();
        ref_point.yaw = yaw;
        ref_point.yaw_rate = yaw_rate;
    }

    void publishCurrentReference(const ros::Time &stamp)
    {
        ommpc_core::ControllerState reference;
        if (!ommpc_controller_.getDesiredState(reference))
        {
            return;
        }

        nav_msgs::Odometry msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.child_frame_id = "reference";
        msg.pose.pose.position.x = reference.p.x();
        msg.pose.pose.position.y = reference.p.y();
        msg.pose.pose.position.z = reference.p.z();
        msg.pose.pose.orientation.w = reference.q.w();
        msg.pose.pose.orientation.x = reference.q.x();
        msg.pose.pose.orientation.y = reference.q.y();
        msg.pose.pose.orientation.z = reference.q.z();
        msg.twist.twist.linear.x = reference.v.x();
        msg.twist.twist.linear.y = reference.v.y();
        msg.twist.twist.linear.z = reference.v.z();
        reference_pub_.publish(msg);
    }

    void execFSMCallback(const ros::TimerEvent &e){
        if (!ros::ok())
        {
            return;
        }
        Controller_Output_t u;
        bool ret;
        ros::Time now_time = ros::Time::now();
        const double now_s = now_time.toSec();
        // std::cout << "exec_traj_state_:" << exec_traj_state_ << std::endl;
        // std::cout << "traj_queue size:" << trajectory_data_.traj_queue.size() << std::endl;
        // std::cout << "now_time" << now_time << std::endl;
        // std::cout << "total_traj_start_time" << trajectory_data_.total_traj_start_time << std::endl;
        // std::cout << "total_traj_end_time" << trajectory_data_.total_traj_end_time << std::endl;
        // std::cout << "first traj_start_time" << trajectory_data_.traj_queue.front().traj_start_time << std::endl;
        switch (exec_traj_state_)
        {
        case HOVER:
        {
            if (takeoff_trigger_ && !has_takeoff_)
            {
                if (state_.mode != mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    ROS_ERROR("[MPCctrl] Not in OFFBOARD mode! Cannot takeoff.");
                    takeoff_trigger_ = false;
                    ret = true;
                    break;
                }
                start_takeoff_land_time = now_s;
                std::string error_message;
                if (mavros_interface_.armDisarm(true, &error_message))
                {
                    setHoverWithOdom();
                    exec_traj_state_ = TAKEOFF;
                    ROS_INFO("[MPCctrl] HOVER --> TAKEOFF");
                }
                else
                {
                    ROS_ERROR_STREAM(error_message);
                }
                ret = true;
            }
            else if (land_trigger_ && !has_land_)
            {
                if (state_.mode != mavros_msgs::State::MODE_PX4_OFFBOARD)
                {
                    ROS_ERROR("[MPCctrl] Not in OFFBOARD mode! Cannot land.");
                    land_trigger_ = false;
                    ret = true;
                    break;
                }
                start_takeoff_land_time = now_s;
                setHoverWithOdom();
                ret = true;
                exec_traj_state_ = LAND;
                ROS_INFO("[MPCctrl] HOVER --> LAND");
            }
            else if (consecutive_failures_ == 0 && is_command_mode_ &&
                now_time >= trajectory_data_.total_traj_start_time &&
                now_time <= trajectory_data_.total_traj_end_time &&
                trajectory_data_.exec_traj == 1 && (!trajectory_data_.traj_queue.empty()) &&
                state_.armed == true && state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
            {
                // same as the below
                setHoverWithOdom();
                oneTraj_Data_t *traj_info = &trajectory_data_.traj_queue.front();
                traj_info = &trajectory_data_.traj_queue.front();
                trajectory_data_.total_traj_start_time = traj_info->traj_start_time;

                double traj_time = (now_time - traj_info->traj_start_time).toSec();
                ret = ommpc_controller_.setTrajectoryReference(
                    traj_info->traj, traj_time, hover_pose_(3), traj_info->yaw_traj, odom_data_);
                if (ret)
                {
                    ret = ommpc_controller_.execMPC(odom_data_, now_s, u);
                }
                exec_traj_state_ = POLY_TRAJ;
                ROS_INFO("[MPCctrl] Receive the trajectory. HOVER --> POLY_TRAJ");
            }
            else if (consecutive_failures_ == 0 &&
                param_.controller.text_reference.enable && is_command_mode_ &&
                state_.armed == true && state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
            {
                exec_traj_state_ = POINTS;
                ROS_INFO("[MPCctrl] Start executing traj from txt. HOVER --> POINTS");
                // same as the below
                double yaw_now = getYawFromQuaternion(odom_data_.q);
                getTxtDes();
                ret = ommpc_controller_.setTextReference(active_reference_, odom_data_, yaw_now);
                if (ret)
                {
                    ret = ommpc_controller_.execMPC(odom_data_, now_s, u);
                }
            }
            else if (state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && state_.armed == true)
            {
                ret = ommpc_controller_.setHoverReference(hover_pose_);
                if (ret)
                {
                    ret = ommpc_controller_.execMPC(odom_data_, now_s, u);
                }
            }
            else
            {
                // Not in offboard: skip MPC but keep publishing so PX4 can enter offboard
                u.bodyrates.setZero();
                u.thrust = 0.0;
                ret = true;
            }
        }
        break;

        case POLY_TRAJ:
        {
            if (now_time < (trajectory_data_.total_traj_start_time) || now_time > trajectory_data_.total_traj_end_time || trajectory_data_.exec_traj != 1 || trajectory_data_.traj_queue.empty())
            {
                if (param_.controller.use_trajectory_ending_pos && trajectory_data_.exec_traj != -1)
                {
                    // tracking the end point of the trajectory
                    // the hover pose is the end point of the trajectory
                    auto &traj_info = trajectory_data_.traj_queue.front().traj;
                    hover_pose_.head<3>() = traj_info.getJuncPos(traj_info.getPieceNum());
                    hover_pose_(3) = getYawFromQuaternion(odom_data_.q);
                    // std::cout << "hover_pose_ = " << hover_pose_.transpose() << std::endl;
                }
                else
                {
                    setHoverWithOdom();
                }
                ret = ommpc_controller_.setHoverReference(hover_pose_);
                if (ret)
                {
                    ret = ommpc_controller_.execMPC(odom_data_, now_s, u);
                }
                exec_traj_state_ = HOVER;
                ROS_INFO("[MPCctrl] Stop execute the trajectory. POLY_TRAJ --> HOVER");
                trajectory_data_.exec_traj = 0;
            }
            else
            {
                setHoverWithOdom();
                // std::cout << "hover_pose_ = " << hover_pose_.transpose() << std::endl;
                oneTraj_Data_t *traj_info = &trajectory_data_.traj_queue.front();
                if (now_time < (traj_info->traj_start_time))
                { // the start time of first trajectory should be whole trajectory start time
                    trajectory_data_.total_traj_start_time = traj_info->traj_start_time;
                    ret = ommpc_controller_.setHoverReference(hover_pose_);
                    if (ret)
                    {
                        ret = ommpc_controller_.execMPC(odom_data_, now_s, u);
                    }
                }
                else
                {
                    if (trajectory_data_.traj_queue.size() > 1)
                    {
                        ROS_WARN("trajectory_data_.traj_queue.size() > 1");
                        oneTraj_Data_t *next_traj_info = &trajectory_data_.traj_queue.at(1);
                        while (now_time > next_traj_info->traj_start_time)
                        { // finish the first trajectory
                            trajectory_data_.traj_queue.pop_front();
                            traj_info = &trajectory_data_.traj_queue.front();
                            trajectory_data_.total_traj_start_time = traj_info->traj_start_time;
                            trajectory_data_.total_traj_end_time = trajectory_data_.traj_queue.back().traj_end_time;
                            if (trajectory_data_.traj_queue.size() == 1)
                            {
                                break;
                            }
                            next_traj_info = &trajectory_data_.traj_queue.at(1);
                        }
                    }
                    double traj_time = (now_time - traj_info->traj_start_time).toSec();
                    ret = ommpc_controller_.setTrajectoryReference(
                        traj_info->traj, traj_time, hover_pose_(3), traj_info->yaw_traj, odom_data_);
                    if (ret)
                    {
                        ret = ommpc_controller_.execMPC(odom_data_, now_s, u);
                    }
                }
            }
        }
        break;

        case POINTS:
        {
            double yaw_now = getYawFromQuaternion(odom_data_.q);
            getTxtDes();
			ret = ommpc_controller_.setTextReference(active_reference_, odom_data_, yaw_now);
            if (ret)
            {
                ret = ommpc_controller_.execMPC(odom_data_, now_s, u);
            }
            if (!is_command_mode_)
            {
                setHoverWithOdom();
                exec_traj_state_ = HOVER;
                ROS_INFO("[MPCctrl] Stop execute the trajectory. POINTS --> HOVER");
            }
        }
        break;

        case TAKEOFF:
        {
            takeoff_trigger_ = false;
            for (int i = 0; i < nstep + 1; ++i)
            {
                double altitude = std::min( 
                    (now_s - start_takeoff_land_time + i * param_.mpc.step_T) * param_.controller.takeoff_land_speed,
                    param_.controller.takeoff_height);
                setReferencePoint(i,
                                  Eigen::Vector3d(hover_pose_(0),
                                                  hover_pose_(1),
                                                  hover_pose_(2) + altitude),
                                  Eigen::Vector3d(0.0, 0.0, param_.controller.takeoff_land_speed),
                                  hover_pose_(3));
            }
            double yaw_now = getYawFromQuaternion(odom_data_.q);
            ret = ommpc_controller_.setTextReference(active_reference_, odom_data_, yaw_now);
            if (ret)
            {
                ret = ommpc_controller_.execMPC(odom_data_, now_s, u);
            }
            if (odom_data_.p(2) > hover_pose_(2) + param_.controller.takeoff_height)
            {
                exec_traj_state_ = HOVER;
                setHoverWithOdom();
                ROS_INFO("[MPCctrl] TAKEOFF succeeded. TAKEOFF --> HOVER");
                has_takeoff_ = true;
                has_land_ = false; // reset land status after takeoff
            }
        }
        break;

        case LAND:
        {
            land_trigger_ = false;
            const double land_height = -3.0;
            double altitude;
            auto now_time = ros::Time::now();
            for (int i = 0; i < nstep + 1; ++i)
            {
                altitude = std::max( 
                    (now_s - start_takeoff_land_time + i * param_.mpc.step_T) * (-param_.controller.takeoff_land_speed),
                    land_height);
                setReferencePoint(i,
                                  Eigen::Vector3d(hover_pose_(0),
                                                  hover_pose_(1),
                                                  hover_pose_(2) + altitude),
                                  Eigen::Vector3d(0.0, 0.0, -param_.controller.takeoff_land_speed),
                                  hover_pose_(3));
            }
            double yaw_now = getYawFromQuaternion(odom_data_.q);
            ret = ommpc_controller_.setTextReference(active_reference_, odom_data_, yaw_now);
            if (ret)
            {
                ret = ommpc_controller_.execMPC(odom_data_, now_s, u);
            }
            static double last_trial_time = 0; // Avoid too frequent calls
            if (odom_data_.v.norm() < 0.1 && altitude < -1.5)
            {
                has_land_ = true;
                has_takeoff_ = false; // reset takeoff status after landing
				if (now_time.toSec() - last_trial_time > 1.0)
				{
                    std::string error_message;
					if (mavros_interface_.armDisarm(false, &error_message)) // try to disarm
					{
						exec_traj_state_ = HOVER;
                        setHoverWithOdom();
						ROS_INFO("[MPCctrl] LAND succeeded. LAND --> HOVER");
					}
                    else
                    {
                        ROS_ERROR_STREAM(error_message);
                    }

					last_trial_time = now_time.toSec();
				}
            }
        }
        break;

        default:
        {
            bool ret = false;
            exec_traj_state_ = HOVER;
            ROS_ERROR("[MPCctrl] Unknown exec_traj_state_! Jump to HOVER.");
        }

        break;
        }

        if (ret)
        {
            consecutive_failures_ = 0;
            mavros_interface_.publishAttitudeTarget(u);
            if (state_.armed && state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
            {
                publishCurrentReference(now_time);
            }
        }
        else
        {
            consecutive_failures_++;
            exec_traj_state_ = HOVER;
            setHoverWithOdom();
            const std::string &core_error = ommpc_controller_.getLastError();
            if (core_error.empty())
            {
                ROS_ERROR("[MPCctrl] Numerical error!");
            }
            else
            {
                ROS_ERROR_STREAM("[MPCctrl] " << core_error);
            }
        }

        if (exec_traj_state_ != LAND)
        {
            land_trigger_ = false; // reset land trigger if not landing, to avoid unexpected landing when switching back to hover
        }
        if (exec_traj_state_ != TAKEOFF)
        {
            takeoff_trigger_ = false; // reset takeoff trigger if not taking off, to avoid unexpected takeoff when switching back to hover
        }

        ROS_INFO_THROTTLE(2.0, "MPC Timing: Latency: %1.3f ms", ommpc_controller_.getTimingMs());
        
        if(state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && state_.armed == true && exec_traj_state_ != TAKEOFF && exec_traj_state_ != LAND)
        {
            ommpc_controller_.estimateThrustModel(imu_data_.a, now_s);
        }
    }

    bool readDataFromFile()
    {
        std::string error_message;
        if (!text_reference_loader_.loadFromPackagePath(
                "ommpc_ros",
                param_.controller.text_reference,
                text_reference_trajectory_,
                &error_message))
        {
            ROS_ERROR_STREAM(error_message);
            return false;
        }

        number_of_steps_ = static_cast<int>(text_reference_trajectory_.size());
        line_cnt_ = 0;
        ROS_INFO_STREAM("Text trajectory loaded with " << number_of_steps_ << " points.");
        return true;
    }

    void getTxtDes()
    {
        if (text_reference_trajectory_.empty())
        {
            return;
        }

        text_reference_loader_.sampleWindow(
            text_reference_trajectory_, line_cnt_, nstep, param_.mpc.step_T, active_reference_);

        line_cnt_++;
        if (line_cnt_ >= number_of_steps_)
            line_cnt_ = number_of_steps_ - 1;
        return;
    }

public:
    OMMPC_EXAMPLE(/* args */){};
    ~OMMPC_EXAMPLE(){};
    void init(ros::NodeHandle &nh){
        ommpc_ros::ParameterLoader::load(nh, param_);
        exec_traj_state_ = HOVER;

        mavros_interface_.init(nh);
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &OMMPC_EXAMPLE::odomCallback, this);
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &OMMPC_EXAMPLE::IMUCallback, this);
        state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &OMMPC_EXAMPLE::stateCallback, this);
        reference_pub_ = nh.advertise<nav_msgs::Odometry>("reference", 10);
        mpc_traj_sub_ = nh.subscribe<traj_utils::PolyTraj>("/drone_0_planning/trajectory",
                                        100,
                                        boost::bind(&Trajectory_Data_t::feedFromTrajUtils, &trajectory_data_, _1),
                                        ros::VoidConstPtr(),
                                        ros::TransportHints().tcpNoDelay());

        int trials = 0;
        while (ros::ok() && !state_.connected)
        {
            ros::spinOnce();
            ros::Duration(1.0).sleep();
            if (trials++ > 5)
                ROS_ERROR("Unable to connnect to PX4!!!");
        }

        dynamic_reconfig_cb_type_ = boost::bind(&OMMPC_EXAMPLE::dynamicReconfigureCallback, this, _1, _2);
        dynamic_reconfig_server_.setCallback(dynamic_reconfig_cb_type_);

        if (param_.controller.text_reference.enable){
            std::cout << "Ref trajectory enabled!" << std::endl;
            if (!readDataFromFile())
                std::cout << "File input error!" << std::endl;
        }

        active_reference_.sample_dt_s = param_.mpc.step_T;
        active_reference_.resize(nstep + 1);

        ommpc_controller_.init(param_);

        hover_pose_ << 0.0, 0.0, 0.0, 0.0;

        exec_timer_ = nh.createTimer(ros::Duration(0.01), &OMMPC_EXAMPLE::execFSMCallback, this);
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "ommpc_example_node");
    ros::NodeHandle nh("~");

    OMMPC_EXAMPLE ommpc_example;
    ommpc_example.init(nh);

    ros::spin();

    return 0;
}
