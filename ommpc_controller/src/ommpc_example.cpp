#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <dynamic_reconfigure/server.h>

#include "ommpc_controller.hpp"
#include "ommpc_controller/fsm_changeConfig.h"

#include <ros/package.h>
#include <fstream>

enum Exec_Traj_State_t
{
    HOVER = 10,     // execute the hover trajectory
    POLY_TRAJ = 11, // execute the polynomial trajectory
    POINTS = 12,    // execute the point trajectory (txt)
    TAKEOFF = 13,
    LAND = 14
};

class OMMPC_EXAMPLE{
private:
    ros::NodeHandle node_;
    ros::Publisher cmd_pub_;
    ros::Subscriber odom_sub_, imu_sub_, state_sub_, mpc_traj_sub_;
    ros::ServiceClient set_mode_client_, arming_client_srv_;
    ros::Timer exec_timer_, read_file_timer_;
    mavros_msgs::State state_;
    Odom_Data_t odom_data_;
    Imu_Data_t imu_data_;
    Exec_Traj_State_t exec_traj_state_;
    Parameter_t param_;
    Trajectory_Data_t trajectory_data_;
    MpcController ommpc_controller_;
    
    bool is_command_mode_ = false;
    bool takeoff_enabled_ = false, last_takeoff_enabled_ = false, takeoff_trigger_ = false;
    bool land_enabled_ = false, last_land_enabled_ = false, land_trigger_ = false;
    double start_takeoff_land_time;
    bool enu_frame_, vel_in_body_;
    Eigen::Vector4d hover_pose_;

    int line_cnt_ = 0, number_of_steps_ = 0;
    std::vector<std::vector<double>> test_trajectory_;
    std::vector<Eigen::Vector3d> quad_positions_;
	std::vector<Eigen::Vector3d> quad_velocities_;
	std::vector<double> yaws_;

    dynamic_reconfigure::Server<ommpc_controller::fsm_changeConfig> state_change_server_;
    dynamic_reconfigure::Server<ommpc_controller::fsm_changeConfig>::CallbackType state_change_cb_type_;

    void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg){
        odom_data_.feed(msg, enu_frame_, vel_in_body_);
    }

    void IMUCallback(const sensor_msgs::Imu::ConstPtr &msg){
        imu_data_.feed(msg, enu_frame_);
    }

    void StateCallback(const mavros_msgs::State::ConstPtr &msg){
        state_ = *msg;
    }

    void stateChangeCallback(ommpc_controller::fsm_changeConfig &config, uint32_t level){
        last_takeoff_enabled_ = takeoff_enabled_;
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

    void send_cmd(const Controller_Output_t &output){
        mavros_msgs::AttitudeTarget cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.body_rate.x = output.bodyrates(0);
        cmd.body_rate.y = output.bodyrates(1);
        cmd.body_rate.z = output.bodyrates(2);
        cmd.thrust = output.thrust;
        cmd.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
        cmd_pub_.publish(cmd);
    }

    void set_hov_with_odom()
    {
        hover_pose_.head<3>() = odom_data_.p;
        hover_pose_(3) = get_yaw_from_quaternion(odom_data_.q);
    }

    double get_yaw_from_quaternion(const Eigen::Quaterniond& q) {
        return atan2(2 * (q.w() * q.z() + q.x() * q.y()), 1 - 2 * (q.y() * q.y() + q.z() * q.z()));
    }

    bool toggle_arm_disarm(bool arm)
    {
        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = arm;
        if (!(arming_client_srv_.call(arm_cmd) && arm_cmd.response.success))
        {
            if (arm)
                ROS_ERROR("ARM rejected by PX4!");
            else
                ROS_ERROR("DISARM rejected by PX4!");

            return false;
        }

        return true;
    }

    void execFSMCallback(const ros::TimerEvent &e){
        exec_timer_.stop();

        Controller_Output_t u;
        bool ret;
        ros::Time now_time = ros::Time::now();
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
            if (takeoff_trigger_ && state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
            {
                start_takeoff_land_time = ros::Time::now().toSec();
                if (toggle_arm_disarm(true))
                {
                    set_hov_with_odom();
                    exec_traj_state_ = TAKEOFF;
                    ROS_INFO("[MPCctrl] Receive the trajectory. HOVER --> TAKEOFF");
                }
                ret = true;
            }
            else if (land_trigger_ && state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
            {
                start_takeoff_land_time = ros::Time::now().toSec();
                set_hov_with_odom();
                ret = true;
                exec_traj_state_ = LAND;
                ROS_INFO("[MPCctrl] Receive the trajectory. HOVER --> LAND");
            }
            else if (now_time >= trajectory_data_.total_traj_start_time &&
                now_time <= trajectory_data_.total_traj_end_time &&
                trajectory_data_.exec_traj == 1 && (!trajectory_data_.traj_queue.empty())
                && is_command_mode_ && state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
            {
                // same as the below
                set_hov_with_odom();
                oneTraj_Data_t *traj_info = &trajectory_data_.traj_queue.front();
                traj_info = &trajectory_data_.traj_queue.front();
                trajectory_data_.total_traj_start_time = traj_info->traj_start_time;

                double traj_time = (now_time - traj_info->traj_start_time).toSec();
                ommpc_controller_.setTrajectoryReference(traj_info->traj, traj_time, hover_pose_(3), traj_info->yaw_traj, odom_data_);
                ret = ommpc_controller_.execMPC(odom_data_, u);
                exec_traj_state_ = POLY_TRAJ;
                ROS_INFO("[MPCctrl] Receive the trajectory. HOVER --> POLY_TRAJ");
            }
            else if (param_.use_ref_txt && is_command_mode_ && state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD)
            {
                exec_traj_state_ = POINTS;
                ROS_INFO("[MPCctrl] Start executing traj from txt. HOVER --> POINTS");
                // same as the below
                double yaw_now = get_yaw_from_quaternion(odom_data_.q);
                get_txt_des();
                ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, yaw_now, yaws_);
                ret = ommpc_controller_.execMPC(odom_data_, u);
            }
            else
            {
                ommpc_controller_.setHoverReference(hover_pose_);
                ret = ommpc_controller_.execMPC(odom_data_, u);
            }
        }
        break;

        case POLY_TRAJ:
        {
            if (now_time < (trajectory_data_.total_traj_start_time) || now_time > trajectory_data_.total_traj_end_time || trajectory_data_.exec_traj != 1 || trajectory_data_.traj_queue.empty())
            {
                if (param_.use_trajectory_ending_pos && trajectory_data_.exec_traj != -1)
                {
                    // tracking the end point of the trajectory
                    // the hover pose is the end point of the trajectory
                    auto &traj_info = trajectory_data_.traj_queue.front().traj;
                    hover_pose_.head<3>() = traj_info.getJuncPos(traj_info.getPieceNum());
                    hover_pose_(3) = get_yaw_from_quaternion(odom_data_.q);
                    // std::cout << "hover_pose_ = " << hover_pose_.transpose() << std::endl;
                }
                else
                {
                    set_hov_with_odom();
                }
                ommpc_controller_.setHoverReference(hover_pose_);
                ret = ommpc_controller_.execMPC(odom_data_, u);
                exec_traj_state_ = HOVER;
                ROS_INFO("[MPCctrl] Stop execute the trajectory. POLY_TRAJ --> HOVER");
                trajectory_data_.exec_traj = 0;
            }
            else
            {
                set_hov_with_odom();
                // std::cout << "hover_pose_ = " << hover_pose_.transpose() << std::endl;
                oneTraj_Data_t *traj_info = &trajectory_data_.traj_queue.front();
                if (now_time < (traj_info->traj_start_time))
                { // the start time of first trajectory should be whole trajectory start time
                    trajectory_data_.total_traj_start_time = traj_info->traj_start_time;
                    ommpc_controller_.setHoverReference(hover_pose_);
                    ret = ommpc_controller_.execMPC(odom_data_, u);
                }
                else
                {
                    if (trajectory_data_.traj_queue.size() > 1)
                    {
                        ROS_ERROR("trajectory_data_.traj_queue.size() > 1");
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
                    ommpc_controller_.setTrajectoryReference(traj_info->traj, traj_time, hover_pose_(3), traj_info->yaw_traj, odom_data_);
                    ret = ommpc_controller_.execMPC(odom_data_, u);
                }
            }
        }
        break;

        case POINTS:
        {
            double yaw_now = get_yaw_from_quaternion(odom_data_.q);
            get_txt_des();
			ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, yaw_now, yaws_);
            ret = ommpc_controller_.execMPC(odom_data_, u);
            if (!is_command_mode_)
            {
                set_hov_with_odom();
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
                    (ros::Time::now().toSec() - start_takeoff_land_time + i * param_.step_T) * param_.takeoff_land_speed, 
                    param_.takeoff_height );
                quad_positions_[i] = Eigen::Vector3d(
                                    hover_pose_(0),
                                    hover_pose_(1),
                                    altitude);
                quad_velocities_[i] = Eigen::Vector3d(0.0, 0.0, param_.takeoff_land_speed);
                yaws_[i] = hover_pose_(4);
            }
            double yaw_now = get_yaw_from_quaternion(odom_data_.q);
            ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, yaw_now, yaws_);
            ret = ommpc_controller_.execMPC(odom_data_, u);
            if (odom_data_.p(2) > param_.takeoff_height)
            {
                exec_traj_state_ = HOVER;
                set_hov_with_odom();
                ROS_INFO("[MPCctrl] TAKEOFF succeeded. TAKEOFF --> HOVER");
            }
        }
        break;

        case LAND:
        {
            land_trigger_ = false;
            const double land_height = -0.5;
            double altitude;
            auto now_time = ros::Time::now();
            for (int i = 0; i < nstep + 1; ++i)
            {
                altitude = std::max( 
                    hover_pose_(2) + (ros::Time::now().toSec() - start_takeoff_land_time + i * param_.step_T) * (-param_.takeoff_land_speed), 
                    land_height);
                quad_positions_[i] = Eigen::Vector3d(
                                    hover_pose_(0),
                                    hover_pose_(1),
                                    altitude);
                quad_velocities_[i] = Eigen::Vector3d(0.0, 0.0, -param_.takeoff_land_speed);
                yaws_[i] = hover_pose_(4);
            }
            double yaw_now = get_yaw_from_quaternion(odom_data_.q);
            ommpc_controller_.setTextReference(quad_positions_, quad_velocities_, odom_data_, yaw_now, yaws_);
            ret = ommpc_controller_.execMPC(odom_data_, u);
            static double last_trial_time = 0; // Avoid too frequent calls
            if (odom_data_.v.norm() < 0.1 && altitude < -0.4)
            {
				if (now_time.toSec() - last_trial_time > 1.0)
				{
					if (toggle_arm_disarm(false)) // disarm
					{
						exec_traj_state_ = HOVER;
                        set_hov_with_odom();
						ROS_INFO("[MPCctrl] LAND succeeded. LAND --> HOVER");
					}

					last_trial_time = now_time.toSec();
				}
            }
        }
        break;

        default:
        {
            exec_traj_state_ = HOVER;
            ROS_ERROR("[MPCctrl] Unknown exec_traj_state_! Jump to HOVER.");
        }

        break;
        }

        if (ret)
        {
            send_cmd(u);
        }
        else
        {
            exec_traj_state_ = HOVER;
            ROS_ERROR("[MPCctrl] Numerical error!");
        }
        
        if(state_.mode == mavros_msgs::State::MODE_PX4_OFFBOARD && state_.armed == true)
        {
            ommpc_controller_.estimateThrustModel(imu_data_.a);
        }
        exec_timer_.start();
    }

    bool readDataFromFile()
    {
        std::string traj_path = ros::package::getPath("ommpc_controller") + param_.ref_filename;
        std::ifstream file(traj_path.c_str());
        std::string line;

        if (file.is_open())
        {
            std::cout << "File " << traj_path  << " opened." << std::endl;
            while (getline(file, line))
            {
                number_of_steps_++;
                std::istringstream linestream(line);
                std::vector<double> linedata;
                double number;

                while (linestream >> number)
                {
                    linedata.push_back(number);
                }
                test_trajectory_.push_back(linedata);
            }
            file.close();
            std::cout << "File closed successfully, " << number_of_steps_ << " lines read." << std::endl;
            return true;
        }

        return false;
    }

    void get_txt_des()
    {
        for (int cnt = line_cnt_; cnt <= line_cnt_ + nstep; cnt++)
        {
            int i = cnt - line_cnt_;
            if (nstep + cnt + 1 < number_of_steps_)
            {
                quad_positions_[i] = Eigen::Vector3d(
                                    test_trajectory_[line_cnt_+i][0],
                                    test_trajectory_[line_cnt_+i][1],
                                    test_trajectory_[line_cnt_+i][2]);
                quad_velocities_[i] = Eigen::Vector3d(
                                    test_trajectory_[line_cnt_+i][3],
                                    test_trajectory_[line_cnt_+i][4],
                                    test_trajectory_[line_cnt_+i][5]);
                yaws_[i] = test_trajectory_[line_cnt_+i][6];
            }
            else
            {
                quad_positions_[i] = Eigen::Vector3d(
                                    test_trajectory_[number_of_steps_ - 1][0],
                                    test_trajectory_[number_of_steps_ - 1][1],
                                    test_trajectory_[number_of_steps_ - 1][2]);
                quad_velocities_[i] = Eigen::Vector3d::Zero();
                yaws_[i] = test_trajectory_[number_of_steps_ - 1][6];
            }
        }
        // std::cout << quad_positions_[0] << std::endl;

        line_cnt_++;
        if (line_cnt_ > number_of_steps_)
            line_cnt_ = number_of_steps_;
        return;
    }

    template <typename TName, typename TVal>
	void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
	{
		if (nh.getParam(name, val))
		{
			// pass
		}
		else
		{
			ROS_ERROR_STREAM("Read param_: " << name << " failed.");
			ROS_BREAK();
		}
	};

public:
    OMMPC_EXAMPLE(/* args */){};
    ~OMMPC_EXAMPLE(){};
    void init(ros::NodeHandle &nh){
        enu_frame_ = true;
        // for real world flight, vel_in_body should be set to false!
        vel_in_body_ = true;
        exec_traj_state_ = HOVER;

        cmd_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
        set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
        arming_client_srv_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, &OMMPC_EXAMPLE::OdomCallback, this);
        imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", 10, &OMMPC_EXAMPLE::IMUCallback, this);
        state_sub_ = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &OMMPC_EXAMPLE::StateCallback, this);
        mpc_traj_sub_ = nh.subscribe<traj_utils::PolyTraj>("/drone_0_planning/trajectory",
                                        100,
                                        boost::bind(&Trajectory_Data_t::feed_from_traj_utils, &trajectory_data_, _1),
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

        state_change_cb_type_ = boost::bind(&OMMPC_EXAMPLE::stateChangeCallback, this, _1, _2);
        state_change_server_.setCallback(state_change_cb_type_);

        read_essential_param(nh, "takeoff_height", param_.takeoff_height);
        read_essential_param(nh, "takeoff_land_speed", param_.takeoff_land_speed);
        read_essential_param(nh, "ref_txt/enable", param_.use_ref_txt);
        read_essential_param(nh, "ref_txt/time_step", param_.ref_time_step);
        read_essential_param(nh, "ref_txt/ref_filename", param_.ref_filename);
        read_essential_param(nh, "hover_percentage", param_.hover_percent);
        read_essential_param(nh, "MPC_params/Q_pos_xy", param_.Q_pos_xy);
        read_essential_param(nh, "MPC_params/Q_pos_z", param_.Q_pos_z);
        read_essential_param(nh, "MPC_params/Q_attitude_rp", param_.Q_attitude_rp);
        read_essential_param(nh, "MPC_params/Q_attitude_yaw", param_.Q_attitude_yaw);
        read_essential_param(nh, "MPC_params/Q_velocity", param_.Q_velocity);
        read_essential_param(nh, "MPC_params/R_thrust", param_.R_thrust);
        read_essential_param(nh, "MPC_params/R_pitchroll", param_.R_pitchroll);
        read_essential_param(nh, "MPC_params/R_yaw", param_.R_yaw);
        read_essential_param(nh, "MPC_params/min_thrust", param_.min_thrust);
        read_essential_param(nh, "MPC_params/max_thrust", param_.max_thrust);
        read_essential_param(nh, "MPC_params/max_bodyrate_xy", param_.max_bodyrate_xy);
        read_essential_param(nh, "MPC_params/max_bodyrate_z", param_.max_bodyrate_z);
        read_essential_param(nh, "MPC_params/state_cost_exponential", param_.state_cost_exponential);
        read_essential_param(nh, "MPC_params/input_cost_exponential", param_.input_cost_exponential);
        read_essential_param(nh, "MPC_params/step_T", param_.step_T);
        read_essential_param(nh, "use_fix_yaw", param_.use_fix_yaw);
        read_essential_param(nh, "use_trajectory_ending_pos", param_.use_trajectory_ending_pos);

        if (param_.use_ref_txt){
            std::cout << "Ref trajectory enabled!" << std::endl;
            if (!readDataFromFile())
                std::cout << "File input error!" << std::endl;
        }

        quad_positions_.clear();
        quad_positions_.resize(nstep + 1);
        quad_velocities_.clear();
        quad_velocities_.resize(nstep + 1);
        yaws_.clear();
        yaws_.resize(nstep + 1);

        ommpc_controller_.init(param_);

        hover_pose_ << 0, 0, 0.0, 0;

        exec_timer_ = nh.createTimer(ros::Duration(0.01), &OMMPC_EXAMPLE::execFSMCallback, this);
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "ommpc_controller_example_node");
    ros::NodeHandle nh("~");

    OMMPC_EXAMPLE ommpc_example;
    ommpc_example.init(nh);

    ros::spin();

    return 0;
}