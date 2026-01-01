# OMMPC-Controller

<!-- TOC -->

- [OMMPC-Controller](#ommpc-controller)
    - [Usage](#usage)
        - [Installing OSQP](#installing-osqp)
        - [Cloning the code and compiling](#cloning-the-code-and-compiling)
        - [Run the code](#run-the-code)
    - [Theory and parameters](#theory-and-parameters)
        - [Reading text trajectories](#reading-text-trajectories)
        - [Theory and parameters](#theory-and-parameters)
        - [Thrust Normalization](#thrust-normalization)
    - [Acknowledgment](#acknowledgment)

<!-- /TOC -->

## Usage

### Installing OSQP

The version of cmake on Ubuntu 20.04 is a bit old and cannot compile the latest version. Therefore, it is necessary to revert to a slightly older version.

```bash
git clone --recursive -b release-0.6.3 https://github.com/osqp/osqp.git
```

If the clone process completes without any errors, proceed directly to the next step. Otherwise, you can consider the following command:

```bash
git submodule update --init --recursive
```

Then, create a build directory:

```bash
cd osqp
mkdir build
cd build
```

Next, create the Makefiles and compile:
```bash
cmake -G "Unix Makefiles" ..
cmake --build .
```

Finally,
```bash
cmake --build . --target install
```
(May require administrator privileges)

Ensure that `libosqp.so` is present, then update the dynamic link library cache using the `ldconfig` command.

### Cloning the code and compiling

```bash
sudo apt install ros-noetic-ddynamic-reconfigure
cd catkin_ws/src
git clone https://github.com/OliverWu515/ommpc_controller.git
cd ..
catkin_make
```

### Run the code

```bash
source ./devel/setup.zsh
roslaunch px4 mavros_posix_sitl.launch
roslaunch ommpc_controller px4_example.launch
```

Takeoff: First, type the following command in the terminal:
```
rosservice call /mavros/set_mode 0 "OFFBOARD"
```

Then, check the `takeoff_enabled` box in the rqt_reconfigure window. The drone will automatically arm and take off.

Command: Check the `command_or_hover` box in the rqt_reconfigure window. The drone will then execute the text trajectory or polynomial trajectory.

Landing: Check the `land_enabled` box in the rqt_reconfigure window. The drone will automatically land and attempt to disarm. If disarming is successful, you can type the following command in the terminal:
```
rosservice call /mavros/set_mode 0 "MANUAL"
```
to return to manual control mode, or you can proceed directly to execute other commands.

## Theory and parameters

### Reading text trajectories

If reading text trajectories for testing is enabled, after entering command control mode, the file stored at the path `ref_txt/ref_filename` will be read line by line with a fixed step size `ref_txt/time_step`. The format of each line is:
```
x-position y-position z-position x-velocity y-velocity z-velocity yaw yaw-rate
```
In the `traj` folder, several scripts are stored for generating text trajectories as needed.  
Note: The text trajectory must not contain empty lines at the end, otherwise a memory error may occur and lead to a crash!

**Related parameters**:  
ref_txt:   
&emsp; enable: Allows the use of text trajectories. If set to `false`, commands from the planner are received instead.  
&emsp; time_step: The sampling step size for the text trajectory. Special attention: The step size for MPC and the text reference trajectory must be consistent. This requires ensuring that **this parameter, `MPC_params/step_T` (the MPC step size), and the step in the script generating the text trajectory** are the same!  
&emsp; ref_filename: The path to the text trajectory file, specified as a **relative path** relative to this project.

### Theory and parameters

For the derivation with respect to the dynamic of quadrotors, refer to the file `OMMPC_derivation.pdf` in this repo.

For more general conclusions, refer to [G. Lu, W. Xu and F. Zhang, "On-Manifold Model Predictive Control for Trajectory Tracking on Robotic Systems," in IEEE Transactions on Industrial Electronics, vol. 70, no. 9, pp. 9192-9202, Sept. 2023](https://ieeexplore.ieee.org/abstract/document/9917382).

There are three ways to set the reference trajectory and control inputs:
- Set a hover reference. Related function: `setHoverReference`. This essentially sets the reference pose for the next several steps to the current pose and the velocity to 0.
- Set the reference from a text trajectory. Related functions: `setTextReference`, `get_txt_des`. This function will also be used while setting references during takeoff and landing.
- Set the reference from a polynomial trajectory. Related functions: `setTrajectoryReference`, `feed_from_traj_utils`. If it is necessary to read polynomial trajectories in other formats, you can implement them yourself.

We adopt Hopf fibration to calculate the full state from the flat outputs.

**Related parameters**:  
mpc_enabled: Whether to enable NMPC control  
use_fix_yaw: Whether to fix the yaw angle   
use_trajectory_ending_pos: Whether to ensure reaching the trajectory endpoint, generally set to `true`  
MPC_params:  
&emsp; step_T: MPC step size. When using text trajectories, note that the text trajectory step size must be consistent with this.  
&emsp; Q_pos_xy, Q_pos_z: Weight for horizontal and vertical error, respectively. Should be set 1-3 orders of magnitude larger than other weights.  
&emsp; Q_attitude_rp, Q_attitude_yaw: Attitude error weight.  
&emsp; Q_velocity: Velocity error weight.  
&emsp; R_thrust: Thrust input penalty.  
&emsp; R_pitchroll, R_yaw: Angular velocity input penalty. These penalties should be relatively smaller, otherwise they affect control accuracy; but cannot be too small, otherwise the smoothness will be affected.  
&emsp; state_cost_exponential, input_cost_exponential: State/input error weight discount factor. The weight at step k is multiplied by exp{-k/total_steps * discount_factor}.  
&emsp; max_bodyrate_xy, max_bodyrate_z: Maximum angular rate. It is recommended to set the z-component relatively smaller.  
&emsp; min_thrust/max_thrust: Minimum/Maximum thrust (actually acceleration, unit: m/s^2)


### Thrust Normalization

Assume that

$$
t_{cmd}=\frac{a_{z,d}}{T_a}
$$

where $T_a$ is the normalization constant, which is determined by the physical characteristics of the quadrotor, and can be estimated by Kalman filtering

$$
\begin{aligned}
x_k&=T_{a,k} \\
z_k&=a_z=t_{cmd}\ T_{a,k}
\end{aligned}
$$

Then

$$
\begin{aligned}
\breve{P}_k&=1/\rho \\
K_k&=\frac{\breve{P}_k\cdot t_{cmd}}{t_{cmd} \breve{P}_k\cdot t_{cmd}+\rho} \\
\hat{T}_{a,k}&=\breve{T}_{a,k}+K_k(a_{z,imu}-t_{cmd} \breve{T}_{a,k}) \\
P_k&=(1-K_k\cdot t_{cmd})\cdot \breve{P}_k \\
\end{aligned}
$$

**Related parameters**  
hover_percentage: Used to set the initial value of Ta 


## Acknowledgment

- [Huizhe Li](https://github.com/haiyu1020) and [Yuhao Fang](https://github.com/fweiI/) for instruction of parameter tuning and basic structures of code
- [Autotrans](https://github.com/HKUST-Aerial-Robotics/AutoTrans/tree/main/) and [RPG MPC](https://github.com/uzh-rpg/rpg_mpc) for providing examples of setting reference for MPC
- [SUPER](https://github.com/hku-mars/Super) for providing supporting material including derivation of OMMPC
- [GCOPTER](https://github.com/ZJU-FAST-Lab/GCOPTER) and [SE(3) Controller](https://github.com/HITSZ-MAS/se3_controller) for providing hints of differential flatness with Hopf fibration
- [Fast-Drone-250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250/) for providing examples of using Finite State Machine (FSM) to manage states of quadrotors