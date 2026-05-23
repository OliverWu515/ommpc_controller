cmake_minimum_required(VERSION 3.0.2)
project(ommpc_ros)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS "YES")

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mavros_msgs
  nav_msgs
  ommpc_core
  roscpp
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  traj_utils
)

find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/state.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ommpc_ros_adapter
  CATKIN_DEPENDS
    dynamic_reconfigure
    geometry_msgs
    mavros_msgs
    nav_msgs
    ommpc_core
    roscpp
    roslib
    rospy
    sensor_msgs
    std_msgs
    tf
    traj_utils
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(ommpc_ros_adapter
  src/mavros_command_interface.cpp
  src/text_reference_loader.cpp
)

target_link_libraries(ommpc_ros_adapter
  ${catkin_LIBRARIES}
)

add_executable(ommpc_example_node
  src/ommpc_example.cpp
)

target_link_libraries(ommpc_example_node
  ommpc_ros_adapter
  ${catkin_LIBRARIES}
)

add_dependencies(ommpc_example_node
  ${PROJECT_NAME}_gencfg
  ${catkin_EXPORTED_TARGETS}
)
