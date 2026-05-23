cmake_minimum_required(VERSION 3.0.2)
project(ommpc_core)
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS "YES")

find_package(catkin REQUIRED)

find_package(Eigen3 REQUIRED)

# OSQP: try pkg-config first, fall back to find_library
find_package(PkgConfig QUIET)
if(PkgConfig_FOUND)
  pkg_check_modules(OSQP QUIET osqp)
endif()
if(NOT OSQP_FOUND)
  find_library(OSQP_LIBRARY NAMES osqp)
  find_path(OSQP_INCLUDE_DIR NAMES osqp/osqp.h)
  if(OSQP_LIBRARY AND OSQP_INCLUDE_DIR)
    set(OSQP_FOUND TRUE)
    set(OSQP_LIBRARIES ${OSQP_LIBRARY})
    set(OSQP_INCLUDE_DIRS ${OSQP_INCLUDE_DIR})
  endif()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ommpc_core
  DEPENDS EIGEN3
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

if(OSQP_INCLUDE_DIRS)
  include_directories(${OSQP_INCLUDE_DIRS})
endif()

add_library(ommpc_core
  src/controller_core.cpp
)

target_link_libraries(ommpc_core
  ${OSQP_LIBRARIES}
)
