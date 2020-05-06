cmake_minimum_required(VERSION 2.8.3)
project(milestone-3)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf2_ros
  roslib
)

set(CMAKE_PREFIX_PATH /usr/local/lib/cmake)
find_package(
  OsqpEigen
)

catkin_package(

)

###########
## Build ##
###########
include_directories(
  inc
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  /usr/local/include/eigen3/Eigen
)

add_executable(hrhc_node
node/hrhc_node.cpp
src/constraints.cpp
src/cost.cpp
src/hrhc.cpp
src/input.cpp
src/model.cpp
src/mpc.cpp
src/occgrid.cpp
src/state.cpp
src/trajectory_planner.cpp
src/transforms.cpp
src/visualizer.cpp
src/trajectory.cpp
)
add_dependencies(hrhc_node ${f110_hrhc_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(
  hrhc_node
  ${catkin_LIBRARIES}
  stdc++fs
  OsqpEigen::OsqpEigen osqp::osqp
)
