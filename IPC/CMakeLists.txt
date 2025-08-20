cmake_minimum_required(VERSION 3.0.2)
project(ipc)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OsqpEigen REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  eigen_conversions
  visualization_msgs
  pcl_ros
  tf
  mars_base
  rog_map
  rosfmt
)

catkin_package(
  INCLUDE_DIRS include
)

add_subdirectory(backward-cpp)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(ipc_node
  src/main_ipc.cpp
  src/ipc_fsm.cpp
  include/astar_rm.cpp
  include/callback.cpp
  include/ciri.cpp
  include/emvp.cpp
  include/mpc.cpp
  ${BACKWARD_ENABLE}
)
add_dependencies(ipc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ipc_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${PCL_LIBRARIES}
  OsqpEigen::OsqpEigen
)

add_backward(ipc_node)
