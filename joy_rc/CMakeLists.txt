cmake_minimum_required(VERSION 3.0.2)
project(joy_rc)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  mavros_msgs
  sensor_msgs
  serial
)

catkin_package(

)

include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)


add_executable(BT_x1_node
  src/BT_x1.cpp
)
target_link_libraries(BT_x1_node
  ${catkin_LIBRARIES}
)

add_executable(car_ctrl_node
  src/car_ctrl.cpp
)
target_link_libraries(car_ctrl_node
  ${catkin_LIBRARIES}
)

add_executable(key_ctrl_node
  src/key_ctrl.cpp
)
target_link_libraries(key_ctrl_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(subs_rc_node
  src/subs_rc.cpp
)
target_link_libraries(subs_rc_node
  ${catkin_LIBRARIES}
)
