cmake_minimum_required(VERSION 3.0.2)
project(mars_base)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  std_msgs
  sensor_msgs
  serial
  tf
  mavros
)

add_message_files(
  DIRECTORY msg
  FILES
  BuckParam.msg
  WS2812.msg
  WSRGB.msg
)

generate_messages(
  DEPENDENCIES
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
  LIBRARIES mars_base
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_library(mars_base
  include/mars_led.cpp
)
target_link_libraries(mars_base
  ${catkin_LIBRARIES}
)

add_executable(mars_led_node
  src/mars_led_node.cpp
)
add_dependencies(mars_led_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mars_led_node
  mars_base
  ${catkin_LIBRARIES}
)

add_executable(mars_led_test_node
  src/mars_led_test.cpp
)
add_dependencies(mars_led_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mars_led_test_node
  mars_base
  ${catkin_LIBRARIES}
)

add_executable(ws2812_ctrl_node
  src/ws2812_ctrl.cpp
)
add_dependencies(ws2812_ctrl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(ws2812_ctrl_node
  mars_base
  ${catkin_LIBRARIES}
)

add_executable(launch_led_node
  src/launch_led.cpp
)
add_dependencies(launch_led_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(launch_led_node
  mars_base
  ${catkin_LIBRARIES}
)

add_executable(swarm_led_node
  src/swarm_led.cpp
)
add_dependencies(swarm_led_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(swarm_led_node
  mars_base
  ${catkin_LIBRARIES}
)

add_executable(mars_rc_node
  src/mars_rc.cpp
)
add_dependencies(mars_rc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mars_rc_node
  mars_base
  ${catkin_LIBRARIES}
)