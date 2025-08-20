cmake_minimum_required(VERSION 2.8.3)
project(viz_toolkit)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        message_generation
        quadrotor_msgs
        mars_planning_utils
        visualization_msgs
        )

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ../../../devel/include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )

add_executable(draw_bbox
        Apps/draw_bounding_box.cpp
        )
target_link_libraries(draw_bbox
        ${THIRD_PARTY}
        )

add_executable(draw_odom_mesh
        Apps/draw_odom.cpp
        )
target_link_libraries(draw_odom_mesh
        ${THIRD_PARTY}
        )


add_executable(draw_pointcloud_map
        Apps/draw_pointcloud_map.cpp
        )
target_link_libraries(draw_pointcloud_map
        ${THIRD_PARTY}
        )