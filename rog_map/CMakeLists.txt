cmake_minimum_required(VERSION 2.8.3)
project(rog_map)

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
        visualization_msgs
        message_filters
        # MaRS packages
        mars_planning_utils
        )
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES rog_map
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
        -ldw
        )

file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h include/*.hpp include/*.cpp)
add_library(rog_map STATIC ${srcs}
        )
target_include_directories(rog_map PUBLIC include)
target_link_libraries(rog_map PUBLIC
        mars_planning_utils
        ${THIRD_PARTY})


add_executable(rog_map_bag_test
        Apps/rog_test_with_bag.cpp
        )
target_link_libraries(rog_map_bag_test
        ${THIRD_PARTY}
        rog_map
        )

add_executable(rogmap_node
        Apps/rogmap_test_node.cpp
        )
target_link_libraries(rogmap_node
        ${THIRD_PARTY}
        rog_map
        )

add_executable(example_astar
        example/example_astar_search.cpp
        )
target_link_libraries(example_astar
        ${THIRD_PARTY}
        rog_map
        )


add_executable(viz_real_bag
        Apps/viz_real_bag.cpp
        )
target_link_libraries(viz_real_bag
        ${THIRD_PARTY}
        rog_map
        )
