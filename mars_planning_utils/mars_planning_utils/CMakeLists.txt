cmake_minimum_required(VERSION 2.8.3)
project(mars_planning_utils)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")
add_compile_options(-Werror=return-type)
add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        pcl_ros
        geometry_msgs
        nav_msgs
        message_generation
        visualization_msgs
        message_filters
        rosfmt
        )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(PythonLibs REQUIRED)
catkin_package(
        INCLUDE_DIRS
        LIBRARIES mars_planning_utils
        CATKIN_DEPENDS roscpp rospy std_msgs
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${PYTHON_LIBRARIES}
        -ldw
        )


#add_library(mars_planning_utils STATIC
#        src/mars_planning_utils.cpp
#        src/visualization_utils.cpp
#        math_utils/test/lbfgs_test.cpp)
#target_link_libraries(mars_planning_utils
#        ${THIRD_PARTY}
#        )


add_subdirectory(math_utils)
add_subdirectory(type_utils)
add_subdirectory(geometry_utils)
add_subdirectory(benchmark_utils)
add_subdirectory(debug_utils)
add_subdirectory(visualization_utils)
add_subdirectory(optimization_utils)


add_library(mars_planning_utils INTERFACE)

target_include_directories(mars_planning_utils INTERFACE
        math_utils
        type_utils
        benchmark_utils
        geometry_utils
        visualization_utils
        debug_utils
        optimization_utils
        )
target_link_libraries(mars_planning_utils INTERFACE
        math_utils
        type_utils
        benchmark_utils
        geometry_utils
        visualization_utils
        debug_utils
        optimization_utils
        ${CATKIN_LIBRARIES}
        )


add_executable(test_mars_planning_utils
        test/test_mars_planning_utils.cpp)
target_link_libraries(test_mars_planning_utils PUBLIC
        ${THIRD_PARTY}
        mars_planning_utils
        )
target_include_directories(test_mars_planning_utils PUBLIC
        mars_planning_utils
        )

add_executable(test_root_finder
        test/root_finder_test.cpp)
target_link_libraries(test_root_finder PUBLIC
        ${THIRD_PARTY}
        mars_planning_utils
        )
target_include_directories(test_root_finder PUBLIC
        mars_planning_utils
        )

add_executable(minco_fix_pv
        test/minco_fix_pv.cpp)
target_link_libraries(minco_fix_pv PUBLIC
        ${THIRD_PARTY}
        mars_planning_utils
        )
target_include_directories(minco_fix_pv PUBLIC
        mars_planning_utils
        )