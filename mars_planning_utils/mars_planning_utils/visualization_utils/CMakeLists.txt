file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(visualization_utils STATIC ${srcs})
target_include_directories(visualization_utils PUBLIC include ${PCL_INCLUDE_DIRS})
target_link_libraries(visualization_utils PUBLIC
        ${catkin_LIBRARIES}
        geometry_utils
        type_utils)