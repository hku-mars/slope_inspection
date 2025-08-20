file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(geometry_utils STATIC ${srcs})
target_include_directories(geometry_utils PUBLIC include)
target_link_libraries(geometry_utils PUBLIC
        visualization_utils
        type_utils
        math_utils)