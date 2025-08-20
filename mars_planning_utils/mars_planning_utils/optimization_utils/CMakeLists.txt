file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(optimization_utils STATIC ${srcs})
target_include_directories(optimization_utils PUBLIC include)
target_link_libraries(optimization_utils PUBLIC
        geometry_utils
        benchmark_utils
        type_utils
        )