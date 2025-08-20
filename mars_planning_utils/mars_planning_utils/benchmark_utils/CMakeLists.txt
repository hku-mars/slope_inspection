file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(benchmark_utils STATIC ${srcs})
target_include_directories(benchmark_utils PUBLIC include)