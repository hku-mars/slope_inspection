file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/math_utils/*.h)
add_library(math_utils STATIC ${srcs})
target_include_directories(math_utils PUBLIC include)
