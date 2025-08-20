file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(debug_utils STATIC ${srcs})
target_include_directories(debug_utils PUBLIC include)