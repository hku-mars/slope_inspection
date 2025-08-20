file(GLOB_RECURSE srcs CONFIGURE_DEPENDS src/*.cpp include/*.h)
add_library(type_utils STATIC ${srcs})
target_include_directories(type_utils PUBLIC include)