project(sbglog_parsers_boost)

# url: https://cmake.org/Wiki/CMake_Useful_Variables
file(GLOB_RECURSE SRC ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)
#MESSAGE(STATUS SRC: ${SRC})
#MESSAGE(STATUS  CMAKE_CURRENT_LIST_DIR : ${CMAKE_CURRENT_LIST_DIR})

add_std_cxx11_flag("${SRC}")

#include_directories(${PROJECT_SOURCE_DIR}/include)
#MESSAGE(STATUS "Catkin directories: ${catkin_INCLUDE_DIRS}")

add_library(
        sbglog_parsers_boost
        ${SRC}
)

# essentiel pour compiler avec boost::variant ... a voir pourquoi !
target_compile_options(sbglog_parsers_boost PRIVATE -fpermissive)