project(sbglog_parsers_boost)

# url: https://cmake.org/Wiki/CMake_Useful_Variables
file(GLOB_RECURSE SRC 
	${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
	${PROJECT_SOURCE_DIR}/../../../include/sbglogparser/boost/*.h
	${PROJECT_SOURCE_DIR}/../../../include/sbglogparser/boost/*.inl
)
MESSAGE(STATUS SRC: ${SRC})
#MESSAGE(STATUS  CMAKE_CURRENT_LIST_DIR : ${CMAKE_CURRENT_LIST_DIR})

add_std_cxx11_flag("${SRC}")

#include_directories(${PROJECT_SOURCE_DIR}/include)
#MESSAGE(STATUS "Catkin directories: ${catkin_INCLUDE_DIRS}")

add_library(
        sbglog_parsers_boost
        STATIC
        ${SRC}
)

# essentiel pour compiler avec boost::variant ... a voir pourquoi !
target_compile_options(sbglog_parsers_boost PRIVATE -fpermissive)

#MESSAGE("[Boost] TEST -> catkin_EXPORTED_TARGETS: ${catkin_EXPORTED_TARGETS}")
# add_dependencies(
#     sbglog_parsers_boost
#     ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#     ${catkin_EXPORTED_TARGETS}
# )