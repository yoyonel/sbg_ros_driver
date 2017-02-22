project(sbglog_parsers_simple)

file(GLOB_RECURSE SRC
	${CMAKE_CURRENT_LIST_DIR}/src/*.cpp
	${PROJECT_SOURCE_DIR}/../../../include/sbglogparser/simple/*.h
)
MESSAGE("SRC: " ${SRC})

add_std_cxx11_flag("${SRC}")

add_library(
    sbglog_parsers_simple
    STATIC
    ${SRC}
)

# add_dependencies(
#     sbglog_parsers_simple
#     ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#     ${catkin_EXPORTED_TARGETS}
# )