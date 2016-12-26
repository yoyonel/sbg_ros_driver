project(sbglog_parsers_std)

file(GLOB_RECURSE SRC ${CMAKE_CURRENT_LIST_DIR}/src/*.cpp)

add_std_cxx11_flag("${SRC}")

add_library(
        sbglog_parsers_std
        STATIC
        ${SRC}
)
