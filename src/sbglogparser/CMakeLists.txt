cmake_minimum_required(VERSION 2.8.3)
project(sbglog_parsers)

macro(add_flags SRC_FILES FLAGS)
    #  MESSAGE("Flags list to add: ${FLAGS}")
    #  MESSAGE("to this sources files:")
    foreach (src ${SRC_FILES})
        #    MESSAGE("-> ${src}")
        set_source_files_properties(${src} PROPERTIES COMPILE_FLAGS ${FLAGS})
    endforeach ()
endmacro()

macro(add_std_cxx11_flag SRC_FILES)
    add_flags("${SRC_FILES}" "-std=c++11")
endmacro()

include_directories(
        ${PROJECT_SOURCE_DIR}/include
)

include(src/sbglogparser/boost/CMakeLists.txt)
include(src/sbglogparser/std/CMakeLists.txt)
include(src/sbglogparser/simple/CMakeLists.txt)

# url: http://stackoverflow.com/questions/34234446/cmake-is-it-possible-to-build-an-executable-from-only-static-libraries-and-no-s
file(WRITE null.cpp "")

add_library(
        sbglog_parsers
        null.cpp
)

target_link_libraries(
        sbglog_parsers
        sbglog_parsers_boost
        sbglog_parsers_std
        sbglog_parsers_simple
)

