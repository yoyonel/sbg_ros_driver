cmake_minimum_required(VERSION 2.8.3)

project(sbg_wrapper)

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

add_std_cxx11_flag("${PROJECT_SOURCE_DIR}/src/sbgwrapper/src/sbgwrapper.cpp")
add_std_cxx11_flag("${PROJECT_SOURCE_DIR}/include/sbgwrapper/sbgwrapper.h")
add_std_cxx11_flag("${PROJECT_SOURCE_DIR}/include/sbgexceptions.h")
#MESSAGE(STATUS "${PROJECT_SOURCE_DIR}/include/sbgexceptions.h")

include_directories(
        ${PROJECT_SOURCE_DIR}/include
        ${catkin_INCLUDE_DIRS}
)

add_library(sbg_wrapper STATIC
        ${PROJECT_SOURCE_DIR}/src/sbgwrapper/src/sbgwrapper.cpp
        ${PROJECT_SOURCE_DIR}/include/sbgwrapper/sbgwrapper.h
        ${PROJECT_SOURCE_DIR}/include/sbgexceptions.h
)
