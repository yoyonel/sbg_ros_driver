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

include_directories(${PROJECT_SOURCE_DIR}/include)

########################################################################################################################
# SUBDIRECTORIES => MODULES
########################################################################################################################
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/boost)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/std)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/simple)
########################################################################################################################

########################################################################################################################
# LIBRARY: sbglog_parsers
########################################################################################################################

# url: http://stackoverflow.com/questions/34234446/cmake-is-it-possible-to-build-an-executable-from-only-static-libraries-and-no-s
# fake source to build the library (no source, just adding modules)
file(WRITE ${CMAKE_CURRENT_LIST_DIR}/.null.cpp "")

add_library(
        sbglog_parsers
        ${CMAKE_CURRENT_LIST_DIR}/.null.cpp
)

#####################################################################
# DEPENDANCIES
#####################################################################
target_link_libraries(
        sbglog_parsers
        sbglog_parsers_boost
        sbglog_parsers_std
        sbglog_parsers_simple
)
#####################################################################

########################################################################################################################