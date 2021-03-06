﻿cmake_minimum_required(VERSION 2.8.3)
project(sbg_driver)

#SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_BUILD_TYPE Debug)

cmake_policy(SET CMP0046 OLD)
cmake_policy(SET CMP0015 NEW)

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

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    message_generation
    message_filters
    dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )
# Generate messages in the 'msg' folder
add_message_files(
    DIRECTORY msg
    FILES
#
    SbgLogAbstract.msg
#
    SbgLogStatusData.msg
    SbgLogImuData.msg
    SbgLogEkfEulerData.msg
    SbgLogEkfQuatData.msg
    SbgLogEkfNavData.msg
    SbgLogShipMotionData.msg
    SbgLogOdometerData.msg
    SbgLogUtcData.msg
    SbgLogGpsPos.msg
    SbgLogGpsVel.msg
    SbgLogGpsHdt.msg
    SbgLogGpsRaw.msg
    SbgLogMag.msg
    SbgLogMagCalib.msg
    SbgLogDvlData.msg
    SbgLogPressureData.msg
    SbgLogUsblData.msg
    SbgLogEvent.msg
    SbgLogDebug0Data.msg
    SbgLogFastImuData.msg
    #
    SbgEComDeviceInfo.msg
    #
    gps.msg
)


## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
        cfg/ELLIPSE_N_settings.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    # INCLUDE_DIRS sbgECom
    # LIBRARIES ${PROJECT_NAME}
    #  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
    # DEPENDS sbgECom
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS message_runtime message_filters std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
#
        ${PROJECT_SOURCE_DIR}/sbgECom/common
        ${PROJECT_SOURCE_DIR}/sbgECom/src
#
        ${PROJECT_SOURCE_DIR}/include
)

MESSAGE(STATUS "Catkin directories: ${catkin_INCLUDE_DIRS}")

# file(GLOB_RECURSE SRC ${PROJECT_SOURCE_DIR}/sbgECom/src/*.c)
# file(GLOB_RECURSE COMMON_SRC ${PROJECT_SOURCE_DIR}/sbgECom/common/*.c)
# list(REMOVE_ITEM COMMON_SRC ${PROJECT_SOURCE_DIR}/sbgECom/common/interfaces/sbgInterfaceSerialWin.c)

# MESSAGE( STATUS ${COMMON_SRC})

## Declare a C++ library
# add_library(sbg
#   src/${PROJECT_NAME}/sbg.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
#add_dependencies(ellipse ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
SET(SRC
    src/ellipse.cpp
    #
    src/sbgwrapper/src/sbgwrapper.cpp
    #
    ##
    src/sbglogparser/boost/src/SBGLogtoROSPublisher.cpp
    src/sbglogparser/boost/src/SBGLogtoROSPublisher_Visitor.cpp
    src/sbglogparser/boost/src/sbglogparser.cpp
    ##
    src/sbglogparser/std/src/sbglogparser.cpp
    ##
    src/sbglogparser/simple/src/sbglogparser.cpp
)

add_std_cxx11_flag("${SRC}")

add_executable(
        ellipse
        ${SRC}
        ${COMMON_SRC}
)

#target_compile_options(ellipse PRIVATE -fpermissive)

# include(src/sbglogparser/CMakeLists.txt)
# include(src/sbgwrapper/CMakeLists.txt)

## Specify libraries to link a library or executable target against
add_subdirectory(sbgECom)

##################
## Dependencies ##
##################

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(sbg_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(ellipse sbgECom)
# add_dependencies(
#     ellipse 
#     ${${PROJECT_NAME}_EXPORTED_TARGETS} 
#     ${catkin_EXPORTED_TARGETS}
# )

#MESSAGE("TEST -> catkin_EXPORTED_TARGETS: ${catkin_EXPORTED_TARGETS}")

target_link_libraries(
    ellipse
    ${catkin_LIBRARIES}
    # sbglog_parsers
    # sbg_wrapper
    sbgECom
)

add_dependencies(ellipse ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# url: http://docs.ros.org/jade/api/catkin/html/howto/format2/building_libraries.html
## Mark executables and/or libraries for installation
# install(TARGETS ellipse sbglog_parsers sbg_wrapper
install(TARGETS ellipse
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
        launch/ELLIPSE_N.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sbg.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



### Declare a C++ executable
#add_std_cxx11_flag("src/test/wrapper.cpp")

#add_executable(
#    wrapper
#    src/test/wrapper.cpp
#)

#target_link_libraries(
#    wrapper
#    ${catkin_LIBRARIES}
#    sbgECom
#)

