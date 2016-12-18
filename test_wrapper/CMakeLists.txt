cmake_minimum_required(VERSION 2.8.3)

project(wrapper)

macro(add_flags SRC_FILES FLAGS)
#  MESSAGE("Flags list to add: ${FLAGS}")
#  MESSAGE("to this sources files:")
  foreach(src ${SRC_FILES})
#    MESSAGE("-> ${src}")
    set_source_files_properties(${src} PROPERTIES COMPILE_FLAGS ${FLAGS})
  endforeach()
endmacro()

macro(add_std_cxx11_flag SRC_FILES)
  add_flags("${SRC_FILES}" "-std=c++11")
endmacro()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  #
  ${PROJECT_SOURCE_DIR}/sbgECom/common
  ${PROJECT_SOURCE_DIR}/sbgECom/src
  #
  ${PROJECT_SOURCE_DIR}/include
)

## Declare a C++ executable
add_std_cxx11_flag("src/test/wrapper.cpp")

add_executable(
    wrapper
    src/test/wrapper.cpp
)

#target_compile_options(ellipse PRIVATE -fpermissive)

# Specify libraries to link a library or executable target against
add_subdirectory(sbgECom)

target_link_libraries(
    wrapper
    ${catkin_LIBRARIES}
    sbgECom
)
