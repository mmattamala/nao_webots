cmake_minimum_required(VERSION 2.8.3)
project(nao_webots)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
  message_generation
  tf
)

find_package(Boost REQUIRED COMPONENTS thread)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## Debug flag
set(CMAKE_BUILD_TYPE Release)

## Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

## Set support for C++11
macro(use_cxx11)
  if (CMAKE_VERSION VERSION_LESS "3.1")
    if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
      set (CMAKE_CXX_FLAGS "--std=c++11 ${CMAKE_CXX_FLAGS}")
    endif ()
  else ()
    set (CMAKE_CXX_STANDARD 11)
  endif ()
endmacro(use_cxx11)

use_cxx11()

## Webots stuff
# Webots environment variable WEBOTS_HOME (should be set before)
# if not defined, set WEBOTS_HOME environment variable to default value
if (DEFINED ENV{WEBOTS_HOME})
    set(WEBOTS_HOME $ENV{WEBOTS_HOME})
else()
    set(WEBOTS_HOME /usr/local/webots)
endif()

#message("WEBOTS HOME: ${WEBOTS_HOME}")

# Webots includes
set(WEBOTS_INCLUDE
    ${WEBOTS_HOME}/include/controller/cpp
    ${WEBOTS_HOME}/include/controller/cpp/utils
)

# Link executable to Webots C++ Controller Library
find_library(WEBOTS_CONTROLLER_LIB CppController ${WEBOTS_HOME}/lib)

## Set source files
list(APPEND SRC_SUPERVISOR_FILES
    src/NaoSupervisor.cpp
    src/NaoSupervisor.h
)

list(APPEND SRC_CONTROLLER_FILES
    src/NaoController.cpp
    src/NaoController.h
)

## Catkin package
catkin_package(
  INCLUDE_DIRS src
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs CppController
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${WEBOTS_INCLUDE}
  src
)

## TODO: Find a clever way to do this
#set(EXECUTABLE_PATH $ENV{HOME}/catkin_ws/devel/lib/nao_webots/nao_webots_node)
set(WEBOTS_CONTROLLER_PATH ${WEBOTS_HOME}/resources/projects/controllers)


## Supervisor node
add_executable(nao_supervisor_node src/supervisor_node.cpp ${SRC_SUPERVISOR_FILES})

## Controller node
add_executable(nao_controller_node src/controller_node.cpp ${SRC_CONTROLLER_FILES})


## Specify libraries to link a library or executable target against
target_link_libraries(nao_controller_node
  ${catkin_LIBRARIES}
  ${WEBOTS_CONTROLLER_LIB}
)

target_link_libraries(nao_supervisor_node
  ${catkin_LIBRARIES}
  ${WEBOTS_CONTROLLER_LIB}
)

# copy executables to webots folder
add_custom_command(TARGET nao_supervisor_node POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${WEBOTS_CONTROLLER_PATH}/nao_supervisor_node/"
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:nao_supervisor_node> "${WEBOTS_CONTROLLER_PATH}/nao_supervisor_node/"
)

add_custom_command(TARGET nao_controller_node POST_BUILD
    #COMMAND ${CMAKE_COMMAND} -E make_directory "${WEBOTS_CONTROLLER_PATH}/nao_controller_node/"
    #COMMAND ${CMAKE_COMMAND} -E make_directory "${WEBOTS_CONTROLLER_PATH}/nao_controller_node/motions"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${WEBOTS_CONTROLLER_PATH}/nao_controller_node/"
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:nao_controller_node> "${WEBOTS_CONTROLLER_PATH}/nao_controller_node/"
    #COMMAND ${CMAKE_COMMAND} -E copy_directory "${PROJECT_SOURCE_DIR}/motions" "${WEBOTS_CONTROLLER_PATH}/nao_controller_node/"
    #COMMAND ln -sf "${PROJECT_SOURCE_DIR}/motions/*.motion" "${WEBOTS_CONTROLLER_PATH}/nao_controller_node/motions/"
    COMMAND ln -sf "${PROJECT_SOURCE_DIR}/motions" "${WEBOTS_CONTROLLER_PATH}/nao_controller_node/motions"
)
