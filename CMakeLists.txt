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

message("WEBOTS HOME: ${WEBOTS_HOME}")

# Webots includes
set(WEBOTS_INCLUDE
    ${WEBOTS_HOME}/include/controller/cpp
    ${WEBOTS_HOME}/include/controller/cpp/utils
)

# Link executable to Webots C++ Controller Library
find_library(WEBOTS_CONTROLLER_LIB CppController ${WEBOTS_HOME}/lib)

## Set source files
list(APPEND SRC_FILES
    src/NaoWebots.cpp
    src/NaoWebots.h
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

## Declare a C++ executable
add_executable("${PROJECT_NAME}_node" src/main_node.cpp ${SRC_FILES})

## Specify libraries to link a library or executable target against
target_link_libraries("${PROJECT_NAME}_node"
  ${catkin_LIBRARIES}
  ${WEBOTS_CONTROLLER_LIB}
)

## Copy the executable into the controllers folder
## TODO: Find a clever way to do this
set(EXECUTABLE_PATH ${HOME}/catkin_ws/devel/lib/nao_webots/nao_webots_node)
set(WEBOTS_CONTROLLER_PATH ${WEBOTS_HOME}/projects/robots/aldebaran/controllers/nao_webots_node/)

file(MAKE_DIRECTORY ${WEBOTS_CONTROLLER_PATH})
execute_process(COMMAND ${CMAKE_COMMAND} -E make_directory ${WEBOTS_CONTROLLER_PATH})

if(EXISTS ${EXECUTABLE_PATH})
    configure_file(${EXECUTABLE_PATH} ${WEBOTS_CONTROLLER_PATH} COPYONLY)
else()
    message("-- WARNING: Controller not compiled yet")
endif()
