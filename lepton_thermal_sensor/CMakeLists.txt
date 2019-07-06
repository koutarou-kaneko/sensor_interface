cmake_minimum_required(VERSION 2.8)
project(lepton_thermal_sensor)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lepton_thermal_sensor
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src/leptonSDK/
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src/leptonSDK)

add_executable(v4l2lepton_node
  src/i2c.cpp
  src/spi.cpp
  src/palettes.cpp
  src/v4l2lepton.cpp
  )

target_link_libraries(v4l2lepton_node
  leptonSDK
  ${catkin_LIBRARIES}
  )
