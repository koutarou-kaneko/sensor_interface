cmake_minimum_required(VERSION 3.0.2)
project(lepton_thermal_sensor)

add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  image_transport
  nodelet
  roscpp
  dynamic_reconfigure
  sensor_msgs
)

generate_dynamic_reconfigure_options(
  cfg/LeptonCamera.cfg
  )


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES leddar_one_driver
  CATKIN_DEPENDS camera_info_manager cv_bridge image_transport nodelet roscpp dynamic_reconfigure sensor_msgs
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

add_library(sensor_handler
  src/i2c.cpp
  src/spi.cpp
  src/palettes.cpp
  src/sensor_handler.cpp)
target_link_libraries(sensor_handler
  leptonSDK
  ${catkin_LIBRARIES})

add_library(LeptonThermalSensorNodelet src/nodelet.cpp)
target_link_libraries(LeptonThermalSensorNodelet sensor_handler leptonSDK ${catkin_LIBRARIES})
add_dependencies(LeptonThermalSensorNodelet ${catkin_EXPORTED_TARGETS})

## option: V4L2 loopback mode
add_executable(v4l2lepton_node
  src/i2c.cpp
  src/spi.cpp
  src/palettes.cpp
  src/v4l2lepton.cpp)
target_link_libraries(v4l2lepton_node leptonSDK ${catkin_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS LeptonThermalSensorNodelet sensor_handler
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS v4l2lepton_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  USE_SOURCE_PERMISSIONS
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  USE_SOURCE_PERMISSIONS
)


install(FILES nodelet_plugins.xml 99-gpio.rules
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
