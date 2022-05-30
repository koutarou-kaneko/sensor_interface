cmake_minimum_required(VERSION 3.0.2)
project(leddar_one)

find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs  dynamic_reconfigure
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS thread)
find_package(PkgConfig)
pkg_check_modules(MODBUS REQUIRED libmodbus)

generate_dynamic_reconfigure_options(
  cfg/LeddarOne.cfg
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES leddar_one_driver
  CATKIN_DEPENDS roscpp sensor_msgs dynamic_reconfigure
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${MODBUS_INCLUDE_DIRS}
  include
)

add_library(leddar_one_driver
  src/leddar_one_driver.cpp
  )
target_link_libraries(leddar_one_driver ${MODBUS_LIBRARIES}  ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
add_dependencies(leddar_one_driver  ${PROJECT_NAME}_gencfg)

add_executable(leddar_one_node src/leddar_one_node.cpp)
target_link_libraries(leddar_one_node  leddar_one_driver  ${catkin_LIBRARIES})
add_dependencies(leddar_one_node  ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS leddar_one_driver
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS leddar_one_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/ 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
