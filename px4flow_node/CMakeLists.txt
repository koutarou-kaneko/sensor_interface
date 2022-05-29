cmake_minimum_required(VERSION 3.0.2)
project(px4flow_node)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs sensor_msgs image_transport)
find_package(Boost REQUIRED COMPONENTS thread system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs image_transport
)

###########                                                                         
## Build ##                                                                         
###########                                                                         

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(px4flow_node
  src/SerialComm.cc
  src/px4flow_node.cc
)

target_link_libraries(px4flow_node
  ${catkin_LIBRARIES}
)

#############                                                                       
## Install ##                                                                       
#############                                                                       

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS px4flow_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)


