cmake_minimum_required(VERSION 2.8.3)
project(jsk_stm)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
find_package(Boost REQUIRED COMPONENTS thread system signals)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES jsk_stm
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS boost
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library (serial_comm src/serial_comm.cpp)
target_link_libraries(serial_comm  ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(jsk_stm_node  src/jsk_stm_node.cpp)

target_link_libraries(jsk_stm_node  ${catkin_LIBRARIES}  ${Boost_LIBRARIES} serial_comm)

install(TARGETS jsk_stm_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
