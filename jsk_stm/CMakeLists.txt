cmake_minimum_required(VERSION 2.8.3)
project(jsk_stm)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs message_generation spinal tf)
find_package(Boost REQUIRED COMPONENTS thread system signals)

catkin_package(
  INCLUDE_DIRS include
  #  LIBRARIES jsk_stm
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs spinal tf
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

add_library (imu_visualization src/imu_visualization.cpp)
target_link_libraries(imu_visualization  ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(imu_visualization_node  src/imu_visualization_node.cpp)
target_link_libraries(imu_visualization_node  ${catkin_LIBRARIES}  ${Boost_LIBRARIES} imu_visualization)

install(TARGETS imu_visualization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
