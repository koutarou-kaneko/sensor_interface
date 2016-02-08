cmake_minimum_required(VERSION 2.8.3)
project(cfs_sensor)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs )
find_package(Boost REQUIRED COMPONENTS  system)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
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


add_executable(cfs_sample  src/sample.c src/rs_comm.c)
add_dependencies(cfs_sample ${PROJECT_NAME}_gencpp)
target_link_libraries(cfs_sample  ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(cfs_node  src/node.cpp src/rs_comm.cpp)
add_dependencies(cfs_node ${PROJECT_NAME}_gencpp)
target_link_libraries(cfs_node  ${catkin_LIBRARIES}  ${Boost_LIBRARIES})



install(TARGETS cfs_sample cfs_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

