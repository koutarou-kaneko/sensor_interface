cmake_minimum_required(VERSION 3.0.2)
project(cfs_sensor)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs )
find_package(Boost REQUIRED COMPONENTS  system)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_executable(cfs_sample  src/sample.c src/rs_comm.c)
target_link_libraries(cfs_sample  ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

add_executable(cfs_node  src/node.cpp src/rs_comm.cpp)
target_link_libraries(cfs_node  ${catkin_LIBRARIES}  ${Boost_LIBRARIES})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS cfs_sample cfs_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

