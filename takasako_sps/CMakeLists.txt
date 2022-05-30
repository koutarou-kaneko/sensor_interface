cmake_minimum_required(VERSION 3.0.2)
project(takasako_sps)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
find_package(Boost REQUIRED COMPONENTS system)

add_message_files(
   FILES
   PowerInfo.msg
 )

generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES scpi_tcp_client
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_library(scpi_tcp_client src/scpi_tcp_client.cpp)
add_dependencies(scpi_tcp_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} takasako_sps_gencpp)
target_link_libraries(scpi_tcp_client ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(scpi_tcp_client_node src/scpi_tcp_client_node.cpp)
add_dependencies(scpi_tcp_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} takasako_sps_gencpp)
target_link_libraries(scpi_tcp_client_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} scpi_tcp_client)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})


install(TARGETS scpi_tcp_client
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS scpi_tcp_client_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
