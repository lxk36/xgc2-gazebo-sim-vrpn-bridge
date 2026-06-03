cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_vrpn_bridge)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  tf2
  tf2_ros
)

find_package(VRPN REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${VRPN_INCLUDE_DIRS}
)

add_executable(gazebo_vrpn_server_node
  src/gazebo_vrpn_server_node.cpp
)

add_dependencies(gazebo_vrpn_server_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(gazebo_vrpn_server_node
  ${catkin_LIBRARIES}
  ${VRPN_SERVER_LIBRARIES}
  Threads::Threads
)

install(TARGETS gazebo_vrpn_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
