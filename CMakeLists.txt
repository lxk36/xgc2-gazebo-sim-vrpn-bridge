cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_vrpn_bridge)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  roscpp
  tf2
  tf2_ros
)

find_package(gazebo REQUIRED)
find_package(VRPN REQUIRED)
find_package(Threads REQUIRED)
find_package(xgc2_math REQUIRED CONFIG)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_sim_vrpn_server_core gazebo_sim_vrpn_system_plugin
  CATKIN_DEPENDS gazebo_msgs geometry_msgs roscpp tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${VRPN_INCLUDE_DIRS}
)

add_library(gazebo_sim_vrpn_server_core SHARED
  src/server_config.cpp
  src/vrpn_tracker_server.cpp
)
target_compile_features(gazebo_sim_vrpn_server_core PUBLIC cxx_std_14)
add_dependencies(gazebo_sim_vrpn_server_core
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gazebo_sim_vrpn_server_core
  PUBLIC
    ${catkin_LIBRARIES}
    ${VRPN_SERVER_LIBRARIES}
    xgc2_math::math
    Threads::Threads
)

add_executable(gazebo_vrpn_server_node
  src/gazebo_vrpn_server_node.cpp
)
target_compile_features(gazebo_vrpn_server_node PRIVATE cxx_std_14)

add_dependencies(gazebo_vrpn_server_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(gazebo_vrpn_server_node
  gazebo_sim_vrpn_server_core
)

add_library(gazebo_sim_vrpn_system_plugin SHARED
  src/gazebo_vrpn_system_plugin.cpp
)
target_compile_features(gazebo_sim_vrpn_system_plugin PRIVATE cxx_std_17)
add_dependencies(gazebo_sim_vrpn_system_plugin
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(gazebo_sim_vrpn_system_plugin
  PRIVATE
    gazebo_sim_vrpn_server_core
    ${GAZEBO_LIBRARIES}
)

install(TARGETS
  gazebo_sim_vrpn_server_core
  gazebo_sim_vrpn_system_plugin
  gazebo_vrpn_server_node
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

catkin_install_python(PROGRAMS tools/compare_vrpn_backends.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(gazebo_sim_vrpn_bridge_mocap_noise_tests test/mocap_noise_tests.cpp)
  if(TARGET gazebo_sim_vrpn_bridge_mocap_noise_tests)
    target_compile_features(gazebo_sim_vrpn_bridge_mocap_noise_tests PRIVATE cxx_std_14)
    target_link_libraries(gazebo_sim_vrpn_bridge_mocap_noise_tests ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(gazebo_sim_vrpn_bridge_measurement_delay_tests test/measurement_delay_tests.cpp)
  if(TARGET gazebo_sim_vrpn_bridge_measurement_delay_tests)
    target_compile_features(gazebo_sim_vrpn_bridge_measurement_delay_tests PRIVATE cxx_std_14)
    target_link_libraries(gazebo_sim_vrpn_bridge_measurement_delay_tests ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(gazebo_sim_vrpn_bridge_model_identity_tests test/model_identity_tests.cpp)
  if(TARGET gazebo_sim_vrpn_bridge_model_identity_tests)
    target_compile_features(gazebo_sim_vrpn_bridge_model_identity_tests PRIVATE cxx_std_14)
    target_link_libraries(gazebo_sim_vrpn_bridge_model_identity_tests gazebo_sim_vrpn_server_core)
  endif()
  catkin_add_nosetests(test/vrpn_server_config_tests.py)
  add_rostest(test/vrpn_auto_mapping_override_e2e.test DEPENDENCIES gazebo_vrpn_server_node)
  add_rostest(test/vrpn_delay_e2e.test DEPENDENCIES gazebo_vrpn_server_node)
  add_rostest(test/vrpn_protocol_e2e.test DEPENDENCIES gazebo_vrpn_server_node)
  add_rostest(test/vrpn_system_plugin_e2e.test DEPENDENCIES gazebo_sim_vrpn_system_plugin)
endif()
