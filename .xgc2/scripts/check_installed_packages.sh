#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-gazebo-sim-vrpn-bridge >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
test "$(rospack find gazebo_sim_vrpn_bridge)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_vrpn_bridge/gazebo_vrpn_server_node"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_vrpn_bridge/compare_vrpn_backends.py"
test -f "/opt/ros/${ROS_DISTRO}/lib/libgazebo_sim_vrpn_server_core.so"
test -f "/opt/ros/${ROS_DISTRO}/lib/libgazebo_sim_vrpn_system_plugin.so"
test -f "/opt/ros/${ROS_DISTRO}/include/gazebo_sim_vrpn_bridge/mocap_noise.h"
test -f "/opt/ros/${ROS_DISTRO}/include/gazebo_sim_vrpn_bridge/measurement_delay.h"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge/config/vrpn_server_delay_simple.yaml"
test -f "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge/config/vrpn_server_delay_complex.yaml"

roslaunch --files gazebo_sim_vrpn_bridge vrpn_server.launch auto_track_known_models:=true port:=3883 publish_rate:=120.0 \
  >/tmp/xgc2-vrpn-server-files.txt

echo "Installed package check passed"
