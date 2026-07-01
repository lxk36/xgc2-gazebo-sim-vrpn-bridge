#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

dpkg -s ros-noetic-xgc2-gazebo-sim-vrpn-bridge >/dev/null
dpkg -s libxgc2-math-dev >/dev/null
test "$(rospack find gazebo_sim_vrpn_bridge)" = "/opt/ros/${ROS_DISTRO}/share/gazebo_sim_vrpn_bridge"
test -x "/opt/ros/${ROS_DISTRO}/lib/gazebo_sim_vrpn_bridge/gazebo_vrpn_server_node"
test -f "/opt/ros/${ROS_DISTRO}/include/gazebo_sim_vrpn_bridge/mocap_noise.h"

roslaunch --files gazebo_sim_vrpn_bridge vrpn_server.launch auto_track_known_models:=true port:=3883 publish_rate:=120.0 \
  >/tmp/xgc2-vrpn-server-files.txt
roslaunch --files gazebo_sim_vrpn_bridge vrpn_client.launch trackers:=[uav1] \
  >/tmp/xgc2-vrpn-client-files.txt
roslaunch --dump-params gazebo_sim_vrpn_bridge vrpn_client.launch trackers:=[uav1] \
  | grep -F "/vrpn_client_node/broadcast_tf: false" >/dev/null

echo "Installed package check passed"
