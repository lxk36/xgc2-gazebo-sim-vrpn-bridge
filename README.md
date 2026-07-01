# XGC2 Gazebo Sim VRPN Bridge

ROS Noetic Gazebo Classic model-state to VRPN tracker bridge for XGC2
simulations.

This repository contains the `gazebo_sim_vrpn_bridge` package. It can be placed
directly under a catkin workspace `src/` directory.

## Build

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

## Launch

```bash
roslaunch gazebo_sim_vrpn_bridge vrpn_server.launch auto_track_known_models:=true
roslaunch gazebo_sim_vrpn_bridge vrpn_client.launch trackers:=[uav1]
```
