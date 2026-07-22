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
```

## Delay Simulation

The default server configs do not define any `delay` parameters. Without a
`delay` block or launch overrides, measurement delay stays disabled and no delay
params are loaded onto the ROS parameter server.

Use `config/vrpn_server_delay_simple.yaml` for a fixed delay experiment, or
`config/vrpn_server_delay_complex.yaml` for common plus per-tracker
slow/jitter/burst delay. Both examples keep `max_delay_ms` at 30 ms.

```bash
roslaunch gazebo_sim_vrpn_bridge vrpn_server.launch \
  config:=$(rospack find gazebo_sim_vrpn_bridge)/config/vrpn_server_delay_simple.yaml
```

`vrpn_server.launch` can also load per-tracker measurement transport delay from
any server YAML config:

```yaml
delay:
  enabled: true
  timestamp_policy: send_time  # send_time or sample_time
  max_delay_ms: 30.0
  history_margin_ms: 20.0
  common:
    base_ms: 15.0
```

For time-varying transport experiments:

```yaml
delay:
  enabled: true
  timestamp_policy: sample_time
  seed: 42
  max_delay_ms: 30.0
  history_margin_ms: 20.0
  common:
    base_ms: 5.0
    slow_stddev_ms: 5.0
    slow_tau_s: 5.0
    jitter_stddev_ms: 2.0
    burst_probability: 0.001
    burst_extra_ms: [8.0, 20.0]
  trackers:
    uav1:
      base_ms: 8.0
      jitter_stddev_ms: 5.0
      max_delay_ms: 30.0
```

The server publishes delayed historical tracker samples from bounded per-tracker
ring buffers. `send_time` keeps VRPN report timestamps at the publish time;
`sample_time` stamps reports with the delayed sample receive time.
