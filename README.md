# XGC2 Gazebo Sim VRPN Bridge

ROS Noetic Gazebo Classic pose-to-VRPN tracker server for XGC2 simulations.

This repository contains the `gazebo_sim_vrpn_bridge` package. It can be placed
directly under a catkin workspace `src/` directory.

The primary backend is `libgazebo_sim_vrpn_system_plugin.so`, loaded directly
by `gzserver`. It reads selected `gazebo::physics::Model` poses inside Gazebo
and therefore does not publish, serialize, copy, or subscribe to the complete
`/gazebo/model_states` array. A dedicated worker owns the VRPN connection,
filtering, noise, and delay state so network work does not run on Gazebo's
simulation update callback.

The original `gazebo_vrpn_server_node` remains available as a compatibility
fallback. Both backends use the same configuration loader and tracker server
core.

## Build

```bash
source /opt/ros/noetic/setup.bash
catkin_make
```

The build produces:

- `devel/lib/libgazebo_sim_vrpn_system_plugin.so`: primary in-process backend
- `devel/lib/gazebo_sim_vrpn_bridge/gazebo_vrpn_server_node`: legacy ROS
  process backend

## In-process launch

Load the configuration before creating the Gazebo world. The plugin reads the
same `/gazebo_vrpn_server` parameter namespace that the legacy node reads:

```bash
source /opt/ros/noetic/setup.bash
source /path/to/catkin_ws/devel/setup.bash
rosparam load \
  "$(rospack find gazebo_sim_vrpn_bridge)/config/vrpn_server.yaml" \
  /gazebo_vrpn_server
rosparam set /gazebo_vrpn_server/auto_track_known_models true
gzserver --verbose \
  -s libgazebo_ros_paths_plugin.so \
  -s libgazebo_ros_api_plugin.so \
  -s libgazebo_sim_vrpn_system_plugin.so \
  /absolute/path/to/world.world
```

`libgazebo_ros_api_plugin.so` must load before the VRPN SystemPlugin. The
in-process backend is hosted by the `/gazebo` process and does not create a
`/gazebo_vrpn_server` ROS node. That name is only its configuration namespace.

The XGC `gazebo-server` process definition performs this parameter load and
plugin ordering automatically. Its readiness requires both Gazebo `/clock` and
the configured VRPN TCP listener; the default ROS basic-services workflow does
not start a separate `gazebo-vrpn-server` process.

## VRPN wire timestamps

The in-process SystemPlugin explicitly encodes `World::SimTime()` in every VRPN
pose, velocity, and acceleration report. A stock `vrpn_client_ros` consumer can
therefore set `use_server_time: true` and receive headers in the same simulation
clock domain as Gazebo `/clock`. While Gazebo is paused, repeated reports keep
the last simulation timestamp; a simulation reset is allowed to move the report
timestamp backwards along with `/clock`.

Capture and publish wall-clock values are still retained internally for
capture-rate limiting, stale-source diagnostics, and bounded measurement-delay
history selection. The delay `timestamp_policy` remains orthogonal: `send_time`
uses the latest simulation time and `sample_time` uses the selected historical
sample's simulation time.

The legacy `/gazebo/model_states` process keeps wall-clock VRPN timestamps for
backward compatibility. `vrpn_client.launch` also keeps
`use_server_time:=false` as its cross-backend default; callers using the
SystemPlugin opt into the corrected simulation stamps with
`use_server_time:=true`.

Hybrid production uses the installed `config/vrpn_server_hybrid.yaml`. It
fixes `delay.timestamp_policy` to `sample_time`, including while delay is
disabled. If a later frozen profile enables measurement delay, the VRPN header
therefore remains the timestamp of the selected pose sample rather than the
time at which the delayed report was sent.

## Legacy fallback

Existing launch files remain valid:

```bash
roslaunch gazebo_sim_vrpn_bridge vrpn_server.launch auto_track_known_models:=true
```

This backend still subscribes to `model_states_topic` (normally
`/gazebo/model_states`). The parameter is ignored by the in-process backend.

Without an explicit mapping or include pattern, automatic tracking recognizes
only the canonical numbered model names `uavN`, `ugvN`, and `mecanumN`.

## Compatibility and quality checks

The in-process regression test protects the public behavior rather than forcing
sample-for-sample equality with the legacy transport. It covers automatic and
manual mapping, body-to-tracker extrinsics, pose/twist/acceleration reports,
monotonic timestamps, a minimum output-rate floor, pause, simulation reset, and
dynamic model creation/deletion/recreation. It also verifies that the backend
has no `/gazebo/model_states` subscriber and no standalone server node. The
legacy protocol, delay, mapping, noise, and identity tests remain enabled.

For an observational A/B run, start one `vrpn_client_ros` instance for each
backend under different namespaces and run:

```bash
rosrun gazebo_sim_vrpn_bridge compare_vrpn_backends.py \
  --legacy-namespace /legacy_vrpn_client \
  --plugin-namespace /plugin_vrpn_client \
  --trackers uav1,ugv1 \
  --duration 15 \
  --output /tmp/vrpn-backend-comparison.json
```

This reports rate, receive-period jitter, timestamp validity, and paired
pose/twist/acceleration differences. It has no acceptance thresholds, is not
registered as a test, and deliberately does not fail when the plugin is faster
or produces better timing quality.

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
ring buffers. `send_time` keeps VRPN report timestamps at the latest time in the
selected wire clock domain; `sample_time` stamps reports with the selected
historical sample's time in that same domain. Delay history selection itself is
always based on wall time.
