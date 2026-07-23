#pragma once

#include <map>
#include <string>
#include <vector>

#include <ros/node_handle.h>
#include <tf2/LinearMath/Transform.h>

#include "gazebo_sim_vrpn_bridge/measurement_delay.h"
#include "gazebo_sim_vrpn_bridge/mocap_noise.h"

namespace gazebo_sim_vrpn_bridge {

struct RobotConfig {
    bool enabled{true};
    std::string gazebo_model_name;
    tf2::Transform body_to_tracker{tf2::Transform::getIdentity()};
};

// Configuration shared by the legacy ROS adapter and the in-process Gazebo
// adapter. Keeping model selection and measurement settings here prevents the
// two server backends from drifting as new options are added.
struct ServerConfig {
    std::string model_states_topic{"/gazebo/model_states"};
    std::string bind_address;
    int port{3883};
    double publish_rate_hz{100.0};
    double stale_timeout_s{1.0};
    double scan_interval_s{2.0};
    double velocity_filter_cutoff_hz{25.0};
    double acceleration_filter_cutoff_hz{25.0};
    double derivative_reset_timeout_s{0.5};
    std::string match_mode{"contains"};
    bool auto_track_known_models{false};
    std::vector<std::string> auto_include_patterns;
    std::vector<std::string> tracker_patterns;
    std::map<std::string, RobotConfig> robot_configs;
    std::map<std::string, std::string> configured_model_to_tracker;
    tf2::Transform default_body_to_tracker{tf2::Transform::getIdentity()};
    MocapNoiseConfig mocap_noise;
    MeasurementDelayConfig delay;

    std::string trackerNameForGazeboModel(const std::string& gazebo_model_name) const;
    tf2::Transform bodyToTrackerFor(const std::string& tracker_name) const;
    void validate() const;
};

ServerConfig loadServerConfig(const ros::NodeHandle& private_node);

} // namespace gazebo_sim_vrpn_bridge
