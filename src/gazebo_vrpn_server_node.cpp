#include <sys/time.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <array>
#include <atomic>
#include <csignal>
#include <map>
#include <memory>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include "gazebo_sim_vrpn_bridge/butterworth_filter.h"
#include "gazebo_sim_vrpn_bridge/mocap_noise.h"

namespace gazebo_sim_vrpn_bridge {

namespace {
std::atomic_bool g_shutdown_requested{false};

void requestShutdownFromSignal(int) {
    g_shutdown_requested.store(true, std::memory_order_relaxed);
}

void installSignalHandlers() {
    struct sigaction action {};
    action.sa_handler = requestShutdownFromSignal;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    sigaction(SIGTERM, &action, nullptr);
}
}  // namespace

class GazeboVrpnServerNode {
public:
    GazeboVrpnServerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
        : nh_(nh), nh_private_(nh_private) {
        nh_private_.param<std::string>("model_states_topic", model_states_topic_, "/gazebo/model_states");
        nh_private_.param<std::string>("bind_address", bind_address_, "");
        nh_private_.param<int>("port", port_, 3883);
        nh_private_.param<double>("publish_rate", publish_rate_hz_, 100.0);
        nh_private_.param<double>("stale_timeout", stale_timeout_s_, 1.0);
        nh_private_.param<double>("scan_interval", scan_interval_s_, 2.0);
        nh_private_.param<double>("velocity_filter_cutoff", velocity_filter_cutoff_hz_, 25.0);
        nh_private_.param<double>("acceleration_filter_cutoff", acceleration_filter_cutoff_hz_, 25.0);
        nh_private_.param<double>("derivative_reset_timeout", derivative_reset_timeout_s_, 0.5);
        nh_private_.param<bool>("mocap_noise_enabled", mocap_noise_config_.enabled, mocap_noise_config_.enabled);
        nh_private_.param<int>("mocap_noise_seed", mocap_noise_seed_param_, static_cast<int>(mocap_noise_config_.seed));
        nh_private_.param<std::string>("match_mode", match_mode_, "contains");
        nh_private_.param<bool>("auto_track_known_models", auto_track_known_models_, false);
        loadConfig();
        validateConfig();
        mocap_noise_config_.seed = mocap_noise_seed_param_ <= 0 ? 1 : static_cast<unsigned int>(mocap_noise_seed_param_);
        mocap_noise_ = MocapNoise(mocap_noise_config_);

        connection_ = vrpn_create_server_connection(
            port_, nullptr, nullptr, bind_address_.empty() ? nullptr : bind_address_.c_str());
        if (connection_ == nullptr) {
            throw std::runtime_error("failed to create VRPN server connection");
        }

        model_states_sub_ = nh_.subscribe(model_states_topic_, 1,
                                          &GazeboVrpnServerNode::modelStatesCallback, this);

        ROS_INFO_STREAM("[GazeboVrpnServerNode] Serving Gazebo models as VRPN trackers on port "
                        << port_ << (bind_address_.empty() ? "" : " bound to " + bind_address_));
        ROS_INFO_STREAM("[GazeboVrpnServerNode] Scan interval is " << scan_interval_s_
                        << " s; match_mode=" << match_mode_);
        if (!tracker_patterns_.empty()) {
            ROS_INFO_STREAM("[GazeboVrpnServerNode] Tracker pattern count: " << tracker_patterns_.size());
        }
        if (!auto_include_patterns_.empty()) {
            ROS_INFO_STREAM("[GazeboVrpnServerNode] Auto mapping pattern count: "
                            << auto_include_patterns_.size());
        }
        if (!auto_track_known_models_) {
            ROS_INFO_STREAM("[GazeboVrpnServerNode] Auto export is disabled; configure trackers or robots");
        }
        if (mocap_noise_config_.enabled) {
            ROS_INFO_STREAM("[GazeboVrpnServerNode] Mocap measurement noise enabled: position stddev xyz=["
                            << mocap_noise_config_.position_stddev_m[0] << ", "
                            << mocap_noise_config_.position_stddev_m[1] << ", "
                            << mocap_noise_config_.position_stddev_m[2] << "] m, rotation stddev rpy=["
                            << mocap_noise_config_.rotation_stddev_rad[0] << ", "
                            << mocap_noise_config_.rotation_stddev_rad[1] << ", "
                            << mocap_noise_config_.rotation_stddev_rad[2] << "] rad, seed="
                            << mocap_noise_config_.seed);
        }
    }

    ~GazeboVrpnServerNode() {
        tracked_models_.clear();
        if (connection_ != nullptr) {
            connection_->removeReference();
            connection_ = nullptr;
        }
    }

    void spin() {
        ros::WallRate rate(publish_rate_hz_);
        while (ros::ok() && !g_shutdown_requested.load(std::memory_order_relaxed)) {
            ros::spinOnce();
            publishLatestPoses();
            for (auto& entry : tracked_models_) {
                entry.second.tracker->mainloop();
            }
            connection_->mainloop();
            rate.sleep();
        }
    }

private:
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        if (!msg || msg->name.size() != msg->pose.size()) {
            ROS_WARN_THROTTLE(2.0, "[GazeboVrpnServerNode] Invalid /gazebo/model_states message");
            return;
        }

        const ros::WallTime now = ros::WallTime::now();
        if (shouldScan(now)) {
            scanModelList(*msg, now);
        }
        updateTrackedModelPoses(*msg, now);
    }

    bool shouldScan(const ros::WallTime& now) const {
        return last_scan_wall_time_.toSec() == 0.0 ||
               (now - last_scan_wall_time_).toSec() >= scan_interval_s_;
    }

    void scanModelList(const gazebo_msgs::ModelStates& msg, const ros::WallTime& now) {
        bool matched_any_model = false;
        for (size_t i = 0; i < msg.name.size(); ++i) {
            const std::string& gazebo_model_name = msg.name[i];
            const std::string tracker_name = trackerNameForGazeboModel(gazebo_model_name);
            if (tracker_name.empty()) {
                continue;
            }

            matched_any_model = true;
            ensureTrackedModel(gazebo_model_name, tracker_name, i);
        }
        last_scan_wall_time_ = now;

        if (!matched_any_model) {
            ROS_WARN_THROTTLE(2.0,
                              "[GazeboVrpnServerNode] No configured tracker matched models in %s",
                              model_states_topic_.c_str());
        }
    }

    struct TrackedModel {
        std::string gazebo_model_name;
        std::string tracker_name;
        geometry_msgs::Pose latest_pose;
        tf2::Transform last_tracker_transform;
        tf2::Vector3 linear_velocity{0.0, 0.0, 0.0};
        tf2::Vector3 angular_velocity{0.0, 0.0, 0.0};
        tf2::Vector3 linear_acceleration{0.0, 0.0, 0.0};
        tf2::Vector3 angular_acceleration{0.0, 0.0, 0.0};
        tf2::Vector3 previous_raw_linear_velocity{0.0, 0.0, 0.0};
        tf2::Vector3 previous_raw_angular_velocity{0.0, 0.0, 0.0};
        std::array<gazebo_sim_vrpn_bridge::SecondOrderButterworthLowPass, 3> linear_velocity_filters;
        std::array<gazebo_sim_vrpn_bridge::SecondOrderButterworthLowPass, 3> angular_velocity_filters;
        std::array<gazebo_sim_vrpn_bridge::SecondOrderButterworthLowPass, 3> linear_acceleration_filters;
        std::array<gazebo_sim_vrpn_bridge::SecondOrderButterworthLowPass, 3> angular_acceleration_filters;
        tf2::Transform body_to_tracker;
        ros::WallTime last_model_state_wall_time;
        double last_model_state_time_s{0.0};
        double last_derivative_dt_s{0.0};
        size_t model_index{0};
        bool have_pose{false};
        bool have_derivative_state{false};
        bool have_raw_velocity{false};
        bool have_velocity{false};
        bool have_acceleration{false};
        std::unique_ptr<vrpn_Tracker_Server> tracker;
    };

    TrackedModel* ensureTrackedModel(const std::string& gazebo_model_name,
                                     const std::string& tracker_name,
                                     size_t model_index) {
        auto existing = tracked_models_.find(gazebo_model_name);
        if (existing != tracked_models_.end()) {
            existing->second.model_index = model_index;
            return &existing->second;
        }

        if (tracker_names_.count(tracker_name) > 0) {
            ROS_WARN_STREAM_THROTTLE(10.0,
                                     "[GazeboVrpnServerNode] Ignoring Gazebo model '" << gazebo_model_name
                                     << "' because tracker name '" << tracker_name
                                     << "' is already in use");
            return nullptr;
        }

        TrackedModel model;
        model.gazebo_model_name = gazebo_model_name;
        model.tracker_name = tracker_name;
        model.body_to_tracker = bodyToTrackerFor(tracker_name);
        model.model_index = model_index;
        model.tracker.reset(new vrpn_Tracker_Server(tracker_name.c_str(), connection_, 1));

        ROS_INFO_STREAM("[GazeboVrpnServerNode] Registered Gazebo model '" << gazebo_model_name
                        << "' as VRPN tracker '" << tracker_name << "'");
        tracker_names_[tracker_name] = gazebo_model_name;
        auto result = tracked_models_.emplace(gazebo_model_name, std::move(model));
        return &result.first->second;
    }

    void updateTrackedModelPoses(const gazebo_msgs::ModelStates& msg, const ros::WallTime& now) {
        for (auto& entry : tracked_models_) {
            TrackedModel& model = entry.second;
            if (model.model_index >= msg.pose.size() ||
                model.model_index >= msg.name.size() ||
                msg.name[model.model_index] != model.gazebo_model_name) {
                model.have_pose = false;
                continue;
            }

            const tf2::Transform world_body = poseToTransform(msg.pose[model.model_index]);
            const tf2::Transform world_tracker = world_body * model.body_to_tracker;
            updateDerivativeState(model, world_tracker, now);
            model.latest_pose = transformToPose(world_tracker);
            model.last_model_state_wall_time = now;
            model.have_pose = true;
        }
    }

    void publishLatestPoses() {
        struct timeval timestamp {};
        gettimeofday(&timestamp, nullptr);

        for (auto& entry : tracked_models_) {
            TrackedModel& model = entry.second;
            if (!model.have_pose) {
                continue;
            }

            const double age_s = (ros::WallTime::now() - model.last_model_state_wall_time).toSec();
            if (age_s > stale_timeout_s_) {
                ROS_WARN_THROTTLE(2.0,
                                  "[GazeboVrpnServerNode] Last Gazebo pose for '%s' is stale: %.3f s",
                                  model.gazebo_model_name.c_str(), age_s);
            }

            const geometry_msgs::Pose measured_pose = mocap_noise_.apply(model.latest_pose);
            const vrpn_float64 position[3] = {
                measured_pose.position.x,
                measured_pose.position.y,
                measured_pose.position.z,
            };
            const vrpn_float64 quaternion[4] = {
                measured_pose.orientation.x,
                measured_pose.orientation.y,
                measured_pose.orientation.z,
                measured_pose.orientation.w,
            };

            const int status = model.tracker->report_pose(0, timestamp, position, quaternion);
            if (status != 0) {
                ROS_WARN_THROTTLE(2.0,
                                  "[GazeboVrpnServerNode] Failed to publish VRPN pose for '%s'",
                                  model.tracker_name.c_str());
            }

            if (model.have_velocity) {
                const vrpn_float64 linear_velocity[3] = {
                    model.linear_velocity.x(),
                    model.linear_velocity.y(),
                    model.linear_velocity.z(),
                };
                vrpn_float64 angular_velocity[4] {};
                vectorRpyToQuaternion(model.angular_velocity, angular_velocity);
                const int velocity_status =
                    model.tracker->report_pose_velocity(0, timestamp, linear_velocity,
                                                        angular_velocity, reportInterval(model));
                if (velocity_status != 0) {
                    ROS_WARN_THROTTLE(2.0,
                                      "[GazeboVrpnServerNode] Failed to publish VRPN twist for '%s'",
                                      model.tracker_name.c_str());
                }
            }

            if (model.have_acceleration) {
                const vrpn_float64 linear_acceleration[3] = {
                    model.linear_acceleration.x(),
                    model.linear_acceleration.y(),
                    model.linear_acceleration.z(),
                };
                vrpn_float64 angular_acceleration[4] {};
                vectorRpyToQuaternion(model.angular_acceleration, angular_acceleration);
                const int acceleration_status =
                    model.tracker->report_pose_acceleration(0, timestamp, linear_acceleration,
                                                            angular_acceleration,
                                                            reportInterval(model));
                if (acceleration_status != 0) {
                    ROS_WARN_THROTTLE(2.0,
                                      "[GazeboVrpnServerNode] Failed to publish VRPN accel for '%s'",
                                      model.tracker_name.c_str());
                }
            }
        }
    }

    void updateDerivativeState(TrackedModel& model,
                               const tf2::Transform& world_tracker,
                               const ros::WallTime& fallback_time) {
        const double sample_time_s = sampleTimeSeconds(fallback_time);
        if (!model.have_derivative_state) {
            initializeDerivativeState(model, world_tracker, sample_time_s);
            return;
        }

        const double dt_s = sample_time_s - model.last_model_state_time_s;
        if (dt_s <= 1.0e-6 || dt_s > derivative_reset_timeout_s_) {
            initializeDerivativeState(model, world_tracker, sample_time_s);
            return;
        }

        const tf2::Vector3 raw_linear_velocity =
            (world_tracker.getOrigin() - model.last_tracker_transform.getOrigin()) / dt_s;
        const tf2::Vector3 raw_angular_velocity =
            angularVelocityBetween(model.last_tracker_transform.getRotation(),
                                   world_tracker.getRotation(), dt_s);

        model.linear_velocity = filterVector(model.linear_velocity_filters, raw_linear_velocity, dt_s);
        model.angular_velocity = filterVector(model.angular_velocity_filters, raw_angular_velocity, dt_s);

        if (model.have_raw_velocity) {
            const tf2::Vector3 raw_linear_acceleration =
                (raw_linear_velocity - model.previous_raw_linear_velocity) / dt_s;
            const tf2::Vector3 raw_angular_acceleration =
                (raw_angular_velocity - model.previous_raw_angular_velocity) / dt_s;
            model.linear_acceleration =
                filterVector(model.linear_acceleration_filters, raw_linear_acceleration, dt_s);
            model.angular_acceleration =
                filterVector(model.angular_acceleration_filters, raw_angular_acceleration, dt_s);
            model.have_acceleration = true;
        }

        model.previous_raw_linear_velocity = raw_linear_velocity;
        model.previous_raw_angular_velocity = raw_angular_velocity;
        model.last_tracker_transform = world_tracker;
        model.last_model_state_time_s = sample_time_s;
        model.last_derivative_dt_s = dt_s;
        model.have_raw_velocity = true;
        model.have_velocity = true;
    }

    void initializeDerivativeState(TrackedModel& model,
                                   const tf2::Transform& world_tracker,
                                   double sample_time_s) const {
        model.last_tracker_transform = world_tracker;
        model.last_model_state_time_s = sample_time_s;
        model.last_derivative_dt_s = 0.0;
        model.linear_velocity.setZero();
        model.angular_velocity.setZero();
        model.linear_acceleration.setZero();
        model.angular_acceleration.setZero();
        model.previous_raw_linear_velocity.setZero();
        model.previous_raw_angular_velocity.setZero();
        resetFilters(model.linear_velocity_filters, velocity_filter_cutoff_hz_, 0.0);
        resetFilters(model.angular_velocity_filters, velocity_filter_cutoff_hz_, 0.0);
        resetFilters(model.linear_acceleration_filters, acceleration_filter_cutoff_hz_, 0.0);
        resetFilters(model.angular_acceleration_filters, acceleration_filter_cutoff_hz_, 0.0);
        model.have_derivative_state = true;
        model.have_raw_velocity = false;
        model.have_velocity = false;
        model.have_acceleration = false;
    }

    static double sampleTimeSeconds(const ros::WallTime& fallback_time) {
        const ros::Time ros_time = ros::Time::now();
        if (!ros_time.isZero()) {
            return ros_time.toSec();
        }
        return fallback_time.toSec();
    }

    static void resetFilters(std::array<gazebo_sim_vrpn_bridge::SecondOrderButterworthLowPass, 3>& filters,
                             double cutoff_hz,
                             double value) {
        for (auto& filter : filters) {
            filter.reset(cutoff_hz, value);
        }
    }

    static tf2::Vector3 filterVector(
        std::array<gazebo_sim_vrpn_bridge::SecondOrderButterworthLowPass, 3>& filters,
        const tf2::Vector3& value,
        double dt_s) {
        return tf2::Vector3(filters[0].filter(value.x(), dt_s),
                            filters[1].filter(value.y(), dt_s),
                            filters[2].filter(value.z(), dt_s));
    }

    static tf2::Vector3 angularVelocityBetween(const tf2::Quaternion& previous,
                                               const tf2::Quaternion& current,
                                               double dt_s) {
        tf2::Quaternion delta = current * previous.inverse();
        delta.normalize();
        if (delta.w() < 0.0) {
            delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());
        }

        const double angle = delta.getAngle();
        if (std::abs(angle) < 1.0e-9) {
            return tf2::Vector3(0.0, 0.0, 0.0);
        }

        const tf2::Vector3 axis = delta.getAxis();
        return axis * (angle / dt_s);
    }

    static void vectorRpyToQuaternion(const tf2::Vector3& rpy,
                                      vrpn_float64 quaternion[4]) {
        tf2::Quaternion q;
        q.setRPY(rpy.x(), rpy.y(), rpy.z());
        q.normalize();
        quaternion[0] = q.x();
        quaternion[1] = q.y();
        quaternion[2] = q.z();
        quaternion[3] = q.w();
    }

    double reportInterval(const TrackedModel& model) const {
        if (model.last_derivative_dt_s > 0.0) {
            return model.last_derivative_dt_s;
        }
        return publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.0;
    }

    static bool startsWith(const std::string& value, const std::string& prefix) {
        return value.size() >= prefix.size() &&
               value.compare(0, prefix.size(), prefix) == 0;
    }

    static std::string trimSlashes(std::string value) {
        while (!value.empty() && value.front() == '/') {
            value.erase(value.begin());
        }
        while (!value.empty() && value.back() == '/') {
            value.pop_back();
        }
        return value;
    }

    static bool isNumberedNamespace(const std::string& value,
                                    const std::string& prefix) {
        if (!startsWith(value, prefix) || value.size() == prefix.size()) {
            return false;
        }
        return std::all_of(value.begin() + static_cast<long>(prefix.size()), value.end(),
                           [](unsigned char ch) { return std::isdigit(ch); });
    }

    static bool isSupportedNamespace(const std::string& value) {
        return isNumberedNamespace(value, "ugv") ||
               isNumberedNamespace(value, "uav") ||
               isNumberedNamespace(value, "tello");
    }

    bool matchesPattern(const std::string& model_name, const std::string& pattern) const {
        if (pattern.empty()) {
            return false;
        }
        if (match_mode_ == "exact") {
            return model_name == pattern;
        }
        if (match_mode_ == "prefix") {
            return startsWith(model_name, pattern);
        }
        return model_name.find(pattern) != std::string::npos;
    }

    bool matchesAutoPattern(const std::string& model_name) const {
        for (const std::string& pattern : auto_include_patterns_) {
            try {
                if (std::regex_match(model_name, std::regex(pattern))) {
                    return true;
                }
            } catch (const std::regex_error& e) {
                throw std::runtime_error("invalid auto_mapping.include_patterns regex '" +
                                         pattern + "': " + e.what());
            }
        }
        return false;
    }

    std::string trackerNameForGazeboModel(const std::string& gazebo_model_name) const {
        const std::string name = trimSlashes(gazebo_model_name);
        if (name.empty()) {
            return {};
        }

        const auto configured = configured_model_to_tracker_.find(name);
        if (configured != configured_model_to_tracker_.end()) {
            return configured->second;
        }

        for (const std::string& pattern : tracker_patterns_) {
            if (matchesPattern(name, pattern)) {
                return pattern;
            }
        }

        if (!auto_track_known_models_) {
            return {};
        }

        if (matchesAutoPattern(name)) {
            return name;
        }

        if (auto_include_patterns_.empty() && isSupportedNamespace(name)) {
            return name;
        }

        const std::string scout_prefix = "scout";
        if (auto_include_patterns_.empty() && startsWith(name, scout_prefix)) {
            std::string suffix = trimSlashes(name.substr(scout_prefix.size()));
            if (isSupportedNamespace(suffix)) {
                return suffix;
            }
        }

        const std::string scout_mini_prefix = "scout_mini_";
        if (auto_include_patterns_.empty() && startsWith(name, scout_mini_prefix)) {
            const std::string suffix = trimSlashes(name.substr(scout_mini_prefix.size()));
            if (isSupportedNamespace(suffix)) {
                return suffix;
            }
        }

        // Legacy single-UGV Gazebo names in this workspace did not encode the namespace.
        if (auto_include_patterns_.empty() &&
            (name == "scout_mini_ros_control" || name == "scout_description")) {
            return "ugv1";
        }

        return {};
    }

    tf2::Transform bodyToTrackerFor(const std::string& tracker_name) const {
        const auto config = robot_configs_.find(tracker_name);
        if (config != robot_configs_.end()) {
            return config->second.body_to_tracker;
        }
        return default_body_to_tracker_;
    }

    struct RobotConfig {
        bool enabled{true};
        std::string gazebo_model_name;
        tf2::Transform body_to_tracker{tf2::Transform::getIdentity()};
    };

    void loadConfig() {
        default_body_to_tracker_.setIdentity();

        XmlRpc::XmlRpcValue trackers;
        if (nh_private_.getParam("trackers", trackers)) {
            tracker_patterns_ = parseStringList(trackers, "trackers");
        }

        XmlRpc::XmlRpcValue enabled_trackers;
        if (tracker_patterns_.empty() && nh_private_.getParam("enabled_trackers", enabled_trackers)) {
            tracker_patterns_ = parseStringList(enabled_trackers, "enabled_trackers");
        }

        XmlRpc::XmlRpcValue default_transform;
        if (nh_private_.getParam("default_body_to_tracker", default_transform)) {
            default_body_to_tracker_ = parseTransform(default_transform, "default_body_to_tracker");
        }

        XmlRpc::XmlRpcValue noise;
        if (nh_private_.getParam("mocap_noise", noise)) {
            parseMocapNoise(noise);
        }

        XmlRpc::XmlRpcValue auto_mapping;
        if (nh_private_.getParam("auto_mapping", auto_mapping)) {
            parseAutoMapping(auto_mapping);
        }

        XmlRpc::XmlRpcValue robots;
        if (nh_private_.getParam("robots", robots)) {
            parseRobots(robots);
        }

        XmlRpc::XmlRpcValue manual_mapping;
        if (nh_private_.getParam("manual_mapping", manual_mapping)) {
            parseManualMapping(manual_mapping);
        }

        XmlRpc::XmlRpcValue extrinsics;
        if (nh_private_.getParam("extrinsics", extrinsics)) {
            parseExtrinsics(extrinsics);
        }
    }

    void parseMocapNoise(XmlRpc::XmlRpcValue& noise) {
        if (noise.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("mocap_noise must be a YAML mapping");
        }
        if (noise.hasMember("enabled")) {
            mocap_noise_config_.enabled = static_cast<bool>(noise["enabled"]);
        }
        if (noise.hasMember("position_stddev_xyz")) {
            mocap_noise_config_.position_stddev_m =
                toArray3(parseDoubleVector(noise["position_stddev_xyz"],
                                           "mocap_noise.position_stddev_xyz", 3));
        }
        if (noise.hasMember("rotation_stddev_rpy")) {
            mocap_noise_config_.rotation_stddev_rad =
                toArray3(parseDoubleVector(noise["rotation_stddev_rpy"],
                                           "mocap_noise.rotation_stddev_rpy", 3));
        }
        if (noise.hasMember("seed")) {
            mocap_noise_seed_param_ = static_cast<int>(xmlRpcToDouble(noise["seed"], "mocap_noise.seed"));
        }
    }

    void parseAutoMapping(XmlRpc::XmlRpcValue& auto_mapping) {
        if (auto_mapping.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("auto_mapping must be a YAML mapping");
        }
        if (auto_mapping.hasMember("enabled")) {
            auto_track_known_models_ = static_cast<bool>(auto_mapping["enabled"]);
        }
        if (auto_mapping.hasMember("include_patterns")) {
            auto_include_patterns_ = parseStringList(auto_mapping["include_patterns"],
                                                     "auto_mapping.include_patterns");
        }
    }

    void parseRobots(XmlRpc::XmlRpcValue& robots) {
        if (robots.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("robots must be a YAML mapping");
        }

        for (auto it = robots.begin(); it != robots.end(); ++it) {
            const std::string tracker_name = it->first;
            XmlRpc::XmlRpcValue& value = it->second;
            if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                throw std::runtime_error("robots." + tracker_name + " must be a YAML mapping");
            }

            RobotConfig config;
            config.body_to_tracker = default_body_to_tracker_;
            if (value.hasMember("enabled")) {
                config.enabled = static_cast<bool>(value["enabled"]);
            }
            if (!config.enabled) {
                robot_configs_[tracker_name] = config;
                continue;
            }
            if (value.hasMember("gazebo_model_name")) {
                config.gazebo_model_name = static_cast<std::string>(value["gazebo_model_name"]);
                configured_model_to_tracker_[trimSlashes(config.gazebo_model_name)] = tracker_name;
            } else if (value.hasMember("gazebo_model")) {
                config.gazebo_model_name = static_cast<std::string>(value["gazebo_model"]);
                configured_model_to_tracker_[trimSlashes(config.gazebo_model_name)] = tracker_name;
            }
            if (value.hasMember("body_to_tracker")) {
                config.body_to_tracker = parseTransform(value["body_to_tracker"],
                                                        "robots." + tracker_name + ".body_to_tracker");
            }
            robot_configs_[tracker_name] = config;
        }
    }

    void parseManualMapping(XmlRpc::XmlRpcValue& manual_mapping) {
        if (manual_mapping.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("manual_mapping must be a YAML mapping");
        }
        for (auto it = manual_mapping.begin(); it != manual_mapping.end(); ++it) {
            XmlRpc::XmlRpcValue& value = it->second;
            if (value.getType() != XmlRpc::XmlRpcValue::TypeString) {
                throw std::runtime_error("manual_mapping." + it->first + " must be a tracker name string");
            }
            const std::string gazebo_model_name = trimSlashes(it->first);
            const std::string tracker_name = trimSlashes(static_cast<std::string>(value));
            if (!gazebo_model_name.empty() && !tracker_name.empty()) {
                configured_model_to_tracker_[gazebo_model_name] = tracker_name;
            }
        }
    }

    void parseExtrinsics(XmlRpc::XmlRpcValue& extrinsics) {
        if (extrinsics.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error("extrinsics must be a YAML mapping");
        }
        for (auto it = extrinsics.begin(); it != extrinsics.end(); ++it) {
            const std::string tracker_name = trimSlashes(it->first);
            if (tracker_name.empty()) {
                continue;
            }
            RobotConfig& config = robot_configs_[tracker_name];
            config.body_to_tracker = parseTransform(it->second, "extrinsics." + tracker_name);
        }
    }

    static std::vector<std::string> parseStringList(XmlRpc::XmlRpcValue& value,
                                                    const std::string& param_name) {
        std::vector<std::string> result;
        std::set<std::string> seen;
        if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
            return splitTrackerString(static_cast<std::string>(value));
        }
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            throw std::runtime_error(param_name + " must be a YAML list");
        }
        for (int i = 0; i < value.size(); ++i) {
            if (value[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
                throw std::runtime_error(param_name + " entries must be strings");
            }
            const std::string item = trimSlashes(static_cast<std::string>(value[i]));
            if (!item.empty() && seen.insert(item).second) {
                result.push_back(item);
            }
        }
        return result;
    }

    static std::vector<std::string> splitTrackerString(const std::string& value) {
        std::vector<std::string> result;
        std::set<std::string> seen;
        std::string normalized;
        normalized.reserve(value.size());
        for (char ch : value) {
            normalized.push_back((ch == ',' || ch == ';') ? ' ' : ch);
        }

        std::istringstream stream(normalized);
        std::string item;
        while (stream >> item) {
            item = trimSlashes(item);
            if (!item.empty() && seen.insert(item).second) {
                result.push_back(item);
            }
        }
        return result;
    }

    void validateConfig() const {
        if (match_mode_ != "contains" && match_mode_ != "exact" && match_mode_ != "prefix") {
            throw std::runtime_error("match_mode must be one of: contains, exact, prefix");
        }
        if (publish_rate_hz_ <= 0.0) {
            throw std::runtime_error("publish_rate must be positive");
        }
        if (scan_interval_s_ <= 0.0) {
            throw std::runtime_error("scan_interval must be positive");
        }
        if (port_ <= 0 || port_ > 65535) {
            throw std::runtime_error("port must be in range 1..65535");
        }
    }

    static tf2::Transform parseTransform(XmlRpc::XmlRpcValue& value,
                                         const std::string& param_name) {
        if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            throw std::runtime_error(param_name + " must be a YAML mapping");
        }

        std::vector<double> xyz{0.0, 0.0, 0.0};
        if (value.hasMember("xyz")) {
            xyz = parseDoubleVector(value["xyz"], param_name + ".xyz", 3);
        } else if (value.hasMember("translation")) {
            xyz = parseDoubleVector(value["translation"], param_name + ".translation", 3);
        }

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, 0.0);
        if (value.hasMember("rpy")) {
            const std::vector<double> rpy = parseDoubleVector(value["rpy"], param_name + ".rpy", 3);
            q.setRPY(rpy[0], rpy[1], rpy[2]);
        } else if (value.hasMember("rotation_rpy")) {
            const std::vector<double> rpy = parseDoubleVector(value["rotation_rpy"],
                                                              param_name + ".rotation_rpy", 3);
            q.setRPY(rpy[0], rpy[1], rpy[2]);
        }
        if (value.hasMember("quaternion")) {
            const std::vector<double> quat = parseDoubleVector(value["quaternion"],
                                                               param_name + ".quaternion", 4);
            q = tf2::Quaternion(quat[0], quat[1], quat[2], quat[3]);
        }
        q.normalize();

        return tf2::Transform(q, tf2::Vector3(xyz[0], xyz[1], xyz[2]));
    }

    static std::vector<double> parseDoubleVector(XmlRpc::XmlRpcValue& value,
                                                 const std::string& param_name,
                                                 int expected_size) {
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
            value.size() != expected_size) {
            throw std::runtime_error(param_name + " must be a YAML list of " +
                                     std::to_string(expected_size) + " numbers");
        }

        std::vector<double> result;
        result.reserve(static_cast<size_t>(expected_size));
        for (int i = 0; i < value.size(); ++i) {
            result.push_back(xmlRpcToDouble(value[i], param_name));
        }
        return result;
    }

    static std::array<double, 3> toArray3(const std::vector<double>& values) {
        return {{values[0], values[1], values[2]}};
    }

    static double xmlRpcToDouble(XmlRpc::XmlRpcValue& value,
                                 const std::string& param_name) {
        if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
            return static_cast<int>(value);
        }
        if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
            return static_cast<double>(value);
        }
        throw std::runtime_error(param_name + " entries must be numeric");
    }

    static tf2::Transform poseToTransform(const geometry_msgs::Pose& pose) {
        tf2::Quaternion q(pose.orientation.x,
                          pose.orientation.y,
                          pose.orientation.z,
                          pose.orientation.w);
        q.normalize();
        return tf2::Transform(q, tf2::Vector3(pose.position.x,
                                              pose.position.y,
                                              pose.position.z));
    }

    static geometry_msgs::Pose transformToPose(const tf2::Transform& transform) {
        geometry_msgs::Pose pose;
        pose.position.x = transform.getOrigin().x();
        pose.position.y = transform.getOrigin().y();
        pose.position.z = transform.getOrigin().z();
        pose.orientation.x = transform.getRotation().x();
        pose.orientation.y = transform.getRotation().y();
        pose.orientation.z = transform.getRotation().z();
        pose.orientation.w = transform.getRotation().w();
        return pose;
    }

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber model_states_sub_;

    std::string model_states_topic_;
    std::string bind_address_;
    int port_{3883};
    double publish_rate_hz_{100.0};
    double stale_timeout_s_{1.0};
    double scan_interval_s_{2.0};
    double velocity_filter_cutoff_hz_{25.0};
    double acceleration_filter_cutoff_hz_{25.0};
    double derivative_reset_timeout_s_{0.5};
    int mocap_noise_seed_param_{1};
    std::string match_mode_{"contains"};
    bool auto_track_known_models_{false};
    std::vector<std::string> auto_include_patterns_;
    ros::WallTime last_scan_wall_time_;

    vrpn_Connection* connection_{nullptr};
    tf2::Transform default_body_to_tracker_;
    MocapNoiseConfig mocap_noise_config_;
    MocapNoise mocap_noise_;
    std::vector<std::string> tracker_patterns_;
    std::map<std::string, RobotConfig> robot_configs_;
    std::map<std::string, std::string> configured_model_to_tracker_;
    std::map<std::string, TrackedModel> tracked_models_;
    std::map<std::string, std::string> tracker_names_;
};

}  // namespace gazebo_sim_vrpn_bridge

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_vrpn_server_node");
    gazebo_sim_vrpn_bridge::installSignalHandlers();
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    try {
        gazebo_sim_vrpn_bridge::GazeboVrpnServerNode node(nh, nh_private);
        node.spin();
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM("[GazeboVrpnServerNode] " << e.what());
        return 1;
    }

    return 0;
}
