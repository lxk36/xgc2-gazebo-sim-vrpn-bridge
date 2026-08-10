#include "gazebo_sim_vrpn_bridge/vrpn_tracker_server.h"

#include <sys/time.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <vrpn_Connection.h>
#include <vrpn_Tracker.h>
#include <xgc2_math/filter/butterworth_filter.hpp>

#include "gazebo_sim_vrpn_bridge/measurement_delay.h"
#include "gazebo_sim_vrpn_bridge/mocap_noise.h"

namespace gazebo_sim_vrpn_bridge {
namespace {

constexpr double kTimeEpsilon = 1.0e-9;

const char* delayTimestampPolicyName(DelayTimestampPolicy policy) {
    return policy == DelayTimestampPolicy::SampleTime ? "sample_time" : "send_time";
}

tf2::Transform poseToTransform(const geometry_msgs::Pose& pose) {
    tf2::Quaternion quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    quaternion.normalize();
    return tf2::Transform(quaternion, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
}

geometry_msgs::Pose transformToPose(const tf2::Transform& transform) {
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

void resetFilters(std::array<xgc2_math::SecondOrderButterworthLowPass, 3>& filters, double cutoff_hz, double value) {
    for (auto& filter : filters) {
        filter.reset(cutoff_hz, value);
    }
}

tf2::Vector3 filterVector(std::array<xgc2_math::SecondOrderButterworthLowPass, 3>& filters, const tf2::Vector3& value,
                          double dt_s) {
    return tf2::Vector3(filters[0].filter(value.x(), dt_s), filters[1].filter(value.y(), dt_s),
                        filters[2].filter(value.z(), dt_s));
}

tf2::Vector3 angularVelocityBetween(const tf2::Quaternion& previous, const tf2::Quaternion& current, double dt_s) {
    tf2::Quaternion delta = current * previous.inverse();
    delta.normalize();
    if (delta.w() < 0.0) {
        delta = tf2::Quaternion(-delta.x(), -delta.y(), -delta.z(), -delta.w());
    }

    const double angle = delta.getAngle();
    if (std::abs(angle) < 1.0e-9) {
        return tf2::Vector3(0.0, 0.0, 0.0);
    }
    return delta.getAxis() * (angle / dt_s);
}

void vectorRpyToQuaternion(const tf2::Vector3& rpy, vrpn_float64 quaternion[4]) {
    tf2::Quaternion value;
    value.setRPY(rpy.x(), rpy.y(), rpy.z());
    value.normalize();
    quaternion[0] = value.x();
    quaternion[1] = value.y();
    quaternion[2] = value.z();
    quaternion[3] = value.w();
}

struct timeval secondsToTimeval(double time_s) {
    time_s = std::max(time_s, 0.0);
    const double whole_seconds = std::floor(time_s);
    struct timeval timestamp {};
    timestamp.tv_sec = static_cast<time_t>(whole_seconds);
    timestamp.tv_usec = static_cast<suseconds_t>(std::round((time_s - whole_seconds) * 1000000.0));
    if (timestamp.tv_usec >= 1000000) {
        ++timestamp.tv_sec;
        timestamp.tv_usec -= 1000000;
    }
    return timestamp;
}

} // namespace

class VrpnTrackerServer::Impl {
  public:
    explicit Impl(const ServerConfig& config)
        : config_(config), mocap_noise_(config.mocap_noise), measurement_delay_(config.delay) {
        connection_ = vrpn_create_server_connection(
            config_.port, nullptr, nullptr, config_.bind_address.empty() ? nullptr : config_.bind_address.c_str());
        if (connection_ == nullptr) {
            throw std::runtime_error("failed to create VRPN server connection");
        }

        ROS_INFO_STREAM("[GazeboVrpnServer] Serving Gazebo models as VRPN trackers on port "
                        << config_.port << (config_.bind_address.empty() ? "" : " bound to " + config_.bind_address));
        ROS_INFO_STREAM("[GazeboVrpnServer] Scan interval is " << config_.scan_interval_s
                                                               << " s; match_mode=" << config_.match_mode);
        if (!config_.tracker_patterns.empty()) {
            ROS_INFO_STREAM("[GazeboVrpnServer] Tracker pattern count: " << config_.tracker_patterns.size());
        }
        if (!config_.auto_include_patterns.empty()) {
            ROS_INFO_STREAM("[GazeboVrpnServer] Auto mapping pattern count: " << config_.auto_include_patterns.size());
        }
        if (!config_.auto_track_known_models) {
            ROS_INFO_STREAM("[GazeboVrpnServer] Auto export is disabled; configure trackers or robots");
        }
        if (config_.mocap_noise.enabled) {
            ROS_INFO_STREAM(
                "[GazeboVrpnServer] Mocap measurement noise enabled: position stddev xyz=["
                << config_.mocap_noise.position_stddev_m[0] << ", " << config_.mocap_noise.position_stddev_m[1] << ", "
                << config_.mocap_noise.position_stddev_m[2] << "] m, rotation stddev rpy=["
                << config_.mocap_noise.rotation_stddev_rad[0] << ", " << config_.mocap_noise.rotation_stddev_rad[1]
                << ", " << config_.mocap_noise.rotation_stddev_rad[2] << "] rad, seed=" << config_.mocap_noise.seed);
        }
        ROS_INFO_STREAM("[GazeboVrpnServer] Measurement delay is "
                        << (config_.delay.enabled ? "enabled" : "disabled")
                        << "; timestamp_policy=" << delayTimestampPolicyName(config_.delay.timestamp_policy));
    }

    ~Impl() {
        tracked_models_.clear();
        if (connection_ != nullptr) {
            connection_->removeReference();
            connection_ = nullptr;
        }
    }

    void processSnapshot(const ModelStateSnapshot& snapshot) {
        if (!have_wire_timestamp_source_) {
            ROS_INFO("[GazeboVrpnServer] VRPN wire timestamp source is %s",
                     wireTimestampSourceName(snapshot.wire_timestamp_source));
        } else if (snapshot.wire_timestamp_source != wire_timestamp_source_) {
            resetMeasurementState();
            ROS_WARN("[GazeboVrpnServer] VRPN wire timestamp source changed from %s to %s; "
                     "measurement state was reset",
                     wireTimestampSourceName(wire_timestamp_source_),
                     wireTimestampSourceName(snapshot.wire_timestamp_source));
        }
        wire_timestamp_source_ = snapshot.wire_timestamp_source;
        have_wire_timestamp_source_ = true;

        if (have_sample_time_ && snapshot.sample_time_s + kTimeEpsilon < last_sample_time_s_) {
            resetMeasurementState();
            ROS_INFO("[GazeboVrpnServer] Simulation time moved backwards; measurement state was reset");
        }
        last_sample_time_s_ = snapshot.sample_time_s;
        have_sample_time_ = true;

        if (shouldScan(snapshot.capture_wall_time_s)) {
            scanModelList(snapshot);
        }

        for (auto& tracked : tracked_models_) {
            tracked.second.have_pose = false;
        }
        for (const ModelPoseSample& sample : snapshot.models) {
            const auto found = tracked_models_.find(sample.gazebo_model_name);
            if (found == tracked_models_.end()) {
                continue;
            }
            const double source_time_s = timestampSecondsForSource(
                snapshot.wire_timestamp_source, snapshot.sample_time_s, snapshot.capture_wall_time_s);
            updateTrackedModel(found->second, sample.pose, snapshot.sample_time_s, snapshot.capture_wall_time_s,
                               source_time_s);
        }
    }

    void publish(double send_wall_time_s) {
        measurement_delay_.startPublishCycle(send_wall_time_s);

        for (auto& entry : tracked_models_) {
            TrackedModel& model = entry.second;
            if (!model.have_pose) {
                continue;
            }

            const double age_s = send_wall_time_s - model.last_model_state_wall_time_s;
            if (age_s > config_.stale_timeout_s) {
                ROS_WARN_THROTTLE(2.0, "[GazeboVrpnServer] Last Gazebo pose for '%s' is stale: %.3f s",
                                  model.gazebo_model_name.c_str(), age_s);
            }

            const TrackerSample* sample = sampleForPublish(model, send_wall_time_s);
            if (sample == nullptr) {
                continue;
            }

            const double send_source_time_s =
                timestampSecondsForSource(wire_timestamp_source_, last_sample_time_s_, send_wall_time_s);
            const double timestamp_s =
                timestampSecondsForPolicy(config_.delay.timestamp_policy, send_source_time_s, *sample);
            const struct timeval timestamp = secondsToTimeval(timestamp_s);
            const geometry_msgs::Pose measured_pose = mocap_noise_.apply(sample->pose);
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

            const int pose_status = model.tracker->report_pose(0, timestamp, position, quaternion);
            if (pose_status != 0) {
                ROS_WARN_THROTTLE(2.0, "[GazeboVrpnServer] Failed to publish VRPN pose for '%s'",
                                  model.tracker_name.c_str());
            }

            if (sample->have_velocity) {
                const vrpn_float64 linear_velocity[3] = {
                    sample->linear_velocity.x(),
                    sample->linear_velocity.y(),
                    sample->linear_velocity.z(),
                };
                vrpn_float64 angular_velocity[4]{};
                vectorRpyToQuaternion(sample->angular_velocity, angular_velocity);
                const int velocity_status = model.tracker->report_pose_velocity(
                    0, timestamp, linear_velocity, angular_velocity, sample->report_interval_s);
                if (velocity_status != 0) {
                    ROS_WARN_THROTTLE(2.0, "[GazeboVrpnServer] Failed to publish VRPN twist for '%s'",
                                      model.tracker_name.c_str());
                }
            }

            if (sample->have_acceleration) {
                const vrpn_float64 linear_acceleration[3] = {
                    sample->linear_acceleration.x(),
                    sample->linear_acceleration.y(),
                    sample->linear_acceleration.z(),
                };
                vrpn_float64 angular_acceleration[4]{};
                vectorRpyToQuaternion(sample->angular_acceleration, angular_acceleration);
                const int acceleration_status = model.tracker->report_pose_acceleration(
                    0, timestamp, linear_acceleration, angular_acceleration, sample->report_interval_s);
                if (acceleration_status != 0) {
                    ROS_WARN_THROTTLE(2.0, "[GazeboVrpnServer] Failed to publish VRPN accel for '%s'",
                                      model.tracker_name.c_str());
                }
            }
        }
    }

    void mainloop() {
        for (auto& entry : tracked_models_) {
            entry.second.tracker->mainloop();
        }
        connection_->mainloop();
    }

    std::size_t trackedModelCount() const { return tracked_models_.size(); }

    bool hasTracker(const std::string& tracker_name) const { return tracker_names_.count(tracker_name) != 0U; }

  private:
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
        std::array<xgc2_math::SecondOrderButterworthLowPass, 3> linear_velocity_filters;
        std::array<xgc2_math::SecondOrderButterworthLowPass, 3> angular_velocity_filters;
        std::array<xgc2_math::SecondOrderButterworthLowPass, 3> linear_acceleration_filters;
        std::array<xgc2_math::SecondOrderButterworthLowPass, 3> angular_acceleration_filters;
        tf2::Transform body_to_tracker;
        double last_model_state_wall_time_s{0.0};
        double last_model_state_time_s{0.0};
        double last_derivative_dt_s{0.0};
        TrackerSampleHistory history{4U};
        bool have_pose{false};
        bool have_derivative_state{false};
        bool have_raw_velocity{false};
        bool have_velocity{false};
        bool have_acceleration{false};
        std::unique_ptr<vrpn_Tracker_Server> tracker;
    };

    bool shouldScan(double capture_wall_time_s) const {
        return last_scan_wall_time_s_ == 0.0 || capture_wall_time_s - last_scan_wall_time_s_ >= config_.scan_interval_s;
    }

    void scanModelList(const ModelStateSnapshot& snapshot) {
        bool matched_any_model = false;
        for (const ModelPoseSample& sample : snapshot.models) {
            const std::string tracker_name = config_.trackerNameForGazeboModel(sample.gazebo_model_name);
            if (tracker_name.empty()) {
                continue;
            }
            matched_any_model = true;
            ensureTrackedModel(sample.gazebo_model_name, tracker_name);
        }
        last_scan_wall_time_s_ = snapshot.capture_wall_time_s;

        if (!matched_any_model) {
            ROS_WARN_THROTTLE(2.0, "[GazeboVrpnServer] No configured tracker matched Gazebo models");
        }
    }

    TrackedModel* ensureTrackedModel(const std::string& gazebo_model_name, const std::string& tracker_name) {
        auto existing = tracked_models_.find(gazebo_model_name);
        if (existing != tracked_models_.end()) {
            return &existing->second;
        }

        if (tracker_names_.count(tracker_name) > 0U) {
            ROS_WARN_STREAM_THROTTLE(10.0, "[GazeboVrpnServer] Ignoring Gazebo model '"
                                               << gazebo_model_name << "' because tracker name '" << tracker_name
                                               << "' is already in use");
            return nullptr;
        }

        TrackedModel model;
        model.gazebo_model_name = gazebo_model_name;
        model.tracker_name = tracker_name;
        model.body_to_tracker = config_.bodyToTrackerFor(tracker_name);
        model.history.reset(measurement_delay_.historyCapacityForTracker(tracker_name, config_.publish_rate_hz));
        model.tracker = std::make_unique<vrpn_Tracker_Server>(tracker_name.c_str(), connection_, 1);

        ROS_INFO_STREAM("[GazeboVrpnServer] Registered Gazebo model '" << gazebo_model_name << "' as VRPN tracker '"
                                                                       << tracker_name << "'");
        tracker_names_[tracker_name] = gazebo_model_name;
        auto result = tracked_models_.emplace(gazebo_model_name, std::move(model));
        return &result.first->second;
    }

    void updateTrackedModel(TrackedModel& model, const geometry_msgs::Pose& pose, double sample_time_s,
                            double capture_wall_time_s, double source_time_s) {
        const tf2::Transform world_body = poseToTransform(pose);
        const tf2::Transform world_tracker = world_body * model.body_to_tracker;
        updateDerivativeState(model, world_tracker, sample_time_s);
        model.latest_pose = transformToPose(world_tracker);
        model.last_model_state_wall_time_s = capture_wall_time_s;
        model.have_pose = true;
        pushTrackerSample(model, capture_wall_time_s, source_time_s);
    }

    void updateDerivativeState(TrackedModel& model, const tf2::Transform& world_tracker, double sample_time_s) const {
        if (!model.have_derivative_state) {
            initializeDerivativeState(model, world_tracker, sample_time_s);
            return;
        }

        const double dt_s = sample_time_s - model.last_model_state_time_s;
        if (dt_s <= 1.0e-6 || dt_s > config_.derivative_reset_timeout_s) {
            initializeDerivativeState(model, world_tracker, sample_time_s);
            return;
        }

        const tf2::Vector3 raw_linear_velocity =
            (world_tracker.getOrigin() - model.last_tracker_transform.getOrigin()) / dt_s;
        const tf2::Vector3 raw_angular_velocity =
            angularVelocityBetween(model.last_tracker_transform.getRotation(), world_tracker.getRotation(), dt_s);

        model.linear_velocity = filterVector(model.linear_velocity_filters, raw_linear_velocity, dt_s);
        model.angular_velocity = filterVector(model.angular_velocity_filters, raw_angular_velocity, dt_s);

        if (model.have_raw_velocity) {
            const tf2::Vector3 raw_linear_acceleration =
                (raw_linear_velocity - model.previous_raw_linear_velocity) / dt_s;
            const tf2::Vector3 raw_angular_acceleration =
                (raw_angular_velocity - model.previous_raw_angular_velocity) / dt_s;
            model.linear_acceleration = filterVector(model.linear_acceleration_filters, raw_linear_acceleration, dt_s);
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

    void initializeDerivativeState(TrackedModel& model, const tf2::Transform& world_tracker,
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
        resetFilters(model.linear_velocity_filters, config_.velocity_filter_cutoff_hz, 0.0);
        resetFilters(model.angular_velocity_filters, config_.velocity_filter_cutoff_hz, 0.0);
        resetFilters(model.linear_acceleration_filters, config_.acceleration_filter_cutoff_hz, 0.0);
        resetFilters(model.angular_acceleration_filters, config_.acceleration_filter_cutoff_hz, 0.0);
        model.have_derivative_state = true;
        model.have_raw_velocity = false;
        model.have_velocity = false;
        model.have_acceleration = false;
    }

    void resetMeasurementState() {
        for (auto& entry : tracked_models_) {
            TrackedModel& model = entry.second;
            model.history.reset(model.history.capacity());
            model.have_pose = false;
            model.have_derivative_state = false;
            model.have_raw_velocity = false;
            model.have_velocity = false;
            model.have_acceleration = false;
        }
    }

    void pushTrackerSample(TrackedModel& model, double capture_wall_time_s, double source_time_s) {
        TrackerSample sample;
        sample.wall_time_s = capture_wall_time_s;
        sample.source_time_s = source_time_s;
        sample.pose = model.latest_pose;
        sample.linear_velocity = model.linear_velocity;
        sample.angular_velocity = model.angular_velocity;
        sample.linear_acceleration = model.linear_acceleration;
        sample.angular_acceleration = model.angular_acceleration;
        sample.have_velocity = model.have_velocity;
        sample.have_acceleration = model.have_acceleration;
        sample.report_interval_s = reportInterval(model);
        model.history.push(sample);
    }

    const TrackerSample* sampleForPublish(TrackedModel& model, double send_wall_time_s) {
        if (!config_.delay.enabled) {
            return model.history.latest();
        }
        const double delay_s = measurement_delay_.delaySecondsForTracker(model.tracker_name, send_wall_time_s);
        return model.history.nearest(send_wall_time_s - delay_s);
    }

    double reportInterval(const TrackedModel& model) const {
        if (model.last_derivative_dt_s > 0.0) {
            return model.last_derivative_dt_s;
        }
        return config_.publish_rate_hz > 0.0 ? 1.0 / config_.publish_rate_hz : 0.0;
    }

    ServerConfig config_;
    vrpn_Connection* connection_{nullptr};
    MocapNoise mocap_noise_;
    MeasurementDelay measurement_delay_;
    std::map<std::string, TrackedModel> tracked_models_;
    std::map<std::string, std::string> tracker_names_;
    double last_scan_wall_time_s_{0.0};
    double last_sample_time_s_{0.0};
    bool have_sample_time_{false};
    WireTimestampSource wire_timestamp_source_{WireTimestampSource::WallTime};
    bool have_wire_timestamp_source_{false};
};

VrpnTrackerServer::VrpnTrackerServer(const ServerConfig& config) : impl_(std::make_unique<Impl>(config)) {}

VrpnTrackerServer::~VrpnTrackerServer() = default;

void VrpnTrackerServer::processSnapshot(const ModelStateSnapshot& snapshot) {
    impl_->processSnapshot(snapshot);
}

void VrpnTrackerServer::publish(double send_wall_time_s) {
    impl_->publish(send_wall_time_s);
}

void VrpnTrackerServer::mainloop() {
    impl_->mainloop();
}

std::size_t VrpnTrackerServer::trackedModelCount() const {
    return impl_->trackedModelCount();
}

bool VrpnTrackerServer::hasTracker(const std::string& tracker_name) const {
    return impl_->hasTracker(tracker_name);
}

} // namespace gazebo_sim_vrpn_bridge
