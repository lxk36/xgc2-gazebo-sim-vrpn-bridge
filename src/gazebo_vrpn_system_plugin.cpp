#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/PhysicsIface.hh>
#include <gazebo/physics/World.hh>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include "gazebo_sim_vrpn_bridge/server_config.h"
#include "gazebo_sim_vrpn_bridge/vrpn_tracker_server.h"

namespace gazebo_sim_vrpn_bridge {
namespace {

geometry_msgs::Pose poseMessage(const ignition::math::Pose3d& value) {
    geometry_msgs::Pose message;
    message.position.x = value.Pos().X();
    message.position.y = value.Pos().Y();
    message.position.z = value.Pos().Z();
    message.orientation.x = value.Rot().X();
    message.orientation.y = value.Rot().Y();
    message.orientation.z = value.Rot().Z();
    message.orientation.w = value.Rot().W();
    return message;
}

} // namespace

// Server-level plugin loaded with gzserver -s. Gazebo model access stays on the
// simulation thread; all filtering, delay simulation, and VRPN network work is
// performed by one dedicated worker.
class GazeboVrpnSystemPlugin final : public gazebo::SystemPlugin {
  public:
    GazeboVrpnSystemPlugin() = default;

    ~GazeboVrpnSystemPlugin() override { stop(); }

    void Load(int /*argc*/, char** /*argv*/) override {
        world_created_connection_ = gazebo::event::Events::ConnectWorldCreated(
            std::bind(&GazeboVrpnSystemPlugin::onWorldCreated, this, std::placeholders::_1));
    }

  private:
    struct TrackedModelHandle {
        std::string gazebo_model_name;
        gazebo::physics::ModelPtr model;
    };

    void onWorldCreated(const std::string& world_name) {
        if (world_) {
            gzerr << "Gazebo VRPN SystemPlugin supports one world per gzserver process\n";
            return;
        }

        world_ = gazebo::physics::get_world(world_name);
        if (!world_) {
            gzerr << "Gazebo VRPN SystemPlugin could not resolve world " << world_name << "\n";
            return;
        }
        if (!ros::isInitialized()) {
            gzerr << "Gazebo VRPN SystemPlugin requires gazebo_ros_api_plugin to load first\n";
            world_.reset();
            return;
        }

        try {
            // This is the same namespace used by the legacy node's private
            // parameters, allowing one YAML document to configure either
            // backend without changing measurement behavior.
            const ros::NodeHandle config_node("/gazebo_vrpn_server");
            config_ = loadServerConfig(config_node);
        } catch (const std::exception& error) {
            ROS_ERROR("[GazeboVrpnSystemPlugin] Invalid configuration: %s", error.what());
            world_.reset();
            return;
        }

        stopping_.store(false, std::memory_order_release);
        worker_ = std::thread(&GazeboVrpnSystemPlugin::workerLoop, this);
        update_connection_ =
            gazebo::event::Events::ConnectWorldUpdateEnd(std::bind(&GazeboVrpnSystemPlugin::onWorldUpdateEnd, this));
        ROS_INFO("[GazeboVrpnSystemPlugin] Attached directly to Gazebo world '%s'; "
                 "ROS model-state transport is disabled for this backend",
                 world_name.c_str());
    }

    void onWorldUpdateEnd() {
        if (!world_ || stopping_.load(std::memory_order_acquire)) {
            return;
        }

        try {
            // Gazebo may update substantially faster than the requested VRPN
            // rate. Capture at the report cadence so large fleets are not
            // copied on simulation steps that the worker cannot consume.
            const auto capture_time = std::chrono::steady_clock::now();
            if (next_capture_time_ != std::chrono::steady_clock::time_point{} && capture_time < next_capture_time_) {
                return;
            }
            const auto capture_period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / config_.publish_rate_hz));
            if (next_capture_time_ == std::chrono::steady_clock::time_point{}) {
                next_capture_time_ = capture_time;
            }
            do {
                next_capture_time_ += capture_period;
            } while (next_capture_time_ <= capture_time);

            const double capture_wall_time_s = ros::WallTime::now().toSec();
            refreshTrackedModels(capture_wall_time_s);

            ModelStateSnapshot snapshot;
            snapshot.sample_time_s = world_->SimTime().Double();
            snapshot.capture_wall_time_s = capture_wall_time_s;
            snapshot.models.reserve(tracked_models_.size());
            for (const TrackedModelHandle& tracked : tracked_models_) {
                if (!tracked.model) {
                    continue;
                }
                snapshot.models.push_back(
                    ModelPoseSample{tracked.gazebo_model_name, poseMessage(tracked.model->WorldPose())});
            }

            {
                std::lock_guard<std::mutex> lock(snapshot_mutex_);
                latest_snapshot_ = std::move(snapshot);
                ++latest_snapshot_sequence_;
            }
        } catch (const std::exception& error) {
            ROS_ERROR_THROTTLE(2.0, "[GazeboVrpnSystemPlugin] Gazebo snapshot failed: %s", error.what());
        }
    }

    void refreshTrackedModels(double wall_time_s) {
        if (last_scan_wall_time_s_ != 0.0 && wall_time_s - last_scan_wall_time_s_ < config_.scan_interval_s) {
            return;
        }

        std::vector<TrackedModelHandle> discovered;
        for (const gazebo::physics::ModelPtr& model : world_->Models()) {
            if (!model) {
                continue;
            }
            const std::string model_name = model->GetName();
            if (config_.trackerNameForGazeboModel(model_name).empty()) {
                continue;
            }
            discovered.push_back(TrackedModelHandle{model_name, model});
        }
        tracked_models_.swap(discovered);
        last_scan_wall_time_s_ = wall_time_s;
    }

    void workerLoop() noexcept {
        try {
            VrpnTrackerServer server(config_);
            const auto period = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / config_.publish_rate_hz));
            auto next_cycle = std::chrono::steady_clock::now();
            std::uint64_t processed_sequence = 0;

            while (!stopping_.load(std::memory_order_acquire)) {
                ModelStateSnapshot snapshot;
                bool have_snapshot = false;
                {
                    std::lock_guard<std::mutex> lock(snapshot_mutex_);
                    if (latest_snapshot_sequence_ != processed_sequence) {
                        snapshot = std::move(latest_snapshot_);
                        processed_sequence = latest_snapshot_sequence_;
                        have_snapshot = true;
                    }
                }

                if (have_snapshot) {
                    server.processSnapshot(snapshot);
                }
                server.publish(ros::WallTime::now().toSec());
                server.mainloop();

                next_cycle += period;
                const auto now = std::chrono::steady_clock::now();
                if (next_cycle + period < now) {
                    next_cycle = now + period;
                }
                std::unique_lock<std::mutex> lock(worker_wait_mutex_);
                worker_wait_.wait_until(lock, next_cycle, [this] {
                    return stopping_.load(std::memory_order_acquire);
                });
            }
        } catch (const std::exception& error) {
            ROS_ERROR("[GazeboVrpnSystemPlugin] VRPN worker stopped: %s", error.what());
        } catch (...) {
            ROS_ERROR("[GazeboVrpnSystemPlugin] VRPN worker stopped after an unknown error");
        }
    }

    void stop() {
        update_connection_.reset();
        world_created_connection_.reset();
        stopping_.store(true, std::memory_order_release);
        worker_wait_.notify_all();
        if (worker_.joinable()) {
            worker_.join();
        }
        tracked_models_.clear();
        world_.reset();
    }

    gazebo::physics::WorldPtr world_;
    gazebo::event::ConnectionPtr world_created_connection_;
    gazebo::event::ConnectionPtr update_connection_;
    ServerConfig config_;
    std::vector<TrackedModelHandle> tracked_models_;
    double last_scan_wall_time_s_{0.0};
    std::chrono::steady_clock::time_point next_capture_time_;

    std::mutex snapshot_mutex_;
    ModelStateSnapshot latest_snapshot_;
    std::uint64_t latest_snapshot_sequence_{0};

    std::atomic_bool stopping_{false};
    std::mutex worker_wait_mutex_;
    std::condition_variable worker_wait_;
    std::thread worker_;
};

GZ_REGISTER_SYSTEM_PLUGIN(GazeboVrpnSystemPlugin)

} // namespace gazebo_sim_vrpn_bridge
