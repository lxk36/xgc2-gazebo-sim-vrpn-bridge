#include <atomic>
#include <csignal>
#include <cstdint>
#include <string>

#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>

#include "gazebo_sim_vrpn_bridge/server_config.h"
#include "gazebo_sim_vrpn_bridge/vrpn_tracker_server.h"

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

double sampleTimeSeconds(const ros::WallTime& fallback_time) {
    const ros::Time ros_time = ros::Time::now();
    return ros_time.isZero() ? fallback_time.toSec() : ros_time.toSec();
}

} // namespace

// Compatibility adapter for deployments that still run the VRPN bridge as a
// separate ROS process. All measurement behavior lives in VrpnTrackerServer so
// this path and the in-process Gazebo plugin share one implementation.
class GazeboVrpnServerNode {
  public:
    GazeboVrpnServerNode(ros::NodeHandle& node, const ros::NodeHandle& private_node)
        : config_(loadServerConfig(private_node)), server_(config_) {
        model_states_subscriber_ =
            node.subscribe(config_.model_states_topic, 1, &GazeboVrpnServerNode::modelStatesCallback, this);
        ROS_INFO_STREAM("[GazeboVrpnServerNode] Reading model state from " << config_.model_states_topic);
    }

    void spin() {
        ros::WallRate rate(config_.publish_rate_hz);
        while (ros::ok() && !g_shutdown_requested.load(std::memory_order_relaxed)) {
            ros::spinOnce();
            processLatestModelStates();
            server_.publish(ros::WallTime::now().toSec());
            server_.mainloop();
            rate.sleep();
        }
    }

  private:
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& message) {
        if (!message || message->name.size() != message->pose.size()) {
            ROS_WARN_THROTTLE(2.0, "[GazeboVrpnServerNode] Invalid /gazebo/model_states message");
            return;
        }

        latest_model_states_ = message;
        latest_model_states_wall_time_ = ros::WallTime::now();
        ++latest_model_states_sequence_;
    }

    void processLatestModelStates() {
        if (!latest_model_states_ || latest_model_states_sequence_ == processed_model_states_sequence_) {
            return;
        }

        const gazebo_msgs::ModelStates::ConstPtr message = latest_model_states_;
        const ros::WallTime capture_wall_time = latest_model_states_wall_time_;
        processed_model_states_sequence_ = latest_model_states_sequence_;

        ModelStateSnapshot snapshot;
        snapshot.sample_time_s = sampleTimeSeconds(capture_wall_time);
        snapshot.capture_wall_time_s = capture_wall_time.toSec();
        snapshot.models.reserve(message->name.size());
        for (std::size_t index = 0; index < message->name.size(); ++index) {
            snapshot.models.push_back(ModelPoseSample{message->name[index], message->pose[index]});
        }
        server_.processSnapshot(snapshot);
    }

    ServerConfig config_;
    VrpnTrackerServer server_;
    ros::Subscriber model_states_subscriber_;
    gazebo_msgs::ModelStates::ConstPtr latest_model_states_;
    ros::WallTime latest_model_states_wall_time_;
    std::uint64_t latest_model_states_sequence_{0};
    std::uint64_t processed_model_states_sequence_{0};
};

} // namespace gazebo_sim_vrpn_bridge

int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_vrpn_server_node");
    gazebo_sim_vrpn_bridge::installSignalHandlers();
    ros::NodeHandle node;
    ros::NodeHandle private_node("~");

    try {
        gazebo_sim_vrpn_bridge::GazeboVrpnServerNode server(node, private_node);
        server.spin();
    } catch (const std::exception& error) {
        ROS_FATAL_STREAM("[GazeboVrpnServerNode] " << error.what());
        return 1;
    }
    return 0;
}
