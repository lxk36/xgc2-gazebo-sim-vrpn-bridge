#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>

#include "gazebo_sim_vrpn_bridge/server_config.h"
#include "gazebo_sim_vrpn_bridge/wire_timestamp.h"

namespace gazebo_sim_vrpn_bridge {

struct ModelPoseSample {
    std::string gazebo_model_name;
    geometry_msgs::Pose pose;
};

struct ModelStateSnapshot {
    // Sample time drives derivatives and supplies the VRPN wire timestamp for
    // a simulation-time source. Capture wall time remains authoritative for
    // staleness and bounded delay-history selection.
    double sample_time_s{0.0};
    double capture_wall_time_s{0.0};
    WireTimestampSource wire_timestamp_source{WireTimestampSource::WallTime};
    std::vector<ModelPoseSample> models;
};

// Single-threaded server core shared by both input adapters. Call all methods
// from one owner thread; the Gazebo plugin intentionally keeps this object off
// the physics update thread so network I/O cannot stall simulation.
class VrpnTrackerServer {
  public:
    explicit VrpnTrackerServer(const ServerConfig& config);
    ~VrpnTrackerServer();

    VrpnTrackerServer(const VrpnTrackerServer&) = delete;
    VrpnTrackerServer& operator=(const VrpnTrackerServer&) = delete;

    void processSnapshot(const ModelStateSnapshot& snapshot);
    void publish(double send_wall_time_s);
    void mainloop();

    std::size_t trackedModelCount() const;
    bool hasTracker(const std::string& tracker_name) const;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace gazebo_sim_vrpn_bridge
