#pragma once

namespace gazebo_sim_vrpn_bridge {

// Selects the clock domain encoded in VRPN report timeval fields. This is an
// input-adapter contract, independent of the measurement-delay timestamp
// policy (send_time versus sample_time).
enum class WireTimestampSource {
    WallTime,
    SimulationTime,
};

inline const char* wireTimestampSourceName(WireTimestampSource source) {
    return source == WireTimestampSource::SimulationTime ? "simulation_time" : "wall_time";
}

inline double timestampSecondsForSource(WireTimestampSource source, double simulation_time_s, double wall_time_s) {
    return source == WireTimestampSource::SimulationTime ? simulation_time_s : wall_time_s;
}

} // namespace gazebo_sim_vrpn_bridge
