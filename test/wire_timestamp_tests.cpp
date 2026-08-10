#include "gazebo_sim_vrpn_bridge/wire_timestamp.h"

#include <gtest/gtest.h>

using gazebo_sim_vrpn_bridge::timestampSecondsForSource;
using gazebo_sim_vrpn_bridge::WireTimestampSource;
using gazebo_sim_vrpn_bridge::wireTimestampSourceName;

TEST(WireTimestamp, WallSourceSelectsWallFact) {
    EXPECT_DOUBLE_EQ(timestampSecondsForSource(WireTimestampSource::WallTime, 12.5, 1700000000.25), 1700000000.25);
    EXPECT_STREQ(wireTimestampSourceName(WireTimestampSource::WallTime), "wall_time");
}

TEST(WireTimestamp, SimulationSourceSelectsWorldSimulationFact) {
    EXPECT_DOUBLE_EQ(timestampSecondsForSource(WireTimestampSource::SimulationTime, 12.5, 1700000000.25), 12.5);
    EXPECT_STREQ(wireTimestampSourceName(WireTimestampSource::SimulationTime), "simulation_time");
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
