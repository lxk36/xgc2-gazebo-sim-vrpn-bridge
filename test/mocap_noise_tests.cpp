#include "gazebo_sim_vrpn_bridge/mocap_noise.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

using gazebo_sim_vrpn_bridge::MocapNoise;
using gazebo_sim_vrpn_bridge::MocapNoiseConfig;

namespace {

geometry_msgs::Pose identityPose() {
    geometry_msgs::Pose pose;
    pose.orientation.w = 1.0;
    return pose;
}

double positionNorm(const geometry_msgs::Pose& pose) {
    return std::sqrt(
        pose.position.x * pose.position.x +
        pose.position.y * pose.position.y +
        pose.position.z * pose.position.z);
}

}  // namespace

TEST(MocapNoise, DisabledLeavesPoseUnchanged) {
    MocapNoiseConfig config;
    config.enabled = false;
    config.position_stddev_m = {{1.0, 1.0, 1.0}};
    MocapNoise noise(config);

    geometry_msgs::Pose pose = identityPose();
    pose.position.x = 1.0;
    const geometry_msgs::Pose out = noise.apply(pose);

    EXPECT_DOUBLE_EQ(out.position.x, 1.0);
    EXPECT_DOUBLE_EQ(out.position.y, 0.0);
    EXPECT_DOUBLE_EQ(out.position.z, 0.0);
    EXPECT_DOUBLE_EQ(out.orientation.w, 1.0);
}

TEST(MocapNoise, DefaultNoiseIsTinyButNonzeroOverSamples) {
    MocapNoise noise;
    bool saw_nonzero = false;
    double max_norm = 0.0;

    for (int i = 0; i < 128; ++i) {
        const geometry_msgs::Pose out = noise.apply(identityPose());
        const double norm = positionNorm(out);
        saw_nonzero = saw_nonzero || norm > 0.0;
        max_norm = std::max(max_norm, norm);
    }

    EXPECT_TRUE(saw_nonzero);
    EXPECT_LT(max_norm, 2.0e-6);
}

TEST(MocapNoise, SeedMakesSequenceReproducible) {
    MocapNoiseConfig config;
    config.seed = 42;
    MocapNoise a(config);
    MocapNoise b(config);

    for (int i = 0; i < 16; ++i) {
        const geometry_msgs::Pose pa = a.apply(identityPose());
        const geometry_msgs::Pose pb = b.apply(identityPose());
        EXPECT_DOUBLE_EQ(pa.position.x, pb.position.x);
        EXPECT_DOUBLE_EQ(pa.position.y, pb.position.y);
        EXPECT_DOUBLE_EQ(pa.position.z, pb.position.z);
    }
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
