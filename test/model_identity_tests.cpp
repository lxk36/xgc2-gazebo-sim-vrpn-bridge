#include "gazebo_sim_vrpn_bridge/model_identity.h"

#include <gtest/gtest.h>

using gazebo_sim_vrpn_bridge::isCanonicalAutoTrackedModelName;

TEST(ModelIdentity, AcceptsCanonicalNumberedRobotModels) {
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("uav1"));
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("uav12"));
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("ugv1"));
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("ugv12"));
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("mecanum1"));
    EXPECT_TRUE(isCanonicalAutoTrackedModelName("mecanum12"));
}

TEST(ModelIdentity, RejectsUnnumberedAndNoncanonicalNames) {
    EXPECT_FALSE(isCanonicalAutoTrackedModelName(""));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("uav"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("ugv"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("mecanum"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("mecanum_1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("mecanum-1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("Mecanum1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("omni1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("scout_mecanum1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("scoutugv1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("scout_mini_ugv1"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("scout_mini_ros_control"));
    EXPECT_FALSE(isCanonicalAutoTrackedModelName("scout_description"));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
