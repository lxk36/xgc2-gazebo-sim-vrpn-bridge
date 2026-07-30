#include "gazebo_sim_vrpn_bridge/model_identity.h"
#include "gazebo_sim_vrpn_bridge/server_config.h"

#include <gtest/gtest.h>

using gazebo_sim_vrpn_bridge::isCanonicalAutoTrackedModelName;
using gazebo_sim_vrpn_bridge::ServerConfig;

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

TEST(TrackerSelectionMode, AutoModeIgnoresRetainedLegacyPatternDefaults) {
    ServerConfig config;
    config.auto_track_known_models = true;
    config.auto_include_patterns = {"^uav[0-9]+$", "^ugv[0-9]+$", "^mecanum[0-9]+$"};
    config.tracker_patterns = {"uav", "ugv", "mecanum"};
    config.match_mode = "contains";

    EXPECT_EQ("ugv1", config.trackerNameForGazeboModel("ugv1"));
    EXPECT_EQ("ugv2", config.trackerNameForGazeboModel("ugv2"));
    EXPECT_TRUE(config.trackerNameForGazeboModel("scout_ugv_alpha").empty());
}

TEST(TrackerSelectionMode, LegacyModeUsesConfiguredPatternAndMatchMode) {
    ServerConfig config;
    config.auto_track_known_models = false;
    config.tracker_patterns = {"ugv"};
    config.match_mode = "contains";

    EXPECT_EQ("ugv", config.trackerNameForGazeboModel("ugv1"));
    EXPECT_EQ("ugv", config.trackerNameForGazeboModel("scout_ugv_alpha"));
}

TEST(TrackerSelectionMode, ExplicitModelMappingAppliesInEitherMode) {
    ServerConfig config;
    config.auto_track_known_models = true;
    config.auto_include_patterns = {"^ugv[0-9]+$"};
    config.tracker_patterns = {"ugv"};
    config.configured_model_to_tracker["scout_blue"] = "rigid_body_7";

    EXPECT_EQ("rigid_body_7", config.trackerNameForGazeboModel("scout_blue"));
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
