#include "gazebo_sim_vrpn_bridge/measurement_delay.h"

#include <gtest/gtest.h>

#include <string>
#include <vector>

using gazebo_sim_vrpn_bridge::DelayTimestampPolicy;
using gazebo_sim_vrpn_bridge::MeasurementDelay;
using gazebo_sim_vrpn_bridge::MeasurementDelayConfig;
using gazebo_sim_vrpn_bridge::TrackerDelayConfig;
using gazebo_sim_vrpn_bridge::TrackerSample;
using gazebo_sim_vrpn_bridge::TrackerSampleHistory;
using gazebo_sim_vrpn_bridge::timestampSecondsForPolicy;

namespace {

TrackerSample sampleAt(double time_s) {
    TrackerSample sample;
    sample.wall_time_s = time_s;
    sample.source_time_s = time_s;
    sample.pose.orientation.w = 1.0;
    return sample;
}

double trackerDelay(MeasurementDelay& delay, const std::string& tracker_name, double now_s) {
    delay.startPublishCycle(now_s);
    return delay.delaySecondsForTracker(tracker_name, now_s);
}

std::vector<double> collectDelays(MeasurementDelay& delay, const std::string& tracker_name) {
    std::vector<double> values;
    for (int i = 0; i < 8; ++i) {
        values.push_back(trackerDelay(delay, tracker_name, 1.0 + static_cast<double>(i) * 0.1));
    }
    return values;
}

void expectInvalid(const MeasurementDelayConfig& config, const std::string& label) {
    SCOPED_TRACE(label);
    MeasurementDelay delay(config);
    EXPECT_THROW(delay.validate(120.0), std::runtime_error);
}

} // namespace

TEST(MeasurementDelay, DisabledDelayIsZeroAndUsesSmallHistory) {
    MeasurementDelay delay;

    EXPECT_FALSE(delay.enabled());
    EXPECT_DOUBLE_EQ(trackerDelay(delay, "uav1", 1.0), 0.0);
    EXPECT_EQ(delay.historyCapacityForTracker("uav1", 120.0), 4U);
    EXPECT_EQ(delay.timestampPolicy(), DelayTimestampPolicy::SendTime);
}

TEST(MeasurementDelay, DisabledDelayIgnoresConfiguredComponents) {
    MeasurementDelayConfig config;
    config.enabled = false;
    config.common.base_ms = 50.0;
    config.common.jitter_stddev_ms = 10.0;
    config.max_delay_ms = 30.0;

    TrackerDelayConfig tracker;
    tracker.base_ms = 20.0;
    tracker.burst_probability = 1.0;
    tracker.burst_extra_min_ms = 10.0;
    tracker.burst_extra_max_ms = 20.0;
    tracker.max_delay_ms = 5.0;
    config.trackers["uav1"] = tracker;

    MeasurementDelay delay(config);

    EXPECT_FALSE(delay.enabled());
    EXPECT_DOUBLE_EQ(trackerDelay(delay, "uav1", 1.0), 0.0);
    EXPECT_EQ(delay.historyCapacityForTracker("uav1", 1000.0), 4U);
}

TEST(MeasurementDelay, EnabledDefaultMaxDelayIsThirtyMilliseconds) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.common.base_ms = 100.0;

    MeasurementDelay delay(config);

    EXPECT_DOUBLE_EQ(trackerDelay(delay, "uav1", 1.0), 0.030);
}

TEST(MeasurementDelay, BaseDelayAddsCommonAndTrackerThenClamps) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.max_delay_ms = 100.0;
    config.common.base_ms = 10.0;

    TrackerDelayConfig tracker;
    tracker.base_ms = 30.0;
    tracker.max_delay_ms = 35.0;
    config.trackers["uav1"] = tracker;

    MeasurementDelay delay(config);

    EXPECT_DOUBLE_EQ(trackerDelay(delay, "uav1", 1.0), 0.035);
    EXPECT_DOUBLE_EQ(trackerDelay(delay, "uav2", 1.0), 0.010);
}

TEST(MeasurementDelay, NegativeDelayContributionsClampToZero) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.common.base_ms = -50.0;
    config.max_delay_ms = 30.0;

    MeasurementDelay delay(config);

    EXPECT_DOUBLE_EQ(trackerDelay(delay, "uav1", 1.0), 0.0);
}

TEST(MeasurementDelay, SeedMakesRandomSequenceReproducible) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.seed = 42;
    config.max_delay_ms = 30.0;
    config.common.slow_stddev_ms = 5.0;
    config.common.slow_tau_s = 2.0;
    config.common.jitter_stddev_ms = 3.0;

    TrackerDelayConfig tracker;
    tracker.jitter_stddev_ms = 4.0;
    tracker.burst_probability = 1.0;
    tracker.burst_extra_min_ms = 20.0;
    tracker.burst_extra_max_ms = 40.0;
    config.trackers["uav1"] = tracker;

    MeasurementDelay first(config);
    MeasurementDelay second(config);

    for (int i = 0; i < 16; ++i) {
        const double time_s = 1.0 + static_cast<double>(i) * 0.1;
        EXPECT_DOUBLE_EQ(trackerDelay(first, "uav1", time_s), trackerDelay(second, "uav1", time_s));
    }
}

TEST(MeasurementDelay, ResetReplaysRandomSequence) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.seed = 23;
    config.max_delay_ms = 30.0;
    config.common.jitter_stddev_ms = 2.0;

    TrackerDelayConfig tracker;
    tracker.jitter_stddev_ms = 3.0;
    tracker.burst_probability = 1.0;
    tracker.burst_extra_min_ms = 2.0;
    tracker.burst_extra_max_ms = 10.0;
    config.trackers["uav1"] = tracker;

    MeasurementDelay delay(config);
    const std::vector<double> first = collectDelays(delay, "uav1");
    delay.reset(config);
    const std::vector<double> second = collectDelays(delay, "uav1");

    ASSERT_EQ(first.size(), second.size());
    for (size_t i = 0; i < first.size(); ++i) {
        EXPECT_DOUBLE_EQ(first[i], second[i]);
    }
}

TEST(MeasurementDelay, TrackerDelayIsSampledOncePerPublishCycle) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.seed = 31;
    config.max_delay_ms = 30.0;
    config.common.jitter_stddev_ms = 2.0;

    TrackerDelayConfig tracker;
    tracker.jitter_stddev_ms = 5.0;
    tracker.burst_probability = 1.0;
    tracker.burst_extra_min_ms = 1.0;
    tracker.burst_extra_max_ms = 20.0;
    config.trackers["uav1"] = tracker;

    MeasurementDelay delay(config);
    delay.startPublishCycle(10.0);
    const double first = delay.delaySecondsForTracker("uav1", 10.0);
    const double second = delay.delaySecondsForTracker("uav1", 10.0);

    EXPECT_DOUBLE_EQ(first, second);
}

TEST(MeasurementDelay, CommonRandomTermIsSharedAcrossTrackersInCycle) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.seed = 37;
    config.max_delay_ms = 30.0;
    config.common.jitter_stddev_ms = 5.0;

    MeasurementDelay delay(config);
    delay.startPublishCycle(20.0);
    const double uav_delay = delay.delaySecondsForTracker("uav1", 20.0);
    const double ugv_delay = delay.delaySecondsForTracker("ugv1", 20.0);

    EXPECT_DOUBLE_EQ(uav_delay, ugv_delay);
}

TEST(MeasurementDelay, BurstRangeAndClampAreApplied) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.seed = 7;
    config.max_delay_ms = 120.0;

    TrackerDelayConfig tracker;
    tracker.burst_probability = 1.0;
    tracker.burst_extra_min_ms = 80.0;
    tracker.burst_extra_max_ms = 200.0;
    config.trackers["uav1"] = tracker;

    MeasurementDelay delay(config);

    for (int i = 0; i < 16; ++i) {
        const double value = trackerDelay(delay, "uav1", 1.0 + static_cast<double>(i) * 0.1);
        EXPECT_GE(value, 0.080);
        EXPECT_LE(value, 0.120);
    }
}

TEST(MeasurementDelay, BurstProbabilityZeroNeverAddsBurst) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.seed = 7;
    config.max_delay_ms = 30.0;

    TrackerDelayConfig tracker;
    tracker.base_ms = 4.0;
    tracker.burst_probability = 0.0;
    tracker.burst_extra_min_ms = 10.0;
    tracker.burst_extra_max_ms = 20.0;
    config.trackers["uav1"] = tracker;

    MeasurementDelay delay(config);

    for (int i = 0; i < 16; ++i) {
        EXPECT_DOUBLE_EQ(trackerDelay(delay, "uav1", 1.0 + static_cast<double>(i) * 0.1), 0.004);
    }
}

TEST(MeasurementDelay, HistoryCapacityUsesMaxDelayAndMargin) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.max_delay_ms = 30.0;
    config.history_margin_ms = 100.0;

    MeasurementDelay delay(config);

    EXPECT_EQ(delay.historyCapacityForTracker("uav1", 120.0), 19U);
}

TEST(MeasurementDelay, HistoryCapacityUsesTrackerOverrideAndGlobalFallback) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.max_delay_ms = 30.0;
    config.history_margin_ms = 20.0;

    TrackerDelayConfig tracker;
    tracker.max_delay_ms = 10.0;
    config.trackers["uav1"] = tracker;

    MeasurementDelay delay(config);

    EXPECT_EQ(delay.historyCapacityForTracker("uav1", 120.0), 7U);
    EXPECT_EQ(delay.historyCapacityForTracker("ugv1", 120.0), 9U);
}

TEST(TrackerSampleHistory, CapacityIsBoundedAndNearestSampleWins) {
    TrackerSampleHistory history(3);

    for (int i = 0; i < 10; ++i) {
        history.push(sampleAt(static_cast<double>(i)));
    }

    ASSERT_EQ(history.capacity(), 3U);
    ASSERT_EQ(history.size(), 3U);
    ASSERT_NE(history.latest(), nullptr);
    EXPECT_DOUBLE_EQ(history.latest()->wall_time_s, 9.0);
    ASSERT_NE(history.nearest(1.0), nullptr);
    EXPECT_DOUBLE_EQ(history.nearest(1.0)->wall_time_s, 7.0);
    ASSERT_NE(history.nearest(8.6), nullptr);
    EXPECT_DOUBLE_EQ(history.nearest(8.6)->wall_time_s, 9.0);
}

TEST(MeasurementDelay, TimestampPolicySelectsSendOrSampleTime) {
    TrackerSample sample = sampleAt(1700000000.25);
    sample.source_time_s = 12.5;

    EXPECT_DOUBLE_EQ(timestampSecondsForPolicy(DelayTimestampPolicy::SendTime, 20.0, sample), 20.0);
    EXPECT_DOUBLE_EQ(timestampSecondsForPolicy(DelayTimestampPolicy::SampleTime, 20.0, sample), 12.5);
    EXPECT_DOUBLE_EQ(sample.wall_time_s, 1700000000.25);
}

TEST(MeasurementDelay, ValidationRejectsInvalidScalarConfig) {
    MeasurementDelayConfig config;
    config.max_delay_ms = -1.0;
    expectInvalid(config, "negative global max delay");

    config = MeasurementDelayConfig{};
    config.history_margin_ms = -1.0;
    expectInvalid(config, "negative history margin");

    config = MeasurementDelayConfig{};
    MeasurementDelay delay(config);
    EXPECT_THROW(delay.validate(0.0), std::runtime_error);
}

TEST(MeasurementDelay, ValidationRejectsInvalidCommonComponent) {
    MeasurementDelayConfig config;
    config.common.slow_stddev_ms = -1.0;
    expectInvalid(config, "negative common slow stddev");

    config = MeasurementDelayConfig{};
    config.common.slow_tau_s = 0.0;
    expectInvalid(config, "non-positive common slow tau");

    config = MeasurementDelayConfig{};
    config.common.jitter_stddev_ms = -1.0;
    expectInvalid(config, "negative common jitter stddev");

    config = MeasurementDelayConfig{};
    config.common.burst_probability = 1.1;
    expectInvalid(config, "common burst probability greater than one");

    config = MeasurementDelayConfig{};
    config.common.burst_probability = -0.1;
    expectInvalid(config, "negative common burst probability");

    config = MeasurementDelayConfig{};
    config.common.burst_extra_min_ms = 20.0;
    config.common.burst_extra_max_ms = 10.0;
    expectInvalid(config, "common reversed burst range");
}

TEST(MeasurementDelay, ValidationRejectsInvalidTrackerComponent) {
    MeasurementDelayConfig config;
    TrackerDelayConfig tracker;
    tracker.jitter_stddev_ms = -1.0;
    config.trackers["uav1"] = tracker;
    expectInvalid(config, "negative tracker jitter stddev");

    config = MeasurementDelayConfig{};
    tracker = TrackerDelayConfig{};
    tracker.max_delay_ms = -2.0;
    config.trackers["uav1"] = tracker;
    expectInvalid(config, "invalid tracker max delay sentinel");

    config = MeasurementDelayConfig{};
    tracker = TrackerDelayConfig{};
    tracker.burst_extra_min_ms = -1.0;
    tracker.burst_extra_max_ms = 10.0;
    config.trackers["uav1"] = tracker;
    expectInvalid(config, "negative tracker burst range");
}

TEST(MeasurementDelay, ValidationRejectsUnboundedHistory) {
    MeasurementDelayConfig config;
    config.enabled = true;
    config.max_delay_ms = 100000.0;

    MeasurementDelay delay(config);

    EXPECT_THROW(delay.validate(120.0, 1000U), std::runtime_error);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
