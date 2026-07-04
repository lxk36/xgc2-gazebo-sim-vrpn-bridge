#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Vector3.h>

namespace gazebo_sim_vrpn_bridge {

enum class DelayTimestampPolicy {
    SendTime,
    SampleTime,
};

struct DelayComponentConfig {
    double base_ms{0.0};
    double slow_stddev_ms{0.0};
    double slow_tau_s{5.0};
    double jitter_stddev_ms{0.0};
    double burst_probability{0.0};
    double burst_extra_min_ms{0.0};
    double burst_extra_max_ms{0.0};
};

struct TrackerDelayConfig : public DelayComponentConfig {
    double max_delay_ms{-1.0};
};

struct MeasurementDelayConfig {
    bool enabled{false};
    DelayTimestampPolicy timestamp_policy{DelayTimestampPolicy::SendTime};
    unsigned int seed{1};
    double max_delay_ms{30.0};
    double history_margin_ms{100.0};
    DelayComponentConfig common;
    std::map<std::string, TrackerDelayConfig> trackers;
};

struct TrackerSample {
    double wall_time_s{0.0};
    geometry_msgs::Pose pose;
    tf2::Vector3 linear_velocity{0.0, 0.0, 0.0};
    tf2::Vector3 angular_velocity{0.0, 0.0, 0.0};
    tf2::Vector3 linear_acceleration{0.0, 0.0, 0.0};
    tf2::Vector3 angular_acceleration{0.0, 0.0, 0.0};
    bool have_velocity{false};
    bool have_acceleration{false};
    double report_interval_s{0.0};
};

inline double timestampSecondsForPolicy(DelayTimestampPolicy policy, double send_time_s, const TrackerSample& sample) {
    return policy == DelayTimestampPolicy::SampleTime ? sample.wall_time_s : send_time_s;
}

class TrackerSampleHistory {
  public:
    explicit TrackerSampleHistory(std::size_t capacity = 1) {
        reset(capacity);
    }

    void reset(std::size_t capacity) {
        samples_.assign(std::max<std::size_t>(capacity, 1U), TrackerSample{});
        next_index_ = 0;
        size_ = 0;
    }

    void push(const TrackerSample& sample) {
        if (samples_.empty()) {
            reset(1);
        }
        samples_[next_index_] = sample;
        next_index_ = (next_index_ + 1U) % samples_.size();
        if (size_ < samples_.size()) {
            ++size_;
        }
    }

    const TrackerSample* latest() const {
        if (size_ == 0U) {
            return nullptr;
        }
        const std::size_t index = (next_index_ + samples_.size() - 1U) % samples_.size();
        return &samples_[index];
    }

    const TrackerSample* nearest(double target_time_s) const {
        if (size_ == 0U) {
            return nullptr;
        }

        const TrackerSample* best = nullptr;
        double best_distance = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < size_; ++i) {
            const TrackerSample& sample = samples_[oldestOffsetToIndex(i)];
            const double distance = std::abs(sample.wall_time_s - target_time_s);
            if (distance <= best_distance) {
                best = &sample;
                best_distance = distance;
            }
        }
        return best;
    }

    std::size_t size() const {
        return size_;
    }

    std::size_t capacity() const {
        return samples_.size();
    }

  private:
    std::size_t oldestOffsetToIndex(std::size_t offset) const {
        const std::size_t oldest_index = size_ == samples_.size() ? next_index_ : 0U;
        return (oldest_index + offset) % samples_.size();
    }

    std::vector<TrackerSample> samples_;
    std::size_t next_index_{0};
    std::size_t size_{0};
};

class MeasurementDelay {
  public:
    explicit MeasurementDelay(const MeasurementDelayConfig& config = {}) : config_(config), rng_(seed(config)) {}

    void reset(const MeasurementDelayConfig& config) {
        config_ = config;
        rng_.seed(seed(config_));
        common_state_ = ComponentState{};
        tracker_states_.clear();
        common_cycle_valid_ = false;
        current_common_delay_ms_ = 0.0;
        current_cycle_time_s_ = 0.0;
    }

    const MeasurementDelayConfig& config() const {
        return config_;
    }

    bool enabled() const {
        return config_.enabled;
    }

    DelayTimestampPolicy timestampPolicy() const {
        return config_.timestamp_policy;
    }

    void startPublishCycle(double now_s) {
        if (!config_.enabled) {
            common_cycle_valid_ = true;
            current_cycle_time_s_ = now_s;
            current_common_delay_ms_ = 0.0;
            return;
        }
        current_common_delay_ms_ = sampleComponentForCycle(config_.common, common_state_, now_s);
        current_cycle_time_s_ = now_s;
        common_cycle_valid_ = true;
    }

    double delaySecondsForTracker(const std::string& tracker_name, double now_s) {
        if (!config_.enabled) {
            return 0.0;
        }
        if (!common_cycle_valid_ || current_cycle_time_s_ != now_s) {
            startPublishCycle(now_s);
        }

        ComponentState& state = tracker_states_[tracker_name];
        const double tracker_delay_ms = sampleComponentForCycle(trackerConfig(tracker_name), state, now_s);
        const double unclamped_ms = current_common_delay_ms_ + tracker_delay_ms;
        const double clamped_ms = clamp(unclamped_ms, 0.0, maxDelayMsForTracker(tracker_name));
        return clamped_ms / 1000.0;
    }

    double maxDelayMsForTracker(const std::string& tracker_name) const {
        const auto found = config_.trackers.find(tracker_name);
        if (found != config_.trackers.end() && found->second.max_delay_ms >= 0.0) {
            return found->second.max_delay_ms;
        }
        return config_.max_delay_ms;
    }

    std::size_t historyCapacityForTracker(const std::string& tracker_name, double publish_rate_hz) const {
        if (!config_.enabled) {
            return 4U;
        }
        const double history_window_s =
            (maxDelayMsForTracker(tracker_name) + std::max(config_.history_margin_ms, 0.0)) / 1000.0;
        const double raw_capacity = std::ceil(history_window_s * publish_rate_hz) + 3.0;
        return std::max<std::size_t>(4U, static_cast<std::size_t>(raw_capacity));
    }

    void validate(double publish_rate_hz, std::size_t max_history_samples = 10000U) const {
        if (publish_rate_hz <= 0.0) {
            throw std::runtime_error("publish_rate must be positive");
        }
        if (config_.max_delay_ms < 0.0) {
            throw std::runtime_error("delay.max_delay_ms must be non-negative");
        }
        if (config_.history_margin_ms < 0.0) {
            throw std::runtime_error("delay.history_margin_ms must be non-negative");
        }
        validateComponent(config_.common, "delay.common");
        validateCapacity("delay", config_.max_delay_ms, publish_rate_hz, max_history_samples);
        for (const auto& tracker : config_.trackers) {
            validateComponent(tracker.second, "delay.trackers." + tracker.first);
            if (tracker.second.max_delay_ms < -1.0) {
                throw std::runtime_error("delay.trackers." + tracker.first + ".max_delay_ms must be non-negative");
            }
            validateCapacity("delay.trackers." + tracker.first, maxDelayMsForTracker(tracker.first), publish_rate_hz,
                             max_history_samples);
        }
    }

  private:
    struct ComponentState {
        double slow_ms{0.0};
        double last_time_s{0.0};
        double cycle_time_s{0.0};
        double cycle_delay_ms{0.0};
        bool have_time{false};
        bool have_cycle_delay{false};
    };

    static unsigned int seed(const MeasurementDelayConfig& config) {
        return config.seed == 0U ? 1U : config.seed;
    }

    static double clamp(double value, double low, double high) {
        return std::min(std::max(value, low), high);
    }

    const DelayComponentConfig& trackerConfig(const std::string& tracker_name) const {
        const auto found = config_.trackers.find(tracker_name);
        if (found != config_.trackers.end()) {
            return found->second;
        }
        static const DelayComponentConfig kDefaultTrackerConfig;
        return kDefaultTrackerConfig;
    }

    double sampleComponent(const DelayComponentConfig& config, ComponentState& state, double now_s) {
        double dt_s = 0.0;
        if (state.have_time) {
            dt_s = std::max(0.0, now_s - state.last_time_s);
        }

        if (state.have_time && config.slow_stddev_ms > 0.0 && config.slow_tau_s > 0.0) {
            const double alpha = std::exp(-dt_s / config.slow_tau_s);
            const double sigma = config.slow_stddev_ms * std::sqrt(std::max(0.0, 1.0 - alpha * alpha));
            state.slow_ms = alpha * state.slow_ms + sampleNormal(0.0, sigma);
        }

        state.last_time_s = now_s;
        state.have_time = true;

        double burst_ms = 0.0;
        if (config.burst_probability > 0.0 && sampleUniform(0.0, 1.0) < config.burst_probability) {
            burst_ms = sampleUniform(config.burst_extra_min_ms, config.burst_extra_max_ms);
        }

        return config.base_ms + state.slow_ms + sampleNormal(0.0, config.jitter_stddev_ms) + burst_ms;
    }

    double sampleComponentForCycle(const DelayComponentConfig& config, ComponentState& state, double now_s) {
        if (state.have_cycle_delay && state.cycle_time_s == now_s) {
            return state.cycle_delay_ms;
        }

        state.cycle_delay_ms = sampleComponent(config, state, now_s);
        state.cycle_time_s = now_s;
        state.have_cycle_delay = true;
        return state.cycle_delay_ms;
    }

    double sampleNormal(double mean, double stddev) {
        if (stddev <= 0.0) {
            return mean;
        }
        std::normal_distribution<double> distribution(mean, stddev);
        return distribution(rng_);
    }

    double sampleUniform(double low, double high) {
        if (high <= low) {
            return low;
        }
        std::uniform_real_distribution<double> distribution(low, high);
        return distribution(rng_);
    }

    static void validateComponent(const DelayComponentConfig& config, const std::string& name) {
        if (config.slow_stddev_ms < 0.0) {
            throw std::runtime_error(name + ".slow_stddev_ms must be non-negative");
        }
        if (config.slow_tau_s <= 0.0) {
            throw std::runtime_error(name + ".slow_tau_s must be positive");
        }
        if (config.jitter_stddev_ms < 0.0) {
            throw std::runtime_error(name + ".jitter_stddev_ms must be non-negative");
        }
        if (config.burst_probability < 0.0 || config.burst_probability > 1.0) {
            throw std::runtime_error(name + ".burst_probability must be in [0, 1]");
        }
        if (config.burst_extra_min_ms < 0.0 || config.burst_extra_max_ms < 0.0 ||
            config.burst_extra_max_ms < config.burst_extra_min_ms) {
            throw std::runtime_error(name + ".burst_extra_ms must be a non-negative [min, max] range");
        }
    }

    void validateCapacity(const std::string& name, double max_delay_ms, double publish_rate_hz,
                          std::size_t max_history_samples) const {
        if (max_delay_ms < 0.0) {
            throw std::runtime_error(name + ".max_delay_ms must be non-negative");
        }
        MeasurementDelayConfig copy = config_;
        copy.enabled = true;
        copy.max_delay_ms = max_delay_ms;
        MeasurementDelay model(copy);
        if (model.historyCapacityForTracker("", publish_rate_hz) > max_history_samples) {
            throw std::runtime_error(name + " history capacity exceeds the hard limit");
        }
    }

    MeasurementDelayConfig config_;
    std::mt19937 rng_;
    ComponentState common_state_;
    std::map<std::string, ComponentState> tracker_states_;
    bool common_cycle_valid_{false};
    double current_cycle_time_s_{0.0};
    double current_common_delay_ms_{0.0};
};

} // namespace gazebo_sim_vrpn_bridge
