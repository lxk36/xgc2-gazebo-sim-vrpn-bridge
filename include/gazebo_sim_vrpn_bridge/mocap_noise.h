#pragma once

#include <array>
#include <random>

#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

namespace gazebo_sim_vrpn_bridge {

struct MocapNoiseConfig {
    bool enabled{true};
    std::array<double, 3> position_stddev_m{{1.0e-7, 1.0e-7, 1.0e-7}};
    std::array<double, 3> rotation_stddev_rad{{0.0, 0.0, 0.0}};
    unsigned int seed{1};
};

class MocapNoise {
  public:
    explicit MocapNoise(const MocapNoiseConfig& config = {})
        : config_(config), rng_(config.seed == 0 ? 1 : config.seed) {}

    geometry_msgs::Pose apply(const geometry_msgs::Pose& pose) {
        if (!config_.enabled) {
            return pose;
        }

        geometry_msgs::Pose noisy = pose;
        noisy.position.x += sample(config_.position_stddev_m[0]);
        noisy.position.y += sample(config_.position_stddev_m[1]);
        noisy.position.z += sample(config_.position_stddev_m[2]);

        const double roll_noise = sample(config_.rotation_stddev_rad[0]);
        const double pitch_noise = sample(config_.rotation_stddev_rad[1]);
        const double yaw_noise = sample(config_.rotation_stddev_rad[2]);
        if (roll_noise != 0.0 || pitch_noise != 0.0 || yaw_noise != 0.0) {
            tf2::Quaternion base(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
            base.normalize();

            tf2::Quaternion noise;
            noise.setRPY(roll_noise, pitch_noise, yaw_noise);
            noise.normalize();

            const tf2::Quaternion out = base * noise;
            noisy.orientation.x = out.x();
            noisy.orientation.y = out.y();
            noisy.orientation.z = out.z();
            noisy.orientation.w = out.w();
        }

        return noisy;
    }

  private:
    double sample(double stddev) {
        if (stddev <= 0.0) {
            return 0.0;
        }
        std::normal_distribution<double> distribution(0.0, stddev);
        return distribution(rng_);
    }

    MocapNoiseConfig config_;
    std::mt19937 rng_;
};

} // namespace gazebo_sim_vrpn_bridge
