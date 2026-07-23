#pragma once

#include <algorithm>
#include <cctype>
#include <string>

namespace gazebo_sim_vrpn_bridge {

inline bool isNumberedRobotModelName(const std::string& value, const std::string& prefix) {
    if (value.size() <= prefix.size() || value.compare(0, prefix.size(), prefix) != 0) {
        return false;
    }

    const auto suffix_begin = value.begin() + static_cast<std::string::difference_type>(prefix.size());
    return std::all_of(suffix_begin, value.end(), [](unsigned char ch) {
        return std::isdigit(ch);
    });
}

inline bool isCanonicalAutoTrackedModelName(const std::string& value) {
    return isNumberedRobotModelName(value, "uav") || isNumberedRobotModelName(value, "ugv") ||
           isNumberedRobotModelName(value, "mecanum");
}

} // namespace gazebo_sim_vrpn_bridge
