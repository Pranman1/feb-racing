#pragma once
// Glue between the team's cone ordering (src/algorithms, unchanged) and the simulator's
// messages: cones as a PoseArray with the colour in orientation.w, the car as a Pose.
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include "algorithms/util.h"
#include "algorithms/cone_ordering.h"

namespace ordering_io {
constexpr int BLUE = 1, YELLOW = 2;

inline double yaw_of(const geometry_msgs::msg::Quaternion &q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// Fill the ordering state from the car pose and the coloured map, run the ordering.
inline void run(ConeOrderingState &state, const geometry_msgs::msg::Pose &car, const geometry_msgs::msg::PoseArray &map) {
    state = ConeOrderingState{};
    state.car = {{car.position.x, car.position.y}, yaw_of(car.orientation)};
    for (const auto &p : map.poses) {
        const int colour = static_cast<int>(std::lround(p.orientation.w));
        if (colour == BLUE) state.bluePoints.push_back({p.position.x, p.position.y});
        else if (colour == YELLOW) state.redPoints.push_back({p.position.x, p.position.y});   // the team calls the right side red
    }
    cone_ordering::update(state);
}

inline geometry_msgs::msg::PoseArray to_array(const std::vector<Point> &pts, const std::string &frame) {
    geometry_msgs::msg::PoseArray arr;
    arr.header.frame_id = frame;
    for (const auto &p : pts) {
        geometry_msgs::msg::Pose q;
        q.position.x = p.x;
        q.position.y = p.y;
        q.orientation.w = 1.0;
        arr.poses.push_back(q);
    }
    return arr;
}
}  // namespace ordering_io
