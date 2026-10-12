#include "ekf_loc/observation_model.hpp"

#include <cmath>
#include <stdexcept>

namespace ekf_loc {
namespace {

struct Offset {
    double dx;
    double dy;
    double range_sq;
    double range;
};

Offset landmark_offset(const Pose& pose, const Landmark& landmark) {
    Offset offset;
    offset.dx = landmark.x - pose.x;
    offset.dy = landmark.y - pose.y;
    offset.range_sq = offset.dx * offset.dx + offset.dy * offset.dy;
    offset.range = std::sqrt(offset.range_sq);
    if (!(offset.range > 1e-9)) {
        throw std::invalid_argument("landmark is coincident with the pose");
    }
    return offset;
}

}  // namespace

Eigen::Vector2d predict_measurement(const Pose& pose, const Landmark& landmark) {
    const Offset offset = landmark_offset(pose, landmark);
    Eigen::Vector2d measurement;
    measurement << offset.range, wrap_angle(std::atan2(offset.dy, offset.dx) - pose.theta);
    return measurement;
}

Eigen::Matrix<double, 2, 3> measurement_jacobian(const Pose& pose, const Landmark& landmark) {
    const Offset offset = landmark_offset(pose, landmark);
    Eigen::Matrix<double, 2, 3> jacobian;
    jacobian << -offset.dx / offset.range, -offset.dy / offset.range, 0.0, offset.dy / offset.range_sq,
        -offset.dx / offset.range_sq, -1.0;
    return jacobian;
}

}  // namespace ekf_loc
