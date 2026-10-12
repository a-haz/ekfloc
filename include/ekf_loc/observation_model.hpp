#pragma once

#include "ekf_loc/types.hpp"

#include <Eigen/Core>

namespace ekf_loc {

/// Predicted range and bearing of a known landmark from a pose.
/// Bearing is wrapped to [-pi, pi).
Eigen::Vector2d predict_measurement(const Pose& pose, const Landmark& landmark);

/// dh/d[x, y, theta] of predict_measurement.
/// The landmark must not lie on the pose; the bearing model is singular there.
Eigen::Matrix<double, 2, 3> measurement_jacobian(const Pose& pose, const Landmark& landmark);

}  // namespace ekf_loc
