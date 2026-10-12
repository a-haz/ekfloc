#pragma once

#include "ekf_loc/types.hpp"

#include <Eigen/Core>

namespace ekf_loc {

/// Propagate a pose through the midpoint unicycle model for one step.
///
/// theta_mid = theta + omega * dt / 2
/// x' = x + v * dt * cos(theta_mid)
/// y' = y + v * dt * sin(theta_mid)
/// theta' = wrap(theta + omega * dt)
Pose propagate(const Pose& pose, const Control& control, double dt);

/// df/d[x, y, theta] of propagate, with heading treated on the circle (derivative of wrap is 1).
Eigen::Matrix3d state_jacobian(const Pose& pose, const Control& control, double dt);

/// df/d[v, omega] of propagate.
Eigen::Matrix<double, 3, 2> control_jacobian(const Pose& pose, const Control& control, double dt);

/// Discrete covariance of a velocity sample held over dt.
///
/// Linear velocity and yaw rate are continuous-time white noise with the given
/// power spectral densities. The average of that noise over a step of length dt
/// has covariance diag(q_v / dt, q_omega / dt).
Eigen::Matrix2d discrete_control_covariance(double velocity_density, double yaw_rate_density, double dt);

}  // namespace ekf_loc
