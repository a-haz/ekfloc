#pragma once

#include "ekf_loc/types.hpp"

#include <Eigen/Core>

namespace ekf_loc {

/// Planar extended Kalman filter for a differential-drive pose.
///
/// Predict integrates a wheel-odometry velocity and inflates the covariance
/// through the control Jacobian. Update fuses one range-bearing observation of
/// a known landmark. Bearing innovations are wrapped to [-pi, pi).
class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter(const Pose& initial_state, const Eigen::Matrix3d& initial_covariance,
                         NoiseParameters noise);

    /// Integrate odometry. dt must be positive.
    void predict(const Control& odometry, double dt);

    /// Fuse one associated landmark observation. Returns the normalized innovation squared.
    double update(const RangeBearing& measurement, const Landmark& landmark);

    const Pose& state() const { return state_; }
    const Eigen::Matrix3d& covariance() const { return covariance_; }
    const NoiseParameters& noise() const { return noise_; }

private:
    Pose state_;
    Eigen::Matrix3d covariance_;
    NoiseParameters noise_;
};

}  // namespace ekf_loc
