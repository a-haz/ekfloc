#pragma once

#include "ekf_loc/types.hpp"

#include <Eigen/Core>
#include <vector>

namespace ekf_loc {

/// Tangent-space pose error. The heading component is wrapped to [-pi, pi).
Eigen::Vector3d pose_error(const Pose& truth, const Pose& estimate);

/// Normalized estimation error squared, e^T P^{-1} e, with wrapped heading error.
double normalized_estimation_error_squared(const Pose& truth, const Pose& estimate,
                                           const Eigen::Matrix3d& covariance);

/// One recorded simulator step.
struct TrajectorySample {
    double time_s{0.0};
    Pose truth;
    Pose dead_reckoning;
    Pose ekf;
    Eigen::Matrix3d covariance_dead_reckoning = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d covariance_ekf = Eigen::Matrix3d::Identity();
    double nees_dead_reckoning{0.0};
    double nees_ekf{0.0};
};

/// Scalar scores for one estimator over a trajectory.
struct MethodScore {
    double rmse_position_m{0.0};
    double rmse_heading_deg{0.0};
    double final_position_error_m{0.0};
    double final_heading_error_deg{0.0};
    double mean_nees{0.0};
};

MethodScore score_dead_reckoning(const std::vector<TrajectorySample>& samples);
MethodScore score_ekf(const std::vector<TrajectorySample>& samples);

}  // namespace ekf_loc
