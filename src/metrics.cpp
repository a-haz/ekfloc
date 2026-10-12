#include "ekf_loc/metrics.hpp"

#include <Eigen/Cholesky>
#include <cmath>
#include <stdexcept>

namespace ekf_loc {
namespace {

MethodScore score_samples(const std::vector<TrajectorySample>& samples, bool use_ekf) {
    if (samples.empty()) {
        throw std::invalid_argument("cannot score an empty trajectory");
    }
    double position_sq = 0.0;
    double heading_sq = 0.0;
    double nees_sum = 0.0;
    for (const TrajectorySample& sample : samples) {
        const Pose& estimate = use_ekf ? sample.ekf : sample.dead_reckoning;
        const Eigen::Vector3d error = pose_error(sample.truth, estimate);
        position_sq += error(0) * error(0) + error(1) * error(1);
        heading_sq += error(2) * error(2);
        nees_sum += use_ekf ? sample.nees_ekf : sample.nees_dead_reckoning;
    }
    const double count = static_cast<double>(samples.size());
    const Eigen::Vector3d final_error =
        pose_error(samples.back().truth, use_ekf ? samples.back().ekf : samples.back().dead_reckoning);
    MethodScore score;
    score.rmse_position_m = std::sqrt(position_sq / count);
    score.rmse_heading_deg = std::sqrt(heading_sq / count) * kRadToDeg;
    score.final_position_error_m = std::hypot(final_error(0), final_error(1));
    score.final_heading_error_deg = std::abs(final_error(2)) * kRadToDeg;
    score.mean_nees = nees_sum / count;
    return score;
}

}  // namespace

Eigen::Vector3d pose_error(const Pose& truth, const Pose& estimate) {
    return Eigen::Vector3d(truth.x - estimate.x, truth.y - estimate.y,
                           wrap_angle(truth.theta - estimate.theta));
}

double normalized_estimation_error_squared(const Pose& truth, const Pose& estimate,
                                           const Eigen::Matrix3d& covariance) {
    const Eigen::Vector3d error = pose_error(truth, estimate);
    Eigen::LDLT<Eigen::Matrix3d> ldlt(covariance);
    if (ldlt.info() != Eigen::Success) {
        throw std::runtime_error("covariance is not positive definite");
    }
    return error.dot(ldlt.solve(error));
}

MethodScore score_dead_reckoning(const std::vector<TrajectorySample>& samples) {
    return score_samples(samples, false);
}

MethodScore score_ekf(const std::vector<TrajectorySample>& samples) {
    return score_samples(samples, true);
}

}  // namespace ekf_loc
