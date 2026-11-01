#include "ekf_loc/metrics.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <stdexcept>
#include <vector>

namespace {

using ekf_loc::kPi;
using ekf_loc::kRadToDeg;
using ekf_loc::Pose;
using ekf_loc::TrajectorySample;

TEST(MetricsNees, DiagonalCovariance) {
    // e = (0.5, 1.0, 0.2), P = diag(0.25, 4, 0.01).
    // NEES = 0.25 / 0.25 + 1 / 4 + 0.04 / 0.01 = 1 + 0.25 + 4 = 5.25.
    const Pose truth{1.0, 2.0, 0.3};
    const Pose estimate{0.5, 1.0, 0.1};
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    covariance.diagonal() << 0.25, 4.0, 0.01;
    EXPECT_NEAR(ekf_loc::normalized_estimation_error_squared(truth, estimate, covariance), 5.25,
                1e-12);
}

TEST(MetricsNees, CorrelatedCovariance) {
    // e = (1, 1, 0). The xy block of P is [[2, 1], [1, 2]], whose inverse is
    // (1/3) [[2, -1], [-1, 2]], so NEES = (2 - 1 - 1 + 2) / 3 = 2/3.
    const Pose truth{1.0, 1.0, 0.0};
    const Pose estimate{0.0, 0.0, 0.0};
    Eigen::Matrix3d covariance;
    covariance << 2.0, 1.0, 0.0,
                  1.0, 2.0, 0.0,
                  0.0, 0.0, 1.0;
    EXPECT_NEAR(ekf_loc::normalized_estimation_error_squared(truth, estimate, covariance),
                2.0 / 3.0, 1e-12);
}

TEST(MetricsNees, UsesWrappedHeadingError) {
    // Raw heading difference is (pi - 0.05) - (-pi + 0.05) = 2 pi - 0.1, which wraps to -0.1.
    // With P = 0.01 I the NEES is 0.1^2 / 0.01 = 1, not about 3900 as it would be unwrapped.
    const Pose truth{0.0, 0.0, kPi - 0.05};
    const Pose estimate{0.0, 0.0, -kPi + 0.05};
    const Eigen::Matrix3d covariance = 0.01 * Eigen::Matrix3d::Identity();
    EXPECT_NEAR(ekf_loc::normalized_estimation_error_squared(truth, estimate, covariance), 1.0,
                1e-9);
}

// Two samples where only the EKF estimate is off. Dead reckoning matches truth exactly.
//   sample 0: EKF position error (3, 4), heading error 0.1 rad, nees_ekf 2.
//   sample 1: EKF position error (5, 0), heading error wraps to -0.7 rad, nees_ekf 4.
std::vector<TrajectorySample> two_samples() {
    TrajectorySample first;
    first.time_s = 0.0;
    first.truth = Pose{3.0, 4.0, 0.2};
    first.ekf = Pose{0.0, 0.0, 0.1};
    first.dead_reckoning = first.truth;
    first.nees_ekf = 2.0;
    first.nees_dead_reckoning = 1.0;

    TrajectorySample second;
    second.time_s = 0.1;
    second.truth = Pose{6.0, 1.0, kPi - 0.3};
    second.ekf = Pose{1.0, 1.0, -kPi + 0.4};
    second.dead_reckoning = second.truth;
    second.nees_ekf = 4.0;
    second.nees_dead_reckoning = 1.0;
    return {first, second};
}

TEST(MetricsScore, EkfScoresMatchHandComputedValues) {
    const ekf_loc::MethodScore score = ekf_loc::score_ekf(two_samples());
    // Position: (9 + 16 + 25 + 0) / 2 = 25, so RMSE = 5 m.
    EXPECT_NEAR(score.rmse_position_m, 5.0, 1e-12);
    // Heading: (0.1^2 + 0.7^2) / 2 = 0.25, so RMSE = 0.5 rad.
    EXPECT_NEAR(score.rmse_heading_deg, 0.5 * kRadToDeg, 1e-9);
    // Final sample: position error (5, 0) has norm 5, and |-0.7| rad is reported in degrees.
    EXPECT_NEAR(score.final_position_error_m, 5.0, 1e-12);
    EXPECT_NEAR(score.final_heading_error_deg, 0.7 * kRadToDeg, 1e-9);
    // Mean NEES: (2 + 4) / 2 = 3.
    EXPECT_NEAR(score.mean_nees, 3.0, 1e-12);
}

TEST(MetricsScore, DeadReckoningUsesItsOwnFields) {
    // Dead reckoning equals truth in both samples, so every error is zero,
    // and its NEES is 1 in both samples, so the mean is 1.
    const ekf_loc::MethodScore score = ekf_loc::score_dead_reckoning(two_samples());
    EXPECT_NEAR(score.rmse_position_m, 0.0, 1e-12);
    EXPECT_NEAR(score.rmse_heading_deg, 0.0, 1e-12);
    EXPECT_NEAR(score.final_position_error_m, 0.0, 1e-12);
    EXPECT_NEAR(score.final_heading_error_deg, 0.0, 1e-12);
    EXPECT_NEAR(score.mean_nees, 1.0, 1e-12);
}

TEST(MetricsScore, RejectsAnEmptyTrajectory) {
    const std::vector<TrajectorySample> empty;
    EXPECT_THROW(ekf_loc::score_ekf(empty), std::invalid_argument);
    EXPECT_THROW(ekf_loc::score_dead_reckoning(empty), std::invalid_argument);
}

}  // namespace
