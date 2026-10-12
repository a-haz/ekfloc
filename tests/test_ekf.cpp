#include "ekf_loc/ekf.hpp"
#include "ekf_loc/observation_model.hpp"

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include <cmath>

namespace {

ekf_loc::NoiseParameters test_noise() {
    ekf_loc::NoiseParameters noise;
    noise.velocity_density = 0.02;
    noise.yaw_rate_density = 0.004;
    noise.sigma_range = 0.08;
    noise.sigma_bearing = 2.0 * ekf_loc::kDegToRad;
    return noise;
}

bool is_symmetric_positive_definite(const Eigen::Matrix3d& covariance) {
    if ((covariance - covariance.transpose()).norm() > 1e-10) {
        return false;
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    return solver.info() == Eigen::Success && solver.eigenvalues().minCoeff() > 0.0;
}

TEST(ExtendedKalmanFilter, AlongTrackVarianceMatchesVelocityDensity) {
    ekf_loc::NoiseParameters noise = test_noise();
    noise.yaw_rate_density = 0.0;
    const Eigen::Matrix3d initial = Eigen::Matrix3d::Identity() * 1e-12;
    ekf_loc::ExtendedKalmanFilter filter(ekf_loc::Pose{}, initial, noise);
    const double dt = 0.05;
    const int steps = 200;
    for (int step = 0; step < steps; ++step) {
        filter.predict(ekf_loc::Control{0.8, 0.0}, dt);
    }
    EXPECT_NEAR(filter.covariance()(0, 0), noise.velocity_density * steps * dt, 1e-9);
    EXPECT_NEAR(filter.covariance()(1, 1), 1e-12, 1e-9);
    EXPECT_NEAR(filter.covariance()(2, 2), 1e-12, 1e-9);
}

TEST(ExtendedKalmanFilter, HeadingVarianceMatchesYawRateDensity) {
    ekf_loc::NoiseParameters noise = test_noise();
    noise.velocity_density = 0.0;
    const Eigen::Matrix3d initial = Eigen::Matrix3d::Identity() * 1e-12;
    ekf_loc::ExtendedKalmanFilter filter(ekf_loc::Pose{}, initial, noise);
    const double dt = 0.05;
    const int steps = 200;
    for (int step = 0; step < steps; ++step) {
        filter.predict(ekf_loc::Control{0.0, 0.0}, dt);
    }
    EXPECT_NEAR(filter.covariance()(2, 2), noise.yaw_rate_density * steps * dt, 1e-9);
    EXPECT_NEAR(filter.covariance()(0, 0), 1e-12, 1e-9);
    EXPECT_NEAR(filter.covariance()(1, 1), 1e-12, 1e-9);
}

TEST(ExtendedKalmanFilter, PredictRejectsNonPositiveDt) {
    ekf_loc::ExtendedKalmanFilter filter(ekf_loc::Pose{}, Eigen::Matrix3d::Identity(), test_noise());
    EXPECT_THROW(filter.predict(ekf_loc::Control{0.2, 0.0}, 0.0), std::invalid_argument);
}

TEST(ExtendedKalmanFilter, CovarianceStaysSymmetricAndPositive) {
    ekf_loc::ExtendedKalmanFilter filter(ekf_loc::Pose{}, Eigen::Matrix3d::Identity() * 0.05, test_noise());
    const ekf_loc::Landmark landmark{0, 3.0, 1.0};
    for (int step = 0; step < 40; ++step) {
        filter.predict(ekf_loc::Control{0.4, 0.2}, 0.05);
        const Eigen::Vector2d predicted = ekf_loc::predict_measurement(filter.state(), landmark);
        ekf_loc::RangeBearing measurement;
        measurement.landmark_id = landmark.id;
        measurement.range = predicted(0);
        measurement.bearing = predicted(1);
        const double nis = filter.update(measurement, landmark);
        EXPECT_GE(nis, 0.0);
        EXPECT_TRUE(is_symmetric_positive_definite(filter.covariance()));
    }
}

TEST(ExtendedKalmanFilter, BearingInnovationWrapsAcrossBranchCut) {
    ekf_loc::NoiseParameters noise = test_noise();
    noise.sigma_range = 0.05;
    noise.sigma_bearing = 0.05;
    ekf_loc::ExtendedKalmanFilter filter(ekf_loc::Pose{}, Eigen::Matrix3d::Identity() * 0.05, noise);
    const ekf_loc::Landmark landmark{0, -2.0, 0.0};
    ekf_loc::RangeBearing measurement;
    measurement.landmark_id = landmark.id;
    measurement.range = 2.0;
    // Predicted bearing is -pi. This measurement sits just on the other side of the cut.
    measurement.bearing = ekf_loc::kPi - 0.02;
    filter.update(measurement, landmark);
    EXPECT_LT(std::abs(filter.state().theta), 0.1);
    EXPECT_LT(std::abs(filter.state().y), 0.1);
    EXPECT_NEAR(filter.state().x, 0.0, 0.05);
}

TEST(ExtendedKalmanFilter, UpdateRejectsMismatchedLandmarkId) {
    ekf_loc::ExtendedKalmanFilter filter(ekf_loc::Pose{}, Eigen::Matrix3d::Identity(), test_noise());
    ekf_loc::RangeBearing measurement;
    measurement.landmark_id = 4;
    measurement.range = 3.0;
    measurement.bearing = 0.1;
    EXPECT_THROW(filter.update(measurement, ekf_loc::Landmark{1, 3.0, 0.0}), std::invalid_argument);
}

TEST(ExtendedKalmanFilter, ZeroInnovationLeavesTheMean) {
    const ekf_loc::Pose pose{0.2, -0.4, 0.3};
    ekf_loc::ExtendedKalmanFilter filter(pose, Eigen::Matrix3d::Identity() * 0.2, test_noise());
    const ekf_loc::Landmark landmark{2, 4.2, -0.4};
    const Eigen::Vector2d predicted = ekf_loc::predict_measurement(pose, landmark);
    ekf_loc::RangeBearing measurement;
    measurement.landmark_id = landmark.id;
    measurement.range = predicted(0);
    measurement.bearing = predicted(1);
    const double nis = filter.update(measurement, landmark);
    EXPECT_NEAR(nis, 0.0, 1e-12);
    EXPECT_NEAR(filter.state().x, pose.x, 1e-12);
    EXPECT_NEAR(filter.state().y, pose.y, 1e-12);
    EXPECT_NEAR(filter.state().theta, pose.theta, 1e-12);
}

TEST(ExtendedKalmanFilter, RepeatedPerfectMeasurementsConverge) {
    ekf_loc::NoiseParameters noise = test_noise();
    noise.sigma_range = 0.02;
    noise.sigma_bearing = 0.01;
    const ekf_loc::Pose truth{};
    const ekf_loc::Pose initial{0.5, -0.4, 0.3};
    ekf_loc::ExtendedKalmanFilter filter(initial, Eigen::Matrix3d::Identity(), noise);
    const ekf_loc::Landmark landmarks[] = {
        ekf_loc::Landmark{0, 3.0, 1.0},
        ekf_loc::Landmark{1, -2.0, 2.5},
        ekf_loc::Landmark{2, 1.0, -3.0},
        ekf_loc::Landmark{3, -3.0, -1.0},
    };
    for (int pass = 0; pass < 12; ++pass) {
        for (const ekf_loc::Landmark& landmark : landmarks) {
            const Eigen::Vector2d predicted = ekf_loc::predict_measurement(truth, landmark);
            ekf_loc::RangeBearing measurement;
            measurement.landmark_id = landmark.id;
            measurement.range = predicted(0);
            measurement.bearing = predicted(1);
            filter.update(measurement, landmark);
        }
    }
    EXPECT_NEAR(filter.state().x, truth.x, 0.02);
    EXPECT_NEAR(filter.state().y, truth.y, 0.02);
    EXPECT_NEAR(filter.state().theta, truth.theta, 0.02);
    EXPECT_TRUE(is_symmetric_positive_definite(filter.covariance()));
}

TEST(ExtendedKalmanFilter, RejectsIndefiniteCovariance) {
    EXPECT_THROW(ekf_loc::ExtendedKalmanFilter(ekf_loc::Pose{}, Eigen::Matrix3d::Zero(), test_noise()),
                 std::invalid_argument);
    EXPECT_THROW(ekf_loc::ExtendedKalmanFilter(ekf_loc::Pose{}, -Eigen::Matrix3d::Identity(), test_noise()),
                 std::invalid_argument);
}

}  // namespace
