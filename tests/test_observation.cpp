#include "ekf_loc/observation_model.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

TEST(ObservationModel, LandmarkAhead) {
    const ekf_loc::Landmark landmark{0, 3.0, 0.0};
    const Eigen::Vector2d measurement = ekf_loc::predict_measurement(ekf_loc::Pose{}, landmark);
    EXPECT_NEAR(measurement(0), 3.0, 1e-12);
    EXPECT_NEAR(measurement(1), 0.0, 1e-12);
}

TEST(ObservationModel, LandmarkToTheLeft) {
    const ekf_loc::Landmark landmark{1, 0.0, 4.0};
    const Eigen::Vector2d measurement = ekf_loc::predict_measurement(ekf_loc::Pose{}, landmark);
    EXPECT_NEAR(measurement(0), 4.0, 1e-12);
    EXPECT_NEAR(measurement(1), ekf_loc::kPi / 2.0, 1e-12);
}

TEST(ObservationModel, LandmarkBehind) {
    const ekf_loc::Landmark landmark{2, -2.0, 0.0};
    const Eigen::Vector2d measurement = ekf_loc::predict_measurement(ekf_loc::Pose{}, landmark);
    EXPECT_NEAR(measurement(0), 2.0, 1e-12);
    EXPECT_NEAR(measurement(1), -ekf_loc::kPi, 1e-12);
}

TEST(ObservationModel, LandmarkToTheRight) {
    const ekf_loc::Landmark landmark{3, 0.0, -3.0};
    const Eigen::Vector2d measurement = ekf_loc::predict_measurement(ekf_loc::Pose{}, landmark);
    EXPECT_NEAR(measurement(0), 3.0, 1e-12);
    EXPECT_NEAR(measurement(1), -ekf_loc::kPi / 2.0, 1e-12);
}

TEST(ObservationModel, HeadingOffsetsBearing) {
    ekf_loc::Pose pose;
    pose.theta = ekf_loc::kPi / 2.0;
    const Eigen::Vector2d ahead = ekf_loc::predict_measurement(pose, ekf_loc::Landmark{0, 0.0, 5.0});
    const Eigen::Vector2d right = ekf_loc::predict_measurement(pose, ekf_loc::Landmark{1, 5.0, 0.0});
    EXPECT_NEAR(ahead(0), 5.0, 1e-12);
    EXPECT_NEAR(ahead(1), 0.0, 1e-12);
    EXPECT_NEAR(right(0), 5.0, 1e-12);
    EXPECT_NEAR(right(1), -ekf_loc::kPi / 2.0, 1e-12);
}

TEST(ObservationModel, ClosedFormJacobian) {
    const ekf_loc::Landmark landmark{0, 3.0, 4.0};
    const Eigen::Vector2d measurement = ekf_loc::predict_measurement(ekf_loc::Pose{}, landmark);
    EXPECT_NEAR(measurement(0), 5.0, 1e-12);
    EXPECT_NEAR(measurement(1), std::atan2(4.0, 3.0), 1e-12);
    const Eigen::Matrix<double, 2, 3> jacobian = ekf_loc::measurement_jacobian(ekf_loc::Pose{}, landmark);
    Eigen::Matrix<double, 2, 3> expected;
    expected << -0.6, -0.8, 0.0, 0.16, -0.12, -1.0;
    EXPECT_LT((jacobian - expected).norm(), 1e-12);
}

TEST(ObservationModel, RejectsCoincidentLandmark) {
    EXPECT_THROW(ekf_loc::predict_measurement(ekf_loc::Pose{}, ekf_loc::Landmark{0, 0.0, 0.0}),
                 std::invalid_argument);
    EXPECT_THROW(ekf_loc::measurement_jacobian(ekf_loc::Pose{1.0, 2.0, 0.0}, ekf_loc::Landmark{1, 1.0, 2.0}),
                 std::invalid_argument);
}

}  // namespace
