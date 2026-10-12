#include "ekf_loc/motion_model.hpp"
#include "ekf_loc/scenario.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

ekf_loc::Pose integrate(const ekf_loc::Pose& start, const ekf_loc::Control& control, double dt, int steps) {
    ekf_loc::Pose pose = start;
    for (int step = 0; step < steps; ++step) {
        pose = ekf_loc::propagate(pose, control, dt);
    }
    return pose;
}

TEST(MotionModel, StraightEast) {
    const ekf_loc::Pose next = ekf_loc::propagate(ekf_loc::Pose{}, ekf_loc::Control{1.0, 0.0}, 2.0);
    EXPECT_NEAR(next.x, 2.0, 1e-12);
    EXPECT_NEAR(next.y, 0.0, 1e-12);
    EXPECT_NEAR(next.theta, 0.0, 1e-12);
}

TEST(MotionModel, StraightNorth) {
    ekf_loc::Pose start;
    start.theta = ekf_loc::kPi / 2.0;
    const ekf_loc::Pose next = ekf_loc::propagate(start, ekf_loc::Control{1.5, 0.0}, 2.0);
    EXPECT_NEAR(next.x, 0.0, 1e-12);
    EXPECT_NEAR(next.y, 3.0, 1e-12);
    EXPECT_NEAR(next.theta, ekf_loc::kPi / 2.0, 1e-12);
}

TEST(MotionModel, StraightSouth) {
    ekf_loc::Pose start;
    start.x = 1.0;
    start.y = 4.0;
    start.theta = -ekf_loc::kPi / 2.0;
    const ekf_loc::Pose next = ekf_loc::propagate(start, ekf_loc::Control{0.5, 0.0}, 2.0);
    EXPECT_NEAR(next.x, 1.0, 1e-12);
    EXPECT_NEAR(next.y, 3.0, 1e-12);
    EXPECT_NEAR(next.theta, -ekf_loc::kPi / 2.0, 1e-12);
}

TEST(MotionModel, InPlaceRotation) {
    const ekf_loc::Pose next = ekf_loc::propagate(ekf_loc::Pose{1.0, -2.0, 0.2}, ekf_loc::Control{0.0, ekf_loc::kPi / 2.0}, 1.0);
    EXPECT_NEAR(next.x, 1.0, 1e-12);
    EXPECT_NEAR(next.y, -2.0, 1e-12);
    EXPECT_NEAR(next.theta, 0.2 + ekf_loc::kPi / 2.0, 1e-12);
}

TEST(MotionModel, InPlaceRotationWraps) {
    const ekf_loc::Pose next =
        ekf_loc::propagate(ekf_loc::Pose{}, ekf_loc::Control{0.0, ekf_loc::kPi}, 1.0);
    EXPECT_NEAR(next.x, 0.0, 1e-12);
    EXPECT_NEAR(next.y, 0.0, 1e-12);
    EXPECT_NEAR(next.theta, -ekf_loc::kPi, 1e-12);
}

TEST(MotionModel, ClosedFormStraightJacobian) {
    const ekf_loc::Pose pose{1.0, 2.0, 0.0};
    const ekf_loc::Control control{0.5, 0.0};
    const double dt = 2.0;
    const Eigen::Matrix3d state = ekf_loc::state_jacobian(pose, control, dt);
    Eigen::Matrix3d expected_state = Eigen::Matrix3d::Identity();
    expected_state(1, 2) = 1.0;
    const Eigen::Matrix<double, 3, 2> control_jac = ekf_loc::control_jacobian(pose, control, dt);
    Eigen::Matrix<double, 3, 2> expected_control;
    expected_control << 2.0, 0.0, 0.0, 1.0, 0.0, 2.0;
    EXPECT_LT((state - expected_state).norm(), 1e-12);
    EXPECT_LT((control_jac - expected_control).norm(), 1e-12);
}

TEST(MotionModel, FineStepsMatchACircle) {
    // dt divides one revolution so the integrated yaw is exactly one turn.
    const int steps = 4000;
    const double dt = (2.0 * ekf_loc::kPi) / static_cast<double>(steps);
    const ekf_loc::Pose end = integrate(ekf_loc::Pose{}, ekf_loc::Control{1.0, 1.0}, dt, steps);
    EXPECT_NEAR(end.x, 0.0, 1e-4);
    EXPECT_NEAR(end.y, 0.0, 1e-4);
    EXPECT_NEAR(end.theta, 0.0, 1e-8);
}

TEST(MotionModel, CourseClosesAfterOneLap) {
    ekf_loc::Pose pose;
    const double dt = 0.05;
    const int steps = static_cast<int>(std::llround(ekf_loc::kCoursePeriodS / dt));
    for (int step = 0; step < steps; ++step) {
        pose = ekf_loc::propagate(pose, ekf_loc::rounded_square_command(step * dt), dt);
    }
    EXPECT_NEAR(pose.x, 0.0, 1e-9);
    EXPECT_NEAR(pose.y, 0.0, 1e-9);
    EXPECT_NEAR(pose.theta, 0.0, 1e-9);
}

TEST(MotionModel, CourseClosesAfterTwoLaps) {
    ekf_loc::Pose pose;
    const double dt = 0.05;
    const int steps = static_cast<int>(std::llround((2.0 * ekf_loc::kCoursePeriodS) / dt));
    for (int step = 0; step < steps; ++step) {
        pose = ekf_loc::propagate(pose, ekf_loc::rounded_square_command(step * dt), dt);
    }
    EXPECT_NEAR(pose.x, 0.0, 1e-9);
    EXPECT_NEAR(pose.y, 0.0, 1e-9);
    EXPECT_NEAR(pose.theta, 0.0, 1e-9);
}

TEST(MotionModel, CommandIsPeriodic) {
    const ekf_loc::Control first = ekf_loc::rounded_square_command(3.25);
    const ekf_loc::Control second = ekf_loc::rounded_square_command(3.25 + ekf_loc::kCoursePeriodS);
    EXPECT_DOUBLE_EQ(first.v, second.v);
    EXPECT_DOUBLE_EQ(first.omega, second.omega);
    const ekf_loc::Control turn = ekf_loc::rounded_square_command(5.5);
    EXPECT_DOUBLE_EQ(turn.v, 0.4);
    EXPECT_NEAR(turn.omega, ekf_loc::kPi / 4.0, 1e-15);
}

TEST(MotionModel, RejectsNonPositiveDt) {
    EXPECT_THROW(ekf_loc::propagate(ekf_loc::Pose{}, ekf_loc::Control{1.0, 0.0}, 0.0), std::invalid_argument);
    EXPECT_THROW(ekf_loc::state_jacobian(ekf_loc::Pose{}, ekf_loc::Control{}, -0.1), std::invalid_argument);
    EXPECT_THROW(ekf_loc::discrete_control_covariance(0.1, 0.1, 0.0), std::invalid_argument);
    EXPECT_THROW(ekf_loc::discrete_control_covariance(-0.1, 0.0, 0.1), std::invalid_argument);
}

TEST(MotionModel, DiscreteCovarianceScalesWithDt) {
    const Eigen::Matrix2d covariance = ekf_loc::discrete_control_covariance(0.02, 0.004, 0.05);
    EXPECT_NEAR(covariance(0, 0), 0.4, 1e-12);
    EXPECT_NEAR(covariance(1, 1), 0.08, 1e-12);
    EXPECT_DOUBLE_EQ(covariance(0, 1), 0.0);
}

}  // namespace
