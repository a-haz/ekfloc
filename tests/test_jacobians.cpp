#include "numeric_diff.hpp"

#include "ekf_loc/motion_model.hpp"
#include "ekf_loc/observation_model.hpp"

#include <gtest/gtest.h>

#include <ostream>

namespace {

struct MotionCase {
    double x;
    double y;
    double theta;
    double v;
    double omega;
    double dt;
    const char* name;
};

std::ostream& operator<<(std::ostream& out, const MotionCase& motion) {
    return out << motion.name;
}

struct ObservationCase {
    double x;
    double y;
    double theta;
    double landmark_x;
    double landmark_y;
    const char* name;
};

std::ostream& operator<<(std::ostream& out, const ObservationCase& observation) {
    return out << observation.name;
}

Eigen::Vector3d pose_output_delta(const Eigen::VectorXd& upper, const Eigen::VectorXd& lower) {
    Eigen::Vector3d delta = upper - lower;
    delta(2) = ekf_loc::wrap_angle(delta(2));
    return delta;
}

Eigen::Vector2d measurement_output_delta(const Eigen::VectorXd& upper, const Eigen::VectorXd& lower) {
    Eigen::Vector2d delta = upper - lower;
    delta(1) = ekf_loc::wrap_angle(delta(1));
    return delta;
}

void expect_jacobian_close(const Eigen::MatrixXd& analytic, const Eigen::MatrixXd& numeric) {
    ASSERT_EQ(analytic.rows(), numeric.rows());
    ASSERT_EQ(analytic.cols(), numeric.cols());
    for (Eigen::Index row = 0; row < analytic.rows(); ++row) {
        for (Eigen::Index col = 0; col < analytic.cols(); ++col) {
            EXPECT_NEAR(analytic(row, col), numeric(row, col), 1e-6)
                << "row " << row << " col " << col;
        }
    }
}

class MotionJacobianTest : public ::testing::TestWithParam<MotionCase> {};

TEST_P(MotionJacobianTest, StateJacobianMatchesCentralDifference) {
    const MotionCase motion = GetParam();
    ekf_loc::Pose pose;
    pose.x = motion.x;
    pose.y = motion.y;
    pose.theta = motion.theta;
    const ekf_loc::Control control{motion.v, motion.omega};
    const Eigen::Matrix3d analytic = ekf_loc::state_jacobian(pose, control, motion.dt);
    const Eigen::VectorXd state = pose.vector();
    const Eigen::MatrixXd numeric = test_util::central_jacobian(
        [&](const Eigen::VectorXd& value) {
            ekf_loc::Pose perturbed;
            perturbed.x = value(0);
            perturbed.y = value(1);
            perturbed.theta = value(2);
            return ekf_loc::propagate(perturbed, control, motion.dt).vector();
        },
        pose_output_delta, state);
    expect_jacobian_close(analytic, numeric);
}

TEST_P(MotionJacobianTest, ControlJacobianMatchesCentralDifference) {
    const MotionCase motion = GetParam();
    ekf_loc::Pose pose;
    pose.x = motion.x;
    pose.y = motion.y;
    pose.theta = motion.theta;
    const ekf_loc::Control control{motion.v, motion.omega};
    const Eigen::Matrix<double, 3, 2> analytic = ekf_loc::control_jacobian(pose, control, motion.dt);
    Eigen::Vector2d input;
    input << control.v, control.omega;
    const Eigen::MatrixXd numeric = test_util::central_jacobian(
        [&](const Eigen::VectorXd& value) {
            return ekf_loc::propagate(pose, ekf_loc::Control{value(0), value(1)}, motion.dt).vector();
        },
        pose_output_delta, input);
    expect_jacobian_close(analytic, numeric);
}

INSTANTIATE_TEST_SUITE_P(
    AnalyticVersusNumeric, MotionJacobianTest,
    ::testing::Values(
        MotionCase{0.0, 0.0, 0.0, 0.8, 0.0, 0.05, "straight"},
        MotionCase{1.0, -2.0, 0.4, 0.4, 0.785398163397, 0.05, "left_turn"},
        MotionCase{0.0, 0.0, ekf_loc::kPi, 0.0, 0.3, 0.1, "spin_at_pi"},
        MotionCase{-1.0, 0.5, -0.2, -0.3, -0.5, 0.05, "reverse"},
        MotionCase{2.0, 3.0, -2.5, 1.2, 0.01, 0.02, "small_yaw_rate"},
        MotionCase{0.5, 0.5, 1.2, 0.0, -1.0, 0.05, "pure_rotation"},
        MotionCase{4.0, 1.0, -ekf_loc::kPi / 2.0, 0.8, 0.0, 0.1, "drive_south"},
        MotionCase{-3.0, 2.0, 2.0, 0.3, 1.5, 0.05, "tight_turn"},
        MotionCase{0.0, 0.0, 0.0, 0.5, 0.0, 1.0, "long_straight_step"},
        MotionCase{1.5, -1.5, 0.7, 0.2, -0.8, 0.2, "right_turn"}));

class ObservationJacobianTest : public ::testing::TestWithParam<ObservationCase> {};

TEST_P(ObservationJacobianTest, MatchesCentralDifference) {
    const ObservationCase observation = GetParam();
    ekf_loc::Pose pose;
    pose.x = observation.x;
    pose.y = observation.y;
    pose.theta = observation.theta;
    const ekf_loc::Landmark landmark{0, observation.landmark_x, observation.landmark_y};
    const Eigen::Matrix<double, 2, 3> analytic = ekf_loc::measurement_jacobian(pose, landmark);
    const Eigen::MatrixXd numeric = test_util::central_jacobian(
        [&](const Eigen::VectorXd& value) {
            ekf_loc::Pose perturbed;
            perturbed.x = value(0);
            perturbed.y = value(1);
            perturbed.theta = value(2);
            return ekf_loc::predict_measurement(perturbed, landmark);
        },
        measurement_output_delta, pose.vector());
    expect_jacobian_close(analytic, numeric);
}

INSTANTIATE_TEST_SUITE_P(
    AnalyticVersusNumeric, ObservationJacobianTest,
    ::testing::Values(ObservationCase{0.0, 0.0, 0.0, 3.0, 4.0, "three_four_five"},
                      ObservationCase{0.0, 0.0, 0.0, -1.0, -1.0, "behind_left"},
                      ObservationCase{0.0, 0.0, 0.0, 6.0, -1.0, "ahead_right"},
                      ObservationCase{4.5, 1.0, 1.2, -1.0, -1.0, "offset_pose"},
                      ObservationCase{4.5, 1.0, 1.2, 6.0, 6.0, "far_corner"},
                      ObservationCase{2.5, 2.4, -2.8, 2.5, 2.5, "nearby"},
                      ObservationCase{-0.5, 4.5, -1.4, -1.5, 6.4, "heading_near_cut"},
                      ObservationCase{1.0, 1.0, ekf_loc::kPi, 1.0, 4.0, "heading_at_pi"}));

}  // namespace
