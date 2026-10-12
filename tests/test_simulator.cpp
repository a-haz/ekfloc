#include "ekf_loc/metrics.hpp"
#include "ekf_loc/simulator.hpp"

#include <gtest/gtest.h>

#include <Eigen/Eigenvalues>

#include <cmath>

namespace {

TEST(Simulator, ZeroProcessNoiseOpenLoopTracksTheCourse) {
    ekf_loc::SimConfig config = ekf_loc::benchmark_config(7);
    config.landmarks.clear();
    config.noise.velocity_density = 0.0;
    config.noise.yaw_rate_density = 0.0;
    config.duration = ekf_loc::kCoursePeriodS;
    const ekf_loc::SimulationResult result = ekf_loc::Simulator(config).run();
    ASSERT_FALSE(result.samples.empty());
    for (const ekf_loc::TrajectorySample& sample : result.samples) {
        EXPECT_NEAR(sample.dead_reckoning.x, sample.truth.x, 1e-9);
        EXPECT_NEAR(sample.dead_reckoning.y, sample.truth.y, 1e-9);
        EXPECT_NEAR(sample.ekf.theta, sample.truth.theta, 1e-9);
    }
    const ekf_loc::TrajectorySample& last = result.samples.back();
    EXPECT_NEAR(last.truth.x, 0.0, 1e-9);
    EXPECT_NEAR(last.truth.y, 0.0, 1e-9);
    EXPECT_NEAR(last.truth.theta, 0.0, 1e-9);
    EXPECT_NEAR(result.dead_reckoning.rmse_position_m, 0.0, 1e-9);
    EXPECT_NEAR(result.ekf.rmse_position_m, 0.0, 1e-9);
    EXPECT_EQ(result.measurement_count, 0u);
}

TEST(Simulator, SameSeedReproducesTheTrajectory) {
    const ekf_loc::SimulationResult first = ekf_loc::Simulator(ekf_loc::benchmark_config(3)).run();
    const ekf_loc::SimulationResult second = ekf_loc::Simulator(ekf_loc::benchmark_config(3)).run();
    ASSERT_EQ(first.samples.size(), second.samples.size());
    EXPECT_EQ(first.measurement_count, second.measurement_count);
    EXPECT_DOUBLE_EQ(first.ekf.rmse_position_m, second.ekf.rmse_position_m);
    EXPECT_DOUBLE_EQ(first.mean_nis, second.mean_nis);
    const std::size_t last = first.samples.size() - 1;
    EXPECT_DOUBLE_EQ(first.samples[last].ekf.x, second.samples[last].ekf.x);
    EXPECT_DOUBLE_EQ(first.samples[last].dead_reckoning.y, second.samples[last].dead_reckoning.y);
    EXPECT_DOUBLE_EQ(first.samples[last].nees_ekf, second.samples[last].nees_ekf);
}

TEST(Simulator, DifferentSeedsChangeDeadReckoning) {
    const ekf_loc::SimulationResult first = ekf_loc::Simulator(ekf_loc::benchmark_config(1)).run();
    const ekf_loc::SimulationResult second = ekf_loc::Simulator(ekf_loc::benchmark_config(2)).run();
    EXPECT_NE(first.dead_reckoning.rmse_position_m, second.dead_reckoning.rmse_position_m);
}

TEST(Simulator, BenchmarkRunProducesFiniteConsistentScores) {
    const ekf_loc::SimulationResult result = ekf_loc::Simulator(ekf_loc::benchmark_config(4)).run();
    EXPECT_GT(result.measurement_count, 1000u);
    EXPECT_GT(result.samples.size(), 1000u);
    EXPECT_LT(result.ekf.rmse_position_m, result.dead_reckoning.rmse_position_m);
    EXPECT_LT(result.ekf.rmse_heading_deg, result.dead_reckoning.rmse_heading_deg);
    EXPECT_TRUE(std::isfinite(result.ekf.mean_nees));
    EXPECT_TRUE(std::isfinite(result.dead_reckoning.mean_nees));
    EXPECT_NEAR(result.mean_nis, 2.0, 0.5);
    EXPECT_GT(result.ekf.mean_nees, 1.0);
    EXPECT_LT(result.ekf.mean_nees, 6.0);

    const ekf_loc::TrajectorySample& last = result.samples.back();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(last.covariance_ekf);
    ASSERT_EQ(solver.info(), Eigen::Success);
    EXPECT_GT(solver.eigenvalues().minCoeff(), 0.0);
    EXPECT_LT((last.covariance_ekf - last.covariance_ekf.transpose()).norm(), 1e-8);

    const ekf_loc::MethodScore recomputed = ekf_loc::score_ekf(result.samples);
    EXPECT_DOUBLE_EQ(recomputed.rmse_position_m, result.ekf.rmse_position_m);
    EXPECT_DOUBLE_EQ(recomputed.mean_nees, result.ekf.mean_nees);
}

TEST(Simulator, LandmarksStayClearOfTheFixedCourse) {
    ekf_loc::SimConfig config = ekf_loc::benchmark_config(1);
    config.landmarks.clear();
    config.noise.velocity_density = 0.0;
    config.noise.yaw_rate_density = 0.0;
    const ekf_loc::SimulationResult result = ekf_loc::Simulator(config).run();
    for (const ekf_loc::Landmark& landmark : ekf_loc::benchmark_landmarks()) {
        double nearest = 1e9;
        for (const ekf_loc::TrajectorySample& sample : result.samples) {
            const double distance =
                std::hypot(landmark.x - sample.truth.x, landmark.y - sample.truth.y);
            nearest = std::min(nearest, distance);
        }
        EXPECT_GT(nearest, 1.5) << "landmark " << landmark.id;
    }
}

TEST(Simulator, RejectsAnEmptyCommand) {
    ekf_loc::SimConfig config = ekf_loc::benchmark_config(1);
    config.command = nullptr;
    EXPECT_THROW(ekf_loc::Simulator{config}, std::invalid_argument);
}

TEST(Metrics, PoseErrorWrapsHeading) {
    const ekf_loc::Pose truth{0.0, 0.0, ekf_loc::kPi - 0.05};
    const ekf_loc::Pose estimate{0.1, -0.2, -ekf_loc::kPi + 0.01};
    const Eigen::Vector3d error = ekf_loc::pose_error(truth, estimate);
    EXPECT_NEAR(error(0), -0.1, 1e-12);
    EXPECT_NEAR(error(1), 0.2, 1e-12);
    EXPECT_NEAR(error(2), -0.06, 1e-12);
}

}  // namespace
