#include "ekf_loc/simulator.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

double mean_of(const std::vector<double>& values) {
    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    return sum / static_cast<double>(values.size());
}

double median_of(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const std::size_t mid = values.size() / 2;
    if (values.size() % 2 == 0) {
        return 0.5 * (values[mid - 1] + values[mid]);
    }
    return values[mid];
}

ekf_loc::SimConfig consistency_config(std::uint64_t seed) {
    ekf_loc::SimConfig config = ekf_loc::benchmark_config(seed);
    config.sample_initial_estimate = true;
    config.initial_covariance = Eigen::Matrix3d::Zero();
    config.initial_covariance(0, 0) = 0.05 * 0.05;
    config.initial_covariance(1, 1) = 0.05 * 0.05;
    config.initial_covariance(2, 2) = (2.0 * ekf_loc::kDegToRad) * (2.0 * ekf_loc::kDegToRad);
    return config;
}

// Independent Monte Carlo trials. The bounds are wider than a 95% chi-square
// interval so a correct filter is stable across standard-library RNGs, and
// narrow enough that a wrong Jacobian or noise mapping fails.
TEST(Consistency, PredictionOnlyFinalNeesAveragesNearStateDimension) {
    constexpr int kRuns = 80;
    std::vector<double> finals;
    finals.reserve(kRuns);
    for (int run = 0; run < kRuns; ++run) {
        ekf_loc::SimConfig config = consistency_config(10000 + static_cast<std::uint64_t>(run));
        config.landmarks.clear();
        config.duration = 8.0;
        const ekf_loc::SimulationResult result = ekf_loc::Simulator(config).run();
        ASSERT_FALSE(result.samples.empty());
        EXPECT_EQ(result.measurement_count, 0u);
        const ekf_loc::TrajectorySample& last = result.samples.back();
        EXPECT_NEAR(last.ekf.x, last.dead_reckoning.x, 1e-12);
        EXPECT_NEAR(last.ekf.y, last.dead_reckoning.y, 1e-12);
        EXPECT_NEAR(last.ekf.theta, last.dead_reckoning.theta, 1e-12);
        finals.push_back(last.nees_ekf);
        EXPECT_TRUE(std::isfinite(last.nees_ekf));
        EXPECT_GE(last.nees_ekf, 0.0);
    }
    const double mean = mean_of(finals);
    const double median = median_of(finals);
    EXPECT_NEAR(mean, 3.0, 0.8) << "mean final NEES " << mean;
    EXPECT_GT(median, 1.2);
    EXPECT_LT(median, 5.0);
}

TEST(Consistency, EkfNeesAndNisMatchTheirDegreesOfFreedom) {
    constexpr int kRuns = 24;
    std::vector<double> nees;
    std::vector<double> nis;
    std::vector<double> ekf_rmse;
    std::vector<double> dr_rmse;
    nees.reserve(kRuns);
    for (int run = 0; run < kRuns; ++run) {
        ekf_loc::SimConfig config = consistency_config(20000 + static_cast<std::uint64_t>(run));
        config.duration = 20.0;
        const ekf_loc::SimulationResult result = ekf_loc::Simulator(config).run();
        ASSERT_GT(result.measurement_count, 100u);
        nees.push_back(result.ekf.mean_nees);
        nis.push_back(result.mean_nis);
        ekf_rmse.push_back(result.ekf.rmse_position_m);
        dr_rmse.push_back(result.dead_reckoning.rmse_position_m);
        EXPECT_LT(result.ekf.rmse_position_m, result.dead_reckoning.rmse_position_m);
    }
    EXPECT_NEAR(mean_of(nees), 3.0, 0.9) << "mean NEES " << mean_of(nees);
    EXPECT_NEAR(mean_of(nis), 2.0, 0.25) << "mean NIS " << mean_of(nis);
    EXPECT_LT(mean_of(ekf_rmse), 0.15);
    EXPECT_GT(mean_of(dr_rmse), 0.3);
    EXPECT_LT(mean_of(ekf_rmse) * 3.0, mean_of(dr_rmse));
}

}  // namespace
