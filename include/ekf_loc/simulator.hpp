#pragma once

#include "ekf_loc/metrics.hpp"
#include "ekf_loc/scenario.hpp"

#include <cstdint>
#include <functional>

namespace ekf_loc {

/// Command evaluated at the start of a step. The argument is time in seconds.
using CommandFunction = std::function<Control(double)>;

/// Full configuration of one simulated run.
struct SimConfig {
    double dt{0.05};
    double duration{56.0};
    double measurement_period{0.2};
    NoiseParameters noise{};
    double max_range{12.0};
    double min_range{0.3};
    Pose initial_truth{};
    Pose initial_estimate{};
    Eigen::Matrix3d initial_covariance = Eigen::Matrix3d::Identity();
    /// When true, the initial estimate is drawn from N(initial_truth, initial_covariance).
    bool sample_initial_estimate{false};
    std::uint64_t seed{1};
    std::vector<Landmark> landmarks{};
    CommandFunction command{rounded_square_command};
};

struct SimulationResult {
    std::vector<TrajectorySample> samples;
    MethodScore dead_reckoning;
    MethodScore ekf;
    double mean_nis{0.0};
    std::size_t measurement_count{0};
};

/// Fixed-course configuration used by the benchmark and the consistency tests.
/// Both estimators start at the true pose. initial_covariance is
/// diag(0.02^2 m^2, 0.02^2 m^2, (0.5 deg)^2).
SimConfig benchmark_config(std::uint64_t seed);

/// 2D differential-drive simulator with configurable Gaussian noise.
///
/// The true robot follows `command` exactly, so the geometric course is fixed.
/// Wheel odometry is that command plus discrete noise consistent with the
/// filter's spectral densities. Dead reckoning and the EKF both integrate the
/// same odometry. Only the EKF applies range-bearing updates. A landmark is
/// reported when its true range lies in [min_range, max_range].
class Simulator {
public:
    explicit Simulator(SimConfig config);

    const SimConfig& config() const { return config_; }

    SimulationResult run() const;

private:
    SimConfig config_;
};

}  // namespace ekf_loc
