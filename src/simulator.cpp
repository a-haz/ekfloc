#include "ekf_loc/simulator.hpp"

#include "ekf_loc/ekf.hpp"
#include "ekf_loc/motion_model.hpp"
#include "ekf_loc/observation_model.hpp"

#include <Eigen/Cholesky>
#include <cmath>
#include <random>
#include <stdexcept>
#include <utility>

namespace ekf_loc {
namespace {

Pose sample_pose(std::mt19937_64& rng, const Pose& mean, const Eigen::Matrix3d& covariance) {
    Eigen::LLT<Eigen::Matrix3d> llt(covariance);
    if (llt.info() != Eigen::Success) {
        throw std::invalid_argument("initial covariance must be positive definite");
    }
    std::normal_distribution<double> unit_normal(0.0, 1.0);
    Eigen::Vector3d standard;
    standard << unit_normal(rng), unit_normal(rng), unit_normal(rng);
    return Pose::from_vector(mean.vector() + llt.matrixL() * standard);
}

void validate(const SimConfig& config) {
    if (!(config.dt > 0.0)) {
        throw std::invalid_argument("dt must be positive");
    }
    if (!(config.duration > 0.0)) {
        throw std::invalid_argument("duration must be positive");
    }
    if (!(config.measurement_period > 0.0)) {
        throw std::invalid_argument("measurement period must be positive");
    }
    if (!(config.max_range > config.min_range) || config.min_range < 0.0) {
        throw std::invalid_argument("range gate is invalid");
    }
    if (!config.command) {
        throw std::invalid_argument("command function is empty");
    }
    if (config.noise.velocity_density < 0.0 || config.noise.yaw_rate_density < 0.0 ||
        config.noise.sigma_range < 0.0 || config.noise.sigma_bearing < 0.0) {
        throw std::invalid_argument("noise parameters must be non-negative");
    }
}

}  // namespace

SimConfig benchmark_config(std::uint64_t seed) {
    SimConfig config;
    config.dt = 0.05;
    config.duration = 2.0 * kCoursePeriodS;
    config.measurement_period = 0.2;
    config.noise.velocity_density = 0.02;
    config.noise.yaw_rate_density = 0.004;
    config.noise.sigma_range = 0.08;
    config.noise.sigma_bearing = 2.0 * kDegToRad;
    config.max_range = 12.0;
    config.min_range = 0.3;
    config.initial_truth = Pose{};
    config.initial_estimate = Pose{};
    config.initial_covariance = Eigen::Matrix3d::Zero();
    config.initial_covariance(0, 0) = 0.02 * 0.02;
    config.initial_covariance(1, 1) = 0.02 * 0.02;
    config.initial_covariance(2, 2) = (0.5 * kDegToRad) * (0.5 * kDegToRad);
    config.sample_initial_estimate = false;
    config.seed = seed;
    config.landmarks = benchmark_landmarks();
    config.command = rounded_square_command;
    return config;
}

Simulator::Simulator(SimConfig config) : config_(std::move(config)) { validate(config_); }

SimulationResult Simulator::run() const {
    std::mt19937_64 rng(config_.seed);
    std::normal_distribution<double> unit_normal(0.0, 1.0);

    Pose truth = config_.initial_truth;
    truth.theta = wrap_angle(truth.theta);
    Pose estimate = config_.sample_initial_estimate
                        ? sample_pose(rng, truth, config_.initial_covariance)
                        : config_.initial_estimate;
    estimate.theta = wrap_angle(estimate.theta);

    ExtendedKalmanFilter dead_reckoning(estimate, config_.initial_covariance, config_.noise);
    ExtendedKalmanFilter ekf(estimate, config_.initial_covariance, config_.noise);

    const int steps = static_cast<int>(std::llround(config_.duration / config_.dt));
    if (steps <= 0) {
        throw std::invalid_argument("duration is shorter than one step");
    }
    const int measurement_stride =
        static_cast<int>(std::llround(config_.measurement_period / config_.dt));
    if (measurement_stride < 1) {
        throw std::invalid_argument("measurement period is shorter than dt");
    }

    SimulationResult result;
    result.samples.reserve(static_cast<std::size_t>(steps));
    double nis_sum = 0.0;
    std::size_t nis_count = 0;

    for (int step = 0; step < steps; ++step) {
        const double time_s = step * config_.dt;
        const Control command = config_.command(time_s);
        truth = propagate(truth, command, config_.dt);

        const Eigen::Matrix2d control_cov = discrete_control_covariance(
            config_.noise.velocity_density, config_.noise.yaw_rate_density, config_.dt);
        Control odometry;
        odometry.v = command.v + unit_normal(rng) * std::sqrt(control_cov(0, 0));
        odometry.omega = command.omega + unit_normal(rng) * std::sqrt(control_cov(1, 1));
        dead_reckoning.predict(odometry, config_.dt);
        ekf.predict(odometry, config_.dt);

        if ((step + 1) % measurement_stride == 0) {
            for (const Landmark& landmark : config_.landmarks) {
                const Eigen::Vector2d predicted = predict_measurement(truth, landmark);
                if (predicted(0) < config_.min_range || predicted(0) > config_.max_range) {
                    continue;
                }
                RangeBearing measurement;
                measurement.landmark_id = landmark.id;
                measurement.range = predicted(0) + unit_normal(rng) * config_.noise.sigma_range;
                measurement.bearing =
                    wrap_angle(predicted(1) + unit_normal(rng) * config_.noise.sigma_bearing);
                nis_sum += ekf.update(measurement, landmark);
                ++nis_count;
            }
        }

        TrajectorySample sample;
        sample.time_s = (step + 1) * config_.dt;
        sample.truth = truth;
        sample.dead_reckoning = dead_reckoning.state();
        sample.ekf = ekf.state();
        sample.covariance_dead_reckoning = dead_reckoning.covariance();
        sample.covariance_ekf = ekf.covariance();
        sample.nees_dead_reckoning = normalized_estimation_error_squared(
            truth, dead_reckoning.state(), dead_reckoning.covariance());
        sample.nees_ekf =
            normalized_estimation_error_squared(truth, ekf.state(), ekf.covariance());
        result.samples.push_back(sample);
    }

    result.dead_reckoning = score_dead_reckoning(result.samples);
    result.ekf = score_ekf(result.samples);
    result.measurement_count = nis_count;
    result.mean_nis = nis_count == 0 ? 0.0 : nis_sum / static_cast<double>(nis_count);
    return result;
}

}  // namespace ekf_loc
