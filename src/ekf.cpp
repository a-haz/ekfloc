#include "ekf_loc/ekf.hpp"

#include "ekf_loc/motion_model.hpp"
#include "ekf_loc/observation_model.hpp"

#include <Eigen/Cholesky>
#include <stdexcept>

namespace ekf_loc {
namespace {

void symmetrize(Eigen::Matrix2d& matrix) { matrix = 0.5 * (matrix + matrix.transpose()); }

void symmetrize(Eigen::Matrix3d& matrix) { matrix = 0.5 * (matrix + matrix.transpose()); }

void require_positive_definite(const Eigen::Matrix3d& covariance) {
    Eigen::LLT<Eigen::Matrix3d> llt(covariance);
    if (llt.info() != Eigen::Success) {
        throw std::invalid_argument("covariance must be positive definite");
    }
}

}  // namespace

ExtendedKalmanFilter::ExtendedKalmanFilter(const Pose& initial_state,
                                           const Eigen::Matrix3d& initial_covariance,
                                           NoiseParameters noise)
    : state_(initial_state), covariance_(initial_covariance), noise_(noise) {
    state_.theta = wrap_angle(state_.theta);
    require_positive_definite(covariance_);
    if (noise_.velocity_density < 0.0 || noise_.yaw_rate_density < 0.0 || noise_.sigma_range < 0.0 ||
        noise_.sigma_bearing < 0.0) {
        throw std::invalid_argument("noise parameters must be non-negative");
    }
    symmetrize(covariance_);
}

void ExtendedKalmanFilter::predict(const Control& odometry, double dt) {
    const Eigen::Matrix3d state_jac = state_jacobian(state_, odometry, dt);
    const Eigen::Matrix<double, 3, 2> control_jac = control_jacobian(state_, odometry, dt);
    const Eigen::Matrix2d control_cov =
        discrete_control_covariance(noise_.velocity_density, noise_.yaw_rate_density, dt);
    const Eigen::Matrix3d process_cov = control_jac * control_cov * control_jac.transpose();
    state_ = propagate(state_, odometry, dt);
    covariance_ = state_jac * covariance_ * state_jac.transpose() + process_cov;
    symmetrize(covariance_);
}

double ExtendedKalmanFilter::update(const RangeBearing& measurement, const Landmark& landmark) {
    if (measurement.landmark_id != landmark.id) {
        throw std::invalid_argument("measurement landmark id does not match the landmark");
    }
    const Eigen::Vector2d predicted = predict_measurement(state_, landmark);
    const Eigen::Matrix<double, 2, 3> measurement_jac = measurement_jacobian(state_, landmark);
    Eigen::Vector2d innovation;
    innovation << measurement.range - predicted(0), wrap_angle(measurement.bearing - predicted(1));

    Eigen::Matrix2d measurement_cov = Eigen::Matrix2d::Zero();
    measurement_cov(0, 0) = noise_.sigma_range * noise_.sigma_range;
    measurement_cov(1, 1) = noise_.sigma_bearing * noise_.sigma_bearing;

    Eigen::Matrix2d innovation_cov =
        measurement_jac * covariance_ * measurement_jac.transpose() + measurement_cov;
    symmetrize(innovation_cov);
    Eigen::LDLT<Eigen::Matrix2d> ldlt(innovation_cov);
    if (ldlt.info() != Eigen::Success) {
        throw std::runtime_error("innovation covariance is not positive definite");
    }
    const Eigen::Matrix2d innovation_cov_inv = ldlt.solve(Eigen::Matrix2d::Identity());
    const double nis = innovation.dot(innovation_cov_inv * innovation);
    const Eigen::Matrix<double, 3, 2> gain = covariance_ * measurement_jac.transpose() * innovation_cov_inv;
    const Eigen::Vector3d correction = gain * innovation;
    state_.x += correction(0);
    state_.y += correction(1);
    state_.theta = wrap_angle(state_.theta + correction(2));

    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
    const Eigen::Matrix3d residual_gain = identity - gain * measurement_jac;
    covariance_ = residual_gain * covariance_ * residual_gain.transpose() +
                  gain * measurement_cov * gain.transpose();
    symmetrize(covariance_);
    return nis;
}

}  // namespace ekf_loc
