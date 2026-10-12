#include "ekf_loc/motion_model.hpp"

#include <stdexcept>

namespace ekf_loc {
namespace {

struct Midpoint {
    double cosine;
    double sine;
};

Midpoint midpoint_heading(const Pose& pose, const Control& control, double dt) {
    const double theta_mid = pose.theta + 0.5 * control.omega * dt;
    return Midpoint{std::cos(theta_mid), std::sin(theta_mid)};
}

void require_positive_dt(double dt) {
    if (!(dt > 0.0)) {
        throw std::invalid_argument("dt must be positive");
    }
}

}  // namespace

Pose propagate(const Pose& pose, const Control& control, double dt) {
    require_positive_dt(dt);
    const Midpoint mid = midpoint_heading(pose, control, dt);
    Pose next;
    next.x = pose.x + control.v * dt * mid.cosine;
    next.y = pose.y + control.v * dt * mid.sine;
    next.theta = wrap_angle(pose.theta + control.omega * dt);
    return next;
}

Eigen::Matrix3d state_jacobian(const Pose& pose, const Control& control, double dt) {
    require_positive_dt(dt);
    const Midpoint mid = midpoint_heading(pose, control, dt);
    Eigen::Matrix3d jacobian = Eigen::Matrix3d::Identity();
    jacobian(0, 2) = -control.v * dt * mid.sine;
    jacobian(1, 2) = control.v * dt * mid.cosine;
    return jacobian;
}

Eigen::Matrix<double, 3, 2> control_jacobian(const Pose& pose, const Control& control, double dt) {
    require_positive_dt(dt);
    const Midpoint mid = midpoint_heading(pose, control, dt);
    Eigen::Matrix<double, 3, 2> jacobian;
    jacobian << dt * mid.cosine, -0.5 * control.v * dt * dt * mid.sine, dt * mid.sine,
        0.5 * control.v * dt * dt * mid.cosine, 0.0, dt;
    return jacobian;
}

Eigen::Matrix2d discrete_control_covariance(double velocity_density, double yaw_rate_density, double dt) {
    require_positive_dt(dt);
    if (velocity_density < 0.0 || yaw_rate_density < 0.0) {
        throw std::invalid_argument("noise densities must be non-negative");
    }
    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    covariance(0, 0) = velocity_density / dt;
    covariance(1, 1) = yaw_rate_density / dt;
    return covariance;
}

}  // namespace ekf_loc
