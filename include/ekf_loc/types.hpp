#pragma once

#include "ekf_loc/angle.hpp"

#include <Eigen/Core>

namespace ekf_loc {

/// Planar pose. Heading is radians, wrapped to [-pi, pi) when constructed from a vector.
struct Pose {
    double x{0.0};
    double y{0.0};
    double theta{0.0};

    Eigen::Vector3d vector() const { return Eigen::Vector3d(x, y, theta); }

    static Pose from_vector(const Eigen::Vector3d& value) {
        Pose pose;
        pose.x = value(0);
        pose.y = value(1);
        pose.theta = wrap_angle(value(2));
        return pose;
    }
};

/// Differential-drive body velocity. v is m/s, omega is rad/s.
struct Control {
    double v{0.0};
    double omega{0.0};
};

/// Landmark with a stable integer id. Positions are meters in the map frame.
struct Landmark {
    int id{0};
    double x{0.0};
    double y{0.0};
};

/// Range (m) and bearing (rad) of one landmark. Data association is the landmark id.
struct RangeBearing {
    int landmark_id{0};
    double range{0.0};
    double bearing{0.0};
};

/// Continuous-time odometry densities and discrete measurement standard deviations.
struct NoiseParameters {
    /// Power spectral density of linear-velocity noise, m^2/s^3.
    double velocity_density{0.0};
    /// Power spectral density of yaw-rate noise, rad^2/s^3.
    double yaw_rate_density{0.0};
    /// Range measurement standard deviation, m.
    double sigma_range{0.0};
    /// Bearing measurement standard deviation, rad.
    double sigma_bearing{0.0};
};

}  // namespace ekf_loc
