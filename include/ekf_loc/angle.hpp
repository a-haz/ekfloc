#pragma once

#include <cmath>

namespace ekf_loc {

/// Pi, so call sites do not depend on a non-standard math constant.
inline constexpr double kPi = 3.14159265358979323846;

/// Degrees to radians.
inline constexpr double kDegToRad = kPi / 180.0;

/// Radians to degrees.
inline constexpr double kRadToDeg = 180.0 / kPi;

/// Wrap an angle in radians into [-pi, pi).
inline double wrap_angle(double angle) {
    const double two_pi = 2.0 * kPi;
    angle = std::fmod(angle + kPi, two_pi);
    if (angle < 0.0) {
        angle += two_pi;
    }
    return angle - kPi;
}

}  // namespace ekf_loc
