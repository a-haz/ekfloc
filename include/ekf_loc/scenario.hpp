#pragma once

#include "ekf_loc/types.hpp"

#include <vector>

namespace ekf_loc {

/// Period of the fixed rounded-square course, in seconds.
inline constexpr double kCoursePeriodS = 28.0;

/// Open-loop body velocity for the fixed course.
///
/// Each 28 s lap has four legs. A leg is a 5 s straight segment at 0.8 m/s
/// followed by a 2 s moving left turn (v = 0.4 m/s, omega = pi/4 rad/s).
Control rounded_square_command(double time_s);

/// Five landmarks placed around the course, all more than 1.7 m from the path.
std::vector<Landmark> benchmark_landmarks();

}  // namespace ekf_loc
