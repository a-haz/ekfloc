#include "ekf_loc/scenario.hpp"

#include <cmath>

namespace ekf_loc {

Control rounded_square_command(double time_s) {
    double phase = std::fmod(time_s, kCoursePeriodS);
    if (phase < 0.0) {
        phase += kCoursePeriodS;
    }
    const double leg = std::fmod(phase, 7.0);
    if (leg < 5.0) {
        return Control{0.8, 0.0};
    }
    return Control{0.4, (kPi / 2.0) / 2.0};
}

std::vector<Landmark> benchmark_landmarks() {
    return {
        Landmark{0, -1.5, -1.2}, Landmark{1, 6.0, -1.2}, Landmark{2, 6.0, 6.4},
        Landmark{3, -1.5, 6.4}, Landmark{4, 2.0, 2.5},
    };
}

}  // namespace ekf_loc
