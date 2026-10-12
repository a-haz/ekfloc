#pragma once

#include "ekf_loc/metrics.hpp"
#include "ekf_loc/types.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace ekf_loc {

inline void write_trace_csv(const std::filesystem::path& path,
                            const std::vector<TrajectorySample>& samples) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("could not write " + path.string());
    }
    out.imbue(std::locale::classic());
    out << std::fixed << std::setprecision(8);
    out << "t_s,x_true_m,y_true_m,theta_true_rad,x_dr_m,y_dr_m,theta_dr_rad,"
           "x_ekf_m,y_ekf_m,theta_ekf_rad,pos_err_dr_m,pos_err_ekf_m,"
           "heading_err_dr_deg,heading_err_ekf_deg,nees_dr,nees_ekf\n";
    for (const TrajectorySample& sample : samples) {
        const Eigen::Vector3d dr_error = pose_error(sample.truth, sample.dead_reckoning);
        const Eigen::Vector3d ekf_error = pose_error(sample.truth, sample.ekf);
        out << sample.time_s << ',' << sample.truth.x << ',' << sample.truth.y << ','
            << sample.truth.theta << ',' << sample.dead_reckoning.x << ',' << sample.dead_reckoning.y
            << ',' << sample.dead_reckoning.theta << ',' << sample.ekf.x << ',' << sample.ekf.y << ','
            << sample.ekf.theta << ',' << std::hypot(dr_error(0), dr_error(1)) << ','
            << std::hypot(ekf_error(0), ekf_error(1)) << ',' << std::abs(dr_error(2)) * kRadToDeg << ','
            << std::abs(ekf_error(2)) * kRadToDeg << ',' << sample.nees_dead_reckoning << ','
            << sample.nees_ekf << '\n';
    }
}

inline void write_landmarks_csv(const std::filesystem::path& path, const std::vector<Landmark>& landmarks) {
    if (path.has_parent_path()) {
        std::filesystem::create_directories(path.parent_path());
    }
    std::ofstream out(path);
    if (!out) {
        throw std::runtime_error("could not write " + path.string());
    }
    out.imbue(std::locale::classic());
    out << std::fixed << std::setprecision(6);
    out << "id,x_m,y_m\n";
    for (const Landmark& landmark : landmarks) {
        out << landmark.id << ',' << landmark.x << ',' << landmark.y << '\n';
    }
}

inline std::string format_score_row(const std::string& seed, const std::string& method, const MethodScore& score) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::fixed << std::setprecision(6);
    out << seed << ',' << method << ',' << score.rmse_position_m << ',' << score.rmse_heading_deg << ','
        << score.final_position_error_m << ',' << score.final_heading_error_deg << ',' << score.mean_nees;
    return out.str();
}

}  // namespace ekf_loc
