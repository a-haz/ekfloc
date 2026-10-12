#include "csv_export.hpp"

#include "ekf_loc/simulator.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <locale>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void print_help() {
    std::cout
        << "ekf_loc_benchmark — compare dead reckoning and the EKF on a fixed course\n"
        << "\n"
        << "Runs several noise seeds. The commanded course does not change.\n"
        << "Both estimators integrate the same wheel odometry; only the EKF uses\n"
        << "range-bearing landmarks. All output is simulated.\n"
        << "\n"
        << "Usage:\n"
        << "  ekf_loc_benchmark [options]\n"
        << "\n"
        << "Options:\n"
        << "  --seeds <int>        Number of seeds (default 8)\n"
        << "  --first-seed <int>   First seed, then consecutive seeds (default 1)\n"
        << "  --trace-seed <int>   Seed written to the trace CSV (default: first seed)\n"
        << "  --output <file>      Summary CSV (default docs/benchmark.csv)\n"
        << "  --trace <file>       Per-step trace of --trace-seed (default docs/sample_run.csv)\n"
        << "  --landmarks <file>   Landmark table (default docs/landmarks.csv)\n"
        << "  --help               Show this text\n";
}

unsigned long long require_ull(const std::string& text, const std::string& flag) {
    try {
        std::size_t used = 0;
        const unsigned long long value = std::stoull(text, &used);
        if (used != text.size() || value == 0) {
            throw std::invalid_argument("bad");
        }
        return value;
    } catch (const std::exception&) {
        throw std::runtime_error("invalid positive integer for " + flag + ": " + text);
    }
}

ekf_loc::MethodScore average_scores(const std::vector<ekf_loc::MethodScore>& scores) {
    ekf_loc::MethodScore mean;
    const double count = static_cast<double>(scores.size());
    for (const ekf_loc::MethodScore& score : scores) {
        mean.rmse_position_m += score.rmse_position_m;
        mean.rmse_heading_deg += score.rmse_heading_deg;
        mean.final_position_error_m += score.final_position_error_m;
        mean.final_heading_error_deg += score.final_heading_error_deg;
        mean.mean_nees += score.mean_nees;
    }
    mean.rmse_position_m /= count;
    mean.rmse_heading_deg /= count;
    mean.final_position_error_m /= count;
    mean.final_heading_error_deg /= count;
    mean.mean_nees /= count;
    return mean;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        int seed_count = 8;
        std::uint64_t first_seed = 1;
        std::uint64_t trace_seed = 1;
        bool trace_seed_set = false;
        std::filesystem::path output = "docs/benchmark.csv";
        std::filesystem::path trace = "docs/sample_run.csv";
        std::filesystem::path landmarks = "docs/landmarks.csv";

        for (int i = 1; i < argc; ++i) {
            const std::string flag = argv[i];
            if (flag == "--help" || flag == "-h") {
                print_help();
                return 0;
            }
            if (i + 1 >= argc) {
                throw std::runtime_error("missing value for " + flag);
            }
            const std::string value = argv[++i];
            if (flag == "--seeds") {
                seed_count = static_cast<int>(require_ull(value, flag));
            } else if (flag == "--first-seed") {
                first_seed = static_cast<std::uint64_t>(require_ull(value, flag));
            } else if (flag == "--trace-seed") {
                trace_seed = static_cast<std::uint64_t>(require_ull(value, flag));
                trace_seed_set = true;
            } else if (flag == "--output") {
                output = value;
            } else if (flag == "--trace") {
                trace = value;
            } else if (flag == "--landmarks") {
                landmarks = value;
            } else {
                throw std::runtime_error("unknown option " + flag);
            }
        }
        if (!trace_seed_set) {
            trace_seed = first_seed;
        }

        if (output.has_parent_path()) {
            std::filesystem::create_directories(output.parent_path());
        }
        std::ofstream summary(output);
        if (!summary) {
            throw std::runtime_error("could not write " + output.string());
        }
        summary.imbue(std::locale::classic());
        summary << "seed,method,rmse_position_m,rmse_heading_deg,final_position_error_m,"
                   "final_heading_error_deg,mean_nees\n";

        bool wrote_trace = false;
        std::vector<ekf_loc::MethodScore> dead_reckoning_scores;
        std::vector<ekf_loc::MethodScore> ekf_scores;
        dead_reckoning_scores.reserve(static_cast<std::size_t>(seed_count));
        ekf_scores.reserve(static_cast<std::size_t>(seed_count));

        for (int index = 0; index < seed_count; ++index) {
            const std::uint64_t seed = first_seed + static_cast<std::uint64_t>(index);
            const ekf_loc::SimulationResult result =
                ekf_loc::Simulator(ekf_loc::benchmark_config(seed)).run();
            const std::string seed_text = std::to_string(seed);
            summary << ekf_loc::format_score_row(seed_text, "dead_reckoning", result.dead_reckoning)
                    << '\n';
            summary << ekf_loc::format_score_row(seed_text, "ekf", result.ekf) << '\n';
            dead_reckoning_scores.push_back(result.dead_reckoning);
            ekf_scores.push_back(result.ekf);
            std::cout << "seed " << seed << "  dr_rmse_m=" << std::fixed << std::setprecision(4)
                      << result.dead_reckoning.rmse_position_m
                      << "  ekf_rmse_m=" << result.ekf.rmse_position_m
                      << "  ekf_nees=" << result.ekf.mean_nees << "  nis=" << result.mean_nis << '\n';
            if (seed == trace_seed) {
                ekf_loc::write_trace_csv(trace, result.samples);
                ekf_loc::write_landmarks_csv(landmarks, ekf_loc::benchmark_landmarks());
                wrote_trace = true;
            }
        }
        if (!wrote_trace) {
            throw std::runtime_error("--trace-seed is outside the seeds that were run");
        }

        const ekf_loc::MethodScore dead_reckoning_mean = average_scores(dead_reckoning_scores);
        const ekf_loc::MethodScore ekf_mean = average_scores(ekf_scores);
        summary << ekf_loc::format_score_row("mean", "dead_reckoning", dead_reckoning_mean) << '\n';
        summary << ekf_loc::format_score_row("mean", "ekf", ekf_mean) << '\n';
        std::cout << "wrote " << output << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ekf_loc_benchmark: " << error.what() << '\n';
        return 1;
    }
}
