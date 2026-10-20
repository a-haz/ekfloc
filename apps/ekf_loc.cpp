#include "csv_export.hpp"

#include "ekf_loc/simulator.hpp"

#include <cstdint>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

void print_help() {
    std::cout
        << "ekf_loc: simulated differential-drive EKF localization\n"
        << "\n"
        << "Fuse wheel odometry with range-bearing observations of known landmarks.\n"
        << "The true robot follows a fixed rounded-square course. Odometry and\n"
        << "landmark observations are corrupted with configurable Gaussian noise.\n"
        << "\n"
        << "Usage:\n"
        << "  ekf_loc run [options]\n"
        << "\n"
        << "Options:\n"
        << "  --seed <int>               RNG seed (default 1)\n"
        << "  --dt <sec>                 Integration step (default 0.05)\n"
        << "  --duration <sec>           Simulated duration (default 56)\n"
        << "  --meas-period <sec>        Range-bearing period (default 0.2)\n"
        << "  --q-v <m^2/s^3>            Linear-velocity noise density (default 0.02)\n"
        << "  --q-omega <rad^2/s^3>      Yaw-rate noise density (default 0.004)\n"
        << "  --sigma-range <m>          Range noise std (default 0.08)\n"
        << "  --sigma-bearing-deg <deg>  Bearing noise std (default 2)\n"
        << "  --max-range <m>            Sensor maximum range (default 12)\n"
        << "  --output <file>            Trace CSV. Omit to print scores only.\n"
        << "  --help                     Show this text\n";
}

double require_double(const std::string& text, const std::string& flag) {
    try {
        std::size_t used = 0;
        const double value = std::stod(text, &used);
        if (used != text.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return value;
    } catch (const std::exception&) {
        throw std::runtime_error("invalid number for " + flag + ": " + text);
    }
}

unsigned long long require_ull(const std::string& text, const std::string& flag) {
    try {
        std::size_t used = 0;
        const unsigned long long value = std::stoull(text, &used);
        if (used != text.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return value;
    } catch (const std::exception&) {
        throw std::runtime_error("invalid integer for " + flag + ": " + text);
    }
}

void print_score(const std::string& name, const ekf_loc::MethodScore& score) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << name << "  rmse_position_m=" << score.rmse_position_m
              << "  rmse_heading_deg=" << score.rmse_heading_deg
              << "  final_position_error_m=" << score.final_position_error_m
              << "  final_heading_error_deg=" << score.final_heading_error_deg
              << "  mean_nees=" << score.mean_nees << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    try {
        if (argc < 2) {
            print_help();
            return 0;
        }
        const std::string command = argv[1];
        if (command == "--help" || command == "-h" || command == "help") {
            print_help();
            return 0;
        }
        if (command != "run") {
            throw std::runtime_error("unknown command '" + command + "'. Try: ekf_loc run --help");
        }

        ekf_loc::SimConfig config = ekf_loc::benchmark_config(1);
        std::string output;
        for (int i = 2; i < argc; ++i) {
            const std::string flag = argv[i];
            if (flag == "--help" || flag == "-h") {
                print_help();
                return 0;
            }
            if (i + 1 >= argc) {
                throw std::runtime_error("missing value for " + flag);
            }
            const std::string value = argv[++i];
            if (flag == "--seed") {
                config.seed = static_cast<std::uint64_t>(require_ull(value, flag));
            } else if (flag == "--dt") {
                config.dt = require_double(value, flag);
            } else if (flag == "--duration") {
                config.duration = require_double(value, flag);
            } else if (flag == "--meas-period") {
                config.measurement_period = require_double(value, flag);
            } else if (flag == "--q-v") {
                config.noise.velocity_density = require_double(value, flag);
            } else if (flag == "--q-omega") {
                config.noise.yaw_rate_density = require_double(value, flag);
            } else if (flag == "--sigma-range") {
                config.noise.sigma_range = require_double(value, flag);
            } else if (flag == "--sigma-bearing-deg") {
                config.noise.sigma_bearing = require_double(value, flag) * ekf_loc::kDegToRad;
            } else if (flag == "--max-range") {
                config.max_range = require_double(value, flag);
            } else if (flag == "--output") {
                output = value;
            } else {
                throw std::runtime_error("unknown option " + flag);
            }
        }

        const ekf_loc::SimulationResult result = ekf_loc::Simulator(config).run();
        print_score("dead_reckoning", result.dead_reckoning);
        print_score("ekf            ", result.ekf);
        std::cout << std::fixed << std::setprecision(6);
        std::cout << "measurements=" << result.measurement_count << "  mean_nis=" << result.mean_nis
                  << '\n';
        if (!output.empty()) {
            ekf_loc::write_trace_csv(output, result.samples);
            std::cout << "wrote " << output << '\n';
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "ekf_loc: " << error.what() << '\n';
        return 1;
    }
}
