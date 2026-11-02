#include <gtest/gtest.h>

#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

#ifndef EKFLOC_BENCHMARK_EXE
#error "EKFLOC_BENCHMARK_EXE must name the ekf_loc_benchmark executable"
#endif

namespace {

namespace fs = std::filesystem;

std::string read_file(const fs::path& path) {
    std::ifstream in(path);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

// Runs ekf_loc_benchmark with its three output files in a fresh directory.
class BenchmarkCli : public ::testing::Test {
protected:
    void SetUp() override {
        const std::string name = ::testing::UnitTest::GetInstance()->current_test_info()->name();
        dir_ = fs::temp_directory_path() / ("ekfloc_cli_" + name + "_" + std::to_string(::getpid()));
        fs::remove_all(dir_);
        fs::create_directories(dir_);
    }

    void TearDown() override { fs::remove_all(dir_); }

    // Exit code of the benchmark, or -1 if it did not exit normally.
    int run(const std::string& arguments) const {
        const std::string command = std::string("'") + EKFLOC_BENCHMARK_EXE + "' " + arguments + " --output '" +
                                    summary().string() + "' --trace '" + trace().string() + "' --landmarks '" +
                                    landmarks().string() + "' > /dev/null 2>&1";
        const int status = std::system(command.c_str());
        if (status == -1 || !WIFEXITED(status)) {
            return -1;
        }
        return WEXITSTATUS(status);
    }

    void write_previous_summary() const {
        std::ofstream out(summary());
        out << "previous summary\n";
    }

    fs::path summary() const { return dir_ / "benchmark.csv"; }
    fs::path trace() const { return dir_ / "trace.csv"; }
    fs::path landmarks() const { return dir_ / "landmarks.csv"; }

private:
    fs::path dir_;
};

TEST_F(BenchmarkCli, WritesSummaryTraceAndLandmarks) {
    ASSERT_EQ(run("--seeds 1 --first-seed 3"), 0);
    const std::string text = read_file(summary());
    EXPECT_EQ(text.rfind("seed,method,rmse_position_m,rmse_heading_deg,final_position_error_m,"
                         "final_heading_error_deg,mean_nees\n",
                         0),
              0u);
    EXPECT_NE(text.find("\n3,dead_reckoning,"), std::string::npos);
    EXPECT_NE(text.find("\n3,ekf,"), std::string::npos);
    EXPECT_NE(text.find("\nmean,ekf,"), std::string::npos);
    EXPECT_TRUE(fs::exists(trace()));
    EXPECT_TRUE(fs::exists(landmarks()));
}

// A bad --trace-seed used to be noticed only after every seed had run, by
// which time the summary had been truncated to the rows written so far.
TEST_F(BenchmarkCli, TraceSeedOutsideTheRunLeavesTheSummaryAlone) {
    write_previous_summary();
    EXPECT_EQ(run("--seeds 2 --trace-seed 9"), 1);
    EXPECT_EQ(read_file(summary()), "previous summary\n");
    EXPECT_FALSE(fs::exists(trace()));
    EXPECT_FALSE(fs::exists(landmarks()));
}

// "-1" used to parse as 2^64 - 1 and 2^32 as zero seeds once cast to int.
TEST_F(BenchmarkCli, BadSeedCountsLeaveTheSummaryAlone) {
    write_previous_summary();
    for (const char* count : {"-1", "0", "4294967296", "2x"}) {
        EXPECT_EQ(run(std::string("--seeds ") + count), 1) << count;
        EXPECT_EQ(read_file(summary()), "previous summary\n") << count;
    }
}

}  // namespace
