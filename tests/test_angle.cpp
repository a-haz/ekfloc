#include "ekf_loc/angle.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace {

TEST(WrapAngle, ZeroStaysZero) { EXPECT_DOUBLE_EQ(ekf_loc::wrap_angle(0.0), 0.0); }

TEST(WrapAngle, PositiveRightAngle) {
    EXPECT_NEAR(ekf_loc::wrap_angle(ekf_loc::kPi / 2.0), ekf_loc::kPi / 2.0, 1e-15);
}

TEST(WrapAngle, NegativeRightAngle) {
    EXPECT_NEAR(ekf_loc::wrap_angle(-ekf_loc::kPi / 2.0), -ekf_loc::kPi / 2.0, 1e-15);
}

TEST(WrapAngle, PiMapsIntoRange) {
    const double wrapped = ekf_loc::wrap_angle(ekf_loc::kPi);
    EXPECT_NEAR(std::sin(wrapped), 0.0, 1e-15);
    EXPECT_NEAR(std::cos(wrapped), -1.0, 1e-15);
    EXPECT_GE(wrapped, -ekf_loc::kPi);
    EXPECT_LT(wrapped, ekf_loc::kPi);
}

TEST(WrapAngle, NegativePiUnchanged) {
    EXPECT_NEAR(ekf_loc::wrap_angle(-ekf_loc::kPi), -ekf_loc::kPi, 1e-15);
}

TEST(WrapAngle, TwoPiMapsToZero) {
    EXPECT_NEAR(ekf_loc::wrap_angle(2.0 * ekf_loc::kPi), 0.0, 1e-12);
}

TEST(WrapAngle, NegativeTwoPiMapsToZero) {
    EXPECT_NEAR(ekf_loc::wrap_angle(-2.0 * ekf_loc::kPi), 0.0, 1e-12);
}

TEST(WrapAngle, ThreeHalfPiMapsToNegativeHalfPi) {
    EXPECT_NEAR(ekf_loc::wrap_angle(1.5 * ekf_loc::kPi), -0.5 * ekf_loc::kPi, 1e-12);
}

TEST(WrapAngle, NegativeThreeHalfPiMapsToHalfPi) {
    EXPECT_NEAR(ekf_loc::wrap_angle(-1.5 * ekf_loc::kPi), 0.5 * ekf_loc::kPi, 1e-12);
}

TEST(WrapAngle, LargePositive) {
    const double wrapped = ekf_loc::wrap_angle(100.0);
    EXPECT_GE(wrapped, -ekf_loc::kPi);
    EXPECT_LT(wrapped, ekf_loc::kPi);
    EXPECT_NEAR(std::sin(wrapped), std::sin(100.0), 1e-12);
    EXPECT_NEAR(std::cos(wrapped), std::cos(100.0), 1e-12);
}

TEST(WrapAngle, LargeNegative) {
    const double wrapped = ekf_loc::wrap_angle(-100.0);
    EXPECT_GE(wrapped, -ekf_loc::kPi);
    EXPECT_LT(wrapped, ekf_loc::kPi);
    EXPECT_NEAR(std::sin(wrapped), std::sin(-100.0), 1e-12);
}

TEST(WrapAngle, IsIdempotent) {
    const double once = ekf_loc::wrap_angle(4.2);
    EXPECT_NEAR(ekf_loc::wrap_angle(once), once, 1e-15);
}

}  // namespace
