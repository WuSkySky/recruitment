#include <cmath>
#include <limits>
#include <gtest/gtest.h>
#include "../plugins/projectile_shooter/DirectionNoise.hh"

TEST(DirectionNoise, DefaultsAndZeroAreExact)
{
  recruitment_sim::DirectionNoise noise(1);
  EXPECT_EQ(noise.Sample(18), ignition::math::Vector3d(18, 0, 0));
  noise.Configure(0, 0);
  for (int i = 0; i < 100; ++i) {
    EXPECT_EQ(noise.Sample(18), ignition::math::Vector3d(18, 0, 0));
  }
}

TEST(DirectionNoise, RejectsInvalidVariances)
{
  recruitment_sim::DirectionNoise noise(1);
  for (double invalid : {-1.0, std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()}) {
    EXPECT_THROW(noise.Configure(invalid, 0), std::invalid_argument);
    EXPECT_THROW(noise.Configure(0, invalid), std::invalid_argument);
  }
}

TEST(DirectionNoise, AxesCanBeDisabledIndependently)
{
  recruitment_sim::DirectionNoise noise(1);
  noise.Configure(0.0001, 0);
  EXPECT_NE(noise.Sample(18).Y(), 0);
  EXPECT_EQ(noise.Sample(18).Z(), 0);
  noise.Configure(0, 0.0001);
  EXPECT_EQ(noise.Sample(18).Y(), 0);
  EXPECT_NE(noise.Sample(18).Z(), 0);
}

TEST(DirectionNoise, AngularStatisticsAndSpeed)
{
  recruitment_sim::DirectionNoise noise(12345);
  noise.Configure(0.0001, 0.0004);
  constexpr int count = 100000;
  double yawSum = 0, pitchSum = 0, yawSquared = 0, pitchSquared = 0, cross = 0;
  for (int i = 0; i < count; ++i) {
    const auto velocity = noise.Sample(18);
    EXPECT_NEAR(velocity.Length(), 18, 1e-12);
    const double yaw = std::atan2(velocity.Y(), velocity.X());
    const double pitch = std::atan2(velocity.Z(), std::hypot(velocity.X(), velocity.Y()));
    yawSum += yaw;
    pitchSum += pitch;
    yawSquared += yaw * yaw;
    pitchSquared += pitch * pitch;
    cross += yaw * pitch;
  }
  EXPECT_NEAR(yawSum / count, 0, 0.0002);
  EXPECT_NEAR(pitchSum / count, 0, 0.0004);
  EXPECT_NEAR(yawSquared / count, 0.0001, 0.000003);
  EXPECT_NEAR(pitchSquared / count, 0.0004, 0.000012);
  EXPECT_NEAR(cross / count, 0, 0.000004);
}
