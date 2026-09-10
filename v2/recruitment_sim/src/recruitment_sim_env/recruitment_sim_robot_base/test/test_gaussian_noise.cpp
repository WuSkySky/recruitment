#include <limits>
#include <stdexcept>

#include <gtest/gtest.h>

#include "recruitment_sim_robot_base/gaussian_noise.hpp"

namespace base = recruitment_sim_robot_base;

TEST(GaussianNoise, ZeroVarianceLeavesValueUnchanged)
{
  base::GaussianNoise noise(0.0);
  for (int i = 0; i < 100; ++i) {
    EXPECT_DOUBLE_EQ(noise.apply(12.5), 12.5);
  }
}

TEST(GaussianNoise, RejectsInvalidVariance)
{
  EXPECT_THROW(base::GaussianNoise(-1.0), std::invalid_argument);
  EXPECT_THROW(
    base::GaussianNoise(std::numeric_limits<double>::infinity()), std::invalid_argument);
}

TEST(GaussianNoise, PositiveVarianceChangesSamples)
{
  base::GaussianNoise noise(1.0);
  bool changed = false;
  for (int i = 0; i < 10; ++i) {
    changed = changed || noise.apply(0.0) != 0.0;
  }
  EXPECT_TRUE(changed);
}
