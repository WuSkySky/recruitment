#include <cmath>

#include <gtest/gtest.h>

#include "recruitment_sim_robot_base/incremental_odometry.hpp"

namespace base = recruitment_sim_robot_base;

TEST(IncrementalOdometry, ZeroNoiseReproducesGroundTruthIncrement)
{
  const base::PlanarPose last_gt{1.0, 2.0, M_PI_2};
  const base::PlanarPose gt{1.0, 3.0, M_PI_2};
  const base::PlanarPose last_odom{4.0, 5.0, M_PI_2};
  const auto odom = base::integrate_odometry_increment(last_gt, gt, last_odom, 0.0, 0.0, 0.0);
  EXPECT_NEAR(odom.x, 4.0, 1e-12);
  EXPECT_NEAR(odom.y, 6.0, 1e-12);
  EXPECT_NEAR(odom.yaw, M_PI_2, 1e-12);
}

TEST(IncrementalOdometry, HeadingDriftRotatesLaterDisplacement)
{
  const base::PlanarPose last_gt{0.0, 0.0, M_PI_2};
  const base::PlanarPose gt{0.0, 1.0, M_PI_2};
  const base::PlanarPose last_odom{0.0, 0.0, 100.0 * M_PI / 180.0};
  const auto odom = base::integrate_odometry_increment(last_gt, gt, last_odom, 0.0, 0.0, 0.0);
  EXPECT_NEAR(odom.x, std::cos(100.0 * M_PI / 180.0), 1e-12);
  EXPECT_NEAR(odom.y, std::sin(100.0 * M_PI / 180.0), 1e-12);
}

TEST(IncrementalOdometry, AddsNoiseInBodyFrame)
{
  const base::PlanarPose pose{0.0, 0.0, M_PI_2};
  const auto odom = base::integrate_odometry_increment(pose, pose, pose, 1.0, 2.0, 0.1);
  EXPECT_NEAR(odom.x, -2.0, 1e-12);
  EXPECT_NEAR(odom.y, 1.0, 1e-12);
  EXPECT_NEAR(odom.yaw, M_PI_2 + 0.1, 1e-12);
}
