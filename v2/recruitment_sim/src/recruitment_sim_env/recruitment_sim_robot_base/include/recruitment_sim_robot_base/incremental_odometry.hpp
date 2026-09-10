#ifndef RECRUITMENT_SIM_ROBOT_BASE__INCREMENTAL_ODOMETRY_HPP_
#define RECRUITMENT_SIM_ROBOT_BASE__INCREMENTAL_ODOMETRY_HPP_

#include <cmath>

namespace recruitment_sim_robot_base
{

struct PlanarPose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline double normalize_angle(double angle)
{
  constexpr double kTwoPi = 6.28318530717958647692;
  return std::remainder(angle, kTwoPi);
}

inline PlanarPose integrate_odometry_increment(
  const PlanarPose & last_ground_truth,
  const PlanarPose & ground_truth,
  const PlanarPose & last_odometry,
  double forward_noise,
  double lateral_noise,
  double yaw_noise)
{
  const double world_dx = ground_truth.x - last_ground_truth.x;
  const double world_dy = ground_truth.y - last_ground_truth.y;

  const double gt_cos = std::cos(last_ground_truth.yaw);
  const double gt_sin = std::sin(last_ground_truth.yaw);
  const double forward = gt_cos * world_dx + gt_sin * world_dy + forward_noise;
  const double lateral = -gt_sin * world_dx + gt_cos * world_dy + lateral_noise;

  const double odom_cos = std::cos(last_odometry.yaw);
  const double odom_sin = std::sin(last_odometry.yaw);
  PlanarPose odometry;
  odometry.x = last_odometry.x + odom_cos * forward - odom_sin * lateral;
  odometry.y = last_odometry.y + odom_sin * forward + odom_cos * lateral;
  odometry.yaw = normalize_angle(
    last_odometry.yaw + normalize_angle(ground_truth.yaw - last_ground_truth.yaw) + yaw_noise);
  return odometry;
}

}  // namespace recruitment_sim_robot_base

#endif  // RECRUITMENT_SIM_ROBOT_BASE__INCREMENTAL_ODOMETRY_HPP_
