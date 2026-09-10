#ifndef RECRUITMENT_SIM_DIRECTION_NOISE_HH_
#define RECRUITMENT_SIM_DIRECTION_NOISE_HH_

#include <cmath>
#include <random>
#include <stdexcept>
#include <ignition/math/Vector3.hh>

namespace recruitment_sim
{
// Independent per-shot angular errors in the muzzle frame (radians).
class DirectionNoise
{
public:
  explicit DirectionNoise(unsigned int seed = std::random_device{}()) : engine_(seed) {}

  void Configure(double yawVariance, double pitchVariance)
  {
    if (!std::isfinite(yawVariance) || yawVariance < 0.0 ||
        !std::isfinite(pitchVariance) || pitchVariance < 0.0) {
      throw std::invalid_argument("projectile direction variances must be finite and non-negative");
    }
    yawStddev_ = std::sqrt(yawVariance);
    pitchStddev_ = std::sqrt(pitchVariance);
  }

  ignition::math::Vector3d Sample(double speed)
  {
    // Zero variance does not consume random samples or construct a degenerate distribution.
    const double yaw = yawStddev_ == 0.0 ? 0.0 : yawStddev_ * normal_(engine_);
    const double pitch = pitchStddev_ == 0.0 ? 0.0 : pitchStddev_ * normal_(engine_);
    return {speed * std::cos(pitch) * std::cos(yaw),
      speed * std::cos(pitch) * std::sin(yaw), speed * std::sin(pitch)};
  }

private:
  std::mt19937 engine_;
  std::normal_distribution<double> normal_{0.0, 1.0};
  double yawStddev_{0.0};
  double pitchStddev_{0.0};
};
}  // namespace recruitment_sim
#endif
