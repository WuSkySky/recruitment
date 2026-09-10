#ifndef RECRUITMENT_SIM_ROBOT_BASE__GAUSSIAN_NOISE_HPP_
#define RECRUITMENT_SIM_ROBOT_BASE__GAUSSIAN_NOISE_HPP_

#include <cmath>
#include <random>
#include <stdexcept>

namespace recruitment_sim_robot_base
{

class GaussianNoise
{
public:
  explicit GaussianNoise(double variance)
  : distribution_(0.0, standard_deviation(variance)) {}

  double apply(double value)
  {
    return distribution_.stddev() == 0.0 ? value : value + distribution_(engine_);
  }

private:
  static double standard_deviation(double variance)
  {
    if (!std::isfinite(variance) || variance < 0.0) {
      throw std::invalid_argument("noise variance must be finite and non-negative");
    }
    return std::sqrt(variance);
  }

  std::mt19937 engine_{std::random_device{}()};
  std::normal_distribution<double> distribution_;
};

}  // namespace recruitment_sim_robot_base

#endif  // RECRUITMENT_SIM_ROBOT_BASE__GAUSSIAN_NOISE_HPP_
