#include "trajectory.hpp"

#include <cmath>
#include <limits>

namespace tools
{
constexpr double g = 9.7833;

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  // Initialize to prevent garbage data if unsolvable
  pitch = 0.0;
  fly_time = 0.0;
  unsolvable = false;

  // Guard unrealistic/degenerate inputs; let caller fallback
  if (v0 <= 0.5 || d <= 1e-6 || !std::isfinite(v0) || !std::isfinite(d) || !std::isfinite(h)) {
    unsolvable = true;
    return;
  }

  const double a = g * d * d / (2.0 * v0 * v0);
  const double b = -d;
  const double c = a + h;

  // Avoid division by zero later
  if (std::abs(a) < 1e-12) {
    unsolvable = true;
    return;
  }

  double delta = b * b - 4.0 * a * c;
  if (delta < 0) {
    // Clamp tiny negatives caused by floating-point round-off
    if (delta > -1e-10) {
      delta = 0;
    } else {
      unsolvable = true;
      return;
    }
  }

  const double denom = 2.0 * a;
  const double tan_pitch_1 = (-b + std::sqrt(delta)) / denom;
  const double tan_pitch_2 = (-b - std::sqrt(delta)) / denom;

  const double pitch_1 = std::atan(tan_pitch_1);
  const double pitch_2 = std::atan(tan_pitch_2);

  const double cos1 = std::cos(pitch_1);
  const double cos2 = std::cos(pitch_2);

  // Treat near-vertical aim as invalid
  const double eps = 1e-6;
  const double t_inf = std::numeric_limits<double>::infinity();
  double t_1 = (std::abs(cos1) < eps) ? t_inf : d / (v0 * cos1);
  double t_2 = (std::abs(cos2) < eps) ? t_inf : d / (v0 * cos2);

  // Accept only positive, finite flight times
  if (!(std::isfinite(t_1) && t_1 > 0) && !(std::isfinite(t_2) && t_2 > 0)) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  if (!(std::isfinite(t_1) && t_1 > 0) || (std::isfinite(t_2) && t_2 > 0 && t_2 < t_1)) {
    pitch = pitch_2;
    fly_time = t_2;
  } else {
    pitch = pitch_1;
    fly_time = t_1;
  }
}

}  // namespace tools
