#include "imu/evaluation.hpp"

#include <cmath>

namespace imu {

void ErrorAccumulator::add(const Quat& estimate, const Quat& truth) noexcept {
  const Euler e = estimate.toEuler();
  const Euler t = truth.toEuler();
  const double dr = radToDeg(wrapPi(e.roll - t.roll));
  const double dp = radToDeg(wrapPi(e.pitch - t.pitch));
  const double dy = radToDeg(wrapPi(e.yaw - t.yaw));
  const double da = radToDeg((truth.conjugate() * estimate).toRotationVector().norm());
  roll_ += dr * dr;
  pitch_ += dp * dp;
  yaw_ += dy * dy;
  angle_ += da * da;
  ++n_;
}

double ErrorAccumulator::rmse(double sumSquares) const noexcept {
  return n_ == 0 ? 0.0 : std::sqrt(sumSquares / static_cast<double>(n_));
}

}  // namespace imu
