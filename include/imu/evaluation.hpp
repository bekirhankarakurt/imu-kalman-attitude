#pragma once

#include <cstddef>

#include "imu/math.hpp"

namespace imu {

/// Accumulates attitude error of an estimate against ground truth.
class ErrorAccumulator {
 public:
  void add(const Quat& estimate, const Quat& truth) noexcept;

  [[nodiscard]] std::size_t count() const noexcept { return n_; }
  [[nodiscard]] double rollRmseDeg() const noexcept { return rmse(roll_); }
  [[nodiscard]] double pitchRmseDeg() const noexcept { return rmse(pitch_); }
  [[nodiscard]] double yawRmseDeg() const noexcept { return rmse(yaw_); }
  /// RMSE of the total rotation angle between estimate and truth.
  [[nodiscard]] double attitudeRmseDeg() const noexcept { return rmse(angle_); }

 private:
  [[nodiscard]] double rmse(double sumSquares) const noexcept;

  std::size_t n_{0};
  double roll_{0.0};
  double pitch_{0.0};
  double yaw_{0.0};
  double angle_{0.0};
};

}  // namespace imu
