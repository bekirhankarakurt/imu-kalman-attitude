#pragma once

#include <cstddef>
#include <cstdint>
#include <random>
#include <string_view>

#include "imu/filters.hpp"
#include "imu/math.hpp"

namespace imu {

enum class Profile { Static, Sway, Aggressive };

[[nodiscard]] std::string_view toString(Profile p) noexcept;

struct SimConfig {
  double rateHz{100.0};
  double durationS{30.0};         ///< <= 0 runs forever
  Profile profile{Profile::Sway};
  double gyroNoise{0.0007};       ///< density [rad/s/sqrt(Hz)]
  double accelNoise{0.005};       ///< density [m/s^2/sqrt(Hz)]
  double magNoise{0.4};           ///< per-sample std [uT]
  Vec3 gyroBias{0.02, -0.015, 0.01};  ///< initial constant bias [rad/s]
  double biasWalk{2e-5};          ///< bias random walk [rad/s/sqrt(s)]
  Vec3 magField{25.0, 0.0, -37.0};    ///< world-frame field [uT]
  bool magnetometer{true};
  std::uint64_t seed{1};
};

struct SimSample {
  ImuSample imu;
  Quat truth;
  Vec3 trueBias;
};

/// Deterministic IMU simulator: smooth ground-truth motion plus gyro bias/random walk,
/// white noise on every sensor and (for moving profiles) linear acceleration.
class ImuSimulator {
 public:
  explicit ImuSimulator(const SimConfig& cfg);

  /// Produces the next sample; returns false once the configured duration is exceeded.
  [[nodiscard]] bool next(SimSample& out);

  [[nodiscard]] static Quat truthAt(Profile p, double t) noexcept;

 private:
  [[nodiscard]] double uniform();
  [[nodiscard]] double gauss();

  SimConfig cfg_;
  std::size_t index_{0};
  Vec3 bias_;
  std::mt19937_64 rng_;
  double spare_{0.0};
  bool hasSpare_{false};
};

}  // namespace imu
