#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string_view>

#include "imu/math.hpp"

namespace imu {

/// One IMU reading. Units: gyro rad/s, accel m/s^2, mag any (only direction is used).
struct ImuSample {
  double t{0.0};  ///< seconds, monotonic
  Vec3 gyro;
  Vec3 accel;
  std::optional<Vec3> mag;
};

/// World frame: x = magnetic north, y = west, z = up. Body frame: x right, y up, z out of the
/// screen for a phone lying flat (Android sensor convention). At rest accel reads +g on z.
class AttitudeFilter {
 public:
  AttitudeFilter() = default;
  AttitudeFilter(const AttitudeFilter&) = delete;
  AttitudeFilter& operator=(const AttitudeFilter&) = delete;
  AttitudeFilter(AttitudeFilter&&) = delete;
  AttitudeFilter& operator=(AttitudeFilter&&) = delete;
  virtual ~AttitudeFilter() = default;

  /// Feeds one sample. The first sample initialises the attitude from gravity (and mag).
  void update(const ImuSample& s);

  [[nodiscard]] const Quat& orientation() const noexcept { return q_; }
  [[nodiscard]] bool initialized() const noexcept { return initialized_; }
  [[nodiscard]] virtual Vec3 gyroBias() const noexcept { return {}; }
  /// True while this filter's own rest detector says the device is still. Only the EKF detects rest.
  [[nodiscard]] virtual bool stationary() const noexcept { return false; }
  [[nodiscard]] virtual std::string_view name() const noexcept = 0;

 protected:
  virtual void onInit(const ImuSample& s) { (void)s; }
  /// Called when the sample gap exceeds the dropout limit, before the clamped step.
  virtual void onDropout() {}
  virtual void step(const ImuSample& s, double dt) = 0;

  Quat q_;

 private:
  bool initialized_{false};
  double lastT_{0.0};
};

struct ComplementaryParams {
  double tau{1.0};     ///< roll/pitch accelerometer time constant [s]
  double magTau{2.0};  ///< yaw magnetometer time constant [s]
};

struct MahonyParams {
  double kp{1.5};  ///< proportional gain [1/s]
  double ki{0.1};  ///< integral (gyro bias) gain [1/s^2]
};

struct EkfParams {
  double gyroNoise{0.001};          ///< gyro noise density [rad/s/sqrt(Hz)]
  double biasWalk{1e-4};            ///< gyro bias random walk [rad/s/sqrt(s)]
  double accelNoise{0.01};          ///< accel noise density [m/s^2/sqrt(Hz)]
  double magYawStd{0.05};           ///< magnetometer heading noise [rad]
  double initialAttitudeStd{0.2};   ///< [rad]
  double initialBiasStd{0.05};      ///< [rad/s]
  /// Below this rotation rate [rad/s] the bias about gravity is treated as unobservable.
  double stillRate{0.05};
  /// Zero-angular-rate update while the stationary detector reports the device is still.
  bool zupt{true};
  /// Stationary-detector window [s].
  double zuptWindowS{0.5};
  /// Max per-axis standard deviation of the raw gyro while still [rad/s].
  double zuptGyroStd{0.01};
  /// Max |accel| deviation from gravity while still [m/s^2].
  double zuptAccelTol{0.3};
};

class ComplementaryFilter final : public AttitudeFilter {
 public:
  explicit ComplementaryFilter(const ComplementaryParams& p = {}) : p_(p) {}
  [[nodiscard]] std::string_view name() const noexcept override { return "complementary"; }

 private:
  void step(const ImuSample& s, double dt) override;
  ComplementaryParams p_;
};

class MahonyFilter final : public AttitudeFilter {
 public:
  explicit MahonyFilter(const MahonyParams& p = {}) : p_(p) {}
  [[nodiscard]] Vec3 gyroBias() const noexcept override { return bias_; }
  [[nodiscard]] std::string_view name() const noexcept override { return "mahony"; }

 private:
  void step(const ImuSample& s, double dt) override;
  MahonyParams p_;
  Vec3 bias_;
};

/// Error-state EKF: nominal quaternion + gyro bias, 6-dimensional error covariance.
class EkfFilter final : public AttitudeFilter {
 public:
  explicit EkfFilter(const EkfParams& p = {}) : p_(p) {}
  [[nodiscard]] Vec3 gyroBias() const noexcept override { return bias_; }
  [[nodiscard]] bool stationary() const noexcept override { return stationary_; }
  [[nodiscard]] std::string_view name() const noexcept override { return "ekf"; }

 private:
  using Mat6 = Mat<6, 6>;
  struct WindowSample {
    double t{0.0};
    Vec3 gyro;
    Vec3 accel;
  };

  void onInit(const ImuSample& s) override;
  void onDropout() override;
  void step(const ImuSample& s, double dt) override;
  void observe(const ImuSample& s);
  void clearWindow() noexcept;
  void applyZupt();
  /// Kalman update. When @p blindAxis is set, the update must not rotate the attitude or
  /// move the gyro bias about that body-frame axis (it carries no information about it).
  template <std::size_t M>
  void correct(const Mat<M, 6>& h, const Mat<M, 1>& residual, const Mat<M, M>& noise,
               const Vec3* blindAxis = nullptr);

  static constexpr std::size_t kWindowCap = 1024;

  EkfParams p_;
  Vec3 bias_;
  Mat6 cov_;
  std::deque<WindowSample> window_;
  Vec3 zuptMean_;
  double zuptSpan_{0.0};
  bool stationary_{false};
};

enum class FilterKind : std::uint8_t { Complementary, Mahony, Ekf };

[[nodiscard]] std::string_view toString(FilterKind k) noexcept;

/// Runs all three filters on the same stream so they can be compared side by side.
class FilterBank {
 public:
  static constexpr std::size_t kCount = 3;

  struct Config {
    ComplementaryParams complementary;
    MahonyParams mahony;
    EkfParams ekf;
  };

  FilterBank();
  explicit FilterBank(const Config& cfg);

  void update(const ImuSample& s);
  [[nodiscard]] const AttitudeFilter& get(FilterKind k) const noexcept {
    return *filters_[static_cast<std::size_t>(k)];
  }

 private:
  std::array<std::unique_ptr<AttitudeFilter>, kCount> filters_;
};

}  // namespace imu
