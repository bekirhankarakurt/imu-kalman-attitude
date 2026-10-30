#include <array>

#include "imu/evaluation.hpp"
#include "imu/filters.hpp"
#include "imu/simulator.hpp"
#include "test_framework.hpp"

using namespace imu;

namespace {

struct Result {
  std::array<ErrorAccumulator, FilterBank::kCount> errors;
  Vec3 biasError[FilterBank::kCount];
};

Result simulate(const SimConfig& cfg, double warmup = 10.0) {
  Result r;
  ImuSimulator sim(cfg);
  FilterBank bank;
  SimSample s;
  while (sim.next(s)) {
    bank.update(s.imu);
    if (s.imu.t < warmup) continue;
    for (std::size_t k = 0; k < FilterBank::kCount; ++k) {
      const AttitudeFilter& f = bank.get(static_cast<FilterKind>(k));
      r.errors[k].add(f.orientation(), s.truth);
      r.biasError[k] = f.gyroBias() - s.trueBias;
    }
  }
  return r;
}

constexpr std::size_t kComp = static_cast<std::size_t>(FilterKind::Complementary);
constexpr std::size_t kMahony = static_cast<std::size_t>(FilterKind::Mahony);
constexpr std::size_t kEkf = static_cast<std::size_t>(FilterKind::Ekf);

}  // namespace

TEST(sway_with_magnetometer) {
  SimConfig cfg;
  cfg.durationS = 60.0;
  const Result r = simulate(cfg);
  CHECK_LT(r.errors[kEkf].attitudeRmseDeg(), 1.0);
  CHECK_LT(r.errors[kMahony].attitudeRmseDeg(), 1.5);
  CHECK_LT(r.errors[kComp].attitudeRmseDeg(), 2.5);
  CHECK_LT(radToDeg(r.biasError[kEkf].norm()), 0.1);
  CHECK_LT(radToDeg(r.biasError[kMahony].norm()), 0.3);
}

TEST(sway_without_magnetometer_roll_and_pitch) {
  SimConfig cfg;
  cfg.durationS = 60.0;
  cfg.magnetometer = false;
  const Result r = simulate(cfg);
  for (std::size_t k = 0; k < FilterBank::kCount; ++k) {
    CHECK_LT(r.errors[k].rollRmseDeg(), 1.5);
    CHECK_LT(r.errors[k].pitchRmseDeg(), 1.5);
  }
  CHECK_LT(r.errors[kEkf].rollRmseDeg(), 0.8);
}

TEST(aggressive_motion_with_linear_acceleration) {
  SimConfig cfg;
  cfg.profile = Profile::Aggressive;
  cfg.durationS = 60.0;
  const Result r = simulate(cfg);
  CHECK_LT(r.errors[kEkf].attitudeRmseDeg(), 4.0);
  CHECK_LT(r.errors[kComp].attitudeRmseDeg(), 4.0);
  CHECK_LT(r.errors[kMahony].attitudeRmseDeg(), 6.0);
  CHECK_LT(radToDeg(r.biasError[kEkf].norm()), 1.5);
}

TEST(static_pose_is_estimated_precisely) {
  SimConfig cfg;
  cfg.profile = Profile::Static;
  cfg.durationS = 40.0;
  const Result r = simulate(cfg);
  CHECK_LT(r.errors[kEkf].attitudeRmseDeg(), 0.2);
  CHECK_LT(r.errors[kMahony].attitudeRmseDeg(), 0.6);
  CHECK_LT(radToDeg(r.biasError[kEkf].norm()), 0.1);
}

TEST(ekf_beats_uncorrected_gyro_integration) {
  SimConfig cfg;
  cfg.durationS = 60.0;
  cfg.magnetometer = false;
  ImuSimulator sim(cfg);
  SimSample s;
  Quat open;
  bool first = true;
  double lastT = 0.0;
  ErrorAccumulator openLoop;
  EkfFilter ekf;
  ErrorAccumulator filtered;
  while (sim.next(s)) {
    ekf.update(s.imu);
    if (first) {
      open = ekf.orientation();
      first = false;
    } else {
      open = (open * Quat::fromRotationVector(s.imu.gyro * (s.imu.t - lastT))).normalized();
    }
    lastT = s.imu.t;
    if (s.imu.t > 30.0) {
      openLoop.add(open, s.truth);
      filtered.add(ekf.orientation(), s.truth);
    }
  }
  CHECK_LT(filtered.rollRmseDeg() * 5.0, openLoop.rollRmseDeg());
}

TEST(simulation_is_deterministic) {
  SimConfig cfg;
  cfg.durationS = 5.0;
  cfg.seed = 42;
  ImuSimulator a(cfg), b(cfg);
  SimSample sa, sb;
  std::size_t n = 0;
  while (a.next(sa)) {
    CHECK(b.next(sb));
    CHECK(sa.imu.gyro.x == sb.imu.gyro.x);
    CHECK(sa.imu.accel.z == sb.imu.accel.z);
    ++n;
  }
  CHECK(!b.next(sb));
  CHECK(n == 501);
  cfg.seed = 43;
  ImuSimulator c(cfg);
  SimSample sc;
  CHECK(c.next(sc));
  CHECK(sc.imu.gyro.x != sa.imu.gyro.x);
}

TEST(ground_truth_gyro_matches_attitude_derivative) {
  SimConfig cfg;
  cfg.gyroNoise = 0.0;
  cfg.accelNoise = 0.0;
  cfg.magNoise = 0.0;
  cfg.biasWalk = 0.0;
  cfg.gyroBias = {};
  cfg.durationS = 20.0;
  ImuSimulator sim(cfg);
  SimSample s;
  Quat q;
  bool first = true;
  double lastT = 0.0;
  double worst = 0.0;
  while (sim.next(s)) {
    if (first) {
      q = s.truth;
      first = false;
    } else {
      q = (q * Quat::fromRotationVector(s.imu.gyro * (s.imu.t - lastT))).normalized();
      worst = std::max(worst, radToDeg((s.truth.conjugate() * q).toRotationVector().norm()));
    }
    lastT = s.imu.t;
  }
  CHECK_LT(worst, 0.5);  // only integration error of a 100 Hz rectangular rule remains
}
