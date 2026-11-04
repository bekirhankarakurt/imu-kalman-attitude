#include "imu/filters.hpp"
#include "imu/simulator.hpp"
#include "test_framework.hpp"

using namespace imu;

namespace {

ImuSample staticSample(double t, const Euler& e, bool mag) {
  const Quat q = Quat::fromEuler(e);
  ImuSample s;
  s.t = t;
  s.accel = q.rotateInverse({0.0, 0.0, kGravity});
  if (mag) s.mag = q.rotateInverse({25.0, 0.0, -37.0});
  return s;
}

double angleBetween(const Quat& a, const Quat& b) {
  return radToDeg((a.conjugate() * b).toRotationVector().norm());
}

}  // namespace

TEST(first_sample_initialises_from_gravity_and_mag) {
  const Euler e{degToRad(20.0), degToRad(-15.0), degToRad(75.0)};
  FilterBank bank;
  bank.update(staticSample(0.0, e, true));
  for (std::size_t k = 0; k < FilterBank::kCount; ++k) {
    const Euler got = bank.get(static_cast<FilterKind>(k)).orientation().toEuler();
    CHECK_NEAR(got.roll, e.roll, 1e-9);
    CHECK_NEAR(got.pitch, e.pitch, 1e-9);
    CHECK_NEAR(got.yaw, e.yaw, 1e-9);
  }
}

TEST(without_mag_yaw_starts_at_zero) {
  FilterBank bank;
  bank.update(staticSample(0.0, {0.1, 0.2, 1.0}, false));
  CHECK_NEAR(bank.get(FilterKind::Ekf).orientation().toEuler().yaw, 0.0, 1e-9);
}

TEST(filters_recover_from_wrong_initial_attitude) {
  const Euler truth{degToRad(30.0), degToRad(-20.0), degToRad(50.0)};
  for (std::size_t k = 0; k < FilterBank::kCount; ++k) {
    FilterBank bank;
    bank.update(staticSample(0.0, {}, true));  // starts level, facing north
    double t = 0.0;
    for (int i = 0; i < 2000; ++i) {
      t += 0.01;
      ImuSample s = staticSample(t, truth, true);
      bank.update(s);
    }
    CHECK_LT(angleBetween(bank.get(static_cast<FilterKind>(k)).orientation(), Quat::fromEuler(truth)), 1.0);
  }
}

TEST(gyro_integration_tracks_constant_rotation) {
  ComplementaryFilter f;
  ImuSample s = staticSample(0.0, {}, false);
  f.update(s);
  // Rotate about body z at 0.5 rad/s with gravity unchanged: yaw is only observable via gyro.
  for (int i = 1; i <= 200; ++i) {
    s.t = 0.01 * i;
    s.gyro = {0.0, 0.0, 0.5};
    f.update(s);
  }
  CHECK_NEAR(f.orientation().toEuler().yaw, 1.0, 1e-6);
}

TEST(accelerometer_in_free_fall_is_ignored) {
  EkfFilter f;
  ImuSample s = staticSample(0.0, {0.3, 0.1, 0.0}, false);
  f.update(s);
  const Quat before = f.orientation();
  s.accel = {0.0, 0.0, 0.0};
  s.t = 0.01;
  f.update(s);
  CHECK_LT(angleBetween(before, f.orientation()), 1e-9);
  s.accel = {50.0, 0.0, 0.0};  // violent shock: far outside the trust gate
  s.t = 0.02;
  f.update(s);
  CHECK_LT(angleBetween(before, f.orientation()), 1e-9);
}

TEST(non_increasing_timestamps_are_ignored) {
  MahonyFilter f;
  ImuSample s = staticSample(1.0, {}, false);
  f.update(s);
  s.t = 1.0;
  s.gyro = {5.0, 0.0, 0.0};
  f.update(s);
  s.t = 0.5;
  f.update(s);
  CHECK_NEAR(f.orientation().w, 1.0, 1e-12);
}

TEST(ekf_estimates_constant_gyro_bias_when_static) {
  SimConfig cfg;
  cfg.profile = Profile::Static;
  cfg.durationS = 60.0;
  cfg.biasWalk = 0.0;
  cfg.gyroBias = {0.03, -0.02, 0.015};
  ImuSimulator sim(cfg);
  EkfFilter ekf;
  SimSample s;
  while (sim.next(s)) ekf.update(s.imu);
  // Static pose with heading from the magnetometer makes all three axes observable.
  CHECK_NEAR(ekf.gyroBias().x, 0.03, 0.003);
  CHECK_NEAR(ekf.gyroBias().y, -0.02, 0.003);
  CHECK_NEAR(ekf.gyroBias().z, 0.015, 0.003);
}
