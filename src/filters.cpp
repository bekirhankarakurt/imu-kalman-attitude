#include "imu/filters.hpp"

#include <algorithm>
#include <cmath>

namespace imu {
namespace {

constexpr double kMaxDt = 0.5;       // longer gaps are treated as dropouts
constexpr double kGateWidth = 0.4;   // accel trusted fully at |a|=g, not at all at |a|=1.4g
constexpr Vec3 kUp{0.0, 0.0, 1.0};

struct Tilt {
  Vec3 expected;  // world "up" seen from the body frame, from the current estimate
  Vec3 measured;  // normalised accelerometer reading
  double weight;  // 0..1 trust in this reading

  [[nodiscard]] Vec3 error() const { return cross(measured, expected); }
};

[[nodiscard]] std::optional<Tilt> tiltOf(const Quat& q, const Vec3& accel) {
  const double n = accel.norm();
  if (n < 1e-3) return std::nullopt;
  const double dev = std::abs(n - kGravity) / kGravity;
  const double weight = std::clamp(1.0 - dev / kGateWidth, 0.0, 1.0);
  if (weight < 0.01) return std::nullopt;
  return Tilt{q.rotateInverse(kUp), accel / n, weight};
}

/// Heading error (rad) between the tilt-compensated magnetometer and the estimate.
[[nodiscard]] std::optional<double> yawErrorOf(const Quat& q, const Vec3& mag) {
  const double yaw = q.toEuler().yaw;
  const Quat level = Quat::fromRotationVector({0.0, 0.0, -yaw}) * q;
  const Vec3 m = level.rotate(mag);
  if (std::hypot(m.x, m.y) < 1e-9) return std::nullopt;
  return wrapPi(std::atan2(-m.y, m.x) - yaw);
}

[[nodiscard]] Quat initialOrientation(const ImuSample& s) {
  Euler e;
  const double n = s.accel.norm();
  if (n > 1e-3) {
    e.roll = std::atan2(s.accel.y, s.accel.z);
    e.pitch = std::atan2(-s.accel.x, std::hypot(s.accel.y, s.accel.z));
  }
  Quat q = Quat::fromEuler(e);
  if (s.mag) {
    if (const auto err = yawErrorOf(q, *s.mag)) {
      e.yaw = *err;
      q = Quat::fromEuler(e);
    }
  }
  return q;
}

/// Body-frame rotation vector combining the tilt and heading errors.
[[nodiscard]] Vec3 correctionVector(const Quat& q, const ImuSample& s, double magGain) {
  Vec3 e;
  if (const auto tilt = tiltOf(q, s.accel)) e += tilt->error() * tilt->weight;
  if (s.mag) {
    if (const auto yawErr = yawErrorOf(q, *s.mag)) e += q.rotateInverse(kUp) * (*yawErr * magGain);
  }
  return e;
}

}  // namespace

void AttitudeFilter::update(const ImuSample& s) {
  if (!initialized_) {
    q_ = initialOrientation(s);
    lastT_ = s.t;
    initialized_ = true;
    onInit(s);
    return;
  }
  const double dt = s.t - lastT_;
  if (dt <= 0.0) return;
  lastT_ = s.t;
  step(s, std::min(dt, kMaxDt));
}

void ComplementaryFilter::step(const ImuSample& s, double dt) {
  q_ = (q_ * Quat::fromRotationVector(s.gyro * dt)).normalized();
  const double accelGain = dt / (p_.tau + dt);
  const double magGain = dt / (p_.magTau + dt);
  Vec3 e;
  if (const auto tilt = tiltOf(q_, s.accel)) e += tilt->error() * (tilt->weight * accelGain);
  if (s.mag) {
    if (const auto yawErr = yawErrorOf(q_, *s.mag)) e += q_.rotateInverse(kUp) * (*yawErr * magGain);
  }
  q_ = (q_ * Quat::fromRotationVector(e)).normalized();
}

void MahonyFilter::step(const ImuSample& s, double dt) {
  const Vec3 e = correctionVector(q_, s, 1.0);
  bias_ = bias_ - e * (p_.ki * dt);
  const Vec3 omega = s.gyro - bias_ + e * p_.kp;
  q_ = (q_ * Quat::fromRotationVector(omega * dt)).normalized();
}

void EkfFilter::onInit(const ImuSample&) {
  cov_ = Mat6{};
  const double a = p_.initialAttitudeStd * p_.initialAttitudeStd;
  const double b = p_.initialBiasStd * p_.initialBiasStd;
  for (std::size_t i = 0; i < 3; ++i) {
    cov_(i, i) = a;
    cov_(3 + i, 3 + i) = b;
  }
}

void EkfFilter::step(const ImuSample& s, double dt) {
  // Predict: integrate bias-corrected gyro, propagate covariance of [dtheta, dbias].
  const Vec3 w = s.gyro - bias_;
  q_ = (q_ * Quat::fromRotationVector(w * dt)).normalized();

  const Mat3 k = skew(w * dt);
  const Mat3 rot = identity<3>() - k + 0.5 * (k * k);
  Mat6 phi = identity<6>();
  for (std::size_t i = 0; i < 3; ++i) {
    for (std::size_t j = 0; j < 3; ++j) phi(i, j) = rot(i, j);
    phi(i, 3 + i) = -dt;
  }
  cov_ = phi * cov_ * transpose(phi);
  const double qTheta = p_.gyroNoise * p_.gyroNoise * dt;
  const double qBias = p_.biasWalk * p_.biasWalk * dt;
  for (std::size_t i = 0; i < 3; ++i) {
    cov_(i, i) += qTheta;
    cov_(3 + i, 3 + i) += qBias;
  }

  // Update 1: gravity direction. h(x) = R^T * up, Jacobian wrt body-frame error = [g_b]x.
  if (const auto tilt = tiltOf(q_, s.accel)) {
    Mat<3, 6> h;
    const Mat3 gx = skew(tilt->expected);
    for (std::size_t i = 0; i < 3; ++i)
      for (std::size_t j = 0; j < 3; ++j) h(i, j) = gx(i, j);
    Mat<3, 1> y;
    const Vec3 r = tilt->measured - tilt->expected;
    y(0, 0) = r.x;
    y(1, 0) = r.y;
    y(2, 0) = r.z;
    const double sigma = p_.accelNoise / std::sqrt(dt) / kGravity;
    correct(h, y, (sigma * sigma / tilt->weight) * identity<3>());
  }

  // Update 2: heading from the tilt-compensated magnetometer. A rotation about world z
  // changes heading one-to-one; its body-frame direction is g_b.
  if (s.mag) {
    if (const auto yawErr = yawErrorOf(q_, *s.mag)) {
      const Vec3 gb = q_.rotateInverse(kUp);
      Mat<1, 6> h;
      h(0, 0) = gb.x;
      h(0, 1) = gb.y;
      h(0, 2) = gb.z;
      Mat<1, 1> y;
      y(0, 0) = *yawErr;
      Mat<1, 1> noise;
      noise(0, 0) = p_.magYawStd * p_.magYawStd;
      correct(h, y, noise);
    }
  }
}

template <std::size_t M>
void EkfFilter::correct(const Mat<M, 6>& h, const Mat<M, 1>& residual, const Mat<M, M>& noise) {
  const Mat<6, M> ht = transpose(h);
  const auto sInv = inverse(h * cov_ * ht + noise);
  if (!sInv) return;
  const Mat<6, M> gain = cov_ * ht * (*sInv);
  const Mat<6, 1> dx = gain * residual;

  q_ = (q_ * Quat::fromRotationVector({dx(0, 0), dx(1, 0), dx(2, 0)})).normalized();
  bias_ += Vec3{dx(3, 0), dx(4, 0), dx(5, 0)};

  // Joseph form keeps the covariance symmetric positive semi-definite.
  const Mat6 ikh = identity<6>() - gain * h;
  cov_ = ikh * cov_ * transpose(ikh) + gain * noise * transpose(gain);
  for (std::size_t i = 0; i < 6; ++i)
    for (std::size_t j = i + 1; j < 6; ++j) {
      const double avg = 0.5 * (cov_(i, j) + cov_(j, i));
      cov_(i, j) = avg;
      cov_(j, i) = avg;
    }
}

std::string_view toString(FilterKind k) noexcept {
  switch (k) {
    case FilterKind::Complementary: return "complementary";
    case FilterKind::Mahony: return "mahony";
    case FilterKind::Ekf: return "ekf";
  }
  return "unknown";
}

FilterBank::FilterBank() : FilterBank(Config{}) {}

FilterBank::FilterBank(const Config& cfg) {
  filters_[static_cast<std::size_t>(FilterKind::Complementary)] =
      std::make_unique<ComplementaryFilter>(cfg.complementary);
  filters_[static_cast<std::size_t>(FilterKind::Mahony)] = std::make_unique<MahonyFilter>(cfg.mahony);
  filters_[static_cast<std::size_t>(FilterKind::Ekf)] = std::make_unique<EkfFilter>(cfg.ekf);
}

void FilterBank::update(const ImuSample& s) {
  for (auto& f : filters_) f->update(s);
}

}  // namespace imu
