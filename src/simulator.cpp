#include "imu/simulator.hpp"

#include <cmath>
#include <stdexcept>

namespace imu {
namespace {

constexpr double kTwoPi = 2.0 * kPi;

[[nodiscard]] Vec3 linearAccel(Profile p, double t) noexcept {
  double scale = 0.0;
  if (p == Profile::Aggressive) {
    scale = 1.5;
  } else if (p == Profile::Sway) {
    scale = 0.3;
  }
  return {scale * std::sin(kTwoPi * 0.7 * t), scale * std::cos(kTwoPi * 0.5 * t),
          0.5 * scale * std::sin(kTwoPi * 0.9 * t)};
}

}  // namespace

std::string_view toString(Profile p) noexcept {
  switch (p) {
    case Profile::Static: return "static";
    case Profile::Sway: return "sway";
    case Profile::Aggressive: return "aggressive";
  }
  return "unknown";
}

ImuSimulator::ImuSimulator(const SimConfig& cfg) : cfg_(cfg), bias_(cfg.gyroBias), rng_(cfg.seed) {
  if (!(cfg.rateHz > 0.0)) throw std::invalid_argument("simulator rate must be positive");
}

Quat ImuSimulator::truthAt(Profile p, double t) noexcept {
  Euler e;
  switch (p) {
    case Profile::Static:
      e = {degToRad(10.0), degToRad(-5.0), degToRad(30.0)};
      break;
    case Profile::Sway:
      e = {degToRad(25.0) * std::sin(kTwoPi * 0.20 * t),
           degToRad(20.0) * std::sin(kTwoPi * 0.15 * t + 1.0),
           degToRad(60.0) * std::sin(kTwoPi * 0.05 * t)};
      break;
    case Profile::Aggressive:
      e = {degToRad(60.0) * std::sin(kTwoPi * 0.50 * t),
           degToRad(40.0) * std::sin(kTwoPi * 0.35 * t + 0.5),
           degToRad(120.0) * std::sin(kTwoPi * 0.20 * t)};
      break;
  }
  return Quat::fromEuler(e);
}

double ImuSimulator::uniform() {
  return (static_cast<double>(rng_() >> 11) + 0.5) * (1.0 / 9007199254740992.0);
}

double ImuSimulator::gauss() {
  if (hasSpare_) {
    hasSpare_ = false;
    return spare_;
  }
  const double r = std::sqrt(-2.0 * std::log(uniform()));
  const double a = kTwoPi * uniform();
  spare_ = r * std::sin(a);
  hasSpare_ = true;
  return r * std::cos(a);
}

bool ImuSimulator::next(SimSample& out) {
  const double dt = 1.0 / cfg_.rateHz;
  const double t = static_cast<double>(index_) * dt;
  if (cfg_.durationS > 0.0 && t > cfg_.durationS + 1e-9) return false;
  ++index_;

  const Quat q = truthAt(cfg_.profile, t);
  constexpr double h = 1e-4;
  const Quat qa = truthAt(cfg_.profile, t - h);
  const Quat qb = truthAt(cfg_.profile, t + h);
  const Vec3 omega = (qa.conjugate() * qb).toRotationVector() / (2.0 * h);

  const auto noise = [this](double sigma) { return Vec3{gauss() * sigma, gauss() * sigma, gauss() * sigma}; };

  if (t > 0.0) bias_ += noise(cfg_.biasWalk * std::sqrt(dt));
  const Vec3 specificForce = linearAccel(cfg_.profile, t) + Vec3{0.0, 0.0, kGravity};

  out.truth = q;
  out.trueBias = bias_;
  out.imu.t = t;
  out.imu.gyro = omega + bias_ + noise(cfg_.gyroNoise / std::sqrt(dt));
  out.imu.accel = q.rotateInverse(specificForce) + noise(cfg_.accelNoise / std::sqrt(dt));
  if (cfg_.magnetometer) {
    out.imu.mag = q.rotateInverse(cfg_.magField) + noise(cfg_.magNoise);
  } else {
    out.imu.mag.reset();
  }
  return true;
}

}  // namespace imu
