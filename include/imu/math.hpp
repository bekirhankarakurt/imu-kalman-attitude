#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace imu {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kGravity = 9.80665;

[[nodiscard]] constexpr double degToRad(double deg) noexcept { return deg * kPi / 180.0; }
[[nodiscard]] constexpr double radToDeg(double rad) noexcept { return rad * 180.0 / kPi; }

/// Wraps an angle to [-pi, pi).
[[nodiscard]] inline double wrapPi(double a) noexcept {
  a = std::fmod(a + kPi, 2.0 * kPi);
  if (a < 0.0) a += 2.0 * kPi;
  return a - kPi;
}

struct Vec3 {
  double x{0.0};
  double y{0.0};
  double z{0.0};

  [[nodiscard]] double norm() const noexcept { return std::sqrt(x * x + y * y + z * z); }

  Vec3& operator+=(const Vec3& o) noexcept {
    x += o.x;
    y += o.y;
    z += o.z;
    return *this;
  }
};

[[nodiscard]] inline Vec3 operator+(const Vec3& a, const Vec3& b) noexcept {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
[[nodiscard]] inline Vec3 operator-(const Vec3& a, const Vec3& b) noexcept {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
[[nodiscard]] inline Vec3 operator*(const Vec3& a, double s) noexcept {
  return {a.x * s, a.y * s, a.z * s};
}
[[nodiscard]] inline Vec3 operator*(double s, const Vec3& a) noexcept { return a * s; }
[[nodiscard]] inline Vec3 operator/(const Vec3& a, double s) noexcept {
  return {a.x / s, a.y / s, a.z / s};
}
[[nodiscard]] inline double dot(const Vec3& a, const Vec3& b) noexcept {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
[[nodiscard]] inline Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/// Aerospace ZYX Euler angles in radians.
struct Euler {
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

/// Unit quaternion (Hamilton convention) rotating body-frame vectors into the world frame.
struct Quat {
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};

  [[nodiscard]] static Quat fromRotationVector(const Vec3& r) noexcept {
    const double angle = r.norm();
    if (angle < 1e-12) return Quat{1.0, 0.5 * r.x, 0.5 * r.y, 0.5 * r.z}.normalized();
    const double s = std::sin(0.5 * angle) / angle;
    return {std::cos(0.5 * angle), r.x * s, r.y * s, r.z * s};
  }

  [[nodiscard]] static Quat fromEuler(const Euler& e) noexcept {
    const double cr = std::cos(0.5 * e.roll), sr = std::sin(0.5 * e.roll);
    const double cp = std::cos(0.5 * e.pitch), sp = std::sin(0.5 * e.pitch);
    const double cy = std::cos(0.5 * e.yaw), sy = std::sin(0.5 * e.yaw);
    return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
  }

  [[nodiscard]] Quat conjugate() const noexcept { return {w, -x, -y, -z}; }

  [[nodiscard]] double norm() const noexcept { return std::sqrt(w * w + x * x + y * y + z * z); }

  [[nodiscard]] Quat normalized() const noexcept {
    const double n = norm();
    if (n < 1e-12) return {};
    return {w / n, x / n, y / n, z / n};
  }

  /// Rotates a body-frame vector into the world frame.
  [[nodiscard]] Vec3 rotate(const Vec3& v) const noexcept {
    const Vec3 u{x, y, z};
    const Vec3 t = 2.0 * cross(u, v);
    return v + w * t + cross(u, t);
  }

  /// Rotates a world-frame vector into the body frame.
  [[nodiscard]] Vec3 rotateInverse(const Vec3& v) const noexcept { return conjugate().rotate(v); }

  /// Axis-angle vector (rad) of the rotation, with angle in [0, pi].
  [[nodiscard]] Vec3 toRotationVector() const noexcept {
    const double sign = w < 0.0 ? -1.0 : 1.0;
    const Vec3 v{sign * x, sign * y, sign * z};
    const double vn = v.norm();
    if (vn < 1e-12) return v * 2.0;
    return v * (2.0 * std::atan2(vn, sign * w) / vn);
  }

  [[nodiscard]] Euler toEuler() const noexcept {
    Euler e;
    e.roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    e.pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    e.yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    return e;
  }
};

[[nodiscard]] inline Quat operator*(const Quat& a, const Quat& b) noexcept {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}

/// Small fixed-size row-major matrix, enough for a 6-state Kalman filter.
template <std::size_t R, std::size_t C>
class Mat {
 public:
  [[nodiscard]] double& operator()(std::size_t r, std::size_t c) noexcept { return v_[r * C + c]; }
  [[nodiscard]] double operator()(std::size_t r, std::size_t c) const noexcept { return v_[r * C + c]; }

 private:
  std::array<double, R * C> v_{};
};

using Mat3 = Mat<3, 3>;

template <std::size_t N>
[[nodiscard]] Mat<N, N> identity() noexcept {
  Mat<N, N> m;
  for (std::size_t i = 0; i < N; ++i) m(i, i) = 1.0;
  return m;
}

template <std::size_t R, std::size_t K, std::size_t C>
[[nodiscard]] Mat<R, C> operator*(const Mat<R, K>& a, const Mat<K, C>& b) noexcept {
  Mat<R, C> out;
  for (std::size_t i = 0; i < R; ++i)
    for (std::size_t k = 0; k < K; ++k)
      for (std::size_t j = 0; j < C; ++j) out(i, j) += a(i, k) * b(k, j);
  return out;
}

template <std::size_t R, std::size_t C>
[[nodiscard]] Mat<R, C> operator*(double s, const Mat<R, C>& a) noexcept {
  Mat<R, C> out;
  for (std::size_t i = 0; i < R; ++i)
    for (std::size_t j = 0; j < C; ++j) out(i, j) = s * a(i, j);
  return out;
}

template <std::size_t R, std::size_t C>
[[nodiscard]] Mat<R, C> operator+(const Mat<R, C>& a, const Mat<R, C>& b) noexcept {
  Mat<R, C> out;
  for (std::size_t i = 0; i < R; ++i)
    for (std::size_t j = 0; j < C; ++j) out(i, j) = a(i, j) + b(i, j);
  return out;
}

template <std::size_t R, std::size_t C>
[[nodiscard]] Mat<R, C> operator-(const Mat<R, C>& a, const Mat<R, C>& b) noexcept {
  Mat<R, C> out;
  for (std::size_t i = 0; i < R; ++i)
    for (std::size_t j = 0; j < C; ++j) out(i, j) = a(i, j) - b(i, j);
  return out;
}

template <std::size_t R, std::size_t C>
[[nodiscard]] Mat<C, R> transpose(const Mat<R, C>& a) noexcept {
  Mat<C, R> out;
  for (std::size_t i = 0; i < R; ++i)
    for (std::size_t j = 0; j < C; ++j) out(j, i) = a(i, j);
  return out;
}

/// Cross-product matrix: skew(a) * b == cross(a, b).
[[nodiscard]] inline Mat3 skew(const Vec3& a) noexcept {
  Mat3 m;
  m(0, 1) = -a.z;
  m(0, 2) = a.y;
  m(1, 0) = a.z;
  m(1, 2) = -a.x;
  m(2, 0) = -a.y;
  m(2, 1) = a.x;
  return m;
}

[[nodiscard]] inline std::optional<Mat<1, 1>> inverse(const Mat<1, 1>& m) noexcept {
  if (std::abs(m(0, 0)) < 1e-300) return std::nullopt;
  Mat<1, 1> out;
  out(0, 0) = 1.0 / m(0, 0);
  return out;
}

[[nodiscard]] inline std::optional<Mat3> inverse(const Mat3& m) noexcept {
  const double a = m(0, 0), b = m(0, 1), c = m(0, 2);
  const double d = m(1, 0), e = m(1, 1), f = m(1, 2);
  const double g = m(2, 0), h = m(2, 1), i = m(2, 2);
  const double co0 = e * i - f * h;
  const double co1 = f * g - d * i;
  const double co2 = d * h - e * g;
  const double det = a * co0 + b * co1 + c * co2;
  if (std::abs(det) < 1e-300) return std::nullopt;
  Mat3 r;
  r(0, 0) = co0 / det;
  r(0, 1) = (c * h - b * i) / det;
  r(0, 2) = (b * f - c * e) / det;
  r(1, 0) = co1 / det;
  r(1, 1) = (a * i - c * g) / det;
  r(1, 2) = (c * d - a * f) / det;
  r(2, 0) = co2 / det;
  r(2, 1) = (b * g - a * h) / det;
  r(2, 2) = (a * e - b * d) / det;
  return r;
}

}  // namespace imu
