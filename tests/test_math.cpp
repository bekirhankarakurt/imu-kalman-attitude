#include "imu/math.hpp"
#include "test_framework.hpp"

using namespace imu;

TEST(euler_round_trip) {
  const Euler in{degToRad(30.0), degToRad(-20.0), degToRad(140.0)};
  const Euler out = Quat::fromEuler(in).toEuler();
  CHECK_NEAR(out.roll, in.roll, 1e-12);
  CHECK_NEAR(out.pitch, in.pitch, 1e-12);
  CHECK_NEAR(out.yaw, in.yaw, 1e-12);
}

TEST(rotate_known_vectors) {
  const Quat yaw90 = Quat::fromEuler({0.0, 0.0, kPi / 2.0});
  const Vec3 v = yaw90.rotate({1.0, 0.0, 0.0});
  CHECK_NEAR(v.x, 0.0, 1e-12);
  CHECK_NEAR(v.y, 1.0, 1e-12);
  const Vec3 back = yaw90.rotateInverse(v);
  CHECK_NEAR(back.x, 1.0, 1e-12);
  CHECK_NEAR(back.y, 0.0, 1e-12);
}

TEST(gravity_in_body_frame_matches_euler_convention) {
  const Euler e{degToRad(25.0), degToRad(-35.0), degToRad(70.0)};
  const Vec3 g = Quat::fromEuler(e).rotateInverse({0.0, 0.0, 1.0});
  CHECK_NEAR(g.x, -std::sin(e.pitch), 1e-12);
  CHECK_NEAR(g.y, std::cos(e.pitch) * std::sin(e.roll), 1e-12);
  CHECK_NEAR(g.z, std::cos(e.pitch) * std::cos(e.roll), 1e-12);
}

TEST(rotation_vector_round_trip) {
  const Vec3 r{0.3, -0.5, 0.2};
  const Vec3 out = Quat::fromRotationVector(r).toRotationVector();
  CHECK_NEAR(out.x, r.x, 1e-12);
  CHECK_NEAR(out.y, r.y, 1e-12);
  CHECK_NEAR(out.z, r.z, 1e-12);
  CHECK_NEAR(Quat::fromRotationVector({}).w, 1.0, 1e-15);
}

TEST(quaternion_product_composes_rotations) {
  const Quat a = Quat::fromRotationVector({0.0, 0.0, 0.4});
  const Quat b = Quat::fromRotationVector({0.0, 0.0, 0.5});
  CHECK_NEAR((a * b).toRotationVector().z, 0.9, 1e-12);
  CHECK_NEAR((a * a.conjugate()).w, 1.0, 1e-12);
}

TEST(wrap_pi_range) {
  CHECK_NEAR(wrapPi(3.0 * kPi), -kPi, 1e-12);
  CHECK_NEAR(wrapPi(-2.5 * kPi), -0.5 * kPi, 1e-12);
  CHECK_NEAR(wrapPi(0.25), 0.25, 1e-15);
}

TEST(matrix_inverse_and_product) {
  Mat3 m;
  const double v[9] = {2, 1, 0, 0, 3, 1, 1, 0, 4};
  for (std::size_t i = 0; i < 9; ++i) m(i / 3, i % 3) = v[i];
  const auto inv = inverse(m);
  CHECK(inv.has_value());
  const Mat3 prod = m * *inv;
  for (std::size_t i = 0; i < 3; ++i)
    for (std::size_t j = 0; j < 3; ++j) CHECK_NEAR(prod(i, j), i == j ? 1.0 : 0.0, 1e-12);
  CHECK(!inverse(Mat3{}).has_value());
}

TEST(skew_matches_cross_product) {
  const Vec3 a{1.0, -2.0, 3.0}, b{0.5, 4.0, -1.0};
  const Mat3 s = skew(a);
  const Vec3 c = cross(a, b);
  CHECK_NEAR(s(0, 0) * b.x + s(0, 1) * b.y + s(0, 2) * b.z, c.x, 1e-12);
  CHECK_NEAR(s(1, 0) * b.x + s(1, 1) * b.y + s(1, 2) * b.z, c.y, 1e-12);
  CHECK_NEAR(s(2, 0) * b.x + s(2, 1) * b.y + s(2, 2) * b.z, c.z, 1e-12);
}
