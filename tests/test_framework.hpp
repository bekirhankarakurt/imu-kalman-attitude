#pragma once

#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

namespace test {

struct Case {
  std::string name;
  std::function<void()> body;
};

inline std::vector<Case>& registry() {
  static std::vector<Case> cases;
  return cases;
}

inline int& failures() {
  static int count = 0;
  return count;
}

struct Registrar {
  Registrar(const char* name, std::function<void()> body) { registry().push_back({name, std::move(body)}); }
};

inline void fail(const char* file, int line, const std::string& message) {
  ++failures();
  std::cerr << file << ':' << line << ": " << message << '\n';
}

}  // namespace test

#define TEST(name)                                          \
  static void test_##name();                                \
  static const test::Registrar registrar_##name(#name, test_##name); \
  static void test_##name()

#define CHECK(cond)                                                              \
  do {                                                                           \
    if (!(cond)) test::fail(__FILE__, __LINE__, "CHECK failed: " #cond);         \
  } while (false)

#define CHECK_NEAR(actual, expected, tol)                                                          \
  do {                                                                                             \
    const double a_ = (actual);                                                                    \
    const double e_ = (expected);                                                                  \
    if (!(std::abs(a_ - e_) <= (tol)))                                                             \
      test::fail(__FILE__, __LINE__,                                                               \
                 std::string("CHECK_NEAR failed: " #actual " = ") + std::to_string(a_) +           \
                     ", expected " + std::to_string(e_) + " +/- " + std::to_string(tol));          \
  } while (false)

#define CHECK_LT(actual, limit)                                                                    \
  do {                                                                                             \
    const double a_ = (actual);                                                                    \
    if (!(a_ < (limit)))                                                                           \
      test::fail(__FILE__, __LINE__,                                                               \
                 std::string("CHECK_LT failed: " #actual " = ") + std::to_string(a_) +             \
                     ", limit " + std::to_string(limit));                                          \
  } while (false)
