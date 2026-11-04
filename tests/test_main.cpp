#include "test_framework.hpp"

int main() {
  for (const test::Case& c : test::registry()) {
    const int before = test::failures();
    try {
      c.body();
    } catch (const std::exception& e) {
      test::fail(__FILE__, __LINE__, std::string("uncaught exception in ") + c.name + ": " + e.what());
    }
    std::cout << (test::failures() == before ? "[ ok ] " : "[FAIL] ") << c.name << '\n';
  }
  std::cout << test::registry().size() << " tests, " << test::failures() << " failed assertions\n";
  return test::failures() == 0 ? 0 : 1;
}
