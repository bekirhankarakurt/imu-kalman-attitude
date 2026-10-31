# Contributing

Thanks for your interest in improving imu-kalman-attitude.

## Build and test

```bash
cmake -S . -B build -DIMU_WARNINGS_AS_ERRORS=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Before opening a pull request also run the sanitizer build and the static analysis used in CI:

```bash
cmake -S . -B build-asan -DCMAKE_BUILD_TYPE=Debug -DIMU_SANITIZE=ON -DIMU_WARNINGS_AS_ERRORS=ON
cmake --build build-asan -j && ctest --test-dir build-asan --output-on-failure

cmake -S . -B build-tidy -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_CXX_COMPILER=clang++
clang-tidy -p build-tidy --quiet src/*.cpp tests/*.cpp
cppcheck --enable=warning,style,performance,portability --error-exitcode=1 --inline-suppr -I include --std=c++17 --quiet src tests
```

## Code style

- Modern C++17, RAII everywhere, no raw `new`/`delete`, const-correct, `[[nodiscard]]` on value-returning functions, `enum class`.
- Zero warnings with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`; the `.clang-tidy` configuration must pass.
- No TODOs, stubs or dead code. Keep the library dependency-free (standard library and POSIX only).
- Hardware-dependent code needs a simulation or replay path so that it is testable without a device.
- New behaviour needs a test in `tests/`; filter changes should keep the accuracy tests passing.

## Commits and pull requests

- Small, logical commits with plain imperative messages ("Add X", "Fix Y"), no prefixes or trailers.
- Describe what changed and why in the pull request, and update `README.md` and `CHANGELOG.md` when user-visible behaviour changes.
- CI (GCC, Clang, sanitizers, static analysis) must be green.
