#pragma once

#include <atomic>
#include <iosfwd>

#include "imu/cli.hpp"

namespace imu {

/// Runs the estimator until the source ends or `stop` is set.
/// Returns 0 on success, 1 on a runtime error (details on `err`).
[[nodiscard]] int run(const Options& options, std::ostream& out, std::ostream& err,
                      const std::atomic<bool>& stop);

}  // namespace imu
