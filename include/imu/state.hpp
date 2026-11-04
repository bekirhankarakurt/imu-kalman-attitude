#pragma once

#include <cstddef>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "imu/math.hpp"

namespace imu {

struct FilterState {
  std::string name;
  Quat q;
  Vec3 bias;  ///< rad/s
};

/// Latest estimates of every filter, published for the HTTP viewer.
struct Snapshot {
  double t{0.0};
  std::size_t samples{0};
  double rateHz{0.0};
  std::string selected;
  std::vector<FilterState> filters;
  std::optional<Quat> truth;
};

/// Serialises a snapshot to compact JSON (angles in degrees, bias in deg/s).
[[nodiscard]] std::string toJson(const Snapshot& s);

/// Thread-safe holder of the most recent snapshot.
class StateStore {
 public:
  void publish(Snapshot s);
  [[nodiscard]] std::string json() const;

 private:
  mutable std::mutex mutex_;
  Snapshot snapshot_;
};

}  // namespace imu
