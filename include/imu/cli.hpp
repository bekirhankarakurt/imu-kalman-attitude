#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "imu/filters.hpp"
#include "imu/simulator.hpp"

namespace imu {

enum class SourceKind : std::uint8_t { Sim, Udp, File };

struct Options {
  SourceKind source{SourceKind::Sim};
  std::uint16_t udpPort{5555};
  std::string file;
  std::string columns{"ax,ay,az,gx,gy,gz"};
  bool gyroDegrees{false};
  bool accelInG{false};
  double rateHz{100.0};
  double duration{30.0};
  Profile profile{Profile::Sway};
  std::uint64_t seed{1};
  Vec3 gyroBias{0.02, -0.015, 0.01};
  double gyroNoise{0.0007};
  double accelNoise{0.005};
  double magNoise{0.4};
  bool noMag{false};
  bool realtime{false};
  std::optional<std::uint16_t> httpPort;
  std::string bindAddress{"127.0.0.1"};
  FilterKind show{FilterKind::Ekf};
  double printHz{2.0};
  std::string logPath;
  bool quiet{false};
  double kp{1.5};
  double ki{0.1};
  double tau{1.0};
  bool help{false};
};

struct ParseResult {
  Options options;
  std::string error;  ///< empty on success
};

[[nodiscard]] ParseResult parseArgs(const std::vector<std::string>& args);
[[nodiscard]] std::string_view usage() noexcept;

}  // namespace imu
