#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "imu/filters.hpp"

namespace imu {

/// Meaning of one CSV field in a sensor-streaming line.
enum class Column {
  Skip,
  TimeSec,
  TimeMs,
  AccelX, AccelY, AccelZ,
  GyroX, GyroY, GyroZ,
  MagX, MagY, MagZ
};

struct StreamFormat {
  std::vector<Column> columns;
  bool gyroDegrees{false};  ///< gyro fields are deg/s instead of rad/s
  bool accelInG{false};     ///< accel fields are in g instead of m/s^2
};

struct ColumnsParse {
  std::vector<Column> columns;
  std::string error;  ///< empty on success
};

/// Parses a layout such as "t,ax,ay,az,gx,gy,gz,mx,my,mz". Names: t (s), tms (ms), ax..mz, skip.
[[nodiscard]] ColumnsParse parseColumns(std::string_view spec);

struct ParsedSample {
  std::optional<double> time;  ///< seconds, when the line carries a timestamp column
  ImuSample imu;               ///< imu.t is left at zero
};

/// Parses one CSV line (',' ';' or tab separated). Returns nullopt for headers, blank lines,
/// short lines and non-finite values. Fields beyond the configured columns are ignored.
[[nodiscard]] std::optional<ParsedSample> parseLine(const StreamFormat& format, std::string_view line);

}  // namespace imu
