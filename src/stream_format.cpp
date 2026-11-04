#include "imu/stream_format.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <set>

namespace imu {
namespace {

[[nodiscard]] std::string_view trim(std::string_view s) {
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
  return s;
}

[[nodiscard]] std::vector<std::string_view> split(std::string_view s, std::string_view delimiters) {
  std::vector<std::string_view> out;
  std::size_t start = 0;
  for (std::size_t i = 0; i <= s.size(); ++i) {
    if (i == s.size() || delimiters.find(s[i]) != std::string_view::npos) {
      out.push_back(trim(s.substr(start, i - start)));
      start = i + 1;
    }
  }
  return out;
}

[[nodiscard]] std::optional<double> toDouble(std::string_view field) {
  if (field.empty()) return std::nullopt;
  const std::string text(field);
  char* end = nullptr;
  const double v = std::strtod(text.c_str(), &end);
  if (end != text.c_str() + text.size() || !std::isfinite(v)) return std::nullopt;
  return v;
}

struct Name {
  std::string_view text;
  Column column;
};

constexpr Name kNames[] = {
    {"skip", Column::Skip},   {"t", Column::TimeSec}, {"tms", Column::TimeMs},
    {"ax", Column::AccelX},   {"ay", Column::AccelY}, {"az", Column::AccelZ},
    {"gx", Column::GyroX},    {"gy", Column::GyroY},  {"gz", Column::GyroZ},
    {"mx", Column::MagX},     {"my", Column::MagY},   {"mz", Column::MagZ},
};

}  // namespace

ColumnsParse parseColumns(std::string_view spec) {
  ColumnsParse result;
  std::set<Column> seen;
  for (const std::string_view token : split(spec, ",")) {
    const auto it = std::find_if(std::begin(kNames), std::end(kNames),
                                 [&](const Name& n) { return n.text == token; });
    if (it == std::end(kNames)) {
      result.error = "unknown column '" + std::string(token) + "'";
      return result;
    }
    if (it->column != Column::Skip && !seen.insert(it->column).second) {
      result.error = "duplicate column '" + std::string(token) + "'";
      return result;
    }
    result.columns.push_back(it->column);
  }

  const auto has = [&](Column c) { return seen.count(c) != 0; };
  for (const Column c : {Column::AccelX, Column::AccelY, Column::AccelZ, Column::GyroX, Column::GyroY,
                         Column::GyroZ}) {
    if (!has(c)) {
      result.error = "columns must include ax,ay,az,gx,gy,gz";
      return result;
    }
  }
  const int mags = int(has(Column::MagX)) + int(has(Column::MagY)) + int(has(Column::MagZ));
  if (mags != 0 && mags != 3) {
    result.error = "magnetometer needs all of mx,my,mz";
    return result;
  }
  if (has(Column::TimeSec) && has(Column::TimeMs)) {
    result.error = "use either t or tms, not both";
    return result;
  }
  return result;
}

std::optional<ParsedSample> parseLine(const StreamFormat& format, std::string_view line) {
  const std::vector<std::string_view> fields = split(trim(line), ",;\t");
  if (fields.size() < format.columns.size()) return std::nullopt;

  ParsedSample out;
  Vec3 mag;
  bool hasMag = false;
  for (std::size_t i = 0; i < format.columns.size(); ++i) {
    const Column c = format.columns[i];
    if (c == Column::Skip) continue;
    const std::optional<double> v = toDouble(fields[i]);
    if (!v) return std::nullopt;
    switch (c) {
      case Column::TimeSec: out.time = *v; break;
      case Column::TimeMs: out.time = *v * 1e-3; break;
      case Column::AccelX: out.imu.accel.x = *v; break;
      case Column::AccelY: out.imu.accel.y = *v; break;
      case Column::AccelZ: out.imu.accel.z = *v; break;
      case Column::GyroX: out.imu.gyro.x = *v; break;
      case Column::GyroY: out.imu.gyro.y = *v; break;
      case Column::GyroZ: out.imu.gyro.z = *v; break;
      case Column::MagX: mag.x = *v; hasMag = true; break;
      case Column::MagY: mag.y = *v; hasMag = true; break;
      case Column::MagZ: mag.z = *v; hasMag = true; break;
      case Column::Skip: break;
    }
  }
  if (hasMag) out.imu.mag = mag;
  if (format.gyroDegrees) out.imu.gyro = out.imu.gyro * (kPi / 180.0);
  if (format.accelInG) out.imu.accel = out.imu.accel * kGravity;
  return out;
}

}  // namespace imu
