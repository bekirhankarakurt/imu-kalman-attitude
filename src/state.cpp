#include "imu/state.hpp"

#include <cmath>
#include <cstdio>

namespace imu {
namespace {

void appendNumber(std::string& out, double v, const char* format = "%.4f") {
  char buf[48];
  std::snprintf(buf, sizeof(buf), format, std::isfinite(v) ? v : 0.0);
  out += buf;
}

void appendAttitude(std::string& out, const Quat& q) {
  const Euler e = q.toEuler();
  out += "\"q\":[";
  appendNumber(out, q.w, "%.6f");
  for (const double c : {q.x, q.y, q.z}) {
    out += ',';
    appendNumber(out, c, "%.6f");
  }
  out += "],\"roll\":";
  appendNumber(out, radToDeg(e.roll));
  out += ",\"pitch\":";
  appendNumber(out, radToDeg(e.pitch));
  out += ",\"yaw\":";
  appendNumber(out, radToDeg(e.yaw));
}

}  // namespace

std::string toJson(const Snapshot& s) {
  std::string out = "{\"t\":";
  appendNumber(out, s.t, "%.3f");
  out += ",\"samples\":" + std::to_string(s.samples) + ",\"rate_hz\":";
  appendNumber(out, s.rateHz, "%.1f");
  out += ",\"selected\":\"" + s.selected + "\",\"filters\":[";
  for (std::size_t i = 0; i < s.filters.size(); ++i) {
    const FilterState& f = s.filters[i];
    if (i != 0) out += ',';
    out += "{\"name\":\"" + f.name + "\",";
    appendAttitude(out, f.q);
    out += ",\"bias_dps\":[";
    appendNumber(out, radToDeg(f.bias.x));
    out += ',';
    appendNumber(out, radToDeg(f.bias.y));
    out += ',';
    appendNumber(out, radToDeg(f.bias.z));
    out += "]}";
  }
  out += "],\"truth\":";
  if (s.truth) {
    out += '{';
    appendAttitude(out, *s.truth);
    out += '}';
  } else {
    out += "null";
  }
  out += '}';
  return out;
}

void StateStore::publish(Snapshot s) {
  const std::lock_guard<std::mutex> lock(mutex_);
  snapshot_ = std::move(s);
}

std::string StateStore::json() const {
  const std::lock_guard<std::mutex> lock(mutex_);
  return toJson(snapshot_);
}

}  // namespace imu
