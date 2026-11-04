#include "imu/state.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace imu {
namespace {

void appendNumber(std::string& out, double v, int precision = 4) {
  std::ostringstream text;
  text << std::fixed << std::setprecision(precision) << (std::isfinite(v) ? v : 0.0);
  out += text.str();
}

void appendAttitude(std::string& out, const Quat& q) {
  const Euler e = q.toEuler();
  out += R"("q":[)";
  appendNumber(out, q.w, 6);
  for (const double c : {q.x, q.y, q.z}) {
    out += ',';
    appendNumber(out, c, 6);
  }
  out += R"(],"roll":)";
  appendNumber(out, radToDeg(e.roll));
  out += R"(,"pitch":)";
  appendNumber(out, radToDeg(e.pitch));
  out += R"(,"yaw":)";
  appendNumber(out, radToDeg(e.yaw));
}

}  // namespace

std::string toJson(const Snapshot& s) {
  std::string out = R"({"t":)";
  appendNumber(out, s.t, 3);
  out += R"(,"samples":)" + std::to_string(s.samples) + R"(,"rate_hz":)";
  appendNumber(out, s.rateHz, 1);
  out += R"(,"selected":")" + s.selected + R"(","filters":[)";
  for (std::size_t i = 0; i < s.filters.size(); ++i) {
    const FilterState& f = s.filters[i];
    if (i != 0) out += ',';
    out += R"({"name":")" + f.name + R"(",)";
    appendAttitude(out, f.q);
    out += R"(,"bias_dps":[)";
    appendNumber(out, radToDeg(f.bias.x));
    out += ',';
    appendNumber(out, radToDeg(f.bias.y));
    out += ',';
    appendNumber(out, radToDeg(f.bias.z));
    out += "]}";
  }
  out += R"(],"truth":)";
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
  const std::scoped_lock<std::mutex> lock(mutex_);
  snapshot_ = std::move(s);
}

std::string StateStore::json() const {
  const std::scoped_lock<std::mutex> lock(mutex_);
  return toJson(snapshot_);
}

}  // namespace imu
