#include "imu/cli.hpp"

#include <cmath>
#include <cstdlib>

namespace imu {
namespace {

bool toDouble(const std::string& text, double& out) {
  if (text.empty()) return false;
  char* end = nullptr;
  out = std::strtod(text.c_str(), &end);
  return end == text.c_str() + text.size() && std::isfinite(out);
}

bool toUInt(const std::string& text, unsigned long long max, unsigned long long& out) {
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) return false;
  char* end = nullptr;
  out = std::strtoull(text.c_str(), &end, 10);
  return end == text.c_str() + text.size() && out <= max;
}

bool toVec3(const std::string& text, Vec3& out) {
  const std::size_t a = text.find(',');
  const std::size_t b = a == std::string::npos ? a : text.find(',', a + 1);
  if (b == std::string::npos) return false;
  return toDouble(text.substr(0, a), out.x) && toDouble(text.substr(a + 1, b - a - 1), out.y) &&
         toDouble(text.substr(b + 1), out.z);
}

}  // namespace

std::string_view usage() noexcept {
  return R"(imu_attitude - IMU attitude estimation (complementary, Mahony, error-state EKF)

Usage: imu_attitude [options]

Input
  --source sim|udp|file   sample source (default sim)
  --port N                UDP port to listen on (default 5555)
  --file PATH             CSV recording to replay
  --columns LIST          CSV layout, e.g. t,ax,ay,az,gx,gy,gz,mx,my,mz (default ax,ay,az,gx,gy,gz)
  --gyro-deg              gyro columns are deg/s (default rad/s)
  --accel-g               accel columns are in g (default m/s^2)
  --no-mag                ignore magnetometer data

Simulator
  --duration S            simulated seconds, 0 = run forever (default 30)
  --rate HZ               sample rate; also replay rate for files without timestamps (default 100)
  --profile NAME          static|sway|aggressive (default sway)
  --seed N                random seed (default 1)
  --gyro-bias X,Y,Z       true gyro bias in rad/s (default 0.02,-0.015,0.01)
  --gyro-noise D          gyro noise density, rad/s/sqrt(Hz) (default 0.0007)
  --accel-noise D         accel noise density, m/s^2/sqrt(Hz) (default 0.005)
  --mag-noise S           magnetometer noise std, uT (default 0.4)

Filters
  --kp G --ki G           Mahony proportional / integral gains (default 1.5 / 0.1)
  --tau S                 complementary filter time constant (default 1.0)
  --show NAME             complementary|mahony|ekf: filter shown in terminal and viewer (default ekf)

Output
  --http PORT             serve the 3D viewer and /api/state on PORT (implies --realtime for sim/file)
  --bind ADDR             HTTP bind address (default 127.0.0.1)
  --realtime              pace sim/file replay at wall-clock speed
  --print-hz HZ           terminal update rate in stream time (default 2, 0 = off)
  --log PATH              write all estimates to a CSV file
  --quiet                 print only the final summary
  -h, --help              show this help
)";
}

ParseResult parseArgs(const std::vector<std::string>& args) {
  ParseResult r;
  Options& o = r.options;

  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string& a = args[i];
    const auto fail = [&](const std::string& msg) {
      r.error = msg;
      return r;
    };
    const auto needValue = [&]() { return i + 1 < args.size(); };

    if (a == "-h" || a == "--help") {
      o.help = true;
      continue;
    }
    if (a == "--gyro-deg") { o.gyroDegrees = true; continue; }
    if (a == "--accel-g") { o.accelInG = true; continue; }
    if (a == "--no-mag") { o.noMag = true; continue; }
    if (a == "--realtime") { o.realtime = true; continue; }
    if (a == "--quiet") { o.quiet = true; continue; }

    if (!needValue()) return fail("option " + a + " needs a value");
    const std::string& v = args[++i];
    double d = 0.0;
    unsigned long long u = 0;

    if (a == "--source") {
      if (v == "sim") o.source = SourceKind::Sim;
      else if (v == "udp") o.source = SourceKind::Udp;
      else if (v == "file") o.source = SourceKind::File;
      else return fail("--source must be sim, udp or file");
    } else if (a == "--port") {
      if (!toUInt(v, 65535, u)) return fail("--port must be 0-65535");
      o.udpPort = static_cast<std::uint16_t>(u);
    } else if (a == "--http") {
      if (!toUInt(v, 65535, u)) return fail("--http must be 0-65535");
      o.httpPort = static_cast<std::uint16_t>(u);
    } else if (a == "--file") {
      o.file = v;
    } else if (a == "--columns") {
      o.columns = v;
    } else if (a == "--bind") {
      o.bindAddress = v;
    } else if (a == "--log") {
      o.logPath = v;
    } else if (a == "--profile") {
      if (v == "static") o.profile = Profile::Static;
      else if (v == "sway") o.profile = Profile::Sway;
      else if (v == "aggressive") o.profile = Profile::Aggressive;
      else return fail("--profile must be static, sway or aggressive");
    } else if (a == "--show") {
      if (v == "complementary") o.show = FilterKind::Complementary;
      else if (v == "mahony") o.show = FilterKind::Mahony;
      else if (v == "ekf") o.show = FilterKind::Ekf;
      else return fail("--show must be complementary, mahony or ekf");
    } else if (a == "--seed") {
      if (!toUInt(v, ~0ULL, u)) return fail("--seed must be a non-negative integer");
      o.seed = u;
    } else if (a == "--gyro-bias") {
      if (!toVec3(v, o.gyroBias)) return fail("--gyro-bias needs X,Y,Z");
    } else if (a == "--rate" || a == "--duration" || a == "--gyro-noise" || a == "--accel-noise" ||
               a == "--mag-noise" || a == "--print-hz" || a == "--kp" || a == "--ki" || a == "--tau") {
      if (!toDouble(v, d)) return fail(a + " needs a number");
      if (a == "--rate") {
        if (d <= 0.0) return fail("--rate must be positive");
        o.rateHz = d;
      } else if (a == "--tau") {
        if (d <= 0.0) return fail("--tau must be positive");
        o.tau = d;
      } else {
        if (d < 0.0) return fail(a + " must not be negative");
        if (a == "--duration") o.duration = d;
        else if (a == "--gyro-noise") o.gyroNoise = d;
        else if (a == "--accel-noise") o.accelNoise = d;
        else if (a == "--mag-noise") o.magNoise = d;
        else if (a == "--print-hz") o.printHz = d;
        else if (a == "--kp") o.kp = d;
        else o.ki = d;
      }
    } else {
      return fail("unknown option " + a);
    }
  }

  if (o.source == SourceKind::File && o.file.empty() && !o.help) r.error = "--source file needs --file PATH";
  return r;
}

}  // namespace imu
