#include "imu/app.hpp"

#include <algorithm>
#include <array>
#include <csignal>
#include <cstdio>
#include <exception>
#include <fstream>
#include <memory>
#include <ostream>
#include <string>

#include "imu/evaluation.hpp"
#include "imu/http_server.hpp"
#include "imu/sources.hpp"
#include "imu/state.hpp"

namespace imu {
namespace {

constexpr double kPublishHz = 50.0;
constexpr double kWarmupSeconds = 10.0;

std::string format(const char* fmt, double a = 0, double b = 0, double c = 0, double d = 0, double e = 0) {
  char buf[256];
  std::snprintf(buf, sizeof(buf), fmt, a, b, c, d, e);
  return buf;
}

std::unique_ptr<SampleSource> makeSource(const Options& o, const StreamFormat& fmt, bool realtime,
                                         const std::atomic<bool>& stop) {
  switch (o.source) {
    case SourceKind::Sim: {
      SimConfig c;
      c.rateHz = o.rateHz;
      c.durationS = o.duration;
      c.profile = o.profile;
      c.seed = o.seed;
      c.gyroBias = o.gyroBias;
      c.gyroNoise = o.gyroNoise;
      c.accelNoise = o.accelNoise;
      c.magNoise = o.magNoise;
      c.magnetometer = !o.noMag;
      return std::make_unique<SimSource>(c, realtime);
    }
    case SourceKind::File:
      return std::make_unique<FileSource>(o.file, fmt, o.rateHz, realtime);
    case SourceKind::Udp:
      return std::make_unique<UdpSource>(o.udpPort, fmt, stop);
  }
  return nullptr;
}

std::size_t rejectedOf(const SampleSource& s) {
  if (const auto* f = dynamic_cast<const FileSource*>(&s)) return f->rejectedLines();
  if (const auto* u = dynamic_cast<const UdpSource*>(&s)) return u->rejectedLines();
  return 0;
}

}  // namespace

int run(const Options& o, std::ostream& out, std::ostream& err, const std::atomic<bool>& stop) {
  try {
    ::signal(SIGPIPE, SIG_IGN);

    StreamFormat fmt;
    fmt.gyroDegrees = o.gyroDegrees;
    fmt.accelInG = o.accelInG;
    if (o.source != SourceKind::Sim) {
      ColumnsParse cols = parseColumns(o.columns);
      if (!cols.error.empty()) {
        err << "error: --columns: " << cols.error << '\n';
        return 1;
      }
      fmt.columns = std::move(cols.columns);
    }

    const bool realtime = o.realtime || (o.httpPort && o.source != SourceKind::Udp);
    std::unique_ptr<SampleSource> source = makeSource(o, fmt, realtime, stop);
    if (const auto* udp = dynamic_cast<const UdpSource*>(source.get()))
      out << "listening for UDP CSV on port " << udp->port() << '\n';

    FilterBank::Config cfg;
    cfg.mahony.kp = o.kp;
    cfg.mahony.ki = o.ki;
    cfg.complementary.tau = o.tau;
    FilterBank bank(cfg);

    StateStore store;
    std::unique_ptr<HttpServer> http;
    if (o.httpPort) {
      http = std::make_unique<HttpServer>(store, o.bindAddress, *o.httpPort);
      out << "viewer: http://" << o.bindAddress << ':' << http->port() << "/\n";
    }

    std::ofstream log;
    if (!o.logPath.empty()) {
      log.open(o.logPath);
      if (!log) {
        err << "error: cannot write '" << o.logPath << "'\n";
        return 1;
      }
      log << "t,truth_roll,truth_pitch,truth_yaw";
      for (std::size_t k = 0; k < FilterBank::kCount; ++k) {
        const std::string n(toString(static_cast<FilterKind>(k)));
        log << ',' << n << "_roll," << n << "_pitch," << n << "_yaw";
      }
      log << '\n';
    }

    const double warmup =
        o.source == SourceKind::Sim && o.duration > 0.0 ? std::min(kWarmupSeconds, 0.25 * o.duration) : 0.0;
    std::array<ErrorAccumulator, FilterBank::kCount> errors;
    Vec3 lastTrueBias;
    bool haveTruth = false;

    Frame frame;
    std::size_t samples = 0;
    double rate = 0.0, lastT = 0.0, nextPublish = 0.0, nextPrint = 0.0;

    while (!stop.load() && source->next(frame)) {
      if (o.noMag) frame.imu.mag.reset();
      bank.update(frame.imu);
      const double t = frame.imu.t;
      if (samples > 0 && t > lastT) {
        const double inst = 1.0 / (t - lastT);
        rate = rate == 0.0 ? inst : 0.98 * rate + 0.02 * inst;
      }
      lastT = t;
      ++samples;

      if (frame.truth) {
        haveTruth = true;
        if (frame.trueBias) lastTrueBias = *frame.trueBias;
        if (t >= warmup)
          for (std::size_t k = 0; k < FilterBank::kCount; ++k)
            errors[k].add(bank.get(static_cast<FilterKind>(k)).orientation(), *frame.truth);
      }

      if (log.is_open()) {
        log << format("%.4f", t);
        const Euler tr = frame.truth ? frame.truth->toEuler() : Euler{};
        if (frame.truth) {
          log << format(",%.3f,%.3f,%.3f", radToDeg(tr.roll), radToDeg(tr.pitch), radToDeg(tr.yaw));
        } else {
          log << ",,,";
        }
        for (std::size_t k = 0; k < FilterBank::kCount; ++k) {
          const Euler e = bank.get(static_cast<FilterKind>(k)).orientation().toEuler();
          log << format(",%.3f,%.3f,%.3f", radToDeg(e.roll), radToDeg(e.pitch), radToDeg(e.yaw));
        }
        log << '\n';
      }

      if (http && t >= nextPublish) {
        nextPublish = t + 1.0 / kPublishHz;
        Snapshot snap;
        snap.t = t;
        snap.samples = samples;
        snap.rateHz = rate;
        snap.selected = std::string(toString(o.show));
        snap.truth = frame.truth;
        for (std::size_t k = 0; k < FilterBank::kCount; ++k) {
          const AttitudeFilter& f = bank.get(static_cast<FilterKind>(k));
          snap.filters.push_back({std::string(f.name()), f.orientation(), f.gyroBias()});
        }
        store.publish(std::move(snap));
      }

      if (!o.quiet && o.printHz > 0.0 && t >= nextPrint) {
        nextPrint = t + 1.0 / o.printHz;
        const AttitudeFilter& f = bank.get(o.show);
        const Euler e = f.orientation().toEuler();
        const Vec3 b = f.gyroBias();
        char line[200];
        std::snprintf(line, sizeof(line),
                      "t=%7.2fs  %-13s roll %7.2f  pitch %7.2f  yaw %7.2f deg | bias %+6.2f %+6.2f %+6.2f deg/s",
                      t, std::string(f.name()).c_str(), radToDeg(e.roll), radToDeg(e.pitch),
                      radToDeg(e.yaw), radToDeg(b.x), radToDeg(b.y), radToDeg(b.z));
        out << line;
        if (frame.truth) {
          const double ang = radToDeg((frame.truth->conjugate() * f.orientation()).toRotationVector().norm());
          out << format(" | err %.2f deg", ang);
        }
        out << '\n';
      }
    }

    out << "\nprocessed " << samples << " samples";
    if (const std::size_t rej = rejectedOf(*source)) out << " (" << rej << " unparsable lines skipped)";
    out << '\n';

    if (haveTruth) {
      out << format("\nerror vs ground truth, RMSE after %.1f s warm-up [deg]", warmup);
      if (o.noMag) out << "  (no magnetometer: yaw is unobservable)";
      out << '\n';
      out << "filter          roll   pitch     yaw  attitude  bias err [deg/s]\n";
      for (std::size_t k = 0; k < FilterBank::kCount; ++k) {
        const AttitudeFilter& f = bank.get(static_cast<FilterKind>(k));
        const ErrorAccumulator& a = errors[k];
        char row[160];
        std::snprintf(row, sizeof(row), "%-13s %6.2f  %6.2f  %6.2f    %6.2f  %6.3f\n",
                      std::string(f.name()).c_str(), a.rollRmseDeg(), a.pitchRmseDeg(), a.yawRmseDeg(),
                      a.attitudeRmseDeg(), radToDeg((f.gyroBias() - lastTrueBias).norm()));
        out << row;
      }
    }
    return 0;
  } catch (const std::exception& ex) {
    err << "error: " << ex.what() << '\n';
    return 1;
  }
}

}  // namespace imu
