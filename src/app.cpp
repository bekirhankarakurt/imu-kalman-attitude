#include "imu/app.hpp"

#include <algorithm>
#include <array>
#include <csignal>
#include <iomanip>
#include <exception>
#include <fstream>
#include <memory>
#include <ostream>
#include <sstream>
#include <string>

#include "imu/evaluation.hpp"
#include "imu/http_server.hpp"
#include "imu/sources.hpp"
#include "imu/state.hpp"

namespace imu {
namespace {

constexpr double kPublishHz = 50.0;
constexpr double kWarmupSeconds = 10.0;

/// Fixed-point text; `width` 0 means no padding.
std::string fixed(double v, int precision, int width = 0, bool showSign = false) {
  std::ostringstream text;
  text << std::fixed << std::setprecision(precision);
  if (showSign) text << std::showpos;
  if (width > 0) text << std::setw(width);
  text << v;
  return text.str();
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
    static_cast<void>(std::signal(SIGPIPE, SIG_IGN));

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
        log << fixed(t, 4);
        const Euler tr = frame.truth ? frame.truth->toEuler() : Euler{};
        if (frame.truth) {
          log << ',' << fixed(radToDeg(tr.roll), 3) << ',' << fixed(radToDeg(tr.pitch), 3) << ','
              << fixed(radToDeg(tr.yaw), 3);
        } else {
          log << ",,,";
        }
        for (std::size_t k = 0; k < FilterBank::kCount; ++k) {
          const Euler e = bank.get(static_cast<FilterKind>(k)).orientation().toEuler();
          log << ',' << fixed(radToDeg(e.roll), 3) << ',' << fixed(radToDeg(e.pitch), 3) << ','
              << fixed(radToDeg(e.yaw), 3);
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
        out << "t=" << fixed(t, 2, 7) << "s  " << std::left << std::setw(13) << f.name() << std::right
            << " roll " << fixed(radToDeg(e.roll), 2, 7) << "  pitch " << fixed(radToDeg(e.pitch), 2, 7)
            << "  yaw " << fixed(radToDeg(e.yaw), 2, 7) << " deg | bias " << fixed(radToDeg(b.x), 2, 6, true)
            << ' ' << fixed(radToDeg(b.y), 2, 6, true) << ' ' << fixed(radToDeg(b.z), 2, 6, true) << " deg/s";
        if (frame.truth) {
          const double ang = radToDeg((frame.truth->conjugate() * f.orientation()).toRotationVector().norm());
          out << " | err " << fixed(ang, 2) << " deg";
        }
        out << '\n';
      }
    }

    out << "\nprocessed " << samples << " samples";
    if (const std::size_t rej = rejectedOf(*source)) out << " (" << rej << " unparsable lines skipped)";
    out << '\n';

    if (haveTruth) {
      out << "\nerror vs ground truth, RMSE after " << fixed(warmup, 1) << " s warm-up [deg]";
      if (o.noMag) out << "  (no magnetometer: yaw is unobservable)";
      out << '\n';
      out << "filter          roll   pitch     yaw  attitude  bias err [deg/s]\n";
      for (std::size_t k = 0; k < FilterBank::kCount; ++k) {
        const AttitudeFilter& f = bank.get(static_cast<FilterKind>(k));
        const ErrorAccumulator& a = errors[k];
        out << std::left << std::setw(13) << f.name() << std::right << ' ' << fixed(a.rollRmseDeg(), 2, 6) << "  "
            << fixed(a.pitchRmseDeg(), 2, 6) << "  " << fixed(a.yawRmseDeg(), 2, 6) << "    "
            << fixed(a.attitudeRmseDeg(), 2, 6) << "  " << fixed(radToDeg((f.gyroBias() - lastTrueBias).norm()), 3, 6)
            << '\n';
      }
    }
    return 0;
  } catch (const std::exception& ex) {
    err << "error: " << ex.what() << '\n';
    return 1;
  }
}

}  // namespace imu
