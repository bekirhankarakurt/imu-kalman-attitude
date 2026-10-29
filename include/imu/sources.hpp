#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "imu/net.hpp"
#include "imu/simulator.hpp"
#include "imu/stream_format.hpp"

namespace imu {

/// One sample as delivered to the pipeline; truth fields are present only in simulation.
struct Frame {
  ImuSample imu;
  std::optional<Quat> truth;
  std::optional<Vec3> trueBias;
};

class SampleSource {
 public:
  SampleSource() = default;
  SampleSource(const SampleSource&) = delete;
  SampleSource& operator=(const SampleSource&) = delete;
  virtual ~SampleSource() = default;

  /// Blocks until the next sample. Returns false at end of stream or when stopped.
  [[nodiscard]] virtual bool next(Frame& out) = 0;
};

/// Sleeps so that stream time advances at wall-clock speed.
class Pacer {
 public:
  explicit Pacer(bool enabled) noexcept : enabled_(enabled) {}
  void waitUntil(double streamTime);

 private:
  bool enabled_;
  bool started_{false};
  double origin_{0.0};
  std::chrono::steady_clock::time_point start_;
};

class SimSource final : public SampleSource {
 public:
  SimSource(const SimConfig& cfg, bool realtime) : sim_(cfg), pacer_(realtime) {}
  [[nodiscard]] bool next(Frame& out) override;

 private:
  ImuSimulator sim_;
  Pacer pacer_;
};

/// Replays a CSV recording. Without a timestamp column samples are spaced at 1/rateHz.
class FileSource final : public SampleSource {
 public:
  /// Throws std::runtime_error if the file cannot be read or has no valid lines.
  FileSource(const std::string& path, const StreamFormat& format, double rateHz, bool realtime);
  [[nodiscard]] bool next(Frame& out) override;
  [[nodiscard]] std::size_t rejectedLines() const noexcept { return rejected_; }

 private:
  std::vector<ImuSample> samples_;
  std::size_t cursor_{0};
  std::size_t rejected_{0};
  Pacer pacer_;
};

/// Receives CSV datagrams (one or more lines each) on a UDP port.
class UdpSource final : public SampleSource {
 public:
  /// Throws std::runtime_error if the port cannot be bound. Port 0 picks a free port.
  UdpSource(std::uint16_t port, const StreamFormat& format, const std::atomic<bool>& stop);
  [[nodiscard]] bool next(Frame& out) override;
  [[nodiscard]] std::uint16_t port() const noexcept { return port_; }
  [[nodiscard]] std::size_t rejectedLines() const noexcept { return rejected_; }

 private:
  Socket socket_;
  std::uint16_t port_{0};
  StreamFormat format_;
  const std::atomic<bool>& stop_;
  std::deque<ImuSample> queue_;
  std::size_t rejected_{0};
  std::optional<std::chrono::steady_clock::time_point> origin_;
};

}  // namespace imu
