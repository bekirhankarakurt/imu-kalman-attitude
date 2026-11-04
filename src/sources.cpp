#include "imu/sources.hpp"

#include <sys/socket.h>

#include <fstream>
#include <stdexcept>
#include <thread>
#include <utility>

namespace imu {

void Pacer::waitUntil(double streamTime) {
  if (!enabled_) return;
  if (!started_) {
    started_ = true;
    origin_ = streamTime;
    start_ = std::chrono::steady_clock::now();
    return;
  }
  std::this_thread::sleep_until(start_ + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                             std::chrono::duration<double>(streamTime - origin_)));
}

bool SimSource::next(Frame& out) {
  SimSample s;
  if (!sim_.next(s)) return false;
  pacer_.waitUntil(s.imu.t);
  out.imu = s.imu;
  out.truth = s.truth;
  out.trueBias = s.trueBias;
  return true;
}

FileSource::FileSource(const std::string& path, const StreamFormat& format, double rateHz, bool realtime)
    : pacer_(realtime) {
  std::ifstream in(path);
  if (!in) throw std::runtime_error("cannot open '" + path + "'");
  std::string line;
  while (std::getline(in, line)) {
    const auto parsed = parseLine(format, line);
    if (!parsed) {
      ++rejected_;
      continue;
    }
    ImuSample s = parsed->imu;
    s.t = parsed->time ? *parsed->time : static_cast<double>(samples_.size()) / rateHz;
    samples_.push_back(s);
  }
  if (samples_.empty()) throw std::runtime_error("no valid samples in '" + path + "'; check --columns");
}

bool FileSource::next(Frame& out) {
  if (cursor_ >= samples_.size()) return false;
  out = Frame{};
  out.imu = samples_[cursor_++];
  pacer_.waitUntil(out.imu.t);
  return true;
}

UdpSource::UdpSource(std::uint16_t port, StreamFormat format, const std::atomic<bool>& stop)
    : socket_(openSocket(SOCK_DGRAM)), format_(std::move(format)), stop_(stop) {
  const sockaddr_in addr = makeAddress("0.0.0.0", port);
  if (::bind(socket_.get(), reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0)
    throw std::runtime_error(errnoMessage("cannot bind UDP port " + std::to_string(port)));
  port_ = localPort(socket_.get());
}

bool UdpSource::next(Frame& out) {
  while (queue_.empty()) {
    if (stop_.load()) return false;
    if (!waitReadable(socket_.get(), 100)) continue;
    char buffer[4096];
    const ssize_t n = ::recv(socket_.get(), buffer, sizeof(buffer), 0);
    if (n <= 0) continue;
    const auto now = std::chrono::steady_clock::now();
    if (!origin_) origin_ = now;
    const double arrival = std::chrono::duration<double>(now - *origin_).count();

    std::size_t start = 0;
    const std::string_view text(buffer, static_cast<std::size_t>(n));
    while (start < text.size()) {
      std::size_t end = text.find('\n', start);
      if (end == std::string_view::npos) end = text.size();
      const std::string_view line = text.substr(start, end - start);
      start = end + 1;
      if (line.find_first_not_of(" \t\r") == std::string_view::npos) continue;
      const auto parsed = parseLine(format_, line);
      if (!parsed) {
        ++rejected_;
        continue;
      }
      ImuSample s = parsed->imu;
      s.t = parsed->time ? *parsed->time : arrival;
      queue_.push_back(s);
    }
  }
  out = Frame{};
  out.imu = queue_.front();
  queue_.pop_front();
  return true;
}

}  // namespace imu
