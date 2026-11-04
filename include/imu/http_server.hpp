#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <string_view>
#include <thread>

#include "imu/net.hpp"
#include "imu/state.hpp"

namespace imu {

/// Minimal single-threaded HTTP/1.1 server: GET / (viewer page) and GET /api/state (JSON).
class HttpServer {
 public:
  /// Binds and starts serving; throws std::runtime_error on failure. Port 0 picks a free port.
  HttpServer(const StateStore& store, const std::string& bindAddress, std::uint16_t port);
  HttpServer(const HttpServer&) = delete;
  HttpServer& operator=(const HttpServer&) = delete;
  HttpServer(HttpServer&&) = delete;
  HttpServer& operator=(HttpServer&&) = delete;
  ~HttpServer();

  [[nodiscard]] std::uint16_t port() const noexcept { return port_; }

  /// Builds the full HTTP response for a raw request. Pure; exposed for testing.
  [[nodiscard]] static std::string respond(const StateStore& store, std::string_view request);

 private:
  void serve();
  void handle(int client) const;

  const StateStore& store_;
  Socket listener_;
  std::uint16_t port_{0};
  std::atomic<bool> stopping_{false};
  std::thread thread_;
};

/// The self-contained HTML/JS 3D viewer.
[[nodiscard]] std::string_view viewerHtml() noexcept;

}  // namespace imu
