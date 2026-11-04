#pragma once

#include <netinet/in.h>

#include <cstdint>
#include <string>

namespace imu {

/// Owning file-descriptor wrapper for sockets (closes on destruction).
class Socket {
 public:
  Socket() noexcept = default;
  explicit Socket(int fd) noexcept : fd_(fd) {}
  Socket(const Socket&) = delete;
  Socket& operator=(const Socket&) = delete;
  Socket(Socket&& o) noexcept : fd_(o.release()) {}
  Socket& operator=(Socket&& o) noexcept;
  ~Socket();

  [[nodiscard]] int get() const noexcept { return fd_; }
  [[nodiscard]] bool valid() const noexcept { return fd_ >= 0; }
  int release() noexcept;

 private:
  int fd_{-1};
};

/// Creates a socket of the given type (SOCK_STREAM / SOCK_DGRAM); throws std::runtime_error on failure.
[[nodiscard]] Socket openSocket(int type);

/// Builds an IPv4 address; throws std::invalid_argument for a malformed address string.
[[nodiscard]] sockaddr_in makeAddress(const std::string& ip, std::uint16_t port);

/// Port a bound socket is listening on.
[[nodiscard]] std::uint16_t localPort(int fd);

/// "<what>: <strerror(errno)>"
[[nodiscard]] std::string errnoMessage(const std::string& what);

/// Waits until the descriptor is readable. Returns false on timeout.
[[nodiscard]] bool waitReadable(int fd, int timeoutMs);

}  // namespace imu
