#include "imu/net.hpp"

#include <arpa/inet.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace imu {

Socket& Socket::operator=(Socket&& o) noexcept {
  if (this != &o) {
    if (fd_ >= 0) ::close(fd_);
    fd_ = o.release();
  }
  return *this;
}

Socket::~Socket() {
  if (fd_ >= 0) ::close(fd_);
}

int Socket::release() noexcept {
  const int fd = fd_;
  fd_ = -1;
  return fd;
}

sockaddr_in makeAddress(const std::string& ip, std::uint16_t port) {
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  if (::inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) != 1)
    throw std::invalid_argument("invalid IPv4 address '" + ip + "'");
  return addr;
}

std::uint16_t localPort(int fd) {
  sockaddr_in addr{};
  socklen_t len = sizeof(addr);
  if (::getsockname(fd, reinterpret_cast<sockaddr*>(&addr), &len) != 0)
    throw std::runtime_error(errnoMessage("getsockname"));
  return ntohs(addr.sin_port);
}

std::string errnoMessage(const std::string& what) { return what + ": " + std::strerror(errno); }

bool waitReadable(int fd, int timeoutMs) {
  pollfd p{};
  p.fd = fd;
  p.events = POLLIN;
  const int rc = ::poll(&p, 1, timeoutMs);
  return rc > 0 && (p.revents & (POLLIN | POLLHUP)) != 0;
}

}  // namespace imu
