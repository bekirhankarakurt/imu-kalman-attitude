#include "imu/http_server.hpp"

#include <sys/socket.h>
#include <sys/time.h>

#include <stdexcept>

namespace imu {
namespace {

std::string response(int status, std::string_view reason, std::string_view type, std::string_view body) {
  std::string out = "HTTP/1.1 " + std::to_string(status) + " " + std::string(reason) +
                    "\r\nContent-Type: " + std::string(type) +
                    "\r\nContent-Length: " + std::to_string(body.size()) +
                    "\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n";
  out.append(body);
  return out;
}

}  // namespace

std::string HttpServer::respond(const StateStore& store, std::string_view request) {
  const std::size_t lineEnd = request.find("\r\n");
  const std::string_view line = request.substr(0, lineEnd);
  const std::size_t sp1 = line.find(' ');
  const std::size_t sp2 = sp1 == std::string_view::npos ? sp1 : line.find(' ', sp1 + 1);
  if (sp2 == std::string_view::npos) return response(400, "Bad Request", "text/plain", "bad request\n");

  if (line.substr(0, sp1) != "GET") return response(405, "Method Not Allowed", "text/plain", "GET only\n");
  std::string_view path = line.substr(sp1 + 1, sp2 - sp1 - 1);
  path = path.substr(0, path.find('?'));

  if (path == "/" || path == "/index.html")
    return response(200, "OK", "text/html; charset=utf-8", viewerHtml());
  if (path == "/api/state") return response(200, "OK", "application/json", store.json());
  return response(404, "Not Found", "text/plain", "not found\n");
}

HttpServer::HttpServer(const StateStore& store, const std::string& bindAddress, std::uint16_t port)
    : store_(store) {
  listener_ = Socket(::socket(AF_INET, SOCK_STREAM, 0));
  if (!listener_.valid()) throw std::runtime_error(errnoMessage("socket"));
  const int on = 1;
  ::setsockopt(listener_.get(), SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
  const sockaddr_in addr = makeAddress(bindAddress, port);
  if (::bind(listener_.get(), reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0)
    throw std::runtime_error(errnoMessage("cannot bind HTTP port " + std::to_string(port)));
  if (::listen(listener_.get(), 8) != 0) throw std::runtime_error(errnoMessage("listen"));
  port_ = localPort(listener_.get());
  thread_ = std::thread([this] { serve(); });
}

HttpServer::~HttpServer() {
  stopping_.store(true);
  if (thread_.joinable()) thread_.join();
}

void HttpServer::serve() {
  while (!stopping_.load()) {
    if (!waitReadable(listener_.get(), 100)) continue;
    Socket client(::accept(listener_.get(), nullptr, nullptr));
    if (client.valid()) handle(client.get());
  }
}

void HttpServer::handle(int client) const {
  const timeval timeout{1, 0};
  ::setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

  std::string request;
  char buffer[1024];
  while (request.size() < 8192 && request.find("\r\n\r\n") == std::string::npos) {
    const ssize_t n = ::recv(client, buffer, sizeof(buffer), 0);
    if (n <= 0) break;
    request.append(buffer, static_cast<std::size_t>(n));
  }
  if (request.empty()) return;

  const std::string reply = respond(store_, request);
  std::size_t sent = 0;
  while (sent < reply.size()) {
    const ssize_t n = ::send(client, reply.data() + sent, reply.size() - sent, 0);
    if (n <= 0) break;
    sent += static_cast<std::size_t>(n);
  }
}

}  // namespace imu
