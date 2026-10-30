#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <string>

#include "imu/http_server.hpp"
#include "imu/sources.hpp"
#include "imu/state.hpp"
#include "test_framework.hpp"

using namespace imu;

namespace {

Snapshot sampleSnapshot() {
  Snapshot s;
  s.t = 1.5;
  s.samples = 150;
  s.rateHz = 100.0;
  s.selected = "ekf";
  s.filters.push_back({"ekf", Quat::fromEuler({0.1, 0.2, 0.3}), {0.01, 0.0, 0.0}});
  return s;
}

std::string httpGet(std::uint16_t port, const std::string& path) {
  Socket sock(::socket(AF_INET, SOCK_STREAM, 0));
  const sockaddr_in addr = makeAddress("127.0.0.1", port);
  if (::connect(sock.get(), reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) return {};
  const std::string req = "GET " + path + " HTTP/1.1\r\nHost: x\r\n\r\n";
  if (::send(sock.get(), req.data(), req.size(), 0) < 0) return {};
  std::string reply;
  char buf[2048];
  ssize_t n = 0;
  while ((n = ::recv(sock.get(), buf, sizeof(buf), 0)) > 0) reply.append(buf, static_cast<std::size_t>(n));
  return reply;
}

}  // namespace

TEST(json_contains_expected_fields) {
  const std::string j = toJson(sampleSnapshot());
  CHECK(j.find("\"selected\":\"ekf\"") != std::string::npos);
  CHECK(j.find("\"roll\":5.7296") != std::string::npos);
  CHECK(j.find("\"truth\":null") != std::string::npos);
  CHECK(j.find("nan") == std::string::npos);
  Snapshot s = sampleSnapshot();
  s.truth = Quat{};
  s.t = std::nan("");
  const std::string j2 = toJson(s);
  CHECK(j2.find("\"truth\":{\"q\"") != std::string::npos);
  CHECK(j2.find("nan") == std::string::npos);
}

TEST(respond_routes_requests) {
  StateStore store;
  store.publish(sampleSnapshot());
  const std::string page = HttpServer::respond(store, "GET / HTTP/1.1\r\n\r\n");
  CHECK(page.rfind("HTTP/1.1 200 OK", 0) == 0);
  CHECK(page.find("<canvas") == std::string::npos);
  CHECK(page.find("matrix3d") != std::string::npos);
  const std::string api = HttpServer::respond(store, "GET /api/state?x=1 HTTP/1.1\r\n\r\n");
  CHECK(api.find("application/json") != std::string::npos);
  CHECK(api.find("\"samples\":150") != std::string::npos);
  CHECK(HttpServer::respond(store, "GET /nope HTTP/1.1\r\n\r\n").rfind("HTTP/1.1 404", 0) == 0);
  CHECK(HttpServer::respond(store, "POST / HTTP/1.1\r\n\r\n").rfind("HTTP/1.1 405", 0) == 0);
  CHECK(HttpServer::respond(store, "garbage").rfind("HTTP/1.1 400", 0) == 0);
}

TEST(http_server_serves_over_loopback) {
  StateStore store;
  store.publish(sampleSnapshot());
  HttpServer server(store, "127.0.0.1", 0);
  CHECK(server.port() != 0);
  const std::string api = httpGet(server.port(), "/api/state");
  CHECK(api.rfind("HTTP/1.1 200 OK", 0) == 0);
  CHECK(api.find("\"name\":\"ekf\"") != std::string::npos);
  CHECK(httpGet(server.port(), "/").find("<title>IMU attitude viewer</title>") != std::string::npos);
}

TEST(udp_source_parses_datagrams) {
  StreamFormat fmt;
  fmt.columns = parseColumns("tms,ax,ay,az,gx,gy,gz").columns;
  const std::atomic<bool> stop{false};
  UdpSource source(0, fmt, stop);

  Socket sender(::socket(AF_INET, SOCK_DGRAM, 0));
  const sockaddr_in dest = makeAddress("127.0.0.1", source.port());
  const std::string datagram = "header,line\n1000,0,0,9.81,0.1,0,0\n2000,0,0,9.81,0.2,0,0\n";
  CHECK(::sendto(sender.get(), datagram.data(), datagram.size(), 0, reinterpret_cast<const sockaddr*>(&dest),
                 sizeof(dest)) > 0);

  Frame f;
  CHECK(source.next(f));
  CHECK_NEAR(f.imu.t, 1.0, 1e-12);
  CHECK_NEAR(f.imu.gyro.x, 0.1, 1e-12);
  CHECK(source.next(f));
  CHECK_NEAR(f.imu.t, 2.0, 1e-12);
  CHECK(source.rejectedLines() == 1);
  CHECK(!f.truth.has_value());
}

TEST(udp_source_stops_when_requested) {
  StreamFormat fmt;
  fmt.columns = parseColumns("ax,ay,az,gx,gy,gz").columns;
  const std::atomic<bool> stop{true};
  UdpSource source(0, fmt, stop);
  Frame f;
  CHECK(!source.next(f));
}
