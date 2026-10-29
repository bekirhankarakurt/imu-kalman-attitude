#include <atomic>
#include <csignal>
#include <iostream>
#include <string>
#include <vector>

#include "imu/app.hpp"
#include "imu/cli.hpp"

namespace {

std::atomic<bool> g_stop{false};

extern "C" void onSignal(int) { g_stop.store(true); }

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);
  const imu::ParseResult parsed = imu::parseArgs(args);
  if (!parsed.error.empty()) {
    std::cerr << "error: " << parsed.error << "\n\n" << imu::usage();
    return 2;
  }
  if (parsed.options.help) {
    std::cout << imu::usage();
    return 0;
  }
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  return imu::run(parsed.options, std::cout, std::cerr, g_stop);
}
