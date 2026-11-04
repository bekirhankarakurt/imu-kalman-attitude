#include <atomic>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include "imu/app.hpp"
#include "imu/cli.hpp"
#include "test_framework.hpp"

using namespace imu;

namespace {

ParseResult parse(std::initializer_list<const char*> args) {
  return parseArgs(std::vector<std::string>(args.begin(), args.end()));
}

struct Outcome {
  int code;
  std::string out;
  std::string err;
};

Outcome runWith(const Options& o) {
  std::ostringstream out, err;
  const std::atomic<bool> stop{false};
  const int code = run(o, out, err, stop);
  return {code, out.str(), err.str()};
}

std::string tempPath(const char* name) { return std::string("imu_test_") + name; }

}  // namespace

TEST(cli_defaults_and_overrides) {
  const ParseResult def = parse({});
  CHECK(def.error.empty());
  CHECK(def.options.source == SourceKind::Sim);
  CHECK(!def.options.httpPort.has_value());

  const ParseResult r = parse({"--source", "udp", "--port", "6000", "--http", "8080", "--show", "mahony",
                               "--gyro-bias", "0.1,0.2,0.3", "--gyro-deg", "--duration", "0"});
  CHECK(r.error.empty());
  CHECK(r.options.source == SourceKind::Udp);
  CHECK(r.options.udpPort == 6000);
  CHECK(*r.options.httpPort == 8080);
  CHECK(r.options.show == FilterKind::Mahony);
  CHECK_NEAR(r.options.gyroBias.z, 0.3, 1e-12);
  CHECK(r.options.gyroDegrees);
  CHECK(parse({"--help"}).options.help);
}

TEST(cli_rejects_bad_input) {
  CHECK(!parse({"--bogus"}).error.empty());
  CHECK(!parse({"--rate"}).error.empty());
  CHECK(!parse({"--rate", "abc"}).error.empty());
  CHECK(!parse({"--rate", "0"}).error.empty());
  CHECK(!parse({"--port", "70000"}).error.empty());
  CHECK(!parse({"--source", "serial"}).error.empty());
  CHECK(!parse({"--gyro-bias", "1,2"}).error.empty());
  CHECK(!parse({"--source", "file"}).error.empty());
  CHECK(!parse({"--profile", "wild"}).error.empty());
}

TEST(run_simulation_prints_accuracy_table) {
  Options o;
  o.duration = 20.0;
  o.printHz = 1.0;
  const Outcome r = runWith(o);
  CHECK(r.code == 0);
  CHECK(r.out.find("roll") != std::string::npos);
  CHECK(r.out.find("error vs ground truth") != std::string::npos);
  CHECK(r.out.find("complementary") != std::string::npos);
  CHECK(r.out.find("processed 2001 samples") != std::string::npos);
}

TEST(run_writes_log_and_replays_it_from_file) {
  const std::string csv = tempPath("replay.csv");
  const std::string log = tempPath("log.csv");

  // Record a simulated stream in the phone CSV layout, then replay it through the file source.
  {
    std::ofstream f(csv);
    f << "timestamp_ms,ax,ay,az,gx,gy,gz\n";
    for (int i = 0; i < 300; ++i) f << i * 10 << ",0,0,9.81,0,0,0.5\n";
  }
  Options o;
  o.source = SourceKind::File;
  o.file = csv;
  o.columns = "tms,ax,ay,az,gx,gy,gz";
  o.logPath = log;
  o.quiet = true;
  const Outcome r = runWith(o);
  CHECK(r.code == 0);
  CHECK(r.out.find("processed 300 samples (1 unparsable lines skipped)") != std::string::npos);
  CHECK(r.out.find("error vs ground truth") == std::string::npos);

  std::ifstream in(log);
  std::string header, firstRow;
  std::getline(in, header);
  std::getline(in, firstRow);
  CHECK(header.rfind("t,truth_roll", 0) == 0);
  CHECK(firstRow.rfind("0.0000,,,,", 0) == 0);
  std::remove(csv.c_str());
  std::remove(log.c_str());
}

TEST(run_reports_errors) {
  Options o;
  o.source = SourceKind::File;
  o.file = tempPath("missing.csv");
  Outcome r = runWith(o);
  CHECK(r.code == 1);
  CHECK(r.err.find("cannot open") != std::string::npos);

  o.file = "x";
  o.columns = "ax,ay";
  r = runWith(o);
  CHECK(r.code == 1);
  CHECK(r.err.find("--columns") != std::string::npos);

  Options sim;
  sim.duration = 1.0;
  sim.logPath = "/nonexistent-dir/log.csv";
  CHECK(runWith(sim).code == 1);
}

TEST(run_serves_viewer_state_while_running) {
  Options o;
  o.duration = 2.0;
  o.quiet = true;
  o.httpPort = 0;
  o.rateHz = 200.0;
  const Outcome r = runWith(o);
  CHECK(r.code == 0);
  CHECK(r.out.find("viewer: http://127.0.0.1:") != std::string::npos);
}
