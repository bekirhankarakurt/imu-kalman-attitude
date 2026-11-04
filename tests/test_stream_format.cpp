#include "imu/stream_format.hpp"
#include "test_framework.hpp"

using namespace imu;

namespace {
StreamFormat formatOf(const char* spec) {
  StreamFormat f;
  f.columns = parseColumns(spec).columns;
  return f;
}
}  // namespace

TEST(columns_accept_valid_layouts) {
  CHECK(parseColumns("ax,ay,az,gx,gy,gz").error.empty());
  const ColumnsParse full = parseColumns(" t , ax,ay,az,gx,gy,gz,skip,mx,my,mz");
  CHECK(full.error.empty());
  CHECK(full.columns.size() == 11);
}

TEST(columns_reject_invalid_layouts) {
  CHECK(!parseColumns("ax,ay,az").error.empty());
  CHECK(!parseColumns("ax,ay,az,gx,gy,gz,mx").error.empty());
  CHECK(!parseColumns("ax,ax,az,gx,gy,gz").error.empty());
  CHECK(!parseColumns("ax,ay,az,gx,gy,gz,foo").error.empty());
  CHECK(!parseColumns("t,tms,ax,ay,az,gx,gy,gz").error.empty());
}

TEST(parse_line_reads_all_fields) {
  const StreamFormat f = formatOf("tms,ax,ay,az,gx,gy,gz,mx,my,mz");
  const auto s = parseLine(f, "1500, 0.1, -0.2, 9.8, 0.01, 0.02, 0.03, 20, 0, -40\r\n");
  CHECK(s.has_value());
  CHECK_NEAR(*s->time, 1.5, 1e-12);
  CHECK_NEAR(s->imu.accel.z, 9.8, 1e-12);
  CHECK_NEAR(s->imu.gyro.y, 0.02, 1e-12);
  CHECK(s->imu.mag.has_value());
  CHECK_NEAR(s->imu.mag->z, -40.0, 1e-12);
}

TEST(parse_line_without_optional_columns) {
  const StreamFormat f = formatOf("ax,ay,az,gx,gy,gz");
  const auto mixed = parseLine(f, "0,0,9.81;0.1;0.2;0.3;extra");  // ',' ';' and tab all separate fields
  CHECK(mixed.has_value());
  CHECK_NEAR(mixed->imu.gyro.z, 0.3, 1e-12);
  const auto tabs = parseLine(f, "0\t0\t9.81\t0.1\t0.2\t0.3");
  CHECK(tabs.has_value());
  CHECK(!tabs->time.has_value());
  CHECK(!tabs->imu.mag.has_value());
}

TEST(parse_line_unit_conversion) {
  StreamFormat f = formatOf("ax,ay,az,gx,gy,gz");
  f.gyroDegrees = true;
  f.accelInG = true;
  const auto s = parseLine(f, "0,0,1,180,0,0");
  CHECK(s.has_value());
  CHECK_NEAR(s->imu.gyro.x, kPi, 1e-12);
  CHECK_NEAR(s->imu.accel.z, kGravity, 1e-12);
}

TEST(parse_line_rejects_bad_input) {
  const StreamFormat f = formatOf("ax,ay,az,gx,gy,gz");
  CHECK(!parseLine(f, "").has_value());
  CHECK(!parseLine(f, "ax,ay,az,gx,gy,gz").has_value());
  CHECK(!parseLine(f, "1,2,3,4,5").has_value());
  CHECK(!parseLine(f, "1,2,3,4,5,nan").has_value());
  CHECK(!parseLine(f, "1,2,3,4,5,6x").has_value());
  CHECK(!parseLine(f, "1,2,,4,5,6").has_value());
}
