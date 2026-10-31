# imu-kalman-attitude

Real-time roll/pitch/yaw from a phone's IMU over UDP, using a complementary filter, Mahony and a quaternion error-state Kalman filter with gyro-bias estimation, plus a built-in simulator and a live 3D browser viewer.

[![CI](https://github.com/bekirhankarakurt/imu-kalman-attitude/actions/workflows/ci.yml/badge.svg)](https://github.com/bekirhankarakurt/imu-kalman-attitude/actions/workflows/ci.yml)
[![CodeQL](https://github.com/bekirhankarakurt/imu-kalman-attitude/actions/workflows/codeql.yml/badge.svg)](https://github.com/bekirhankarakurt/imu-kalman-attitude/actions/workflows/codeql.yml)
[![Coverage](https://img.shields.io/endpoint?url=https%3A%2F%2Fbekirhankarakurt.github.io%2Fimu-kalman-attitude%2Fcoverage.json)](https://bekirhankarakurt.github.io/imu-kalman-attitude/coverage/)
[![Release](https://img.shields.io/github/v/release/bekirhankarakurt/imu-kalman-attitude)](https://github.com/bekirhankarakurt/imu-kalman-attitude/releases)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![C++17](https://img.shields.io/badge/C%2B%2B-17-00599C.svg)](https://en.cppreference.com/w/cpp/17)
[![Platform](https://img.shields.io/badge/platform-Linux-lightgrey.svg)](#requirements)

![Terminal demo of imu_attitude on simulated motion](docs/demo.gif)

**Documentation:** [API reference and coverage report](https://bekirhankarakurt.github.io/imu-kalman-attitude/) - [Changelog](CHANGELOG.md) - [Contributing](CONTRIBUTING.md)

## Why / what

- **Three estimators, one stream.** Complementary, Mahony and an error-state EKF run side by side on identical data, so you can compare them directly.
- **Gyro bias estimation.** Mahony (integral term) and the EKF (explicit bias states) learn the gyro offset online; the EKF reports it in deg/s.
- **Hardware optional.** A deterministic simulator (bias, random walk, white noise, linear acceleration, magnetometer) provides ground truth, so accuracy is measured, not guessed. Every test runs without a phone.
- **Real input.** Streams CSV over UDP from a free Android app, or replays a recorded CSV file.
- **Live view.** A tiny built-in HTTP server hosts a self-contained page (no CDN, no build step) that polls JSON and rotates a 3D box.
- **Dependency-free.** C++17 standard library plus POSIX sockets; no Eigen, no Boost.

## Demo

`./build/imu_attitude --duration 12 --print-hz 1` (simulated sway motion, EKF shown, error measured against ground truth):

```text
t=   0.00s  ekf           roll    2.21  pitch   16.84  yaw   -2.98 deg | bias  +0.00  +0.00  +0.00 deg/s | err 4.20 deg
t=   1.00s  ekf           roll   21.97  pitch   19.94  yaw   19.97 deg | bias  +5.76  -4.79  -1.29 deg/s | err 2.95 deg
t=   2.00s  ekf           roll   14.30  pitch    4.45  yaw   39.04 deg | bias  +2.02  -1.67  -2.04 deg/s | err 3.87 deg
t=   4.00s  ekf           roll  -23.15  pitch  -19.93  yaw   56.65 deg | bias  +1.07  -0.99  +0.95 deg/s | err 0.63 deg
t=   8.00s  ekf           roll  -14.80  pitch   15.03  yaw   35.45 deg | bias  +1.18  -0.83  +0.59 deg/s | err 0.51 deg
t=  12.00s  ekf           roll   14.94  pitch   -5.24  yaw  -35.14 deg | bias  +1.13  -0.85  +0.57 deg/s | err 0.33 deg

processed 1201 samples

error vs ground truth, RMSE after 3.0 s warm-up [deg]
filter          roll   pitch     yaw  attitude  bias err [deg/s]
complementary   1.22    0.77    1.17      1.80   1.548
mahony          0.73    0.51    0.62      1.05   0.757
ekf             0.52    0.43    0.67      0.87   0.028
```

The simulated gyro bias is (1.15, -0.86, 0.57) deg/s; the EKF converges to it within seconds (final error 0.03 deg/s).

## Quick start (60 seconds)

Prebuilt `linux-x86_64` / `linux-aarch64` archives are on the [Releases](https://github.com/bekirhankarakurt/imu-kalman-attitude/releases) page. To build from source:

```bash
git clone https://github.com/bekirhankarakurt/imu-kalman-attitude.git
cd imu-kalman-attitude
cmake -S . -B build && cmake --build build -j
ctest --test-dir build --output-on-failure

./build/imu_attitude --duration 0 --http 8080   # open http://127.0.0.1:8080/
```

With a phone ([Android setup](#using-a-phone)):

```bash
./build/imu_attitude --source udp --port 5555 --columns ax,ay,az,gx,gy,gz,mx,my,mz --http 8080
```

## Requirements

| Item | Version |
|---|---|
| OS | Linux (POSIX sockets; macOS should work but is not tested in CI) |
| Compiler | GCC >= 9 or Clang >= 10 with C++17 |
| CMake | >= 3.16 |
| Libraries | none (pthreads from the C library) |
| Phone (optional) | Android with a free sensor-streaming app that sends CSV over UDP |

### Build options

| CMake option | Default | Meaning |
|---|---|---|
| `CMAKE_BUILD_TYPE` | `Release` | Standard CMake build type |
| `IMU_BUILD_TESTS` | `ON` | Build the unit and accuracy tests |
| `IMU_WARNINGS_AS_ERRORS` | `OFF` | Add `-Werror` (CI uses `ON`) |
| `IMU_SANITIZE` | `OFF` | AddressSanitizer + UndefinedBehaviorSanitizer |

The code builds warning-free with `-Wall -Wextra -Wpedantic -Wshadow -Wconversion`.

## Using a phone

1. Install **HyperIMU** (free, Android, by ianovir) from Google Play.
2. Enable *Accelerometer*, *Gyroscope* and (optionally) *Magnetometer*. Choose the **UDP** stream protocol, set the PC's IP address and port (e.g. `5555`), and start streaming.
3. The line is plain CSV: one value per enabled sensor axis, in the order the app lists them. Pass that order to `--columns`. If you enable the app's timestamp, add `t` (seconds) or `tms` (milliseconds) as the first column; otherwise packet arrival time is used.

Any app that sends CSV lines over UDP works, as long as it uses Android units (accelerometer m/s^2, gyroscope rad/s). Use `--gyro-deg` / `--accel-g` if the app reports deg/s or g, and `skip` in `--columns` for fields you do not need.

Phone orientation convention (Android): x to the right, y to the top, z out of the screen. Lying flat, the accelerometer reads +9.81 on z, which the filters treat as roll = pitch = 0. Yaw is relative to magnetic north when the magnetometer is enabled, otherwise to the heading at start-up.

## Usage

```text
imu_attitude [options]
```

| Option | Default | Description |
|---|---|---|
| `--source sim\|udp\|file` | `sim` | Sample source |
| `--port N` | `5555` | UDP port to listen on |
| `--file PATH` | - | CSV recording to replay |
| `--columns LIST` | `ax,ay,az,gx,gy,gz` | CSV layout: `t`, `tms`, `ax ay az`, `gx gy gz`, `mx my mz`, `skip` |
| `--gyro-deg` / `--accel-g` | off | Input gyro in deg/s / accel in g |
| `--no-mag` | off | Ignore magnetometer data |
| `--duration S` | `30` | Simulated seconds; `0` runs until Ctrl-C |
| `--rate HZ` | `100` | Simulator rate; replay rate for files without timestamps |
| `--profile NAME` | `sway` | Simulated motion: `static`, `sway`, `aggressive` |
| `--seed N` | `1` | Simulator random seed |
| `--gyro-bias X,Y,Z` | `0.02,-0.015,0.01` | True simulated gyro bias [rad/s] |
| `--gyro-noise` / `--accel-noise` / `--mag-noise` | `0.0007` / `0.005` / `0.4` | Simulator noise: gyro [rad/s/sqrt(Hz)], accel [m/s^2/sqrt(Hz)], mag [uT] |
| `--kp` / `--ki` | `1.5` / `0.1` | Mahony proportional / integral gain |
| `--tau S` | `1.0` | Complementary filter time constant |
| `--show NAME` | `ekf` | Filter shown in the terminal and selected in the viewer |
| `--http PORT` | off | Serve the viewer and `/api/state`; implies `--realtime` for sim/file |
| `--bind ADDR` | `127.0.0.1` | HTTP bind address (use `0.0.0.0` to view from another device) |
| `--realtime` | off | Pace sim/file replay at wall-clock speed |
| `--print-hz HZ` | `2` | Terminal update rate in stream time; `0` disables |
| `--log PATH` | - | Write all estimates (and truth) to CSV |
| `--quiet` | off | Final summary only |

Exit codes: `0` success, `1` runtime error (bad file, port in use), `2` usage error.

### Recipes

**1. Compare filters on aggressive motion with linear acceleration**

```bash
./build/imu_attitude --profile aggressive --duration 60 --quiet
```

**2. See what happens without a magnetometer** (roll/pitch stay accurate, yaw drifts)

```bash
./build/imu_attitude --duration 60 --quiet --no-mag
```

**3. Replay a recorded CSV file** (here: a 5 s recording made with `awk`, rotating about z at 0.5 rad/s)

```bash
awk 'BEGIN{for(i=0;i<500;i++) printf "%d,0.0,0.0,9.81,0.0,0.0,0.5\n", i*10}' > rec.csv
./build/imu_attitude --source file --file rec.csv --columns tms,ax,ay,az,gx,gy,gz --print-hz 0.5
```

**4. Test the UDP path without a phone** (bash only; run in a second terminal)

```bash
./build/imu_attitude --source udp --port 5555 &
echo "0.0,0.0,9.81,0.0,0.0,0.1" > /dev/udp/127.0.0.1/5555
```

**5. Log estimates for plotting**

```bash
./build/imu_attitude --duration 60 --quiet --log estimates.csv
head -2 estimates.csv   # t,truth_roll,truth_pitch,truth_yaw,complementary_roll,...,ekf_yaw (degrees)
```

## Architecture

```text
 Android app ──UDP CSV──┐
 CSV file ──────────────┼─► SampleSource ─► FilterBank ─┬─► Complementary ─┐
 ImuSimulator (+truth) ─┘   (stream_format) (same data) ├─► Mahony         ├─► terminal / CSV log
                                                        └─► Error-state EKF┘        │
                                                                            StateStore (mutex)
                                                                                    │
                                                        browser ◄── GET /api/state ─┴─ HttpServer
```

| Filter | Corrects with | Bias | Cost / notes |
|---|---|---|---|
| Complementary | accel tilt + mag heading, fixed time constant | none (steady-state error ~ bias x tau) | Simplest, robust |
| Mahony | same error signal, PI feedback on gyro | integral term | Two gains to tune |
| Error-state EKF | gravity vector + mag heading with covariance | 3 explicit states | Gains adapt to noise settings; 6x6 covariance, Joseph-form update |

Design trade-offs:

- **Error-state, not full-state.** The nominal quaternion is propagated by integration and the filter estimates a 3-D body-frame rotation error plus bias, so the covariance stays 6x6 and the unit-norm constraint is never violated.
- **Gravity as a vector measurement.** The EKF uses the normalised accelerometer vector (Jacobian `[g_b]x`), not Euler angles, so there is no gimbal-lock singularity in the update.
- **Adaptive accelerometer trust.** All filters scale or skip the accelerometer correction when `|a|` deviates from 1 g (up to 0.4 g taper). This protects against linear acceleration at the cost of slower correction during hard manoeuvres.
- **Yaw needs a magnetometer.** Without it yaw and the z-axis gyro bias are unobservable; yaw drifts and only roll/pitch are reliable. Magnetometer is used as a tilt-compensated heading only (no hard/soft-iron calibration; calibrate on the phone if heading is off).
- **Single-threaded HTTP.** One request at a time with a 1 s read timeout: plenty for a polling viewer, not a general web server.
- **Heading convention.** World frame is x = magnetic north, y = west, z = up; angles are ZYX Euler (roll, pitch, yaw).

## Data formats

### Input line

| Column name | Field | Unit |
|---|---|---|
| `t` / `tms` | timestamp (optional) | s / ms |
| `ax` `ay` `az` | accelerometer | m/s^2 (`--accel-g`: g) |
| `gx` `gy` `gz` | gyroscope | rad/s (`--gyro-deg`: deg/s) |
| `mx` `my` `mz` | magnetometer (optional, all three) | any (only direction is used) |
| `skip` | ignored field | - |

Fields are separated by `,` `;` or tab. Header lines, short lines and non-numeric values are skipped and counted. Extra trailing fields are ignored. Several lines per UDP datagram are accepted (include a timestamp column in that case).

### `GET /api/state`

```json
{"t":1.430,"samples":144,"rate_hz":100.0,"selected":"ekf",
 "filters":[{"name":"ekf","q":[w,x,y,z],"roll":0.0,"pitch":0.0,"yaw":0.0,"bias_dps":[0.0,0.0,0.0]}],
 "truth":null}
```

Angles in degrees; `q` is the body-to-world quaternion; `truth` is present only for simulated input.

## Testing

```bash
ctest --test-dir build --output-on-failure
```

| Test | Covers |
|---|---|
| `test_math` | Quaternion/Euler conversions, rotation vectors, matrix inverse |
| `test_stream_format` | Column layouts, delimiters, units, malformed lines |
| `test_filters` | Initialisation, convergence, gating, timestamp handling, EKF bias estimate |
| `test_accuracy` | RMSE against simulated ground truth for sway, aggressive and static motion, with and without magnetometer; EKF vs open-loop gyro; simulator determinism |
| `test_net` | JSON, HTTP routing and loopback server, UDP source |
| `test_app` | CLI parsing, end-to-end runs, file replay, CSV logging, error paths |

CI builds with GCC and Clang (warnings as errors), runs the whole suite again under ASan + UBSan, runs cppcheck and clang-tidy (`.clang-tidy`), CodeQL, and publishes a line-coverage report and Doxygen API docs to GitHub Pages.

## Troubleshooting / FAQ

**Nothing arrives over UDP.** Check the phone and PC are on the same network, the port matches, and the firewall allows UDP on it. Test the PC side with recipe 4.

**"unparsable lines skipped" is large.** `--columns` does not match the app's field order, or the app is sending a header/text fields. Add `skip` for unused fields.

**Roll/pitch look inverted or scaled wrongly.** The app is probably reporting deg/s or g; use `--gyro-deg` / `--accel-g`.

**Yaw drifts.** Expected without a magnetometer. Enable it in the app and include `mx,my,mz` in `--columns`. Near steel or electronics the heading will be biased.

**Viewer is empty or says "disconnected".** Open the URL printed at start-up. To view from another device use `--bind 0.0.0.0` (there is no authentication; use trusted networks only).

**Can I use it on Windows?** Not currently; the sockets layer is POSIX-only.

## Project layout

```text
.
├── CMakeLists.txt            build, warnings, sanitizer option
├── include/imu/
│   ├── math.hpp              Vec3, Quat, Euler, small fixed-size Mat
│   ├── filters.hpp           complementary, Mahony, error-state EKF, FilterBank
│   ├── simulator.hpp         deterministic IMU simulator with ground truth
│   ├── evaluation.hpp        RMSE accumulator
│   ├── stream_format.hpp     CSV column layout and line parser
│   ├── sources.hpp           simulation / file / UDP sample sources
│   ├── state.hpp             snapshot + JSON serialisation
│   ├── http_server.hpp       tiny HTTP server, embedded viewer
│   ├── net.hpp               RAII socket helpers
│   ├── cli.hpp, app.hpp      options and the run loop
├── src/                      implementations; viewer_html.cpp holds the 3D page; main.cpp
├── tests/                    dependency-free test framework + CTest suites
├── docs/                     demo.cast (asciinema recording) and demo.gif
├── Doxyfile, .clang-tidy     API docs and static-analysis configuration
├── CHANGELOG.md, SECURITY.md, CONTRIBUTING.md
└── .github/                  workflows (ci, codeql, pages, release), issue/PR templates, dependabot
```

## Roadmap

- [x] Complementary, Mahony and error-state EKF with gyro bias
- [x] Simulator with ground truth and accuracy tests
- [x] UDP CSV streaming, file replay, live 3D viewer
- [ ] Magnetometer hard/soft-iron calibration
- [ ] Accelerometer bias and linear-acceleration modelling in the EKF
- [ ] WebSocket push instead of polling
- [x] Static analysis, CodeQL, coverage, API docs and tagged releases
- [ ] macOS job in CI

## License

MIT - see [LICENSE](LICENSE).

Bekirhan Karakurt - [github.com/bekirhankarakurt](https://github.com/bekirhankarakurt)
