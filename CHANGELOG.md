# Changelog

All notable changes to this project are documented here.
The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses [Semantic Versioning](https://semver.org/).

## [Unreleased]

### Fixed
- EKF yaw no longer drifts by several deg/s on a device lying still without a magnetometer. Tilt updates moved the unobservable z gyro bias through spurious cross-covariance; while the rotation rate is below `EkfParams::stillRate` (0.05 rad/s) the gravity update now leaves attitude and bias about the gravity axis untouched. Static no-magnetometer yaw RMSE over 60 s drops from 10-82 deg to 14 deg (same as Mahony); all other simulated cases are unchanged.

## [1.0.0] - 2026-11-04

### Added
- Complementary filter, Mahony filter and quaternion error-state extended Kalman filter with gyro bias estimation.
- Deterministic IMU simulator (gyro bias and random walk, white noise, linear acceleration, magnetometer) with ground truth.
- UDP CSV input from Android sensor-streaming apps, CSV file replay and configurable column layouts.
- Terminal roll/pitch/yaw output, CSV logging and an RMSE accuracy report against simulated truth.
- Built-in HTTP server with a self-contained 3D viewer and a `/api/state` JSON endpoint.
- Unit and accuracy tests run through CTest; CI with GCC, Clang, ASan/UBSan, cppcheck and clang-tidy.

[1.0.0]: https://github.com/bekirhankarakurt/imu-kalman-attitude/releases/tag/v1.0.0
