# Security Policy

## Supported versions

| Version | Supported |
|---|---|
| 1.x | yes |
| < 1.0 | no |

## Reporting a vulnerability

Please report vulnerabilities privately through GitHub Security Advisories:
[Report a vulnerability](https://github.com/bekirhankarakurt/imu-kalman-attitude/security/advisories/new).
Do not open a public issue for security problems.

Include the affected version, a description of the issue and steps to reproduce it. You can expect an initial response within a week.

## Scope notes

- The built-in HTTP server has no authentication and binds to `127.0.0.1` by default. Exposing it with `--bind 0.0.0.0` is intended for trusted networks only.
- The UDP listener accepts datagrams from any host on the configured port; malformed input is rejected line by line.
