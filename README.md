# pypilot-boatimu

Backend-neutral BoatIMU abstraction layer for the modular C++ pypilot port.

This repository is not a hardcoded I2C IMU driver. It defines normalized sample types and runtime-facing interfaces for IMU, AHRS, compass, and marine-motion inputs. Backends such as RTIMULib2, ocean-imu, serial AHRS devices, network feeds, or replay sources should adapt their outputs into these common sample types.

## Original PyPilot context

The original PyPilot project supports RTIMULib2-based IMU workflows. This C++ port keeps that path possible but does not make RTIMULib2 the core abstraction. BoatIMU consumes samples; drivers and libraries produce samples.

## Design goals

- Accept already-calibrated IMU input.
- Accept already-fused attitude input.
- Accept heading-only sources.
- Accept marine-motion outputs such as heave/surge/sway when available.
- Keep low-level transport outside the core: I2C, SPI, UART, USB serial, TCP, UDP, CAN, SignalK, NMEA, and file replay are backend concerns.
- Keep source calibration separate from boat mounting alignment.
- Allow optional adapters for RTIMULib2 and ocean-imu without forcing those dependencies into the core module.

## Current contents

- Repository scaffold.
- Linux CMake build and tests.
- Arduino library metadata and compile example.
- Core sample types:
  - `CalibratedImuSample`
  - `FusedAttitudeSample`
  - `HeadingSample`
  - `MarineMotionSample`
- Sample validation helpers.
- Source/device arbitration.
- Stale source timeout handling.
- Mounting alignment helpers for calibrated IMU vectors, heading, Euler attitude, quaternion attitude, and marine-motion samples.
- Runtime coordinator that accepts calibrated IMU, fused attitude, heading, and marine-motion samples.

## Planned additions

- Data-model writer.
- Optional RTIMULib2 adapter.
- Optional ocean-imu adapter.
- Replay and generic transport adapters.

## Build on Linux

```bash
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

## Arduino compile smoke test

```bash
arduino-cli compile --fqbn arduino:avr:mega --libraries . examples/arduino/BoatImuSamplesExample
```
