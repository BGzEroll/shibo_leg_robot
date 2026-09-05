# foc_motor source provenance

This directory contains the subset of SimpleFOC used by this firmware.

- Upstream project: Arduino-FOC/simplefoc
- Upstream package: askuric/Simple FOC
- Imported version: 2.2.3
- License: MIT; see `LICENSE`
- Local change: `SimpleFOC.h` only includes the motor, 3-PWM driver,
  AS5600-compatible I2C sensor, PID controller, and low-pass filter used by
  this project.
- Local change: `BLDCMotor::loopFOC()` no longer reads its linked sensor.
  Runtime samples are supplied by the application before each FOC iteration;
  explicit sensor reads used during `initFOC()` calibration remain unchanged.

The original public types and function signatures are retained. Runtime sensor
ownership differs from upstream as documented above.
