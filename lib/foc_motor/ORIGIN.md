# foc_motor source provenance

This directory contains the subset of SimpleFOC used by this firmware.

- Upstream project: Arduino-FOC/simplefoc
- Upstream package: askuric/Simple FOC
- Imported version: 2.2.3
- License: MIT; see `LICENSE`
- Local change: `SimpleFOC.h` only includes the motor, 3-PWM driver,
  AS5600-compatible I2C sensor, PID controller, and low-pass filter used by
  this project.

The original public types and behavior are retained so existing motor and
control call sites do not need an API migration.
