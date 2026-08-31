# Pitch-Axis Stabilization Loop

A single-axis flight controller on an ESP32: an MPU6050 is fused with a complementary
filter to estimate pitch, and a PID loop drives two brushed motors to hold the arm level
against hand disturbances.

## Demo

<video src="https://github.com/dinud2/Pitch-Axis-Stabilization-loop/raw/main/media/demo.mp4" poster="https://github.com/dinud2/Pitch-Axis-Stabilization-loop/raw/main/media/demo-poster.jpg" width="320" controls muted playsinline></video>

The arm is knocked off level by hand and the loop drives it back to the setpoint.
If the player above does not load, [watch the demo here](media/demo.mp4).

## How it works

- **Sensing** — MPU6050 over I²C; accelerometer gives absolute pitch, gyro gives rate.
- **Estimation** — complementary filter (`ALPHA = 0.98`) blends gyro integration with the
  accelerometer angle to get a drift-free, low-noise pitch.
- **Control** — PID on pitch error, with integral clamping and an output limit to keep
  corrections inside motor authority.
- **Actuation** — 20 kHz / 10-bit LEDC PWM to two motors, mixed differentially around a
  base throttle.
- **Safety** — arming gate (button or serial `a1`/`a0`) so the motors cannot spin up
  unattended, plus a raw-duty test mode for finding the motor stall point.

Live tuning of `Kp`/`Ki`/`Kd`, setpoint, base throttle, and mix sign is exposed over the
serial console, so gains can be adjusted without reflashing.
