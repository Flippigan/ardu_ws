# Arduino Direct Servo — Remove PCA9685

**Date:** 2026-04-03
**Status:** Approved
**Supersedes:** `2026-04-02-arduino-pca9685-payload-servo-design.md` (PCA9685 approach)

## Summary

Replace the PCA9685 I2C servo driver with direct PWM output from the Arduino Nano on pin D9. The payload mechanism uses a single servo — a 16-channel driver is unnecessary. The serial protocol (`S<channel>:<pwm_us>\n` -> `OK\n` / `ERR:<reason>\n`) remains unchanged, preserving full compatibility with `arduino_interface_node` and all existing tests.

## Motivation

- Simpler wiring: eliminates I2C bus (SDA/SCL) and PCA9685 board
- Fewer failure points: one less component in the signal path
- No library dependency on `Adafruit_PWMServoDriver`

## Design

### Arduino Sketch Changes (`payload_servo_controller.ino`)

**Remove:**
- `#include <Wire.h>`
- `#include <Adafruit_PWMServoDriver.h>`
- `Adafruit_PWMServoDriver pwm` object
- `Wire.begin()`, `pwm.begin()`, `pwm.setPWMFreq(50)`
- PCA9685 tick conversion (`pwm_us * 4096 / 20000`)
- `pwm.setPWM(channel, 0, ticks)` call
- `MAX_CHANNEL = 15` constant

**Add:**
- `#include <Servo.h>`
- `Servo payloadServo` object
- `payloadServo.attach(9)` in `setup()` (pin D9)
- `payloadServo.writeMicroseconds(pwm_us)` in `handleCommand()`

**Channel handling:**
- Accept channel 0 only; reject all others with `ERR:channel out of range`
- This is backward compatible with `payload_servo_channel: 0` in `mission_params.yaml`

**PWM range validation:**
- Keep existing 0-20000 microsecond range check (the `Servo` library handles clamping internally, but explicit validation maintains clear error messages)

### Config Change (`mission_params.yaml`)

- Update comment from `# ── Payload Servo (Arduino + PCA9685)` to `# ── Payload Servo (Arduino Nano direct)`
- No parameter value changes required

### No Changes Required

- `arduino_interface_node.py` — protocol unchanged
- `test_arduino_interface.py` — tests pure protocol functions only
- `DoSetServo.srv` — message definition unchanged
- Launch files — no Arduino sketch awareness
- `mission_sequencer_node.py` — calls `DoSetServo` service, unaware of hardware

## Wiring Reference

| Servo Wire | Arduino Nano Pin |
|------------|------------------|
| Signal (orange/white) | D9 (PWM-capable) |
| VCC (red) | External 5-6V supply |
| GND (brown/black) | GND (shared with external supply) |

**Do not power the servo from the Arduino 5V pin** — the servo draws too much current.
