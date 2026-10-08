# Arduino Direct Servo — Remove PCA9685 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the PCA9685 I2C servo driver with direct Arduino Nano PWM output on pin D9, simplifying wiring and removing the Adafruit library dependency.

**Architecture:** The Arduino sketch is rewritten to use the built-in `Servo` library instead of `Adafruit_PWMServoDriver`. Channel 0 is the only valid channel (mapped to pin D9). The serial protocol (`S<channel>:<pwm_us>\n` -> `OK\n` / `ERR:<reason>\n`) is unchanged, so `arduino_interface_node.py`, all 6 existing tests in `test_arduino_interface.py`, and launch files require zero modifications.

**Tech Stack:** Arduino (Servo.h), C++

---

## Design Reference

Full approved spec: `docs/superpowers/specs/2026-04-03-arduino-direct-servo-design.md`

Supersedes: `docs/superpowers/plans/2026-04-02-arduino-pca9685-payload-servo.md` (PCA9685 approach)

---

## File Map

| File | Action | Purpose |
|------|--------|---------|
| `src/dbvf_autonomy/arduino/payload_servo_controller/payload_servo_controller.ino` | Rewrite | Replace PCA9685 with Servo.h direct PWM |
| `src/dbvf_autonomy/config/mission_params.yaml` | Modify line 43 comment | Update hardware description |

### Files explicitly NOT changed

| File | Reason |
|------|--------|
| `src/dbvf_autonomy/dbvf_autonomy/arduino_interface_node.py` | Serial protocol unchanged |
| `src/dbvf_autonomy/test/test_arduino_interface.py` | Tests cover pure protocol functions only |
| Launch files | No Arduino sketch awareness |
| `src/dbvf_msgs/srv/DoSetServo.srv` | Message definition unchanged |

---

### Task 1: Rewrite Arduino sketch to use direct Servo library

**Files:**
- Rewrite: `src/dbvf_autonomy/arduino/payload_servo_controller/payload_servo_controller.ino`

- [ ] **Step 1: Replace the sketch contents**

Replace the entire contents of `src/dbvf_autonomy/arduino/payload_servo_controller/payload_servo_controller.ino` with:

```cpp
// payload_servo_controller.ino
// Serial-to-Servo PWM bridge for payload servo mechanism.
// Protocol: S<channel>:<pwm_us>\n -> OK\n or ERR:<reason>\n
// Identical to arduino_interface_node protocol.
//
// Hardware: single servo on pin D9 (channel 0).
// Replaces previous PCA9685 I2C approach — same protocol, simpler wiring.

#include <Servo.h>

static const unsigned long BAUD_RATE = 115200;
static const int SERVO_PIN = 9;

Servo payloadServo;

String inputBuffer = "";

void setup() {
  Serial.begin(BAUD_RATE);
  payloadServo.attach(SERVO_PIN);
  delay(10);
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(inputBuffer);
      inputBuffer = "";
    } else {
      inputBuffer += c;
    }
  }
}

void handleCommand(const String& cmd) {
  // Expected format: S<channel>:<pwm_us>
  if (cmd.length() < 4 || cmd.charAt(0) != 'S') {
    Serial.println("ERR:bad format");
    return;
  }

  int colonIdx = cmd.indexOf(':');
  if (colonIdx < 0) {
    Serial.println("ERR:missing colon");
    return;
  }

  int channel = cmd.substring(1, colonIdx).toInt();
  int pwm_us = cmd.substring(colonIdx + 1).toInt();

  if (channel != 0) {
    Serial.println("ERR:channel out of range");
    return;
  }

  if (pwm_us < 0 || pwm_us > 20000) {
    Serial.println("ERR:pwm out of range");
    return;
  }

  payloadServo.writeMicroseconds(pwm_us);

  Serial.println("OK");
}
```

Key changes from PCA9685 version:
- Removed: `Wire.h`, `Adafruit_PWMServoDriver.h`, `pwm` object, `Wire.begin()`, `pwm.begin()`, `pwm.setPWMFreq(50)`, PCA9685 tick conversion, `MAX_CHANNEL = 15`
- Added: `Servo.h`, `payloadServo` object, `payloadServo.attach(9)`, `payloadServo.writeMicroseconds()`
- Changed: channel validation from `0-15` to `== 0` only (single servo on D9)
- Unchanged: serial protocol, baud rate, `handleCommand()` structure, PWM range validation, error message format

- [ ] **Step 2: Verify the sketch compiles**

Open the sketch in Arduino IDE (or use `arduino-cli`) and verify it compiles for Arduino Nano (ATmega328P):

```bash
arduino-cli compile --fqbn arduino:avr:nano src/dbvf_autonomy/arduino/payload_servo_controller/payload_servo_controller.ino
```

If `arduino-cli` is not installed, open in Arduino IDE: File > Open > navigate to `payload_servo_controller.ino`, select Board: Arduino Nano, Processor: ATmega328P, click Verify (checkmark button).

Expected: compilation succeeds with no errors.

- [ ] **Step 3: Run existing ROS2 tests to confirm protocol compatibility**

The existing 6 tests in `test_arduino_interface.py` validate the serial protocol pure functions (`format_servo_command`, `parse_servo_response`). Since the protocol is unchanged, these must still pass:

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
cd /home/finn/Documents/ardu_ws
python -m pytest src/dbvf_autonomy/test/test_arduino_interface.py -v
```

Expected: all 6 tests pass. These tests validate that the ROS2 node side of the protocol is unchanged — confirming end-to-end compatibility.

- [ ] **Step 4: Commit the sketch rewrite**

```bash
git add src/dbvf_autonomy/arduino/payload_servo_controller/payload_servo_controller.ino
git commit -m "refactor: replace PCA9685 with direct Servo library on pin D9

Remove Adafruit_PWMServoDriver I2C dependency. Single servo on D9
using Arduino built-in Servo library. Serial protocol unchanged —
arduino_interface_node and all tests remain compatible.

Channel validation tightened: accept channel 0 only (was 0-15)."
```

---

### Task 2: Update config comment

**Files:**
- Modify: `src/dbvf_autonomy/config/mission_params.yaml:43`

- [ ] **Step 1: Update the section comment**

In `src/dbvf_autonomy/config/mission_params.yaml`, change line 43 from:

```yaml
    # ── Payload Servo (Arduino + PCA9685) ─────────────────────────
```

to:

```yaml
    # ── Payload Servo (Arduino Nano direct) ───────────────────────
```

- [ ] **Step 2: Run full test suite**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
cd /home/finn/Documents/ardu_ws
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```

Expected: all tests pass (YAML comment change has no effect on behavior).

- [ ] **Step 3: Commit config update**

```bash
git add src/dbvf_autonomy/config/mission_params.yaml
git commit -m "docs: update mission_params.yaml comment — Arduino Nano direct servo"
```
