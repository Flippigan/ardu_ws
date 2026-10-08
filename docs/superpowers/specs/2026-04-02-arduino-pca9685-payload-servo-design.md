# Arduino PCA9685 Payload Servo Controller Design

**Date:** 2026-04-02
**Status:** Approved

## Overview

Replace the Cube Orange servo output for payload operations with an Arduino + PCA9685 16-channel PWM driver controlling a single rotary servo mechanism. The Arduino acts as a dumb serial-to-PWM bridge — all sequencing logic stays in the ROS2 mission sequencer.

## Physical Mechanism

A rotary mechanism (carousel/gate) with 5 absolute positions:

| # | Position | Purpose |
|---|----------|---------|
| 1 | Hold | Pre-loaded payload, drone takes off in this position |
| 2 | Dispense | Rotates to endpoint (CW) to release all payloads at once |
| 3 | Drop | Rotates (CCW) to drop/release the cradle mechanism itself |
| 4 | Pickup | Continues (CCW) to endpoint presenting cradle open for crew loading |
| 5 | Lock | Rotates to secure newly loaded payload in place |

## Hardware

- **PCA9685** 16-channel 12-bit PWM driver, I2C address `0x40`
- **Arduino** (Nano or Uno — code is identical), connected to Jetson Orin Nano via USB serial
- **Single servo** on PCA9685 channel 0 (configurable)
- I2C wiring: Arduino A4 (SDA) → PCA9685 SDA, Arduino A5 (SCL) → PCA9685 SCL
- PCA9685 frequency: 50Hz (standard servo)

## Arduino Sketch

**Location:** `src/dbvf_autonomy/arduino/payload_servo_controller/payload_servo_controller.ino`

**Dependencies:** `Wire.h`, `Adafruit_PWMServoDriver` (Adafruit PWM Servo Driver Library)

**Behavior:**
1. Initialize PCA9685 at `0x40`, set frequency to 50Hz
2. Open serial at 115200 baud
3. Listen for `S<channel>:<pwm_us>\n` commands
4. Convert PWM microseconds to PCA9685 tick count: `ticks = pwm_us * 4096 / 20000`
5. Call `setPWM(channel, 0, ticks)` on PCA9685
6. Reply `OK\n` on success, `ERR:<reason>\n` on parse failure

**Protocol:** Unchanged from existing `arduino_interface_node` — `S<num>:<pwm>\n` → `OK\n` / `ERR:<msg>\n`

## ROS-Side Changes

### Config: `mission_params.yaml`

**Remove:**
```yaml
drop_servo_number: 9
drop_servo_pwm_release: 1100
drop_servo_pwm_hold: 1500
pickup_servo_number: 1
pickup_servo_pwm_release: 1100
pickup_servo_pwm_pickup: 1500
pickup_settle_time_s: 2.0
```

**Add:**
```yaml
# ── Payload Servo (Arduino + PCA9685) ────────────────────────
payload_servo_channel: 0            # PCA9685 channel (0-15)
payload_servo_pwm_hold: 0           # Position 1: pre-loaded hold (TBD during testing)
payload_servo_pwm_dispense: 0       # Position 2: release all payloads (TBD)
payload_servo_pwm_drop: 0           # Position 3: drop cradle mechanism (TBD)
payload_servo_pwm_pickup: 0         # Position 4: open for crew loading (TBD)
payload_servo_pwm_lock: 0           # Position 5: lock new payload (TBD)
payload_settle_time_s: 2.0          # Wait time after each servo move
```

All PWM values are placeholders until tuned on physical hardware.

### State Machine: `mission_state_machine.py`

**21-state FSM** (was 20). Changes:

- **Rename** `WA_SERVO_RESET` → `WA_PICKUP_READY` (semantic clarity)
- **Add** `WA_LOCK_PAYLOAD` state between `LAND_WA_FINAL` and `TAKEOFF_WA`
- **Change** `DROP_PAYLOAD` entry action: `servo_release` → `arduino_servo_dispense`
- **Change** `DROP_PAYLOAD_2` entry action: `servo_release` → `arduino_servo_dispense`
- **Change** `WA_DROP_OLD_PAYLOAD` entry action: `arduino_servo_release` → `arduino_servo_drop`
- **Change** `WA_PICKUP_READY` (was `WA_SERVO_RESET`) entry action: `arduino_servo_pickup` (unchanged)
- **New** `WA_LOCK_PAYLOAD` entry action: `arduino_servo_lock`, waits `payload_settle_time_s`, transitions to `TAKEOFF_WA`

Updated WA reload sequence:
```
LAND_WA_DESCEND → WA_DROP_OLD_PAYLOAD → WA_PICKUP_READY → LAND_WA_FINAL → WA_LOCK_PAYLOAD → TAKEOFF_WA
```

Phase mapping: `WA_LOCK_PAYLOAD` → FM3 (same as surrounding states).

### Mission Sequencer Node: `mission_sequencer_node.py`

**New action handlers in `_execute_action()`:**
- `arduino_servo_dispense` → `_call_arduino_servo(cfg['payload_servo_channel'], cfg['payload_servo_pwm_dispense'])`
- `arduino_servo_drop` → `_call_arduino_servo(cfg['payload_servo_channel'], cfg['payload_servo_pwm_drop'])`
- `arduino_servo_lock` → `_call_arduino_servo(cfg['payload_servo_channel'], cfg['payload_servo_pwm_lock'])`
- `arduino_servo_pickup` → `_call_arduino_servo(cfg['payload_servo_channel'], cfg['payload_servo_pwm_pickup'])`

**Remove dead code:**
- `servo_cli` service client (Cube Orange `do_set_servo`)
- `_call_set_servo()` method
- `servo_release` action handler
- `drop_servo_*` parameter declarations

**Update parameter declarations** to use new `payload_servo_*` names.

### Mission Helpers: `mission_helpers.py`

- Update `DEFAULT_MISSION_CONFIG` with new `payload_servo_*` keys
- Update `validate_mission_config()` to validate new param names
- Remove validation for old `drop_servo_*` and `pickup_servo_*` keys

### Unchanged Components

- `arduino_interface_node.py` — no changes, protocol identical
- `DoSetServo.srv` — no changes
- `hardware_params.yaml` — `arduino_interface` section unchanged
- Launch files — `arduino_interface_node` already included
- `precision_landing_node.py` — unaffected
- `tag_detector_adapter_node.py` — unaffected
- `mavlink_interface_node.py` — unaffected

## Mission Sequence with New Servo Positions

```
IDLE
  → PREFLIGHT_CHECK
  → TAKEOFF_H              (servo at HOLD position — pre-loaded)
  → TRANSIT_H_TO_L
  → LAND_L
  → WAIT_FLAGGER
  → TAKEOFF_L
  → TRANSIT_TO_DROP
  → DROP_PAYLOAD            entry: arduino_servo_dispense (position 2)
  → TRANSIT_TO_WA
  → LAND_WA_DESCEND         entry: start_precision_landing
  → WA_DROP_OLD_PAYLOAD     entry: arduino_servo_drop (position 3)
  → WA_PICKUP_READY         entry: arduino_servo_pickup (position 4)
  → LAND_WA_FINAL           (crew loads new payload during descent/landing)
  → WA_LOCK_PAYLOAD         entry: arduino_servo_lock (position 5)
  → TAKEOFF_WA
  → TRANSIT_TO_DROP_2
  → DROP_PAYLOAD_2           entry: arduino_servo_dispense (position 2)
  → TRANSIT_TO_H
  → LAND_H
  → COMPLETE
```

## Testing

| File | Changes |
|------|---------|
| `test_mission_state_machine.py` | Add `WA_LOCK_PAYLOAD` transition tests. Update `WA_SERVO_RESET` → `WA_PICKUP_READY` references. Test `arduino_servo_dispense` in `DROP_PAYLOAD`/`DROP_PAYLOAD_2`. Test full 21-state sequence. |
| `test_mission_config.py` | Update for new `payload_servo_*` param names. Remove old `drop_servo_*`/`pickup_servo_*` validation tests. |
| `test_arduino_interface.py` | No changes — protocol unchanged |
| Existing tests | Update any references to removed params or renamed state |
