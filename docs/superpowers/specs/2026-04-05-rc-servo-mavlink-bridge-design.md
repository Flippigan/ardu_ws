# RC Servo MAVLink Bridge Design

**Date:** 2026-04-05
**Status:** Approved
**Problem:** Cube Orange AUX/MAIN outputs are all DShot for ESCs — cannot pass through RC channel PWM to the Arduino for manual servo control.
**Solution:** Route RC switch state through the existing MAVLink serial link: Cube → `RC_CHANNELS` → Jetson `mavlink_interface_node` → Arduino via `arduino_interface_node`.

---

## Background

The payload servo controller Arduino currently reads RC receiver channels (CH11/CH12) directly via `pulseIn()` on GPIO pins to give the pilot manual CW/CCW servo control. This required the Cube Orange to mirror those RC channels as PWM outputs on its AUX/MAIN rail, but all outputs are configured for DShot (ESC protocol), making PWM pass-through impossible.

The Cube already sends `RC_CHANNELS` MAVLink messages over its serial telemetry link to the Jetson. The `mavlink_interface_node` already parses these messages for ch14/ch15 mission triggers. This design extends that same path to carry CH11/CH12 servo commands to the Arduino.

---

## Architecture

### Data Flow

```
Pilot RC switch (CH11/CH12)
  → RC Receiver
  → Cube Orange
  → MAVLink RC_CHANNELS (serial @ ~2-4Hz)
  → Jetson mavlink_interface_node
  → /dbvf/arduino/set_servo service call (async)
  → arduino_interface_node
  → USB serial S0:<pwm>\n
  → Arduino payload_servo_controller
  → Servo CW / CCW / STOP
```

### Behavior

**Level-triggered:** Servo spins while the switch is held high, stops when released.

- CH11 high → CW (1700 µs)
- CH12 high → CCW (1300 µs)
- Both low → STOP (1500 µs)
- Both high → CW wins (CH11 priority, matches original Arduino sketch)

**State-change only:** The node tracks the last command sent (`"cw"`, `"ccw"`, or `"stop"`). A service call is only made when the desired state differs from the last sent state. This avoids flooding the Arduino with redundant commands on every `RC_CHANNELS` message.

### Latency Budget

| Segment | Latency |
|---------|---------|
| RC receiver | ~20 ms |
| Cube MAVLink RC_CHANNELS rate | ~250–500 ms |
| Jetson service call | ~5 ms |
| USB serial to Arduino | ~5 ms |
| **Total** | **~300–550 ms** |

Acceptable for manual payload drop operations.

---

## Changes

### 1. `mavlink_interface_node.py`

**New parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rc_servo_cw_channel` | 11 | RC channel for CW spin |
| `rc_servo_ccw_channel` | 12 | RC channel for CCW spin |
| `rc_servo_threshold` | 1700 | PWM above this = switch high |
| `rc_servo_channel` | 0 | Arduino servo channel number |
| `rc_servo_cw_pwm` | 1700 | PWM sent for CW command |
| `rc_servo_ccw_pwm` | 1300 | PWM sent for CCW command |
| `rc_servo_stop_pwm` | 1500 | PWM sent for stop command |

**New pure function:**

```python
def determine_rc_servo_command(ch_cw_pwm, ch_ccw_pwm, threshold):
    """Determine servo command from RC channel PWM values.

    Returns "cw", "ccw", or "stop".
    CH_CW wins if both channels are high.
    """
```

**New state:** `self._rc_servo_state = "stop"` — tracks last command sent.

**New service client:** `/dbvf/arduino/set_servo` (DoSetServo).

**Modified `_handle_rc_channels()`:** After existing ch14/ch15 edge-trigger logic, add:

1. Read CH11/CH12 via `get_rc_channel_pwm()`
2. Call `determine_rc_servo_command()` to get desired state
3. If state differs from `_rc_servo_state`, call `/dbvf/arduino/set_servo` async with the configured PWM value
4. Update `_rc_servo_state`

### 2. `payload_servo_controller.ino`

**Remove:**
- `RC_CH11_PIN`, `RC_CH12_PIN` pin constants and `pinMode()` calls
- `RC_THRESHOLD`, `RC_TIMEOUT_US` constants
- `pulseIn()` calls in `loop()`
- RC priority block (CH11/CH12 override logic)

**Keep:**
- Serial command handling (`handleCommand()`)
- Limit switch logic (hardware safety: stop on contact, latch until new command)
- `FORCE`/`NOFORCE` override
- `CW_PWM`, `CCW_PWM`, `STOP_PWM` constants (documentation reference)

**Simplified priority:** Limit switch latch > serial commands.

### 3. Config files

Add to `hardware_params.yaml` under `mavlink_interface.ros__parameters`:

```yaml
rc_servo_cw_channel: 11
rc_servo_ccw_channel: 12
rc_servo_threshold: 1700
rc_servo_channel: 0
rc_servo_cw_pwm: 1700
rc_servo_ccw_pwm: 1300
rc_servo_stop_pwm: 1500
```

---

## Failure Modes

| Failure | Behavior |
|---------|----------|
| Arduino USB disconnected | `arduino_interface_node` returns `success=False`, warning logged. Servo doesn't move. |
| Jetson down | No servo control (manual or autonomous). Same as existing autonomous path. |
| MAVLink serial lost | No `RC_CHANNELS` received. Servo stays in last state. Limit switch remains as hardware safety. |
| Both CH11 and CH12 high | CH11 (CW) wins. |
| Pilot overrides during mission | Last command wins — pilot's RC command overrides mission sequencer's servo state. Desirable behavior. |

---

## Testing

New pure function `determine_rc_servo_command()` tested in `test_rc_servo_bridge.py`:

- CH11 high, CH12 low → `"cw"`
- CH11 low, CH12 high → `"ccw"`
- Both low → `"stop"`
- Both high → `"cw"` (CH11 priority)
- Exactly at threshold → `"stop"` (threshold is exclusive)
- Zero values (no signal) → `"stop"`

No integration tests needed — wiring pattern is identical to existing ch14/ch15 RC triggers.

---

## Non-Goals

- Changing the `arduino_interface_node` or its serial protocol (unchanged)
- Adding a new ROS2 node or topic
- Supporting edge-triggered servo control from RC
- Modifying mission sequencer servo logic (it continues to call `/dbvf/arduino/set_servo` directly)
