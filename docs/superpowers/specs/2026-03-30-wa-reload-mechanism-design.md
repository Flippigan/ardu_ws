# WA Reload Mechanism & Tag Hierarchy Update Design Spec

**Date:** 2026-03-30
**Status:** Draft
**Scope:** WA precision landing reload sequence (drop old payload, pick up new), three-tier AprilTag hierarchy, Arduino servo interface node

---

## Overview

Update the FM-3 WA (Water Autonomous) landing sequence to support a payload reload cycle: during precision descent, drop old payloads via an Arduino-controlled servo before landing magnetically on the new payload set. This also introduces a third (ultra-small) AprilTag for final precision, and a new `arduino_interface_node` for servo control.

### What This Design Covers

- New mission sequencer states for WA reload (drop old payload, reset servo, final land)
- Three-tier AprilTag hierarchy (GPS → 0.15m → 0.10m) replacing the current two-tier (0.6m → 0.15m)
- New `arduino_interface_node` (serial communication with Arduino servo controller)
- Tag detector adapter changes (third tag support)
- Precision landing FSM changes (new tag IDs, GPS-first approach)
- Config and test updates

### What This Design Does NOT Cover

- Arduino sketch / firmware (hardware team responsibility)
- Physical AprilTag model creation in Gazebo (separate team task)
- Clicking sensor integration (deferred — using PWM + settle time for now)
- Magnetic pickup mechanism hardware

### Dependencies on Other Teams

- **Simulation team:** Remove 0.6m tag (ID 0) from Gazebo world, add 0.10m tag (ID 2) centered on WA pad, keep 0.15m tag (ID 1)
- **Hardware team:** Arduino sketch accepting serial commands `S<num>:<pwm>\n`, USB serial device path

---

## Mechanical Sequence at WA

The reload at WA follows this physical sequence:

1. Drone approaches WA using GPS, begins precision descent on 0.15m tag (ID 1)
2. As altitude decreases, the 0.10m ultra-small tag (ID 2) becomes visible
3. Both tags confirmed detected for ≥2 seconds (drone is low, ~0.3–0.5m AGL)
4. **Drop old payload:** Arduino servo rotates to release position, old payload falls ~0.3m to ground
5. **Servo reset:** Arduino servo rotates to pickup position (magnets oriented for grab)
6. **Final precision descent:** Drone continues landing guided by ultra-small tag (ID 2)
7. **Magnetic pickup:** Drone lands, magnets attach to fresh payload set
8. Takeoff and continue to drop zone

---

## Updated AprilTag Hierarchy

### Before (Current)

| Tag | ID | Size | Role |
|-----|----|------|------|
| Primary | 0 | 0.6m | Coarse descent, visible from ~8m |
| Secondary | 1 | 0.15m | Final precision landing |

### After (New)

| Tag | ID | Size | Role |
|-----|----|------|------|
| ~~Primary~~ | ~~0~~ | ~~0.6m~~ | **Removed** |
| Primary | 1 | 0.15m | Coarse descent, visible from ~2–3m |
| Tertiary | 2 | 0.10m | Ultra-precise final landing |

### Descent Tier Logic

```
GPS approach (APPROACH state)
  │  within position_tolerance of WA GPS coords
  ▼
Slow guided descent (SEARCH state)
  │  ID 1 (0.15m) detected, confirmed 5 frames
  ▼
PID visual servo on ID 1 (DESCEND_COARSE)
  │  ID 2 (0.10m) confirmed detected for 2s alongside ID 1
  ▼
Both tags visible → triggers WA reload sequence
  │  (drop old payload, reset servo)
  ▼
PID visual servo on ID 2 (DESCEND_FINAL)
  │  landed
  ▼
Magnetic pickup complete
```

### Impact on Tag Detector Adapter

With the 0.6m tag removed, we still have exactly two physical tags (0.15m and 0.10m). The adapter's existing primary/secondary config is sufficient — we re-assign the IDs via config:

- `primary_tag_id: 1` (0.15m, was ID 0 / 0.6m)
- `secondary_tag_id: 2` (0.10m, was ID 1 / 0.15m)
- `primary_tag_size: 0.15`, `secondary_tag_size: 0.10`

No structural code changes needed. The `select_best_tag()` function and debounce filter work as-is with the new IDs.

### Impact on Precision Landing Node

The precision landing FSM's `secondary_tag_id` config (currently 1) becomes the ultra-small tag ID (2). The "small tag confirmed" trigger in `DESCEND_COARSE` (both tags detected for >2s) remains the same logic but now detects ID 2 as the secondary.

Config changes:
- `secondary_tag_id: 2` (was 1)
- No code changes needed in the precision landing FSM itself — it already uses config-driven tag IDs

**Key insight:** The precision landing FSM doesn't need to know about the reload. The mission sequencer intercepts the landing state and injects the drop/reset sequence between the "both tags confirmed" event and the final descent.

---

## Updated Mission State Machine

### New States

Three new states replace the current single `LAND_WA` state:

```
TRANSIT_TO_WA
  │  within position tolerance
  ▼
LAND_WA_DESCEND                    ← start precision landing
  │  landing_state == 'DESCEND_HOLD'  (both tags confirmed, drone is low)
  ▼
WA_DROP_OLD_PAYLOAD                ← servo release via Arduino, wait settle
  │  settle time elapsed
  ▼
WA_SERVO_RESET                     ← servo to pickup position via Arduino, wait settle
  │  settle time elapsed
  ▼
LAND_WA_FINAL                      ← precision landing continues to LANDED
  │  landing_state == 'LANDED'
  ▼
TAKEOFF_WA
```

### State Descriptions

**LAND_WA_DESCEND:** Replaces old `LAND_WA`. Calls `start_precision_landing` with WA GPS coordinates. Monitors `/dbvf/landing_state`. Transitions to `WA_DROP_OLD_PAYLOAD` when landing state reaches `DESCEND_HOLD` (meaning both tags confirmed for >2s and drone has paused descent — this is the low-altitude hold point). If precision landing reports `ABORT_LAND`, transition to ABORT.

**WA_DROP_OLD_PAYLOAD:** Commands Arduino servo to release position (`pickup_servo_pwm_release`). Waits `pickup_settle_time_s` (configurable, default 2.0s). The drone is hovering in the precision landing's DESCEND_HOLD state during this time (the precision landing FSM continues running, holding position via PID). Transitions to `WA_SERVO_RESET` when settle time elapses.

**WA_SERVO_RESET:** Commands Arduino servo to pickup position (`pickup_servo_pwm_pickup`). Waits `pickup_settle_time_s`. Transitions to `LAND_WA_FINAL` when settle time elapses.

**LAND_WA_FINAL:** The precision landing FSM is still running (it was in DESCEND_HOLD during the drop/reset). The mission sequencer now waits for the precision landing to progress through DESCEND_OFFSET → DESCEND_FINAL → LANDED. Monitors `/dbvf/landing_state` for `LANDED`. If `ABORT_LAND`, transition to ABORT.

### Coordination with Precision Landing FSM

The precision landing FSM runs independently — it doesn't know about the reload. The mission sequencer observes its state transitions via `/dbvf/landing_state`:

1. Precision landing enters `DESCEND_HOLD` → mission sequencer knows both tags confirmed, drone is holding low
2. Mission sequencer performs servo drop/reset (precision landing stays in HOLD, PID maintains position)
3. Precision landing naturally progresses DESCEND_HOLD → DESCEND_OFFSET → DESCEND_FINAL → LANDED
4. Mission sequencer observes `LANDED` → transitions to TAKEOFF_WA

**Important:** The precision landing's `hold_stabilize_time` (currently 1.0s) must be long enough for the entire drop + reset sequence. We increase it to accommodate: `hold_stabilize_time >= 2 * pickup_settle_time_s + margin`. Default: 6.0s (2s drop settle + 2s reset settle + 2s margin).

Alternatively, the mission sequencer can pause the precision landing's progression by sending velocity hold commands during the drop sequence. However, since the precision landing FSM already holds position in DESCEND_HOLD, simply increasing the hold time is the simpler approach and avoids adding cross-FSM control coupling.

### Updated State Enum (20 states + ABORT)

```python
class MissionState(Enum):
    IDLE = 'IDLE'
    PREFLIGHT_CHECK = 'PREFLIGHT_CHECK'
    TAKEOFF_H = 'TAKEOFF_H'
    TRANSIT_H_TO_L = 'TRANSIT_H_TO_L'
    LAND_L = 'LAND_L'
    WAIT_FLAGGER = 'WAIT_FLAGGER'
    TAKEOFF_L = 'TAKEOFF_L'
    TRANSIT_TO_DROP = 'TRANSIT_TO_DROP'
    DROP_PAYLOAD = 'DROP_PAYLOAD'
    TRANSIT_TO_WA = 'TRANSIT_TO_WA'
    LAND_WA_DESCEND = 'LAND_WA_DESCEND'       # was LAND_WA
    WA_DROP_OLD_PAYLOAD = 'WA_DROP_OLD_PAYLOAD' # new
    WA_SERVO_RESET = 'WA_SERVO_RESET'           # new
    LAND_WA_FINAL = 'LAND_WA_FINAL'             # new
    TAKEOFF_WA = 'TAKEOFF_WA'
    TRANSIT_TO_DROP_2 = 'TRANSIT_TO_DROP_2'
    DROP_PAYLOAD_2 = 'DROP_PAYLOAD_2'
    TRANSIT_TO_H = 'TRANSIT_TO_H'
    LAND_H = 'LAND_H'
    COMPLETE = 'COMPLETE'
    ABORT = 'ABORT'
```

### Updated Phase Map

All new WA states remain in phase `FM3`:

```python
MissionState.LAND_WA_DESCEND: 'FM3',
MissionState.WA_DROP_OLD_PAYLOAD: 'FM3',
MissionState.WA_SERVO_RESET: 'FM3',
MissionState.LAND_WA_FINAL: 'FM3',
```

---

## Arduino Interface Node

### Purpose

Single owner of USB serial connection to the Arduino servo controller board. Translates ROS2 service calls into serial commands.

### Serial Protocol

**Commands (ROS2 → Arduino):**
```
S<servo_number>:<pwm>\n
```
Example: `S1:1100\n` sets servo 1 to PWM 1100.

**Responses (Arduino → ROS2):**
```
OK\n        — command accepted
ERR:<msg>\n — command failed
```

The node waits up to `serial_timeout_s` (default 1.0s) for a response after each command.

### ROS2 Interface

**Service:** `/dbvf/arduino/set_servo`
- Uses the existing `dbvf_msgs/srv/DoSetServo` service definition (same fields: `servo_number`, `pwm` → `success`, `message`)
- No new message types needed

**Parameters:**
```yaml
arduino_interface:
  ros__parameters:
    serial_port: "/dev/ttyACM0"     # USB serial device
    baud_rate: 115200
    serial_timeout_s: 1.0
```

### Node Implementation

```python
class ArduinoInterfaceNode(Node):
    # - Opens serial port on startup
    # - Exposes /dbvf/arduino/set_servo service
    # - Sends S<num>:<pwm>\n, waits for OK/ERR response
    # - Logs warnings on timeout, returns success=False
    # - Reconnects on serial errors (with backoff)
```

### File Location

`src/dbvf_autonomy/dbvf_autonomy/arduino_interface_node.py`

---

## Configuration Changes

### mission_params.yaml — New Parameters

```yaml
mission_sequencer:
  ros__parameters:
    # ... existing params unchanged ...

    # ── WA Reload Servo (Arduino) ──────────────────────────────────
    pickup_servo_number: 1              # Arduino servo number
    pickup_servo_pwm_release: 1100      # PWM to drop old payload
    pickup_servo_pwm_pickup: 1500       # PWM for magnetic pickup orientation
    pickup_settle_time_s: 2.0           # Wait after each servo command
```

### sim_params.yaml — Tag Hierarchy Changes

```yaml
tag_detector_adapter:
  ros__parameters:
    primary_tag_id: 1                   # was 0 (0.6m tag removed)
    secondary_tag_id: 2                 # was 1 (new ultra-small tag)
    primary_tag_size: 0.15              # was 0.6
    secondary_tag_size: 0.10            # was 0.15
    # ... other params unchanged ...

precision_landing:
  ros__parameters:
    secondary_tag_id: 2                 # was 1
    hold_stabilize_time: 6.0            # was 1.0 (accommodate drop+reset)
    # ... other params unchanged ...

arduino_interface:
  ros__parameters:
    serial_port: "/dev/ttyACM0"
    baud_rate: 115200
    serial_timeout_s: 1.0
```

---

## Changes to Existing Code

### tag_detector_adapter_node.py

Minimal changes:
- Update default parameter values: `primary_tag_id=1`, `secondary_tag_id=2`, `primary_tag_size=0.15`, `secondary_tag_size=0.10`
- No structural code changes — the adapter already supports arbitrary tag IDs via config

### precision_landing_node.py

Config-only change:
- `secondary_tag_id: 2` (was 1)
- `hold_stabilize_time: 6.0` (was 1.0, to accommodate drop+reset during hold)
- No code changes to the FSM

### mission_state_machine.py

- Replace `LAND_WA` state with `LAND_WA_DESCEND`, `WA_DROP_OLD_PAYLOAD`, `WA_SERVO_RESET`, `LAND_WA_FINAL`
- Add `_wa_drop_start_time` tracking field
- Add three new state handlers
- Update phase map and handler dispatch table

### mission_sequencer_node.py

- Add service client for `/dbvf/arduino/set_servo`
- Add new entry actions: `arduino_servo_release`, `arduino_servo_pickup`
- Handle new action strings in `_execute_action()`
- Add `_call_arduino_servo()` helper

### mission_params.yaml

- Add `pickup_servo_number`, `pickup_servo_pwm_release`, `pickup_servo_pwm_pickup`, `pickup_settle_time_s`

### Launch files

- `mission_sim.launch.py`: add `arduino_interface_node` to launch
- `precision_landing_sim.launch.py`: no changes (doesn't include mission sequencer)

---

## Testing Strategy

### Unit Tests — New/Updated

| Test File | Changes | What It Covers |
|-----------|---------|----------------|
| `test_mission_state_machine.py` | Update existing + add ~8 new tests | LAND_WA_DESCEND → WA_DROP_OLD_PAYLOAD → WA_SERVO_RESET → LAND_WA_FINAL transitions, settle timing, abort from each new state |
| `test_mission_config.py` | Add ~3 tests | Validation of new pickup servo config params |

### Unit Tests — New File

| Test File | Tests | What It Covers |
|-----------|-------|----------------|
| `test_arduino_interface.py` | ~6 | Serial command formatting, response parsing, timeout handling (mock serial) |

### Integration Test Procedure (manual, in sim)

1. Launch Gazebo + SITL + mission stack (including `arduino_interface_node`)
2. Arm, start mission, resume after flagger
3. Observe FM-3:
   - TRANSIT_TO_WA → GPS approach
   - LAND_WA_DESCEND → precision landing starts, descends on ID 1
   - Mission state shows `WA_DROP_OLD_PAYLOAD` when both tags confirmed
   - Servo command sent to Arduino (or logged if no Arduino connected)
   - `WA_SERVO_RESET` → second servo command
   - `LAND_WA_FINAL` → precision landing completes to LANDED
   - TAKEOFF_WA → continues to drop

**Sim without Arduino:** The `arduino_interface_node` logs warnings if serial port unavailable but the service still returns (with `success=false`). The mission sequencer proceeds on settle time regardless — the FSM doesn't gate on servo confirmation. This allows full sim testing without hardware.

---

## File Summary

| File | Action | Package |
|------|--------|---------|
| `dbvf_autonomy/arduino_interface_node.py` | **New** | dbvf_autonomy |
| `dbvf_autonomy/mission_state_machine.py` | **Modified** (new WA states) | dbvf_autonomy |
| `dbvf_autonomy/mission_sequencer_node.py` | **Modified** (Arduino servo client + new actions) | dbvf_autonomy |
| `config/mission_params.yaml` | **Modified** (pickup servo params) | dbvf_autonomy |
| `config/sim_params.yaml` | **Modified** (tag IDs, hold time) | dbvf_autonomy |
| `launch/mission_sim.launch.py` | **Modified** (add Arduino node) | dbvf_autonomy |
| `test/test_mission_state_machine.py` | **Modified** (new state tests) | dbvf_autonomy |
| `test/test_mission_config.py` | **Modified** (new param validation) | dbvf_autonomy |
| `test/test_arduino_interface.py` | **New** | dbvf_autonomy |

No new message/service definitions needed — reuses existing `DoSetServo.srv`.
