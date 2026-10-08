# Mission Sequencer Design Spec

**Date:** 2026-03-29
**Status:** Approved
**Scope:** Autonomous mission sequencing for VFS DBVF competition (FM-1, FM-2, FM-3, Return Home)

---

## Overview

A new `mission_sequencer_node` that orchestrates the full autonomous competition sequence: takeoff from Home, land at waypoint L, wait for flagger approval, drop red payloads at F1/F2, precision-land at WA to pick up yellow payloads, drop yellow payloads at F1/F2, and return to Home.

The node is a linear state machine that delegates all flight control to existing services (`set_mode`, `arm_motors`, `send_guided_position`, `start_precision_landing`) and adds a new `DoSetServo` service for payload release.

### What This Design Covers

- Mission sequencer node (state machine, service interfaces, config)
- New `dbvf_msgs` service definitions
- One addition to `mavlink_interface_node` (`DoSetServo` service)
- New launch file and test strategy

### What This Design Does NOT Cover

- Simulation course layout / Gazebo world changes (separate task)
- GUI implementation (separate team member)
- Precision landing system changes (already implemented)
- Payload mechanism hardware design

---

## Competition Context

### Course Layout

Waypoints arranged linearly (approximate distances from RFP):

```
F2 --100ft-- F1 --100ft-- L --30ft-- WM --150ft-- WA --150ft-- H
(3x3ft)      (7x7ft)      (15x15ft)  (20x20ft)   (20x20ft)   (15x15ft)
```

- **H (Home):** Takeoff/landing start and end point
- **L (Land):** Flagger-controlled landing zone for FM-1
- **WA (Water Autonomous):** AprilTag precision landing pad, yellow payloads pre-staged (up to 5)
- **WM (Water Manual):** Blue payload pickup (not used in autonomous sequence)
- **F1 (Fire 1):** Larger drop zone, +2.5pts/payload
- **F2 (Fire 2):** Smaller drop zone, +5pts/payload

GPS coordinates are provided day-of-competition. The system reads coordinates from a YAML config file that the GUI team's interface writes to.

### Scoring Strategy

- FM-1 autonomous takeoff + land at L: +30 bonus points
- FM-2 autonomous payload drop: x2 multiplier per payload + 20pt bonus
- FM-3 autonomous pickup from WA + drop: x3 multiplier per payload + 50pt bonus
- Single FM-3 cycle only (WA yellow payloads), then return to H
- Pilot can manually fly additional WM cycles if time permits after autonomous sequence completes

### Autonomy Rules

- Pilot must not touch controls while aircraft is airborne in autonomous mode
- Manual re-arm while landed at L is permitted without losing autonomy points
- Manual takeoff input does not impact autonomy points
- Aircraft must be in autonomous flight mode from moment it leaves ground to complete each mission phase

---

## State Machine

### States (17 total + ABORT)

```
IDLE
  │  StartMission service called
  ▼
PREFLIGHT_CHECK ── fail ──► ABORT
  │  armed, GPS lock, GUIDED mode
  ▼
┌─────────────────────────────────────┐
│ FM-1: Takeoff & Land at L           │
│                                     │
│ TAKEOFF_H                           │
│   │  rangefinder ≥ 35ft AGL         │
│   ▼                                 │
│ TRANSIT_H_TO_L                      │
│   │  within position tolerance of L │
│   ▼                                 │
│ LAND_L                              │
│   │  vehicle landed/disarmed        │
│   ▼                                 │
└─────────────────────────────────────┘
WAIT_FLAGGER ◄── blocks until ResumeMission service called
  │
  ▼
┌─────────────────────────────────────┐
│ FM-2: Drop Red Payloads             │
│                                     │
│ TAKEOFF_L                           │
│   │  rangefinder ≥ 35ft AGL         │
│   ▼                                 │
│ TRANSIT_TO_DROP                     │
│   │  over F1/F2 (position check)    │
│   ▼                                 │
│ DROP_PAYLOAD                        │
│   │  servo actuated + settle time   │
│   ▼                                 │
└─────────────────────────────────────┘
┌─────────────────────────────────────┐
│ FM-3: Pickup from WA & Re-drop      │
│                                     │
│ TRANSIT_TO_WA                       │
│   │  within position tolerance      │
│   ▼                                 │
│ LAND_WA                             │
│   │  precision landing → LANDED     │
│   ▼                                 │
│ TAKEOFF_WA                          │
│   │  rangefinder ≥ 35ft AGL         │
│   ▼                                 │
│ TRANSIT_TO_DROP_2                   │
│   │  over F1/F2 (position check)    │
│   ▼                                 │
│ DROP_PAYLOAD_2                      │
│   │  servo actuated + settle time   │
│   ▼                                 │
└─────────────────────────────────────┘
┌─────────────────────────────────────┐
│ Return Home                         │
│                                     │
│ TRANSIT_TO_H                        │
│   │  within position tolerance of H │
│   ▼                                 │
│ LAND_H                              │
│   │  vehicle landed                 │
│   ▼                                 │
│ COMPLETE                            │
└─────────────────────────────────────┘

ABORT ◄── reachable from ANY state (timeout, failure, heartbeat loss, manual abort)
  │  switch to LAND mode immediately
  ▼
  (terminal)
```

### State Descriptions

**IDLE:** Node is running, waiting for `StartMission` service call. No flight activity.

**PREFLIGHT_CHECK:** Verify vehicle is armed, has GPS lock, and is in GUIDED mode. If any check fails, transition to ABORT. This state does not arm the vehicle — arming is done manually or by the GUI before calling StartMission.

**TAKEOFF_H:** Send `send_guided_position` to H's GPS at 35ft AGL. Monitor rangefinder altitude. Transition when `range_alt >= 35ft (10.668m)`. If rangefinder unavailable (`range_alt == -1.0`), fall back to `alt_rel`.

**TRANSIT_H_TO_L:** Send `send_guided_position` to L's GPS at 35ft AGL. Re-send position command every 1.0s to maintain GUIDED target. Transition when haversine distance to L < `position_tolerance_m` (default 3.0m).

**LAND_L:** Call `set_mode("LAND")`. Monitor `vehicle_state` for landed condition: `armed == false` OR (`alt_rel < 0.3m` AND `abs(vz) < 0.1 m/s`). Transition to WAIT_FLAGGER.

**WAIT_FLAGGER:** Idle state. No timeout. Waiting for `ResumeMission` service call (triggered by human button press when flagger raises flag). The aircraft remains disarmed on the ground. Manual re-arm at L is permitted per competition rules.

**TAKEOFF_L:** Call `arm_motors(true)`, then `set_mode("GUIDED")`, then `send_guided_position` to L's GPS at 35ft AGL. Transition when rangefinder confirms altitude.

**TRANSIT_TO_DROP:** Send `send_guided_position` to F1 or F2 GPS (based on `drop_target` config) at 35ft AGL. Transition when over target.

**DROP_PAYLOAD:** Call `DoSetServo` with release PWM. Wait `drop_settle_time_s` (2.0s) for payloads to clear. Transition to TRANSIT_TO_WA.

**TRANSIT_TO_WA:** Send `send_guided_position` to WA's GPS at 35ft AGL. Transition when within position tolerance.

**LAND_WA:** Call `start_precision_landing` with WA's GPS coordinates. Subscribe to `/dbvf/landing_state` and wait for `"LANDED"`. If precision landing reports `"ABORT_LAND"` or the landing timeout (60s) expires, transition to ABORT. This delegates entirely to the existing precision landing FSM.

**TAKEOFF_WA:** Call `arm_motors(true)`, `set_mode("GUIDED")`, send guided position at 35ft AGL. Transition when altitude confirmed. Payloads are attached via magnets on landing — no grab command needed.

**TRANSIT_TO_DROP_2:** Same as TRANSIT_TO_DROP. Uses same drop target config.

**DROP_PAYLOAD_2:** Same as DROP_PAYLOAD. Call `DoSetServo` with release PWM.

**TRANSIT_TO_H:** Send `send_guided_position` to H's GPS at 35ft AGL. Transition when within position tolerance.

**LAND_H:** Call `set_mode("LAND")`. Monitor for landed condition. Transition to COMPLETE.

**COMPLETE:** Terminal state. Publish mission complete status. No further action.

**ABORT:** Reachable from any state. Call `set_mode("LAND")` immediately. Publish abort reason. Terminal state. Triggered by:
- Mission timeout (default 540s / 9 minutes)
- MAVLink heartbeat loss (heartbeat_status == false for > 5s)
- Precision landing failure at WA
- Manual `AbortMission` service call
- Any service call failure after retry

### Transition Logic

All transitions are checked in a 10Hz timer callback that:
1. Reads latest `/dbvf/vehicle_state`
2. Checks transition condition for current state
3. If met, executes entry action for next state and updates state

Position tolerance checks use haversine distance between vehicle GPS and target GPS. The default tolerance of 3.0m triggers the state transition when the aircraft is near the target — it does not guarantee the aircraft is centered over the zone. Actual drop accuracy depends on GPS precision (~2-3m CEP on Cube Orange). For F2 (0.91m square), GPS accuracy is the limiting factor; a tighter software tolerance would not improve accuracy and could cause the state machine to stall. For larger zones (L at 4.57m, WA at 6.1m), the 3.0m tolerance is comfortably within bounds.

---

## New Service Definitions

### StartMission.srv (in dbvf_msgs)

```
# Request
string config_path    # Path to mission_params.yaml (empty = use default)
---
# Response
bool success
string message
```

### ResumeMission.srv (in dbvf_msgs)

```
# Request (empty — just a trigger)
---
# Response
bool success
string message        # e.g., "Resuming from WAIT_FLAGGER"
```

### AbortMission.srv (in dbvf_msgs)

```
# Request
string reason         # e.g., "Manual abort by operator"
---
# Response
bool success
string message
```

### DoSetServo.srv (in dbvf_msgs)

```
# Request
int32 servo_number    # ArduPilot servo output number (e.g., 9)
int32 pwm             # PWM value (e.g., 1100 for release, 1500 for hold)
---
# Response
bool success
string message
```

---

## New Topics

| Topic | Type | Rate | Publisher | Purpose |
|-------|------|------|----------|---------|
| `/dbvf/mission_state` | `std_msgs/String` | 5 Hz | mission_sequencer | Current state name |
| `/dbvf/mission_phase` | `std_msgs/String` | On change | mission_sequencer | "FM1", "FM2", "FM3", "RTH", "COMPLETE", "ABORT" |

---

## Changes to Existing Code

### mavlink_interface_node.py — Add DoSetServo Service

One new service handler following the exact pattern of `SetMode` and `ArmMotors`:

```python
# New service server (in __init__)
self.do_set_servo_srv = self.create_service(
    DoSetServo, '/dbvf/do_set_servo', self._do_set_servo_cb)

# Handler
def _do_set_servo_cb(self, request, response):
    with self._lock:
        self._mav.command_long_send(
            self._target_system, self._target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,                          # confirmation
            request.servo_number,       # param1: servo number
            request.pwm,                # param2: PWM value
            0, 0, 0, 0, 0)             # params 3-7 unused
    response.success = True
    response.message = f"Servo {request.servo_number} set to {request.pwm}"
    return response
```

No other existing files are modified.

---

## Configuration

### mission_params.yaml

```yaml
mission_sequencer:
  ros__parameters:
    # Waypoint GPS coordinates
    # These are simulation defaults. GUI team replaces with competition coords.
    home_lat: -35.3632621
    home_lon: 149.1652374
    landing_lat: -35.3640000
    landing_lon: 149.1652374
    wa_lat: -35.3632531
    wa_lon: 149.1657896
    f1_lat: -35.3650000
    f1_lon: 149.1652374
    f2_lat: -35.3660000
    f2_lon: 149.1652374

    # Flight parameters
    transit_altitude_ft: 35.0
    position_tolerance_m: 3.0
    takeoff_complete_alt_ft: 33.0

    # Payload servo configuration
    drop_servo_number: 9
    drop_servo_pwm_release: 1100
    drop_servo_pwm_hold: 1500
    drop_settle_time_s: 2.0

    # Drop zone target ("F1" or "F2")
    drop_target: "F1"

    # Safety
    mission_timeout_s: 540
    heartbeat_loss_timeout_s: 5.0
    service_call_timeout_s: 5.0
    guided_resend_interval_s: 1.0

    # Altitude source
    prefer_rangefinder: true
```

GPS coordinates use flat ROS2 parameter names (not nested dicts) for compatibility with `rclpy` parameter declaration and the GUI team's tooling.

---

## Launch File

### mission_sim.launch.py

Includes all nodes from `precision_landing_sim.launch.py` plus the mission sequencer:

1. `apriltag_ros/apriltag_node` — AprilTag detection
2. `dbvf_autonomy/tag_detector_adapter_node` — Tag bridge
3. `dbvf_autonomy/mavlink_interface_node` — MAVLink bridge (now with DoSetServo)
4. `dbvf_autonomy/precision_landing_node` — Precision landing FSM
5. `dbvf_autonomy/tag_visualizer_node` — Debug visualization
6. **`dbvf_autonomy/mission_sequencer_node`** — New mission orchestrator

All nodes load from `sim_params.yaml` (existing) and `mission_params.yaml` (new).

---

## Testing Strategy

### Unit Tests (pure-function, no ROS2 runtime)

| Test File | Tests | What It Covers |
|-----------|-------|----------------|
| `test_mission_state_machine.py` | ~20 | All state transitions, entry/exit actions, abort from each state |
| `test_mission_helpers.py` | ~8 | Altitude conversion (ft↔m), haversine distance, position tolerance checks |
| `test_mission_config.py` | ~5 | Config loading, validation, default values |

The state machine logic is extracted into a pure Python class (same pattern as `precision_landing_node.py`) that can be tested without ROS2 infrastructure.

### Integration Test Procedure (manual, in sim)

1. Launch: `iris_runway.launch.py` + `mission_sim.launch.py`
2. In MAVProxy: `mode guided`, `arm throttle`
3. Call: `ros2 service call /dbvf/start_mission dbvf_msgs/srv/StartMission "{}"`
4. Monitor: `ros2 topic echo /dbvf/mission_state`
5. Observe FM-1 (takeoff H → transit L → land L)
6. Call: `ros2 service call /dbvf/resume_mission dbvf_msgs/srv/ResumeMission "{}"`
7. Observe FM-2 (takeoff L → transit F1/F2 → drop)
8. Observe FM-3 (transit WA → precision land → takeoff → transit F1/F2 → drop)
9. Observe RTH (transit H → land H → COMPLETE)

### What Is Not Tested Here

- Precision landing accuracy (already has 38 tests)
- Gazebo world layout (separate task)
- GUI (separate team member)
- Physical servo actuation (hardware-only)

---

## Interface Contract for GUI Team

The GUI team needs to:

1. **Write GPS coordinates** to `mission_params.yaml` (or call ROS2 parameter updates)
2. **Set `drop_target`** to `"F1"` or `"F2"` in the config
3. **Call `StartMission`** service to begin the autonomous sequence
4. **Call `ResumeMission`** service when flagger approves (button press)
5. **Optionally call `AbortMission`** for manual abort
6. **Monitor `/dbvf/mission_state`** and `/dbvf/mission_phase` for status display

All services use the standard `bool success, string message` response pattern.

---

## File Summary

| File | Action | Package |
|------|--------|---------|
| `dbvf_autonomy/mission_sequencer_node.py` | **New** | dbvf_autonomy |
| `config/mission_params.yaml` | **New** | dbvf_autonomy |
| `launch/mission_sim.launch.py` | **New** | dbvf_autonomy |
| `test/test_mission_state_machine.py` | **New** | dbvf_autonomy |
| `test/test_mission_helpers.py` | **New** | dbvf_autonomy |
| `test/test_mission_config.py` | **New** | dbvf_autonomy |
| `dbvf_autonomy/mavlink_interface_node.py` | **Modified** (add DoSetServo) | dbvf_autonomy |
| `srv/StartMission.srv` | **New** | dbvf_msgs |
| `srv/ResumeMission.srv` | **New** | dbvf_msgs |
| `srv/AbortMission.srv` | **New** | dbvf_msgs |
| `srv/DoSetServo.srv` | **New** | dbvf_msgs |
