# Mission Sequencer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build an autonomous mission sequencer node that orchestrates the full DBVF competition sequence — FM-1 (takeoff + land at L), FM-2 (red payload drops at F1/F2), FM-3 (precision land at WA, pickup yellow payloads, re-drop), and return to Home.

**Architecture:** A 17-state + ABORT linear state machine extracted into a pure Python class (`MissionStateMachine`) for testability, wrapped by a ROS2 node (`MissionSequencerNode`). The node delegates flight control to existing services (`set_mode`, `arm_motors`, `send_guided_position`, `start_precision_landing`) and adds a new `DoSetServo` service for payload release. Configuration is read from a YAML parameter file with GPS coordinates set by the GUI team.

**Tech Stack:** ROS2 Humble, Python 3, ament_cmake + ament_cmake_python, pytest (pure-function tests, no ROS2 runtime), pymavlink (MAVLink interface only), dbvf_msgs (service/message definitions).

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `src/dbvf_msgs/srv/StartMission.srv` | **Create** | Service definition: start autonomous mission |
| `src/dbvf_msgs/srv/ResumeMission.srv` | **Create** | Service definition: resume from WAIT_FLAGGER |
| `src/dbvf_msgs/srv/AbortMission.srv` | **Create** | Service definition: manual abort |
| `src/dbvf_msgs/srv/DoSetServo.srv` | **Create** | Service definition: actuate servo via MAVLink |
| `src/dbvf_msgs/CMakeLists.txt` | **Modify** | Register new .srv files with rosidl |
| `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py` | **Modify** | Add DoSetServo service handler |
| `src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py` | **Create** | Pure functions: ft/m conversion, haversine, position tolerance, config validation |
| `src/dbvf_autonomy/test/test_mission_helpers.py` | **Create** | Tests for helper functions |
| `src/dbvf_autonomy/test/test_mission_config.py` | **Create** | Tests for config validation |
| `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py` | **Create** | Pure Python state machine class (17 states + ABORT) |
| `src/dbvf_autonomy/test/test_mission_state_machine.py` | **Create** | Tests for all state transitions |
| `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py` | **Create** | ROS2 node wrapping the state machine |
| `src/dbvf_autonomy/scripts/mission_sequencer_node` | **Create** | Executable entry point |
| `src/dbvf_autonomy/config/mission_params.yaml` | **Create** | GPS waypoints, flight params, servo config |
| `src/dbvf_autonomy/launch/mission_sim.launch.py` | **Create** | Launch file including all nodes |
| `src/dbvf_autonomy/CMakeLists.txt` | **Modify** | Register new tests, scripts, config |

---

## Task 1: New Service Definitions in dbvf_msgs

**Files:**
- Create: `src/dbvf_msgs/srv/StartMission.srv`
- Create: `src/dbvf_msgs/srv/ResumeMission.srv`
- Create: `src/dbvf_msgs/srv/AbortMission.srv`
- Create: `src/dbvf_msgs/srv/DoSetServo.srv`
- Modify: `src/dbvf_msgs/CMakeLists.txt`

- [ ] **Step 1: Create StartMission.srv**

Create `src/dbvf_msgs/srv/StartMission.srv`:

```
string config_path
---
bool success
string message
```

- [ ] **Step 2: Create ResumeMission.srv**

Create `src/dbvf_msgs/srv/ResumeMission.srv`:

```
---
bool success
string message
```

- [ ] **Step 3: Create AbortMission.srv**

Create `src/dbvf_msgs/srv/AbortMission.srv`:

```
string reason
---
bool success
string message
```

- [ ] **Step 4: Create DoSetServo.srv**

Create `src/dbvf_msgs/srv/DoSetServo.srv`:

```
int32 servo_number
int32 pwm
---
bool success
string message
```

- [ ] **Step 5: Register new services in CMakeLists.txt**

In `src/dbvf_msgs/CMakeLists.txt`, add the four new `.srv` files to the `rosidl_generate_interfaces` call. The full call becomes:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/LandingTargetPose.msg"
  "msg/TagStatus.msg"
  "msg/VehicleState.msg"
  "srv/SetMode.srv"
  "srv/ArmMotors.srv"
  "srv/SendGuidedPosition.srv"
  "srv/SendGuidedVelocity.srv"
  "srv/StartPrecisionLanding.srv"
  "srv/StartMission.srv"
  "srv/ResumeMission.srv"
  "srv/AbortMission.srv"
  "srv/DoSetServo.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
```

- [ ] **Step 6: Build dbvf_msgs and verify**

Run:
```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select dbvf_msgs
```
Expected: Build succeeds with no errors.

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 interface show dbvf_msgs/srv/DoSetServo
```
Expected: Shows the service definition fields.

- [ ] **Step 7: Commit**

```bash
git add src/dbvf_msgs/srv/StartMission.srv src/dbvf_msgs/srv/ResumeMission.srv \
        src/dbvf_msgs/srv/AbortMission.srv src/dbvf_msgs/srv/DoSetServo.srv \
        src/dbvf_msgs/CMakeLists.txt
git commit -m "feat: add mission sequencer service definitions (StartMission, ResumeMission, AbortMission, DoSetServo)"
```

---

## Task 2: Add DoSetServo Service to mavlink_interface_node

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`

- [ ] **Step 1: Add DoSetServo import and service server**

In `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`:

Change the import line from:

```python
from dbvf_msgs.srv import SetMode, ArmMotors, SendGuidedPosition, SendGuidedVelocity
```

to:

```python
from dbvf_msgs.srv import SetMode, ArmMotors, SendGuidedPosition, SendGuidedVelocity, DoSetServo
```

In `MavlinkInterfaceNode.__init__`, after the `SendGuidedVelocity` service creation (line 99), add:

```python
        self.create_service(
            DoSetServo, '/dbvf/do_set_servo', self._do_set_servo_cb)
```

- [ ] **Step 2: Add the service callback**

In `MavlinkInterfaceNode`, add the handler method after `_guided_velocity_cb` (before the `main()` function at line 307):

```python
    def _do_set_servo_cb(self, request, response):
        if not self.conn:
            response.success = False
            response.message = 'Not connected'
            return response
        with self.lock:
            self.conn.mav.command_long_send(
                self.conn.target_system, self.conn.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,                          # confirmation
                request.servo_number,       # param1: servo number
                request.pwm,                # param2: PWM value
                0, 0, 0, 0, 0)             # params 3-7 unused
        response.success = True
        response.message = f"Servo {request.servo_number} set to {request.pwm}"
        return response
```

- [ ] **Step 3: Build and verify**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select dbvf_msgs dbvf_autonomy
```
Expected: Build succeeds.

- [ ] **Step 4: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py
git commit -m "feat: add DoSetServo service handler to mavlink_interface_node"
```

---

## Task 3: Mission Helper Functions + Tests

**Files:**
- Create: `src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py`
- Create: `src/dbvf_autonomy/test/test_mission_helpers.py`
- Modify: `src/dbvf_autonomy/CMakeLists.txt`

- [ ] **Step 1: Write failing tests for helper functions**

Create `src/dbvf_autonomy/test/test_mission_helpers.py`:

```python
"""Tests for mission helper functions — pure math, no ROS2."""
import math

from dbvf_autonomy.mission_helpers import (
    ft_to_m,
    m_to_ft,
    haversine_distance_m,
    is_within_tolerance,
)


# ---------------------------------------------------------------------------
# Altitude conversion
# ---------------------------------------------------------------------------

def test_ft_to_m_35ft():
    assert abs(ft_to_m(35.0) - 10.668) < 0.001


def test_ft_to_m_zero():
    assert ft_to_m(0.0) == 0.0


def test_m_to_ft_10m():
    assert abs(m_to_ft(10.0) - 32.8084) < 0.01


def test_ft_m_roundtrip():
    """ft -> m -> ft should return the original value."""
    assert abs(m_to_ft(ft_to_m(100.0)) - 100.0) < 0.001


# ---------------------------------------------------------------------------
# Haversine distance
# ---------------------------------------------------------------------------

def test_haversine_same_point():
    d = haversine_distance_m(-35.363262, 149.165237, -35.363262, 149.165237)
    assert d < 0.01


def test_haversine_known_distance():
    """~111m per degree of latitude at any longitude."""
    d = haversine_distance_m(-35.0, 149.0, -35.001, 149.0)
    assert abs(d - 111.0) < 2.0  # Within 2m


def test_haversine_longitude_distance():
    """Longitude distance depends on latitude. At -35 deg, 1 deg lon ~ 91km."""
    d = haversine_distance_m(-35.0, 149.0, -35.0, 149.001)
    assert 80.0 < d < 100.0  # ~91m at lat -35


# ---------------------------------------------------------------------------
# Position tolerance
# ---------------------------------------------------------------------------

def test_within_tolerance_true():
    """Same point is within any positive tolerance."""
    assert is_within_tolerance(-35.363262, 149.165237,
                               -35.363262, 149.165237, 3.0)


def test_within_tolerance_false():
    """111m away is not within 3m tolerance."""
    assert not is_within_tolerance(-35.0, 149.0, -35.001, 149.0, 3.0)


def test_within_tolerance_boundary():
    """Point ~2m away is within 3m tolerance."""
    # ~2m north
    offset_lat = 2.0 / 111000.0
    assert is_within_tolerance(-35.0, 149.0,
                               -35.0 + offset_lat, 149.0, 3.0)
```

- [ ] **Step 2: Register test in CMakeLists.txt**

In `src/dbvf_autonomy/CMakeLists.txt`, inside the `if(BUILD_TESTING)` block, add after the last `ament_add_pytest_test` line:

```cmake
  ament_add_pytest_test(test_mission_helpers test/test_mission_helpers.py)
```

- [ ] **Step 3: Run tests to verify they fail**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select dbvf_autonomy && colcon test --packages-select dbvf_autonomy --pytest-args test/test_mission_helpers.py && colcon test-result --verbose
```
Expected: FAIL — `ModuleNotFoundError: No module named 'dbvf_autonomy.mission_helpers'`

- [ ] **Step 4: Implement helper functions**

Create `src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py`:

```python
"""Pure helper functions for the mission sequencer — no ROS2 dependencies."""
import math

# Conversion factor
_FT_PER_METER = 3.28084


def ft_to_m(feet):
    """Convert feet to meters."""
    return feet / _FT_PER_METER


def m_to_ft(meters):
    """Convert meters to feet."""
    return meters * _FT_PER_METER


def haversine_distance_m(lat1, lon1, lat2, lon2):
    """Haversine distance between two GPS points in meters."""
    R = 6371000.0  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = (math.sin(dphi / 2.0) ** 2
         + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2.0) ** 2)
    return R * 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))


def is_within_tolerance(lat1, lon1, lat2, lon2, tolerance_m):
    """Return True if the two GPS points are within tolerance_m meters."""
    return haversine_distance_m(lat1, lon1, lat2, lon2) < tolerance_m
```

- [ ] **Step 5: Build and run tests to verify they pass**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select dbvf_autonomy && colcon test --packages-select dbvf_autonomy --pytest-args test/test_mission_helpers.py && colcon test-result --verbose
```
Expected: All 10 tests PASS.

- [ ] **Step 6: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py \
        src/dbvf_autonomy/test/test_mission_helpers.py \
        src/dbvf_autonomy/CMakeLists.txt
git commit -m "feat: add mission helper functions (ft/m conversion, haversine, tolerance)"
```

---

## Task 4: Mission Config Validation + Tests

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py` (append validation)
- Create: `src/dbvf_autonomy/test/test_mission_config.py`
- Modify: `src/dbvf_autonomy/CMakeLists.txt`

- [ ] **Step 1: Write failing tests for config validation**

Create `src/dbvf_autonomy/test/test_mission_config.py`:

```python
"""Tests for mission config validation — pure functions, no ROS2."""
from dbvf_autonomy.mission_helpers import validate_mission_config, DEFAULT_MISSION_CONFIG


def _make_config(**overrides):
    """Return a valid config dict with optional overrides."""
    cfg = dict(DEFAULT_MISSION_CONFIG)
    cfg.update(overrides)
    return cfg


def test_valid_config_passes():
    errors = validate_mission_config(_make_config())
    assert errors == []


def test_missing_required_gps_field():
    cfg = _make_config()
    del cfg['home_lat']
    errors = validate_mission_config(cfg)
    assert any('home_lat' in e for e in errors)


def test_invalid_drop_target():
    cfg = _make_config(drop_target='F3')
    errors = validate_mission_config(cfg)
    assert any('drop_target' in e for e in errors)


def test_negative_timeout():
    cfg = _make_config(mission_timeout_s=-1.0)
    errors = validate_mission_config(cfg)
    assert any('mission_timeout_s' in e for e in errors)


def test_zero_altitude_invalid():
    cfg = _make_config(transit_altitude_ft=0.0)
    errors = validate_mission_config(cfg)
    assert any('transit_altitude_ft' in e for e in errors)
```

- [ ] **Step 2: Register test in CMakeLists.txt**

In `src/dbvf_autonomy/CMakeLists.txt`, inside the `if(BUILD_TESTING)` block, add:

```cmake
  ament_add_pytest_test(test_mission_config test/test_mission_config.py)
```

- [ ] **Step 3: Run tests to verify they fail**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select dbvf_autonomy && colcon test --packages-select dbvf_autonomy --pytest-args test/test_mission_config.py && colcon test-result --verbose
```
Expected: FAIL — `ImportError: cannot import name 'validate_mission_config'`

- [ ] **Step 4: Implement config validation and defaults**

Append to `src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py` (after the existing functions):

```python

# ---------------------------------------------------------------------------
# Mission config defaults and validation
# ---------------------------------------------------------------------------

_REQUIRED_GPS_KEYS = [
    'home_lat', 'home_lon',
    'landing_lat', 'landing_lon',
    'wa_lat', 'wa_lon',
    'f1_lat', 'f1_lon',
    'f2_lat', 'f2_lon',
]

_REQUIRED_POSITIVE = [
    'transit_altitude_ft',
    'position_tolerance_m',
    'takeoff_complete_alt_ft',
    'mission_timeout_s',
    'heartbeat_loss_timeout_s',
    'service_call_timeout_s',
    'guided_resend_interval_s',
    'drop_settle_time_s',
]

DEFAULT_MISSION_CONFIG = {
    # GPS coordinates (simulation defaults)
    'home_lat': -35.3632621,
    'home_lon': 149.1652374,
    'landing_lat': -35.3640000,
    'landing_lon': 149.1652374,
    'wa_lat': -35.3632531,
    'wa_lon': 149.1657896,
    'f1_lat': -35.3650000,
    'f1_lon': 149.1652374,
    'f2_lat': -35.3660000,
    'f2_lon': 149.1652374,

    # Flight parameters
    'transit_altitude_ft': 35.0,
    'position_tolerance_m': 3.0,
    'takeoff_complete_alt_ft': 33.0,

    # Payload servo
    'drop_servo_number': 9,
    'drop_servo_pwm_release': 1100,
    'drop_servo_pwm_hold': 1500,
    'drop_settle_time_s': 2.0,

    # Drop zone target
    'drop_target': 'F1',

    # Safety
    'mission_timeout_s': 540.0,
    'heartbeat_loss_timeout_s': 5.0,
    'service_call_timeout_s': 5.0,
    'guided_resend_interval_s': 1.0,

    # Altitude source
    'prefer_rangefinder': True,
}


def validate_mission_config(config):
    """Validate a mission config dict. Returns a list of error strings (empty = valid)."""
    errors = []

    for key in _REQUIRED_GPS_KEYS:
        if key not in config:
            errors.append(f"Missing required GPS field: {key}")

    for key in _REQUIRED_POSITIVE:
        if key in config and config[key] <= 0.0:
            errors.append(f"{key} must be positive, got {config[key]}")

    if config.get('drop_target') not in ('F1', 'F2'):
        errors.append(f"drop_target must be 'F1' or 'F2', got {config.get('drop_target')!r}")

    return errors
```

- [ ] **Step 5: Build and run tests to verify they pass**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select dbvf_autonomy && colcon test --packages-select dbvf_autonomy --pytest-args test/test_mission_config.py && colcon test-result --verbose
```
Expected: All 5 tests PASS.

- [ ] **Step 6: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py \
        src/dbvf_autonomy/test/test_mission_config.py \
        src/dbvf_autonomy/CMakeLists.txt
git commit -m "feat: add mission config defaults and validation"
```

---

## Task 5: Mission State Machine — All States + FM-1 Tests

**Files:**
- Create: `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py`
- Create: `src/dbvf_autonomy/test/test_mission_state_machine.py`
- Modify: `src/dbvf_autonomy/CMakeLists.txt`

- [ ] **Step 1: Write failing tests for IDLE, PREFLIGHT, and FM-1 states**

Create `src/dbvf_autonomy/test/test_mission_state_machine.py`:

```python
"""Tests for mission state machine — pure Python, no ROS2."""
from dbvf_autonomy.mission_state_machine import MissionStateMachine, MissionState
from dbvf_autonomy.mission_helpers import DEFAULT_MISSION_CONFIG, ft_to_m


# ---------------------------------------------------------------------------
# Mock vehicle state (same pattern as test_state_machine.py)
# ---------------------------------------------------------------------------

class MockVehicleState:
    def __init__(self, lat=-35.3632621, lon=149.1652374, alt_rel=0.0,
                 armed=True, mode='GUIDED', vz=0.0, range_alt=-1.0,
                 heartbeat_ok=True):
        self.lat = lat
        self.lon = lon
        self.alt_rel = alt_rel
        self.armed = armed
        self.mode = mode
        self.vz = vz
        self.range_alt = range_alt
        self.heartbeat_ok = heartbeat_ok


def _make_config(**overrides):
    cfg = dict(DEFAULT_MISSION_CONFIG)
    cfg.update(overrides)
    return cfg


# Convenient GPS coords from default config
H_LAT, H_LON = -35.3632621, 149.1652374
L_LAT, L_LON = -35.3640000, 149.1652374
WA_LAT, WA_LON = -35.3632531, 149.1657896
F1_LAT, F1_LON = -35.3650000, 149.1652374


# ---------------------------------------------------------------------------
# Basic state + start
# ---------------------------------------------------------------------------

def test_starts_idle():
    sm = MissionStateMachine(_make_config())
    assert sm.state == MissionState.IDLE


def test_start_transitions_to_preflight():
    sm = MissionStateMachine(_make_config())
    sm.start()
    assert sm.state == MissionState.PREFLIGHT_CHECK


# ---------------------------------------------------------------------------
# PREFLIGHT_CHECK
# ---------------------------------------------------------------------------

def test_preflight_pass():
    sm = MissionStateMachine(_make_config())
    sm.start()
    vs = MockVehicleState(armed=True, mode='GUIDED')
    state, info = sm.update(vs, 0.0)
    assert state == MissionState.TAKEOFF_H
    assert info['action'] == 'preflight_pass'


def test_preflight_fail_not_armed():
    sm = MissionStateMachine(_make_config())
    sm.start()
    vs = MockVehicleState(armed=False, mode='GUIDED')
    state, info = sm.update(vs, 0.0)
    assert state == MissionState.ABORT
    assert 'armed' in info['reason'].lower()


def test_preflight_fail_wrong_mode():
    sm = MissionStateMachine(_make_config())
    sm.start()
    vs = MockVehicleState(armed=True, mode='STABILIZE')
    state, info = sm.update(vs, 0.0)
    assert state == MissionState.ABORT
    assert 'mode' in info['reason'].lower()


# ---------------------------------------------------------------------------
# TAKEOFF_H
# ---------------------------------------------------------------------------

def _to_takeoff_h(sm):
    sm.start()
    vs = MockVehicleState(armed=True, mode='GUIDED')
    sm.update(vs, 0.0)
    assert sm.state == MissionState.TAKEOFF_H
    return vs


def test_takeoff_h_waits_for_altitude():
    sm = MissionStateMachine(_make_config())
    _to_takeoff_h(sm)
    vs = MockVehicleState(alt_rel=5.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 1.0)
    assert state == MissionState.TAKEOFF_H
    assert info['action'] == 'climbing'


def test_takeoff_h_complete_with_rangefinder():
    sm = MissionStateMachine(_make_config(prefer_rangefinder=True))
    _to_takeoff_h(sm)
    alt = ft_to_m(33.0) + 0.1  # Just above 33ft
    vs = MockVehicleState(alt_rel=alt, range_alt=alt, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 1.0)
    assert state == MissionState.TRANSIT_H_TO_L
    assert info['action'] == 'takeoff_complete'


def test_takeoff_h_complete_fallback_to_alt_rel():
    sm = MissionStateMachine(_make_config(prefer_rangefinder=True))
    _to_takeoff_h(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(alt_rel=alt, range_alt=-1.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 1.0)
    assert state == MissionState.TRANSIT_H_TO_L


# ---------------------------------------------------------------------------
# TRANSIT_H_TO_L
# ---------------------------------------------------------------------------

def _to_transit_h_to_l(sm):
    _to_takeoff_h(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(alt_rel=alt, range_alt=alt, armed=True, mode='GUIDED')
    sm.update(vs, 1.0)
    assert sm.state == MissionState.TRANSIT_H_TO_L
    return vs


def test_transit_h_to_l_not_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_h_to_l(sm)
    vs = MockVehicleState(lat=H_LAT, lon=H_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 2.0)
    assert state == MissionState.TRANSIT_H_TO_L
    assert info['action'] == 'transiting'


def test_transit_h_to_l_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_h_to_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 2.0)
    assert state == MissionState.LAND_L
    assert info['action'] == 'arrived_l'


# ---------------------------------------------------------------------------
# LAND_L
# ---------------------------------------------------------------------------

def _to_land_l(sm):
    _to_transit_h_to_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 2.0)
    assert sm.state == MissionState.LAND_L


def test_land_l_waits_while_airborne():
    sm = MissionStateMachine(_make_config())
    _to_land_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=5.0, armed=True, vz=-0.5)
    state, info = sm.update(vs, 3.0)
    assert state == MissionState.LAND_L
    assert info['action'] == 'landing'


def test_land_l_complete_disarmed():
    sm = MissionStateMachine(_make_config())
    _to_land_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.1, armed=False, vz=0.0)
    state, info = sm.update(vs, 3.0)
    assert state == MissionState.WAIT_FLAGGER
    assert info['action'] == 'landed_l'


def test_land_l_complete_low_and_slow():
    sm = MissionStateMachine(_make_config())
    _to_land_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.2, armed=True, vz=0.05)
    state, info = sm.update(vs, 3.0)
    assert state == MissionState.WAIT_FLAGGER


# ---------------------------------------------------------------------------
# WAIT_FLAGGER
# ---------------------------------------------------------------------------

def _to_wait_flagger(sm):
    _to_land_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.1, armed=False)
    sm.update(vs, 3.0)
    assert sm.state == MissionState.WAIT_FLAGGER


def test_wait_flagger_stays_without_resume():
    sm = MissionStateMachine(_make_config())
    _to_wait_flagger(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.0, armed=False)
    state, info = sm.update(vs, 100.0)
    assert state == MissionState.WAIT_FLAGGER
    assert info['action'] == 'waiting'


def test_wait_flagger_resumes():
    sm = MissionStateMachine(_make_config())
    _to_wait_flagger(sm)
    sm.resume()
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.0, armed=False)
    state, info = sm.update(vs, 4.0)
    assert state == MissionState.TAKEOFF_L
    assert info['action'] == 'flagger_resume'
```

- [ ] **Step 2: Register test in CMakeLists.txt**

In `src/dbvf_autonomy/CMakeLists.txt`, inside the `if(BUILD_TESTING)` block, add:

```cmake
  ament_add_pytest_test(test_mission_fsm test/test_mission_state_machine.py)
```

- [ ] **Step 3: Run tests to verify they fail**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select dbvf_autonomy && colcon test --packages-select dbvf_autonomy --pytest-args test/test_mission_state_machine.py && colcon test-result --verbose
```
Expected: FAIL — `ModuleNotFoundError: No module named 'dbvf_autonomy.mission_state_machine'`

- [ ] **Step 4: Implement the full state machine**

Create `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py`:

```python
"""Mission sequencer state machine — pure Python, no ROS2 dependencies."""
from enum import Enum

from dbvf_autonomy.mission_helpers import ft_to_m, haversine_distance_m


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
    LAND_WA = 'LAND_WA'
    TAKEOFF_WA = 'TAKEOFF_WA'
    TRANSIT_TO_DROP_2 = 'TRANSIT_TO_DROP_2'
    DROP_PAYLOAD_2 = 'DROP_PAYLOAD_2'
    TRANSIT_TO_H = 'TRANSIT_TO_H'
    LAND_H = 'LAND_H'
    COMPLETE = 'COMPLETE'
    ABORT = 'ABORT'


# States grouped by mission phase (for /dbvf/mission_phase topic)
_STATE_TO_PHASE = {
    MissionState.IDLE: 'IDLE',
    MissionState.PREFLIGHT_CHECK: 'PREFLIGHT',
    MissionState.TAKEOFF_H: 'FM1',
    MissionState.TRANSIT_H_TO_L: 'FM1',
    MissionState.LAND_L: 'FM1',
    MissionState.WAIT_FLAGGER: 'FM1',
    MissionState.TAKEOFF_L: 'FM2',
    MissionState.TRANSIT_TO_DROP: 'FM2',
    MissionState.DROP_PAYLOAD: 'FM2',
    MissionState.TRANSIT_TO_WA: 'FM3',
    MissionState.LAND_WA: 'FM3',
    MissionState.TAKEOFF_WA: 'FM3',
    MissionState.TRANSIT_TO_DROP_2: 'FM3',
    MissionState.DROP_PAYLOAD_2: 'FM3',
    MissionState.TRANSIT_TO_H: 'RTH',
    MissionState.LAND_H: 'RTH',
    MissionState.COMPLETE: 'COMPLETE',
    MissionState.ABORT: 'ABORT',
}


def get_mission_phase(state):
    """Return the mission phase string for a given state."""
    return _STATE_TO_PHASE.get(state, 'UNKNOWN')


class MissionStateMachine:
    def __init__(self, config):
        self.state = MissionState.IDLE
        self.config = config
        self.start_time = None
        self.abort_reason = ''

        # WAIT_FLAGGER resume flag
        self._resume_requested = False

        # DROP_PAYLOAD timing
        self._drop_start_time = None

        # LAND_WA tracking
        self._landing_state = None

        # Guided position resend tracking
        self._last_guided_send_time = 0.0

        # Precompute altitudes in meters
        self._transit_alt_m = ft_to_m(config['transit_altitude_ft'])
        self._takeoff_complete_alt_m = ft_to_m(config['takeoff_complete_alt_ft'])
        self._tolerance = config['position_tolerance_m']

    def start(self):
        """Begin the mission — transition from IDLE to PREFLIGHT_CHECK."""
        self.state = MissionState.PREFLIGHT_CHECK
        self.start_time = None
        self.abort_reason = ''
        self._resume_requested = False
        self._drop_start_time = None
        self._landing_state = None
        self._last_guided_send_time = 0.0

    def resume(self):
        """Signal that the flagger has approved — used during WAIT_FLAGGER."""
        self._resume_requested = True

    def set_landing_state(self, landing_state_str):
        """Called by the ROS node when /dbvf/landing_state updates."""
        self._landing_state = landing_state_str

    def abort(self, reason):
        """Force transition to ABORT from any state."""
        self.abort_reason = reason
        self.state = MissionState.ABORT

    def update(self, vehicle_state, current_time):
        """Advance the state machine. Returns (state, action_dict).

        action_dict keys:
          'action': str describing what happened or is happening
          'reason': str (only on ABORT)
          'entry_actions': list of str describing actions the ROS node should take
        """
        vs = vehicle_state

        if self.state == MissionState.IDLE:
            return self.state, {'action': None, 'entry_actions': []}

        if self.state == MissionState.COMPLETE:
            return self.state, {'action': 'complete', 'entry_actions': []}

        if self.state == MissionState.ABORT:
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': []}

        # Start mission timer on first non-IDLE update
        if self.start_time is None:
            self.start_time = current_time

        # Global timeout check (skip for WAIT_FLAGGER — no timeout there)
        if (self.state != MissionState.WAIT_FLAGGER
                and current_time - self.start_time > self.config['mission_timeout_s']):
            self.state = MissionState.ABORT
            self.abort_reason = 'Mission timeout'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': ['set_mode_land']}

        # Heartbeat check (skip for WAIT_FLAGGER — aircraft is on ground)
        if (self.state != MissionState.WAIT_FLAGGER
                and hasattr(vs, 'heartbeat_ok') and not vs.heartbeat_ok):
            self.state = MissionState.ABORT
            self.abort_reason = 'Heartbeat loss'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': ['set_mode_land']}

        # Dispatch to state handler
        handler = self._handlers.get(self.state)
        if handler:
            return handler(self, vs, current_time)
        return self.state, {'action': None, 'entry_actions': []}

    # -- State handlers -------------------------------------------------------

    def _preflight(self, vs, t):
        if not vs.armed:
            self.state = MissionState.ABORT
            self.abort_reason = 'Preflight failed: not armed'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': []}
        if vs.mode != 'GUIDED':
            self.state = MissionState.ABORT
            self.abort_reason = 'Preflight failed: mode is not GUIDED'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': []}
        self.state = MissionState.TAKEOFF_H
        return self.state, {'action': 'preflight_pass',
                            'entry_actions': ['send_guided_position_h']}

    def _takeoff_h(self, vs, t):
        alt = self._get_altitude(vs)
        if alt >= self._takeoff_complete_alt_m:
            self.state = MissionState.TRANSIT_H_TO_L
            return self.state, {'action': 'takeoff_complete',
                                'entry_actions': ['send_guided_position_l']}
        return self.state, {'action': 'climbing', 'entry_actions': []}

    def _transit_h_to_l(self, vs, t):
        target_lat = self.config['landing_lat']
        target_lon = self.config['landing_lon']
        dist = haversine_distance_m(vs.lat, vs.lon, target_lat, target_lon)
        if dist < self._tolerance:
            self.state = MissionState.LAND_L
            return self.state, {'action': 'arrived_l',
                                'entry_actions': ['set_mode_land']}
        entry = []
        if t - self._last_guided_send_time >= self.config['guided_resend_interval_s']:
            entry.append('send_guided_position_l')
            self._last_guided_send_time = t
        return self.state, {'action': 'transiting', 'entry_actions': entry}

    def _land_l(self, vs, t):
        if self._is_landed(vs):
            self.state = MissionState.WAIT_FLAGGER
            return self.state, {'action': 'landed_l', 'entry_actions': []}
        return self.state, {'action': 'landing', 'entry_actions': []}

    def _wait_flagger(self, vs, t):
        if self._resume_requested:
            self._resume_requested = False
            self.state = MissionState.TAKEOFF_L
            return self.state, {'action': 'flagger_resume',
                                'entry_actions': ['arm', 'set_mode_guided',
                                                  'send_guided_position_l_alt']}
        return self.state, {'action': 'waiting', 'entry_actions': []}

    def _takeoff_l(self, vs, t):
        alt = self._get_altitude(vs)
        if alt >= self._takeoff_complete_alt_m:
            self.state = MissionState.TRANSIT_TO_DROP
            return self.state, {'action': 'takeoff_complete',
                                'entry_actions': ['send_guided_position_drop']}
        return self.state, {'action': 'climbing', 'entry_actions': []}

    def _transit_to_drop(self, vs, t):
        drop_lat, drop_lon = self._get_drop_coords()
        dist = haversine_distance_m(vs.lat, vs.lon, drop_lat, drop_lon)
        if dist < self._tolerance:
            self._drop_start_time = t
            self.state = MissionState.DROP_PAYLOAD
            return self.state, {'action': 'arrived_drop',
                                'entry_actions': ['servo_release']}
        entry = []
        if t - self._last_guided_send_time >= self.config['guided_resend_interval_s']:
            entry.append('send_guided_position_drop')
            self._last_guided_send_time = t
        return self.state, {'action': 'transiting', 'entry_actions': entry}

    def _drop_payload(self, vs, t):
        elapsed = t - self._drop_start_time
        if elapsed >= self.config['drop_settle_time_s']:
            self.state = MissionState.TRANSIT_TO_WA
            self._last_guided_send_time = 0.0
            return self.state, {'action': 'drop_complete',
                                'entry_actions': ['send_guided_position_wa']}
        return self.state, {'action': 'dropping', 'entry_actions': []}

    def _transit_to_wa(self, vs, t):
        wa_lat = self.config['wa_lat']
        wa_lon = self.config['wa_lon']
        dist = haversine_distance_m(vs.lat, vs.lon, wa_lat, wa_lon)
        if dist < self._tolerance:
            self.state = MissionState.LAND_WA
            self._landing_state = None
            return self.state, {'action': 'arrived_wa',
                                'entry_actions': ['start_precision_landing']}
        entry = []
        if t - self._last_guided_send_time >= self.config['guided_resend_interval_s']:
            entry.append('send_guided_position_wa')
            self._last_guided_send_time = t
        return self.state, {'action': 'transiting', 'entry_actions': entry}

    def _land_wa(self, vs, t):
        if self._landing_state == 'LANDED':
            self.state = MissionState.TAKEOFF_WA
            return self.state, {'action': 'landed_wa',
                                'entry_actions': ['arm', 'set_mode_guided',
                                                  'send_guided_position_wa_alt']}
        if self._landing_state == 'ABORT_LAND':
            self.state = MissionState.ABORT
            self.abort_reason = 'Precision landing failed at WA'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': ['set_mode_land']}
        return self.state, {'action': 'precision_landing', 'entry_actions': []}

    def _takeoff_wa(self, vs, t):
        alt = self._get_altitude(vs)
        if alt >= self._takeoff_complete_alt_m:
            self.state = MissionState.TRANSIT_TO_DROP_2
            self._last_guided_send_time = 0.0
            return self.state, {'action': 'takeoff_complete',
                                'entry_actions': ['send_guided_position_drop']}
        return self.state, {'action': 'climbing', 'entry_actions': []}

    def _transit_to_drop_2(self, vs, t):
        drop_lat, drop_lon = self._get_drop_coords()
        dist = haversine_distance_m(vs.lat, vs.lon, drop_lat, drop_lon)
        if dist < self._tolerance:
            self._drop_start_time = t
            self.state = MissionState.DROP_PAYLOAD_2
            return self.state, {'action': 'arrived_drop_2',
                                'entry_actions': ['servo_release']}
        entry = []
        if t - self._last_guided_send_time >= self.config['guided_resend_interval_s']:
            entry.append('send_guided_position_drop')
            self._last_guided_send_time = t
        return self.state, {'action': 'transiting', 'entry_actions': entry}

    def _drop_payload_2(self, vs, t):
        elapsed = t - self._drop_start_time
        if elapsed >= self.config['drop_settle_time_s']:
            self.state = MissionState.TRANSIT_TO_H
            self._last_guided_send_time = 0.0
            return self.state, {'action': 'drop_complete',
                                'entry_actions': ['send_guided_position_h']}
        return self.state, {'action': 'dropping', 'entry_actions': []}

    def _transit_to_h(self, vs, t):
        h_lat = self.config['home_lat']
        h_lon = self.config['home_lon']
        dist = haversine_distance_m(vs.lat, vs.lon, h_lat, h_lon)
        if dist < self._tolerance:
            self.state = MissionState.LAND_H
            return self.state, {'action': 'arrived_h',
                                'entry_actions': ['set_mode_land']}
        entry = []
        if t - self._last_guided_send_time >= self.config['guided_resend_interval_s']:
            entry.append('send_guided_position_h')
            self._last_guided_send_time = t
        return self.state, {'action': 'transiting', 'entry_actions': entry}

    def _land_h(self, vs, t):
        if self._is_landed(vs):
            self.state = MissionState.COMPLETE
            return self.state, {'action': 'landed_h', 'entry_actions': []}
        return self.state, {'action': 'landing', 'entry_actions': []}

    # Handler dispatch table (defined after methods so they exist)
    _handlers = {
        MissionState.PREFLIGHT_CHECK: _preflight,
        MissionState.TAKEOFF_H: _takeoff_h,
        MissionState.TRANSIT_H_TO_L: _transit_h_to_l,
        MissionState.LAND_L: _land_l,
        MissionState.WAIT_FLAGGER: _wait_flagger,
        MissionState.TAKEOFF_L: _takeoff_l,
        MissionState.TRANSIT_TO_DROP: _transit_to_drop,
        MissionState.DROP_PAYLOAD: _drop_payload,
        MissionState.TRANSIT_TO_WA: _transit_to_wa,
        MissionState.LAND_WA: _land_wa,
        MissionState.TAKEOFF_WA: _takeoff_wa,
        MissionState.TRANSIT_TO_DROP_2: _transit_to_drop_2,
        MissionState.DROP_PAYLOAD_2: _drop_payload_2,
        MissionState.TRANSIT_TO_H: _transit_to_h,
        MissionState.LAND_H: _land_h,
    }

    # -- Helpers --------------------------------------------------------------

    def _get_altitude(self, vs):
        """Return best available altitude in meters."""
        if self.config.get('prefer_rangefinder', True) and vs.range_alt >= 0.0:
            return vs.range_alt
        return vs.alt_rel

    def _get_drop_coords(self):
        """Return (lat, lon) of the configured drop target."""
        if self.config['drop_target'] == 'F2':
            return self.config['f2_lat'], self.config['f2_lon']
        return self.config['f1_lat'], self.config['f1_lon']

    @staticmethod
    def _is_landed(vs):
        """Detect landed condition: disarmed OR (low alt AND low vertical speed)."""
        if not vs.armed:
            return True
        return vs.alt_rel < 0.3 and abs(vs.vz) < 0.1
```

- [ ] **Step 5: Build and run tests to verify FM-1 tests pass**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select dbvf_autonomy && colcon test --packages-select dbvf_autonomy --pytest-args test/test_mission_state_machine.py && colcon test-result --verbose
```
Expected: All 15 tests PASS.

- [ ] **Step 6: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py \
        src/dbvf_autonomy/test/test_mission_state_machine.py \
        src/dbvf_autonomy/CMakeLists.txt
git commit -m "feat: add mission state machine with FM-1 transitions (IDLE through WAIT_FLAGGER)"
```

---

## Task 6: FM-2, FM-3, RTH, and ABORT Tests

**Files:**
- Modify: `src/dbvf_autonomy/test/test_mission_state_machine.py` (append tests)

- [ ] **Step 1: Add FM-2 tests**

Append to `src/dbvf_autonomy/test/test_mission_state_machine.py`:

```python


# ---------------------------------------------------------------------------
# FM-2: TAKEOFF_L -> TRANSIT_TO_DROP -> DROP_PAYLOAD
# ---------------------------------------------------------------------------

def _to_takeoff_l(sm):
    _to_wait_flagger(sm)
    sm.resume()
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.0, armed=False)
    sm.update(vs, 4.0)
    assert sm.state == MissionState.TAKEOFF_L
    return vs


def test_takeoff_l_waits_for_altitude():
    sm = MissionStateMachine(_make_config())
    _to_takeoff_l(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=5.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 5.0)
    assert state == MissionState.TAKEOFF_L
    assert info['action'] == 'climbing'


def test_takeoff_l_complete():
    sm = MissionStateMachine(_make_config())
    _to_takeoff_l(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=alt, range_alt=alt,
                          armed=True, mode='GUIDED')
    state, info = sm.update(vs, 5.0)
    assert state == MissionState.TRANSIT_TO_DROP


def _to_transit_to_drop(sm):
    _to_takeoff_l(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=alt, range_alt=alt,
                          armed=True, mode='GUIDED')
    sm.update(vs, 5.0)
    assert sm.state == MissionState.TRANSIT_TO_DROP


def test_transit_to_drop_not_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_drop(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 6.0)
    assert state == MissionState.TRANSIT_TO_DROP
    assert info['action'] == 'transiting'


def test_transit_to_drop_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_drop(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 6.0)
    assert state == MissionState.DROP_PAYLOAD
    assert info['action'] == 'arrived_drop'
    assert 'servo_release' in info['entry_actions']


def _to_drop_payload(sm):
    _to_transit_to_drop(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 6.0)
    assert sm.state == MissionState.DROP_PAYLOAD


def test_drop_payload_waits_settle():
    sm = MissionStateMachine(_make_config())
    _to_drop_payload(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 7.0)  # 1.0s < 2.0s settle
    assert state == MissionState.DROP_PAYLOAD
    assert info['action'] == 'dropping'


def test_drop_payload_complete():
    sm = MissionStateMachine(_make_config())
    _to_drop_payload(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 8.1)  # 2.1s > 2.0s settle
    assert state == MissionState.TRANSIT_TO_WA
    assert info['action'] == 'drop_complete'
```

- [ ] **Step 2: Add FM-3 tests**

Append to `src/dbvf_autonomy/test/test_mission_state_machine.py`:

```python


# ---------------------------------------------------------------------------
# FM-3: TRANSIT_TO_WA -> LAND_WA -> TAKEOFF_WA -> TRANSIT_TO_DROP_2 -> DROP_PAYLOAD_2
# ---------------------------------------------------------------------------

def _to_transit_to_wa(sm):
    _to_drop_payload(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 8.1)  # drop_settle_time elapsed
    assert sm.state == MissionState.TRANSIT_TO_WA


def test_transit_to_wa_not_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_wa(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 9.0)
    assert state == MissionState.TRANSIT_TO_WA


def test_transit_to_wa_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_wa(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 9.0)
    assert state == MissionState.LAND_WA
    assert 'start_precision_landing' in info['entry_actions']


def _to_land_wa(sm):
    _to_transit_to_wa(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 9.0)
    assert sm.state == MissionState.LAND_WA


def test_land_wa_waiting():
    sm = MissionStateMachine(_make_config())
    _to_land_wa(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=5.0, armed=True, mode='LAND')
    state, info = sm.update(vs, 10.0)
    assert state == MissionState.LAND_WA
    assert info['action'] == 'precision_landing'


def test_land_wa_success():
    sm = MissionStateMachine(_make_config())
    _to_land_wa(sm)
    sm.set_landing_state('LANDED')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.1, armed=False)
    state, info = sm.update(vs, 10.0)
    assert state == MissionState.TAKEOFF_WA
    assert info['action'] == 'landed_wa'


def test_land_wa_abort():
    sm = MissionStateMachine(_make_config())
    _to_land_wa(sm)
    sm.set_landing_state('ABORT_LAND')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=1.0, armed=True)
    state, info = sm.update(vs, 10.0)
    assert state == MissionState.ABORT
    assert 'Precision landing failed' in info['reason']


def _to_takeoff_wa(sm):
    _to_land_wa(sm)
    sm.set_landing_state('LANDED')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.1, armed=False)
    sm.update(vs, 10.0)
    assert sm.state == MissionState.TAKEOFF_WA


def test_takeoff_wa_complete():
    sm = MissionStateMachine(_make_config())
    _to_takeoff_wa(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=alt, range_alt=alt,
                          armed=True, mode='GUIDED')
    state, info = sm.update(vs, 11.0)
    assert state == MissionState.TRANSIT_TO_DROP_2


def _to_transit_to_drop_2(sm):
    _to_takeoff_wa(sm)
    alt = ft_to_m(33.0) + 0.1
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=alt, range_alt=alt,
                          armed=True, mode='GUIDED')
    sm.update(vs, 11.0)
    assert sm.state == MissionState.TRANSIT_TO_DROP_2


def test_transit_to_drop_2_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_drop_2(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 12.0)
    assert state == MissionState.DROP_PAYLOAD_2
    assert 'servo_release' in info['entry_actions']


def _to_drop_payload_2(sm):
    _to_transit_to_drop_2(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 12.0)
    assert sm.state == MissionState.DROP_PAYLOAD_2


def test_drop_payload_2_complete():
    sm = MissionStateMachine(_make_config())
    _to_drop_payload_2(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 14.1)  # 2.1s > 2.0s settle
    assert state == MissionState.TRANSIT_TO_H
    assert info['action'] == 'drop_complete'
```

- [ ] **Step 3: Add RTH and ABORT tests**

Append to `src/dbvf_autonomy/test/test_mission_state_machine.py`:

```python


# ---------------------------------------------------------------------------
# Return Home: TRANSIT_TO_H -> LAND_H -> COMPLETE
# ---------------------------------------------------------------------------

def _to_transit_to_h(sm):
    _to_drop_payload_2(sm)
    vs = MockVehicleState(lat=F1_LAT, lon=F1_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 14.1)
    assert sm.state == MissionState.TRANSIT_TO_H


def test_transit_to_h_arrived():
    sm = MissionStateMachine(_make_config())
    _to_transit_to_h(sm)
    vs = MockVehicleState(lat=H_LAT, lon=H_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    state, info = sm.update(vs, 15.0)
    assert state == MissionState.LAND_H
    assert 'set_mode_land' in info['entry_actions']


def _to_land_h(sm):
    _to_transit_to_h(sm)
    vs = MockVehicleState(lat=H_LAT, lon=H_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 15.0)
    assert sm.state == MissionState.LAND_H


def test_land_h_complete():
    sm = MissionStateMachine(_make_config())
    _to_land_h(sm)
    vs = MockVehicleState(lat=H_LAT, lon=H_LON, alt_rel=0.1, armed=False, vz=0.0)
    state, info = sm.update(vs, 16.0)
    assert state == MissionState.COMPLETE
    assert info['action'] == 'landed_h'


# ---------------------------------------------------------------------------
# ABORT: reachable from any state
# ---------------------------------------------------------------------------

def test_abort_manual():
    sm = MissionStateMachine(_make_config())
    _to_transit_h_to_l(sm)
    sm.abort('Manual abort')
    vs = MockVehicleState()
    state, info = sm.update(vs, 5.0)
    assert state == MissionState.ABORT
    assert info['reason'] == 'Manual abort'


def test_abort_timeout():
    sm = MissionStateMachine(_make_config(mission_timeout_s=10.0))
    _to_takeoff_h(sm)
    vs = MockVehicleState(alt_rel=5.0, armed=True, mode='GUIDED')
    sm.update(vs, 0.0)  # set start_time
    state, info = sm.update(vs, 11.0)  # 11s > 10s timeout
    assert state == MissionState.ABORT
    assert 'timeout' in info['reason'].lower()


def test_abort_heartbeat_loss():
    sm = MissionStateMachine(_make_config())
    _to_transit_h_to_l(sm)
    vs = MockVehicleState(lat=H_LAT, lon=H_LON, alt_rel=11.0, armed=True,
                          mode='GUIDED', heartbeat_ok=False)
    state, info = sm.update(vs, 5.0)
    assert state == MissionState.ABORT
    assert 'heartbeat' in info['reason'].lower()


def test_wait_flagger_ignores_timeout():
    """WAIT_FLAGGER should NOT abort on mission timeout."""
    sm = MissionStateMachine(_make_config(mission_timeout_s=10.0))
    _to_wait_flagger(sm)
    vs = MockVehicleState(lat=L_LAT, lon=L_LON, alt_rel=0.0, armed=False)
    state, info = sm.update(vs, 1000.0)  # Way past timeout
    assert state == MissionState.WAIT_FLAGGER


# ---------------------------------------------------------------------------
# Drop target selection
# ---------------------------------------------------------------------------

def test_drop_target_f2():
    sm = MissionStateMachine(_make_config(drop_target='F2'))
    lat, lon = sm._get_drop_coords()
    assert lat == -35.3660000
    assert lon == 149.1652374


# ---------------------------------------------------------------------------
# Mission phase mapping
# ---------------------------------------------------------------------------

def test_phase_fm1():
    from dbvf_autonomy.mission_state_machine import get_mission_phase
    assert get_mission_phase(MissionState.TAKEOFF_H) == 'FM1'
    assert get_mission_phase(MissionState.TRANSIT_H_TO_L) == 'FM1'
    assert get_mission_phase(MissionState.LAND_L) == 'FM1'


def test_phase_fm2():
    from dbvf_autonomy.mission_state_machine import get_mission_phase
    assert get_mission_phase(MissionState.TAKEOFF_L) == 'FM2'
    assert get_mission_phase(MissionState.DROP_PAYLOAD) == 'FM2'


def test_phase_fm3():
    from dbvf_autonomy.mission_state_machine import get_mission_phase
    assert get_mission_phase(MissionState.LAND_WA) == 'FM3'
    assert get_mission_phase(MissionState.DROP_PAYLOAD_2) == 'FM3'


def test_phase_rth():
    from dbvf_autonomy.mission_state_machine import get_mission_phase
    assert get_mission_phase(MissionState.TRANSIT_TO_H) == 'RTH'
    assert get_mission_phase(MissionState.LAND_H) == 'RTH'
```

- [ ] **Step 4: Build and run all state machine tests**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select dbvf_autonomy && colcon test --packages-select dbvf_autonomy --pytest-args test/test_mission_state_machine.py && colcon test-result --verbose
```
Expected: All ~35 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/dbvf_autonomy/test/test_mission_state_machine.py
git commit -m "feat: add FM-2, FM-3, RTH, and ABORT tests for mission state machine"
```

---

## Task 7: Mission Sequencer ROS2 Node

**Files:**
- Create: `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py`
- Create: `src/dbvf_autonomy/scripts/mission_sequencer_node`
- Modify: `src/dbvf_autonomy/CMakeLists.txt`

- [ ] **Step 1: Create the ROS2 node**

Create `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py`:

```python
"""Mission sequencer node — ROS2 wrapper around MissionStateMachine."""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from dbvf_msgs.msg import VehicleState
from dbvf_msgs.srv import (
    SetMode, ArmMotors, SendGuidedPosition, StartPrecisionLanding,
    StartMission, ResumeMission, AbortMission, DoSetServo,
)

from dbvf_autonomy.mission_state_machine import MissionStateMachine, MissionState, get_mission_phase
from dbvf_autonomy.mission_helpers import ft_to_m, validate_mission_config


class MissionSequencerNode(Node):
    def __init__(self):
        super().__init__('mission_sequencer')

        # Declare all parameters with defaults
        param_defaults = {
            'home_lat': -35.3632621,
            'home_lon': 149.1652374,
            'landing_lat': -35.3640000,
            'landing_lon': 149.1652374,
            'wa_lat': -35.3632531,
            'wa_lon': 149.1657896,
            'f1_lat': -35.3650000,
            'f1_lon': 149.1652374,
            'f2_lat': -35.3660000,
            'f2_lon': 149.1652374,
            'transit_altitude_ft': 35.0,
            'position_tolerance_m': 3.0,
            'takeoff_complete_alt_ft': 33.0,
            'drop_servo_number': 9,
            'drop_servo_pwm_release': 1100,
            'drop_servo_pwm_hold': 1500,
            'drop_settle_time_s': 2.0,
            'drop_target': 'F1',
            'mission_timeout_s': 540.0,
            'heartbeat_loss_timeout_s': 5.0,
            'service_call_timeout_s': 5.0,
            'guided_resend_interval_s': 1.0,
            'prefer_rangefinder': True,
        }
        for name, default in param_defaults.items():
            self.declare_parameter(name, default)

        # Build config dict from parameters
        self.mission_config = {
            name: self.get_parameter(name).value for name in param_defaults
        }

        # Validate config
        errors = validate_mission_config(self.mission_config)
        if errors:
            for e in errors:
                self.get_logger().error(f'Config error: {e}')

        self.fsm = MissionStateMachine(self.mission_config)
        self.latest_vehicle_state = None
        self._last_heartbeat_time = time.time()
        self._prev_phase = 'IDLE'

        # Precompute transit altitude in meters
        self._transit_alt_m = ft_to_m(self.mission_config['transit_altitude_ft'])

        # Subscribers
        self.create_subscription(
            VehicleState, '/dbvf/vehicle_state', self._vehicle_state_cb, 10)
        self.create_subscription(
            Bool, '/dbvf/heartbeat_status', self._heartbeat_cb, 10)
        self.create_subscription(
            String, '/dbvf/landing_state', self._landing_state_cb, 10)

        # Publishers
        self.state_pub = self.create_publisher(String, '/dbvf/mission_state', 10)
        self.phase_pub = self.create_publisher(String, '/dbvf/mission_phase', 10)

        # Service clients
        self.set_mode_cli = self.create_client(SetMode, '/dbvf/set_mode')
        self.arm_cli = self.create_client(ArmMotors, '/dbvf/arm_motors')
        self.guided_cli = self.create_client(
            SendGuidedPosition, '/dbvf/send_guided_position')
        self.precision_land_cli = self.create_client(
            StartPrecisionLanding, '/dbvf/start_precision_landing')
        self.servo_cli = self.create_client(DoSetServo, '/dbvf/do_set_servo')

        # Service servers
        self.create_service(
            StartMission, '/dbvf/start_mission', self._start_mission_cb)
        self.create_service(
            ResumeMission, '/dbvf/resume_mission', self._resume_mission_cb)
        self.create_service(
            AbortMission, '/dbvf/abort_mission', self._abort_mission_cb)

        # 10 Hz control loop
        self.create_timer(0.1, self._control_loop)
        self.get_logger().info('Mission sequencer node started')

    # -- Subscriber callbacks -------------------------------------------------

    def _vehicle_state_cb(self, msg):
        self.latest_vehicle_state = msg

    def _heartbeat_cb(self, msg):
        if msg.data:
            self._last_heartbeat_time = time.time()

    def _landing_state_cb(self, msg):
        self.fsm.set_landing_state(msg.data)

    # -- Service servers ------------------------------------------------------

    def _start_mission_cb(self, request, response):
        if self.fsm.state != MissionState.IDLE:
            response.success = False
            response.message = f'Already active: {self.fsm.state.value}'
            return response
        self.get_logger().info('Starting mission')
        self.fsm.start()
        response.success = True
        response.message = 'Mission started'
        return response

    def _resume_mission_cb(self, request, response):
        if self.fsm.state != MissionState.WAIT_FLAGGER:
            response.success = False
            response.message = f'Not in WAIT_FLAGGER: {self.fsm.state.value}'
            return response
        self.get_logger().info('Resuming mission from WAIT_FLAGGER')
        self.fsm.resume()
        response.success = True
        response.message = 'Resuming from WAIT_FLAGGER'
        return response

    def _abort_mission_cb(self, request, response):
        reason = request.reason or 'Manual abort'
        self.get_logger().warn(f'Abort requested: {reason}')
        self.fsm.abort(reason)
        self._call_set_mode('LAND')
        response.success = True
        response.message = f'Abort: {reason}'
        return response

    # -- Control loop ---------------------------------------------------------

    def _control_loop(self):
        if self.fsm.state == MissionState.IDLE:
            return

        vs = self.latest_vehicle_state
        if vs is None:
            return

        # Inject heartbeat status into vehicle state for FSM
        hb_timeout = self.mission_config['heartbeat_loss_timeout_s']
        vs.heartbeat_ok = (time.time() - self._last_heartbeat_time) < hb_timeout

        now = time.time()
        prev_state = self.fsm.state

        state, info = self.fsm.update(vs, now)

        # Log state transitions
        if state != prev_state:
            self.get_logger().info(
                f'{prev_state.value} -> {state.value} ({info["action"]})')

        # Execute entry actions
        for action in info.get('entry_actions', []):
            self._execute_action(action)

        # Publish state
        state_msg = String()
        state_msg.data = state.value
        self.state_pub.publish(state_msg)

        # Publish phase on change
        phase = get_mission_phase(state)
        if phase != self._prev_phase:
            phase_msg = String()
            phase_msg.data = phase
            self.phase_pub.publish(phase_msg)
            self._prev_phase = phase

    # -- Action executor ------------------------------------------------------

    def _execute_action(self, action):
        cfg = self.mission_config
        alt = self._transit_alt_m

        if action == 'set_mode_land':
            self._call_set_mode('LAND')
        elif action == 'set_mode_guided':
            self._call_set_mode('GUIDED')
        elif action == 'arm':
            self._call_arm(True)
        elif action == 'send_guided_position_h':
            self._call_guided_position(cfg['home_lat'], cfg['home_lon'], alt)
        elif action == 'send_guided_position_l':
            self._call_guided_position(cfg['landing_lat'], cfg['landing_lon'], alt)
        elif action == 'send_guided_position_l_alt':
            self._call_guided_position(cfg['landing_lat'], cfg['landing_lon'], alt)
        elif action == 'send_guided_position_drop':
            if cfg['drop_target'] == 'F2':
                self._call_guided_position(cfg['f2_lat'], cfg['f2_lon'], alt)
            else:
                self._call_guided_position(cfg['f1_lat'], cfg['f1_lon'], alt)
        elif action == 'send_guided_position_wa':
            self._call_guided_position(cfg['wa_lat'], cfg['wa_lon'], alt)
        elif action == 'send_guided_position_wa_alt':
            self._call_guided_position(cfg['wa_lat'], cfg['wa_lon'], alt)
        elif action == 'start_precision_landing':
            self._call_start_precision_landing(cfg['wa_lat'], cfg['wa_lon'])
        elif action == 'servo_release':
            self._call_set_servo(
                cfg['drop_servo_number'], cfg['drop_servo_pwm_release'])

    # -- Service call helpers -------------------------------------------------

    def _call_set_mode(self, mode):
        if not self.set_mode_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('set_mode service unavailable')
            return
        req = SetMode.Request()
        req.mode = mode
        future = self.set_mode_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Mode: {f.result().message}') if f.result() else None)

    def _call_arm(self, arm):
        if not self.arm_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('arm_motors service unavailable')
            return
        req = ArmMotors.Request()
        req.arm = arm
        future = self.arm_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Arm: {f.result().message}') if f.result() else None)

    def _call_guided_position(self, lat, lon, alt):
        if not self.guided_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('send_guided_position service unavailable')
            return
        req = SendGuidedPosition.Request()
        req.lat = lat
        req.lon = lon
        req.alt = alt
        self.guided_cli.call_async(req)

    def _call_start_precision_landing(self, lat, lon):
        if not self.precision_land_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('start_precision_landing service unavailable')
            return
        req = StartPrecisionLanding.Request()
        req.target_lat = lat
        req.target_lon = lon
        future = self.precision_land_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Precision landing: {f.result().message}') if f.result() else None)

    def _call_set_servo(self, servo_number, pwm):
        if not self.servo_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('do_set_servo service unavailable')
            return
        req = DoSetServo.Request()
        req.servo_number = servo_number
        req.pwm = pwm
        future = self.servo_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Servo: {f.result().message}') if f.result() else None)


def main():
    rclpy.init()
    node = MissionSequencerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 2: Create the executable script**

Create `src/dbvf_autonomy/scripts/mission_sequencer_node`:

```python
#!/usr/bin/env python3
from dbvf_autonomy.mission_sequencer_node import main
main()
```

Make it executable:
```bash
chmod +x src/dbvf_autonomy/scripts/mission_sequencer_node
```

- [ ] **Step 3: Register in CMakeLists.txt**

In `src/dbvf_autonomy/CMakeLists.txt`, add `scripts/mission_sequencer_node` to the `install(PROGRAMS ...)` block:

```cmake
install(PROGRAMS
  scripts/mavlink_interface_node
  scripts/tag_detector_adapter_node
  scripts/precision_landing_node
  scripts/tag_visualizer_node
  scripts/mission_sequencer_node
  DESTINATION lib/${PROJECT_NAME}
)
```

- [ ] **Step 4: Build and verify**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select dbvf_msgs dbvf_autonomy
```
Expected: Build succeeds.

- [ ] **Step 5: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py \
        src/dbvf_autonomy/scripts/mission_sequencer_node \
        src/dbvf_autonomy/CMakeLists.txt
git commit -m "feat: add mission_sequencer_node ROS2 node"
```

---

## Task 8: Mission Config YAML and Launch File

**Files:**
- Create: `src/dbvf_autonomy/config/mission_params.yaml`
- Create: `src/dbvf_autonomy/launch/mission_sim.launch.py`

- [ ] **Step 1: Create mission_params.yaml**

Create `src/dbvf_autonomy/config/mission_params.yaml`:

```yaml
mission_sequencer:
  ros__parameters:
    # Waypoint GPS coordinates (simulation defaults)
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
    mission_timeout_s: 540.0
    heartbeat_loss_timeout_s: 5.0
    service_call_timeout_s: 5.0
    guided_resend_interval_s: 1.0

    # Altitude source
    prefer_rangefinder: true
```

- [ ] **Step 2: Create mission_sim.launch.py**

Create `src/dbvf_autonomy/launch/mission_sim.launch.py`:

```python
"""Launch full mission stack for Gazebo simulation.

Prerequisites: iris_runway.launch.py must be running separately.
Includes all nodes from precision_landing_sim.launch.py plus the mission sequencer.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('dbvf_autonomy')
    sim_config = os.path.join(pkg_dir, 'config', 'sim_params.yaml')
    mission_config = os.path.join(pkg_dir, 'config', 'mission_params.yaml')

    return LaunchDescription([
        # AprilTag detector
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            remappings=[
                ('image_rect', '/camera/image'),
                ('camera_info', '/camera/camera_info'),
                ('detections', '/apriltag/detections'),
            ],
            parameters=[{
                'family': '36h11',
                'size': 0.6,
                'tag.ids': [0, 1],
                'tag.sizes': [0.6, 0.15],
            }],
        ),

        # Tag detector adapter
        Node(
            package='dbvf_autonomy',
            executable='tag_detector_adapter_node',
            name='tag_detector_adapter',
            parameters=[sim_config],
        ),

        # MAVLink interface
        Node(
            package='dbvf_autonomy',
            executable='mavlink_interface_node',
            name='mavlink_interface',
            parameters=[sim_config],
        ),

        # Precision landing state machine
        Node(
            package='dbvf_autonomy',
            executable='precision_landing_node',
            name='precision_landing',
            parameters=[sim_config],
        ),

        # Tag detection visualizer
        Node(
            package='dbvf_autonomy',
            executable='tag_visualizer_node',
            name='tag_visualizer',
        ),

        # Mission sequencer
        Node(
            package='dbvf_autonomy',
            executable='mission_sequencer_node',
            name='mission_sequencer',
            parameters=[mission_config],
        ),
    ])
```

- [ ] **Step 3: Build and verify**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon build --packages-select dbvf_autonomy
```
Expected: Build succeeds (config and launch dirs are already installed by CMakeLists.txt `install(DIRECTORY launch config ...)`).

- [ ] **Step 4: Commit**

```bash
git add src/dbvf_autonomy/config/mission_params.yaml \
        src/dbvf_autonomy/launch/mission_sim.launch.py
git commit -m "feat: add mission config and launch file for sim"
```

---

## Task 9: Run Full Test Suite

**Files:**
- No new files. Verification step.

- [ ] **Step 1: Build both packages**

Run:
```bash
source /opt/ros/humble/setup.bash && colcon build --packages-select dbvf_msgs dbvf_autonomy
```
Expected: Both packages build successfully.

- [ ] **Step 2: Run all tests**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select dbvf_autonomy && colcon test-result --verbose
```
Expected: All tests pass — existing tests (test_mavlink, test_debounce, test_angles, test_tags, test_fsm, test_pid, test_pose, test_tag_pose_est, test_descent_clamp) plus new tests (test_mission_helpers, test_mission_config, test_mission_fsm).

- [ ] **Step 3: Verify new services are available**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 interface list | grep dbvf_msgs/srv
```
Expected output includes:
```
dbvf_msgs/srv/AbortMission
dbvf_msgs/srv/ArmMotors
dbvf_msgs/srv/DoSetServo
dbvf_msgs/srv/ResumeMission
dbvf_msgs/srv/SendGuidedPosition
dbvf_msgs/srv/SendGuidedVelocity
dbvf_msgs/srv/SetMode
dbvf_msgs/srv/StartMission
dbvf_msgs/srv/StartPrecisionLanding
```

---

## Task 10: Integration Test Procedure

**Files:**
- No code files. Manual test script for simulation.

- [ ] **Step 1: Run integration test**

```bash
# Terminal 1: Gazebo + ArduPilot SITL
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true

# Terminal 2: Full mission stack
ros2 launch dbvf_autonomy mission_sim.launch.py

# Terminal 3: MAVProxy for initial arming
mavproxy.py --master udpin:0.0.0.0:14550 --console
# In MAVProxy: mode guided -> arm throttle

# Terminal 4: Monitor
ros2 topic echo /dbvf/mission_state
ros2 topic echo /dbvf/mission_phase

# Terminal 5: Trigger mission
ros2 service call /dbvf/start_mission dbvf_msgs/srv/StartMission "{}"

# Observe FM-1: PREFLIGHT_CHECK -> TAKEOFF_H -> TRANSIT_H_TO_L -> LAND_L -> WAIT_FLAGGER

# When ready (simulating flagger):
ros2 service call /dbvf/resume_mission dbvf_msgs/srv/ResumeMission "{}"

# Observe FM-2: TAKEOFF_L -> TRANSIT_TO_DROP -> DROP_PAYLOAD
# Observe FM-3: TRANSIT_TO_WA -> LAND_WA -> TAKEOFF_WA -> TRANSIT_TO_DROP_2 -> DROP_PAYLOAD_2
# Observe RTH: TRANSIT_TO_H -> LAND_H -> COMPLETE

# To abort at any time:
ros2 service call /dbvf/abort_mission dbvf_msgs/srv/AbortMission "{reason: 'test abort'}"
```

---

## Self-Review Checklist

### Spec Coverage

| Spec Section | Task(s) | Covered? |
|---|---|---|
| 17 states + ABORT | Task 5, 6 | Yes |
| StartMission, ResumeMission, AbortMission services | Task 1, 7 | Yes |
| DoSetServo service | Task 1, 2 | Yes |
| State transition logic (10Hz timer) | Task 5, 7 | Yes |
| WAIT_FLAGGER blocks until resume | Task 5, 6 | Yes |
| Position tolerance via haversine | Task 3, 5 | Yes |
| Altitude check with rangefinder fallback | Task 5 | Yes |
| Landed detection (disarmed OR low+slow) | Task 5 | Yes |
| LAND_WA delegates to precision landing | Task 5, 6 | Yes |
| LAND_WA aborts on ABORT_LAND | Task 6 | Yes |
| Mission timeout (skip WAIT_FLAGGER) | Task 6 | Yes |
| Heartbeat loss abort | Task 6 | Yes |
| mission_params.yaml | Task 8 | Yes |
| mission_sim.launch.py | Task 8 | Yes |
| /dbvf/mission_state topic | Task 7 | Yes |
| /dbvf/mission_phase topic (on change) | Task 7 | Yes |
| Guided position resend every 1.0s | Task 5 | Yes |
| Drop settle time | Task 5, 6 | Yes |
| GUI interface contract | Task 7 | Yes |

### Placeholder Scan

No TBD, TODO, "implement later", or "similar to Task N" found.

### Type Consistency

- `MissionState` enum used consistently across state machine, tests, and node
- `get_mission_phase()` returns strings matching spec: 'FM1', 'FM2', 'FM3', 'RTH', 'COMPLETE', 'ABORT'
- `update()` returns `(state, info_dict)` with consistent keys: `action`, `entry_actions`, optional `reason`
- Config keys match between `DEFAULT_MISSION_CONFIG`, `mission_params.yaml`, and node parameter declarations
- Service names: `/dbvf/start_mission`, `/dbvf/resume_mission`, `/dbvf/abort_mission`, `/dbvf/do_set_servo`
