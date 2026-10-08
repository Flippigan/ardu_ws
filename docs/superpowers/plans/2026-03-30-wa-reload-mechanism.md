# WA Reload Mechanism & Tag Hierarchy Update — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the single LAND_WA state with a 4-state reload sequence (descend, drop old payload via Arduino servo, reset servo, final land) and update the tag hierarchy from 2-tier (0.6m/0.15m) to 2-tier (0.15m/0.10m) with new IDs.

**Architecture:** The mission state machine gains 3 net new states (LAND_WA splits into 4). A new `arduino_interface_node` owns the USB serial connection to an Arduino servo controller. The mission sequencer dispatches `arduino_servo_release` and `arduino_servo_pickup` entry actions to the Arduino node. Config-only changes update tag IDs and precision landing hold time. No changes to precision_landing_node.py code.

**Tech Stack:** Python 3, ROS2 Humble, pyserial, pytest, ament_cmake_python

---

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py` | Modify | Add pickup config defaults + validation |
| `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py` | Modify | Replace LAND_WA with 4 new WA states |
| `src/dbvf_autonomy/dbvf_autonomy/arduino_interface_node.py` | **Create** | Serial interface to Arduino servo controller |
| `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py` | Modify | Arduino servo client + new entry actions |
| `src/dbvf_autonomy/config/sim_params.yaml` | Modify | Tag IDs, hold time, Arduino params |
| `src/dbvf_autonomy/config/mission_params.yaml` | Modify | Pickup servo params |
| `src/dbvf_autonomy/launch/mission_sim.launch.py` | Modify | Add Arduino node, update apriltag config |
| `src/dbvf_autonomy/scripts/arduino_interface_node` | **Create** | Entry point script |
| `src/dbvf_autonomy/CMakeLists.txt` | Modify | Register new script + test |
| `src/dbvf_autonomy/test/test_mission_config.py` | Modify | Validate new pickup params |
| `src/dbvf_autonomy/test/test_mission_state_machine.py` | Modify | Replace LAND_WA tests with 4-state WA tests |
| `src/dbvf_autonomy/test/test_arduino_interface.py` | **Create** | Serial protocol pure function tests |

---

### Task 1: Add pickup config defaults and validation to mission_helpers.py

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py`
- Test: `src/dbvf_autonomy/test/test_mission_config.py`

- [ ] **Step 1: Write failing tests for new pickup config validation**

Add these tests to the end of `src/dbvf_autonomy/test/test_mission_config.py`:

```python
def test_negative_pickup_settle_time():
    cfg = _make_config(pickup_settle_time_s=-1.0)
    errors = validate_mission_config(cfg)
    assert any('pickup_settle_time_s' in e for e in errors)


def test_zero_pickup_settle_time():
    cfg = _make_config(pickup_settle_time_s=0.0)
    errors = validate_mission_config(cfg)
    assert any('pickup_settle_time_s' in e for e in errors)


def test_valid_config_with_pickup_params():
    cfg = _make_config(
        pickup_servo_number=1,
        pickup_servo_pwm_release=1100,
        pickup_servo_pwm_pickup=1500,
        pickup_settle_time_s=2.0,
    )
    errors = validate_mission_config(cfg)
    assert errors == []
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_mission_config.py -v`
Expected: `test_negative_pickup_settle_time` and `test_zero_pickup_settle_time` FAIL (pickup_settle_time_s not in _REQUIRED_POSITIVE so no validation error generated). `test_valid_config_with_pickup_params` PASS (extra keys don't cause errors).

- [ ] **Step 3: Add pickup defaults and validation to mission_helpers.py**

In `src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py`:

Add `'pickup_settle_time_s'` to the `_REQUIRED_POSITIVE` list:

```python
_REQUIRED_POSITIVE = [
    'transit_altitude_ft',
    'position_tolerance_m',
    'takeoff_complete_alt_ft',
    'mission_timeout_s',
    'heartbeat_loss_timeout_s',
    'service_call_timeout_s',
    'guided_resend_interval_s',
    'drop_settle_time_s',
    'pickup_settle_time_s',
]
```

Add pickup params to `DEFAULT_MISSION_CONFIG` (after the existing `drop_settle_time_s` entry):

```python
    # WA reload servo (Arduino)
    'pickup_servo_number': 1,
    'pickup_servo_pwm_release': 1100,
    'pickup_servo_pwm_pickup': 1500,
    'pickup_settle_time_s': 2.0,
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_mission_config.py -v`
Expected: All tests PASS (including original 5 + 3 new).

- [ ] **Step 5: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py src/dbvf_autonomy/test/test_mission_config.py
git commit -m "feat: add pickup servo config defaults and validation"
```

---

### Task 2: Replace LAND_WA with 4 new WA states in mission_state_machine.py

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py`
- Modify: `src/dbvf_autonomy/test/test_mission_state_machine.py`

- [ ] **Step 1: Write updated and new tests for the WA state sequence**

Replace the entire FM-3 test section in `src/dbvf_autonomy/test/test_mission_state_machine.py`. The section starts at the comment `# FM-3: TRANSIT_TO_WA -> LAND_WA -> ...` and ends before `# Return Home: ...`.

Remove the old `_to_land_wa`, `test_land_wa_waiting`, `test_land_wa_success`, `test_land_wa_abort`, and `_to_takeoff_wa` functions. Replace with:

```python
# ---------------------------------------------------------------------------
# FM-3: TRANSIT_TO_WA -> LAND_WA_DESCEND -> WA_DROP_OLD_PAYLOAD ->
#        WA_SERVO_RESET -> LAND_WA_FINAL -> TAKEOFF_WA -> ...
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
    assert state == MissionState.LAND_WA_DESCEND
    assert 'start_precision_landing' in info['entry_actions']


# -- LAND_WA_DESCEND --------------------------------------------------------

def _to_land_wa_descend(sm):
    _to_transit_to_wa(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=11.0, armed=True, mode='GUIDED')
    sm.update(vs, 9.0)
    assert sm.state == MissionState.LAND_WA_DESCEND


def test_land_wa_descend_waiting():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_descend(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=5.0, armed=True, mode='LAND')
    state, info = sm.update(vs, 10.0)
    assert state == MissionState.LAND_WA_DESCEND
    assert info['action'] == 'precision_landing'


def test_land_wa_descend_to_drop():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_descend(sm)
    sm.set_landing_state('DESCEND_HOLD')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 10.0)
    assert state == MissionState.WA_DROP_OLD_PAYLOAD
    assert info['action'] == 'descend_hold_reached'
    assert 'arduino_servo_release' in info['entry_actions']


def test_land_wa_descend_abort():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_descend(sm)
    sm.set_landing_state('ABORT_LAND')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=1.0, armed=True)
    state, info = sm.update(vs, 10.0)
    assert state == MissionState.ABORT
    assert 'Precision landing failed' in info['reason']


# -- WA_DROP_OLD_PAYLOAD -----------------------------------------------------

def _to_wa_drop_old_payload(sm):
    _to_land_wa_descend(sm)
    sm.set_landing_state('DESCEND_HOLD')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    sm.update(vs, 10.0)
    assert sm.state == MissionState.WA_DROP_OLD_PAYLOAD


def test_wa_drop_old_payload_waiting():
    sm = MissionStateMachine(_make_config())
    _to_wa_drop_old_payload(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 11.0)  # 1.0s < 2.0s settle
    assert state == MissionState.WA_DROP_OLD_PAYLOAD
    assert info['action'] == 'dropping_old_payload'


def test_wa_drop_old_payload_complete():
    sm = MissionStateMachine(_make_config())
    _to_wa_drop_old_payload(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 12.1)  # 2.1s > 2.0s settle
    assert state == MissionState.WA_SERVO_RESET
    assert info['action'] == 'drop_old_complete'
    assert 'arduino_servo_pickup' in info['entry_actions']


# -- WA_SERVO_RESET ----------------------------------------------------------

def _to_wa_servo_reset(sm):
    _to_wa_drop_old_payload(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    sm.update(vs, 12.1)  # settle elapsed
    assert sm.state == MissionState.WA_SERVO_RESET


def test_wa_servo_reset_waiting():
    sm = MissionStateMachine(_make_config())
    _to_wa_servo_reset(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 13.0)  # 0.9s < 2.0s settle
    assert state == MissionState.WA_SERVO_RESET
    assert info['action'] == 'resetting_servo'


def test_wa_servo_reset_complete():
    sm = MissionStateMachine(_make_config())
    _to_wa_servo_reset(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 14.2)  # 2.1s > 2.0s settle
    assert state == MissionState.LAND_WA_FINAL
    assert info['action'] == 'servo_reset_complete'


# -- LAND_WA_FINAL -----------------------------------------------------------

def _to_land_wa_final(sm):
    _to_wa_servo_reset(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    sm.update(vs, 14.2)  # settle elapsed
    assert sm.state == MissionState.LAND_WA_FINAL


def test_land_wa_final_waiting():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_final(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.3, armed=True)
    state, info = sm.update(vs, 15.0)
    assert state == MissionState.LAND_WA_FINAL
    assert info['action'] == 'final_descent'


def test_land_wa_final_success():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_final(sm)
    sm.set_landing_state('LANDED')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.1, armed=False)
    state, info = sm.update(vs, 15.0)
    assert state == MissionState.TAKEOFF_WA
    assert info['action'] == 'landed_wa'


def test_land_wa_final_abort():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_final(sm)
    sm.set_landing_state('ABORT_LAND')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.5, armed=True)
    state, info = sm.update(vs, 15.0)
    assert state == MissionState.ABORT
    assert 'Precision landing failed' in info['reason']


# -- TAKEOFF_WA (updated helper) ---------------------------------------------

def _to_takeoff_wa(sm):
    _to_land_wa_final(sm)
    sm.set_landing_state('LANDED')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.1, armed=False)
    sm.update(vs, 15.0)
    assert sm.state == MissionState.TAKEOFF_WA
```

Also update `test_phase_fm3` at the bottom of the file — replace `MissionState.LAND_WA` with `MissionState.LAND_WA_DESCEND`:

```python
def test_phase_fm3():
    from dbvf_autonomy.mission_state_machine import get_mission_phase
    assert get_mission_phase(MissionState.LAND_WA_DESCEND) == 'FM3'
    assert get_mission_phase(MissionState.WA_DROP_OLD_PAYLOAD) == 'FM3'
    assert get_mission_phase(MissionState.WA_SERVO_RESET) == 'FM3'
    assert get_mission_phase(MissionState.LAND_WA_FINAL) == 'FM3'
    assert get_mission_phase(MissionState.DROP_PAYLOAD_2) == 'FM3'
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_mission_state_machine.py -v`
Expected: Multiple FAIL — `MissionState.LAND_WA_DESCEND` doesn't exist yet.

- [ ] **Step 3: Implement the 4 new WA states in mission_state_machine.py**

In `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py`:

**3a. Update the `MissionState` enum** — replace `LAND_WA = 'LAND_WA'` with:

```python
    LAND_WA_DESCEND = 'LAND_WA_DESCEND'
    WA_DROP_OLD_PAYLOAD = 'WA_DROP_OLD_PAYLOAD'
    WA_SERVO_RESET = 'WA_SERVO_RESET'
    LAND_WA_FINAL = 'LAND_WA_FINAL'
```

**3b. Update `_STATE_TO_PHASE`** — replace `MissionState.LAND_WA: 'FM3',` with:

```python
    MissionState.LAND_WA_DESCEND: 'FM3',
    MissionState.WA_DROP_OLD_PAYLOAD: 'FM3',
    MissionState.WA_SERVO_RESET: 'FM3',
    MissionState.LAND_WA_FINAL: 'FM3',
```

**3c. Update `__init__`** — add after `self._drop_start_time = None`:

```python
        # WA reload timing
        self._wa_drop_start_time = None
```

**3d. Update `start()`** — add after `self._drop_start_time = None`:

```python
        self._wa_drop_start_time = None
```

**3e. Update `_transit_to_wa`** — change `MissionState.LAND_WA` to `MissionState.LAND_WA_DESCEND`:

```python
    def _transit_to_wa(self, vs, t):
        wa_lat = self.config['wa_lat']
        wa_lon = self.config['wa_lon']
        dist = haversine_distance_m(vs.lat, vs.lon, wa_lat, wa_lon)
        if dist < self._tolerance:
            self.state = MissionState.LAND_WA_DESCEND
            self._landing_state = None
            return self.state, {'action': 'arrived_wa',
                                'entry_actions': ['start_precision_landing']}
        entry = []
        if t - self._last_guided_send_time >= self.config['guided_resend_interval_s']:
            entry.append('send_guided_position_wa')
            self._last_guided_send_time = t
        return self.state, {'action': 'transiting', 'entry_actions': entry}
```

**3f. Replace `_land_wa` method** with 4 new methods:

```python
    def _land_wa_descend(self, vs, t):
        if self._landing_state == 'DESCEND_HOLD':
            self.state = MissionState.WA_DROP_OLD_PAYLOAD
            self._wa_drop_start_time = t
            return self.state, {'action': 'descend_hold_reached',
                                'entry_actions': ['arduino_servo_release']}
        if self._landing_state == 'ABORT_LAND':
            self.state = MissionState.ABORT
            self.abort_reason = 'Precision landing failed at WA'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': ['set_mode_land']}
        return self.state, {'action': 'precision_landing', 'entry_actions': []}

    def _wa_drop_old_payload(self, vs, t):
        elapsed = t - self._wa_drop_start_time
        if elapsed >= self.config['pickup_settle_time_s']:
            self._wa_drop_start_time = t
            self.state = MissionState.WA_SERVO_RESET
            return self.state, {'action': 'drop_old_complete',
                                'entry_actions': ['arduino_servo_pickup']}
        return self.state, {'action': 'dropping_old_payload', 'entry_actions': []}

    def _wa_servo_reset(self, vs, t):
        elapsed = t - self._wa_drop_start_time
        if elapsed >= self.config['pickup_settle_time_s']:
            self.state = MissionState.LAND_WA_FINAL
            return self.state, {'action': 'servo_reset_complete',
                                'entry_actions': []}
        return self.state, {'action': 'resetting_servo', 'entry_actions': []}

    def _land_wa_final(self, vs, t):
        if self._landing_state == 'LANDED':
            self.state = MissionState.TAKEOFF_WA
            return self.state, {'action': 'landed_wa',
                                'entry_actions': ['arm', 'set_mode_guided',
                                                  'takeoff']}
        if self._landing_state == 'ABORT_LAND':
            self.state = MissionState.ABORT
            self.abort_reason = 'Precision landing failed at WA'
            return self.state, {'action': 'aborted', 'reason': self.abort_reason,
                                'entry_actions': ['set_mode_land']}
        return self.state, {'action': 'final_descent', 'entry_actions': []}
```

**3g. Update the `_handlers` dispatch table** — replace `MissionState.LAND_WA: _land_wa,` with:

```python
        MissionState.LAND_WA_DESCEND: _land_wa_descend,
        MissionState.WA_DROP_OLD_PAYLOAD: _wa_drop_old_payload,
        MissionState.WA_SERVO_RESET: _wa_servo_reset,
        MissionState.LAND_WA_FINAL: _land_wa_final,
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_mission_state_machine.py -v`
Expected: All tests PASS.

- [ ] **Step 5: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py src/dbvf_autonomy/test/test_mission_state_machine.py
git commit -m "feat: replace LAND_WA with 4-state WA reload sequence

LAND_WA_DESCEND waits for DESCEND_HOLD, WA_DROP_OLD_PAYLOAD and
WA_SERVO_RESET use pickup_settle_time_s timing, LAND_WA_FINAL waits
for LANDED. All map to FM3 phase."
```

---

### Task 3: Create arduino_interface_node.py with tests

**Files:**
- Create: `src/dbvf_autonomy/dbvf_autonomy/arduino_interface_node.py`
- Create: `src/dbvf_autonomy/test/test_arduino_interface.py`
- Create: `src/dbvf_autonomy/scripts/arduino_interface_node`
- Modify: `src/dbvf_autonomy/CMakeLists.txt`

- [ ] **Step 1: Write tests for the serial protocol pure functions**

Create `src/dbvf_autonomy/test/test_arduino_interface.py`:

```python
"""Tests for Arduino interface — serial protocol pure functions."""
from dbvf_autonomy.arduino_interface_node import format_servo_command, parse_servo_response


def test_format_servo_command_basic():
    assert format_servo_command(1, 1100) == 'S1:1100\n'


def test_format_servo_command_different_values():
    assert format_servo_command(3, 1500) == 'S3:1500\n'


def test_parse_response_ok():
    success, msg = parse_servo_response('OK\n')
    assert success is True
    assert msg == 'OK'


def test_parse_response_ok_no_newline():
    success, msg = parse_servo_response('OK')
    assert success is True
    assert msg == 'OK'


def test_parse_response_error():
    success, msg = parse_servo_response('ERR:invalid servo\n')
    assert success is False
    assert msg == 'invalid servo'


def test_parse_response_unexpected():
    success, msg = parse_servo_response('WHAT\n')
    assert success is False
    assert 'Unexpected' in msg
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_arduino_interface.py -v`
Expected: ImportError — `arduino_interface_node` module doesn't exist.

- [ ] **Step 3: Create the arduino_interface_node module**

Create `src/dbvf_autonomy/dbvf_autonomy/arduino_interface_node.py`:

```python
"""Arduino serial interface node — owns USB serial to Arduino servo controller."""
import serial

import rclpy
from rclpy.node import Node

from dbvf_msgs.srv import DoSetServo


def format_servo_command(servo_number, pwm):
    """Format a servo command string for the Arduino serial protocol.

    Protocol: S<servo_number>:<pwm>\n
    """
    return f'S{servo_number}:{pwm}\n'


def parse_servo_response(response):
    """Parse an Arduino serial response. Returns (success, message)."""
    response = response.strip()
    if response == 'OK':
        return True, 'OK'
    if response.startswith('ERR:'):
        return False, response[4:]
    return False, f'Unexpected response: {response}'


class ArduinoInterfaceNode(Node):
    def __init__(self):
        super().__init__('arduino_interface')

        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout_s', 1.0)

        self._serial = None
        self._connect()

        self.create_service(
            DoSetServo, '/dbvf/arduino/set_servo', self._set_servo_cb)
        self.get_logger().info('Arduino interface node started')

    def _connect(self):
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        timeout = self.get_parameter('serial_timeout_s').value
        try:
            self._serial = serial.Serial(port, baud, timeout=timeout)
            self.get_logger().info(f'Serial connected: {port} @ {baud}')
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial connection failed: {e}')
            self._serial = None

    def _set_servo_cb(self, request, response):
        if self._serial is None:
            self._connect()
        if self._serial is None:
            response.success = False
            response.message = 'Serial port unavailable'
            return response

        cmd = format_servo_command(request.servo_number, request.pwm)
        try:
            self._serial.write(cmd.encode('ascii'))
            raw = self._serial.readline().decode('ascii')
            if not raw:
                response.success = False
                response.message = 'Serial timeout'
                return response
            success, msg = parse_servo_response(raw)
            response.success = success
            response.message = msg
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial error: {e}')
            self._serial = None
            response.success = False
            response.message = f'Serial error: {e}'
        return response


def main():
    rclpy.init()
    node = ArduinoInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node._serial:
            node._serial.close()
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_arduino_interface.py -v`
Expected: All 6 tests PASS.

- [ ] **Step 5: Create entry point script and register in CMakeLists.txt**

Create `src/dbvf_autonomy/scripts/arduino_interface_node`:

```python
#!/usr/bin/env python3
from dbvf_autonomy.arduino_interface_node import main
main()
```

Make it executable: `chmod +x src/dbvf_autonomy/scripts/arduino_interface_node`

In `src/dbvf_autonomy/CMakeLists.txt`, add the script to the install block (after `scripts/mission_sequencer_node`):

```cmake
install(PROGRAMS
  scripts/mavlink_interface_node
  scripts/tag_detector_adapter_node
  scripts/precision_landing_node
  scripts/tag_visualizer_node
  scripts/mission_sequencer_node
  scripts/arduino_interface_node
  DESTINATION lib/${PROJECT_NAME}
)
```

Add the test registration (after the existing `test_rc_trigger` line):

```cmake
  ament_add_pytest_test(test_arduino test/test_arduino_interface.py)
```

- [ ] **Step 6: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/arduino_interface_node.py \
        src/dbvf_autonomy/test/test_arduino_interface.py \
        src/dbvf_autonomy/scripts/arduino_interface_node \
        src/dbvf_autonomy/CMakeLists.txt
git commit -m "feat: add arduino_interface_node for serial servo control

Exposes /dbvf/arduino/set_servo (DoSetServo). Serial protocol:
S<num>:<pwm>\n -> OK\n or ERR:<msg>\n. Gracefully returns
success=False if serial port unavailable."
```

---

### Task 4: Update mission_sequencer_node.py for Arduino servo actions

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py`

- [ ] **Step 1: Add new params, Arduino service client, and new entry actions**

In `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py`:

**1a. Add new param defaults** to the `param_defaults` dict (after `'drop_settle_time_s': 2.0,`):

```python
            'pickup_servo_number': 1,
            'pickup_servo_pwm_release': 1100,
            'pickup_servo_pwm_pickup': 1500,
            'pickup_settle_time_s': 2.0,
```

**1b. Add Arduino service client** after the existing `self.takeoff_cli` line:

```python
        self.arduino_servo_cli = self.create_client(
            DoSetServo, '/dbvf/arduino/set_servo')
```

**1c. Add new action handlers** in `_execute_action()`, after the `'servo_release'` elif block:

```python
        elif action == 'arduino_servo_release':
            self._call_arduino_servo(
                cfg['pickup_servo_number'], cfg['pickup_servo_pwm_release'])
        elif action == 'arduino_servo_pickup':
            self._call_arduino_servo(
                cfg['pickup_servo_number'], cfg['pickup_servo_pwm_pickup'])
```

**1d. Add `_call_arduino_servo` helper** after the existing `_call_set_servo` method:

```python
    def _call_arduino_servo(self, servo_number, pwm):
        if not self.arduino_servo_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('arduino/set_servo service unavailable')
            return
        req = DoSetServo.Request()
        req.servo_number = servo_number
        req.pwm = pwm
        future = self.arduino_servo_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Arduino servo: {f.result().message}') if f.result() else None)
```

- [ ] **Step 2: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py
git commit -m "feat: add Arduino servo actions to mission sequencer

New service client for /dbvf/arduino/set_servo. Handles
arduino_servo_release and arduino_servo_pickup entry actions.
Adds pickup_servo_* config params."
```

---

### Task 5: Update config files

**Files:**
- Modify: `src/dbvf_autonomy/config/sim_params.yaml`
- Modify: `src/dbvf_autonomy/config/mission_params.yaml`

- [ ] **Step 1: Update sim_params.yaml**

In `src/dbvf_autonomy/config/sim_params.yaml`:

Change tag_detector_adapter params:
```yaml
tag_detector_adapter:
  ros__parameters:
    primary_tag_id: 1
    secondary_tag_id: 2
    primary_tag_size: 0.15
    secondary_tag_size: 0.10
```
(Keep all other tag_detector_adapter params unchanged.)

Change precision_landing params — update `secondary_tag_id` and `hold_stabilize_time`:
```yaml
    secondary_tag_id: 2
    hold_stabilize_time: 6.0
```
(Keep all other precision_landing params unchanged.)

Add arduino_interface section at the end of the file:
```yaml

arduino_interface:
  ros__parameters:
    serial_port: "/dev/ttyACM0"
    baud_rate: 115200
    serial_timeout_s: 1.0
```

- [ ] **Step 2: Update mission_params.yaml**

In `src/dbvf_autonomy/config/mission_params.yaml`, add pickup servo params after the `drop_settle_time_s: 2.0` line:

```yaml
    # ── WA Reload Servo (Arduino) ─────────────────────────────────
    pickup_servo_number: 1
    pickup_servo_pwm_release: 1100
    pickup_servo_pwm_pickup: 1500
    pickup_settle_time_s: 2.0
```

- [ ] **Step 3: Commit**

```bash
git add src/dbvf_autonomy/config/sim_params.yaml src/dbvf_autonomy/config/mission_params.yaml
git commit -m "config: update tag hierarchy (ID 1/2), hold time 6s, add pickup servo params"
```

---

### Task 6: Update launch file

**Files:**
- Modify: `src/dbvf_autonomy/launch/mission_sim.launch.py`

- [ ] **Step 1: Add Arduino node and update apriltag config**

In `src/dbvf_autonomy/launch/mission_sim.launch.py`:

Update the apriltag_node parameters to reflect new tag IDs (1 and 2) and sizes:
```python
            parameters=[{
                'family': '36h11',
                'size': 0.15,
                'tag.ids': [1, 2],
                'tag.sizes': [0.15, 0.10],
            }],
```

Add the Arduino interface node before the mission sequencer node:
```python
        # Arduino serial interface
        Node(
            package='dbvf_autonomy',
            executable='arduino_interface_node',
            name='arduino_interface',
            parameters=[sim_config],
        ),
```

- [ ] **Step 2: Commit**

```bash
git add src/dbvf_autonomy/launch/mission_sim.launch.py
git commit -m "launch: add arduino_interface_node, update apriltag tag IDs to 1/2"
```

---

### Task 7: Build and run all tests

- [ ] **Step 1: Build**

```bash
cd /home/finn/Documents/ardu_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dbvf_msgs dbvf_autonomy
```

Expected: Build succeeds.

- [ ] **Step 2: Run all tests**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```

Expected: All tests pass (previous 151 - 3 removed LAND_WA tests + 11 new WA tests + 3 new config tests + 6 arduino tests = 168 tests).

- [ ] **Step 3: Fix any failing tests**

If any tests fail, diagnose and fix the root cause. Common issues:
- Import errors: ensure `arduino_interface_node.py` is in the `dbvf_autonomy` package directory
- State machine tests: verify timing values in test helpers match config defaults
- Config tests: ensure `DEFAULT_MISSION_CONFIG` has all required keys

- [ ] **Step 4: Final commit (if fixes needed)**

```bash
git add -A
git commit -m "fix: resolve test failures from WA reload implementation"
```
