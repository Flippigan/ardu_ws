# Arduino PCA9685 Payload Servo Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace Cube Orange servo output with Arduino + PCA9685 for all payload operations, unifying drop and pickup servos into a single rotary mechanism with 5 positions.

**Architecture:** A single PCA9685 servo (channel 0) controlled via Arduino serial bridge replaces both the Cube Orange drop servo and the separate pickup servo. The existing `arduino_interface_node` and `DoSetServo.srv` are reused unchanged. The mission state machine gains one new state (`WA_LOCK_PAYLOAD`, 21 total) and renames `WA_SERVO_RESET` to `WA_PICKUP_READY` for semantic clarity.

**Tech Stack:** Arduino (C++, Adafruit PWM Servo Driver library), Python (ROS2 Humble), pytest

**Spec:** `docs/superpowers/specs/2026-04-02-arduino-pca9685-payload-servo-design.md`

---

### Task 1: Arduino Sketch

**Files:**
- Create: `src/dbvf_autonomy/arduino/payload_servo_controller/payload_servo_controller.ino`

This is a standalone Arduino sketch with no ROS2 dependencies. No unit tests (hardware-only).

- [ ] **Step 1: Create the Arduino sketch**

```cpp
// payload_servo_controller.ino
// Serial-to-PCA9685 PWM bridge for payload servo mechanism.
// Protocol: S<channel>:<pwm_us>\n -> OK\n or ERR:<reason>\n
// Identical to existing arduino_interface_node protocol.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

static const unsigned long BAUD_RATE = 115200;
static const uint8_t MAX_CHANNEL = 15;

String inputBuffer = "";

void setup() {
  Serial.begin(BAUD_RATE);
  Wire.begin();
  pwm.begin();
  pwm.setPWMFreq(50);  // 50Hz for standard servos
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

  if (channel < 0 || channel > MAX_CHANNEL) {
    Serial.println("ERR:channel out of range");
    return;
  }

  if (pwm_us < 0 || pwm_us > 20000) {
    Serial.println("ERR:pwm out of range");
    return;
  }

  // Convert microseconds to PCA9685 ticks (12-bit, 20ms period)
  uint16_t ticks = (uint16_t)((long)pwm_us * 4096 / 20000);
  pwm.setPWM(channel, 0, ticks);

  Serial.println("OK");
}
```

- [ ] **Step 2: Commit**

```bash
git add src/dbvf_autonomy/arduino/payload_servo_controller/payload_servo_controller.ino
git commit -m "feat: add Arduino PCA9685 payload servo controller sketch"
```

---

### Task 2: Update Mission Helpers (Config + Validation)

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py`
- Modify: `src/dbvf_autonomy/test/test_mission_config.py`

Replace old `drop_servo_*` / `pickup_servo_*` / `pickup_settle_time_s` params with 6 new `payload_servo_*` params + `payload_settle_time_s` in `DEFAULT_MISSION_CONFIG` and `_REQUIRED_POSITIVE`. Update tests to match.

- [ ] **Step 1: Update test expectations in test_mission_config.py**

Replace `test_negative_pickup_settle_time`, `test_zero_pickup_settle_time`, and `test_valid_config_with_pickup_params`:

```python
def test_negative_payload_settle_time():
    cfg = _make_config(payload_settle_time_s=-1.0)
    errors = validate_mission_config(cfg)
    assert any('payload_settle_time_s' in e for e in errors)


def test_zero_payload_settle_time():
    cfg = _make_config(payload_settle_time_s=0.0)
    errors = validate_mission_config(cfg)
    assert any('payload_settle_time_s' in e for e in errors)


def test_valid_config_with_payload_params():
    cfg = _make_config(
        payload_servo_channel=0,
        payload_servo_pwm_hold=500,
        payload_servo_pwm_dispense=1000,
        payload_servo_pwm_drop=1500,
        payload_servo_pwm_pickup=2000,
        payload_servo_pwm_lock=2500,
        payload_settle_time_s=2.0,
    )
    errors = validate_mission_config(cfg)
    assert errors == []


def test_default_payload_servo_params_present():
    """DEFAULT_MISSION_CONFIG should include all payload_servo_* keys."""
    assert 'payload_servo_channel' in DEFAULT_MISSION_CONFIG
    assert 'payload_servo_pwm_hold' in DEFAULT_MISSION_CONFIG
    assert 'payload_servo_pwm_dispense' in DEFAULT_MISSION_CONFIG
    assert 'payload_servo_pwm_drop' in DEFAULT_MISSION_CONFIG
    assert 'payload_servo_pwm_pickup' in DEFAULT_MISSION_CONFIG
    assert 'payload_servo_pwm_lock' in DEFAULT_MISSION_CONFIG
    assert 'payload_settle_time_s' in DEFAULT_MISSION_CONFIG
```

Also remove the assertions about old keys: `pickup_servo_number`, `pickup_servo_pwm_release`, etc. should no longer appear in any test.

- [ ] **Step 2: Run tests to verify they fail**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_mission_config.py -v`
Expected: FAIL — new param names don't exist in DEFAULT_MISSION_CONFIG yet.

- [ ] **Step 3: Update DEFAULT_MISSION_CONFIG in mission_helpers.py**

Replace the old servo sections (lines 77-88):

```python
    # Payload servo (Arduino + PCA9685)
    'payload_servo_channel': 0,
    'payload_servo_pwm_hold': 0,
    'payload_servo_pwm_dispense': 0,
    'payload_servo_pwm_drop': 0,
    'payload_servo_pwm_pickup': 0,
    'payload_servo_pwm_lock': 0,
    'drop_settle_time_s': 2.0,
    'payload_settle_time_s': 2.0,
```

Remove these keys entirely:
- `drop_servo_number`
- `drop_servo_pwm_release`
- `drop_servo_pwm_hold`
- `pickup_servo_number`
- `pickup_servo_pwm_release`
- `pickup_servo_pwm_pickup`
- `pickup_settle_time_s`

- [ ] **Step 4: Update _REQUIRED_POSITIVE in mission_helpers.py**

Replace `'pickup_settle_time_s'` with `'payload_settle_time_s'` in the `_REQUIRED_POSITIVE` list (line 57).

- [ ] **Step 5: Run tests to verify they pass**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_mission_config.py -v`
Expected: All PASS.

- [ ] **Step 6: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py src/dbvf_autonomy/test/test_mission_config.py
git commit -m "refactor: replace drop/pickup servo params with unified payload_servo_* config"
```

---

### Task 3: Update Mission State Machine (States + Transitions)

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py`
- Modify: `src/dbvf_autonomy/test/test_mission_state_machine.py`

Rename `WA_SERVO_RESET` → `WA_PICKUP_READY`, add `WA_LOCK_PAYLOAD` state (21 total), change entry actions for DROP_PAYLOAD / DROP_PAYLOAD_2 / WA_DROP_OLD_PAYLOAD.

- [ ] **Step 1: Update test_mission_state_machine.py — rename all WA_SERVO_RESET references**

Replace every occurrence of `WA_SERVO_RESET` with `WA_PICKUP_READY` in the test file. This affects:
- `test_wa_drop_old_payload_complete` (line 398): assert `MissionState.WA_PICKUP_READY`
- `_to_wa_servo_reset` → rename to `_to_wa_pickup_ready` (line 405-409)
- `test_wa_servo_reset_waiting` → rename to `test_wa_pickup_ready_waiting` (line 412-418)
- `test_wa_servo_reset_complete` → rename to `test_wa_pickup_ready_complete` (line 421-427)
- `_to_land_wa_final` helper (line 432-436): call `_to_wa_pickup_ready(sm)`
- `test_phase_fm3` (line 633): `MissionState.WA_PICKUP_READY`

- [ ] **Step 2: Update test_mission_state_machine.py — change entry action assertions**

Change `servo_release` → `arduino_servo_dispense`:
- `test_transit_to_drop_arrived` (line 278): `assert 'arduino_servo_dispense' in info['entry_actions']`
- `test_transit_to_drop_2_arrived` (line 503): `assert 'arduino_servo_dispense' in info['entry_actions']`

Change `arduino_servo_release` → `arduino_servo_drop`:
- `test_land_wa_descend_to_drop` (line 361): `assert 'arduino_servo_drop' in info['entry_actions']`

- [ ] **Step 3: Update test_mission_state_machine.py — change LAND_WA_FINAL success transition**

`test_land_wa_final_success` (line 448-455): The LANDED transition now goes to `WA_LOCK_PAYLOAD` instead of `TAKEOFF_WA`:

```python
def test_land_wa_final_success():
    sm = MissionStateMachine(_make_config())
    _to_land_wa_final(sm)
    sm.set_landing_state('LANDED')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.1, armed=False)
    state, info = sm.update(vs, 15.0)
    assert state == MissionState.WA_LOCK_PAYLOAD
    assert info['action'] == 'landed_wa'
    assert 'arduino_servo_lock' in info['entry_actions']
```

- [ ] **Step 4: Add WA_LOCK_PAYLOAD tests**

Add after `test_land_wa_final_abort`:

```python
# -- WA_LOCK_PAYLOAD ---------------------------------------------------------

def _to_wa_lock_payload(sm):
    _to_land_wa_final(sm)
    sm.set_landing_state('LANDED')
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.1, armed=False)
    sm.update(vs, 15.0)
    assert sm.state == MissionState.WA_LOCK_PAYLOAD


def test_wa_lock_payload_waiting():
    sm = MissionStateMachine(_make_config())
    _to_wa_lock_payload(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.1, armed=False)
    state, info = sm.update(vs, 16.0)  # 1.0s < 2.0s settle
    assert state == MissionState.WA_LOCK_PAYLOAD
    assert info['action'] == 'locking_payload'


def test_wa_lock_payload_complete():
    sm = MissionStateMachine(_make_config())
    _to_wa_lock_payload(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.1, armed=False)
    state, info = sm.update(vs, 17.1)  # 2.1s > 2.0s settle
    assert state == MissionState.TAKEOFF_WA
    assert info['action'] == 'lock_complete'
    assert 'set_mode_guided' in info['entry_actions']
    assert 'arm' in info['entry_actions']
    assert 'takeoff' in info['entry_actions']


def test_phase_wa_lock_payload():
    from dbvf_autonomy.mission_state_machine import get_mission_phase
    assert get_mission_phase(MissionState.WA_LOCK_PAYLOAD) == 'FM3'
```

- [ ] **Step 5: Update _to_takeoff_wa helper to go through WA_LOCK_PAYLOAD**

```python
def _to_takeoff_wa(sm):
    _to_wa_lock_payload(sm)
    vs = MockVehicleState(lat=WA_LAT, lon=WA_LON, alt_rel=0.1, armed=False)
    sm.update(vs, 17.1)  # lock settle elapsed
    assert sm.state == MissionState.TAKEOFF_WA
```

- [ ] **Step 6: Run tests to verify they fail**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_mission_state_machine.py -v`
Expected: FAIL — `WA_PICKUP_READY` and `WA_LOCK_PAYLOAD` don't exist yet, entry actions still have old names.

- [ ] **Step 7: Update MissionState enum in mission_state_machine.py**

Replace `WA_SERVO_RESET = 'WA_SERVO_RESET'` (line 20) with `WA_PICKUP_READY = 'WA_PICKUP_READY'`.

Add `WA_LOCK_PAYLOAD = 'WA_LOCK_PAYLOAD'` between `LAND_WA_FINAL` and `TAKEOFF_WA` (after line 21, before line 22).

The enum should become:
```python
    WA_DROP_OLD_PAYLOAD = 'WA_DROP_OLD_PAYLOAD'
    WA_PICKUP_READY = 'WA_PICKUP_READY'
    LAND_WA_FINAL = 'LAND_WA_FINAL'
    WA_LOCK_PAYLOAD = 'WA_LOCK_PAYLOAD'
    TAKEOFF_WA = 'TAKEOFF_WA'
```

- [ ] **Step 8: Update _STATE_TO_PHASE in mission_state_machine.py**

Replace `MissionState.WA_SERVO_RESET: 'FM3'` with `MissionState.WA_PICKUP_READY: 'FM3'`.

Add `MissionState.WA_LOCK_PAYLOAD: 'FM3'` between `LAND_WA_FINAL` and `TAKEOFF_WA` entries.

- [ ] **Step 9: Update _transit_to_drop entry action**

Line 228: Change `'servo_release'` → `'arduino_servo_dispense'`:

```python
            return self.state, {'action': 'arrived_drop',
                                'entry_actions': ['arduino_servo_dispense']}
```

- [ ] **Step 10: Update _wa_drop_old_payload**

Line 264: Change `'arduino_servo_release'` → `'arduino_servo_drop'`:

```python
            return self.state, {'action': 'descend_hold_reached',
                                'entry_actions': ['arduino_servo_drop']}
```

Line 274: Change `self.config['pickup_settle_time_s']` → `self.config['payload_settle_time_s']`.

Line 276: Change `MissionState.WA_SERVO_RESET` → `MissionState.WA_PICKUP_READY`.

- [ ] **Step 11: Rename _wa_servo_reset to _wa_pickup_ready and update config key**

Rename method `_wa_servo_reset` → `_wa_pickup_ready`.

Line 283: Change `self.config['pickup_settle_time_s']` → `self.config['payload_settle_time_s']`.

- [ ] **Step 12: Update _land_wa_final LANDED transition**

Change `_land_wa_final` LANDED transition (lines 290-293):

```python
        if self._landing_state == 'LANDED':
            self._wa_drop_start_time = t
            self.state = MissionState.WA_LOCK_PAYLOAD
            return self.state, {'action': 'landed_wa',
                                'entry_actions': ['arduino_servo_lock']}
```

- [ ] **Step 13: Add _wa_lock_payload handler**

Add after `_wa_pickup_ready` method:

```python
    def _wa_lock_payload(self, vs, t):
        elapsed = t - self._wa_drop_start_time
        if elapsed >= self.config['payload_settle_time_s']:
            self.state = MissionState.TAKEOFF_WA
            return self.state, {'action': 'lock_complete',
                                'entry_actions': ['set_mode_guided', 'arm',
                                                  'takeoff']}
        return self.state, {'action': 'locking_payload', 'entry_actions': []}
```

- [ ] **Step 14: Update _transit_to_drop_2 entry action**

Line 318: Change `'servo_release'` → `'arduino_servo_dispense'`:

```python
            return self.state, {'action': 'arrived_drop_2',
                                'entry_actions': ['arduino_servo_dispense']}
```

- [ ] **Step 15: Update _handlers dict**

Replace `MissionState.WA_SERVO_RESET: _wa_servo_reset` with `MissionState.WA_PICKUP_READY: _wa_pickup_ready`.

Add `MissionState.WA_LOCK_PAYLOAD: _wa_lock_payload`.

- [ ] **Step 16: Run tests to verify they pass**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_mission_state_machine.py -v`
Expected: All PASS.

- [ ] **Step 17: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py src/dbvf_autonomy/test/test_mission_state_machine.py
git commit -m "feat: add WA_LOCK_PAYLOAD state, rename WA_SERVO_RESET, unify servo actions"
```

---

### Task 4: Update Mission Sequencer Node

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py`

Remove Cube Orange servo code, update param declarations, add new action handlers.

- [ ] **Step 1: Update param_defaults dict**

Replace the old servo params (lines 35-44):

Remove:
```python
            'drop_servo_number': 9,
            'drop_servo_pwm_release': 1100,
            'drop_servo_pwm_hold': 1500,
            ...
            'pickup_servo_number': 1,
            'pickup_servo_pwm_release': 1100,
            'pickup_servo_pwm_pickup': 1500,
            'pickup_settle_time_s': 2.0,
```

Add:
```python
            'payload_servo_channel': 0,
            'payload_servo_pwm_hold': 0,
            'payload_servo_pwm_dispense': 0,
            'payload_servo_pwm_drop': 0,
            'payload_servo_pwm_pickup': 0,
            'payload_servo_pwm_lock': 0,
            'payload_settle_time_s': 2.0,
```

- [ ] **Step 2: Remove servo_cli service client**

Remove line 96:
```python
        self.servo_cli = self.create_client(DoSetServo, '/dbvf/do_set_servo')
```

- [ ] **Step 3: Update _execute_action — remove old handlers, add new ones**

Remove the `servo_release` and `arduino_servo_release` handlers.

Add/update handlers:
```python
        elif action == 'arduino_servo_dispense':
            self._call_arduino_servo(
                cfg['payload_servo_channel'], cfg['payload_servo_pwm_dispense'])
        elif action == 'arduino_servo_drop':
            self._call_arduino_servo(
                cfg['payload_servo_channel'], cfg['payload_servo_pwm_drop'])
        elif action == 'arduino_servo_pickup':
            self._call_arduino_servo(
                cfg['payload_servo_channel'], cfg['payload_servo_pwm_pickup'])
        elif action == 'arduino_servo_lock':
            self._call_arduino_servo(
                cfg['payload_servo_channel'], cfg['payload_servo_pwm_lock'])
```

- [ ] **Step 4: Remove _call_set_servo method**

Remove the entire `_call_set_servo` method (lines 312-321).

- [ ] **Step 5: Run full test suite**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select dbvf_autonomy && colcon test-result --verbose`
Expected: All tests PASS.

- [ ] **Step 6: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py
git commit -m "refactor: remove Cube Orange servo code, add payload servo action handlers"
```

---

### Task 5: Update Mission Params YAML

**Files:**
- Modify: `src/dbvf_autonomy/config/mission_params.yaml`

- [ ] **Step 1: Replace servo sections in mission_params.yaml**

Remove the old sections (lines 43-54):
```yaml
    # -- Payload Servo --
    drop_servo_number: 9
    drop_servo_pwm_release: 1100
    drop_servo_pwm_hold: 1500
    drop_settle_time_s: 2.0

    # -- WA Reload Servo (Arduino) --
    pickup_servo_number: 1
    pickup_servo_pwm_release: 1100
    pickup_servo_pwm_pickup: 1500
    pickup_settle_time_s: 2.0
```

Replace with:
```yaml
    # ── Payload Servo (Arduino + PCA9685) ─────────────────────────
    payload_servo_channel: 0            # PCA9685 channel (0-15)
    payload_servo_pwm_hold: 0           # Position 1: pre-loaded hold (TBD)
    payload_servo_pwm_dispense: 0       # Position 2: release all payloads
    payload_servo_pwm_drop: 0           # Position 3: drop cradle mechanism
    payload_servo_pwm_pickup: 0         # Position 4: open for crew loading
    payload_servo_pwm_lock: 0           # Position 5: lock new payload
    drop_settle_time_s: 2.0
    payload_settle_time_s: 2.0          # Wait time after each servo move
```

- [ ] **Step 2: Commit**

```bash
git add src/dbvf_autonomy/config/mission_params.yaml
git commit -m "config: replace drop/pickup servo params with payload_servo_* config"
```

---

### Task 6: Update CLAUDE.md Documentation

**Files:**
- Modify: `src/dbvf_autonomy/.claude/CLAUDE.md`
- Modify: `.claude/CLAUDE.md` (root)

- [ ] **Step 1: Update dbvf_autonomy CLAUDE.md**

In the `mission_sequencer_node` section, update the state list from 20 to 21 states:
- Replace `WA_SERVO_RESET` with `WA_PICKUP_READY`
- Add `WA_LOCK_PAYLOAD` between `LAND_WA_FINAL` and `TAKEOFF_WA`
- Change "20-state linear FSM" to "21-state linear FSM"
- Update "4-state WA reload" to "5-state WA reload" where applicable

In `mission_helpers.py` description, update "includes pickup servo params" to "includes payload servo params".

In the test list, update `test_mission_config.py` description and `test_mission_state_machine.py` description to reference 21 states.

- [ ] **Step 2: Update root CLAUDE.md**

In the `mission_sequencer_node` description:
- "20-state linear FSM" → "21-state linear FSM"
- Replace `WA_SERVO_RESET` with `WA_PICKUP_READY`
- Add `WA_LOCK_PAYLOAD` in the WA reload sequence
- "4-state WA reload" → "5-state WA reload"

In the `mission_helpers.py` line: update "(includes pickup servo params" → "(includes payload servo params"

In the test count / description area, reference 21 states where 20 was mentioned.

Add the Arduino sketch entry:
```
- **Arduino payload servo sketch** | `src/dbvf_autonomy/arduino/payload_servo_controller/payload_servo_controller.ino`
```

- [ ] **Step 3: Commit**

```bash
git add src/dbvf_autonomy/.claude/CLAUDE.md .claude/CLAUDE.md
git commit -m "docs: update CLAUDE.md for 21-state FSM and payload servo changes"
```
