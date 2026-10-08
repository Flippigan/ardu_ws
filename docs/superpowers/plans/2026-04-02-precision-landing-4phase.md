# ISS-013: 4-Phase Precision Landing Final Approach — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the direct DESCEND_COARSE → DESCEND_OFFSET transition with a 4-phase sequence (HOLD_ABOVE_TAG → ALIGN_YAW → OFFSET_LATERAL → DESCEND_FINAL) that adds explicit position stabilization and yaw alignment before lateral offset and final descent.

**Architecture:** The `LandingStateMachine` (pure Python FSM) gains two new states (HOLD_ABOVE_TAG, ALIGN_YAW) and renames DESCEND_OFFSET to OFFSET_LATERAL. Each new state handles tag-lost timeout independently. The ROS node (`PrecisionLandingNode`) manages phase transitions via PID error checks (position stability, yaw alignment) and sends yaw rate commands during ALIGN_YAW. A P controller on tag-relative yaw drives rotation. All phases except DESCEND_FINAL operate at vz=0.0. The `StartPrecisionLanding` service gains a `target_yaw` parameter (radians, relative to tag orientation).

**Tech Stack:** Python 3, ROS2 Humble, pytest, MAVLink SET_POSITION_TARGET_LOCAL_NED (MAV_FRAME_BODY_NED)

**Dependency:** ISS-015 (tag-relative yaw on LandingTargetPose) must be implemented first. This plan designs as if `tag_yaw` is available on `LandingTargetPose`. Without ISS-015, the ALIGN_YAW phase auto-skips (graceful degradation via `getattr(target, 'tag_yaw', None)`).

---

## FSM State Flow (After This Change)

```
DESCEND_COARSE ──(small tag confirmed 2.0s)──→ HOLD_ABOVE_TAG
                                                    │
                                          (position stable 0.5s)
                                                    ↓
                                               ALIGN_YAW
                                                    │
                                          (yaw aligned 0.5s)
                                                    ↓
                                             OFFSET_LATERAL
                                                    │
                                          (offset achieved)
                                                    ↓
                                             DESCEND_FINAL
                                                    │
                                               (landed)
                                                    ↓
                                                 LANDED

SMALL_TAG_SEARCH ──(small tag found)──→ HOLD_ABOVE_TAG

All new states: tag lost for 4s → SEARCH
```

## File Map

| File | Action | Responsibility |
|------|--------|----------------|
| `src/dbvf_msgs/srv/StartPrecisionLanding.srv` | Modify | Add `target_yaw` field |
| `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py` | Modify | New states, handlers, wrap_angle, ROS node changes |
| `src/dbvf_autonomy/test/test_state_machine.py` | Modify | Tests for new states, transitions, wrap_angle, tag guard |
| `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py` | Modify | Rename `'DESCEND_OFFSET'` → `'OFFSET_LATERAL'` string check |
| `src/dbvf_autonomy/test/test_mission_state_machine.py` | Modify | Update string values in WA tests |
| `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py` | Modify | Pass `target_yaw` to start_precision_landing |
| `src/dbvf_autonomy/config/sim_params.yaml` | Modify | Add 6 new precision landing params |
| `src/dbvf_autonomy/config/hardware_params.yaml` | Modify | Add 6 new precision landing params |
| `src/dbvf_autonomy/config/mission_params.yaml` | Modify | Add `wa_target_yaw` |

## New Config Parameters (precision_landing)

| Parameter | Default | Unit | Purpose |
|-----------|---------|------|---------|
| `hold_position_tolerance` | 0.10 | m | Max lateral error to consider position "stable" |
| `hold_stabilize_time` | 0.5 | s | Duration position must stay stable before ALIGN_YAW |
| `yaw_alignment_tolerance` | 0.087 | rad (~5°) | Max yaw error to consider "aligned" |
| `yaw_alignment_hold_time` | 0.5 | s | Duration yaw must stay aligned before OFFSET_LATERAL |
| `yaw_kp` | 0.5 | — | Proportional gain for yaw P controller |
| `max_yaw_rate` | 0.35 | rad/s (~20°/s) | Maximum commanded yaw rotation rate |

---

### Task 1: Add target_yaw to StartPrecisionLanding.srv

**Files:**
- Modify: `src/dbvf_msgs/srv/StartPrecisionLanding.srv`

- [ ] **Step 1: Add target_yaw field to the service definition**

In `src/dbvf_msgs/srv/StartPrecisionLanding.srv`, add the `target_yaw` field after `offset_right`:

```
float64 target_lat
float64 target_lon
float64 offset_forward 0.0
float64 offset_right 0.0
float64 target_yaw 0.0
---
bool success
string message
```

`target_yaw` is in radians relative to tag orientation. 0.0 = aligned with tag top edge.

- [ ] **Step 2: Rebuild dbvf_msgs**

Run: `cd /home/finn/Documents/ardu_ws && colcon build --packages-select dbvf_msgs`
Expected: BUILD SUCCESS

- [ ] **Step 3: Commit**

```bash
git add src/dbvf_msgs/srv/StartPrecisionLanding.srv
git commit -m "feat(ISS-013): add target_yaw to StartPrecisionLanding.srv

Add float64 target_yaw (default 0.0) field — radians relative to tag
orientation. 0.0 = aligned with tag top edge. Used by 4-phase final
approach for yaw alignment before lateral offset."
```

---

### Task 2: Add wrap_angle pure function (TDD)

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`
- Modify: `src/dbvf_autonomy/test/test_state_machine.py`

- [ ] **Step 1: Write failing tests for wrap_angle**

Add at the top of `src/dbvf_autonomy/test/test_state_machine.py`, updating the import line:

```python
from dbvf_autonomy.precision_landing_node import (
    LandingStateMachine, LandingState, PIDController, wrap_angle,
)
```

Add these tests after the existing `test_body_frame_yaw_hold_uses_zero` test:

```python
# ---------------------------------------------------------------------------
# wrap_angle tests
# ---------------------------------------------------------------------------

def test_wrap_angle_zero():
    assert wrap_angle(0.0) == 0.0


def test_wrap_angle_positive_within_range():
    import math
    assert abs(wrap_angle(1.0) - 1.0) < 1e-9


def test_wrap_angle_negative_within_range():
    import math
    assert abs(wrap_angle(-1.0) - (-1.0)) < 1e-9


def test_wrap_angle_greater_than_pi():
    import math
    # 3*pi/2 should wrap to -pi/2
    result = wrap_angle(3 * math.pi / 2)
    assert abs(result - (-math.pi / 2)) < 1e-9


def test_wrap_angle_less_than_neg_pi():
    import math
    # -3*pi/2 should wrap to pi/2
    result = wrap_angle(-3 * math.pi / 2)
    assert abs(result - (math.pi / 2)) < 1e-9


def test_wrap_angle_exactly_pi():
    import math
    # pi should stay as pi (boundary)
    result = wrap_angle(math.pi)
    assert abs(result - math.pi) < 1e-9 or abs(result - (-math.pi)) < 1e-9


def test_wrap_angle_two_pi():
    import math
    result = wrap_angle(2 * math.pi)
    assert abs(result) < 1e-9
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_state_machine.py::test_wrap_angle_zero -v`
Expected: ImportError — `wrap_angle` not defined

- [ ] **Step 3: Implement wrap_angle**

In `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`, add after the `clamp_descent_rate` function (after line 33):

```python
def wrap_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return (angle + math.pi) % (2 * math.pi) - math.pi
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_state_machine.py -k wrap_angle -v`
Expected: All 7 wrap_angle tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py src/dbvf_autonomy/test/test_state_machine.py
git commit -m "feat(ISS-013): add wrap_angle pure function with tests

Wraps angle to [-pi, pi] range. Used by yaw alignment phase to compute
shortest rotation path."
```

---

### Task 3: Update LandingState enum + compute_preferred_tag_id

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`
- Modify: `src/dbvf_autonomy/test/test_state_machine.py`

- [ ] **Step 1: Write failing tests for compute_preferred_tag_id with new states**

Add these tests after the existing `test_preferred_tag_primary_when_range_alt_invalid` test in `src/dbvf_autonomy/test/test_state_machine.py`:

```python
def test_preferred_tag_secondary_during_hold_above_tag():
    """HOLD_ABOVE_TAG → preferred=2 (secondary)."""
    result = _compute_preferred_tag(LandingState.HOLD_ABOVE_TAG, 1.5, 2.0)
    assert result == 2


def test_preferred_tag_secondary_during_align_yaw():
    """ALIGN_YAW → preferred=2 (secondary)."""
    result = _compute_preferred_tag(LandingState.ALIGN_YAW, 1.5, 2.0)
    assert result == 2


def test_preferred_tag_secondary_during_offset_lateral():
    """OFFSET_LATERAL → preferred=2 (secondary)."""
    result = _compute_preferred_tag(LandingState.OFFSET_LATERAL, 1.5, 2.0)
    assert result == 2
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_state_machine.py::test_preferred_tag_secondary_during_hold_above_tag -v`
Expected: AttributeError — `LandingState` has no attribute `HOLD_ABOVE_TAG`

- [ ] **Step 3: Update LandingState enum**

In `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`, replace the `LandingState` enum (lines 65-74):

```python
class LandingState(Enum):
    IDLE = 'IDLE'
    APPROACH = 'APPROACH'
    SEARCH = 'SEARCH'
    DESCEND_COARSE = 'DESCEND_COARSE'
    HOLD_ABOVE_TAG = 'HOLD_ABOVE_TAG'
    ALIGN_YAW = 'ALIGN_YAW'
    OFFSET_LATERAL = 'OFFSET_LATERAL'
    DESCEND_FINAL = 'DESCEND_FINAL'
    SMALL_TAG_SEARCH = 'SMALL_TAG_SEARCH'
    LANDED = 'LANDED'
    ABORT_LAND = 'ABORT_LAND'
```

- [ ] **Step 4: Update compute_preferred_tag_id**

Replace the `compute_preferred_tag_id` function (lines 77-87):

```python
def compute_preferred_tag_id(state, range_alt, slow_descent_altitude,
                             primary_tag_id=1, secondary_tag_id=2):
    """Return the preferred tag ID for adapter coordination."""
    if state in (LandingState.HOLD_ABOVE_TAG, LandingState.ALIGN_YAW,
                 LandingState.OFFSET_LATERAL,
                 LandingState.DESCEND_FINAL, LandingState.SMALL_TAG_SEARCH):
        return secondary_tag_id
    if (state == LandingState.DESCEND_COARSE
            and range_alt >= 0.0
            and range_alt <= slow_descent_altitude):
        return secondary_tag_id
    return primary_tag_id
```

- [ ] **Step 5: Fix all references to DESCEND_OFFSET in precision_landing_node.py**

Use find-and-replace within `precision_landing_node.py` to change every `LandingState.DESCEND_OFFSET` to `LandingState.OFFSET_LATERAL` and every `_descend_offset` method name to `_offset_lateral`. Specifically:

In `LandingStateMachine.update()` dispatch (around line 144):
```python
        if self.state == LandingState.OFFSET_LATERAL:
            return self._offset_lateral(vehicle_state, tag_status, current_time)
```

In `_descend_coarse` transition (around line 199):
```python
                self.state = LandingState.OFFSET_LATERAL
```

Rename the method `_descend_offset` to `_offset_lateral` (around line 233):
```python
    def _offset_lateral(self, vs, tag_status, current_time):
```

In `_small_tag_search` transition (around line 281):
```python
            self.state = LandingState.OFFSET_LATERAL
```

- [ ] **Step 6: Fix all references to DESCEND_OFFSET in test_state_machine.py**

Update these references throughout `src/dbvf_autonomy/test/test_state_machine.py`:

Replace `LandingState.DESCEND_OFFSET` with `LandingState.OFFSET_LATERAL` everywhere in the file (approximately 15 occurrences). Update test docstrings that mention `DESCEND_OFFSET` to say `OFFSET_LATERAL`.

Update the `_apply_tag_guard` function:
```python
def _apply_tag_guard(state, target, secondary_tag_id=2):
    """Replicates the tag ID guard logic from _velocity_servo."""
    expected_secondary = (state in (LandingState.HOLD_ABOVE_TAG,
                                    LandingState.ALIGN_YAW,
                                    LandingState.OFFSET_LATERAL,
                                    LandingState.DESCEND_FINAL))
    if (expected_secondary and target is not None
            and target.tag_id != secondary_tag_id):
        target = None
    return target
```

Update test names and docstrings:
- `test_descend_coarse_to_offset` → update docstring to reference `OFFSET_LATERAL`
- `test_descend_offset_to_final` → rename to `test_offset_lateral_to_final`, update docstring
- `test_descend_offset_tag_lost` → rename to `test_offset_lateral_tag_lost`, update docstring
- `test_global_timeout_from_descend_offset` → rename to `test_global_timeout_from_offset_lateral`, update docstring
- `test_velocity_servo_rejects_wrong_tag_in_offset` → update to use `OFFSET_LATERAL`
- `test_velocity_servo_accepts_correct_tag_in_offset` → update to use `OFFSET_LATERAL`
- `test_search_pattern_finds_tag` → update assertion to `OFFSET_LATERAL`

- [ ] **Step 7: Run all tests to verify**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_state_machine.py -v`
Expected: All tests PASS (existing tests updated, new compute_preferred_tag_id tests pass)

- [ ] **Step 8: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py src/dbvf_autonomy/test/test_state_machine.py
git commit -m "feat(ISS-013): add HOLD_ABOVE_TAG, ALIGN_YAW states; rename DESCEND_OFFSET → OFFSET_LATERAL

Update LandingState enum with two new states for the 4-phase final
approach. Rename DESCEND_OFFSET to OFFSET_LATERAL for clarity. Update
compute_preferred_tag_id to return secondary tag for all new states.
Update all tests and tag guard logic."
```

---

### Task 4: FSM HOLD_ABOVE_TAG handler (TDD)

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`
- Modify: `src/dbvf_autonomy/test/test_state_machine.py`

- [ ] **Step 1: Write failing tests for HOLD_ABOVE_TAG**

Add a helper to advance FSM to HOLD_ABOVE_TAG in `src/dbvf_autonomy/test/test_state_machine.py`, after the `_to_descend_coarse` helper:

```python
def _to_hold_above_tag(sm):
    """Advance from IDLE to HOLD_ABOVE_TAG by directly setting state.

    In production, DESCEND_COARSE transitions here after small tag
    confirmation. We set state directly to decouple from the transition
    target update (Task 6). The natural transition path is tested there.
    """
    vs = _to_descend_coarse(sm)
    sm.state = LandingState.HOLD_ABOVE_TAG
    return vs
```

Add these tests:

```python
# ---------------------------------------------------------------------------
# HOLD_ABOVE_TAG tests
# ---------------------------------------------------------------------------

def test_hold_above_tag_returns_hold_action():
    """HOLD_ABOVE_TAG returns vz=0.0 and use_offset=False."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_hold_above_tag(sm)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    state, info = sm.update(vs, small_tag, 13.0)
    assert state == LandingState.HOLD_ABOVE_TAG
    assert info['action'] == 'holding'
    assert info['vz'] == 0.0
    assert info['use_offset'] is False


def test_hold_above_tag_tag_lost_to_search():
    """HOLD_ABOVE_TAG → SEARCH when secondary tag lost for tag_lost_timeout."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_hold_above_tag(sm)
    no_tag = MockTagStatus(detected=False)
    sm.update(vs, no_tag, 13.0)  # Start lost timer
    sm.update(vs, no_tag, 15.0)  # 2s < 4s
    state, info = sm.update(vs, no_tag, 17.1)  # 4.1s > 4s
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


def test_hold_above_tag_primary_only_triggers_tag_lost():
    """HOLD_ABOVE_TAG with only primary tag visible → tag_lost timer runs."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_hold_above_tag(sm)
    primary_tag = MockTagStatus(detected=True, active_tag_id=1)
    sm.update(vs, primary_tag, 13.0)  # Primary only — secondary lost
    sm.update(vs, primary_tag, 15.0)
    state, info = sm.update(vs, primary_tag, 17.1)  # 4.1s > 4s
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


def test_hold_above_tag_to_landed():
    """HOLD_ABOVE_TAG → LANDED on disarm."""
    sm = LandingStateMachine(CONFIG)
    _to_hold_above_tag(sm)
    landed = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.05, armed=False, vz=0.0)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    state, info = sm.update(landed, small_tag, 13.0)
    assert state == LandingState.LANDED
    assert info['action'] == 'landed'


def test_hold_above_tag_tag_reacquired_resets_timer():
    """HOLD_ABOVE_TAG tag reacquired before timeout resets lost timer."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_hold_above_tag(sm)
    no_tag = MockTagStatus(detected=False)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    sm.update(vs, no_tag, 13.0)   # Lost at t=13
    sm.update(vs, no_tag, 15.0)   # 2s elapsed
    sm.update(vs, small_tag, 15.5)  # Reacquired — resets timer
    sm.update(vs, no_tag, 16.0)   # Lost again at t=16
    state, _ = sm.update(vs, no_tag, 19.5)  # 3.5s < 4s
    assert state == LandingState.HOLD_ABOVE_TAG
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_state_machine.py::test_hold_above_tag_returns_hold_action -v`
Expected: FAIL — FSM update() does not dispatch to HOLD_ABOVE_TAG handler

- [ ] **Step 3: Implement _hold_above_tag handler**

In `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`, add the handler to `LandingStateMachine` after `_descend_coarse` (around line 231):

```python
    def _hold_above_tag(self, vs, tag_status, current_time):
        if self._is_landed(vs):
            self.state = LandingState.LANDED
            return self.state, {'action': 'landed'}

        tag_detected = tag_status and tag_status.detected
        small_tag_detected = (tag_detected
                              and tag_status.active_tag_id
                              == self.config.get('secondary_tag_id', 1))

        if not small_tag_detected:
            if self.tag_lost_time is None:
                self.tag_lost_time = current_time
            elif (current_time - self.tag_lost_time
                  > self.config['tag_lost_timeout']):
                self.tag_lost_time = None
                self.state = LandingState.SEARCH
                return self.state, {'action': 'tag_lost'}
        else:
            self.tag_lost_time = None

        return self.state, {
            'action': 'holding',
            'vz': 0.0,
            'use_offset': False,
        }
```

Add dispatch in `update()`, after the DESCEND_COARSE dispatch:

```python
        if self.state == LandingState.HOLD_ABOVE_TAG:
            return self._hold_above_tag(vehicle_state, tag_status, current_time)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_state_machine.py -k hold_above_tag -v`
Expected: All 5 HOLD_ABOVE_TAG tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py src/dbvf_autonomy/test/test_state_machine.py
git commit -m "feat(ISS-013): add HOLD_ABOVE_TAG FSM handler with tests

Holds position above secondary tag at vz=0.0 with no offset. Tracks
tag-lost timeout (4s → SEARCH). Phase 1 of the 4-phase final approach."
```

---

### Task 5: FSM ALIGN_YAW handler (TDD)

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`
- Modify: `src/dbvf_autonomy/test/test_state_machine.py`

- [ ] **Step 1: Write failing tests for ALIGN_YAW**

Add a helper to advance FSM to ALIGN_YAW in `src/dbvf_autonomy/test/test_state_machine.py`, after the `_to_hold_above_tag` helper:

```python
def _to_align_yaw(sm):
    """Advance from IDLE to ALIGN_YAW by directly setting state.

    In production, the ROS node triggers HOLD_ABOVE_TAG → ALIGN_YAW
    when position is stable. We simulate that by setting state directly.
    """
    vs = _to_hold_above_tag(sm)
    sm.state = LandingState.ALIGN_YAW
    return vs
```

Add these tests:

```python
# ---------------------------------------------------------------------------
# ALIGN_YAW tests
# ---------------------------------------------------------------------------

def test_align_yaw_returns_aligning_action():
    """ALIGN_YAW returns vz=0.0 and use_offset=False."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_align_yaw(sm)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    state, info = sm.update(vs, small_tag, 14.0)
    assert state == LandingState.ALIGN_YAW
    assert info['action'] == 'aligning_yaw'
    assert info['vz'] == 0.0
    assert info['use_offset'] is False


def test_align_yaw_tag_lost_to_search():
    """ALIGN_YAW → SEARCH when secondary tag lost for tag_lost_timeout."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_align_yaw(sm)
    no_tag = MockTagStatus(detected=False)
    sm.update(vs, no_tag, 14.0)  # Start lost timer
    sm.update(vs, no_tag, 16.0)  # 2s < 4s
    state, info = sm.update(vs, no_tag, 18.1)  # 4.1s > 4s
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


def test_align_yaw_primary_only_triggers_tag_lost():
    """ALIGN_YAW with only primary tag visible → tag_lost timer runs."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_align_yaw(sm)
    primary_tag = MockTagStatus(detected=True, active_tag_id=1)
    sm.update(vs, primary_tag, 14.0)
    sm.update(vs, primary_tag, 16.0)
    state, info = sm.update(vs, primary_tag, 18.1)
    assert state == LandingState.SEARCH
    assert info['action'] == 'tag_lost'


def test_align_yaw_to_landed():
    """ALIGN_YAW → LANDED on disarm."""
    sm = LandingStateMachine(CONFIG)
    _to_align_yaw(sm)
    landed = MockVehicleState(lat=LAT, lon=LON, alt_rel=0.05, armed=False, vz=0.0)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    state, info = sm.update(landed, small_tag, 14.0)
    assert state == LandingState.LANDED
    assert info['action'] == 'landed'


def test_align_yaw_tag_reacquired_resets_timer():
    """ALIGN_YAW tag reacquired before timeout resets lost timer."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_align_yaw(sm)
    no_tag = MockTagStatus(detected=False)
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    sm.update(vs, no_tag, 14.0)   # Lost
    sm.update(vs, no_tag, 16.0)   # 2s
    sm.update(vs, small_tag, 16.5)  # Reacquired
    sm.update(vs, no_tag, 17.0)   # Lost again
    state, _ = sm.update(vs, no_tag, 20.5)  # 3.5s < 4s
    assert state == LandingState.ALIGN_YAW
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_state_machine.py::test_align_yaw_returns_aligning_action -v`
Expected: FAIL — FSM update() does not dispatch to ALIGN_YAW handler

- [ ] **Step 3: Implement _align_yaw handler**

In `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`, add the handler to `LandingStateMachine` after `_hold_above_tag`:

```python
    def _align_yaw(self, vs, tag_status, current_time):
        if self._is_landed(vs):
            self.state = LandingState.LANDED
            return self.state, {'action': 'landed'}

        tag_detected = tag_status and tag_status.detected
        small_tag_detected = (tag_detected
                              and tag_status.active_tag_id
                              == self.config.get('secondary_tag_id', 1))

        if not small_tag_detected:
            if self.tag_lost_time is None:
                self.tag_lost_time = current_time
            elif (current_time - self.tag_lost_time
                  > self.config['tag_lost_timeout']):
                self.tag_lost_time = None
                self.state = LandingState.SEARCH
                return self.state, {'action': 'tag_lost'}
        else:
            self.tag_lost_time = None

        return self.state, {
            'action': 'aligning_yaw',
            'vz': 0.0,
            'use_offset': False,
        }
```

Add dispatch in `update()`, after the HOLD_ABOVE_TAG dispatch:

```python
        if self.state == LandingState.ALIGN_YAW:
            return self._align_yaw(vehicle_state, tag_status, current_time)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_state_machine.py -k align_yaw -v`
Expected: All 5 ALIGN_YAW tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py src/dbvf_autonomy/test/test_state_machine.py
git commit -m "feat(ISS-013): add ALIGN_YAW FSM handler with tests

Holds position above secondary tag at vz=0.0 while ROS node performs
yaw alignment. Tracks tag-lost timeout (4s → SEARCH). Phase 2 of the
4-phase final approach."
```

---

### Task 6: Update FSM transitions + update existing tests

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`
- Modify: `src/dbvf_autonomy/test/test_state_machine.py`

This task changes the FSM transition targets so that DESCEND_COARSE and SMALL_TAG_SEARCH
now go to HOLD_ABOVE_TAG instead of OFFSET_LATERAL.

- [ ] **Step 1: Update _descend_coarse transition target**

In `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`, in `_descend_coarse` (around line 199), change:

```python
                self.state = LandingState.OFFSET_LATERAL
```
to:
```python
                self.state = LandingState.HOLD_ABOVE_TAG
```

- [ ] **Step 2: Update _small_tag_search transition target**

In `_small_tag_search` (around line 281), change:

```python
            self.state = LandingState.OFFSET_LATERAL
```
to:
```python
            self.state = LandingState.HOLD_ABOVE_TAG
```

- [ ] **Step 3: Update existing tests for new transition targets**

In `src/dbvf_autonomy/test/test_state_machine.py`:

Update `test_descend_coarse_to_offset` (rename and fix assertion):
```python
def test_descend_coarse_to_hold_above_tag():
    """DESCEND_COARSE -> HOLD_ABOVE_TAG when small tag detected for small_tag_confirm_time."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    # First detection starts the timer
    sm.update(vs, small_tag, 10.0)
    assert sm.state == LandingState.DESCEND_COARSE

    # Not enough time yet
    sm.update(vs, small_tag, 11.0)
    assert sm.state == LandingState.DESCEND_COARSE

    # 2.0 seconds of continuous detection → transition
    state, info = sm.update(vs, small_tag, 12.1)
    assert state == LandingState.HOLD_ABOVE_TAG
    assert info['action'] == 'small_tag_confirmed'
```

Update `test_search_pattern_finds_tag`:
```python
def test_search_pattern_finds_tag():
    """SMALL_TAG_SEARCH -> HOLD_ABOVE_TAG when small tag detected during search."""
    sm = LandingStateMachine(CONFIG)
    vs = _to_descend_coarse(sm)

    # Trigger search pattern at floor altitude with no tag visible
    low_vs = MockVehicleState(lat=LAT, lon=LON, alt_rel=1.5)
    no_tag = MockTagStatus(detected=False)
    sm.update(low_vs, no_tag, 10.0)  # -> SMALL_TAG_SEARCH
    assert sm.state == LandingState.SMALL_TAG_SEARCH

    # Small tag appears during search
    small_tag = MockTagStatus(detected=True, active_tag_id=2)
    state, info = sm.update(low_vs, small_tag, 11.0)
    assert state == LandingState.HOLD_ABOVE_TAG
    assert info['action'] == 'small_tag_found'
```

- [ ] **Step 4: Run all state machine tests**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_state_machine.py -v`
Expected: All tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py src/dbvf_autonomy/test/test_state_machine.py
git commit -m "feat(ISS-013): route small tag confirmation to HOLD_ABOVE_TAG

DESCEND_COARSE and SMALL_TAG_SEARCH now transition to HOLD_ABOVE_TAG
instead of directly to OFFSET_LATERAL. This inserts the hold + yaw
alignment phases before lateral offset."
```

---

### Task 7: Update mission_state_machine.py string check + tests

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py:260`
- Modify: `src/dbvf_autonomy/test/test_mission_state_machine.py`

The mission state machine checks the precision landing state string to trigger WA reload. Update the string from `'DESCEND_OFFSET'` to `'OFFSET_LATERAL'`.

- [ ] **Step 1: Update string check in mission_state_machine.py**

In `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py`, line 260, change:

```python
        if self._landing_state == 'DESCEND_OFFSET':
```
to:
```python
        if self._landing_state == 'OFFSET_LATERAL':
```

- [ ] **Step 2: Update test_mission_state_machine.py**

In `src/dbvf_autonomy/test/test_mission_state_machine.py`, update all occurrences of `'DESCEND_OFFSET'`:

Line 356:
```python
    sm.set_landing_state('OFFSET_LATERAL')
```

Line 378:
```python
    sm.set_landing_state('OFFSET_LATERAL')
```

- [ ] **Step 3: Run mission state machine tests**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_mission_state_machine.py -v`
Expected: All tests PASS

- [ ] **Step 4: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py src/dbvf_autonomy/test/test_mission_state_machine.py
git commit -m "refactor(ISS-013): update mission FSM to use OFFSET_LATERAL string

Rename DESCEND_OFFSET → OFFSET_LATERAL in the _land_wa_descend check
and corresponding tests. Keeps WA reload trigger at the same logical
point in the landing sequence."
```

---

### Task 8: ROS node — new parameters, target_yaw, control loop updates

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`

This task updates the `PrecisionLandingNode` class. No new tests — the ROS node is not unit-tested (it uses ROS2 runtime). The pure FSM logic was tested in Tasks 4-6.

- [ ] **Step 1: Declare new parameters**

In `PrecisionLandingNode.__init__()`, after the `self.declare_parameter('servo_max_speed', 0.5)` line (around line 370), add:

```python
        self.declare_parameter('hold_position_tolerance', 0.10)
        self.declare_parameter('hold_stabilize_time', 0.5)
        self.declare_parameter('yaw_alignment_tolerance', 0.087)
        self.declare_parameter('yaw_alignment_hold_time', 0.5)
        self.declare_parameter('yaw_kp', 0.5)
        self.declare_parameter('max_yaw_rate', 0.35)
```

- [ ] **Step 2: Store parameters and add instance variables**

After the PID controller initialization (after `self.pid_y = ...`), add:

```python
        self._hold_position_tolerance = self.get_parameter('hold_position_tolerance').value
        self._hold_stabilize_time = self.get_parameter('hold_stabilize_time').value
        self._yaw_alignment_tolerance = self.get_parameter('yaw_alignment_tolerance').value
        self._yaw_alignment_hold_time = self.get_parameter('yaw_alignment_hold_time').value
        self._yaw_kp = self.get_parameter('yaw_kp').value
        self._max_yaw_rate = self.get_parameter('max_yaw_rate').value
```

After `self._last_control_time = 0.0`, add:

```python
        self.target_yaw = 0.0
        self._hold_stable_since = None
        self._yaw_aligned_since = None
```

- [ ] **Step 3: Update _start_landing_cb to store target_yaw**

In `_start_landing_cb`, after `self.offset_right = request.offset_right`, add:

```python
        self.target_yaw = request.target_yaw
```

Update the log message to include target_yaw:

```python
        self.get_logger().info(
            f'Starting precision landing at '
            f'{request.target_lat:.7f}, {request.target_lon:.7f} '
            f'(offset fwd={self.offset_forward:.3f}, right={self.offset_right:.3f}, '
            f'yaw={self.target_yaw:.3f}rad)')
```

After `self._last_control_time = 0.0`, add resets:

```python
        self._hold_stable_since = None
        self._yaw_aligned_since = None
```

- [ ] **Step 4: Update control loop transition handlers**

In `_control_loop`, in the `if state != prev_state:` block, update the transition handlers. Replace the `DESCEND_OFFSET` handler:

```python
            elif state == LandingState.HOLD_ABOVE_TAG:
                self.pid_x.reset()
                self.pid_y.reset()
                self._hold_stable_since = None
            elif state == LandingState.ALIGN_YAW:
                self.pid_x.reset()
                self.pid_y.reset()
                self._yaw_aligned_since = None
            elif state == LandingState.OFFSET_LATERAL:
                self.pid_x.reset()
                self.pid_y.reset()
```

- [ ] **Step 5: Update control loop state dispatch**

Replace the velocity servo dispatch (around line 535-537):

```python
        elif state in (LandingState.DESCEND_COARSE,
                       LandingState.HOLD_ABOVE_TAG, LandingState.ALIGN_YAW,
                       LandingState.OFFSET_LATERAL, LandingState.DESCEND_FINAL):
            self._velocity_servo(state, info, now)
```

- [ ] **Step 6: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py
git commit -m "feat(ISS-013): add ROS node params and control loop for 4-phase landing

Declare 6 new parameters (hold tolerance/time, yaw tolerance/time,
yaw_kp, max_yaw_rate). Store target_yaw from service request. Update
control loop to dispatch new states to _velocity_servo."
```

---

### Task 9: ROS node — _velocity_servo stability/yaw transitions + yaw rate command

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`

- [ ] **Step 1: Update tag ID guard in _velocity_servo**

In `_velocity_servo`, replace the `expected_secondary` check:

```python
        expected_secondary = (state in (LandingState.HOLD_ABOVE_TAG,
                                        LandingState.ALIGN_YAW,
                                        LandingState.OFFSET_LATERAL,
                                        LandingState.DESCEND_FINAL))
```

- [ ] **Step 2: Add HOLD_ABOVE_TAG → ALIGN_YAW stability transition**

In `_velocity_servo`, inside the `if tag_detected:` block, after computing `error_x` and `error_y` (and before the offset check), add:

```python
            # HOLD_ABOVE_TAG → ALIGN_YAW (position stability check)
            if state == LandingState.HOLD_ABOVE_TAG:
                if math.sqrt(error_x**2 + error_y**2) < self._hold_position_tolerance:
                    if self._hold_stable_since is None:
                        self._hold_stable_since = now
                    elif now - self._hold_stable_since >= self._hold_stabilize_time:
                        self.fsm.state = LandingState.ALIGN_YAW
                        self._hold_stable_since = None
                        self.get_logger().info(
                            f'{state.value} -> ALIGN_YAW (position_stable)')
                else:
                    self._hold_stable_since = None
```

- [ ] **Step 3: Add ALIGN_YAW → OFFSET_LATERAL yaw alignment transition + yaw rate**

After the HOLD_ABOVE_TAG block, add:

```python
            # ALIGN_YAW → OFFSET_LATERAL (yaw alignment check + yaw rate command)
            if state == LandingState.ALIGN_YAW:
                tag_yaw = getattr(target, 'tag_yaw', None)
                if tag_yaw is not None:
                    yaw_error = wrap_angle(self.target_yaw - tag_yaw)
                    yaw_rate_cmd = self._yaw_kp * yaw_error
                    yaw_rate_cmd = max(-self._max_yaw_rate,
                                      min(self._max_yaw_rate, yaw_rate_cmd))
                    if abs(yaw_error) < self._yaw_alignment_tolerance:
                        if self._yaw_aligned_since is None:
                            self._yaw_aligned_since = now
                        elif (now - self._yaw_aligned_since
                              >= self._yaw_alignment_hold_time):
                            self.fsm.state = LandingState.OFFSET_LATERAL
                            self._yaw_aligned_since = None
                            self.get_logger().info(
                                f'{state.value} -> OFFSET_LATERAL (yaw_aligned)')
                            yaw_rate_cmd = None  # Stop rotating
                    else:
                        self._yaw_aligned_since = None
                else:
                    # No tag_yaw available (ISS-015 not implemented) — skip alignment
                    self.fsm.state = LandingState.OFFSET_LATERAL
                    self.get_logger().info(
                        f'{state.value} -> OFFSET_LATERAL (no_tag_yaw, skipped)')
                    yaw_rate_cmd = None
```

Note: `yaw_rate_cmd` is a local variable that will be used in the velocity command below.

- [ ] **Step 4: Update OFFSET_LATERAL offset check**

Rename the existing DESCEND_OFFSET check (around line 593):

```python
            if (state == LandingState.OFFSET_LATERAL
                    and math.sqrt(error_x**2 + error_y**2)
                    < self.offset_tolerance):
                self.fsm.state = LandingState.DESCEND_FINAL
                self.get_logger().info(
                    f'{state.value} -> DESCEND_FINAL (offset_achieved)')
```

- [ ] **Step 5: Wire yaw_rate_cmd into velocity command**

Initialize `yaw_rate_cmd = None` at the top of `_velocity_servo` (before the tag_detected check).

Replace the `self._call_guided_velocity(vx, vy, vz)` call at the end of `_velocity_servo` with:

```python
        self._call_guided_velocity(vx, vy, vz, yaw_rate=yaw_rate_cmd)
```

- [ ] **Step 6: Modify _call_guided_velocity to accept yaw_rate**

Update the method signature and body:

```python
    def _call_guided_velocity(self, vx, vy, vz, yaw_rate=None):
        # Global descent rate clamp near ground (ISS-011)
        vs = self.latest_vehicle_state
        if vs is not None:
            clamped_vz = clamp_descent_rate(
                vz, vs.range_alt,
                self._slow_descent_altitude, self._slow_descent_rate)
            if clamped_vz != vz:
                if not self._slow_descent_active:
                    self._slow_descent_active = True
                    self.get_logger().info(
                        f'Slow descent clamp active: range_alt={vs.range_alt:.1f}m, '
                        f'vz {vz:.2f} -> {clamped_vz:.2f} m/s')
                vz = clamped_vz
            elif self._slow_descent_active:
                self._slow_descent_active = False

        if not self.velocity_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('send_guided_velocity service unavailable')
            return
        req = SendGuidedVelocity.Request()
        req.vx = vx
        req.vy = vy
        req.vz = vz

        if yaw_rate is not None:
            req.yaw_rate = yaw_rate
            req.use_yaw_rate = True
        elif vs is not None:
            req.yaw = 0.0
            req.use_yaw = True

        self.velocity_cli.call_async(req)
```

- [ ] **Step 7: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py
git commit -m "feat(ISS-013): add stability/yaw transitions and yaw rate command

HOLD_ABOVE_TAG → ALIGN_YAW when position stable for 0.5s.
ALIGN_YAW → OFFSET_LATERAL when yaw aligned for 0.5s (or auto-skip
if tag_yaw unavailable). P controller on yaw error sends yaw_rate via
SendGuidedVelocity. _call_guided_velocity now accepts optional
yaw_rate parameter."
```

---

### Task 10: Config files + mission sequencer integration

**Files:**
- Modify: `src/dbvf_autonomy/config/sim_params.yaml`
- Modify: `src/dbvf_autonomy/config/hardware_params.yaml`
- Modify: `src/dbvf_autonomy/config/mission_params.yaml`
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py`

- [ ] **Step 1: Add new parameters to sim_params.yaml**

In `src/dbvf_autonomy/config/sim_params.yaml`, in the `precision_landing.ros__parameters` section, after the `servo_max_speed: 0.5` line, add:

```yaml
    # 4-phase final approach (ISS-013)
    hold_position_tolerance: 0.10   # metres — max lateral error for "stable"
    hold_stabilize_time: 0.5        # seconds — hold stable before ALIGN_YAW
    yaw_alignment_tolerance: 0.087  # radians (~5 deg) — max yaw error for "aligned"
    yaw_alignment_hold_time: 0.5    # seconds — hold aligned before OFFSET_LATERAL
    yaw_kp: 0.5                     # P gain for yaw controller
    max_yaw_rate: 0.35              # rad/s (~20 deg/s) — max rotation rate
```

- [ ] **Step 2: Add new parameters to hardware_params.yaml**

In `src/dbvf_autonomy/config/hardware_params.yaml`, in the `precision_landing.ros__parameters` section, after the `servo_max_speed: 0.5` line, add:

```yaml
    # 4-phase final approach (ISS-013)
    hold_position_tolerance: 0.10
    hold_stabilize_time: 0.5
    yaw_alignment_tolerance: 0.087
    yaw_alignment_hold_time: 0.5
    yaw_kp: 0.5
    max_yaw_rate: 0.35
```

- [ ] **Step 3: Add wa_target_yaw to mission_params.yaml**

In `src/dbvf_autonomy/config/mission_params.yaml`, after the `wa_offset_right` line, add:

```yaml
    wa_target_yaw: 0.0        # radians, relative to tag orientation (0.0 = aligned with tag top)
```

- [ ] **Step 4: Update mission_sequencer_node.py to pass target_yaw**

In `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py`, update `_call_start_precision_landing` signature:

```python
    def _call_start_precision_landing(self, lat, lon,
                                      offset_forward=0.0, offset_right=0.0,
                                      target_yaw=0.0):
        if not self.precision_land_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('start_precision_landing service unavailable')
            return
        req = StartPrecisionLanding.Request()
        req.target_lat = lat
        req.target_lon = lon
        req.offset_forward = offset_forward
        req.offset_right = offset_right
        req.target_yaw = target_yaw
        future = self.precision_land_cli.call_async(req)
        future.add_done_callback(lambda f: self.get_logger().info(
            f'Precision landing: {f.result().message}') if f.result() else None)
```

Update the call site in the action dispatcher (around line 241-245):

```python
        elif action == 'start_precision_landing':
            self._call_start_precision_landing(
                cfg['wa_lat'], cfg['wa_lon'],
                cfg.get('wa_offset_forward', 0.0),
                cfg.get('wa_offset_right', 0.0),
                cfg.get('wa_target_yaw', 0.0))
```

- [ ] **Step 5: Rebuild dbvf_autonomy**

Run: `cd /home/finn/Documents/ardu_ws && colcon build --packages-select dbvf_msgs dbvf_autonomy`
Expected: BUILD SUCCESS

- [ ] **Step 6: Commit**

```bash
git add src/dbvf_autonomy/config/sim_params.yaml src/dbvf_autonomy/config/hardware_params.yaml src/dbvf_autonomy/config/mission_params.yaml src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py
git commit -m "feat(ISS-013): add 4-phase config params and mission sequencer target_yaw

Add hold/yaw tolerance and timing params to sim and hardware configs.
Add wa_target_yaw to mission config. Mission sequencer now passes
target_yaw through to start_precision_landing service."
```

---

### Task 11: Full verification

**Files:** None (verification only)

- [ ] **Step 1: Run all dbvf_autonomy tests**

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select dbvf_autonomy && colcon test-result --verbose`
Expected: All tests PASS (existing 202 + new wrap_angle/hold/yaw tests)

- [ ] **Step 2: Build all DBVF packages**

Run: `cd /home/finn/Documents/ardu_ws && colcon build --packages-select dbvf_msgs dbvf_autonomy`
Expected: BUILD SUCCESS

- [ ] **Step 3: Verify state flow in test**

Confirm the complete FSM test flow by running:

Run: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_state_machine.py -v --tb=short`
Expected: All state machine tests PASS, including:
- `test_descend_coarse_to_hold_above_tag` — DESCEND_COARSE → HOLD_ABOVE_TAG
- `test_hold_above_tag_*` — HOLD_ABOVE_TAG behavior (5 tests)
- `test_align_yaw_*` — ALIGN_YAW behavior (5 tests)
- `test_offset_lateral_*` — OFFSET_LATERAL behavior (renamed from DESCEND_OFFSET)
- `test_search_pattern_finds_tag` — SMALL_TAG_SEARCH → HOLD_ABOVE_TAG
- `test_wrap_angle_*` — wrap_angle function (7 tests)
- `test_preferred_tag_secondary_during_*` — compute_preferred_tag_id for new states (3 tests)

---

## Summary of Changes

| Area | What Changed |
|------|-------------|
| **Service** | `StartPrecisionLanding.srv` gains `float64 target_yaw 0.0` |
| **States** | Added HOLD_ABOVE_TAG, ALIGN_YAW; renamed DESCEND_OFFSET → OFFSET_LATERAL |
| **FSM handlers** | `_hold_above_tag()`: vz=0, tag-lost tracking, no offset |
| | `_align_yaw()`: vz=0, tag-lost tracking, no offset |
| **FSM transitions** | DESCEND_COARSE → HOLD_ABOVE_TAG (was → DESCEND_OFFSET) |
| | SMALL_TAG_SEARCH → HOLD_ABOVE_TAG (was → DESCEND_OFFSET) |
| **Node transitions** | HOLD_ABOVE_TAG → ALIGN_YAW (position stable for hold_stabilize_time) |
| | ALIGN_YAW → OFFSET_LATERAL (yaw aligned for yaw_alignment_hold_time, or auto-skip if no tag_yaw) |
| | OFFSET_LATERAL → DESCEND_FINAL (offset achieved — unchanged logic) |
| **Yaw control** | P controller: `yaw_rate = kp * wrap_angle(target_yaw - tag_yaw)`, clamped to max_yaw_rate |
| **Velocity cmd** | `_call_guided_velocity` gains `yaw_rate=` kwarg → `use_yaw_rate=True` in BODY_NED |
| **Mission FSM** | `'DESCEND_OFFSET'` string → `'OFFSET_LATERAL'` in WA reload trigger |
| **Mission sequencer** | Passes `target_yaw` through to precision landing service |
| **Config** | 6 new precision_landing params, 1 new mission param (`wa_target_yaw`) |
| **Tests** | ~17 new tests (7 wrap_angle + 5 HOLD_ABOVE_TAG + 5 ALIGN_YAW + 3 compute_preferred), updated existing tests |
