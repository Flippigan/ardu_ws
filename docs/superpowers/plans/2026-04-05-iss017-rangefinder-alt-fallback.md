# ISS-017: Rangefinder Alt Fallback for Preferred Tag Switch

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the 4-phase precision landing never being entered because `compute_preferred_tag_id()` gates on `range_alt >= 0.0`, which stays at sentinel (-1.0) when RANGEFINDER MAVLink messages are not received.

**Architecture:** Two-pronged fix: (1) add `alt_rel` (barometric altitude) fallback in `compute_preferred_tag_id()` so the preferred tag switch works even without a rangefinder, and (2) add explicit RANGEFINDER stream rate param to ArduPilot config so the rangefinder data arrives in sim. The fallback ensures the 4-phase landing works on any platform regardless of rangefinder availability. When `range_alt` is valid it is preferred (more accurate near ground); when it's sentinel, `alt_rel` is used instead.

**Tech Stack:** Python (ROS2 Humble), pytest, ArduPilot SITL params

---

## Root Cause Reference

See ISS-017 in `src/dbvf_autonomy/docs/Features/Auto Landing April Tags/Final Positioning during landing/issues.md`. Summary:

1. `compute_preferred_tag_id()` (`precision_landing_node.py:83-94`) returns secondary only when `range_alt >= 0.0 and range_alt <= slow_descent_altitude`
2. `range_alt` initializes to -1.0 (sentinel) in `mavlink_interface_node.py:27,136` and only updates on RANGEFINDER MAVLink messages
3. If RANGEFINDER messages never arrive, `range_alt` stays -1.0 the entire flight
4. Preferred tag never switches to secondary → adapter never reports secondary as active → `small_tag_confirmed` timer never starts → 4-phase landing never entered → offsets never applied

---

## File Map

| File | Action | Purpose |
|------|--------|---------|
| `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py` | Modify `compute_preferred_tag_id()` (lines 83-94) + call site (lines 646-651) | Add `alt_rel` parameter and fallback logic |
| `src/dbvf_autonomy/test/test_state_machine.py` | Add tests | 4 new tests for alt_rel fallback behavior |
| `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm` | Add stream rate param | Ensure RANGEFINDER messages are sent at 10Hz in SITL |

---

### Task 1: Write failing tests for alt_rel fallback in `compute_preferred_tag_id`

**Files:**
- Modify: `src/dbvf_autonomy/test/test_state_machine.py` (append after the existing preferred tag tests near line 635)

- [ ] **Step 1: Write the failing tests**

Add to `src/dbvf_autonomy/test/test_state_machine.py` after the `test_preferred_tag_secondary_during_offset_lateral` test (around line 635):

```python
# ---------------------------------------------------------------------------
# ISS-017: alt_rel fallback in compute_preferred_tag_id
# ---------------------------------------------------------------------------

def test_preferred_tag_secondary_with_sentinel_range_alt_and_low_alt_rel():
    """DESCEND_COARSE with range_alt=-1.0 (sentinel) + alt_rel=1.5 (below 2.0m) → preferred=2."""
    result = _compute_preferred_tag(LandingState.DESCEND_COARSE, -1.0, 2.0, alt_rel=1.5)
    assert result == 2


def test_preferred_tag_primary_with_sentinel_range_alt_and_high_alt_rel():
    """DESCEND_COARSE with range_alt=-1.0 (sentinel) + alt_rel=5.0 (above 2.0m) → preferred=1."""
    result = _compute_preferred_tag(LandingState.DESCEND_COARSE, -1.0, 2.0, alt_rel=5.0)
    assert result == 1


def test_preferred_tag_primary_with_sentinel_both_alts():
    """DESCEND_COARSE with range_alt=-1.0 + alt_rel=-1.0 (both sentinel) → preferred=1."""
    result = _compute_preferred_tag(LandingState.DESCEND_COARSE, -1.0, 2.0, alt_rel=-1.0)
    assert result == 1


def test_preferred_tag_range_alt_preferred_over_alt_rel():
    """DESCEND_COARSE with valid range_alt=1.8 uses range_alt, not alt_rel=5.0."""
    result = _compute_preferred_tag(LandingState.DESCEND_COARSE, 1.8, 2.0, alt_rel=5.0)
    assert result == 2
```

- [ ] **Step 2: Update the `_compute_preferred_tag` test helper to pass `alt_rel`**

In the same file, update the `_compute_preferred_tag` helper function (around line 594) from:

```python
def _compute_preferred_tag(state, range_alt, slow_descent_altitude):
    """Replicates the preferred tag publish logic from _control_loop."""
    from dbvf_autonomy.precision_landing_node import compute_preferred_tag_id
    return compute_preferred_tag_id(state, range_alt, slow_descent_altitude)
```

to:

```python
def _compute_preferred_tag(state, range_alt, slow_descent_altitude, alt_rel=-1.0):
    """Replicates the preferred tag publish logic from _control_loop."""
    from dbvf_autonomy.precision_landing_node import compute_preferred_tag_id
    return compute_preferred_tag_id(state, range_alt, slow_descent_altitude, alt_rel=alt_rel)
```

- [ ] **Step 3: Run tests to verify they fail**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
cd /home/finn/Documents/ardu_ws
python -m pytest src/dbvf_autonomy/test/test_state_machine.py::test_preferred_tag_secondary_with_sentinel_range_alt_and_low_alt_rel src/dbvf_autonomy/test/test_state_machine.py::test_preferred_tag_primary_with_sentinel_range_alt_and_high_alt_rel src/dbvf_autonomy/test/test_state_machine.py::test_preferred_tag_primary_with_sentinel_both_alts src/dbvf_autonomy/test/test_state_machine.py::test_preferred_tag_range_alt_preferred_over_alt_rel -v
```

Expected: FAIL — `compute_preferred_tag_id` does not accept `alt_rel` parameter yet.

- [ ] **Step 4: Commit failing tests**

```bash
git add src/dbvf_autonomy/test/test_state_machine.py
git commit -m "test(ISS-017): add failing tests for alt_rel fallback in compute_preferred_tag_id"
```

---

### Task 2: Add `alt_rel` fallback to `compute_preferred_tag_id()`

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py:83-94` (function definition)
- Modify: `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py:646-651` (call site in `_control_loop`)

- [ ] **Step 1: Update the function signature and logic**

In `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`, change `compute_preferred_tag_id` (lines 83-94) from:

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

to:

```python
def compute_preferred_tag_id(state, range_alt, slow_descent_altitude,
                             primary_tag_id=1, secondary_tag_id=2,
                             alt_rel=-1.0):
    """Return the preferred tag ID for adapter coordination.

    During DESCEND_COARSE, switches to secondary tag when altitude is at or
    below slow_descent_altitude. Prefers range_alt (rangefinder) when valid;
    falls back to alt_rel (barometric) when range_alt is sentinel (-1.0).
    """
    if state in (LandingState.HOLD_ABOVE_TAG, LandingState.ALIGN_YAW,
                 LandingState.OFFSET_LATERAL,
                 LandingState.DESCEND_FINAL, LandingState.SMALL_TAG_SEARCH):
        return secondary_tag_id
    if state == LandingState.DESCEND_COARSE:
        # Prefer rangefinder; fall back to barometric altitude
        effective_alt = range_alt if range_alt >= 0.0 else alt_rel
        if effective_alt >= 0.0 and effective_alt <= slow_descent_altitude:
            return secondary_tag_id
    return primary_tag_id
```

- [ ] **Step 2: Update the call site in `_control_loop`**

In the same file, update the `compute_preferred_tag_id` call (lines 646-651) from:

```python
        preferred.data = compute_preferred_tag_id(
            state, vs.range_alt, self._slow_descent_altitude,
            primary_tag_id=self.fsm.config['primary_tag_id'],
            secondary_tag_id=self.fsm.config['secondary_tag_id'])
```

to:

```python
        preferred.data = compute_preferred_tag_id(
            state, vs.range_alt, self._slow_descent_altitude,
            primary_tag_id=self.fsm.config['primary_tag_id'],
            secondary_tag_id=self.fsm.config['secondary_tag_id'],
            alt_rel=vs.alt_rel)
```

- [ ] **Step 3: Run the new tests to verify they pass**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
cd /home/finn/Documents/ardu_ws
python -m pytest src/dbvf_autonomy/test/test_state_machine.py::test_preferred_tag_secondary_with_sentinel_range_alt_and_low_alt_rel src/dbvf_autonomy/test/test_state_machine.py::test_preferred_tag_primary_with_sentinel_range_alt_and_high_alt_rel src/dbvf_autonomy/test/test_state_machine.py::test_preferred_tag_primary_with_sentinel_both_alts src/dbvf_autonomy/test/test_state_machine.py::test_preferred_tag_range_alt_preferred_over_alt_rel -v
```

Expected: all 4 PASS.

- [ ] **Step 4: Run the full test suite to verify no regressions**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon build --packages-select dbvf_autonomy
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```

Expected: 242 tests pass (238 existing + 4 new). The existing `test_preferred_tag_primary_when_range_alt_invalid` test (sentinel range_alt without alt_rel) still passes because `alt_rel` defaults to -1.0 (also sentinel), so the function returns primary — same behavior as before.

- [ ] **Step 5: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py src/dbvf_autonomy/test/test_state_machine.py
git commit -m "fix(ISS-017): add alt_rel fallback in compute_preferred_tag_id

When range_alt is sentinel (-1.0) — no RANGEFINDER MAVLink messages —
fall back to alt_rel (barometric altitude) for the preferred tag switch
during DESCEND_COARSE. This ensures the 4-phase landing sequence
(HOLD_ABOVE_TAG → ALIGN_YAW → OFFSET_LATERAL → DESCEND_FINAL) is
entered even without rangefinder data, fixing offsets having no effect."
```

---

### Task 3: Add RANGEFINDER stream rate to ArduPilot SITL config

**Files:**
- Modify: `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm` (append after RNGFND1 block)

- [ ] **Step 1: Add SR0_EXTRA3 parameter**

In `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm`, add after the RNGFND1 block (after `RNGFND1_ORIENT   25`):

```
# Stream rate: ensure RANGEFINDER messages are sent (EXTRA3 stream)
SR0_EXTRA3       10
```

`SR0_EXTRA3` controls the rate (Hz) of the EXTRA3 data stream for port 0, which includes RANGEFINDER messages. The mavlink_interface already sends `MAV_DATA_STREAM_ALL` at 10Hz via `request_data_stream_send`, but ArduPilot SITL may not honor that for all message types. The explicit param guarantees RANGEFINDER messages at 10Hz.

- [ ] **Step 2: Verify the parm file is valid**

```bash
cd /home/finn/Documents/ardu_ws
cat src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm
```

Expected: file ends with `SR0_EXTRA3       10` and all existing params are intact.

- [ ] **Step 3: Build ardupilot_gazebo to install the updated parm file**

```bash
colcon build --packages-select ardupilot_gazebo
```

Expected: build succeeds.

- [ ] **Step 4: Commit**

```bash
git add src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm
git commit -m "fix(ISS-017): add SR0_EXTRA3 stream rate for RANGEFINDER in SITL

Explicitly request RANGEFINDER messages at 10Hz so range_alt is
populated during precision landing simulation. Without this,
RANGEFINDER MAVLink messages may not arrive despite RNGFND1
being configured."
```

---

### Task 4: Update ISS-017 status in issues.md

**Files:**
- Modify: `src/dbvf_autonomy/docs/Features/Auto Landing April Tags/Final Positioning during landing/issues.md` (ISS-017 section)

- [ ] **Step 1: Update status and add resolution**

In `src/dbvf_autonomy/docs/Features/Auto Landing April Tags/Final Positioning during landing/issues.md`, change the ISS-017 header area from:

```markdown
**Date:** 2026-04-05
**Status:** Open — root cause identified
```

to:

```markdown
**Date:** 2026-04-05
**Status:** FIXED
**Implementation plan:** `docs/superpowers/plans/2026-04-05-iss017-rangefinder-alt-fallback.md`
```

And add a "### Resolution" section before the "### Relationship to other issues" section:

```markdown
### Resolution

Two-pronged fix:

1. **`alt_rel` fallback in `compute_preferred_tag_id()`** — when `range_alt` is sentinel (-1.0), the function now falls back to `alt_rel` (barometric altitude from `GLOBAL_POSITION_INT`). This ensures the preferred tag switches to secondary during DESCEND_COARSE even without rangefinder data, enabling the 4-phase landing sequence.

2. **`SR0_EXTRA3 10` in `gazebo-iris-hardmount.parm`** — explicitly requests RANGEFINDER messages at 10Hz in SITL. When rangefinder data is available, it is preferred over barometric altitude (more accurate near ground).

### Files changed

| File | Change |
|------|--------|
| `precision_landing_node.py:83-94` | Added `alt_rel` parameter to `compute_preferred_tag_id()`, fallback logic |
| `precision_landing_node.py:646-651` | Pass `vs.alt_rel` at call site |
| `test_state_machine.py` | 4 new tests for alt_rel fallback |
| `gazebo-iris-hardmount.parm` | Added `SR0_EXTRA3 10` |
```

- [ ] **Step 2: Commit**

```bash
git add "src/dbvf_autonomy/docs/Features/Auto Landing April Tags/Final Positioning during landing/issues.md"
git commit -m "docs(ISS-017): mark fixed, add resolution details"
```
