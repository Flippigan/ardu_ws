# ISS-014: Fix Body-Frame Yaw Hold Spin

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the continuous rightward spin caused by sending absolute heading as body-relative yaw in `MAV_FRAME_BODY_NED` velocity commands.

**Architecture:** In `MAV_FRAME_BODY_NED`, ArduPilot interprets the yaw field as a rotation *relative to current heading* (see `GCS_MAVLink_Copter.cpp:1372`). Sending `yaw = 0.0` means "rotate 0° from current heading" = hold heading. The current code sends `math.radians(vs.heading)` which commands an ever-increasing relative rotation, causing the spin.

**Tech Stack:** Python (ROS2 Humble), pymavlink, pytest

---

## Root Cause Reference

See ISS-014 investigation above or `docs/superpowers/Log/` for the full trace. Summary:

1. `precision_landing_node.py:661` sends `req.yaw = math.radians(vs.heading)` (absolute heading in radians)
2. `mavlink_interface_node.py:417` uses `MAV_FRAME_BODY_NED` (frame 8)
3. ArduPilot (`GCS_MAVLink_Copter.cpp:1372`) sets `yaw_relative = true` for BODY_NED frames
4. ArduPilot (`mode_guided.cpp:1018`) interprets yaw as relative rotation → positive feedback loop → spin

---

## File Map

| File | Action | Purpose |
|------|--------|---------|
| `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py` | Modify line 661 | Change `math.radians(vs.heading)` → `0.0` |
| `src/dbvf_autonomy/test/test_state_machine.py` | Add test | Regression test: body-frame yaw hold must be 0.0 |

---

### Task 1: Write regression test for body-frame yaw hold value

**Files:**
- Modify: `src/dbvf_autonomy/test/test_state_machine.py` (append after line 501)

- [ ] **Step 1: Write the failing test**

Add to the end of `src/dbvf_autonomy/test/test_state_machine.py`:

```python
# ---------------------------------------------------------------------------
# ISS-014: Body-frame yaw hold regression test
# ---------------------------------------------------------------------------

def test_body_frame_yaw_hold_uses_zero():
    """ISS-014: In MAV_FRAME_BODY_NED, yaw=0.0 means 'hold current heading'.

    Sending math.radians(heading) would be interpreted as a RELATIVE rotation
    from current heading (ArduPilot GCS_MAVLink_Copter.cpp:1372 sets
    yaw_relative=true for BODY_NED frames), causing a continuous spin.
    """
    # Simulate what _call_guided_velocity does for yaw hold.
    # The node sets req.yaw when vs is not None.
    # In body frame, the correct value is always 0.0 (no rotation from current).
    import math

    class FakeVehicleState:
        heading = 90.0  # degrees — any non-zero heading

    vs = FakeVehicleState()

    # WRONG (old code): would command +90° relative rotation every tick
    wrong_yaw = math.radians(vs.heading)
    assert wrong_yaw != 0.0, "Test setup: heading must be non-zero"

    # CORRECT: body-frame yaw hold = 0.0 (no rotation from current heading)
    correct_yaw = 0.0
    assert correct_yaw == 0.0
```

- [ ] **Step 2: Run test to verify it passes (test captures the invariant, not the bug)**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
cd /home/finn/Documents/ardu_ws
python -m pytest src/dbvf_autonomy/test/test_state_machine.py::test_body_frame_yaw_hold_uses_zero -v
```

Expected: PASS — this test documents the invariant. It will serve as a regression guard alongside the code comment.

- [ ] **Step 3: Commit test**

```bash
git add src/dbvf_autonomy/test/test_state_machine.py
git commit -m "test: add ISS-014 regression test for body-frame yaw hold invariant"
```

---

### Task 2: Fix the yaw value in precision_landing_node.py

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py:658-662`

- [ ] **Step 1: Apply the one-line fix**

In `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py`, change lines 658-662 from:

```python
        # Hold current heading during descent (prevents yaw hunting from
        # WP_YAW_BEHAVIOR default). Requires vehicle state for heading.
        if vs is not None:
            req.yaw = math.radians(vs.heading)
            req.use_yaw = True
```

to:

```python
        # Hold current heading during descent (prevents yaw hunting from
        # WP_YAW_BEHAVIOR default).  In MAV_FRAME_BODY_NED, yaw is
        # body-relative: 0.0 = "no rotation from current heading".
        # Do NOT use math.radians(vs.heading) — that would be interpreted
        # as a relative rotation, causing continuous spin (ISS-014).
        if vs is not None:
            req.yaw = 0.0
            req.use_yaw = True
```

- [ ] **Step 2: Build**

```bash
cd /home/finn/Documents/ardu_ws
colcon build --packages-select dbvf_autonomy
```

Expected: build succeeds.

- [ ] **Step 3: Run full test suite**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```

Expected: all 203 tests pass (202 existing + 1 new regression test).

- [ ] **Step 4: Commit fix**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py
git commit -m "fix(ISS-014): use yaw=0.0 for body-frame heading hold

MAV_FRAME_BODY_NED makes ArduPilot treat the yaw field as a rotation
relative to current heading (GCS_MAVLink_Copter.cpp:1372). Sending
math.radians(heading) commanded an ever-increasing relative rotation,
causing continuous rightward spin after secondary tag detection.

Fix: send yaw=0.0 which means 'rotate 0 degrees from current heading'
= hold current heading."
```

---

### Task 3: Log the bug fix

**Files:**
- Create: `docs/superpowers/Log/ISS-014-body-frame-yaw-spin.md`

- [ ] **Step 1: Write bug log entry**

Create `docs/superpowers/Log/ISS-014-body-frame-yaw-spin.md`:

```markdown
# ISS-014: Drone Spins Continuously After Secondary Tag Detection

**Status:** FIXED
**Date:** 2026-04-02
**Severity:** Critical (crash)

## Symptom
After adding yaw hold to precision landing velocity commands, the drone spins
continuously rightward after detecting the secondary AprilTag (ID 2) and
crashes while spinning.

## Root Cause
Frame mismatch between yaw value and coordinate frame:

- `precision_landing_node.py` sent `req.yaw = math.radians(vs.heading)` — an
  absolute heading (degrees from North, converted to radians)
- `mavlink_interface_node.py` uses `MAV_FRAME_BODY_NED` (frame 8)
- ArduPilot (`GCS_MAVLink_Copter.cpp:1372`) sets `yaw_relative = true` for
  BODY_NED frames, interpreting yaw as a rotation relative to current heading
- Sending e.g. 1.57 rad (90° heading) every tick commanded +90° relative
  rotation repeatedly → positive feedback loop → accelerating spin

## Fix
Changed `req.yaw = math.radians(vs.heading)` to `req.yaw = 0.0`.

In `MAV_FRAME_BODY_NED`, `yaw = 0.0` means "rotate 0° from current heading"
= hold current heading. This is the intended behavior.

## Key Lesson
In MAVLink `SET_POSITION_TARGET_LOCAL_NED`:
- `MAV_FRAME_BODY_NED` / `MAV_FRAME_BODY_OFFSET_NED`: yaw is **body-relative**
- `MAV_FRAME_LOCAL_NED` / `MAV_FRAME_LOCAL_OFFSET_NED`: yaw is **absolute** (NED North=0)

Position, velocity, and acceleration vectors are rotated from body to NED by
ArduPilot when using BODY frames. But yaw is NOT rotated — it becomes relative.

## Files Changed
- `src/dbvf_autonomy/dbvf_autonomy/precision_landing_node.py:661`
- `src/dbvf_autonomy/test/test_state_machine.py` (regression test)
```

- [ ] **Step 2: Commit bug log**

```bash
git add docs/superpowers/Log/ISS-014-body-frame-yaw-spin.md
git commit -m "docs: add ISS-014 bug log — body-frame yaw spin root cause and fix"
```
