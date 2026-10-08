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
