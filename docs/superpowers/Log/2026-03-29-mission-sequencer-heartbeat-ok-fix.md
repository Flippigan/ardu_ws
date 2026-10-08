# ISS: mission_sequencer_node crashes on first control loop tick

**Date:** 2026-03-29
**Component:** `dbvf_autonomy/mission_sequencer_node.py`
**Severity:** Blocker — node crashes immediately after `start_mission` service call

## Symptom

After launching `mission_sim.launch.py` and calling `/dbvf/start_mission`, the `mission_sequencer_node` crashes with:

```
AttributeError: 'VehicleState' object has no attribute 'heartbeat_ok'
```

Full traceback:

```
[mission_sequencer_node-6] Traceback (most recent call last):
  File "mission_sequencer_node.py", line 281, in main
    rclpy.spin(node)
  ...
  File "mission_sequencer_node.py", line 160, in _control_loop
    vs.heartbeat_ok = (time.time() - self._last_heartbeat_time) < hb_timeout
AttributeError: 'VehicleState' object has no attribute 'heartbeat_ok'
```

## Root Cause

The `_control_loop` method attempted to set `heartbeat_ok` as a dynamic attribute on a `VehicleState` ROS2 message object (`vs.heartbeat_ok = ...`). ROS2 message classes are generated with `__slots__` — they do not allow setting attributes beyond the fields defined in the `.msg` file. `heartbeat_ok` is not a field in `VehicleState.msg`.

The pure-Python unit tests did not catch this because `MockVehicleState` is a plain Python class that allows arbitrary attribute assignment.

## Fix

Wrapped the ROS2 `VehicleState` message fields into a lightweight plain Python object (`_VState`) before passing to the FSM's `update()` method. This object includes all fields the FSM reads (`lat`, `lon`, `alt_rel`, `armed`, `mode`, `vz`, `range_alt`) plus the computed `heartbeat_ok` flag.

**File changed:** `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py` — `_control_loop()` method

## Lessons

- ROS2 message objects use `__slots__` and reject dynamic attributes. Never assign fields that aren't in the `.msg` definition.
- Pure-Python test mocks (plain classes) don't reproduce this constraint. Integration testing or ROS-aware mocks would have caught this earlier.
