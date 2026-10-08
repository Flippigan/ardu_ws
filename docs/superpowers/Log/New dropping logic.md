# New Dropping Logic — Bug Log

## Branch: `feat/wa-reload-mechanism`

---

### BUG-001: Drone does not take off — immediate TAKEOFF_H -> TRANSIT_H_TO_L

**Date:** 2026-03-31
**Severity:** Critical
**Status:** Open

**Symptom:**
After calling `/dbvf/start_mission`, the mission sequencer immediately transitions through `PREFLIGHT_CHECK -> TAKEOFF_H -> TRANSIT_H_TO_L` without the drone actually leaving the ground. The takeoff service responds "Takeoff to 10.7m" but the drone remains stationary. The mission then stays stuck in `TRANSIT_H_TO_L` indefinitely.

**Observed log sequence:**
```
[mission_sequencer_node-7] Starting mission
[mission_sequencer_node-7] PREFLIGHT_CHECK -> TAKEOFF_H (preflight_pass)
[mission_sequencer_node-7] Takeoff: Takeoff to 10.7m
[mission_sequencer_node-7] TAKEOFF_H -> TRANSIT_H_TO_L (takeoff_complete)
```

The transition from `TAKEOFF_H` to `TRANSIT_H_TO_L` happens within ~100ms of the takeoff command — far too fast for the drone to actually reach altitude. The FSM thinks takeoff is complete while the drone is still on the ground.

**Other observations:**
- Arduino serial warning is expected (no Arduino in sim): `could not open port /dev/ttyACM0`
- All nodes start successfully
- MAVLink connection established (`sysid=1 compid=1`)
- After getting stuck in `TRANSIT_H_TO_L`, the state never changes

**Reproduction:**
1. Launch Gazebo + SITL: `ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true`
2. Launch mission stack: `ros2 launch dbvf_autonomy mission_sim.launch.py`
3. In MAVProxy: `mode guided`, `arm throttle`
4. Start mission: `ros2 service call /dbvf/start_mission dbvf_msgs/srv/StartMission "{}"`

**Likely root cause (not yet investigated):**
The `TAKEOFF_H` state checks `alt >= takeoff_complete_alt_m` (33ft / ~10.1m). If the vehicle state altitude is being reported incorrectly (e.g., stale data, wrong field, or rangefinder returning a positive value on the ground), the FSM could immediately think it has reached altitude. This is a pre-existing issue — the WA reload changes did not modify takeoff logic.

**Note:** This bug may not be related to the WA reload mechanism changes. The takeoff/transit logic was not modified in this branch. Needs investigation to determine if this is a regression or pre-existing.

**Root cause (confirmed 2026-03-31):**

The Gazebo `gpu_lidar` rangefinder sensor is not publishing (`/rangefinder` topic has no publisher — ogre2 rendering failing in server mode). Confirmed `range_alt = 60.0` on the ground via `ros2 topic echo /dbvf/vehicle_state --field range_alt --once`.

**Full data flow (traced through 6 layers):**

1. **Gazebo gpu_lidar** (`iris_with_standoffs/model.sdf:177`) — sensor not publishing, no data on `/rangefinder`
2. **ArduPilotPlugin RangeCb** (`ArduPilotPlugin.cc:286-304`) — initializes `sample_min = 2 * range_max = 60.0`, never receives a real reading to replace it. Sends `"rng_1": 60.0` in JSON to SITL (`ArduPilotPlugin.cc:1956-1957`)
3. **SITL JSON backend** (`SIM_JSON.h:125`, `SIM_JSON.cpp:348`) — `"rng_1"` → `state.rng[0]` → `rangefinder_m[0] = 60.0`
4. **AP_RangeFinder_SITL** (`AP_RangeFinder_SITL.cpp:33-48`) — `get_rangefinder(0)` returns 60.0; `update_status()` (`AP_RangeFinder_Backend.cpp:60-70`) marks `OutOfRangeHigh` but does NOT clamp `distance_m`
5. **mavlink_interface_node** (`mavlink_interface_node.py:209-210`) — stores `msg.distance = 60.0` into `self.range_alt`, publishes as `VehicleState.range_alt = 60.0`
6. **mission_state_machine** (`mission_state_machine.py:378-382`) — `_get_altitude()` checks `range_alt >= 0.0` (sentinel filter only), returns 60.0. `_takeoff_h()` line 178: `60.0 >= 10.06` → immediate transition

**Two independent problems:**

1. **gpu_lidar not working** — ogre2 rendering context issue in Gazebo server mode. External to dbvf_autonomy.
2. **`_get_altitude()` has no upper-bound check** — accepts any `range_alt >= 0.0`, including implausible 60m on the ground. The sentinel check (`>= 0.0`) was designed to filter -1.0, but doesn't reject out-of-range values.

**ArduPilotPlugin index mapping (verified correct):**
- SDF `<index>1</index>` is 1-based → plugin uses `index - 1 = 0` internally (`ArduPilotPlugin.cc:971`)
- `ranges[0]` → JSON `"rng_1"` → SITL `rangefinder_m[0]` → `RNGFND1` instance 0
- The data path works correctly when the lidar actually publishes

**Relevant files:**

| File | Relevance |
|------|-----------|
| `src/dbvf_autonomy/dbvf_autonomy/mission_state_machine.py` | `_takeoff_h()` — checks `alt >= takeoff_complete_alt_m`, `_get_altitude()` — selects rangefinder vs alt_rel |
| `src/dbvf_autonomy/dbvf_autonomy/mission_sequencer_node.py` | `_control_loop()` — wraps VehicleState into `_VState`, `_call_takeoff()` — sends takeoff command, `_execute_action()` — dispatches `'takeoff'` entry action |
| `src/dbvf_autonomy/dbvf_autonomy/mission_helpers.py` | `ft_to_m()` — converts `takeoff_complete_alt_ft` (33ft) to meters (~10.1m) |
| `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py` | Publishes `/dbvf/vehicle_state` (alt_rel, range_alt fields), handles `/dbvf/takeoff` service |
| `src/dbvf_autonomy/config/mission_params.yaml` | `takeoff_complete_alt_ft: 33.0`, `transit_altitude_ft: 35.0`, `prefer_rangefinder: true` |
| `src/dbvf_autonomy/config/sim_params.yaml` | MAVLink connection params, precision landing params |
| `src/dbvf_autonomy/launch/mission_sim.launch.py` | Launch file used to reproduce |
| `src/dbvf_autonomy/test/test_mission_state_machine.py` | `test_takeoff_h_*` tests — existing tests for takeoff FSM transitions |
| `src/dbvf_msgs/msg/VehicleState.msg` | Message definition for alt_rel, range_alt fields |
| `src/ardupilot_gazebo/src/ArduPilotPlugin.cc` | RangeCb (line 286), JSON serialization (line 1956) |
| `src/ardupilot/libraries/AP_RangeFinder/AP_RangeFinder_SITL.cpp` | SITL rangefinder backend |
| `src/ardupilot/libraries/AP_RangeFinder/AP_RangeFinder_Backend.cpp` | `update_status()` — sets status, doesn't clamp distance |
| `src/ardupilot/libraries/SITL/SIM_JSON.h` | JSON field mapping: `"rng_1"` → `state.rng[0]` |
| `src/ardupilot_gazebo/config/gazebo-iris-hardmount.parm` | `RNGFND1_MAX_CM 3000` (30m max range) |
