# RC Mission Control & Competition Operations — Design Spec

**Date:** 2026-03-29
**Status:** Draft
**Scope:** RC channel-based mission triggers, config file improvements, competition operations guide

---

## Problem

The DBVF autonomy stack runs on an NVIDIA Orin companion computer connected to the Cube Orange flight controller via serial. At the competition, the GCS operator and pilot cannot interact with the Orin directly during flight — only through MAVLink messages relayed via the Cube Orange.

The team needs:
1. A way to **start and resume the mission** from the ground during flight using RC switches
2. A clear **config file** they can edit pre-flight to swap GPS coordinates for the competition field
3. A **step-by-step operations document** so team members who didn't write the code can operate it

MissionPlanner runs alongside as the required GCS (live map, telemetry, parameters, failsafes). This system handles only the autonomy-specific commands.

---

## Architecture

### Communication Path

```
Pilot (RC Transmitter)
    → RC Receiver → Cube Orange (ArduPilot)
        → MAVLink RC_CHANNELS message → Serial → Orin
            → mavlink_interface_node reads channels 14/15
                → Calls StartMission / ResumeMission services internally
                    → mission_sequencer_node receives service call
```

### In-Flight Interactions

| Action | Trigger | Mechanism |
|--------|---------|-----------|
| Start mission | RC channel 14 high | Edge-detected in mavlink_interface_node, calls `/dbvf/start_mission` |
| Resume after flagger | RC channel 15 high | Edge-detected in mavlink_interface_node, calls `/dbvf/resume_mission` |
| Abort mission | Pilot switches out of GUIDED mode | mission_sequencer detects mode change via `/dbvf/vehicle_state` |

### Pre-Flight Configuration

The team SSHs into the Orin while the drone is on the ground and edits `config/mission_params.yaml` with the GPS coordinates received from the competition organizers. Nodes are then restarted to load the new config.

---

## Deliverable 1: RC Channel Monitoring in mavlink_interface_node

### What Changes

Add RC channel monitoring to the existing MAVLink message processing loop in `mavlink_interface_node.py`. The node already reads every MAVLink message from the Cube Orange in `_read_timer()`.

### New Parameters

Added to the `mavlink_interface` node (declared in `__init__`, read from config or launch args):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `rc_start_channel` | `14` | RC channel that triggers StartMission |
| `rc_resume_channel` | `15` | RC channel that triggers ResumeMission |
| `rc_trigger_pwm` | `1700` | PWM threshold — trigger fires when channel crosses above this value |

### Behavior

1. **Read RC_CHANNELS messages** — In the existing `_read_timer()` message loop, add a handler for the `RC_CHANNELS` MAVLink message type. Extract the PWM values for the configured channels.

2. **Edge detection** — Maintain previous PWM state for each monitored channel. A trigger fires only on a **rising edge**: the channel transitions from below `rc_trigger_pwm` to at or above `rc_trigger_pwm`. This prevents repeated triggers while a switch is held high.

3. **Service calls** — When channel 14 rising edge is detected, call `/dbvf/start_mission` as a ROS2 service client. When channel 15 rising edge is detected, call `/dbvf/resume_mission`. These are the same services the mission_sequencer_node already serves.

4. **Safety** — The StartMission service only succeeds when the FSM is in IDLE state. The ResumeMission service only succeeds in WAIT_FLAGGER state. Accidental or repeated triggers are safe — the services reject them and return `success=False`.

5. **Logging** — Publish trigger events to `/dbvf/rc_trigger` (`std_msgs/String`) with value `"START"` or `"RESUME"` for observability. Also log at INFO level.

### RC_CHANNELS Message Details

ArduPilot sends `RC_CHANNELS` (message ID 65) at ~2Hz by default (can be increased via `SR*_RC_CHAN` stream rate). The message contains `chan1_raw` through `chan18_raw` as uint16 PWM values (typically 1000-2000, 0 if unused). Channel 14 is `chan14_raw`, channel 15 is `chan15_raw`.

### Implementation Scope

- New instance variables: `_rc_prev_ch14`, `_rc_prev_ch15` (initialized to `None` — first reading is recorded but does not trigger, preventing boot-time accidental activation if a switch is already high)
- New service clients: `start_mission_client`, `resume_mission_client` (created in `__init__`)
- New publisher: `rc_trigger_pub` (String, `/dbvf/rc_trigger`)
- New method: `_handle_rc_channels(msg)` called from `_read_timer()` when `mtype == 'RC_CHANNELS'`
- New parameters declared in `__init__`: `rc_start_channel`, `rc_resume_channel`, `rc_trigger_pwm`
- Estimated: ~40 lines of new code

### Service Client Pattern

The mavlink_interface_node will create ROS2 service clients for StartMission and ResumeMission. When an RC trigger fires, it calls the service asynchronously (non-blocking) since the `_read_timer` runs on the main executor thread. The result callback logs success/failure.

```python
# In __init__:
self.start_mission_client = self.create_client(StartMission, '/dbvf/start_mission')
self.resume_mission_client = self.create_client(ResumeMission, '/dbvf/resume_mission')

# In _handle_rc_channels:
future = self.start_mission_client.call_async(request)
future.add_done_callback(self._start_mission_done)
```

---

## Deliverable 2: Updated mission_params.yaml

### What Changes

The existing `config/mission_params.yaml` is updated with:

1. **Inline comments on every GPS parameter** — describing what the waypoint is, its zone dimensions per the RFP, and a prompt to replace for competition
2. **Commented-out competition placeholder coords** — directly below each active simulation GPS value, a commented line with Harford Airport area coordinates for the team to uncomment/replace
3. **RC trigger parameters** — new section for the mavlink_interface_node's RC channel config

### Updated File Structure

```yaml
mission_sequencer:
  ros__parameters:
    # ── GPS Waypoints ──────────────────────────────────────────────
    # Simulation defaults (Canberra, Australia).
    # For competition: replace with coords provided by organizers.
    # Format: decimal degrees (e.g., 39.56731, -76.20527)

    # Home (H) — takeoff/landing start & end point, 15x15ft zone
    home_lat: -35.3632621
    home_lon: 149.1652374
    # COMPETITION: home_lat: 39.5673
    # COMPETITION: home_lon: -76.2053

    # Landing zone (L) — FM-1 land target, 15x15ft zone with flagger
    landing_lat: -35.3640000
    landing_lon: 149.1652374
    # COMPETITION: landing_lat: <from organizers>
    # COMPETITION: landing_lon: <from organizers>

    # Water Autonomous (WA) — FM-3 precision landing pad, 20x20ft zone
    wa_lat: -35.3632531
    wa_lon: 149.1657896
    # COMPETITION: wa_lat: <from organizers>
    # COMPETITION: wa_lon: <from organizers>

    # Fire 1 (F1) — drop zone, 7x7ft, +2.5pts/payload
    f1_lat: -35.3650000
    f1_lon: 149.1652374
    # COMPETITION: f1_lat: <from organizers>
    # COMPETITION: f1_lon: <from organizers>

    # Fire 2 (F2) — drop zone, 3x3ft, +5pts/payload
    f2_lat: -35.3660000
    f2_lon: 149.1652374
    # COMPETITION: f2_lat: <from organizers>
    # COMPETITION: f2_lon: <from organizers>

    # ── Flight Parameters ──────────────────────────────────────────
    transit_altitude_ft: 35.0        # Must be >= 30ft AGL per RFP
    position_tolerance_m: 3.0       # GPS accuracy limit
    takeoff_complete_alt_ft: 33.0   # Altitude to consider takeoff complete

    # ── Payload Servo ──────────────────────────────────────────────
    drop_servo_number: 9
    drop_servo_pwm_release: 1100
    drop_servo_pwm_hold: 1500
    drop_settle_time_s: 2.0

    # ── Drop Zone Target ──────────────────────────────────────────
    # "F1" (7x7ft, +2.5pts) or "F2" (3x3ft, +5pts)
    drop_target: "F1"

    # ── Safety ─────────────────────────────────────────────────────
    mission_timeout_s: 540.0        # 9 min (RFP allows 10 min per attempt)
    heartbeat_loss_timeout_s: 5.0
    service_call_timeout_s: 5.0
    guided_resend_interval_s: 1.0

    # ── Altitude Source ────────────────────────────────────────────
    prefer_rangefinder: true

mavlink_interface:
  ros__parameters:
    # ── RC Mission Triggers ────────────────────────────────────────
    # RC channel numbers (1-18) for mission control switches
    rc_start_channel: 14            # Flip high to start mission (IDLE → PREFLIGHT)
    rc_resume_channel: 15           # Flip high to resume after flagger (WAIT_FLAGGER → TAKEOFF_L)
    rc_trigger_pwm: 1700            # PWM threshold for rising-edge detection
```

### Key Design Decisions

- **Simulation coords stay unchanged** — Australian defaults remain active. Competition coords are commented-out placeholders.
- **No WM waypoint** — the mission sequencer doesn't navigate to Water Manual, so it's not in the config.
- **RC params live under `mavlink_interface` namespace** — they belong to that node, not the mission sequencer.
- **COMPETITION comment prefix** — easy to grep for when switching configs: `grep COMPETITION mission_params.yaml`.

---

## Deliverable 3: Competition Operations Document

### Location

`src/dbvf_autonomy/docs/competition_operations.md`

### Contents

#### 1. Pre-Flight Setup (on the ground, before arming)

Step-by-step procedure:
1. Power on the drone (flight controller battery only — propulsion disconnected)
2. Connect laptop to Orin via SSH: `ssh dbvf@<orin-ip>`
3. Navigate to config: `cd ~/ardu_ws/src/dbvf_autonomy/config/`
4. Edit `mission_params.yaml` with received GPS coordinates:
   - Replace `home_lat`/`home_lon` with Home (H) coordinates
   - Replace `landing_lat`/`landing_lon` with Landing zone (L) coordinates
   - Replace `wa_lat`/`wa_lon` with Water Autonomous (WA) coordinates
   - Replace `f1_lat`/`f1_lon` with Fire 1 (F1) coordinates
   - Replace `f2_lat`/`f2_lon` with Fire 2 (F2) coordinates
   - Set `drop_target` to `"F1"` or `"F2"` based on strategy
5. Verify coordinates: `grep -E 'lat|lon' mission_params.yaml`
6. Restart autonomy nodes: `ros2 launch dbvf_autonomy mission_sim.launch.py` (or competition launch file)
7. Verify nodes are running: `ros2 node list | grep dbvf`
8. Verify params loaded: `ros2 param get /mission_sequencer home_lat`

#### 2. RC Switch Mapping

| Switch | RC Channel | Action | When to Use |
|--------|-----------|--------|-------------|
| [TBD switch on transmitter] | 14 | Start mission | After arming in GUIDED mode, when ready to begin |
| [TBD switch on transmitter] | 15 | Resume mission | After flagger raises flag at L, approving second takeoff |
| Mode switch | — | Abort (switch out of GUIDED) | Emergency — pilot takes manual control |

**Important:** Switches must be momentary or start in the LOW position. The system triggers on the LOW→HIGH transition. Holding a switch high does not re-trigger.

#### 3. Mission Flow

What happens at each phase, what the pilot/GCS operator should observe, and what actions are needed:

| Phase | What Happens | Pilot Action | Expected Duration |
|-------|-------------|-------------|-------------------|
| Pre-arm | Drone on ground at H, nodes running | Arm in GUIDED mode via MissionPlanner or RC | — |
| FM-1 Start | Flip channel 14 HIGH | None — hands off controls | — |
| FM-1 Takeoff | Drone climbs to 35ft at H | Observe vertical climb | ~10s |
| FM-1 Transit | Drone flies to L at 35ft | Observe horizontal flight | ~5s |
| FM-1 Land | Drone lands at L | Observe landing | ~10s |
| Wait Flagger | Drone idle on ground at L | Wait for flagger to raise flag | Variable |
| FM-2 Start | Flip channel 15 HIGH | None — hands off controls | — |
| FM-2 Takeoff | Drone climbs from L to 35ft | Observe vertical climb | ~10s |
| FM-2 Transit | Drone flies to F1 or F2 | Observe horizontal flight | ~10-20s |
| FM-2 Drop | Servo releases red payload | Observe payload release | ~2s |
| FM-3 Transit | Drone flies to WA | Observe horizontal flight | ~10s |
| FM-3 Land | Precision landing on AprilTag at WA | Observe slow descent | ~30s |
| FM-3 Takeoff | Drone climbs from WA to 35ft | Observe vertical climb | ~10s |
| FM-3 Drop | Servo releases yellow payload at F1/F2 | Observe payload release | ~2s |
| RTH | Drone flies back to H | Observe horizontal flight | ~10-20s |
| Land H | Drone lands at H | Observe landing | ~10s |
| Complete | Mission done, drone on ground | Disarm via RC or MissionPlanner | — |

#### 4. Abort Procedure

- **Normal abort:** Pilot switches flight mode out of GUIDED on the RC transmitter. The mission sequencer detects the mode change and enters ABORT state. The drone follows ArduPilot's failsafe behavior for the selected mode (e.g., LAND = descend vertically, RTL = fly home).
- **Emergency kill:** Pull the propulsion battery plug (physical kill switch on airframe).
- **GCS abort:** Call abort service via terminal if available: `ros2 service call /dbvf/abort_mission dbvf_msgs/srv/AbortMission "{reason: 'Operator abort'}"`

#### 5. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Channel 14 switch doesn't start mission | FSM not in IDLE state | Check `/dbvf/mission_state` — must show `IDLE` |
| Channel 14 switch doesn't start mission | RC channel not mapped correctly | Verify channel 14 on transmitter, check `ros2 topic echo /dbvf/rc_trigger` |
| Channel 15 switch doesn't resume | FSM not in WAIT_FLAGGER | Check `/dbvf/mission_state` — must show `WAIT_FLAGGER` |
| Drone doesn't takeoff after start | Not armed or not in GUIDED mode | Arm and set GUIDED via MissionPlanner first |
| Wrong GPS coordinates | Config not reloaded | Restart nodes after editing YAML |
| No heartbeat | Orin not connected to Cube Orange | Check serial cable, verify `ros2 topic echo /dbvf/heartbeat_status` |
| Mission times out | 9-minute timeout exceeded | Check `mission_timeout_s` parameter |

#### 6. GPS Coordinate Format

Coordinates from organizers should be in **decimal degrees** (e.g., `39.56731, -76.20527`). If received in a different format:
- **Degrees Minutes Seconds (DMS):** Convert using an online tool or: `decimal = degrees + minutes/60 + seconds/3600`
- **Degrees Decimal Minutes (DDM):** Convert: `decimal = degrees + decimal_minutes/60`
- **UTM:** Use an online converter to get decimal degrees

The competition is at Harford Airport, Churchville, MD. Coordinates will be approximately: latitude ~39.567°, longitude ~-76.205°.

---

## Files Changed

| File | Change |
|------|--------|
| `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py` | Add RC_CHANNELS handler, edge detection, service clients, rc_trigger publisher (~40 lines) |
| `src/dbvf_autonomy/config/mission_params.yaml` | Add comments, competition placeholders, RC trigger params |
| `src/dbvf_autonomy/docs/competition_operations.md` | New file — competition day operations guide |

## Files With Minor Changes

| File | Change |
|------|--------|
| `src/dbvf_autonomy/launch/mission_sim.launch.py` | Load `mission_params.yaml` for the `mavlink_interface` node (currently only loaded for `mission_sequencer`) |

## Files NOT Changed

- `mission_sequencer_node.py` — no changes, existing services handle triggers
- `mission_state_machine.py` — no changes to FSM logic
- Message/service definitions in `dbvf_msgs` — no new messages needed
- Gazebo worlds or models — simulation layout stays as-is

---

## Testing

### RC Trigger Unit Tests

Add to existing test suite (`test/test_rc_trigger.py`):

1. **Rising edge detection** — verify trigger fires when PWM goes from 1000 → 1800
2. **No trigger on sustained high** — verify no trigger when PWM stays at 1800
3. **No trigger below threshold** — verify no trigger at PWM 1600 (below 1700 threshold)
4. **Falling edge no trigger** — verify no trigger when PWM goes from 1800 → 1000
5. **Re-trigger after reset** — verify trigger fires again after channel goes low then high again
6. **Channel mapping** — verify correct service is called for channel 14 vs 15
7. **Configurable threshold** — verify custom `rc_trigger_pwm` values are respected

Tests should be pure-Python (no ROS2 runtime needed), testing the edge detection logic as a pure function.

### Manual Integration Test

In simulation:
1. Launch full mission stack
2. Use MAVProxy to inject RC channel overrides: `rc 14 1800` (should trigger start)
3. Wait for WAIT_FLAGGER state
4. `rc 15 1800` (should trigger resume)
5. Verify full mission completes

---

## Open Questions

None — scope is fully defined.
