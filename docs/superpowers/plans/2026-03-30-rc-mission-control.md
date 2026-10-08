# RC Mission Control Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add RC channel-based mission triggers (start/resume via RC switches), improve mission config for competition use, and create a competition operations guide.

**Architecture:** The existing `mavlink_interface_node` already reads every MAVLink message in `_read_timer()`. We add a handler for `RC_CHANNELS` messages that edge-detects channels 14/15 crossing a PWM threshold, then calls the existing `StartMission`/`ResumeMission` ROS2 services. Edge detection logic is extracted as a pure function for testability.

**Tech Stack:** Python 3, ROS2 Humble, pymavlink, pytest (via ament_cmake_pytest)

---

### Task 1: Edge Detection Pure Functions + Tests (TDD)

**Files:**
- Create: `src/dbvf_autonomy/test/test_rc_trigger.py`
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py` (add 2 pure functions after `build_landing_target_params`, before the class)

- [ ] **Step 1: Write the test file**

Create `src/dbvf_autonomy/test/test_rc_trigger.py`:

```python
from dbvf_autonomy.mavlink_interface_node import (
    detect_rc_rising_edge,
    get_rc_channel_pwm,
)


# -- Rising edge detection ---------------------------------------------------

def test_rising_edge_fires():
    """Trigger fires when PWM crosses from below to at/above threshold."""
    assert detect_rc_rising_edge(1800, 1000, 1700) is True


def test_sustained_high_does_not_refire():
    """No trigger when PWM stays above threshold."""
    assert detect_rc_rising_edge(1800, 1800, 1700) is False


def test_below_threshold_does_not_fire():
    """No trigger when PWM is below threshold."""
    assert detect_rc_rising_edge(1600, 1000, 1700) is False


def test_falling_edge_does_not_fire():
    """No trigger when PWM drops from above to below threshold."""
    assert detect_rc_rising_edge(1000, 1800, 1700) is False


def test_no_trigger_on_first_reading():
    """First reading (prev=None) records but does not trigger — boot safety."""
    assert detect_rc_rising_edge(1800, None, 1700) is False


def test_retrigger_after_reset():
    """Trigger fires again after channel goes low then high."""
    # First: high (from low) → fires
    assert detect_rc_rising_edge(1800, 1000, 1700) is True
    # Channel goes low — no trigger
    assert detect_rc_rising_edge(1000, 1800, 1700) is False
    # Channel goes high again — fires
    assert detect_rc_rising_edge(1800, 1000, 1700) is True


def test_exact_threshold_fires():
    """PWM exactly at threshold counts as high."""
    assert detect_rc_rising_edge(1700, 1000, 1700) is True


def test_one_below_threshold_does_not_fire():
    """PWM one below threshold does not fire."""
    assert detect_rc_rising_edge(1699, 1000, 1700) is False


def test_configurable_threshold():
    """Custom threshold values are respected."""
    # Threshold 1500: 1600 is above
    assert detect_rc_rising_edge(1600, 1000, 1500) is True
    # Threshold 1500: 1400 is below
    assert detect_rc_rising_edge(1400, 1000, 1500) is False


# -- Channel PWM extraction --------------------------------------------------

def test_get_channel_14():
    """Extract channel 14 PWM from RC_CHANNELS message."""
    class FakeMsg:
        chan14_raw = 1800
    assert get_rc_channel_pwm(FakeMsg(), 14) == 1800


def test_get_channel_15():
    """Extract channel 15 PWM from RC_CHANNELS message."""
    class FakeMsg:
        chan15_raw = 1200
    assert get_rc_channel_pwm(FakeMsg(), 15) == 1200


def test_get_channel_1():
    """Extract channel 1 PWM from RC_CHANNELS message."""
    class FakeMsg:
        chan1_raw = 1500
    assert get_rc_channel_pwm(FakeMsg(), 1) == 1500
```

- [ ] **Step 2: Run tests to verify they fail**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python -m pytest src/dbvf_autonomy/test/test_rc_trigger.py -v
```
Expected: FAIL with `ImportError: cannot import name 'detect_rc_rising_edge'`

- [ ] **Step 3: Implement the pure functions**

In `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`, add after `build_landing_target_params` (after line 54), before the `class MavlinkInterfaceNode` definition:

```python
def detect_rc_rising_edge(current_pwm, prev_pwm, threshold):
    """Return True if PWM crossed above threshold (rising edge).

    Returns False if prev_pwm is None (first reading — boot safety).
    """
    if prev_pwm is None:
        return False
    return current_pwm >= threshold and prev_pwm < threshold


def get_rc_channel_pwm(msg, channel):
    """Extract PWM value for a specific RC channel (1-18) from RC_CHANNELS message."""
    return getattr(msg, f'chan{channel}_raw')
```

- [ ] **Step 4: Run tests to verify they pass**

Run:
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python -m pytest src/dbvf_autonomy/test/test_rc_trigger.py -v
```
Expected: All 13 tests PASS

- [ ] **Step 5: Commit**

```bash
git add src/dbvf_autonomy/test/test_rc_trigger.py src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py
git commit -m "feat: add RC edge detection pure functions with tests (TDD)"
```

---

### Task 2: RC Monitoring Integration in mavlink_interface_node

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`

This task adds the ROS2 integration: parameters, service clients, publisher, state variables, and the `_handle_rc_channels` method. No new tests — the pure logic is already tested in Task 1; the integration wires it into the node.

- [ ] **Step 1: Add imports**

In `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`, update the `dbvf_msgs.srv` import line (line 11):

Change:
```python
from dbvf_msgs.srv import SetMode, ArmMotors, SendGuidedPosition, SendGuidedVelocity, DoSetServo, Takeoff
```
To:
```python
from dbvf_msgs.srv import SetMode, ArmMotors, SendGuidedPosition, SendGuidedVelocity, DoSetServo, Takeoff, StartMission, ResumeMission
```

Also add `String` to the `std_msgs` import (line 8):

Change:
```python
from std_msgs.msg import Bool
```
To:
```python
from std_msgs.msg import Bool, String
```

- [ ] **Step 2: Add parameters, service clients, publisher, and state variables to `__init__`**

In `__init__`, after the existing `declare_parameter` calls (after line 65), add:

```python
        self.declare_parameter('rc_start_channel', 14)
        self.declare_parameter('rc_resume_channel', 15)
        self.declare_parameter('rc_trigger_pwm', 1700)
```

After the existing publishers section (after line 85), add:

```python
        self.rc_trigger_pub = self.create_publisher(String, '/dbvf/rc_trigger', 10)
```

After the existing service definitions (after line 103), add:

```python
        # RC trigger service clients
        self.start_mission_client = self.create_client(
            StartMission, '/dbvf/start_mission')
        self.resume_mission_client = self.create_client(
            ResumeMission, '/dbvf/resume_mission')
```

In the vehicle state cache section (after `self.last_heartbeat_time = 0.0` on line 81), add:

```python
        # RC channel edge detection state (None = no reading yet)
        self._rc_prev = {}
```

- [ ] **Step 3: Add `_handle_rc_channels` method and done callbacks**

After `_read_timer` method (after line 196), add:

```python
    def _handle_rc_channels(self, msg):
        """Process RC_CHANNELS message for mission trigger edge detection."""
        threshold = self.get_parameter('rc_trigger_pwm').value
        triggers = [
            (self.get_parameter('rc_start_channel').value,
             self.start_mission_client, 'START', StartMission.Request()),
            (self.get_parameter('rc_resume_channel').value,
             self.resume_mission_client, 'RESUME', ResumeMission.Request()),
        ]
        for channel, client, label, request in triggers:
            current = get_rc_channel_pwm(msg, channel)
            prev = self._rc_prev.get(channel)
            if detect_rc_rising_edge(current, prev, threshold):
                self.get_logger().info(
                    f'RC trigger: {label} (ch{channel} PWM={current})')
                trigger_msg = String()
                trigger_msg.data = label
                self.rc_trigger_pub.publish(trigger_msg)
                future = client.call_async(request)
                future.add_done_callback(
                    lambda f, l=label: self._rc_service_done(f, l))
            self._rc_prev[channel] = current

    def _rc_service_done(self, future, label):
        """Log result of RC-triggered service call."""
        try:
            result = future.result()
            if result.success:
                self.get_logger().info(f'RC {label}: {result.message}')
            else:
                self.get_logger().warn(f'RC {label} rejected: {result.message}')
        except Exception as e:
            self.get_logger().warn(f'RC {label} service call failed: {e}')
```

- [ ] **Step 4: Add RC_CHANNELS case to `_read_timer`**

In the `_read_timer` method, after the `elif mtype == 'RANGEFINDER':` block (after line 182), add:

```python
                elif mtype == 'RC_CHANNELS':
                    self._handle_rc_channels(msg)
```

- [ ] **Step 5: Build and verify**

Run:
```bash
colcon build --packages-select dbvf_autonomy
source install/setup.bash
```
Expected: Build succeeds with no errors.

- [ ] **Step 6: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py
git commit -m "feat: add RC channel monitoring for mission start/resume triggers"
```

---

### Task 3: Register Test in CMakeLists.txt

**Files:**
- Modify: `src/dbvf_autonomy/CMakeLists.txt`

- [ ] **Step 1: Add test registration**

In `src/dbvf_autonomy/CMakeLists.txt`, after the last `ament_add_pytest_test` line (line 43, `test_mission_fsm`), add:

```cmake
  ament_add_pytest_test(test_rc_trigger test/test_rc_trigger.py)
```

- [ ] **Step 2: Build and run tests**

Run:
```bash
colcon build --packages-select dbvf_autonomy
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```
Expected: All 164 tests pass (151 existing + 13 new).

- [ ] **Step 3: Commit**

```bash
git add src/dbvf_autonomy/CMakeLists.txt
git commit -m "test: register RC trigger tests in CMakeLists.txt"
```

---

### Task 4: Update mission_params.yaml

**Files:**
- Modify: `src/dbvf_autonomy/config/mission_params.yaml`

- [ ] **Step 1: Update the config file with comments, placeholders, and RC params**

Replace the full contents of `src/dbvf_autonomy/config/mission_params.yaml` with:

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
    # COMPETITION: home_lat: <from organizers>
    # COMPETITION: home_lon: <from organizers>

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

- [ ] **Step 2: Verify YAML is valid**

Run:
```bash
python3 -c "import yaml; yaml.safe_load(open('src/dbvf_autonomy/config/mission_params.yaml')); print('YAML OK')"
```
Expected: `YAML OK`

- [ ] **Step 3: Commit**

```bash
git add src/dbvf_autonomy/config/mission_params.yaml
git commit -m "docs: add GPS comments, competition placeholders, and RC trigger params to mission config"
```

---

### Task 5: Update mission_sim.launch.py

**Files:**
- Modify: `src/dbvf_autonomy/launch/mission_sim.launch.py`

- [ ] **Step 1: Add mission_config to mavlink_interface node**

In `src/dbvf_autonomy/launch/mission_sim.launch.py`, change the mavlink_interface Node (lines 47-51) from:

```python
        # MAVLink interface
        Node(
            package='dbvf_autonomy',
            executable='mavlink_interface_node',
            name='mavlink_interface',
            parameters=[sim_config],
        ),
```

To:

```python
        # MAVLink interface
        Node(
            package='dbvf_autonomy',
            executable='mavlink_interface_node',
            name='mavlink_interface',
            parameters=[sim_config, mission_config],
        ),
```

- [ ] **Step 2: Build and verify**

Run:
```bash
colcon build --packages-select dbvf_autonomy
```
Expected: Build succeeds.

- [ ] **Step 3: Commit**

```bash
git add src/dbvf_autonomy/launch/mission_sim.launch.py
git commit -m "fix: load mission_params.yaml for mavlink_interface node (RC trigger params)"
```

---

### Task 6: Create Competition Operations Document

**Files:**
- Create: `src/dbvf_autonomy/docs/competition_operations.md`

- [ ] **Step 1: Write the operations document**

Create `src/dbvf_autonomy/docs/competition_operations.md`:

```markdown
# DBVF Competition Operations Guide

Step-by-step guide for operating the autonomy stack at the VFS DBVF competition.

**Competition Location:** Harford Airport, Churchville, MD (~39.567°N, 76.205°W)

---

## 1. Pre-Flight Setup (On the Ground, Before Arming)

1. Power on the drone (flight controller battery only — propulsion disconnected)
2. Connect laptop to Orin via SSH: `ssh dbvf@<orin-ip>`
3. Navigate to config:
   ```bash
   cd ~/ardu_ws/src/dbvf_autonomy/config/
   ```
4. Edit `mission_params.yaml` with coordinates received from organizers:
   - Replace `home_lat` / `home_lon` → Home (H) coordinates
   - Replace `landing_lat` / `landing_lon` → Landing zone (L) coordinates
   - Replace `wa_lat` / `wa_lon` → Water Autonomous (WA) coordinates
   - Replace `f1_lat` / `f1_lon` → Fire 1 (F1) coordinates
   - Replace `f2_lat` / `f2_lon` → Fire 2 (F2) coordinates
   - Set `drop_target` → `"F1"` or `"F2"` based on team strategy
5. Verify coordinates:
   ```bash
   grep -E 'lat|lon' mission_params.yaml
   ```
6. Build and launch:
   ```bash
   cd ~/ardu_ws
   colcon build --packages-select dbvf_autonomy
   source install/setup.bash
   ros2 launch dbvf_autonomy mission_sim.launch.py
   ```
7. Verify nodes are running:
   ```bash
   ros2 node list | grep dbvf
   ```
8. Verify parameters loaded:
   ```bash
   ros2 param get /mission_sequencer home_lat
   ros2 param get /mavlink_interface rc_start_channel
   ```

---

## 2. RC Switch Mapping

| Switch | RC Channel | Action | When to Use |
|--------|-----------|--------|-------------|
| [TBD — assign on transmitter] | 14 | Start mission | After arming in GUIDED mode, when ready to begin |
| [TBD — assign on transmitter] | 15 | Resume mission | After flagger raises flag at L, approving second takeoff |
| Mode switch | — | Abort (switch out of GUIDED) | Emergency — pilot takes manual control |

**Important:** Switches must be momentary or start in the LOW position. The system triggers on the LOW→HIGH transition (PWM crosses above 1700). Holding a switch high does not re-trigger.

---

## 3. Mission Flow

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

---

## 4. Abort Procedure

- **Normal abort:** Pilot switches flight mode out of GUIDED on the RC transmitter. The mission sequencer detects the mode change and enters ABORT state. The drone follows ArduPilot's failsafe behavior for the selected mode (e.g., LAND = descend vertically, RTL = fly home).
- **Emergency kill:** Pull the propulsion battery plug (physical kill switch on airframe).
- **GCS abort:** Call abort service via terminal if available:
  ```bash
  ros2 service call /dbvf/abort_mission dbvf_msgs/srv/AbortMission "{reason: 'Operator abort'}"
  ```

---

## 5. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Ch14 switch doesn't start mission | FSM not in IDLE state | Check `ros2 topic echo /dbvf/mission_state` — must show `IDLE` |
| Ch14 switch doesn't start mission | RC channel not mapped correctly | Verify ch14 on transmitter, check `ros2 topic echo /dbvf/rc_trigger` |
| Ch15 switch doesn't resume | FSM not in WAIT_FLAGGER | Check `ros2 topic echo /dbvf/mission_state` — must show `WAIT_FLAGGER` |
| Drone doesn't takeoff after start | Not armed or not in GUIDED mode | Arm and set GUIDED via MissionPlanner first |
| Wrong GPS coordinates | Config not reloaded | Rebuild (`colcon build`) and restart nodes after editing YAML |
| No heartbeat | Orin not connected to Cube Orange | Check serial cable, verify `ros2 topic echo /dbvf/heartbeat_status` |
| Mission times out | 9-minute timeout exceeded | Check `mission_timeout_s` parameter |

---

## 6. GPS Coordinate Format

Coordinates from organizers must be in **decimal degrees** (e.g., `39.56731, -76.20527`).

If received in a different format:
- **Degrees Minutes Seconds (DMS):** `decimal = degrees + minutes/60 + seconds/3600`
- **Degrees Decimal Minutes (DDM):** `decimal = degrees + decimal_minutes/60`
- **UTM:** Use an online converter to get decimal degrees

**Sanity check:** Competition coordinates should be approximately latitude ~39.567°, longitude ~-76.205° (Harford Airport area).

---

## 7. Monitoring During Flight

```bash
# Vehicle telemetry (position, altitude, mode)
ros2 topic echo /dbvf/vehicle_state

# Mission FSM state
ros2 topic echo /dbvf/mission_state

# Mission phase (FM1, FM2, FM3, RTH)
ros2 topic echo /dbvf/mission_phase

# RC trigger events
ros2 topic echo /dbvf/rc_trigger

# AprilTag detection status
ros2 topic echo /dbvf/tag_status

# Precision landing state (during WA landing)
ros2 topic echo /dbvf/landing_state
```
```

- [ ] **Step 2: Commit**

```bash
git add src/dbvf_autonomy/docs/competition_operations.md
git commit -m "docs: add competition operations guide for DBVF field day"
```

---

### Summary of Changes

| Deliverable | Files | Tests |
|-------------|-------|-------|
| RC channel monitoring | `mavlink_interface_node.py` (~40 lines new code) | 13 new tests in `test_rc_trigger.py` |
| Mission config improvements | `mission_params.yaml` (comments + RC params) | — |
| Competition operations guide | `docs/competition_operations.md` (new) | — |
| Launch file fix | `mission_sim.launch.py` (1 line) | — |
| Test registration | `CMakeLists.txt` (1 line) | — |
