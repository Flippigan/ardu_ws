# RC Mission Control — Subagent Dispatch Prompt

> **Instructions:** Paste everything below the `---` into a new Claude Code session (or this one) to execute the plan. All subagents use Opus 4.6 (`model: "opus"`). Do NOT modify the plan file — it is the source of truth.

---

## Execute: RC Mission Control Implementation Plan

Use the **superpowers:subagent-driven-development** skill to execute the plan at `docs/superpowers/plans/2026-03-30-rc-mission-control.md`.

**Model override:** ALL subagents (implementers, spec reviewers, code quality reviewers) MUST use `model: "opus"`.

**Working directory:** `/home/finn/Documents/ardu_ws`

**Plan location:** `docs/superpowers/plans/2026-03-30-rc-mission-control.md`

**Design spec:** `docs/superpowers/specs/2026-03-29-rc-mission-control-design.md`

### Codebase Context (provide to every implementer subagent)

- This is a ROS2 Humble colcon workspace for autonomous drone competition
- Messages/services are in the **separate `dbvf_msgs` package** — always `from dbvf_msgs.msg import ...` / `from dbvf_msgs.srv import ...`
- Tests are pure-Python (no ROS2 runtime) but need ROS2 environment sourced for imports
- Tests are registered in `src/dbvf_autonomy/CMakeLists.txt` via `ament_add_pytest_test`
- Build: `colcon build --packages-select dbvf_autonomy`
- Test: `source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select dbvf_autonomy && colcon test-result --verbose`
- The `StartMission.srv` has request field `config_path` (string); `ResumeMission.srv` has empty request. Both return `success` (bool) + `message` (string).
- Existing test pattern: import pure functions from module, test with simple assertions, use fake/stub classes for MAVLink messages (see `test/test_mavlink_messages.py`)

### Task Execution Order (sequential — do NOT parallelize implementers)

Execute tasks 1 through 6 in order. Each task follows the cycle:

1. **Dispatch implementer** (Agent, model: opus, general-purpose)
2. Wait for completion, handle DONE/DONE_WITH_CONCERNS/NEEDS_CONTEXT/BLOCKED
3. **Dispatch spec reviewer** (Agent, model: opus, general-purpose)
4. If issues found → send fixes back to implementer → re-review
5. **Dispatch code quality reviewer** (Agent, model: opus, subagent_type: superpowers:code-reviewer)
6. If issues found → send fixes back to implementer → re-review
7. Mark task complete

---

### TASK 1 — Implementer Prompt

```
Agent(
  description: "Implement Task 1: Edge detection functions + tests",
  model: "opus",
  prompt: below
)
```

```markdown
You are implementing Task 1: Edge Detection Pure Functions + Tests (TDD)

## Task Description

**Files:**
- Create: `src/dbvf_autonomy/test/test_rc_trigger.py`
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py` (add 2 pure functions after `build_landing_target_params`, before the class)

Steps:

1. Write the test file `src/dbvf_autonomy/test/test_rc_trigger.py` with these tests:

```python
from dbvf_autonomy.mavlink_interface_node import (
    detect_rc_rising_edge,
    get_rc_channel_pwm,
)


def test_rising_edge_fires():
    assert detect_rc_rising_edge(1800, 1000, 1700) is True

def test_sustained_high_does_not_refire():
    assert detect_rc_rising_edge(1800, 1800, 1700) is False

def test_below_threshold_does_not_fire():
    assert detect_rc_rising_edge(1600, 1000, 1700) is False

def test_falling_edge_does_not_fire():
    assert detect_rc_rising_edge(1000, 1800, 1700) is False

def test_no_trigger_on_first_reading():
    assert detect_rc_rising_edge(1800, None, 1700) is False

def test_retrigger_after_reset():
    assert detect_rc_rising_edge(1800, 1000, 1700) is True
    assert detect_rc_rising_edge(1000, 1800, 1700) is False
    assert detect_rc_rising_edge(1800, 1000, 1700) is True

def test_exact_threshold_fires():
    assert detect_rc_rising_edge(1700, 1000, 1700) is True

def test_one_below_threshold_does_not_fire():
    assert detect_rc_rising_edge(1699, 1000, 1700) is False

def test_configurable_threshold():
    assert detect_rc_rising_edge(1600, 1000, 1500) is True
    assert detect_rc_rising_edge(1400, 1000, 1500) is False

def test_get_channel_14():
    class FakeMsg:
        chan14_raw = 1800
    assert get_rc_channel_pwm(FakeMsg(), 14) == 1800

def test_get_channel_15():
    class FakeMsg:
        chan15_raw = 1200
    assert get_rc_channel_pwm(FakeMsg(), 15) == 1200

def test_get_channel_1():
    class FakeMsg:
        chan1_raw = 1500
    assert get_rc_channel_pwm(FakeMsg(), 1) == 1500
```

2. Run tests — verify they FAIL with ImportError (functions don't exist yet).

3. Add two pure functions to `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`, AFTER the `build_landing_target_params` function (ends at line 54), BEFORE the `class MavlinkInterfaceNode` line (line 57):

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

4. Run tests — verify all 13 pass.

5. Commit:
```bash
git add src/dbvf_autonomy/test/test_rc_trigger.py src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py
git commit -m "feat: add RC edge detection pure functions with tests (TDD)"
```

## Context

This is a ROS2 Humble workspace at `/home/finn/Documents/ardu_ws`. The `mavlink_interface_node.py` is the single owner of the pymavlink connection to ArduPilot. You are adding two standalone pure functions that will be used by later tasks to detect RC switch transitions. The test pattern follows existing tests in `test/test_mavlink_messages.py` — import pure functions, test with simple assertions.

To run tests: `source /opt/ros/humble/setup.bash && source install/setup.bash && python -m pytest src/dbvf_autonomy/test/test_rc_trigger.py -v`

## Before You Begin

If you have questions about the requirements, approach, or anything unclear — ask them now.

## Your Job

1. Implement exactly what the task specifies
2. Write tests (TDD — tests first, then implementation)
3. Verify implementation works
4. Commit your work
5. Self-review: completeness, quality, discipline, testing
6. Report back

Work from: `/home/finn/Documents/ardu_ws`

## Report Format

- **Status:** DONE | DONE_WITH_CONCERNS | BLOCKED | NEEDS_CONTEXT
- What you implemented
- What you tested and test results
- Files changed
- Self-review findings
- Any issues or concerns
```

### TASK 1 — Spec Reviewer Prompt

```
Agent(
  description: "Review spec compliance Task 1",
  model: "opus",
  prompt: below
)
```

```markdown
You are reviewing whether an implementation matches its specification.

## What Was Requested

Task 1 from the RC Mission Control plan requires:
- Create `src/dbvf_autonomy/test/test_rc_trigger.py` with 13 tests covering: rising edge fires, sustained high doesn't refire, below threshold doesn't fire, falling edge doesn't fire, no trigger on first reading (None prev), retrigger after reset, exact threshold fires, one below threshold, configurable threshold, get channel 14/15/1.
- Add two pure functions to `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`: `detect_rc_rising_edge(current_pwm, prev_pwm, threshold)` and `get_rc_channel_pwm(msg, channel)`.
- Functions placed after `build_landing_target_params`, before `class MavlinkInterfaceNode`.
- `detect_rc_rising_edge` returns False when prev_pwm is None (boot safety).
- TDD: tests written first, then implementation.
- Commit with message starting "feat: add RC edge detection"

## What Implementer Claims They Built

[INSERT IMPLEMENTER REPORT HERE]

## CRITICAL: Do Not Trust the Report

Read the actual code. Verify:

1. **Missing requirements:** Did they implement everything? All 13 tests? Both functions? Boot safety (None check)?
2. **Extra/unneeded work:** Did they add things not in the spec? Extra functions, extra parameters?
3. **Misunderstandings:** Do the functions match the specified signatures exactly?

Files to read:
- `src/dbvf_autonomy/test/test_rc_trigger.py`
- `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py` (look for the two new functions)

Report:
- ✅ Spec compliant (if everything matches)
- ❌ Issues found: [list with file:line references]
```

### TASK 1 — Code Quality Reviewer

```
Agent(
  description: "Code quality review Task 1",
  model: "opus",
  subagent_type: "superpowers:code-reviewer",
  prompt: below
)
```

```markdown
Review code quality for Task 1: Edge Detection Pure Functions + Tests

WHAT_WAS_IMPLEMENTED: Two pure functions (`detect_rc_rising_edge`, `get_rc_channel_pwm`) added to mavlink_interface_node.py, plus 13 unit tests in test_rc_trigger.py.
PLAN_OR_REQUIREMENTS: Task 1 from docs/superpowers/plans/2026-03-30-rc-mission-control.md
BASE_SHA: [commit before task 1]
HEAD_SHA: [commit after task 1]
DESCRIPTION: Pure function extraction for RC channel edge detection with comprehensive tests.

Additional checks:
- Does each file have one clear responsibility?
- Are the functions placed correctly (module-level, before the class)?
- Do tests follow the existing pattern in test/test_mavlink_messages.py?
```

---

### TASK 2 — Implementer Prompt

```
Agent(
  description: "Implement Task 2: RC monitoring integration",
  model: "opus",
  prompt: below
)
```

```markdown
You are implementing Task 2: RC Monitoring Integration in mavlink_interface_node

## Task Description

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`

This task adds ROS2 integration to the mavlink_interface_node: parameters, service clients, publisher, state variables, and the `_handle_rc_channels` method.

Steps:

1. Update imports — change line 8 and line 11:

Change: `from std_msgs.msg import Bool`
To: `from std_msgs.msg import Bool, String`

Change: `from dbvf_msgs.srv import SetMode, ArmMotors, SendGuidedPosition, SendGuidedVelocity, DoSetServo, Takeoff`
To: `from dbvf_msgs.srv import SetMode, ArmMotors, SendGuidedPosition, SendGuidedVelocity, DoSetServo, Takeoff, StartMission, ResumeMission`

2. In `__init__`, after existing `declare_parameter` calls (after line 65), add:
```python
        self.declare_parameter('rc_start_channel', 14)
        self.declare_parameter('rc_resume_channel', 15)
        self.declare_parameter('rc_trigger_pwm', 1700)
```

3. After existing publishers (after line 85), add:
```python
        self.rc_trigger_pub = self.create_publisher(String, '/dbvf/rc_trigger', 10)
```

4. After existing service definitions (after line 103), add:
```python
        # RC trigger service clients
        self.start_mission_client = self.create_client(
            StartMission, '/dbvf/start_mission')
        self.resume_mission_client = self.create_client(
            ResumeMission, '/dbvf/resume_mission')
```

5. In vehicle state cache section (after `self.last_heartbeat_time = 0.0`), add:
```python
        # RC channel edge detection state (None = no reading yet)
        self._rc_prev = {}
```

6. After `_read_timer` method, add two new methods:
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

7. In `_read_timer`, after the `elif mtype == 'RANGEFINDER':` block (after line 182), add:
```python
                elif mtype == 'RC_CHANNELS':
                    self._handle_rc_channels(msg)
```

8. Build: `colcon build --packages-select dbvf_autonomy`

9. Commit:
```bash
git add src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py
git commit -m "feat: add RC channel monitoring for mission start/resume triggers"
```

## Context

This is the second task. Task 1 already added `detect_rc_rising_edge` and `get_rc_channel_pwm` as pure functions in this same file. You are now wiring them into the ROS2 node.

The `StartMission.srv` has a request field `config_path` (string) — use an empty `StartMission.Request()` (default empty string is fine). `ResumeMission.srv` has an empty request.

The `_read_timer` method processes MAVLink messages in a `while True` loop. It already handles HEARTBEAT, GLOBAL_POSITION_INT, and RANGEFINDER. You're adding RC_CHANNELS as a fourth case.

No new tests needed — the pure logic is tested in Task 1. This task is ROS2 integration only.

Work from: `/home/finn/Documents/ardu_ws`

## Before You Begin

If you have questions, ask now.

## Your Job

1. Implement exactly what the task specifies
2. Verify build succeeds
3. Commit your work
4. Self-review
5. Report back

## Report Format

- **Status:** DONE | DONE_WITH_CONCERNS | BLOCKED | NEEDS_CONTEXT
- What you implemented
- Build result
- Files changed
- Self-review findings
- Any issues or concerns
```

### TASK 2 — Spec Reviewer Prompt

```
Agent(
  description: "Review spec compliance Task 2",
  model: "opus",
  prompt: below
)
```

```markdown
You are reviewing whether an implementation matches its specification.

## What Was Requested

Task 2 from the RC Mission Control plan requires modifications to `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py`:

1. Add `String` to std_msgs import, add `StartMission, ResumeMission` to dbvf_msgs.srv import
2. Declare 3 new parameters: `rc_start_channel` (default 14), `rc_resume_channel` (default 15), `rc_trigger_pwm` (default 1700)
3. Create publisher: `rc_trigger_pub` on `/dbvf/rc_trigger` (String)
4. Create 2 service clients: `start_mission_client` for `/dbvf/start_mission`, `resume_mission_client` for `/dbvf/resume_mission`
5. Initialize `self._rc_prev = {}` for edge detection state
6. Add `_handle_rc_channels(self, msg)` method that: reads threshold param, iterates configured channels, calls `get_rc_channel_pwm` and `detect_rc_rising_edge`, publishes to rc_trigger topic, calls service async with done callback, updates prev state
7. Add `_rc_service_done(self, future, label)` callback that logs success/failure
8. Add `elif mtype == 'RC_CHANNELS':` case in `_read_timer` calling `_handle_rc_channels`
9. Build succeeds
10. Commit with message starting "feat: add RC channel monitoring"

## What Implementer Claims They Built

[INSERT IMPLEMENTER REPORT HERE]

## CRITICAL: Do Not Trust the Report

Read `src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py` and verify all 10 items above. Check imports, parameters, publisher, clients, state init, methods, _read_timer case, and lambda closure (the `l=label` default argument pattern).

Report:
- ✅ Spec compliant
- ❌ Issues found: [list with file:line references]
```

### TASK 2 — Code Quality Reviewer

```
Agent(
  description: "Code quality review Task 2",
  model: "opus",
  subagent_type: "superpowers:code-reviewer",
  prompt: below
)
```

```markdown
Review code quality for Task 2: RC Monitoring Integration

WHAT_WAS_IMPLEMENTED: RC channel monitoring added to mavlink_interface_node — 3 params, 2 service clients, 1 publisher, edge detection handler, async service calls with callbacks.
PLAN_OR_REQUIREMENTS: Task 2 from docs/superpowers/plans/2026-03-30-rc-mission-control.md
BASE_SHA: [commit before task 2]
HEAD_SHA: [commit after task 2]
DESCRIPTION: ROS2 integration wiring for RC-triggered mission start/resume.

Additional checks:
- Lambda closure correctness (default arg `l=label` to avoid late binding)
- Thread safety (handler runs inside `with self.lock` block in _read_timer)
- No blocking calls in the timer callback (call_async, not call)
```

---

### TASK 3 — Implementer Prompt

```
Agent(
  description: "Implement Task 3: Register test in CMakeLists",
  model: "opus",
  prompt: below
)
```

```markdown
You are implementing Task 3: Register Test in CMakeLists.txt

## Task Description

**Files:**
- Modify: `src/dbvf_autonomy/CMakeLists.txt`

Steps:

1. In `src/dbvf_autonomy/CMakeLists.txt`, after the last `ament_add_pytest_test` line (which is `ament_add_pytest_test(test_mission_fsm test/test_mission_state_machine.py)`), add:
```cmake
  ament_add_pytest_test(test_rc_trigger test/test_rc_trigger.py)
```

2. Build and run all tests:
```bash
colcon build --packages-select dbvf_autonomy
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```
Expected: All tests pass (151 existing + 13 new = 164 total).

3. Commit:
```bash
git add src/dbvf_autonomy/CMakeLists.txt
git commit -m "test: register RC trigger tests in CMakeLists.txt"
```

## Context

Tasks 1 and 2 already created `test/test_rc_trigger.py` and the functions it tests. This task just registers the test file with the ament build system so `colcon test` picks it up.

Work from: `/home/finn/Documents/ardu_ws`

## Your Job

1. Add the one line to CMakeLists.txt
2. Build and run ALL tests — verify everything passes
3. Commit
4. Report back

## Report Format

- **Status:** DONE | DONE_WITH_CONCERNS | BLOCKED | NEEDS_CONTEXT
- What you changed
- Full test results (count of passing tests)
- Files changed
```

### TASK 3 — Spec Reviewer Prompt

```
Agent(
  description: "Review spec compliance Task 3",
  model: "opus",
  prompt: below
)
```

```markdown
You are reviewing whether an implementation matches its specification.

## What Was Requested

Task 3: Add `ament_add_pytest_test(test_rc_trigger test/test_rc_trigger.py)` to `src/dbvf_autonomy/CMakeLists.txt`, after the existing test registrations. Build and run tests — all should pass.

## What Implementer Claims They Built

[INSERT IMPLEMENTER REPORT HERE]

## CRITICAL: Do Not Trust the Report

Read `src/dbvf_autonomy/CMakeLists.txt` and verify:
1. The new line exists
2. It's placed after existing test registrations (inside `if(BUILD_TESTING)` block)
3. The test name is `test_rc_trigger` and path is `test/test_rc_trigger.py`

Report:
- ✅ Spec compliant
- ❌ Issues found: [list]
```

### TASK 3 — Code Quality Reviewer

```
Agent(
  description: "Code quality review Task 3",
  model: "opus",
  subagent_type: "superpowers:code-reviewer",
  prompt: below
)
```

```markdown
Review code quality for Task 3: Register test in CMakeLists.txt

WHAT_WAS_IMPLEMENTED: One line added to CMakeLists.txt to register test_rc_trigger.py.
PLAN_OR_REQUIREMENTS: Task 3 from docs/superpowers/plans/2026-03-30-rc-mission-control.md
BASE_SHA: [commit before task 3]
HEAD_SHA: [commit after task 3]
DESCRIPTION: Test registration — single line addition.
```

---

### TASK 4 — Implementer Prompt

```
Agent(
  description: "Implement Task 4: Update mission_params.yaml",
  model: "opus",
  prompt: below
)
```

```markdown
You are implementing Task 4: Update mission_params.yaml

## Task Description

**Files:**
- Modify: `src/dbvf_autonomy/config/mission_params.yaml`

Replace the full contents of the file with the version below. IMPORTANT: Do NOT change any GPS coordinate values — only add comments, placeholders, and the new `mavlink_interface` section.

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

Steps:

1. Read the existing file to confirm current values.
2. Write the new version (use the Write tool — this is a full replacement).
3. Verify YAML is valid:
```bash
python3 -c "import yaml; yaml.safe_load(open('src/dbvf_autonomy/config/mission_params.yaml')); print('YAML OK')"
```
4. Commit:
```bash
git add src/dbvf_autonomy/config/mission_params.yaml
git commit -m "docs: add GPS comments, competition placeholders, and RC trigger params to mission config"
```

## Context

This config file is loaded by the mission_sequencer_node and (after Task 5) the mavlink_interface_node. The GPS values are simulation defaults — they MUST NOT change. You are adding documentation comments, commented-out competition placeholders (prefix `# COMPETITION:`), and a new `mavlink_interface` section for RC trigger parameters.

Work from: `/home/finn/Documents/ardu_ws`

## Your Job

1. Replace the file contents exactly as specified
2. Verify YAML validity
3. Commit
4. Report back

## Report Format

- **Status:** DONE | DONE_WITH_CONCERNS | BLOCKED | NEEDS_CONTEXT
- Confirmation that GPS values are unchanged
- YAML validation result
- Files changed
```

### TASK 4 — Spec Reviewer Prompt

```
Agent(
  description: "Review spec compliance Task 4",
  model: "opus",
  prompt: below
)
```

```markdown
You are reviewing whether an implementation matches its specification.

## What Was Requested

Task 4: Update `src/dbvf_autonomy/config/mission_params.yaml`:
1. Add inline comments to every GPS param (waypoint name, zone size, "REPLACE WITH COMPETITION COORDS" context)
2. Add `# COMPETITION:` placeholder lines below each GPS value pair
3. Add `mavlink_interface: ros__parameters:` section with `rc_start_channel: 14`, `rc_resume_channel: 15`, `rc_trigger_pwm: 1700`
4. Do NOT change any GPS coordinate values (home_lat: -35.3632621, home_lon: 149.1652374, landing_lat: -35.3640000, landing_lon: 149.1652374, wa_lat: -35.3632531, wa_lon: 149.1657896, f1_lat: -35.3650000, f1_lon: 149.1652374, f2_lat: -35.3660000, f2_lon: 149.1652374)
5. Do NOT change any non-GPS parameter values

## What Implementer Claims They Built

[INSERT IMPLEMENTER REPORT HERE]

## CRITICAL: Do Not Trust the Report

Read `src/dbvf_autonomy/config/mission_params.yaml` and verify:
1. Every GPS value matches the originals exactly (diff against the values listed above)
2. Comments exist on GPS params
3. `# COMPETITION:` placeholders exist
4. `mavlink_interface` section exists with correct values
5. All non-GPS params (transit_altitude_ft, drop_servo_number, etc.) are unchanged

Report:
- ✅ Spec compliant
- ❌ Issues found: [list]
```

### TASK 4 — Code Quality Reviewer

```
Agent(
  description: "Code quality review Task 4",
  model: "opus",
  subagent_type: "superpowers:code-reviewer",
  prompt: below
)
```

```markdown
Review code quality for Task 4: Update mission_params.yaml

WHAT_WAS_IMPLEMENTED: Config file updated with inline comments, competition placeholders, and RC trigger parameter section.
PLAN_OR_REQUIREMENTS: Task 4 from docs/superpowers/plans/2026-03-30-rc-mission-control.md
BASE_SHA: [commit before task 4]
HEAD_SHA: [commit after task 4]
DESCRIPTION: Documentation and config additions to mission_params.yaml.

Additional checks:
- YAML is well-formed and parseable
- Comments are clear and actionable
- Parameter namespacing is correct (mavlink_interface vs mission_sequencer)
```

---

### TASK 5 — Implementer Prompt

```
Agent(
  description: "Implement Task 5: Update launch file",
  model: "opus",
  prompt: below
)
```

```markdown
You are implementing Task 5: Update mission_sim.launch.py

## Task Description

**Files:**
- Modify: `src/dbvf_autonomy/launch/mission_sim.launch.py`

Change the mavlink_interface Node definition to also load `mission_config`.

Find this block (around lines 47-51):
```python
        # MAVLink interface
        Node(
            package='dbvf_autonomy',
            executable='mavlink_interface_node',
            name='mavlink_interface',
            parameters=[sim_config],
        ),
```

Change `parameters=[sim_config]` to `parameters=[sim_config, mission_config]`:
```python
        # MAVLink interface
        Node(
            package='dbvf_autonomy',
            executable='mavlink_interface_node',
            name='mavlink_interface',
            parameters=[sim_config, mission_config],
        ),
```

Steps:

1. Read the file to confirm current state.
2. Make the one-line edit.
3. Build: `colcon build --packages-select dbvf_autonomy`
4. Commit:
```bash
git add src/dbvf_autonomy/launch/mission_sim.launch.py
git commit -m "fix: load mission_params.yaml for mavlink_interface node (RC trigger params)"
```

## Context

The launch file already loads `mission_config` for the mission_sequencer node. The `mission_config` variable is already defined at the top of the file (line 16). You just need to add it to the mavlink_interface node's parameters list so the RC trigger params from `mission_params.yaml` get loaded.

Work from: `/home/finn/Documents/ardu_ws`

## Your Job

1. Make the one-line edit
2. Build
3. Commit
4. Report back

## Report Format

- **Status:** DONE | DONE_WITH_CONCERNS | BLOCKED | NEEDS_CONTEXT
- What you changed
- Build result
- Files changed
```

### TASK 5 — Spec Reviewer Prompt

```
Agent(
  description: "Review spec compliance Task 5",
  model: "opus",
  prompt: below
)
```

```markdown
You are reviewing whether an implementation matches its specification.

## What Was Requested

Task 5: In `src/dbvf_autonomy/launch/mission_sim.launch.py`, change the mavlink_interface Node's `parameters=[sim_config]` to `parameters=[sim_config, mission_config]`.

## What Implementer Claims They Built

[INSERT IMPLEMENTER REPORT HERE]

## CRITICAL: Do Not Trust the Report

Read `src/dbvf_autonomy/launch/mission_sim.launch.py` and verify:
1. The mavlink_interface Node now has `parameters=[sim_config, mission_config]`
2. No other nodes were changed
3. The `mission_config` variable reference matches the one already used for mission_sequencer

Report:
- ✅ Spec compliant
- ❌ Issues found: [list]
```

### TASK 5 — Code Quality Reviewer

```
Agent(
  description: "Code quality review Task 5",
  model: "opus",
  subagent_type: "superpowers:code-reviewer",
  prompt: below
)
```

```markdown
Review code quality for Task 5: Update mission_sim.launch.py

WHAT_WAS_IMPLEMENTED: Added mission_config to mavlink_interface node's parameters list.
PLAN_OR_REQUIREMENTS: Task 5 from docs/superpowers/plans/2026-03-30-rc-mission-control.md
BASE_SHA: [commit before task 5]
HEAD_SHA: [commit after task 5]
DESCRIPTION: One-line launch file change.
```

---

### TASK 6 — Implementer Prompt

```
Agent(
  description: "Implement Task 6: Competition operations doc",
  model: "opus",
  prompt: below
)
```

```markdown
You are implementing Task 6: Create Competition Operations Document

## Task Description

**Files:**
- Create: `src/dbvf_autonomy/docs/competition_operations.md`

Create the file with the exact contents below. Use the Write tool.

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

Steps:

1. Ensure the `docs/` directory exists: `ls src/dbvf_autonomy/docs/` (create if needed: `mkdir -p src/dbvf_autonomy/docs/`)
2. Write the file with the exact contents above.
3. Commit:
```bash
git add src/dbvf_autonomy/docs/competition_operations.md
git commit -m "docs: add competition operations guide for DBVF field day"
```

## Context

This is a standalone documentation file. It references the config file updated in Task 4 and the RC trigger functionality from Tasks 1-2. No code changes.

Work from: `/home/finn/Documents/ardu_ws`

## Your Job

1. Create the docs directory if needed
2. Write the file
3. Commit
4. Report back

## Report Format

- **Status:** DONE | DONE_WITH_CONCERNS | BLOCKED | NEEDS_CONTEXT
- Confirmation file was created
- Files changed
```

### TASK 6 — Spec Reviewer Prompt

```
Agent(
  description: "Review spec compliance Task 6",
  model: "opus",
  prompt: below
)
```

```markdown
You are reviewing whether an implementation matches its specification.

## What Was Requested

Task 6: Create `src/dbvf_autonomy/docs/competition_operations.md` with 7 sections:
1. Pre-Flight Setup (8 steps, SSH, config editing, launch, verify)
2. RC Switch Mapping (table: ch14=start, ch15=resume, mode switch=abort)
3. Mission Flow (table with all 17 phases)
4. Abort Procedure (3 methods: mode switch, kill switch, GCS service call)
5. Troubleshooting (7-row table)
6. GPS Coordinate Format (decimal degrees, DMS/DDM/UTM conversion, Harford Airport sanity check)
7. Monitoring During Flight (6 ros2 topic echo commands)

## What Implementer Claims They Built

[INSERT IMPLEMENTER REPORT HERE]

## CRITICAL: Do Not Trust the Report

Read `src/dbvf_autonomy/docs/competition_operations.md` and verify all 7 sections exist with the required content. Check that:
- All service/topic names match the actual system (`/dbvf/start_mission`, `/dbvf/rc_trigger`, etc.)
- Competition location is mentioned (Harford Airport, Churchville MD)
- RC channels 14/15 are correctly described
- Abort procedure includes all 3 methods

Report:
- ✅ Spec compliant
- ❌ Issues found: [list]
```

### TASK 6 — Code Quality Reviewer

```
Agent(
  description: "Code quality review Task 6",
  model: "opus",
  subagent_type: "superpowers:code-reviewer",
  prompt: below
)
```

```markdown
Review code quality for Task 6: Competition Operations Document

WHAT_WAS_IMPLEMENTED: New competition operations guide at src/dbvf_autonomy/docs/competition_operations.md.
PLAN_OR_REQUIREMENTS: Task 6 from docs/superpowers/plans/2026-03-30-rc-mission-control.md
BASE_SHA: [commit before task 6]
HEAD_SHA: [commit after task 6]
DESCRIPTION: Competition day operations documentation — no code changes.

Additional checks:
- Is the document clear for someone who didn't write the code?
- Are all commands copy-pasteable?
- Are topic/service names accurate?
```

---

### FINAL — Full Implementation Code Review

After all 6 tasks are complete, dispatch a final code reviewer:

```
Agent(
  description: "Final code review: RC mission control",
  model: "opus",
  subagent_type: "superpowers:code-reviewer",
  prompt: below
)
```

```markdown
Review the complete RC Mission Control implementation across all commits.

WHAT_WAS_IMPLEMENTED: RC channel-based mission triggers (start/resume via RC switches ch14/ch15), updated mission config with competition placeholders, competition operations guide, launch file fix.
PLAN_OR_REQUIREMENTS: docs/superpowers/plans/2026-03-30-rc-mission-control.md
SPEC: docs/superpowers/specs/2026-03-29-rc-mission-control-design.md
BASE_SHA: [commit before task 1]
HEAD_SHA: [commit after task 6]
DESCRIPTION: Full feature implementation — 6 tasks, ~40 lines of new code in mavlink_interface_node, 13 new tests, config improvements, operations doc.

Key files to review:
- src/dbvf_autonomy/dbvf_autonomy/mavlink_interface_node.py (pure functions + node integration)
- src/dbvf_autonomy/test/test_rc_trigger.py (13 tests)
- src/dbvf_autonomy/config/mission_params.yaml (comments + RC params)
- src/dbvf_autonomy/launch/mission_sim.launch.py (1-line change)
- src/dbvf_autonomy/docs/competition_operations.md (new doc)
- src/dbvf_autonomy/CMakeLists.txt (test registration)

Focus areas:
- Thread safety of RC handler (runs inside _read_timer's lock context)
- Lambda closure correctness in done callbacks
- Edge detection logic correctness
- Config namespacing (mavlink_interface vs mission_sequencer)
- Overall coherence across all changes
```

---

### POST-COMPLETION

After final review passes, use **superpowers:finishing-a-development-branch** to decide how to integrate (merge, PR, or cleanup).
