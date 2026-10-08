# ISS-018: Stale Tag Config Fix — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix stale apriltag_ros config, stale parameter files, and undersized secondary tag model so that tag ID 2 is detected during precision landing and the 4-phase final approach (ISS-013) activates.

**Architecture:** Config-only fix across two git repos. No logic changes. The apriltag_ros launch parameters, three YAML config files, and one Gazebo model all reference stale tag IDs or sizes from before the tag layout was restructured. After these fixes, all config/model files agree: primary tag = ID 1 @ 0.15m, secondary tag = ID 2 @ 0.10m.

**Tech Stack:** ROS2 Humble launch files (Python), YAML config, Gazebo SDF models

**Issue tracker:** `src/dbvf_autonomy/docs/Features/Auto Landing April Tags/Final Positioning during landing/issues.md` (ISS-018)

---

## File Map

All changes are config/model fixes — no Python source code or tests are modified.

**Repo: `src/dbvf_autonomy/.git` (branch `feat/wa-reload-mechanism`)**

| File | Change |
|------|--------|
| `src/dbvf_autonomy/launch/precision_landing_sim.launch.py:27-31` | `tag.ids: [0, 1]` → `[1, 2]`, `tag.sizes: [0.6, 0.15]` → `[0.15, 0.10]`, `size: 0.6` → `0.15` |
| `src/dbvf_autonomy/config/sim_params.yaml:14` | `secondary_tag_size: 0.05` → `0.10` |
| `src/dbvf_autonomy/config/hardware_params.yaml:25` | `secondary_tag_size: 0.05` → `0.10` |
| `src/dbvf_autonomy/config/jetson_params.yaml:11-14,40` | Tag IDs 0/1 → 1/2, sizes 0.6/0.15 → 0.15/0.10, precision_landing secondary_tag_id 1 → 2 |
| `src/dbvf_autonomy/docs/.../issues.md:297` | ISS-018 status: Open → FIXED |

**Repo: `src/ardupilot_gazebo/.git` (branch `ros2`)**

| File | Change |
|------|--------|
| `src/ardupilot_gazebo/models/Apriltag36_11_00002/model.sdf:8,24` | Box size `0.05 0.05 0.001` → `0.10 0.10 0.001` |
| `src/ardupilot_gazebo/models/Apriltag36_11_00002/model.config:6` | Description `0.05m` → `0.10m` |

### Additional finding: `hardware_params.yaml` secondary_tag_size

The user identified `sim_params.yaml:14` as needing `secondary_tag_size: 0.05` → `0.10`. The same stale value exists at `hardware_params.yaml:25`. Both real hardware launch files (`precision_landing_real.launch.py`, `mission_real.launch.py`) load `hardware_params.yaml` — this would break real hardware tag size estimation. Included in the plan.

### Note: `jetson_params.yaml` is broader-stale

`jetson_params.yaml` is not referenced by any launch file (real launches use `hardware_params.yaml`). It's a legacy config from early design docs. Beyond the tag ID/size fixes in this plan, it's also missing many parameters added by ISS-011/013/017 (slow_descent_altitude, 4-phase params, etc.). A full sync is out of scope for ISS-018 — this plan fixes only the tag-related values to prevent confusion.

---

## Task 1: Fix `precision_landing_sim.launch.py` apriltag_ros config

**Files:**
- Modify: `src/dbvf_autonomy/launch/precision_landing_sim.launch.py:27-31`

This is the primary blocker. The launch file tells apriltag_ros to look for tag IDs [0, 1] at sizes [0.6, 0.15]. Tag ID 0 doesn't exist in the Gazebo world. Tag ID 2 is missing from the list, so apriltag_ros uses the wrong default size for it. The other three launch files (`mission_sim`, `precision_landing_real`, `mission_real`) already have the correct `[1, 2]` / `[0.15, 0.10]` config.

- [ ] **Step 1: Edit the apriltag_ros parameters**

In `src/dbvf_autonomy/launch/precision_landing_sim.launch.py`, change lines 27-31 from:

```python
            parameters=[{
                'family': '36h11',
                'size': 0.6,
                'tag.ids': [0, 1],
                'tag.sizes': [0.6, 0.15],
            }],
```

to:

```python
            parameters=[{
                'family': '36h11',
                'size': 0.15,
                'tag.ids': [1, 2],
                'tag.sizes': [0.15, 0.10],
            }],
```

This matches `mission_sim.launch.py:29-33` exactly.

- [ ] **Step 2: Verify all four launch files now agree**

Run:
```bash
grep -A 4 "'tag.ids'" src/dbvf_autonomy/launch/*.launch.py
```

Expected: all four files show `'tag.ids': [1, 2]` and `'tag.sizes': [0.15, 0.10]`.

- [ ] **Step 3: Commit (dbvf_autonomy repo)**

```bash
cd src/dbvf_autonomy
git add launch/precision_landing_sim.launch.py
git commit -m "fix(ISS-018): update stale apriltag_ros config in precision_landing_sim launch

tag.ids [0,1] → [1,2], tag.sizes [0.6,0.15] → [0.15,0.10], default size
0.6 → 0.15. Tag ID 0 no longer exists in the Gazebo world. The other
three launch files already had the correct config."
```

---

## Task 2: Fix `sim_params.yaml` and `hardware_params.yaml` secondary tag size

**Files:**
- Modify: `src/dbvf_autonomy/config/sim_params.yaml:14`
- Modify: `src/dbvf_autonomy/config/hardware_params.yaml:25`

Both config files have `secondary_tag_size: 0.05` — this tells `tag_detector_adapter_node` the secondary tag is 0.05m, but the Gazebo model is being enlarged to 0.10m (Task 5) and the launch file `tag.sizes` already reference 0.10m. The adapter uses this value for homography-based pose estimation, so a mismatch causes incorrect position estimates.

- [ ] **Step 1: Edit `sim_params.yaml`**

In `src/dbvf_autonomy/config/sim_params.yaml`, change line 14 from:

```yaml
    secondary_tag_size: 0.05
```

to:

```yaml
    secondary_tag_size: 0.10
```

- [ ] **Step 2: Edit `hardware_params.yaml`**

In `src/dbvf_autonomy/config/hardware_params.yaml`, change line 25 from:

```yaml
    secondary_tag_size: 0.05
```

to:

```yaml
    secondary_tag_size: 0.10
```

- [ ] **Step 3: Verify all three config files agree on secondary tag size**

Run:
```bash
grep 'secondary_tag_size' src/dbvf_autonomy/config/*.yaml
```

Expected: `sim_params.yaml`, `hardware_params.yaml`, and `jetson_params.yaml` (fixed in Task 3) all show `0.10` (jetson_params.yaml will show `0.15` until Task 3 is done — that's expected at this step).

- [ ] **Step 4: Commit (dbvf_autonomy repo)**

```bash
cd src/dbvf_autonomy
git add config/sim_params.yaml config/hardware_params.yaml
git commit -m "fix(ISS-018): update secondary_tag_size 0.05 → 0.10 in sim and hardware configs

Matches the enlarged Apriltag36_11_00002 Gazebo model (0.10m) and the
tag.sizes values in all four launch files. Affects tag_detector_adapter
pose estimation accuracy."
```

---

## Task 3: Fix `jetson_params.yaml` stale tag IDs and sizes

**Files:**
- Modify: `src/dbvf_autonomy/config/jetson_params.yaml:11-14,40`

`jetson_params.yaml` is a legacy config (no launch file references it — real launches use `hardware_params.yaml`), but it's installed by CMake and referenced in design docs. It has stale tag IDs (0/1 instead of 1/2), stale tag sizes (0.6/0.15 instead of 0.15/0.10), and a stale `secondary_tag_id` in the precision_landing section.

- [ ] **Step 1: Fix tag_detector_adapter section (lines 11-14)**

In `src/dbvf_autonomy/config/jetson_params.yaml`, change lines 11-14 from:

```yaml
    primary_tag_id: 0
    secondary_tag_id: 1
    primary_tag_size: 0.6
    secondary_tag_size: 0.15
```

to:

```yaml
    primary_tag_id: 1
    secondary_tag_id: 2
    primary_tag_size: 0.15
    secondary_tag_size: 0.10
```

- [ ] **Step 2: Fix precision_landing section (line 40)**

In `src/dbvf_autonomy/config/jetson_params.yaml`, change line 40 from:

```yaml
    secondary_tag_id: 1
```

to:

```yaml
    secondary_tag_id: 2
```

- [ ] **Step 3: Verify tag IDs are consistent across all config files**

Run:
```bash
grep -E '(primary_tag_id|secondary_tag_id)' src/dbvf_autonomy/config/*.yaml
```

Expected: all files show `primary_tag_id: 1` and `secondary_tag_id: 2`.

- [ ] **Step 4: Commit (dbvf_autonomy repo)**

```bash
cd src/dbvf_autonomy
git add config/jetson_params.yaml
git commit -m "fix(ISS-018): update stale tag IDs and sizes in jetson_params.yaml

primary_tag_id 0→1, secondary_tag_id 1→2, primary_tag_size 0.6→0.15,
secondary_tag_size 0.15→0.10. Also fixed secondary_tag_id in
precision_landing section. jetson_params.yaml is legacy (real launches
use hardware_params.yaml) but kept consistent to prevent confusion."
```

---

## Task 4: Enlarge secondary tag Gazebo model to 0.10m

**Files:**
- Modify: `src/ardupilot_gazebo/models/Apriltag36_11_00002/model.sdf:8,24`
- Modify: `src/ardupilot_gazebo/models/Apriltag36_11_00002/model.config:6`

The secondary tag model is 0.05m (5cm) — too small for reliable AprilTag detection at 1-2m altitude (~16px on camera). Enlarging to 0.10m matches the `tag.sizes` and `secondary_tag_size` values in all configs/launches. For reference, the primary tag model (`Apriltag36_11_00001`) uses `0.15 0.15 0.001`.

- [ ] **Step 1: Update model.sdf visual geometry**

In `src/ardupilot_gazebo/models/Apriltag36_11_00002/model.sdf`, change line 8 from:

```xml
          <box><size>0.05 0.05 0.001</size></box>
```

to:

```xml
          <box><size>0.10 0.10 0.001</size></box>
```

- [ ] **Step 2: Update model.sdf collision geometry**

In the same file, change line 24 from:

```xml
          <box><size>0.05 0.05 0.001</size></box>
```

to:

```xml
          <box><size>0.10 0.10 0.001</size></box>
```

- [ ] **Step 3: Update model.config description**

In `src/ardupilot_gazebo/models/Apriltag36_11_00002/model.config`, change line 6 from:

```xml
  <description>AprilTag tag36h11 ID 2 — 0.05m tertiary landing target for ultra-precise final alignment at WA.</description>
```

to:

```xml
  <description>AprilTag tag36h11 ID 2 — 0.10m secondary landing target for precision landing.</description>
```

- [ ] **Step 4: Verify model dimensions match primary tag pattern**

Run:
```bash
grep '<size>' src/ardupilot_gazebo/models/Apriltag36_11_0000*/model.sdf
```

Expected:
```
Apriltag36_11_00001/model.sdf:          <box><size>0.15 0.15 0.001</size></box>   (×2)
Apriltag36_11_00002/model.sdf:          <box><size>0.10 0.10 0.001</size></box>   (×2)
```

- [ ] **Step 5: Commit (ardupilot_gazebo repo)**

```bash
cd src/ardupilot_gazebo
git add models/Apriltag36_11_00002/model.sdf models/Apriltag36_11_00002/model.config
git commit -m "fix(ISS-018): enlarge secondary AprilTag model from 0.05m to 0.10m

0.05m tag was ~16px at 2m altitude — too small for reliable apriltag_ros
detection. 0.10m matches tag.sizes in all launch files and
secondary_tag_size in config YAML files."
```

---

## Task 5: Mark ISS-018 as FIXED in issues tracker

**Files:**
- Modify: `src/dbvf_autonomy/docs/Features/Auto Landing April Tags/Final Positioning during landing/issues.md:297`

- [ ] **Step 1: Update ISS-018 status**

In `src/dbvf_autonomy/docs/Features/Auto Landing April Tags/Final Positioning during landing/issues.md`, change line 297 from:

```markdown
**Status:** Open — root cause identified (stale launch config + small tag)
```

to:

```markdown
**Status:** FIXED (2026-04-05)
```

- [ ] **Step 2: Add resolution section at end of ISS-018**

Append the following after the existing `### Files involved` section (after line 371), before the closing of the file:

```markdown

### Resolution (2026-04-05)

All config and model files updated to agree on: primary tag = ID 1 @ 0.15m, secondary tag = ID 2 @ 0.10m.

| File | Before | After |
|------|--------|-------|
| `launch/precision_landing_sim.launch.py` | `tag.ids: [0, 1]`, `tag.sizes: [0.6, 0.15]` | `tag.ids: [1, 2]`, `tag.sizes: [0.15, 0.10]` |
| `config/sim_params.yaml` | `secondary_tag_size: 0.05` | `secondary_tag_size: 0.10` |
| `config/hardware_params.yaml` | `secondary_tag_size: 0.05` | `secondary_tag_size: 0.10` |
| `config/jetson_params.yaml` | `primary_tag_id: 0`, `secondary_tag_id: 1`, sizes 0.6/0.15 | IDs 1/2, sizes 0.15/0.10 |
| `models/Apriltag36_11_00002/model.sdf` | `0.05 0.05 0.001` | `0.10 0.10 0.001` |

Additional find: `hardware_params.yaml` had the same `secondary_tag_size: 0.05` stale value — included in fix.
```

- [ ] **Step 3: Commit (dbvf_autonomy repo)**

```bash
cd src/dbvf_autonomy
git add "docs/Features/Auto Landing April Tags/Final Positioning during landing/issues.md"
git commit -m "docs(ISS-018): mark ISS-018 as fixed"
```

---

## Task 6: Build and run existing tests

No new tests are needed (these are config/model fixes, not logic changes). But the existing 238 tests must still pass after the config changes.

- [ ] **Step 1: Build both packages**

```bash
cd /home/finn/Documents/ardu_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select dbvf_msgs dbvf_autonomy
```

Expected: builds cleanly.

- [ ] **Step 2: Run all tests**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```

Expected: 238 tests, 0 failures. Config YAML changes don't affect unit tests (tests don't load config files), but this confirms nothing was accidentally broken.

- [ ] **Step 3: Build ardupilot_gazebo**

```bash
colcon build --packages-select ardupilot_gazebo
```

Expected: builds cleanly. This installs the updated model files.

---

## Verification (manual, post-implementation)

After all tasks are committed and both packages rebuilt, verify the fix end-to-end in SITL:

1. Launch Gazebo + ArduPilot SITL: `ros2 launch ardupilot_gz_bringup iris_runway.launch.py`
2. Launch precision landing stack: `ros2 launch dbvf_autonomy precision_landing_sim.launch.py`
3. In MAVProxy: `mode guided` → `arm throttle` → `takeoff 10`
4. Trigger landing: `ros2 service call /dbvf/start_precision_landing dbvf_msgs/srv/StartPrecisionLanding "{target_lat: -35.363262, target_lon: 149.165237}"`
5. Monitor: `ros2 topic echo /dbvf/tag_status` — confirm `active_tag_id: 2` appears during descent
6. Monitor: `ros2 topic echo /dbvf/landing_state` — confirm FSM enters `HOLD_ABOVE_TAG` → `ALIGN_YAW` → `OFFSET_LATERAL` → `DESCEND_FINAL`

**Success criteria:** Tag ID 2 is detected, the 4-phase landing sequence activates, and the drone completes the landing.

---

## Summary of all values after fix

| Parameter | sim_params | hardware_params | jetson_params | Launch files | Gazebo model |
|-----------|-----------|-----------------|---------------|-------------|-------------|
| Primary tag ID | 1 | 1 | 1 | `tag.ids: [1, 2]` | Apriltag36_11_00001 |
| Primary tag size | 0.15 | 0.15 | 0.15 | `tag.sizes: [0.15, 0.10]` | 0.15m |
| Secondary tag ID | 2 | 2 | 2 | `tag.ids: [1, 2]` | Apriltag36_11_00002 |
| Secondary tag size | 0.10 | 0.10 | 0.10 | `tag.sizes: [0.15, 0.10]` | 0.10m |
