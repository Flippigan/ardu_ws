# ISS-015: Tag-Relative Yaw Extraction — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extract tag-relative yaw from the AprilTag homography decomposition so the drone knows its heading relative to the tag's printed orientation.

**Architecture:** The existing `estimate_tag_position()` in `tag_detector_adapter_node.py` already decomposes `K^-1 * H` into rotation columns and a translation column, but discards the rotation. A new pure function `estimate_tag_yaw()` will extract yaw from the first rotation column, transform it through the configurable camera-to-body axis remap, and return body-frame yaw. The result is published via a new `tag_yaw` field on `LandingTargetPose.msg`.

**Tech Stack:** Python 3, ROS2 Humble, `dbvf_msgs` (ament_cmake + rosidl), `dbvf_autonomy` (ament_cmake_python), math (atan2), pytest

**Issue:** `src/dbvf_autonomy/docs/Features/Auto Landing April Tags/Final Positioning during landing/issues.md` — ISS-015

---

## File Structure

| Action | File | Responsibility |
|--------|------|---------------|
| Modify | `src/dbvf_msgs/msg/LandingTargetPose.msg` | Add `float64 tag_yaw` field |
| Modify | `src/dbvf_autonomy/dbvf_autonomy/tag_detector_adapter_node.py` | Add `estimate_tag_yaw()` pure function, populate `tag_yaw` in `_publish_target()` |
| Modify | `src/dbvf_autonomy/test/test_tag_pose_estimation.py` | Add yaw extraction tests alongside existing position tests |

No new files. Three modifications across two packages.

---

## Math Background

The homography `H` from apriltag_ros is a 9-element row-major 3×3 matrix. The decomposition `M = K^-1 * H` yields:

```
M = [col0 | col1 | col2]
    rotation  rotation  translation
    column 0  column 1  (scaled)
```

Where `col0 = (m00, m10, m20)` is the tag's X-axis direction in camera frame. The camera-frame yaw (rotation around the optical axis Z) is:

```
camera_yaw = atan2(m10, m00)
```

To convert to body frame, apply `camera_to_body()` to the rotation column vector `(m00, m10, m20)` → `(bx, by, bz)`, then:

```
body_yaw = atan2(by, bx)
```

With the default camera-to-body transform (`body_x = -cam_y, body_y = cam_x, body_z = cam_z`):
- `bx = -m10`, `by = m00`
- `body_yaw = atan2(m00, -m10)` = `camera_yaw + π/2`

For a tag with no in-plane rotation (θ=0), `m00 ∝ 1, m10 ∝ 0` → camera_yaw = 0, body_yaw = π/2. This offset is expected — the camera-to-body transform is a 90° rotation of the in-plane axes.

---

## Task 1: Add `tag_yaw` field to LandingTargetPose.msg

**Files:**
- Modify: `src/dbvf_msgs/msg/LandingTargetPose.msg`

- [ ] **Step 1: Add the field**

Add `float64 tag_yaw` after `bool position_valid` in `LandingTargetPose.msg`:

```
std_msgs/Header header
int32 tag_id
float64 tag_size
float64 angle_x
float64 angle_y
float64 position_x
float64 position_y
float64 position_z
bool position_valid
float64 tag_yaw
```

The field defaults to 0.0 when not set, which is safe for existing consumers — they don't read `tag_yaw` yet.

- [ ] **Step 2: Rebuild dbvf_msgs**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dbvf_msgs
```

Expected: successful build, no errors.

- [ ] **Step 3: Verify the field exists in the generated Python module**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 -c "from dbvf_msgs.msg import LandingTargetPose; m = LandingTargetPose(); print('tag_yaw:', m.tag_yaw)"
```

Expected output: `tag_yaw: 0.0`

- [ ] **Step 4: Commit**

```bash
git add src/dbvf_msgs/msg/LandingTargetPose.msg
git commit -m "feat(dbvf_msgs): add tag_yaw field to LandingTargetPose

ISS-015: Adds float64 tag_yaw to LandingTargetPose.msg for publishing
the tag-relative yaw angle extracted from homography decomposition."
```

---

## Task 2: Write failing tests for `estimate_tag_yaw`

**Files:**
- Modify: `src/dbvf_autonomy/test/test_tag_pose_estimation.py`

- [ ] **Step 1: Add the rotated homography test helper**

Add this helper function after the existing `_build_homography` helper (after line 24) in `test_tag_pose_estimation.py`:

```python
def _build_rotated_homography(tx, ty, tz, theta, fx, fy, cx, cy, tag_size):
    """Build homography for a tag at (tx,ty,tz) rotated by theta around optical axis.

    theta is the in-plane rotation of the tag in the camera frame (radians).
    With theta=0, this is equivalent to _build_homography (tag parallel to image plane).
    """
    hs = tag_size / 2.0
    c, s = math.cos(theta), math.sin(theta)
    return [
        fx * c * hs / tz, -fx * s * hs / tz, fx * tx / tz + cx,
        fy * s * hs / tz,  fy * c * hs / tz, fy * ty / tz + cy,
        0.0,               0.0,               1.0,
    ]
```

- [ ] **Step 2: Add import for `estimate_tag_yaw` and `camera_to_body`**

Update the import line at the top of the file:

```python
from dbvf_autonomy.tag_detector_adapter_node import estimate_tag_position, estimate_tag_yaw, camera_to_body
```

- [ ] **Step 3: Add test constants for camera-to-body transforms**

Add after the `CX, CY` constants (after line 7):

```python
# Camera-to-body transforms for yaw tests
# Identity: body axes = camera axes
IDENTITY_TRANSFORM = [(0, 1.0), (1, 1.0), (2, 1.0)]
# Default drone config: body_x = -cam_y, body_y = cam_x, body_z = cam_z
DEFAULT_TRANSFORM = [(1, -1.0), (0, 1.0), (2, 1.0)]
```

- [ ] **Step 4: Add yaw tests with identity transform**

Add these tests at the end of the file:

```python
# --- estimate_tag_yaw tests ---

def test_yaw_zero_rotation_identity_transform():
    """Tag with no rotation, identity cam-to-body → yaw = 0."""
    h = _build_rotated_homography(0.0, 0.0, 5.0, 0.0, FX, FY, CX, CY, 0.15)
    yaw = estimate_tag_yaw(h, FX, FY, CX, CY, IDENTITY_TRANSFORM)
    assert abs(yaw - 0.0) < 1e-6


def test_yaw_45_degrees_identity_transform():
    """Tag rotated 45° in camera frame, identity transform → yaw = π/4."""
    h = _build_rotated_homography(0.0, 0.0, 5.0, math.pi / 4, FX, FY, CX, CY, 0.15)
    yaw = estimate_tag_yaw(h, FX, FY, CX, CY, IDENTITY_TRANSFORM)
    assert abs(yaw - math.pi / 4) < 1e-6


def test_yaw_negative_90_identity_transform():
    """Tag rotated -90° in camera frame, identity transform → yaw = -π/2."""
    h = _build_rotated_homography(0.0, 0.0, 5.0, -math.pi / 2, FX, FY, CX, CY, 0.15)
    yaw = estimate_tag_yaw(h, FX, FY, CX, CY, IDENTITY_TRANSFORM)
    assert abs(yaw - (-math.pi / 2)) < 1e-6
```

- [ ] **Step 5: Add yaw tests with default drone transform**

```python
def test_yaw_zero_rotation_default_transform():
    """Tag with no rotation, default drone transform → yaw = π/2.

    Default: body_x = -cam_y, body_y = cam_x. The 90° axis remap adds π/2
    to the camera-frame angle.
    """
    h = _build_rotated_homography(0.0, 0.0, 5.0, 0.0, FX, FY, CX, CY, 0.15)
    yaw = estimate_tag_yaw(h, FX, FY, CX, CY, DEFAULT_TRANSFORM)
    assert abs(yaw - math.pi / 2) < 1e-6


def test_yaw_45_degrees_default_transform():
    """Tag rotated 45° in camera frame, default transform → yaw = 3π/4."""
    h = _build_rotated_homography(0.0, 0.0, 5.0, math.pi / 4, FX, FY, CX, CY, 0.15)
    yaw = estimate_tag_yaw(h, FX, FY, CX, CY, DEFAULT_TRANSFORM)
    assert abs(yaw - 3 * math.pi / 4) < 1e-6
```

- [ ] **Step 6: Add invariance tests**

```python
def test_yaw_scale_invariance():
    """Yaw should be identical regardless of homography scaling."""
    h_norm = _build_rotated_homography(0.5, 0.3, 4.0, math.pi / 6, FX, FY, CX, CY, 0.15)
    h_scaled = [v * 7.0 for v in h_norm]
    yaw1 = estimate_tag_yaw(h_norm, FX, FY, CX, CY, IDENTITY_TRANSFORM)
    yaw2 = estimate_tag_yaw(h_scaled, FX, FY, CX, CY, IDENTITY_TRANSFORM)
    assert abs(yaw1 - yaw2) < 1e-6


def test_yaw_independent_of_translation():
    """Yaw depends only on rotation, not on tag position."""
    yaw_a = estimate_tag_yaw(
        _build_rotated_homography(0.0, 0.0, 5.0, math.pi / 3, FX, FY, CX, CY, 0.15),
        FX, FY, CX, CY, IDENTITY_TRANSFORM)
    yaw_b = estimate_tag_yaw(
        _build_rotated_homography(2.0, -1.0, 3.0, math.pi / 3, FX, FY, CX, CY, 0.15),
        FX, FY, CX, CY, IDENTITY_TRANSFORM)
    assert abs(yaw_a - yaw_b) < 1e-6
```

- [ ] **Step 7: Run tests to verify they fail**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/dbvf_autonomy
python3 -m pytest test/test_tag_pose_estimation.py -v 2>&1 | tail -20
```

Expected: All new `test_yaw_*` tests FAIL with `ImportError: cannot import name 'estimate_tag_yaw'`. The 5 existing `test_tag_*` tests still PASS.

---

## Task 3: Implement `estimate_tag_yaw`

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/tag_detector_adapter_node.py`

- [ ] **Step 1: Add the `estimate_tag_yaw` function**

Add this function after `estimate_tag_position` (after line 118, before `camera_to_body`):

```python
def estimate_tag_yaw(homography, fx, fy, cx, cy, cam_body_transform):
    """Extract tag-relative yaw from homography, returned in body frame.

    Decomposes the first rotation column of K^-1 * H, transforms it through
    the camera-to-body axis remap, and returns atan2(body_y, body_x).

    Args:
        homography: 9-element flat array (row-major 3x3) from AprilTagDetection.
        fx, fy, cx, cy: Camera intrinsics from CameraInfo K matrix.
        cam_body_transform: List of 3 (axis_index, sign) tuples from _parse_axis().

    Returns:
        Yaw angle in radians (body frame). 0 = tag X-axis aligned with body X.
    """
    h = homography
    # First rotation column of K^-1 * H
    m00 = h[0] / fx - h[6] * cx / fx
    m10 = h[3] / fy - h[6] * cy / fy
    m20 = h[6]

    # Transform rotation column to body frame
    bx, by, _bz = camera_to_body(m00, m10, m20, cam_body_transform)

    return math.atan2(by, bx)
```

- [ ] **Step 2: Run tests to verify they pass**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/dbvf_autonomy
python3 -m pytest test/test_tag_pose_estimation.py -v
```

Expected: All 12 tests PASS (5 existing position tests + 7 new yaw tests).

- [ ] **Step 3: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/tag_detector_adapter_node.py src/dbvf_autonomy/test/test_tag_pose_estimation.py
git commit -m "feat(dbvf_autonomy): add estimate_tag_yaw pure function with tests

ISS-015: Extracts yaw from homography rotation column, applies
camera-to-body axis remap. 7 tests covering identity/default transforms,
scale invariance, and translation independence."
```

---

## Task 4: Wire up `tag_yaw` in `_publish_target`

**Files:**
- Modify: `src/dbvf_autonomy/dbvf_autonomy/tag_detector_adapter_node.py:237-263`

- [ ] **Step 1: Add `tag_yaw` population in `_publish_target`**

In the `_publish_target` method, add the `tag_yaw` assignment after `target.position_valid = True` (after line 261):

```python
        target.position_valid = True

        target.tag_yaw = estimate_tag_yaw(
            detection.homography, self.fx, self.fy, self.cx, self.cy,
            self.cam_body_transform)
```

- [ ] **Step 2: Rebuild both packages**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dbvf_msgs dbvf_autonomy
```

Expected: Successful build, no errors.

- [ ] **Step 3: Run the full test suite**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```

Expected: All 209 tests pass (202 existing + 7 new yaw tests). No regressions.

- [ ] **Step 4: Commit**

```bash
git add src/dbvf_autonomy/dbvf_autonomy/tag_detector_adapter_node.py
git commit -m "feat(dbvf_autonomy): publish tag_yaw in LandingTargetPose

ISS-015: tag_detector_adapter_node now populates the tag_yaw field
on every LandingTargetPose message using estimate_tag_yaw()."
```

---

## Verification Checklist

After all tasks complete:

- [ ] `colcon build --packages-select dbvf_msgs dbvf_autonomy` succeeds
- [ ] `colcon test --packages-select dbvf_autonomy && colcon test-result --verbose` — all tests pass, no regressions
- [ ] `python3 -c "from dbvf_msgs.msg import LandingTargetPose; print(LandingTargetPose().tag_yaw)"` prints `0.0`
- [ ] `ros2 topic echo /dbvf/landing_target_pose` (in sim with tag visible) shows `tag_yaw` populated with non-zero values when the drone is not aligned with the tag

---

## Out of Scope (Future Work — ISS-013)

This plan does **not** implement:
- Active yaw alignment in `precision_landing_node` (consuming `tag_yaw` to close the yaw loop)
- `target_yaw` parameter on `StartPrecisionLanding` service
- New FSM states for hold/align/offset sequencing

These are ISS-013 tasks that depend on this ISS-015 foundation.
