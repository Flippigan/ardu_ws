# Mesh Rotation & Course Alignment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rotate the photogrammetry mesh ~57deg clockwise in iris_runway.sdf to align its 215m long axis with the +X (East) course direction, covering all 5 competition zones (H through F2 at 152.4m).

**Architecture:** A standalone Python script loads the DAE mesh with trimesh, applies yaw rotation, sweeps candidate translations with ray-cast validation at all 5 zone positions, and outputs SDF pose values + AprilTag poses. Those values are then applied to the SDF world file manually. No autonomy node code changes.

**Tech Stack:** Python 3, trimesh, pycollada, numpy, ROS2 Humble (test verification only)

---

## File Structure

| File | Action | Responsibility |
|------|--------|----------------|
| `src/dbvf_autonomy/scripts/compute_mesh_rotation.py` | Create | Standalone mesh rotation computation script |
| `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf` | Modify (lines 82, 85, 120, 126) | Terrain pose + AprilTag poses |
| `src/dbvf_autonomy/config/mission_params.yaml` | Verify only | Confirm zone GPS coords still valid |
| `src/dbvf_autonomy/docs/Features/Course orrientation/mesh-orientation-analysis.md` | Append | Post-rotation results |

---

### Task 1: Install Dependencies and Verify Mesh Loading

**Files:**
- None (pip install + verification)

- [ ] **Step 1: Install trimesh with COLLADA support**

```bash
pip install trimesh pycollada numpy
```

- [ ] **Step 2: Verify trimesh can load the DAE mesh**

```bash
python3 -c "
import trimesh
scene = trimesh.load('/home/finn/Documents/ardu_ws/meshes/VFS Model v1 - Closed Squares/Untitled.dae')
if isinstance(scene, trimesh.Scene):
    mesh = trimesh.util.concatenate([g for g in scene.geometry.values() if isinstance(g, trimesh.Trimesh)])
else:
    mesh = scene
print(f'Vertices: {len(mesh.vertices):,}')
print(f'Faces: {len(mesh.faces):,}')
print(f'Bounds min: {mesh.bounds[0]}')
print(f'Bounds max: {mesh.bounds[1]}')
"
```

Expected: Prints vertex/face counts and bounding box matching known values (~156m X, ~201m Y, ~7m Z range). If COLLADA loading fails, try `pip install pyassimp` as a fallback loader.

---

### Task 2: Create Computation Script -- Mesh Loading and Rotation

**Files:**
- Create: `src/dbvf_autonomy/scripts/compute_mesh_rotation.py`

- [ ] **Step 1: Create script with constants, load, and rotation functions**

Create `src/dbvf_autonomy/scripts/compute_mesh_rotation.py`:

```python
#!/usr/bin/env python3
"""Compute mesh rotation and translation for VFS DBVF course alignment.

Loads the VFS photogrammetry DAE mesh, applies a yaw rotation to align the
strip's long axis with +X (East), finds the optimal translation so all 5
competition zones are covered, and outputs SDF pose values + AprilTag poses.

Usage:
    python3 compute_mesh_rotation.py
    python3 compute_mesh_rotation.py --yaw -60
    python3 compute_mesh_rotation.py --mesh /path/to/other.dae
"""

import argparse
import math
import sys

import numpy as np
import trimesh

# --- Constants ---

DEFAULT_MESH_PATH = (
    "/home/finn/Documents/ardu_ws/meshes/"
    "VFS Model v1 - Closed Squares/Untitled.dae"
)
DEFAULT_YAW_DEG = -57.0

# Competition zone world coordinates (X = East, Y = North)
ZONES = {
    "H":  (0.0, 0.0),
    "WA": (50.0, 1.0),
    "L":  (91.4, 0.0),
    "F1": (121.9, 0.0),
    "F2": (152.4, 0.0),
}

# AprilTag XY positions and Z offset above mesh surface
APRILTAG_POSITIONS = {
    "apriltag_wa_primary":   (50.0, 1.0),
    "apriltag_wa_secondary": (50.5, 1.0),
}
APRILTAG_Z_OFFSET = 0.01  # meters above mesh surface


def load_mesh(path: str) -> trimesh.Trimesh:
    """Load a DAE mesh file and return a single consolidated Trimesh."""
    loaded = trimesh.load(path)
    if isinstance(loaded, trimesh.Scene):
        meshes = [
            g for g in loaded.geometry.values()
            if isinstance(g, trimesh.Trimesh)
        ]
        if not meshes:
            print("ERROR: No triangle meshes found in DAE file.", file=sys.stderr)
            sys.exit(1)
        mesh = trimesh.util.concatenate(meshes)
    elif isinstance(loaded, trimesh.Trimesh):
        mesh = loaded
    else:
        print(
            f"ERROR: Unexpected type from trimesh.load: {type(loaded)}",
            file=sys.stderr,
        )
        sys.exit(1)
    return mesh


def apply_yaw_rotation(mesh: trimesh.Trimesh, yaw_deg: float) -> None:
    """Apply a Z-axis yaw rotation to the mesh vertices (in-place)."""
    yaw_rad = math.radians(yaw_deg)
    rot = trimesh.transformations.rotation_matrix(yaw_rad, [0, 0, 1])
    mesh.apply_transform(rot)
```

- [ ] **Step 2: Verify mesh loads and rotation changes bounds**

```bash
cd /home/finn/Documents/ardu_ws
python3 -c "
import sys; sys.path.insert(0, 'src/dbvf_autonomy/scripts')
from compute_mesh_rotation import load_mesh, apply_yaw_rotation
mesh = load_mesh('meshes/VFS Model v1 - Closed Squares/Untitled.dae')
print(f'Original bounds: X=[{mesh.bounds[0][0]:.1f}, {mesh.bounds[1][0]:.1f}]  '
      f'Y=[{mesh.bounds[0][1]:.1f}, {mesh.bounds[1][1]:.1f}]')
apply_yaw_rotation(mesh, -57.0)
print(f'Rotated bounds:  X=[{mesh.bounds[0][0]:.1f}, {mesh.bounds[1][0]:.1f}]  '
      f'Y=[{mesh.bounds[0][1]:.1f}, {mesh.bounds[1][1]:.1f}]')
"
```

Expected: After -57deg rotation, the X extent should increase (diagonal strip becomes more horizontal) and Y extent should decrease. X span should approach ~215m (the strip's true length).

- [ ] **Step 3: Commit**

```bash
git add src/dbvf_autonomy/scripts/compute_mesh_rotation.py
git commit -m "feat: add mesh rotation script with loading and rotation"
```

---

### Task 3: Add Ray Casting and Translation Sweep

**Files:**
- Modify: `src/dbvf_autonomy/scripts/compute_mesh_rotation.py`

- [ ] **Step 1: Add ray_cast_z and find_optimal_translation functions**

Append to `compute_mesh_rotation.py` after `apply_yaw_rotation`:

```python
def ray_cast_z(mesh: trimesh.Trimesh, x: float, y: float) -> float | None:
    """Cast a vertical ray downward at (x, y), return highest surface Z hit."""
    ray_origin = np.array([[x, y, 10000.0]])
    ray_direction = np.array([[0.0, 0.0, -1.0]])
    locations, _, _ = mesh.ray.intersects_location(
        ray_origins=ray_origin,
        ray_directions=ray_direction,
    )
    if len(locations) == 0:
        return None
    return float(locations[:, 2].max())


def find_optimal_translation(
    mesh: trimesh.Trimesh,
    zones: dict[str, tuple[float, float]],
) -> tuple[float, float, dict[str, float]]:
    """Find (tx, ty) so all zone points land on the rotated mesh surface.

    The SDF pose (tx, ty, tz, 0, 0, yaw) means:
        world_point = R_yaw * local_point + (tx, ty, tz)
    Since we pre-rotated the mesh for analysis, the relationship simplifies to:
        world_point = rotated_local_point + (tx, ty, tz)
    So a zone at world (zx, zy) maps to rotated-mesh coords (zx - tx, zy - ty).

    Strategy:
    1. Coarse sweep (2m steps) over candidate (tx, ty).
    2. For each candidate, bounds-check then ray-cast all 5 zones.
    3. Score by: (a) all zones hit, (b) H surface Z above median,
       (c) maximize F2 margin from mesh edge.
    4. Fine sweep (0.5m steps) around the coarse winner.

    Returns (tx, ty, zone_elevations_dict).
    """
    bounds = mesh.bounds  # [[xmin, ymin, zmin], [xmax, ymax, zmax]]
    x_min, y_min = bounds[0][0], bounds[0][1]
    x_max, y_max = bounds[1][0], bounds[1][1]

    median_z = float(np.median(mesh.vertices[:, 2]))
    f2_x = zones["F2"][0]

    def score_candidate(
        tx: float, ty: float
    ) -> tuple[float, dict[str, float]]:
        """Return (score, zone_z_dict). Negative score = invalid."""
        zone_z: dict[str, float] = {}

        # Quick bounds check for all zones
        for name, (zx, zy) in zones.items():
            mx, my = zx - tx, zy - ty
            if mx < x_min or mx > x_max or my < y_min or my > y_max:
                return -1.0, {}

        # Detailed ray-cast check
        for name, (zx, zy) in zones.items():
            mx, my = zx - tx, zy - ty
            z = ray_cast_z(mesh, mx, my)
            if z is None:
                return -1.0, {}
            zone_z[name] = z

        # Scoring
        h_above_median = 1.0 if zone_z["H"] > median_z else 0.0
        f2_mx = f2_x - tx
        f2_margin = min(f2_mx - x_min, x_max - f2_mx)

        return h_above_median * 1000.0 + f2_margin, zone_z

    # --- Coarse sweep ---
    coarse_step = 2.0
    tx_range = np.arange(x_min, x_max - f2_x, coarse_step)
    ty_range = np.arange(y_min + 5, y_max - 5, coarse_step)

    print(
        f"  Coarse sweep: {len(tx_range)} x {len(ty_range)} = "
        f"{len(tx_range) * len(ty_range)} candidates (step={coarse_step}m)"
    )

    best_score = -float("inf")
    best_tx, best_ty = 0.0, 0.0
    best_zone_z: dict[str, float] = {}

    for tx in tx_range:
        for ty in ty_range:
            s, zz = score_candidate(tx, ty)
            if s > best_score:
                best_score = s
                best_tx, best_ty = tx, ty
                best_zone_z = zz

    if best_score < 0:
        print(
            "ERROR: No valid translation found. All 5 zones cannot "
            "be covered simultaneously. Try a different --yaw angle.",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"  Coarse best: tx={best_tx:.1f}, ty={best_ty:.1f}, score={best_score:.1f}")

    # --- Fine sweep around coarse best ---
    fine_step = 0.5
    fine_range = 5.0
    tx_fine = np.arange(best_tx - fine_range, best_tx + fine_range, fine_step)
    ty_fine = np.arange(best_ty - fine_range, best_ty + fine_range, fine_step)

    for tx in tx_fine:
        for ty in ty_fine:
            s, zz = score_candidate(tx, ty)
            if s > best_score:
                best_score = s
                best_tx, best_ty = tx, ty
                best_zone_z = zz

    print(f"  Fine best:   tx={best_tx:.1f}, ty={best_ty:.1f}, score={best_score:.1f}")

    return best_tx, best_ty, best_zone_z
```

- [ ] **Step 2: Verify sweep finds a valid translation with all 5 zones covered**

```bash
cd /home/finn/Documents/ardu_ws
python3 -c "
import sys; sys.path.insert(0, 'src/dbvf_autonomy/scripts')
from compute_mesh_rotation import load_mesh, apply_yaw_rotation, find_optimal_translation, ZONES
mesh = load_mesh('meshes/VFS Model v1 - Closed Squares/Untitled.dae')
apply_yaw_rotation(mesh, -57.0)
tx, ty, zone_z = find_optimal_translation(mesh, ZONES)
print(f'\\nResult: tx={tx:.2f}, ty={ty:.2f}')
for name, z in zone_z.items():
    print(f'  {name}: Z={z:.3f}')
"
```

Expected: All 5 zones print Z values (none missing). `tx` and `ty` are within mesh bounds. If the sweep finds no valid candidate, the error message will guide you to adjust `--yaw`.

- [ ] **Step 3: Commit**

```bash
git add src/dbvf_autonomy/scripts/compute_mesh_rotation.py
git commit -m "feat: add ray casting and translation sweep to mesh rotation script"
```

---

### Task 4: Add Main Function and Complete Output

**Files:**
- Modify: `src/dbvf_autonomy/scripts/compute_mesh_rotation.py`

- [ ] **Step 1: Add main function with formatted output**

Append to `compute_mesh_rotation.py`:

```python
def main() -> None:
    parser = argparse.ArgumentParser(
        description="Compute mesh rotation for VFS DBVF course alignment",
    )
    parser.add_argument(
        "--mesh", default=DEFAULT_MESH_PATH, help="Path to DAE mesh file",
    )
    parser.add_argument(
        "--yaw",
        type=float,
        default=DEFAULT_YAW_DEG,
        help="Yaw rotation in degrees (default: -57)",
    )
    args = parser.parse_args()

    yaw_rad = math.radians(args.yaw)

    # --- Load and rotate ---
    print(f"Loading mesh: {args.mesh}")
    mesh = load_mesh(args.mesh)
    print(f"  Vertices: {len(mesh.vertices):,}")
    print(f"  Faces:    {len(mesh.faces):,}")
    print(
        f"  Bounds:   "
        f"X=[{mesh.bounds[0][0]:.1f}, {mesh.bounds[1][0]:.1f}]  "
        f"Y=[{mesh.bounds[0][1]:.1f}, {mesh.bounds[1][1]:.1f}]  "
        f"Z=[{mesh.bounds[0][2]:.1f}, {mesh.bounds[1][2]:.1f}]"
    )

    print(f"\nApplying yaw rotation: {args.yaw} deg ({yaw_rad:.6f} rad)")
    apply_yaw_rotation(mesh, args.yaw)
    print(
        f"  Rotated:  "
        f"X=[{mesh.bounds[0][0]:.1f}, {mesh.bounds[1][0]:.1f}]  "
        f"Y=[{mesh.bounds[0][1]:.1f}, {mesh.bounds[1][1]:.1f}]  "
        f"Z=[{mesh.bounds[0][2]:.1f}, {mesh.bounds[1][2]:.1f}]"
    )

    # --- Find optimal translation ---
    print("\nFinding optimal translation...")
    tx, ty, zone_z = find_optimal_translation(mesh, ZONES)

    # Compute tz: place H mesh surface at world Z = 0 (ground level)
    h_z_rotated = zone_z["H"]
    tz = -h_z_rotated

    # --- Zone elevation table ---
    print("\n" + "=" * 66)
    print("ZONE ELEVATION TABLE (World Coordinates)")
    print("=" * 66)
    print(f"{'Zone':<6} {'World (X, Y)':<18} {'Surface Z':>12} {'Status':>16}")
    print("-" * 66)
    for name, (zx, zy) in ZONES.items():
        z_rotated = zone_z.get(name)
        if z_rotated is not None:
            world_z = z_rotated + tz
            print(
                f"{name:<6} ({zx:>6.1f}, {zy:>4.1f})       "
                f"{world_z:>10.3f}m       {'COVERED':>8}"
            )
        else:
            print(
                f"{name:<6} ({zx:>6.1f}, {zy:>4.1f})              "
                f"N/A       {'OFF MESH':>8}"
            )

    # --- SDF model pose ---
    print("\n" + "=" * 66)
    print("SDF MODEL POSE (for custom_terrain_model in iris_runway.sdf)")
    print("=" * 66)
    print(f"\n  <pose>{tx:.4f} {ty:.4f} {tz:.4f} 0 0 {yaw_rad:.6f}</pose>\n")

    # --- AprilTag poses ---
    print("=" * 66)
    print("APRILTAG POSES")
    print("=" * 66)
    for tag_name, (ax, ay) in APRILTAG_POSITIONS.items():
        mx, my = ax - tx, ay - ty
        tag_z_rotated = ray_cast_z(mesh, mx, my)
        if tag_z_rotated is not None:
            tag_z_world = tag_z_rotated + tz + APRILTAG_Z_OFFSET
            print(f"\n  {tag_name}:")
            print(f"    <pose>{ax} {ay} {tag_z_world:.3f} 0 0 0</pose>")
        else:
            print(f"\n  {tag_name}: NO MESH SURFACE at ({ax}, {ay})")

    # --- Summary ---
    print("\n" + "=" * 66)
    print("SUMMARY")
    print("=" * 66)
    print(f"  Yaw rotation:  {args.yaw} deg ({yaw_rad:.6f} rad)")
    print(f"  Translation:   tx={tx:.4f}  ty={ty:.4f}  tz={tz:.4f}")
    print(f"  H surface Z:   {0.0:.3f}m (world)")
    wa_z = zone_z.get("WA")
    if wa_z is not None:
        print(f"  WA surface Z:  {wa_z + tz:.3f}m (world)")
    f2_z = zone_z.get("F2")
    if f2_z is not None:
        print(f"  F2 surface Z:  {f2_z + tz:.3f}m (world)")
    print(f"  Zones covered: {sum(1 for v in zone_z.values() if v is not None)}/5")
    print()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Make executable and run the complete script**

```bash
chmod +x /home/finn/Documents/ardu_ws/src/dbvf_autonomy/scripts/compute_mesh_rotation.py
cd /home/finn/Documents/ardu_ws
python3 src/dbvf_autonomy/scripts/compute_mesh_rotation.py
```

Expected output includes:
- Zone elevation table with all 5 zones showing "COVERED"
- SDF model pose line with yaw ~= -0.9948 rad
- Two AprilTag pose lines with computed Z values
- Summary showing 5/5 zones covered

- [ ] **Step 3: Verify output sanity**

Check these conditions in the script output:
- All 5 zones show "COVERED" (not "OFF MESH")
- Surface Z values span a reasonable range (the mesh has ~7m total elevation variation)
- AprilTag Z values are slightly above (~0.01m) their respective zone surface Z
- F2 at 152.4m is within the mesh boundary

If any zone shows "OFF MESH", try `--yaw -55` or `--yaw -60` and note which value covers all zones.

- [ ] **Step 4: Commit**

```bash
git add src/dbvf_autonomy/scripts/compute_mesh_rotation.py
git commit -m "feat: complete mesh rotation script with output formatting

Standalone script that loads the VFS photogrammetry DAE mesh,
applies yaw rotation, finds optimal translation via ray-cast
sweep, and outputs SDF pose + AprilTag pose values."
```

---

### Task 5: Update iris_runway.sdf with Computed Values

**Files:**
- Modify: `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf` (lines 82, 85, 120, 126)

**Prerequisite:** Record the exact values from the Task 4 Step 2 script output before starting this task. You need the SDF pose line and both AprilTag pose lines.

- [ ] **Step 1: Update the terrain model comment and pose**

In `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf`, replace the comment on line 82:

```xml
<!-- Before (line 82) -->
<!-- Pose compensated for mesh offset: geometry centered at (-0.48, -0.92, -0.28) after 0.01 scale -->

<!-- After -->
<!-- Pose: rotated -57 deg to align strip long axis with +X course direction -->
```

Then replace the terrain model pose on line 85:

```xml
<!-- Before (line 85) -->
<pose>30.48 0.92 30.2 0 0 0</pose>

<!-- After: paste EXACT values from script "SDF MODEL POSE" output -->
<pose>TX TY TZ 0 0 YAW_RAD</pose>
```

Replace `TX TY TZ` and `YAW_RAD` with the exact numeric values from the script output.

- [ ] **Step 2: Update AprilTag primary pose**

Replace the primary AprilTag pose on line 120:

```xml
<!-- Before (line 120) -->
<pose>50.0 1.0 2.457 0 0 0</pose>

<!-- After: paste EXACT Z from script "APRILTAG POSES" / apriltag_wa_primary output -->
<pose>50.0 1.0 Z_NEW 0 0 0</pose>
```

- [ ] **Step 3: Update AprilTag secondary pose**

Replace the secondary AprilTag pose on line 126:

```xml
<!-- Before (line 126) -->
<pose>50.5 1.0 2.457 0 0 0</pose>

<!-- After: paste EXACT Z from script "APRILTAG POSES" / apriltag_wa_secondary output -->
<pose>50.5 1.0 Z_NEW 0 0 0</pose>
```

- [ ] **Step 4: Verify the SDF is valid XML**

```bash
python3 -c "
import xml.etree.ElementTree as ET
ET.parse('/home/finn/Documents/ardu_ws/src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf')
print('SDF XML is valid')
"
```

Expected: "SDF XML is valid" with no errors.

- [ ] **Step 5: Commit**

```bash
git add src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf
git commit -m "feat: rotate terrain mesh -57 deg for full course coverage

All 5 competition zones (H through F2 at 152.4m) now have mesh
surface coverage. AprilTag Z positions updated to match new
terrain surface elevation."
```

---

### Task 6: Verify Mission Parameters

**Files:**
- Verify: `src/dbvf_autonomy/config/mission_params.yaml`

- [ ] **Step 1: Confirm all zones are covered -- no mission_params changes needed**

The GPS coordinates in `mission_params.yaml` map to world XY positions via the SDF `<spherical_coordinates>`. The mesh rotation changes what terrain is UNDER those positions, not the positions themselves. Since the script (Task 4) confirmed all 5 zones are "COVERED", the GPS waypoints remain valid.

Read `src/dbvf_autonomy/config/mission_params.yaml` and verify:
- `home_lat/home_lon` maps to world origin (0, 0) -- H zone, covered
- `wa_lat/wa_lon` maps to approximately (50, 1) -- WA zone, covered
- `f1_lat/f1_lon` and `f2_lat/f2_lon` -- now newly covered by the rotated mesh
- Altitude values (`transit_altitude_ft: 35.0`) are AGL, unaffected by mesh position

If all zones are covered, **no changes to mission_params.yaml are needed**. Only modify if the script output showed "OFF MESH" for any zone.

---

### Task 7: Run Existing Tests

**Files:**
- None (verification only)

- [ ] **Step 1: Build DBVF packages**

```bash
source /opt/ros/humble/setup.bash
cd /home/finn/Documents/ardu_ws
colcon build --packages-select dbvf_msgs dbvf_autonomy
```

Expected: Build succeeds with no errors.

- [ ] **Step 2: Run unit tests**

```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
colcon test --packages-select dbvf_autonomy
colcon test-result --verbose
```

Expected: All tests pass (currently 38 tests across 5 files). These are pure-function tests for mode maps, debounce, angles, tag selection, and FSM transitions -- none depend on mesh geometry.

---

### Task 8: Update Documentation

**Files:**
- Modify: `src/dbvf_autonomy/docs/Features/Course orrientation/mesh-orientation-analysis.md` (append)

- [ ] **Step 1: Append post-rotation results to analysis doc**

Append to the end of `src/dbvf_autonomy/docs/Features/Course orrientation/mesh-orientation-analysis.md`. Fill in all placeholder values from the Task 4 script output:

```markdown

---

## Post-Rotation Results (2026-03-29)

**Applied rotation:** -57 deg (-0.9948 rad) clockwise about Z axis
**New SDF model pose:** `<pose>TX TY TZ 0 0 YAW_RAD</pose>`
**Previous SDF model pose:** `<pose>30.48 0.92 30.2 0 0 0</pose>`

### Course Zone Coverage After Rotation

| Zone | World (X, Y) | Surface Z | Coverage Status |
|------|-------------|-----------|-----------------|
| H    | (0, 0)      | Z_H m     | COVERED         |
| WA   | (50.0, 1.0) | Z_WA m    | COVERED         |
| L    | (91.4, 0)   | Z_L m     | COVERED         |
| F1   | (121.9, 0)  | Z_F1 m    | COVERED         |
| F2   | (152.4, 0)  | Z_F2 m    | COVERED         |

All 5 zones previously limited to 61m of coverage now have full mesh surface data up to 152.4m.

### AprilTag Poses After Rotation

| Tag | Pose |
|-----|------|
| apriltag_wa_primary   | `<pose>50.0 1.0 Z_PRI 0 0 0</pose>` |
| apriltag_wa_secondary | `<pose>50.5 1.0 Z_SEC 0 0 0</pose>` |

### Verification

- All 5 competition zones have mesh surface coverage
- F1 and F2 (previously 60-90m beyond mesh edge) are now within the rotated strip
- Existing unit tests pass (38/38) -- no regressions
- Mission parameters (GPS coords, altitudes) unchanged
- Computation script: `src/dbvf_autonomy/scripts/compute_mesh_rotation.py`
```

Replace all `TX`, `TY`, `TZ`, `YAW_RAD`, `Z_*` placeholders with actual values from the script output.

- [ ] **Step 2: Commit**

```bash
git add "src/dbvf_autonomy/docs/Features/Course orrientation/mesh-orientation-analysis.md"
git commit -m "docs: add post-rotation mesh coverage results"
```
