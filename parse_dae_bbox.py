#!/usr/bin/env python3
"""Parse a COLLADA .dae file to determine bounding box dimensions."""

import xml.etree.ElementTree as ET
import sys
import numpy as np

DAE_FILE = "/home/finn/Documents/ardu_ws/meshes/VFS Model v1 - Closed Squares/Untitled.dae"

NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}

tree = ET.parse(DAE_FILE)
root = tree.getroot()

# 1. Determine up_axis
up_axis_el = root.find(".//c:asset/c:up_axis", NS)
up_axis = up_axis_el.text if up_axis_el is not None else "Y_UP"
print(f"Up axis: {up_axis}")

# 2. Determine unit scale
unit_el = root.find(".//c:asset/c:unit", NS)
if unit_el is not None:
    unit_name = unit_el.get("name", "unknown")
    unit_meter = float(unit_el.get("meter", "1"))
    print(f"Unit: {unit_name} (1 unit = {unit_meter} meters)")
else:
    unit_meter = 1.0
    print("Unit: not specified (assuming meters)")

# 3. Find all position float_arrays
all_vertices = []

# Look for sources with "positions" in their id
for source in root.iter(f"{{{NS['c']}}}source"):
    source_id = source.get("id", "")
    if "position" not in source_id.lower():
        continue

    float_array_el = source.find(f"{{{NS['c']}}}float_array", NS)
    if float_array_el is None:
        continue

    count = int(float_array_el.get("count", "0"))
    print(f"\nFound position source: {source_id}")
    print(f"  Float count: {count} ({count // 3} vertices)")

    # Parse the float data
    floats = np.array(float_array_el.text.split(), dtype=np.float64)
    assert len(floats) == count, f"Expected {count} floats, got {len(floats)}"

    # Reshape to Nx3
    vertices = floats.reshape(-1, 3)
    all_vertices.append(vertices)

if not all_vertices:
    print("ERROR: No position data found!")
    sys.exit(1)

# Combine all vertices
vertices = np.vstack(all_vertices)
print(f"\nTotal vertices: {len(vertices)}")

# 4. Compute bounding box
mins = vertices.min(axis=0)
maxs = vertices.max(axis=0)
dims = maxs - mins
center = (mins + maxs) / 2.0

# COLLADA axis labels depend on up_axis convention
if up_axis == "Z_UP":
    axis_labels = ["X", "Y", "Z"]
    print("\nCOLLADA Z_UP convention: X=right, Y=forward, Z=up")
    print("Gazebo ENU mapping: COLLADA X->Gazebo X(East), COLLADA Y->Gazebo Y(North), COLLADA Z->Gazebo Z(Up)")
elif up_axis == "Y_UP":
    axis_labels = ["X", "Y", "Z"]
    print("\nCOLLADA Y_UP convention: X=right, Y=up, Z=back")
    print("NOTE: Gazebo expects Z-up. You may need to rotate -90 deg around X when importing.")
else:
    axis_labels = ["X", "Y", "Z"]

print("\n" + "=" * 60)
print("BOUNDING BOX ANALYSIS (in file units = meters)")
print("=" * 60)

print(f"\n{'Axis':<6} {'Min':>12} {'Max':>12} {'Dimension':>12} {'Center':>12}")
print("-" * 60)
for i, label in enumerate(axis_labels):
    print(f"{label:<6} {mins[i]:12.4f} {maxs[i]:12.4f} {dims[i]:12.4f} {center[i]:12.4f}")

print(f"\n{'Total vertices:':<20} {len(vertices)}")

# Identify long axis
long_axis_idx = np.argmax(dims)
print(f"\nLong axis: {axis_labels[long_axis_idx]} ({dims[long_axis_idx]:.4f} m)")
short_axis_idx = np.argmin(dims)
print(f"Short axis: {axis_labels[short_axis_idx]} ({dims[short_axis_idx]:.4f} m)")

# Origin analysis
print(f"\n--- Origin Position Relative to Geometry ---")
for i, label in enumerate(axis_labels):
    if abs(center[i]) < 0.01 * dims[i]:
        loc = "centered"
    elif abs(mins[i]) < 0.01 * dims[i]:
        loc = f"at min edge (origin near {label}-min)"
    elif abs(maxs[i]) < 0.01 * dims[i]:
        loc = f"at max edge (origin near {label}-max)"
    else:
        pct = (0 - mins[i]) / dims[i] * 100
        loc = f"offset: origin at {pct:.1f}% from min"
    print(f"  {label}: {loc} (center at {center[i]:.4f})")

# Summary
print(f"\n--- Summary ---")
if up_axis == "Z_UP":
    print(f"Width  (X): {dims[0]:.4f} m")
    print(f"Depth  (Y): {dims[1]:.4f} m")
    print(f"Height (Z): {dims[2]:.4f} m")
    print(f"\nIn Gazebo (ENU, Z-up): no rotation needed.")
    print(f"  East-West extent:  {dims[0]:.4f} m")
    print(f"  North-South extent: {dims[1]:.4f} m")
    print(f"  Vertical extent:   {dims[2]:.4f} m")
elif up_axis == "Y_UP":
    print(f"Width  (X): {dims[0]:.4f} m")
    print(f"Height (Y): {dims[1]:.4f} m")
    print(f"Depth  (Z): {dims[2]:.4f} m")
    print(f"\nIn Gazebo (ENU, Z-up): rotate -90 deg around X axis.")
    print(f"  This maps COLLADA Y-up to Gazebo Z-up.")
