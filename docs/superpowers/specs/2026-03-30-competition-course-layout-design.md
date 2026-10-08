# DBVF Competition Course Simulation Layout — Design Spec

**Date:** 2026-03-30
**Status:** Approved

## Overview

Model the VFS DBVF competition course in Gazebo simulation using reusable zone models placed on the existing photogrammetry mesh. The course consists of 6 zones arranged along an east-west axis, with the drone starting at Home (origin) and the course extending eastward (+X).

## Course Layout

The course runs +X (East) from the drone's starting position at the Gazebo world origin. All zones except WA/WM sit on the main east-west axis. WA and WM straddle the axis at the same X position, with their midpoint on the centerline.

### Zone Positions (Gazebo ENU Frame)

| Zone | Purpose | Model | X (m) | Y (m) | Z (m) |
|------|---------|-------|-------|-------|-------|
| H | Home / Takeoff & Return | zone_15x15ft | 0.0 | 0.0 | 0.000 |
| WM | Blue payload pickup (manual) | zone_20x20ft | 45.7 | +4.57 | 1.272 |
| WA | Yellow payload pickup (autonomous) | zone_20x20ft | 45.7 | -4.57 | 1.272 |
| L | Precision landing (FM-1) | zone_15x15ft | 91.4 | 0.0 | 0.916 |
| F1 | Drop zone — 2.5 pts/payload | zone_7x7ft | 121.9 | 0.0 | -0.055 |
| F2 | Drop zone — 5 pts/payload | zone_3x3ft | 152.4 | 0.0 | -1.421 |

### Distances

| From | To | Distance | Direction |
|------|----|----------|-----------|
| H | WA/WM | 150 ft (45.7 m) | East |
| WA | WM | 30 ft (9.14 m) | North-South |
| WA/WM | L | 150 ft (45.7 m) | East |
| L | F1 | 100 ft (30.5 m) | East |
| F1 | F2 | 100 ft (30.5 m) | East |

Total course length: ~600 ft (~183 m).

### Z Values

Z positions are sampled from the photogrammetry mesh surface (`meshes/VFS Model v1 - Closed Squares/Untitled.dae`) at each zone's XY coordinate. The mesh has ~2.7m of elevation variation across the course. Each zone is placed flush on the mesh surface.

## Zone Models

Four reusable Gazebo models created in `src/ardupilot_gazebo/models/`:

### Model Specifications

| Model | Tarp Dimensions | Used By |
|-------|----------------|---------|
| `zone_15x15ft` | 4.572 × 4.572 m | H, L |
| `zone_20x20ft` | 6.096 × 6.096 m | WA, WM |
| `zone_7x7ft` | 2.134 × 2.134 m | F1 |
| `zone_3x3ft` | 0.914 × 0.914 m | F2 |

### Model Components

Each model contains:

- **Tarp:** Flat box, 0.005m thick, blue PBR material (mimicking standard department-store blue tarps). Positioned at z = 0.0025m (half thickness) relative to model origin.
- **4 corner stakes:** Cylinders, 0.0254m diameter (~1 inch), 0.3048m tall (12 inches — RFP max). Brown wood color. Placed at the four corners of the tarp.
- **Static:** Yes. No physics simulation.
- **Collision:** Disabled or minimal — the drone should not interact with these physically.
- **File structure:** Standard Gazebo model with `model.config` + `model.sdf`.

### Visual Style

- Tarp: Blue PBR material (RGB ~0.1, 0.3, 0.7), slight roughness (0.6-0.8) to avoid unrealistic reflections.
- Stakes: Brown PBR material (RGB ~0.5, 0.3, 0.15), wood-like appearance.

## AprilTag Placement

- Existing `Apriltag36_11_00000` (0.6m, tag36h11 family) placed centered on WA zone at (45.7, -4.57, 1.272).
- Existing `Apriltag36_11_00001` (0.15m, tag36h11 family) placed centered on WA zone, co-located with primary tag.
- No AprilTags on other zones. L uses GPS-based precision landing.

## World File Changes

Modify `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf` to add `<include>` blocks for each zone and reposition the AprilTags onto WA.

The existing photogrammetry mesh (`custom_terrain_model`) and runway model remain unchanged.

## Files

| Action | Path |
|--------|------|
| CREATE | `src/ardupilot_gazebo/models/zone_15x15ft/model.config` |
| CREATE | `src/ardupilot_gazebo/models/zone_15x15ft/model.sdf` |
| CREATE | `src/ardupilot_gazebo/models/zone_20x20ft/model.config` |
| CREATE | `src/ardupilot_gazebo/models/zone_20x20ft/model.sdf` |
| CREATE | `src/ardupilot_gazebo/models/zone_7x7ft/model.config` |
| CREATE | `src/ardupilot_gazebo/models/zone_7x7ft/model.sdf` |
| CREATE | `src/ardupilot_gazebo/models/zone_3x3ft/model.config` |
| CREATE | `src/ardupilot_gazebo/models/zone_3x3ft/model.sdf` |
| MODIFY | `src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf` |

## Out of Scope

- GUI for swapping GPS coordinates (handled by another team)
- Ground zones 1 & 2 (team member positions, not relevant to drone sim)
- Payload models or pickup/drop mechanics
- Autonomous mission sequencing logic
- Changes to the photogrammetry mesh
