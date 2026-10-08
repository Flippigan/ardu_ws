# Per-Landing Offset for Precision Landing

**Date:** 2026-04-01
**Status:** Approved

## Problem

The PiCam is mounted forward of center on the drone (~4-6 inches / 10-15 cm), while the reload mechanism is at the center bottom. During WA precision landing, the drone needs to position the mechanism (not the camera) over the AprilTag. Other landing sites (L, H) should land centered (camera over tag).

The precision landing node already has `offset_forward` and `offset_right` parameters and DESCEND_OFFSET / DESCEND_FINAL states that apply them, but the offsets are hardcoded at launch time and set to 0.0.

## Solution

Pass offset values per-landing via the `StartPrecisionLanding` service request. The mission sequencer provides the appropriate offset for each landing site. Config-file only — values live in `mission_params.yaml`.

## Changes

### 1. `StartPrecisionLanding.srv`

Add two fields with default values of 0.0 for backward compatibility:

```
float64 target_lat
float64 target_lon
float64 offset_forward 0.0
float64 offset_right 0.0
---
bool success
string message
```

Standalone callers that omit offset fields get centered landing (0.0, 0.0).

### 2. `precision_landing_node.py`

In `_start_landing_cb`, read `request.offset_forward` and `request.offset_right` and store them on `self.offset_forward` / `self.offset_right`. These override the parameter defaults for the duration of the landing.

The existing `_velocity_servo` method already uses `self.offset_forward` and `self.offset_right` in the PID error computation — no changes needed there.

Remove `offset_forward` and `offset_right` from the node's declared parameters since they are now per-landing, not per-node.

### 3. `mission_params.yaml`

Add under the existing WA config section:

```yaml
# WA Precision Landing Offset (camera-to-mechanism)
wa_offset_forward: 0.0   # metres, positive = mechanism behind camera
wa_offset_right: 0.0     # metres, positive = mechanism right of camera
```

### 4. `mission_sequencer_node.py`

When calling `start_precision_landing` for WA landing sites (`LAND_WA_DESCEND`, `LAND_WA_FINAL`), pass `wa_offset_forward` and `wa_offset_right` from the mission config. For L and H landings, pass (0.0, 0.0).

### 5. `mission_helpers.py`

Add `wa_offset_forward` (default 0.0) and `wa_offset_right` (default 0.0) to `DEFAULT_MISSION_CONFIG` and `validate_mission_config`.

### 6. `sim_params.yaml`

Remove `offset_forward` and `offset_right` from the precision_landing section (they move to the service request).

### 7. Tests

- Update service call tests for the new `StartPrecisionLanding` fields.
- Update mission config validation tests for new keys.
- Add test: precision landing node uses request offset values (not parameter defaults).
- Add test: mission state machine / sequencer passes correct offsets per landing site.

## Physical Measurements Required

With the drone on a flat surface, measure from the **center of the PiCam lens** to the **center of the reload mechanism**:

| Measurement | Axis | Sign Convention |
|---|---|---|
| `wa_offset_forward` | Nose-to-tail | Positive = mechanism is behind camera |
| `wa_offset_right` | Left-to-right | Positive = mechanism is right of camera |

For the current hardware layout (PiCam forward, mechanism at center), expect `wa_offset_forward` ~ 0.10-0.15m and `wa_offset_right` ~ 0.0m.

### How to Measure

1. Place drone on flat surface, level.
2. Mark a point directly below the center of the PiCam lens on the surface.
3. Mark a point directly below the center of the reload mechanism.
4. Measure the forward/backward distance between marks (along the drone's nose-tail axis). If the mechanism mark is behind the camera mark, the value is positive.
5. Measure the left/right distance between marks (perpendicular to nose-tail). If the mechanism mark is to the right of the camera mark, the value is positive.
6. Convert inches to metres (divide by 39.37) and enter into `mission_params.yaml`.

## Data Flow

```
mission_params.yaml
    ↓ (wa_offset_forward, wa_offset_right)
mission_sequencer_node
    ↓ StartPrecisionLanding(target_lat, target_lon, offset_forward, offset_right)
precision_landing_node
    ↓ stores as self.offset_forward, self.offset_right
    ↓ DESCEND_OFFSET state: error_x -= offset_forward, error_y -= offset_right
    ↓ PID drives drone so mechanism is over tag
```
