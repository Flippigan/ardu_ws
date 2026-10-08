# Jetson Orin Nano — Cube Orange Integration Design

**Date:** 2026-04-01
**Status:** Approved
**Scope:** Physical wiring, Orin Nano software stack, config-driven sim/real switchover, CSI camera pipeline

## Overview

This spec describes how to transition the `dbvf_autonomy` stack from simulation (ArduPilot SITL + Gazebo on a development machine) to a physical aircraft (Cube Orange flight controller + Jetson Orin Nano companion computer + Arducam IMX219 CSI camera).

The design minimizes code changes — only `mavlink_interface_node` gains serial/UDP connection support and a new `hardware_params.yaml` config file is added. All other nodes (tag detector, precision landing, mission sequencer) run unchanged.

### Hardware

- **Flight controller:** Cube Orange (standard, no Ethernet)
- **Companion computer:** Jetson Orin Nano 4GB, JetPack 6.x (L4T R36.x, Ubuntu 22.04)
- **Camera:** Arducam IMX219 8MP, CSI connector
- **Connection:** TELEM2 serial (UART), 921600 baud

### Approach

Hybrid (Approach C): Direct serial connection for immediate testing, with `hardware_params.yaml` supporting `tcp`, `serial`, and `udp` connection types. When QGC access is needed alongside autonomous operation, `mavlink-routerd` can be added and the config flipped to `udp` — no code changes required.

---

## 1. Physical Wiring

### Cube Orange TELEM2 → Orin Nano UART1

The Cube Orange TELEM2 port is a 6-pin JST-GH connector. Three wires are needed:

| Cube Orange TELEM2 Pin | Signal   | Orin Nano J12 Pin | Signal    |
|------------------------|----------|-------------------|-----------|
| Pin 2                  | TX (out) | Pin 10            | UART1 RX  |
| Pin 3                  | RX (in)  | Pin 8             | UART1 TX  |
| Pin 6                  | GND      | Pin 6 (any GND)   | GND       |

**Do NOT connect the 5V pin** (TELEM2 Pin 1). The Orin has its own power supply. Both sides are 3.3V logic — no level shifter is needed.

### Cube Orange ArduPilot Parameters

Set via QGC or MAVProxy on the real aircraft:

```
SERIAL2_PROTOCOL = 2      # MAVLink2
SERIAL2_BAUD = 921        # 921600 baud
MAV_SYS_ID = 1            # Vehicle system ID
```

The existing PLND (precision landing) parameters from `gazebo-iris-hardmount.parm` apply identically on the real aircraft. The MAVLink LANDING_TARGET messages are transport-agnostic.

### Orin Nano UART Setup (one-time)

```bash
# Disable serial console on UART1
sudo systemctl stop nvgetty
sudo systemctl disable nvgetty

# Add user to dialout group for serial access
sudo usermod -aG dialout $USER

# Reboot, then verify device exists
ls -la /dev/ttyTHS1
```

---

## 2. Orin Nano Software Stack

### 2a. ROS2 Humble

```bash
sudo apt install ros-humble-ros-base ros-humble-image-transport \
  ros-humble-camera-info-manager python3-colcon-common-extensions
```

Use `ros-base` (not `ros-desktop`) — saves ~1.5GB. No rviz or Gazebo needed on the aircraft.

### 2b. CSI Camera Node

Pipeline: `IMX219 CSI → nvarguscamerasrc (HW ISP) → ROS2 image topic`

The camera node uses this GStreamer pipeline:

```
nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! appsink
```

It publishes to `/camera/image` and `/camera/camera_info` — the same topics the existing `tag_detector_adapter_node` subscribes to. No downstream changes needed.

Use a lightweight GStreamer-based ROS2 camera node (e.g., `ros2_csi_camera` or a custom node in `dbvf_autonomy`). Avoid Isaac ROS `jetson_camera` — its dependency chain is too heavy for 4GB.

### 2c. apriltag Library + apriltag_ros

```bash
# Build apriltag C library into /usr/local on the Orin
cd /tmp && git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag && cmake -B build && cmake --build build && sudo cmake --install build

# apriltag_ros v3.3.0 in the colcon workspace (same version as dev machine)
```

### 2d. Workspace Packages

Only the packages needed for the real aircraft — no simulation dependencies:

```
src/
  dbvf_msgs/          # Messages and services
  dbvf_autonomy/      # 5 autonomy nodes
  apriltag_ros/       # v3.3.0
  apriltag_msgs/      # Message definitions
```

### 2e. pymavlink

```bash
pip3 install pymavlink==2.4.43   # Same version as dev machine
```

### 2f. Camera Calibration

The real IMX219 needs a calibration YAML file for accurate tag pose estimation. Generate with:

```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  --ros-args -r image:=/camera/image -r camera_info:=/camera/camera_info
```

The resulting YAML is referenced in `hardware_params.yaml` via `camera_info_url`.

### Memory Budget (4GB Orin Nano)

| Component                          | Estimated RAM |
|------------------------------------|---------------|
| JetPack 6 base                     | ~600MB        |
| ROS2 Humble (ros-base)             | ~150MB        |
| 5 autonomy nodes + pymavlink       | ~100MB        |
| CSI camera pipeline (nvarguscamerasrc) | ~200MB    |
| apriltag_ros                       | ~100MB        |
| **Total**                          | **~1.15GB**   |
| **Remaining headroom**             | **~2.85GB**   |

---

## 3. Config-Driven Sim/Real Switchover

### New File: `config/hardware_params.yaml`

```yaml
mavlink:
  connection_type: "serial"           # "tcp", "serial", or "udp"
  # Serial settings (connection_type: serial)
  serial_device: "/dev/ttyTHS1"
  serial_baud: 921600
  # TCP settings (connection_type: tcp)
  tcp_host: "127.0.0.1"
  tcp_port: 5762
  # UDP settings (connection_type: udp)
  udp_host: "127.0.0.1"
  udp_port: 14550

camera:
  type: "csi"                         # "csi" or "gazebo"
  sensor_id: 0
  width: 1280
  height: 720
  framerate: 30
  camera_info_url: "file:///path/to/colcon_ws/src/dbvf_autonomy/config/imx219_calibration.yaml"
```

`sim_params.yaml` stays unchanged.

### Launch File Strategy

Create `mission_real.launch.py` that:
- Loads `hardware_params.yaml` instead of `sim_params.yaml`
- Launches `csi_camera_node` instead of relying on Gazebo camera
- Launches the same 5 autonomy nodes with identical configuration
- Does NOT launch any Gazebo or simulation nodes

Alternatively, add `use_sim` argument to existing launch files:

```bash
# Simulation (unchanged)
ros2 launch dbvf_autonomy mission_sim.launch.py

# Real aircraft
ros2 launch dbvf_autonomy mission_real.launch.py
```

### mavlink_interface_node Code Change

The only code change. Currently:

```python
self.connection = mavutil.mavlink_connection('tcp:127.0.0.1:5762')
```

Updated to read config and branch:

```python
if connection_type == "serial":
    self.connection = mavutil.mavlink_connection(serial_device, baud=serial_baud)
elif connection_type == "tcp":
    self.connection = mavutil.mavlink_connection(f'tcp:{tcp_host}:{tcp_port}')
elif connection_type == "udp":
    self.connection = mavutil.mavlink_connection(f'udpin:{udp_host}:{udp_port}')
```

Everything downstream (services, publishers, LANDING_TARGET messages) is identical. MAVLink is transport-agnostic.

---

## 4. Camera Pipeline — Sim vs Real

### The Contract

The tag detection pipeline (`apriltag_ros` → `tag_detector_adapter_node` → `precision_landing_node`) subscribes to `/camera/image` and `/camera/camera_info`. It does not care about the image source.

### Sim Pipeline (unchanged)

```
Gazebo camera plugin → /camera/image + /camera/camera_info
```

### Real Pipeline

```
IMX219 CSI → nvarguscamerasrc → csi_camera_node → /camera/image + /camera/camera_info
```

### What Changes vs What Stays the Same

| Component                    | Sim                  | Real                     |
|------------------------------|----------------------|--------------------------|
| Image source                 | Gazebo plugin        | `csi_camera_node`        |
| Camera intrinsics            | SDF hardcoded        | Calibration YAML         |
| MAVLink transport            | TCP to SITL          | Serial to Cube Orange    |
| Config file                  | `sim_params.yaml`    | `hardware_params.yaml`   |
| Launch command               | `mission_sim.launch.py` | `mission_real.launch.py` |

| Component (unchanged)        | Notes                                          |
|------------------------------|-------------------------------------------------|
| `apriltag_ros`               | Same config, tag36h11, same tag sizes           |
| `tag_detector_adapter_node`  | Same dual-tag switching, same debounce          |
| `precision_landing_node`     | Same state machine, same PLND handoff           |
| `mavlink_interface_node`     | Same MAVLink messages, different transport only  |
| `mission_sequencer_node`     | Completely unchanged                            |

---

## 5. Future: MAVLink Router Upgrade Path

When the GCS operator needs QGC telemetry access simultaneously with autonomous operation:

1. Install `mavlink-routerd` on the Orin Nano
2. Configure it to own `/dev/ttyTHS1` and expose UDP endpoints
3. Change `hardware_params.yaml`: `connection_type: "udp"`, `udp_port: 14550`
4. QGC on the GCS laptop connects to the Orin's IP on a second UDP port

No code changes required — the `udp` path is already implemented.

---

## 6. Orin Nano Setup Checklist

Ordered steps for setting up a fresh Orin Nano:

1. Flash JetPack 6.x via SDK Manager (if not already done)
2. Disable nvgetty, add user to dialout group, reboot
3. Install ROS2 Humble (`ros-humble-ros-base` + deps)
4. Install pymavlink 2.4.43
5. Build apriltag C library from source into `/usr/local`
6. Clone/copy workspace: `dbvf_msgs`, `dbvf_autonomy`, `apriltag_ros`, `apriltag_msgs`
7. `colcon build` the workspace
8. Connect Arducam IMX219 to CSI, verify with `nvgstcapture-1.0`
9. Run camera calibration, save YAML
10. Update `camera_info_url` in `hardware_params.yaml`
11. Wire TELEM2 → UART1 (TX/RX/GND)
12. Set Cube Orange params (`SERIAL2_PROTOCOL=2`, `SERIAL2_BAUD=921`)
13. Test MAVLink link: run `mavlink_interface_node` alone, verify heartbeat
14. Test camera pipeline: run `csi_camera_node` + `apriltag_ros`, verify tag detection
15. Full stack test: `ros2 launch dbvf_autonomy mission_real.launch.py`
