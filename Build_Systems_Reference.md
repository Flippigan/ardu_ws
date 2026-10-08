# Build Systems Reference Guide

This document specifies the build system used by each package and component in the ardu_ws workspace.

---

## Build System Overview

| Build System | Purpose | Build Command |
|--------------|---------|---------------|
| **waf** | ArduPilot firmware compilation | `./waf configure && ./waf copter` |
| **colcon** | ROS2 workspace orchestrator | `colcon build` |
| **ament_cmake** | ROS2 C++ packages | Via colcon |
| **ament_python** | ROS2 Python packages | Via colcon |
| **CMake** | Standalone C++ projects | `cmake .. && make` |
| **Gradle** | Java-based code generators | `./gradlew assemble` |
| **Snapcraft** | Snap package deployment | `snapcraft` |

---

## ArduPilot Components (waf)

These components use the **waf** build system (Python-based).

| Package | Location | Build Command |
|---------|----------|---------------|
| ardupilot (main) | `src/ardupilot/` | `./waf configure --board sitl && ./waf copter` |
| ardupilot1 | `src/ardupilot1/` | `./waf configure --board sitl --enable-dds && ./waf copter` |
| ardupilot2 | `src/ardupilot2/` | `./waf configure --board sitl --enable-dds && ./waf copter` |
| ardupilot3 | `src/ardupilot3/` | `./waf configure --board sitl --enable-dds && ./waf copter` |
| ardupilot4 | `src/ardupilot4/` | `./waf configure --board sitl --enable-dds && ./waf copter` |
| ardupilot5 | `src/ardupilot5/` | `./waf configure --board sitl --enable-dds && ./waf copter` |

**Notes:**
- Each instance produces `build/sitl/bin/arducopter` executable
- `--enable-dds` flag required for ROS2 DDS communication
- Clean with `./waf clean` or `rm -rf build/`

---

## ROS2 Packages via Colcon (ament_cmake)

These packages use **ament_cmake** and are built via `colcon build`.

### Custom Project Packages

| Package | Location | Description |
|---------|----------|-------------|
| formation_msgs | `src/formation_msgs/` | Custom message definitions |

### ArduPilot ROS2 Integration

| Package | Location | Description |
|---------|----------|-------------|
| ardupilot_msgs | `src/ardupilot/Tools/ros2/ardupilot_msgs/` | ArduPilot message definitions |
| ardupilot_sitl | `src/ardupilot/Tools/ros2/ardupilot_sitl/` | SITL launch files |

### Gazebo Integration (ardupilot_gz)

| Package | Location | Description |
|---------|----------|-------------|
| ardupilot_gz_bringup | `src/ardupilot_gz/ardupilot_gz_bringup/` | Launch files for simulation |
| ardupilot_gz_description | `src/ardupilot_gz/ardupilot_gz_description/` | Robot descriptions |
| ardupilot_gz_application | `src/ardupilot_gz/ardupilot_gz_application/` | Application nodes |
| ardupilot_gz_gazebo | `src/ardupilot_gz/ardupilot_gz_gazebo/` | Gazebo world files |

### ArduPilot Gazebo Plugin

| Package | Location | Description |
|---------|----------|-------------|
| ardupilot_gazebo | `src/ardupilot_gazebo/` | ArduPilotPlugin for Gazebo |

### ROS-Gazebo Bridge (ros_gz)

| Package | Location | Description |
|---------|----------|-------------|
| ros_gz | `src/ros_gz/ros_gz/` | Metapackage |
| ros_gz_bridge | `src/ros_gz/ros_gz_bridge/` | Topic/service bridge |
| ros_gz_image | `src/ros_gz/ros_gz_image/` | Image transport bridge |
| ros_gz_interfaces | `src/ros_gz/ros_gz_interfaces/` | Interface definitions |
| ros_gz_sim | `src/ros_gz/ros_gz_sim/` | Simulation integration |
| ros_gz_sim_demos | `src/ros_gz/ros_gz_sim_demos/` | Demo launch files |
| test_ros_gz_bridge | `src/ros_gz/test_ros_gz_bridge/` | Bridge tests |

### Legacy Ignition Packages (ros_gz)

| Package | Location | Description |
|---------|----------|-------------|
| ros_ign | `src/ros_gz/ros_ign/` | Legacy metapackage |
| ros_ign_bridge | `src/ros_gz/ros_ign_bridge/` | Legacy bridge |
| ros_ign_gazebo | `src/ros_gz/ros_ign_gazebo/` | Legacy sim integration |
| ros_ign_gazebo_demos | `src/ros_gz/ros_ign_gazebo_demos/` | Legacy demos |
| ros_ign_image | `src/ros_gz/ros_ign_image/` | Legacy image bridge |
| ros_ign_interfaces | `src/ros_gz/ros_ign_interfaces/` | Legacy interfaces |

### Micro-ROS Agent

| Package | Location | Description |
|---------|----------|-------------|
| micro_ros_agent | `src/micro_ros_agent/micro_ros_agent/` | DDS-XRCE bridge agent |

### SDFormat URDF

| Package | Location | Description |
|---------|----------|-------------|
| sdformat_urdf | `src/sdformat_urdf/sdformat_urdf/` | SDF to URDF conversion |

### SITL Models

| Package | Location | Description |
|---------|----------|-------------|
| ardupilot_sitl_models | `src/ardupilot_sitl_models/Gazebo/` | Gazebo model assets |

---

## ROS2 Packages via Colcon (ament_python)

These packages use **ament_python** and are built via `colcon build`.

| Package | Location | Description |
|---------|----------|-------------|
| formation_control | `src/formation_control/` | Formation control nodes |
| ardupilot_dds_tests | `src/ardupilot/Tools/ros2/ardupilot_dds_tests/` | DDS integration tests |

---

## Standalone CMake Projects

These use **CMake** directly (not via colcon).

| Package | Location | Build Command |
|---------|----------|---------------|
| sdformat_test_files | `src/sdformat_urdf/sdformat_test_files/` | `cmake .. && make` |

---

## Gradle Projects

These use **Gradle** for Java-based builds.

| Package | Location | Build Command |
|---------|----------|---------------|
| Micro-XRCE-DDS-Gen | `src/Micro-XRCE-DDS-Gen/` | `./gradlew assemble` |

**Notes:**
- Generates DDS code from IDL definitions
- Test with `./gradlew test`
- Clean with `./gradlew clean`

---

## Snapcraft Deployment

These use **Snapcraft** for Snap package creation.

| Package | Location | Config File |
|---------|----------|-------------|
| micro-ros-agent | `src/micro_ros_agent/` | `snap/snapcraft.yaml` |

**Build Command:** `snapcraft`

**Architectures:** amd64, arm64, armhf, ppc64el

---

## Other Components (No Build Required)

| Component | Location | Description |
|-----------|----------|-------------|
| swarm-skybrush-server | `src/swarm-skybrush-server/` | Configuration/scripts only |

---

## Build Order Dependencies

For a clean rebuild, follow this order:

```
1. ArduPilot SITL (waf)
   └── Required for: ardupilot_sitl launch files

2. Micro-XRCE-DDS-Gen (Gradle)
   └── Required for: DDS code generation

3. ROS2 Packages (colcon)
   └── Builds all ament_cmake and ament_python packages
   └── Order handled automatically by colcon

4. Snap Packages (Snapcraft) [Optional]
   └── For deployment only
```

---

## Quick Reference Commands

### Build Everything

```bash
# 1. Build ArduPilot instances
for i in 1 2 3 4 5; do
  cd src/ardupilot$i
  ./waf configure --board sitl --enable-dds
  ./waf copter
  cd ../..
done

# 2. Build ROS2 workspace
colcon build --symlink-install

# 3. Source environment
source install/setup.bash
```

### Build Specific Systems

```bash
# ArduPilot only
cd src/ardupilot1 && ./waf copter

# Single ROS2 package
colcon build --packages-select formation_control

# DDS Generator only
cd src/Micro-XRCE-DDS-Gen && ./gradlew assemble
```

### Clean Specific Systems

```bash
# ArduPilot
cd src/ardupilot1 && ./waf clean

# ROS2 workspace
rm -rf build/ install/ log/

# Gradle
cd src/Micro-XRCE-DDS-Gen && ./gradlew clean
```

---

## Summary Table

| Build System | Package Count | Primary Use |
|--------------|---------------|-------------|
| waf | 6 | ArduPilot firmware |
| ament_cmake | 25+ | ROS2 C++ packages |
| ament_python | 7+ | ROS2 Python packages |
| CMake | 1 | Test files |
| Gradle | 1 | DDS code generator |
| Snapcraft | 1 | Deployment |
