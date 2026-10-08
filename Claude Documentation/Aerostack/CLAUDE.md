# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Development Commands

### Build Commands
Use the custom AS2 CLI tool for all build operations:
- `as2 build` - Build all packages
- `as2 build <package_name>` - Build up to a specific package
- `as2 build -d` - Build in debug mode
- `as2 build -v` - Build with verbose output

### Test Commands
- `as2 test` - Run all tests
- `as2 test <package_name>` - Run tests for a specific package
- `as2 test -v` - Run tests with verbose output

### Other Utility Commands
- `as2 list` - List all packages in the workspace
- `as2 clean` - Clean the workspace
- `as2 project` - List and install aerostack2 projects

Note: The AS2 CLI requires `$AEROSTACK2_PATH` and `$AEROSTACK2_WORKSPACE` environment variables to be set.

## Architecture Overview

Aerostack2 is a ROS 2 framework for autonomous multi-aerial robot systems with complete modularity and platform independence. The architecture follows a hierarchical structure:

### Core Components
- **as2_core**: Foundation package containing base classes (`Node`, `AerialPlatform`, `Sensor`) and utilities (frame transforms, GPS, TF2, YAML parsing)
- **as2_msgs**: Custom ROS 2 messages, services, and actions for the framework
- **as2_behavior**: Base behavior system using ROS 2 actions for autonomous behaviors
- **as2_python_api**: High-level Python interface for drone control and mission execution

### Main Subsystems

**Motion Stack**:
- **as2_motion_controller**: Plugin-based controller system (PID, differential flatness)
- **as2_motion_reference_handlers**: Motion reference generators for different control modes
- **as2_state_estimator**: Plugin-based state estimation (ground truth, mocap, odometry fusion)

**Behavior System**:
- **as2_behaviors**: Collection of autonomous behaviors (takeoff, land, go_to, follow_path, etc.)
- **as2_behavior_tree**: BehaviorTree.CPP integration for complex mission execution

**Platform Integration**:
- **as2_aerial_platforms**: Platform-specific interfaces (Gazebo, multirotor simulator)
- **as2_hardware_drivers**: Hardware interfaces (RealSense, USB cameras)

**Utilities**:
- **as2_user_interfaces**: Visualization and teleoperation tools
- **as2_simulation_assets**: Gazebo models and simulation environments

### Plugin Architecture
The framework uses extensive plugin systems:
- Motion controllers via `pluginlib`
- State estimators via `pluginlib`
- Behavior implementations via inheritance
- Platform drivers via inheritance from `AerialPlatform`

### Package Structure
Each package follows ROS 2 conventions:
- `package.xml` with format 3
- CMake build system with `ament_cmake`
- Standard directories: `src/`, `include/`, `launch/`, `config/`, `tests/`
- Plugin packages include `plugins.xml` for pluginlib registration

### Testing
All packages include comprehensive testing:
- Unit tests using `ament_cmake_gtest`
- Linting with `ament_lint_common` and `ament_lint_auto`
- Integration tests for behaviors and platform interfaces

### Configuration
- YAML configuration files in `config/` directories
- Launch files use Python-based ROS 2 launch system
- Parameters loaded from config files with validation