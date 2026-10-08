CI CD Experience Report for Multi-Drone ROS 2 Workspace

Author: Finn Picotoli
Project: ardu ws Multi-Drone Formation Control System
Date: November 2024

Executive Summary

This report documents the Continuous Integration and Continuous Deployment practices I implemented in a complex multi-drone ROS 2 workspace. The project integrates five independent ArduPilot Software In The Loop instances with Gazebo simulation, requiring sophisticated build automation, branch management, and deployment configurations across multiple build systems.

Section One: Project Complexity Overview

Architecture Requiring CI CD

This workspace manages over sixty-eight ROS 2 packages across multiple repositories, each with distinct build systems. The ArduPilot Software In The Loop components, of which there are six instances, use the waf build system, which is Python-based, to compile the core flight controllers. The ROS 2 packages, totaling fifty-two packages, use the colcon and ament build system. The Gazebo plugins use CMake to build the ArduPilot Plugin. The DDS Code Generator uses Gradle to build Micro XRCE DDS Gen. Finally, the Snap packages use Snapcraft to build the micro ros agent.

The repository structure consists of a main workspace called ardu ws, which contains a source directory with multiple components. These include the main ArduPilot instance, five additional ArduPilot instances numbered one through five for multi-instance simulation on the swarm branch, the custom formation control ROS 2 package, custom message definitions in formation msgs, Gazebo integration in ardupilot gz, the ROS Gazebo bridge in ros gz, the DDS bridge agent in micro ros agent, and model format conversion utilities in sdformat urdf. The workspace also contains build directories with colcon build artifacts totaling approximately seven hundred seventy-six megabytes, install directories with installed packages totaling approximately seven hundred one megabytes, and a dot claude directory containing CI CD scripts and documentation.

Section Two: Custom Build Automation Scripts

Full Branch Switching Pipeline

I created a script called switch branch dot sh located in the dot claude directory. The purpose of this script is to provide automated environment switching between the swarm and vision development branches.

The script begins with a shebang line and sets the exit on error flag, which implements fail-fast pipeline behavior. This means if any command fails, the entire script stops immediately rather than continuing with potentially corrupted state.

One of the key features is configuration-driven branch mappings. I defined associative arrays that map repository names to their corresponding branch names. For the swarm configuration, the formation control repository maps to the swarm branch, ardupilot gazebo maps to swarm-ardupilot-gazebo, ardupilot gz maps to swarm-ardupilot-gz, micro ros agent maps to swarm-micro-ros-agent, and ros gz maps to swarm-ros-gz. In total, nine repositories are managed this way.

The script implements an eight-stage automation pipeline. Stage one handles ArduPilot instance management, which involves asset relocation between the workspace and storage. Stage two cleans all build artifacts, implementing cache invalidation. Stage three switches git branches across all nine repositories, coordinating version control. Stage four synchronizes git submodules for proper dependency management. Stage five runs the ArduPilot waf build for the embedded build system. Stage six runs the ROS 2 colcon build for package compilation. Stage seven sources the environment for runtime configuration. Stage eight performs verification and status checks for build validation.

The script also implements color-coded output for CI CD visibility. I created functions called print step, print success, print warning, and print error, each using different terminal color codes. Blue indicates a step in progress, green indicates success, yellow indicates warnings, and red indicates errors. This makes it easy to scan through build output and identify issues quickly.

For error handling, the script gracefully handles missing repositories. Before attempting to switch a branch, it checks if the repository path exists and if it contains a dot git directory. If the repository is not found or is not a git repository, it prints a warning and continues rather than failing the entire pipeline.

Quick Branch Switcher

I also created a lightweight version called switch branches only dot sh. This script's purpose is to perform branch switching without triggering a full rebuild, which is useful for code review or inspection scenarios.

This demonstrates my understanding of incremental CI principles. Not every operation requires a full rebuild. Sometimes you just need to check out different code to review it. The script loops through all repositories and switches their branches, then reminds the user that they may need to update git submodules and rebuild the workspace if needed.

Section Three: Multi-Build System Integration

Waf Build System for ArduPilot

ArduPilot uses the Python-based waf build system for embedded firmware compilation. The build process has three phases. The configuration phase runs waf configure with flags for the sitl board, disabling warnings as errors, and enabling DDS. The build phase runs waf copter to compile the ArduCopter binary. The clean phase runs waf clean to remove build artifacts.

The CI CD integration points for waf include build configuration stored in wscript files, DDS enable and disable as a build-time feature flag, and parallel instance builds for the swarm configuration where we need to build five separate ArduPilot instances.

CMake Build System for Gazebo Plugins

The Gazebo plugins use CMake for building. The process involves running cmake with the build type set to Release, then running make with parallel jobs equal to the number of processors. The products include libArduPilotPlugin dot so, which is the physics bridge, along with other Gazebo system plugins totaling approximately twenty-six megabytes.

Colcon Build System for ROS 2

ROS 2 packages use colcon for building. The standard build command is colcon build with symlink install. For parallel builds with release optimization, you can add parallel workers equal to the number of processors and cmake args for release build type. For selective package builds, you can use packages select to build only specific packages like formation control and formation msgs.

The package configuration is defined in package dot xml files using format three. These files specify the package name, build type such as ament python, runtime dependencies like rclpy and geometry msgs, and test dependencies for CI integration including ament copyright, ament flake eight, ament pep two fifty-seven, and python three pytest.

Gradle Build System for DDS Generator

The Micro XRCE DDS Generator uses Gradle for building. The assemble task builds the project, the test task runs tests with a version specification, and the clean task removes build artifacts.

Section Four: Pre-commit Hooks and Code Quality

ArduPilot Pre-commit Configuration

The ArduPilot repository includes a pre-commit configuration file at dot pre-commit-config dot yaml. This configuration uses version four point four point zero of the pre-commit hooks repository and includes several quality checks.

The mixed line ending hook enforces LF line endings and applies to Python, C, C plus plus, and shell files. The check added large files hook prevents accidentally committing large binary files. The check executables have shebangs hook ensures all executable scripts have proper shebang lines. The check merge conflict hook detects merge conflict markers that were accidentally left in files. The check xml and check yaml hooks validate XML and YAML syntax.

For Python formatting, the configuration uses Black version twenty-three point seven point zero. The Black hook applies specifically to files in the AP DDS library directory and the Tools ros2 directory.

The quality gates enforced by this configuration include line ending consistency with LF enforcement, large file prevention, shebang validation for executables, merge conflict detection, XML and YAML syntax validation, and Python code formatting with Black.

Gazebo Integration Pre-commit

The ardupilot gz repository has its own pre-commit configuration using version four point two point zero of the hooks. It includes trailing whitespace removal, end of file fixing, and mixed line ending checks. Additionally, it uses xmllint from the LSST repository for formatting and validating SDF and URDF model files. Black version twenty-three point one point zero handles Python formatting.

Section Five: GitHub Actions Workflows

Micro XRCE DDS Gen Continuous Integration

The Micro XRCE DDS Gen repository includes a GitHub Actions workflow at dot github slash workflows slash ci dot yaml. The workflow is named Continuous Integration and triggers on workflow dispatch for manual triggers, push events to the master branch, and pull request events to all branches.

The workflow defines a job called ubuntu build test that runs on ubuntu latest. The steps include checking out the code with actions checkout version three with recursive submodules enabled. It then uses a custom composite action to install apt packages. The workflow uses the lukka get cmake action to ensure CMake version three point sixteen point three, which is the minimum supported version. The build step changes to the source directory and runs gradlew assemble. The test step runs gradlew test with a branch parameter set to version two point three point zero.

The CI CD patterns demonstrated include reusable composite actions, CMake version pinning, submodule recursive checkout, and parameterized testing.

Sdformat URDF Matrix CI

The sdformat urdf repository includes a workflow that demonstrates matrix builds. The workflow triggers on pull requests and defines a job that runs on Ubuntu twenty-two point zero four with a strategy matrix. The matrix includes ROS distributions humble and rolling, and Gazebo version fortress.

The workflow uses ros tooling setup ros version zero point seven to set up the required ROS distributions, then uses ros tooling action ros ci version zero point two to build and test with the target ROS 2 distribution from the matrix.

The CI CD patterns demonstrated include matrix builds across multiple ROS distributions, ROS-specific CI tooling integration, and environment variable injection.

ROS GZ Build and Test Script

The ros gz repository includes a shell script for building and testing. The script sets exit on error and verbose mode. It configures environment variables for the colcon workspace and sets the Debian frontend to noninteractive.

The script includes conditional Gazebo version setup. If the GZ VERSION variable is set to harmonic, it adds the Gazebo stable repository, sets the GZ DEPS variable to libgz sim eight dev, and configures rosdep arguments to skip the sdformat urdf key.

For dependency resolution, the script initializes and updates rosdep, then installs dependencies from the current path with the specified ROS distribution. The build phase sources the ROS setup script, creates the workspace directory, copies the repository, and runs colcon build with console direct event handlers. The test phase runs colcon test and colcon test result to report results.

The CI CD patterns demonstrated include conditional dependency installation, rosdep integration for dependency resolution, and test result reporting.

Section Six: Package Management and Deployment

Snapcraft Deployment Configuration

The micro ros agent repository includes a Snapcraft configuration for deploying the agent as a Snap package. The snap is named micro-ros-agent, based on core twenty, with version derived from git, stable grade, and strict confinement.

The configuration specifies build architectures including amd64, arm64, armhf, and ppc64el, enabling cross-platform deployment.

Package repositories are configured to include the ROS 2 apt repository with the appropriate GPG key for secure package installation.

The parts section defines how to build the snap. The uros agent part uses the colcon plugin, takes the source from the current directory, and passes cmake arguments to enable the superbuild. The override pull section derives the version from git describe, extracting tags and formatting them appropriately. If the version contains plus git, indicating a development build, the grade is set to devel; otherwise, it is set to stable.

The apps section defines the micro ros agent application with its command path, environment configuration for Fast DDS, and plugs for network, network bind, and serial port access. A daemon configuration enables running the agent as a system service.

The deployment features demonstrated include multi-architecture builds for four platforms, automatic version derivation from git tags, grade promotion from development to stable based on release tags, daemon service configuration, and security confinement with plug permissions.

Python Package Configuration

The formation control package includes a setup dot py file using setuptools. The setup function specifies the package name, version, packages list, and data files for ament index registration.

The entry points section defines console scripts that become executable commands after installation. These include formation commander, safety monitor, emergency controller, drone interface, formation visualizer, swarm takeoff commander, and collision avoidance controller. Each entry point maps to a main function in the corresponding module.

Section Seven: Environment Configuration

ROS Gazebo Bridge Configuration

The bridge configuration file at ardupilot gz bringup config iris bridge dot yaml defines topic bridging between ROS and Gazebo. Each entry specifies the ROS topic name, the Gazebo topic name, the ROS type name, the Gazebo type name, and the direction of data flow.

For example, the clock topic bridges from Gazebo to ROS, mapping the Gazebo clock message type to the ROS rosgraph msgs Clock type. The odometry topic bridges the Gazebo odometry model to the ROS nav msgs Odometry type. The IMU topic bridges sensor data from the Gazebo IMU sensor to the ROS sensor msgs IMU type. All these examples use the GZ TO ROS direction, meaning data flows from Gazebo simulation to ROS nodes.

Multi-Drone Launch Configuration

The multi-drone launch file at ardupilot gz bringup launch iris multi uav launch dot py orchestrates the startup of all five drones. This demonstrates infrastructure as code principles.

The launch description implements staggered launch with timing dependencies. The first SITL instance starts immediately, the second starts after seven seconds, the third after fourteen seconds, the fourth after twenty-one seconds, and the fifth after twenty-eight seconds. The Gazebo simulation server and GUI start after thirty-five seconds, once all SITL instances are initialized.

The configuration also demonstrates event-based process orchestration. Register Event Handler is used with On Process Start to trigger dependent processes. For example, when the first bridge process starts, it triggers the first TF relay process to start.

The infrastructure as code principles demonstrated include declarative process orchestration, parameterized instance configuration, event-driven dependency management, and environment variable propagation.

Section Eight: Documentation and Operational Runbooks

Workspace Clean and Rebuild Guide

I created a comprehensive guide at dot claude WORKSPACE CLEAN REBUILD GUIDE dot md, totaling six hundred forty-five lines. This document provides several key sections.

The build artifact documentation section includes size analysis showing approximately three point nine gigabytes of artifacts, directory structure mapping explaining what each directory contains, and cleanup procedures for safely removing build outputs.

The branch switching workflow section provides a pre-switch checklist, submodule synchronization instructions, and post-switch verification steps.

The troubleshooting runbook section covers common issues and their resolutions. Package not found errors are typically resolved by removing and rebuilding the workspace. DDS client configuration issues require rebuilding ArduPilot with DDS enabled. Gazebo plugin loading failures require rebuilding the plugins and setting the correct plugin path. Python import errors require cleaning the Python cache and rebuilding message packages. Git submodule state recovery involves force updating submodules. Disk space management involves cleaning accumulated build artifacts.

The quick reference commands section provides essential commands for cleaning and building. The essential clean command removes the build, install, log, and ArduPilot build directories. The essential build sequence configures and builds ArduPilot, runs colcon build, and sources the setup script.

Launch Workflow Documentation

I created operational runbooks at dot claude Skill Documentation Swarm Launch Workflow dot md. This documents the deployment sequence including environment sourcing, multi-UAV launch, readiness monitoring for EKF initialization, formation control activation, and position verification via topic monitoring.

Summary of CI CD Competencies Demonstrated

This project demonstrates competencies across multiple CI CD categories.

For build automation, I implemented custom shell scripts with error handling, colored output, and multi-stage pipelines.

For multi-build system integration, I coordinated waf, CMake, colcon, and Gradle build systems in a unified workflow.

For version control, I managed multiple repositories with branch management and submodule synchronization.

For code quality, I configured pre-commit hooks including Black for Python formatting, xmllint for XML validation, and merge conflict detection.

For continuous integration, I worked with GitHub Actions implementing matrix builds and ROS CI tooling.

For deployment, I configured Snapcraft for multi-architecture packaging and daemon services.

For configuration management, I created YAML bridge configurations and launch file parameterization.

For documentation, I created operational runbooks and troubleshooting guides.

For infrastructure as code, I implemented declarative launch descriptions and event-driven orchestration.

Appendix: File Locations

The full branch switcher script is located at dot claude slash switch branch dot sh. The quick branch switcher is at dot claude slash switch branches only dot sh. The clean and rebuild guide is at dot claude slash WORKSPACE CLEAN REBUILD GUIDE dot md. The launch workflow documentation is at dot claude slash Skill Documentation slash Swarm Launch Workflow dot md.

The ArduPilot pre-commit configuration is at src slash ardupilot slash dot pre-commit-config dot yaml. The Gazebo pre-commit configuration is at src slash ardupilot gz slash dot pre-commit-config dot yaml.

The DDS CI workflow is at src slash Micro-XRCE-DDS-Gen slash dot github slash workflows slash ci dot yaml. The sdformat CI workflow is at src slash sdformat urdf slash dot github slash workflows slash ci dot yaml. The ros gz CI script is at src slash ros gz slash dot github slash workflows slash build-and-test dot sh.

The Snap deployment configuration is at src slash micro ros agent slash snap slash snapcraft dot yaml. The formation package setup is at src slash formation control slash setup dot py. The bridge configuration is at src slash ardupilot gz slash ardupilot gz bringup slash config slash iris bridge dot yaml. The multi-UAV launch file is at src slash ardupilot gz slash ardupilot gz bringup slash launch slash iris multi uav launch dot py.

End of Report
