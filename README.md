# ardu_ws — ArduPilot + Gazebo + ROS 2 workspace

A complete snapshot of the `ardu_ws` colcon workspace as of **2026-10-08**. It covers ArduPilot SITL, Gazebo Harmonic, ROS 2 Humble, the VFS world with the `iris` drone, and the `circle_detector` vision node.

This repo holds only the top-level workspace files. Every package under `src/` is its own git repo, pinned to an exact commit in [`workspace.repos`](workspace.repos). Some of those commits are work-in-progress snapshots and live on `share/2026-10-08` branches.

## Tested environment

| | |
|---|---|
| OS | Ubuntu 22.04 |
| ROS | ROS 2 Humble (`ros-humble-desktop`) |
| Gazebo | Harmonic (gz-sim 8) |

## 1. System dependencies

```bash
# ROS 2 Humble desktop: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
# Gazebo Harmonic:      https://gazebosim.org/docs/harmonic/install_ubuntu

sudo apt install python3-vcstool python3-colcon-common-extensions python3-rosdep default-jdk
sudo rosdep init 2>/dev/null; rosdep update

# ros_gz is built from source here and must target Harmonic
echo 'export GZ_VERSION=harmonic' >> ~/.bashrc && export GZ_VERSION=harmonic

pip install --user MAVProxy pymavlink
```

## 2. Clone the workspace

The paths below assume `~/Documents/ardu_ws`, which matches the launch commands.

```bash
mkdir -p ~/Documents && cd ~/Documents
git clone https://github.com/Flippigan/ardu_ws.git
cd ardu_ws
vcs import --recursive < workspace.repos
```

> `dbvf_autonomy` and `formation_msgs` are **private** repos. If you weren't given access, `vcs import` reports an error for those two and continues with the rest. Neither is needed for the sim + circle detector steps below.

### Fix the hardcoded mesh path

The world file loads the VFS field mesh by absolute path (`/home/finn/...`). If your username isn't `finn`, or you cloned somewhere other than `~/Documents/ardu_ws`, run this:

```bash
sed -i "s#/home/finn/Documents/ardu_ws#$PWD#g" src/ardupilot_gz/ardupilot_gz_gazebo/worlds/iris_runway.sdf
```

## 3. Toolchain prerequisites

```bash
# ArduPilot build prerequisites (SITL)
src/ardupilot/Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile

# Micro-XRCE-DDS-Gen (needed for ArduPilot's DDS / ROS 2 interface)
cd src/Micro-XRCE-DDS-Gen && ./gradlew assemble && cd ../..
echo "export PATH=\$PATH:$PWD/src/Micro-XRCE-DDS-Gen/scripts" >> ~/.bashrc
export PATH=$PATH:$PWD/src/Micro-XRCE-DDS-Gen/scripts
microxrceddsgen -help   # should print usage

# AprilTag C library 3.4.5 (needed by apriltag_ros)
git clone --branch v3.4.5 https://github.com/AprilRobotics/apriltag.git /tmp/apriltag
cmake -S /tmp/apriltag -B /tmp/apriltag/build -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_WRAPPER=OFF
sudo cmake --build /tmp/apriltag/build --target install
sudo ldconfig
```

## 4. Build

```bash
cd ~/Documents/ardu_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

If the machine runs out of memory during the build, use `colcon build --executor sequential` instead.

## 5. Run

### Terminal 1: Gazebo + ArduPilot SITL + RViz
```bash
cd ~/Documents/ardu_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true
```

### Terminal 2: MAVProxy connected to the running SITL
```bash
mavproxy.py --master udp:127.0.0.1:14550 --console --map
```
Then, inside MAVProxy:
```
module load gimbal
```

### Terminal 3: circle detector
```bash
cd ~/Documents/ardu_ws
colcon build --packages-select circle_detector
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run circle_detector circle_detector
```
`circle_detector` lives in `src/formation_control/circle_detector/`.

## Notes

- **Camera:** the world spawns `iris_with_hardmount_camera`, a fixed downward camera on `camera_link` with **no gimbal**. `iris_bridge.yaml` matches it. `module load gimbal` is harmless but has nothing to drive. The older `iris_with_gimbal` setup uses `pitch_link` instead. `switch_branch.sh` toggles between the vision and swarm configurations.
- `src/workspace_setup_notes.md` holds the original author's setup notes, including troubleshooting history.
- The original `dependencies.repos` (apriltag only) is kept for reference. `workspace.repos` supersedes it.
