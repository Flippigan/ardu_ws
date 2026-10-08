# Swarm Flocking Testing with Gazebo Harmonic - Aerostack2

## Prerequisites

### 1. Gazebo Harmonic Installation
Since you already have Gazebo Harmonic installed, ensure you have the ROS 2 bridge:
```bash
sudo apt install ros-humble-ros-gzharmonic
```

### 2. Aerostack2 Workspace Setup
```bash
# Build the workspace
as2 build

# Source the workspace
source install/setup.bash
```

### 3. Verify Gazebo Harmonic Integration
```bash
# Check Gazebo version
gz sim --version

# Verify ROS-Gazebo bridge is available
ros2 pkg list | grep ros_gzharmonic
```

## Multi-Drone Configuration for Swarm Testing

### Create Swarm Configuration File

Create a multi-drone configuration file for testing swarm formation. Save as `swarm_test_config.json`:

```json
{
    "world_name": "empty",
    "drones": [
        {
            "model_type": "quadrotor_base",
            "model_name": "drone_0",
            "xyz": [0.0, 0.0, 0.2],
            "rpy": [0, 0, 0],
            "flight_time": 300
        },
        {
            "model_type": "quadrotor_base", 
            "model_name": "drone_1",
            "xyz": [2.0, 0.0, 0.2],
            "rpy": [0, 0, 0],
            "flight_time": 300
        },
        {
            "model_type": "quadrotor_base",
            "model_name": "drone_2", 
            "xyz": [4.0, 0.0, 0.2],
            "rpy": [0, 0, 0],
            "flight_time": 300
        },
        {
            "model_type": "quadrotor_base",
            "model_name": "drone_3",
            "xyz": [0.0, 2.0, 0.2],
            "rpy": [0, 0, 0],
            "flight_time": 300
        }
    ]
}
```

## Complete Launch Sequence for Gazebo Harmonic Testing

### Step 1: Launch Gazebo Simulation
```bash
# Terminal 1: Launch Gazebo Harmonic with multi-drone setup
ros2 launch as2_gazebo_assets launch_simulation.py simulation_config_file:=/path/to/swarm_test_config.json
```

This will:
- Start Gazebo Harmonic
- Spawn 4 drones in the simulation
- Create ROS-Gazebo bridges for each drone

### Step 2: Launch Platform Interfaces (Per Drone)
For each drone, launch the platform interface:

```bash
# Terminal 2: Drone 0 Platform
ros2 launch as2_platform_gazebo platform_gazebo_launch.py namespace:=drone_0

# Terminal 3: Drone 1 Platform  
ros2 launch as2_platform_gazebo platform_gazebo_launch.py namespace:=drone_1

# Terminal 4: Drone 2 Platform
ros2 launch as2_platform_gazebo platform_gazebo_launch.py namespace:=drone_2

# Terminal 5: Drone 3 Platform
ros2 launch as2_platform_gazebo platform_gazebo_launch.py namespace:=drone_3
```

### Step 3: Launch State Estimators (Per Drone)
For Gazebo simulation, use ground truth state estimator:

```bash
# Terminal 6-9: State Estimators
ros2 launch as2_state_estimator ground_truth_state_estimator.launch.py namespace:=drone_0 use_sim_time:=true
ros2 launch as2_state_estimator ground_truth_state_estimator.launch.py namespace:=drone_1 use_sim_time:=true
ros2 launch as2_state_estimator ground_truth_state_estimator.launch.py namespace:=drone_2 use_sim_time:=true
ros2 launch as2_state_estimator ground_truth_state_estimator.launch.py namespace:=drone_3 use_sim_time:=true
```

### Step 4: Launch Motion Controllers (Per Drone)
```bash
# Terminal 10-13: Motion Controllers
ros2 launch as2_motion_controller controller_launch.py namespace:=drone_0 use_sim_time:=true
ros2 launch as2_motion_controller controller_launch.py namespace:=drone_1 use_sim_time:=true
ros2 launch as2_motion_controller controller_launch.py namespace:=drone_2 use_sim_time:=true
ros2 launch as2_motion_controller controller_launch.py namespace:=drone_3 use_sim_time:=true
```

### Step 5: Launch Motion Behaviors (Per Drone)
```bash
# Terminal 14-17: Motion Behaviors (includes FollowReference)
ros2 launch as2_behaviors_motion motion_behaviors_launch.py namespace:=drone_0 use_sim_time:=true
ros2 launch as2_behaviors_motion motion_behaviors_launch.py namespace:=drone_1 use_sim_time:=true
ros2 launch as2_behaviors_motion motion_behaviors_launch.py namespace:=drone_2 use_sim_time:=true
ros2 launch as2_behaviors_motion motion_behaviors_launch.py namespace:=drone_3 use_sim_time:=true
```

### Step 6: Launch Swarm Flocking Behavior
```bash
# Terminal 18: Swarm Coordinator
ros2 launch as2_behaviors_swarm_flocking swarm_flocking_behavior.launch.py use_sim_time:=true
```

### Step 7: Optional - Launch Visualization
```bash
# Terminal 19: RViz Visualization
rviz2 -d /path/to/aerostack2/config/swarm_visualization.rviz

# Or launch swarm-specific visualization
ros2 launch as2_visualization swarm_viz.launch.py
```

## Pre-Flight Setup and Testing

### 1. Arm and Takeoff Individual Drones
Before starting swarm formation, ensure all drones are airborne:

```bash
# Arm all drones
ros2 service call /drone_0/platform/set_arming_state as2_msgs/srv/SetArmingState "{arming_state: true}"
ros2 service call /drone_1/platform/set_arming_state as2_msgs/srv/SetArmingState "{arming_state: true}"
ros2 service call /drone_2/platform/set_arming_state as2_msgs/srv/SetArmingState "{arming_state: true}"
ros2 service call /drone_3/platform/set_arming_state as2_msgs/srv/SetArmingState "{arming_state: true}"

# Set offboard mode
ros2 service call /drone_0/platform/set_offboard_mode as2_msgs/srv/SetOffboardMode "{offboard_mode: true}"
ros2 service call /drone_1/platform/set_offboard_mode as2_msgs/srv/SetOffboardMode "{offboard_mode: true}"
ros2 service call /drone_2/platform/set_offboard_mode as2_msgs/srv/SetOffboardMode "{offboard_mode: true}"
ros2 service call /drone_3/platform/set_offboard_mode as2_msgs/srv/SetOffboardMode "{offboard_mode: true}"

# Takeoff to 5m altitude
ros2 action send_goal /drone_0/TakeoffBehavior as2_msgs/action/Takeoff "{takeoff_height: 5.0, takeoff_speed: 1.0}"
ros2 action send_goal /drone_1/TakeoffBehavior as2_msgs/action/Takeoff "{takeoff_height: 5.0, takeoff_speed: 1.0}"
ros2 action send_goal /drone_2/TakeoffBehavior as2_msgs/action/Takeoff "{takeoff_height: 5.0, takeoff_speed: 1.0}"
ros2 action send_goal /drone_3/TakeoffBehavior as2_msgs/action/Takeoff "{takeoff_height: 5.0, takeoff_speed: 1.0}"
```

### 2. Test Swarm Formation

#### Formation 1: Square Formation
```bash
ros2 action send_goal /Swarm/SwarmFlockingBehavior as2_msgs/action/SwarmFlocking '{
  virtual_centroid: {
    header: {frame_id: "earth"},
    pose: {
      position: {x: 5.0, y: 5.0, z: 5.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  swarm_formation: [
    {id: "drone_0", pose: {position: {x: -1.0, y: -1.0, z: 0.0}}},
    {id: "drone_1", pose: {position: {x: 1.0, y: -1.0, z: 0.0}}},
    {id: "drone_2", pose: {position: {x: 1.0, y: 1.0, z: 0.0}}},
    {id: "drone_3", pose: {position: {x: -1.0, y: 1.0, z: 0.0}}}
  ],
  drones_namespace: ["drone_0", "drone_1", "drone_2", "drone_3"]
}'
```

#### Formation 2: Line Formation
```bash
ros2 action send_goal /Swarm/SwarmFlockingBehavior as2_msgs/action/SwarmFlocking '{
  virtual_centroid: {
    header: {frame_id: "earth"},
    pose: {
      position: {x: 10.0, y: 5.0, z: 5.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  swarm_formation: [
    {id: "drone_0", pose: {position: {x: -3.0, y: 0.0, z: 0.0}}},
    {id: "drone_1", pose: {position: {x: -1.0, y: 0.0, z: 0.0}}},
    {id: "drone_2", pose: {position: {x: 1.0, y: 0.0, z: 0.0}}},
    {id: "drone_3", pose: {position: {x: 3.0, y: 0.0, z: 0.0}}}
  ],
  drones_namespace: ["drone_0", "drone_1", "drone_2", "drone_3"]
}'
```

## Monitoring and Debugging

### Key Topics to Monitor
```bash
# Monitor swarm action status
ros2 topic echo /Swarm/SwarmFlockingBehavior/_action/status

# Monitor individual drone poses
ros2 topic echo /drone_0/self_localization/pose
ros2 topic echo /drone_1/self_localization/pose

# Monitor TF transforms
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo earth Swarm
```

### Check System Status
```bash
# Verify all nodes are running
ros2 node list | grep -E "(drone_[0-3]|Swarm)"

# Check action servers
ros2 action list | grep -E "(FollowReference|SwarmFlocking)"

# Monitor Gazebo topics
gz topic -l | grep drone
```

## Gazebo Harmonic Specific Features

### Physics Configuration
Gazebo Harmonic provides improved physics simulation for multi-drone scenarios:
- Better collision detection between drones
- More accurate aerodynamic modeling
- Improved sensor simulation

### Performance Optimization
For better performance with multiple drones:
- Use real-time factor adjustment: Set `<real_time_factor>` in world SDF
- Monitor system resources during simulation
- Consider reducing sensor update rates for large swarms

### Visual Debugging
- Use Gazebo Harmonic GUI to visualize formation changes
- Enable TF visualization in RViz for coordinate frame debugging
- Monitor drone trajectories using RViz Path display

## Troubleshooting Gazebo Harmonic Issues

### Common Problems and Solutions

1. **Simulation time synchronization**
   ```bash
   # Ensure all nodes use simulation time
   export ROS_DOMAIN_ID=0
   ros2 param set /use_sim_time true
   ```

2. **ROS-Gazebo bridge issues**
   ```bash
   # Restart the simulation if topics are not available
   # Check bridge status
   ros2 topic list | grep /world/
   ```

3. **Performance issues with multiple drones**
   - Reduce real-time factor in Gazebo
   - Decrease sensor update rates
   - Use simplified drone models for large swarms

## Success Metrics

The swarm formation test is successful when:
1. All drones maintain their relative positions within 0.3m tolerance
2. Formation changes smoothly when new commands are sent
3. Drones coordinate movement as a single unit
4. No collisions occur during formation changes

This setup provides a complete testing environment for evaluating Aerostack2's swarm flocking capabilities using Gazebo Harmonic simulation.

---
*Guide updated for Gazebo Harmonic and Aerostack2 v1.1.3*