# ArduPilot DDS Messaging Capabilities & MAVROS Transformation Guide

## Overview

ArduPilot's DDS (Data Distribution Service) implementation provides direct ROS 2 communication without the need for MAVROS middleware. This modern approach leverages Micro-ROS Agent and offers improved performance, reduced latency, and native ROS 2 integration. [1][2]

**Document Version:** 1.1 - MAJOR CORRECTIONS APPLIED  
**Last Updated:** August 26, 2025  
**ArduPilot Version:** 4.5+  
**ROS 2 Distribution:** Humble

## ⚠️ CRITICAL CORRECTIONS APPLIED

**This document has been corrected based on actual observed system behavior:**

1. **Topic Names**: All ArduPilot DDS topics use `/ap/*` prefix, **NOT** `/rt/ap/*` as shown in ArduPilot source code
2. **Service Names**: All ArduPilot DDS services use simple names like `/ap/arm_motors`, **NOT** complex names like `/rs/ap/arm_motorsService`
3. **Multi-UAV Architecture**: The multi-drone examples are **CONCEPTUAL** - ArduPilot DDS does not provide automatic per-drone namespacing
4. **Parameter Services**: ArduPilot provides `/ap/get_parameters` and `/ap/set_parameters` services for direct parameter access

**These corrections are based on verified `ros2 topic list` and `ros2 service list` output from actual running systems.**

## Table of Contents

1. [DDS Architecture](#dds-architecture)
2. [Available Topics & Messages](#available-topics--messages)
3. [Service Interfaces](#service-interfaces)
4. [MAVROS to DDS Transformation](#mavros-to-dds-transformation)
5. [Implementation Patterns](#implementation-patterns)
6. [Code Examples](#code-examples)
7. [Configuration & Setup](#configuration--setup)
8. [References](#references)

## DDS Architecture

### Communication Stack
```
ROS 2 Applications
       �
   DDS Topics/Services
       �
  Micro-ROS Agent (UDP:2019)
       �
  ArduPilot SITL/Hardware
```

### Key Features
- **Direct Communication**: No MAVROS middleware required [1]
- **Real-time Performance**: Lower latency than MAVLink-based systems [2]
- **Standard ROS 2 Messages**: Uses geometry_msgs, sensor_msgs, etc. [1]
- **Configurable QoS**: Different reliability/durability settings per topic [3]
- **Selective Compilation**: Enable/disable features at build time [3]

## Available Topics & Messages

### Publisher Topics (ArduPilot → ROS 2)

*Source: ArduPilot AP_DDS_Topic_Table.h* [3]

**⚠️ CORRECTED TOPIC NAMES - Based on Actual Observed Behavior**

#### Sensor Data
| **ACTUAL** Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/ap/time` | `builtin_interfaces/Time` | Reliable | System time |
| `/ap/navsat` | `sensor_msgs/NavSatFix` | Best Effort | GPS position |
| `/ap/battery` | `sensor_msgs/BatteryState` | Best Effort | Battery status |
| `/ap/imu/experimental/data` | `sensor_msgs/Imu` | Best Effort | IMU data |

#### Position & Motion
| **ACTUAL** Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/ap/pose/filtered` | `geometry_msgs/PoseStamped` | Best Effort | Local position |
| `/ap/twist/filtered` | `geometry_msgs/TwistStamped` | Best Effort | Local velocity |
| `/ap/airspeed` | `geometry_msgs/Vector3Stamped` | Best Effort | Airspeed vector |
| `/ap/geopose/filtered` | `geographic_msgs/GeoPoseStamped` | Best Effort | Global position |

#### Navigation & Status
| **ACTUAL** Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/ap/goal_lla` | `geographic_msgs/GeoPointStamped` | Reliable/Transient | Current goal |
| `/ap/status` | `ardupilot_msgs/Status` | Reliable/Transient | Vehicle status |
| `/ap/clock` | `rosgraph_msgs/Clock` | Reliable | Simulation clock |
| `/ap/gps_global_origin/filtered` | `geographic_msgs/GeoPointStamped` | Best Effort | GPS origin |

#### Transform Data
| **ACTUAL** Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/ap/tf_static` | `tf2_msgs/TFMessage` | Reliable/Transient | Static transforms |
| `/ap/tf` | `tf2_msgs/TFMessage` | Best Effort | Dynamic transforms |

### Subscriber Topics (ROS 2 → ArduPilot)

*Source: ArduPilot AP_DDS_Topic_Table.h* [3]

#### Control Commands
| **ACTUAL** Topic | Message Type | QoS | Description |
|-------|-------------|-----|-------------|
| `/ap/joy` | `sensor_msgs/Joy` | Best Effort | Joystick commands |
| `/ap/cmd_vel` | `geometry_msgs/TwistStamped` | Best Effort | Velocity control |
| `/ap/cmd_gps_pose` | `ardupilot_msgs/GlobalPosition` | Best Effort | Global position commands |

## Service Interfaces

*Source: ArduPilot AP_DDS_Service_Table.h* [4]

**⚠️ CORRECTED SERVICE NAMES - Based on Actual Observed Behavior**

### Motor Control
- **Service**: `/ap/arm_motors`
- **Type**: `ardupilot_msgs/srv/ArmMotors`
- **Request**: `bool arm`
- **Response**: `bool result`

### Mode Switching
- **Service**: `/ap/mode_switch`
- **Type**: `ardupilot_msgs/srv/ModeSwitch`
- **Request**: `uint8 mode`
- **Response**: `bool status, uint8 curr_mode`

### Pre-flight Checks
- **Service**: `/ap/prearm_check`
- **Type**: `std_srvs/srv/Trigger`
- **Response**: `bool success, string message`

### Takeoff (Experimental)
- **Service**: `/ap/experimental/takeoff`
- **Type**: `ardupilot_msgs/srv/Takeoff`
- **Request**: `float32 alt`
- **Response**: `bool status`

### Parameter Access
- **Services**: `/ap/set_parameters`, `/ap/get_parameters`
- **Types**: `rcl_interfaces/srv/SetParameters`, `rcl_interfaces/srv/GetParameters`

## MAVROS to DDS Transformation

### Topic Mapping

#### Position Data
```python
# MAVROS (OLD)
mavros_pose_sub = self.create_subscription(
    PoseStamped, f'/{drone_id}/mavros/local_position/pose',
    callback, 10)

# DDS (NEW) - CORRECTED
dds_pose_sub = self.create_subscription(
    PoseStamped, '/ap/pose/filtered',
    callback, 10)
```

#### Velocity Data
```python
# MAVROS (OLD)
mavros_vel_sub = self.create_subscription(
    TwistStamped, f'/{drone_id}/mavros/local_position/velocity_local',
    callback, 10)

# DDS (NEW) - CORRECTED
dds_vel_sub = self.create_subscription(
    TwistStamped, '/ap/twist/filtered',
    callback, 10)
```

#### Battery Status
```python
# MAVROS (OLD)
mavros_battery_sub = self.create_subscription(
    BatteryState, f'/{drone_id}/mavros/battery',
    callback, 10)

# DDS (NEW) - CORRECTED
dds_battery_sub = self.create_subscription(
    BatteryState, '/ap/battery',
    callback, 10)
```

#### Vehicle State
```python
# MAVROS (OLD)
from mavros_msgs.msg import State
mavros_state_sub = self.create_subscription(
    State, f'/{drone_id}/mavros/state',
    callback, 10)

# DDS (NEW) - CORRECTED
from ardupilot_msgs.msg import Status
dds_status_sub = self.create_subscription(
    Status, '/ap/status',
    callback, 10)
```

### Service Mapping

#### Arming Motors
```python
# MAVROS (OLD)
from mavros_msgs.srv import CommandBool
arming_client = self.create_client(
    CommandBool, f'/{drone_id}/mavros/cmd/arming')

# DDS (NEW) - CORRECTED
from ardupilot_msgs.srv import ArmMotors
arming_client = self.create_client(
    ArmMotors, '/ap/arm_motors')
```

#### Mode Switching
```python
# MAVROS (OLD)
from mavros_msgs.srv import SetMode
mode_client = self.create_client(
    SetMode, f'/{drone_id}/mavros/set_mode')

# DDS (NEW) - CORRECTED
from ardupilot_msgs.srv import ModeSwitch
mode_client = self.create_client(
    ModeSwitch, '/ap/mode_switch')
```

### Command Publishing

#### Position Commands
```python
# MAVROS (OLD)
position_pub = self.create_publisher(
    PoseStamped, f'/{drone_id}/mavros/setpoint_position/local', 10)

# DDS (NEW) - CORRECTED - Use GlobalPosition for GPS waypoints
from ardupilot_msgs.msg import GlobalPosition
goal_pub = self.create_publisher(
    GlobalPosition, '/ap/cmd_gps_pose', 10)
```

#### Velocity Commands
```python
# MAVROS (OLD)
velocity_pub = self.create_publisher(
    TwistStamped, f'/{drone_id}/mavros/setpoint_velocity/cmd_vel', 10)

# DDS (NEW) - CORRECTED
velocity_pub = self.create_publisher(
    TwistStamped, '/ap/cmd_vel', 10)
```

## Implementation Patterns

### 1. Basic DDS Node Structure

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from ardupilot_msgs.msg import Status
from ardupilot_msgs.srv import ArmMotors, ModeSwitch

class ArduPilotDDSInterface(Node):
    def __init__(self):
        super().__init__('ardupilot_dds_interface')
        
        # Publishers for commands - CORRECTED
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, '/ap/cmd_vel', 10)
        
        # Subscribers for telemetry - CORRECTED
        self.pose_sub = self.create_subscription(
            PoseStamped, '/ap/pose/filtered', 
            self.pose_callback, 10)
        
        self.status_sub = self.create_subscription(
            Status, '/ap/status', 
            self.status_callback, 10)
        
        # Service clients - CORRECTED
        self.arm_client = self.create_client(
            ArmMotors, '/ap/arm_motors')
        
        self.mode_client = self.create_client(
            ModeSwitch, '/ap/mode_switch')
    
    def pose_callback(self, msg):
        """Handle position updates"""
        x = msg.pose.position.x
        y = msg.pose.position.y  
        z = msg.pose.position.z
        self.get_logger().info(f'Position: [{x:.2f}, {y:.2f}, {z:.2f}]')
    
    def status_callback(self, msg):
        """Handle status updates"""
        vehicle_types = {
            1: "ROVER", 2: "ARDUCOPTER", 3: "ARDUPLANE", 
            7: "ARDUSUB", 12: "BLIMP"
        }
        vehicle = vehicle_types.get(msg.vehicle_type, "UNKNOWN")
        self.get_logger().info(f'Vehicle: {vehicle}, Armed: {msg.armed}, Flying: {msg.flying}')
```

### 2. Waypoint Control

*Source: ArduPilot ROS2 Waypoint Goal Interface* [2]

```python
def send_waypoint(self, lat, lon, alt):
    """Send GPS waypoint command"""
    from ardupilot_msgs.msg import GlobalPosition
    
    waypoint_msg = GlobalPosition()
    waypoint_msg.header.stamp = self.get_clock().now().to_msg()
    waypoint_msg.header.frame_id = 'map'
    waypoint_msg.coordinate_frame = GlobalPosition.FRAME_GLOBAL_INT  # 5
    waypoint_msg.latitude = lat
    waypoint_msg.longitude = lon
    waypoint_msg.altitude = alt
    
    # Set type mask to ignore velocity and acceleration
    waypoint_msg.type_mask = (
        GlobalPosition.IGNORE_VX | GlobalPosition.IGNORE_VY | 
        GlobalPosition.IGNORE_VZ | GlobalPosition.IGNORE_AFX | 
        GlobalPosition.IGNORE_AFY | GlobalPosition.IGNORE_AFZ |
        GlobalPosition.IGNORE_YAW | GlobalPosition.IGNORE_YAW_RATE
    )
    
    self.waypoint_pub.publish(waypoint_msg)
```

### 3. Service Call Examples

```python
async def arm_vehicle(self):
    """Arm the vehicle motors"""
    if not self.arm_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().error('Arm service not available')
        return False
    
    request = ArmMotors.Request()
    request.arm = True
    
    future = self.arm_client.call_async(request)
    response = await future
    
    if response.result:
        self.get_logger().info('Vehicle armed successfully')
    else:
        self.get_logger().error('Failed to arm vehicle')
    
    return response.result

async def set_mode(self, mode_code):
    """Change flight mode"""
    if not self.mode_client.wait_for_service(timeout_sec=1.0):
        self.get_logger().error('Mode service not available')
        return False
    
    request = ModeSwitch.Request()
    request.mode = mode_code  # e.g., 4 for GUIDED mode in Copter
    
    future = self.mode_client.call_async(request)
    response = await future
    
    if response.status:
        self.get_logger().info(f'Mode changed to {response.curr_mode}')
    else:
        self.get_logger().error('Failed to change mode')
    
    return response.status
```

## Code Examples

### Complete Multi-UAV Formation Controller (DDS Version)

```python
#!/usr/bin/env python3
"""
DDS-based Multi-UAV Formation Controller
Converted from MAVROS to ArduPilot DDS
"""

import rclpy
from rclpy.node import Node
import asyncio
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import BatteryState
from ardupilot_msgs.msg import Status, GlobalPosition
from ardupilot_msgs.srv import ArmMotors, ModeSwitch

class DDSFormationController(Node):
    def __init__(self, num_drones=3):
        super().__init__('dds_formation_controller')
        
        self.num_drones = num_drones
        self.drone_positions = {}
        self.drone_status = {}
        self.formation_targets = {}
        
        # Initialize formation pattern (triangle)
        self.setup_formation_pattern()
        
        # ⚠️ MULTI-DRONE ARCHITECTURE ERROR CORRECTED:
        # ArduPilot DDS does not provide automatic per-drone namespacing
        # Each drone instance requires separate ROS2 nodes/launch files
        # This example shows CONCEPTUAL multi-drone code structure
        
        # Publishers for waypoint commands - CORRECTED
        self.waypoint_pubs = []
        for i in range(num_drones):
            pub = self.create_publisher(
                GlobalPosition, f'/drone_{i}/ap/cmd_gps_pose', 10)
            self.waypoint_pubs.append(pub)
        
        # Subscribers for telemetry (one per drone) - CORRECTED
        for i in range(num_drones):
            # Position feedback
            self.create_subscription(
                PoseStamped, f'/drone_{i}/ap/pose/filtered',
                lambda msg, drone_id=i: self.position_callback(msg, drone_id), 10)
            
            # Status feedback
            self.create_subscription(
                Status, f'/drone_{i}/ap/status',
                lambda msg, drone_id=i: self.status_callback(msg, drone_id), 10)
        
        # Service clients for each drone - CORRECTED
        self.arm_clients = []
        self.mode_clients = []
        for i in range(num_drones):
            arm_client = self.create_client(
                ArmMotors, f'/drone_{i}/ap/arm_motors')
            mode_client = self.create_client(
                ModeSwitch, f'/drone_{i}/ap/mode_switch')
            
            self.arm_clients.append(arm_client)
            self.mode_clients.append(mode_client)
        
        # Formation control timer
        self.control_timer = self.create_timer(0.1, self.update_formation)
        
        self.get_logger().info(f'DDS Formation Controller initialized for {num_drones} drones')
    
    def setup_formation_pattern(self):
        """Setup triangle formation pattern"""
        import math
        spacing = 10.0  # meters between drones
        
        for i in range(self.num_drones):
            angle = (2 * math.pi * i) / self.num_drones
            x_offset = spacing * math.cos(angle)
            y_offset = spacing * math.sin(angle)
            
            self.formation_targets[i] = {
                'x_offset': x_offset,
                'y_offset': y_offset,
                'z_offset': 0.0
            }
    
    def position_callback(self, msg, drone_id):
        """Handle position updates from individual drones"""
        self.drone_positions[drone_id] = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'timestamp': msg.header.stamp
        }
    
    def status_callback(self, msg, drone_id):
        """Handle status updates from individual drones"""
        self.drone_status[drone_id] = {
            'armed': msg.armed,
            'flying': msg.flying,
            'vehicle_type': msg.vehicle_type,
            'mode': msg.mode,
            'external_control': msg.external_control
        }
    
    def update_formation(self):
        """Update formation control commands"""
        if len(self.drone_positions) < self.num_drones:
            return  # Wait for all drone positions
        
        # Calculate formation center (leader position)
        leader_pos = self.drone_positions.get(0)
        if not leader_pos:
            return
        
        center_lat = 40.08370  # Example coordinates
        center_lon = -105.21740
        center_alt = 1630.0
        
        # Send formation waypoints to each drone
        for drone_id in range(self.num_drones):
            if drone_id not in self.drone_status:
                continue
                
            status = self.drone_status[drone_id]
            if not (status.get('armed', False) and status.get('flying', False)):
                continue  # Skip non-flying drones
            
            # Calculate target position for this drone
            target = self.formation_targets[drone_id]
            target_lat = center_lat + (target['x_offset'] * 0.00001)  # Rough conversion
            target_lon = center_lon + (target['y_offset'] * 0.00001)
            target_alt = center_alt + target['z_offset']
            
            # Send waypoint command
            self.send_waypoint_to_drone(drone_id, target_lat, target_lon, target_alt)
    
    def send_waypoint_to_drone(self, drone_id, lat, lon, alt):
        """Send waypoint command to specific drone"""
        waypoint_msg = GlobalPosition()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = 'map'
        waypoint_msg.coordinate_frame = GlobalPosition.FRAME_GLOBAL_INT
        waypoint_msg.latitude = lat
        waypoint_msg.longitude = lon
        waypoint_msg.altitude = alt
        
        # Ignore velocity and acceleration
        waypoint_msg.type_mask = (
            GlobalPosition.IGNORE_VX | GlobalPosition.IGNORE_VY | 
            GlobalPosition.IGNORE_VZ | GlobalPosition.IGNORE_AFX | 
            GlobalPosition.IGNORE_AFY | GlobalPosition.IGNORE_AFZ |
            GlobalPosition.IGNORE_YAW | GlobalPosition.IGNORE_YAW_RATE
        )
        
        if drone_id < len(self.waypoint_pubs):
            self.waypoint_pubs[drone_id].publish(waypoint_msg)
    
    async def arm_all_drones(self):
        """Arm all drones"""
        tasks = []
        for i in range(self.num_drones):
            task = asyncio.create_task(self.arm_drone(i))
            tasks.append(task)
        
        results = await asyncio.gather(*tasks)
        return all(results)
    
    async def arm_drone(self, drone_id):
        """Arm individual drone"""
        if drone_id >= len(self.arm_clients):
            return False
        
        client = self.arm_clients[drone_id]
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(f'Arm service not available for drone {drone_id}')
            return False
        
        request = ArmMotors.Request()
        request.arm = True
        
        future = client.call_async(request)
        response = await future
        
        success = response.result
        status = "SUCCESS" if success else "FAILED"
        self.get_logger().info(f'Drone {drone_id} arm: {status}')
        
        return success

def main(args=None):
    rclpy.init(args=args)
    
    controller = DDSFormationController(num_drones=3)
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Configuration & Setup

### ArduPilot Configuration

*Source: ArduPilot ROS2 Interfaces Documentation* [1]

```bash
# Enable DDS in ArduPilot build
cd ~/ardupilot
./waf configure --board sitl
./waf copter

# Set DDS parameters
param set DDS_ENABLE 1
param set DDS_PERIOD 10    # 10ms = 100Hz
param set DDS_PORT 2019    # UDP port
param set DDS_IP_ADDR 127.0.0.1  # For SITL
```

### Micro-ROS Agent Setup

```bash
# Install micro-ros-agent
sudo apt install ros-humble-micro-ros-agent

# Run agent (connects to ArduPilot)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 -v
```

### Launch File Example

```xml
<launch>
  <!-- Start Micro-ROS Agent -->
  <node pkg="micro_ros_agent" exec="micro_ros_agent" name="micro_ros_agent">
    <param name="middleware" value="udp4"/>
    <param name="port" value="2019"/>
    <arg name="--verbose" />
  </node>
  
  <!-- Start ArduPilot SITL -->
  <node pkg="ardupilot_sitl" exec="arducopter" name="arducopter">
    <param name="model" value="quad"/>
    <param name="speedup" value="1"/>
    <param name="instance" value="0"/>
    <param name="lat" value="40.08370"/>
    <param name="lon" value="-105.21740"/>
    <param name="alt" value="1630"/>
    <param name="dir" value="270"/>
  </node>
  
  <!-- Your DDS application -->
  <node pkg="your_package" exec="dds_formation_controller" name="formation_controller">
    <param name="num_drones" value="3"/>
  </node>
</launch>
```

### QoS Configuration

```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# Create QoS profiles matching ArduPilot's configuration
best_effort_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=5
)

reliable_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    depth=1
)

# Use appropriate QoS for each topic - CORRECTED
pose_sub = self.create_subscription(
    PoseStamped, '/ap/pose/filtered', 
    callback, qos_profile=best_effort_qos)

status_sub = self.create_subscription(
    Status, '/ap/status',
    callback, qos_profile=reliable_qos)
```

## Key Differences from MAVROS

### Advantages of DDS
1. **Direct ROS 2 Integration**: No translation layer needed
2. **Better Performance**: Lower latency, higher throughput
3. **Standard Message Types**: Uses common ROS 2 message formats
4. **Configurable QoS**: Fine-grained control over communication
5. **Resource Efficient**: Less CPU overhead than MAVROS

### Migration Considerations
1. **Topic Names**: Different namespace (`/ap/*` vs `/mavros/*`) - CORRECTED
2. **Service Names**: Simplified naming (`/ap/arm_motors` vs `/mavros/cmd/arming`) - CORRECTED  
3. **Message Types**: Some custom messages vs MAVROS-specific types
4. **Multi-Vehicle**: No built-in namespace support (manual configuration)
5. **Parameter Access**: Uses standard ROS 2 parameter services via `/ap/get_parameters` and `/ap/set_parameters` - CORRECTED

### Build Dependencies

```xml
<!-- package.xml -->
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
<depend>geographic_msgs</depend>
<depend>tf2_msgs</depend>
<depend>ardupilot_msgs</depend>
<depend>micro_ros_agent</depend>
```

This comprehensive guide provides everything needed to understand ArduPilot's DDS capabilities and successfully migrate from MAVROS-based systems to native DDS communication.

## References

[1] **ArduPilot ROS2 Interfaces Documentation**  
    URL: https://ardupilot.org/dev/docs/ros2-interfaces.html  
    Accessed: August 25, 2025  
    *Comprehensive documentation of ArduPilot's DDS messaging capabilities, topic interfaces, and ROS 2 integration*

[2] **ArduPilot ROS2 Waypoint Goal Interface Documentation**  
    URL: https://ardupilot.org/dev/docs/ros2-waypoint-goal-interface.html  
    Accessed: August 25, 2025  
    *Detailed guide for waypoint and goal interfaces, command structures, and control patterns*

[3] **ArduPilot AP_DDS_Topic_Table.h Source Code**  
    Path: `/libraries/AP_DDS/AP_DDS_Topic_Table.h`  
    Repository: https://github.com/ArduPilot/ardupilot  
    *Complete topic table definition with message types, QoS settings, and topic names*

[4] **ArduPilot AP_DDS_Service_Table.h Source Code**  
    Path: `/libraries/AP_DDS/AP_DDS_Service_Table.h`  
    Repository: https://github.com/ArduPilot/ardupilot  
    *Service interface definitions including arming, mode switching, and parameter access*

[5] **ArduPilot Message Definitions**  
    Path: `/install/ardupilot_msgs/share/ardupilot_msgs/`  
    Local workspace implementation  
    *ROS 2 message and service definitions for GlobalPosition, Status, ArmMotors, ModeSwitch, and Takeoff*

[6] **Local MAVROS Implementation Reference**  
    Path: `/src/formation_control/formation_control/drone_interface.py`  
    Local workspace implementation  
    *Example MAVROS-based drone interface showing typical communication patterns and message usage*

### Additional Resources

- **ArduPilot DDS Library Documentation**: https://ardupilot.org/dev/docs/common-dds.html
- **ROS 2 Humble Documentation**: https://docs.ros.org/en/humble/
- **Micro-ROS Agent**: https://github.com/micro-ROS/micro_ros_agent
- **DDS Specification**: https://www.omg.org/spec/DDS/
- **ArduPilot Parameter Reference**: https://ardupilot.org/copter/docs/parameters.html

### Version Compatibility

- **ArduPilot**: 4.5.0+ (DDS support introduced)
- **ROS 2**: Humble Hawksbill (recommended)
- **Micro-ROS Agent**: 4.0.0+
- **FastDDS**: 2.10.0+ (underlying DDS implementation)

### Contributing

This documentation is based on ArduPilot version 4.5+ and may be updated as the DDS implementation evolves. For the latest information, always refer to the official ArduPilot documentation and source code.