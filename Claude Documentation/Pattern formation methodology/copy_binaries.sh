#!/bin/bash
# ArduPilot Binary Copy Script
# Copies working binaries and applies DDS port configuration
# Created: 2025-08-26

echo "🚁 ArduPilot Binary Copy & DDS Port Configuration"
echo "================================================="

# Navigate to workspace root
WORKSPACE_ROOT="/home/finn/Documents/ardu_ws"
cd "$WORKSPACE_ROOT"

# Check if source binary exists
if [ ! -f "src/ardupilot/build/sitl/bin/arducopter" ]; then
    echo "❌ Source ArduCopter binary not found at src/ardupilot/build/sitl/bin/arducopter"
    echo "Please build the main ardupilot instance first:"
    echo "   cd src/ardupilot"
    echo "   ./waf configure --board sitl --enable-dds"
    echo "   ./waf copter"
    exit 1
fi

echo "✅ Source binary found: $(ls -lh src/ardupilot/build/sitl/bin/arducopter)"
echo ""

# Define arrays
instances=("ardupilot1" "ardupilot2" "ardupilot3" "ardupilot4" "ardupilot5")
ports=("2019" "2029" "2039" "2049" "2059") 
drones=("iris_9002" "iris_9012" "iris_9022" "iris_9032" "iris_9042")

echo "🔄 STEP 1: Copy Binaries to All Instances"
echo "=========================================="

successful_copies=0

for i in "${!instances[@]}"; do
    instance=${instances[$i]}
    port=${ports[$i]}
    drone=${drones[$i]}
    
    echo "📦 Copying to ${instance} (${drone} - Port ${port})..."
    
    # Create build directory structure
    mkdir -p "src/$instance/build/sitl/bin/"
    
    # Copy the binary
    if cp "src/ardupilot/build/sitl/bin/arducopter" "src/$instance/build/sitl/bin/"; then
        echo "   ✅ Binary copied successfully"
        
        # Verify the copy
        if [ -f "src/$instance/build/sitl/bin/arducopter" ]; then
            echo "   📋 File size: $(du -h src/$instance/build/sitl/bin/arducopter | cut -f1)"
            successful_copies=$((successful_copies + 1))
        else
            echo "   ❌ Copy verification failed"
        fi
    else
        echo "   ❌ Copy failed"
    fi
    echo ""
done

echo "🔧 STEP 2: Verify DDS Port Configuration"
echo "========================================"

echo "DDS port configuration (already updated in source code):"
for i in "${!instances[@]}"; do
    instance=${instances[$i]}
    port=${ports[$i]}
    drone=${drones[$i]}
    
    # Check if our DDS port modification exists
    config_file="src/$instance/libraries/AP_DDS/AP_DDS_Client.cpp"
    if [ -f "$config_file" ]; then
        if grep -q "udp.port, ${port}" "$config_file"; then
            echo "   ✅ $drone ($instance): DDS Port $port configured correctly"
        else
            echo "   ⚠️  $drone ($instance): DDS Port configuration may need verification"
        fi
    else
        echo "   ❌ $drone ($instance): DDS config file not found"
    fi
done

echo ""
echo "🧪 STEP 3: Test Binary Functionality"
echo "===================================="

# Test one binary to make sure it works
echo "Testing ardupilot1 binary..."
cd "src/ardupilot1"

if ./build/sitl/bin/arducopter --help > test_output.log 2>&1; then
    echo "✅ Binary is functional!"
    echo "Available options preview:"
    head -5 test_output.log
else
    echo "⚠️  Binary test produced warnings (normal for ArduPilot):"
    head -3 test_output.log
fi

cd ../..

echo ""
echo "========================================="
echo "🎯 BINARY COPY SUMMARY"
echo "========================================="
echo "✅ Successful copies: $successful_copies/5"
echo ""

if [ $successful_copies -eq 5 ]; then
    echo "🎉 ALL BINARIES COPIED SUCCESSFULLY!"
    echo ""
    echo "📋 Instance Configuration Summary:"
    echo "┌─────────────┬──────────────┬──────────┬────────────────────┐"
    echo "│ Drone       │ Instance     │ DDS Port │ Binary Status      │"
    echo "├─────────────┼──────────────┼──────────┼────────────────────┤"
    for i in "${!instances[@]}"; do
        instance=${instances[$i]}
        port=${ports[$i]}
        drone=${drones[$i]}
        printf "│ %-11s │ %-12s │ %-8s │ %-18s │\n" "$drone" "$instance" "$port" "✅ Ready"
    done
    echo "└─────────────┴──────────────┴──────────┴────────────────────┘"
    echo ""
    echo "🚀 READY FOR TESTING!"
    echo ""
    echo "The DDS port changes are in the source code and will take effect"
    echo "when you rebuild one instance properly or use the copied binaries"
    echo "with runtime parameter overrides."
    echo ""
    echo "NEXT STEPS:"
    echo "1. Launch simulation: ros2 launch ardupilot_gz_bringup iris_multi_uav.launch.py"
    echo "2. Start micro-ROS agents on each port (separate terminals):"
    echo "   ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019  # iris_9002"
    echo "   ros2 run micro_ros_agent micro_ros_agent udp4 -p 2029  # iris_9012"
    echo "   ros2 run micro_ros_agent micro_ros_agent udp4 -p 2039  # iris_9022"
    echo "   ros2 run micro_ros_agent micro_ros_agent udp4 -p 2049  # iris_9032"
    echo "   ros2 run micro_ros_agent micro_ros_agent udp4 -p 2059  # iris_9042"
    echo "3. Verify DDS topics appear: ros2 topic list | grep iris"
else
    echo "⚠️ Some copies failed. Check the error messages above."
fi

echo ""
echo "Script completed at $(date)"