#!/bin/bash

################################################################################
# Branch Switching Script for ardu_ws Multi-Drone Workspace
#
# This script automates switching between 'swarm' and 'vision' branch
# configurations, including:
#   - Moving ardupilot1-5 in/out of storage
#   - Cleaning all build artifacts
#   - Switching git branches across all repositories
#   - Updating submodules
#   - Rebuilding the workspace
#
# Dependencies:
#   - ROS2 Humble (or set ROS2_SETUP_PATH to your ROS2 installation)
#   - Git
#   - Colcon build tools
#
# Environment Variables:
#   ROS2_SETUP_PATH - Path to ROS2 setup.bash (default: /opt/ros/humble/setup.bash)
#   ROS2_DISTRO     - ROS2 distribution name (default: humble)
#
# Usage:
  # Switch to swarm branches
  #./switch_branch.sh swarm

  # Switch to vision branches
  #./switch_branch.sh vision

  # Use custom ROS2 installation
  #ROS2_SETUP_PATH=/custom/path/setup.bash ./switch_branch.sh vision

  # Use different distro
  #ROS2_DISTRO=jazzy ./switch_branch.sh vision
################################################################################

set -e  # Exit on any error

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
WORKSPACE="/home/finn/Documents/ardu_ws"
STORAGE_DIR="/home/finn/Documents/Swarm Specific File Storage"
SRC_DIR="${WORKSPACE}/src"

# ROS2 Configuration
ROS2_DISTRO="${ROS2_DISTRO:-humble}"
ROS2_SETUP_PATH="${ROS2_SETUP_PATH:-/opt/ros/${ROS2_DISTRO}/setup.bash}"

# Gazebo Configuration
# Set GZ_VERSION to use sdformat14 (harmonic) instead of sdformat12 (fortress)
export GZ_VERSION="${GZ_VERSION:-harmonic}"

# Build Parallelism
# colcon's --parallel-workers only limits packages built at once; each package's
# compiler still defaults to one job per core. ros_gz_bridge at 16 jobs exhausts
# RAM and freezes the machine, so cap compile jobs per package.
export MAKEFLAGS="${MAKEFLAGS:--j2}"
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"

# Function to print colored messages
print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if directory exists
check_dir() {
    if [ ! -d "$1" ]; then
        print_error "Directory not found: $1"
        return 1
    fi
    return 0
}

# Function to switch git branch in a repository
switch_repo_branch() {
    local repo_path="$1"
    local branch_name="$2"
    local repo_name=$(basename "$repo_path")

    if [ ! -d "$repo_path" ]; then
        print_warning "Repository not found: $repo_name (skipping)"
        return 0
    fi

    if [ ! -d "$repo_path/.git" ]; then
        print_warning "$repo_name is not a git repository (skipping)"
        return 0
    fi

    echo "  Switching $repo_name to branch: $branch_name"
    cd "$repo_path"

    # Fetch latest changes
    git fetch --all 2>/dev/null || true

    # Switch branch
    if git checkout "$branch_name" 2>/dev/null; then
        print_success "$repo_name → $branch_name"
    else
        print_error "Failed to checkout $branch_name in $repo_name"
        return 1
    fi
}

# Main script starts here
echo ""
echo "=========================================="
echo "  ardu_ws Branch Switching Script"
echo "=========================================="
echo ""

# Validate argument
if [ $# -ne 1 ]; then
    print_error "Usage: $0 [swarm|vision]"
    exit 1
fi

MODE="$1"

if [ "$MODE" != "swarm" ] && [ "$MODE" != "vision" ]; then
    print_error "Invalid mode: $MODE"
    print_error "Usage: $0 [swarm|vision]"
    exit 1
fi

echo "Target configuration: $MODE"
echo ""

# Verify workspace exists
if ! check_dir "$WORKSPACE"; then
    exit 1
fi

cd "$WORKSPACE"

# Create storage directory if it doesn't exist
if [ ! -d "$STORAGE_DIR" ]; then
    print_step "Creating storage directory: $STORAGE_DIR"
    mkdir -p "$STORAGE_DIR"
    print_success "Storage directory created"
fi

################################################################################
# STEP 1: Move ArduPilot instances (1-5) - Vision mode only
################################################################################

print_step "[1/8] Managing ArduPilot instances..."

if [ "$MODE" == "vision" ]; then
    # Moving TO vision: Move ardupilot1-5 OUT to storage BEFORE build
    echo "  Vision mode: Moving ardupilot1-5 to storage"

    for i in 1 2 3 4 5; do
        if [ -d "$SRC_DIR/ardupilot${i}" ]; then
            echo "    Moving ardupilot${i} to storage..."
            mv "$SRC_DIR/ardupilot${i}" "$STORAGE_DIR/"
        else
            print_warning "ardupilot${i} not found in src/ (may already be in storage)"
        fi
    done

    print_success "ArduPilot instances moved to storage"

elif [ "$MODE" == "swarm" ]; then
    # Swarm mode: ardupilot1-5 will be moved AFTER build (in step 8)
    # to avoid duplicate package name conflicts during colcon build
    echo "  Swarm mode: ardupilot1-5 will be moved back after build completes"
    print_success "Proceeding with build first"
fi

echo ""

################################################################################
# STEP 2: Clean all build artifacts
################################################################################

print_step "[2/8] Cleaning build artifacts..."

# Clean ROS2 workspace builds
echo "  Removing build/, install/, log/ directories..."
rm -rf "$WORKSPACE/build"
rm -rf "$WORKSPACE/install"
rm -rf "$WORKSPACE/log"

# Clean main ArduPilot build (always present)
if [ -d "$SRC_DIR/ardupilot" ]; then
    echo "  Cleaning main ardupilot build..."
    cd "$SRC_DIR/ardupilot"
    ./waf clean 2>/dev/null || true
fi

# Clean in-tree builds
echo "  Cleaning in-tree builds..."
rm -rf "$SRC_DIR/ardupilot_gazebo/build" 2>/dev/null || true
rm -rf "$SRC_DIR/Micro-XRCE-DDS-Gen/build" 2>/dev/null || true

# Clean Python cache
echo "  Cleaning Python cache..."
find "$SRC_DIR" -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find "$SRC_DIR" -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true

print_success "Build artifacts cleaned"
echo ""

################################################################################
# STEP 3: Switch branches for all repositories
################################################################################

print_step "[3/8] Switching git branches..."

cd "$WORKSPACE"

# Define branch mappings
declare -A VISION_BRANCHES=(
    ["formation_control"]="main"
    ["ardupilot_gazebo"]="ros2"
    ["ardupilot_gz"]="humble"
    ["ardupilot_sitl_models"]="main"
    ["micro_ros_agent"]="humble"
    ["Micro-XRCE-DDS-Gen"]="master"
    ["ros_gz"]="humble"
    ["sdformat_urdf"]="humble"
    [".claude"]="master"
)

declare -A SWARM_BRANCHES=(
    ["formation_control"]="swarm"
    ["ardupilot_gazebo"]="swarm-ardupilot_gazebo"
    ["ardupilot_gz"]="swarm-ardupilot_gz"
    ["ardupilot_sitl_models"]="swarm-ardupilot_sitl_models"
    ["micro_ros_agent"]="swarm-micro_ros_agent"
    ["Micro-XRCE-DDS-Gen"]="swarm-Micro-XRCE-DDS-Gen"
    ["ros_gz"]="swarm-ros_gz"
    ["sdformat_urdf"]="swarm-sdformat_urdf"
    [".claude"]="swarm"
)

# Select the appropriate branch mapping
if [ "$MODE" == "vision" ]; then
    declare -n BRANCHES=VISION_BRANCHES
else
    declare -n BRANCHES=SWARM_BRANCHES
fi

# Switch branches for each repository
for repo in "${!BRANCHES[@]}"; do
    branch="${BRANCHES[$repo]}"
    # .claude is at workspace root, not in src/
    if [ "$repo" == ".claude" ]; then
        switch_repo_branch "$WORKSPACE/$repo" "$branch"
    else
        switch_repo_branch "$SRC_DIR/$repo" "$branch"
    fi
done

print_success "All branches switched"
echo ""

################################################################################
# STEP 4: Update git submodules
################################################################################

print_step "[4/8] Updating git submodules..."

# Update submodules in main ardupilot
if [ -d "$SRC_DIR/ardupilot" ]; then
    echo "  Updating submodules in ardupilot..."
    cd "$SRC_DIR/ardupilot"
    git submodule update --init --recursive
fi

# Update submodules in ardupilot1-5 (only in swarm mode)
if [ "$MODE" == "swarm" ]; then
    for i in 1 2 3 4 5; do
        if [ -d "$SRC_DIR/ardupilot${i}" ]; then
            echo "  Updating submodules in ardupilot${i}..."
            cd "$SRC_DIR/ardupilot${i}"
            git submodule update --init --recursive
        fi
    done
fi

print_success "Submodules updated"
echo ""

################################################################################
# STEP 5: Rebuild ArduPilot (main instance)
################################################################################

print_step "[5/8] Rebuilding ArduPilot SITL (main instance)..."

if [ -d "$SRC_DIR/ardupilot" ]; then
    cd "$SRC_DIR/ardupilot"
    echo "  Configuring ArduPilot for SITL..."
    ./waf configure --board sitl
    echo "  Building ArduCopter (limited to 2 jobs)..."
    ./waf copter -j1
    print_success "ArduPilot main instance built"
else
    print_warning "Main ardupilot directory not found"
fi

echo ""

################################################################################
# STEP 6: Build ardupilot_gazebo plugins with CMake
################################################################################

print_step "[6/8] Building ardupilot_gazebo plugins with CMake..."

if [ -d "/home/finn/Documents/ardu_ws/src/ardupilot_gazebo" ]; then
    echo "  Changing to ardupilot_gazebo directory..."
    cd /home/finn/Documents/ardu_ws/src/ardupilot_gazebo

    # Create build directory
    echo "  Creating build directory..."
    mkdir -p build
    cd build

    # Configure with CMake
    echo "  Configuring with CMake..."
    cmake .. -DCMAKE_BUILD_TYPE=Release

    # Build the plugins
    echo "  Building plugins..."
    make -j1

    # Verify plugins were built
    echo "  Verifying plugins were built..."
    ls -lh *.so

    print_success "ardupilot_gazebo plugins built successfully"
else
    print_warning "ardupilot_gazebo directory not found, skipping plugin build"
fi

echo ""

################################################################################
# STEP 7: Rebuild ROS2 workspace
################################################################################

print_step "[7/8] Rebuilding ROS2 workspace..."

# Source ROS2 environment (CRITICAL for clean builds)
# This sets AMENT_PREFIX_PATH which allows colcon to discover ROS2 installation
if [ -f "$ROS2_SETUP_PATH" ]; then
    echo "  Sourcing ROS2 ${ROS2_DISTRO} environment..."
    source "$ROS2_SETUP_PATH"
    print_success "ROS2 environment sourced from $ROS2_SETUP_PATH"
else
    print_error "ROS2 setup.bash not found at $ROS2_SETUP_PATH"
    print_error "Set ROS2_SETUP_PATH environment variable or install ROS2 ${ROS2_DISTRO}"
    exit 1
fi

# Validate that critical environment variables are set
if [ -z "$AMENT_PREFIX_PATH" ]; then
    print_error "AMENT_PREFIX_PATH not set after sourcing ROS2 setup"
    print_error "This indicates a problem with your ROS2 installation"
    exit 1
fi

if [ -z "$ROS_DISTRO" ]; then
    print_warning "ROS_DISTRO not set (expected 'humble')"
fi

echo "  Environment validated:"
echo "    ROS_DISTRO: ${ROS_DISTRO:-not set}"
echo "    AMENT_PREFIX_PATH: ${AMENT_PREFIX_PATH:0:60}..."
echo ""

cd "$WORKSPACE"
echo "  Running colcon build (limited to 2 parallel workers)..."
colcon build --parallel-workers 1 --executor sequential

print_success "ROS2 workspace built"
echo ""

################################################################################
# STEP 8: Source environment
################################################################################

print_step "[8/8] Sourcing environment..."

if [ -f "$WORKSPACE/install/setup.bash" ]; then
    source "$WORKSPACE/install/setup.bash"
    print_success "Environment sourced"
else
    print_error "install/setup.bash not found!"
    exit 1
fi

echo ""

################################################################################
# STEP 9: Move ArduPilot instances back (Swarm mode only)
################################################################################

if [ "$MODE" == "swarm" ]; then
    print_step "[9/9] Moving ArduPilot instances 1-5 back to workspace..."

    # Now it's safe to move ardupilot1-5 back after build is complete
    echo "  Swarm mode: Moving ardupilot1-5 from storage to workspace"

    for i in 1 2 3 4 5; do
        if [ -d "$STORAGE_DIR/ardupilot${i}" ]; then
            echo "    Moving ardupilot${i} back to workspace..."
            mv "$STORAGE_DIR/ardupilot${i}" "$SRC_DIR/"
        else
            print_warning "ardupilot${i} not found in storage (may already be in workspace)"
        fi
    done

    print_success "ArduPilot instances moved back to workspace"
    echo ""
fi

################################################################################
# Final verification
################################################################################

echo "=========================================="
echo "  Branch Switch Complete!"
echo "=========================================="
echo ""
echo "Configuration: $MODE"
echo ""

if [ "$MODE" == "swarm" ]; then
    echo "Swarm branch status:"
    echo "  - ArduPilot instances 1-5: In workspace (src/ardupilot1-5)"
    echo "  - All repositories on swarm branches"
    echo ""
    echo "Next steps:"
    echo "  1. Launch multi-drone simulation:"
    echo "     ros2 launch ardupilot_gz_bringup iris_multi_uav.launch.py rviz:=true"
else
    echo "Vision branch status:"
    echo "  - ArduPilot instances 1-5: In storage ($STORAGE_DIR)"
    echo "  - All repositories on vision/main branches"
    echo ""
    echo "Next steps:"
    echo "  1. Launch vision-based simulation"
    echo "  2. Run your vision processing nodes"
fi

echo ""
print_success "You can now use the workspace with the $MODE configuration"
echo ""
