#!/bin/bash

################################################################################
# Quick Branch Switching Script for ardu_ws
#
# This script only switches git branches across all repositories without
# cleaning, rebuilding, or moving any files.
#
# Usage:  
 # Switch to swarm branches
  #./switch_branches_only.sh swarm

  # Switch to vision branches
  #./switch_branches_only.sh vision
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
SRC_DIR="${WORKSPACE}/src"

# Function to print colored messages
print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
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
        echo -e "    ${GREEN}✓${NC} $repo_name → $branch_name"
    else
        print_error "Failed to checkout $branch_name in $repo_name"
        return 1
    fi
}

# Main script starts here
echo ""
echo "=========================================="
echo "  Quick Branch Switcher"
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
if [ ! -d "$WORKSPACE" ]; then
    print_error "Workspace not found: $WORKSPACE"
    exit 1
fi

cd "$WORKSPACE"

################################################################################
# Switch branches for all repositories
################################################################################

echo "Switching git branches..."
echo ""

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

echo ""
print_success "All branches switched to $MODE configuration"
echo ""
echo "Note: You may need to:"
echo "  1. Update git submodules: git submodule update --init --recursive"
echo "  2. Rebuild the workspace if needed: colcon build"
echo ""
