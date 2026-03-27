#!/usr/bin/env bash
# setup.sh — Python 3.14 with ROS Noetic support
# Run with: ./setup.sh

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$PROJECT_ROOT/venv"
CATKIN_WS="$PROJECT_ROOT/catkin_ws"
REQUIREMENTS_FILE="$PROJECT_ROOT/requirements.txt"
ROS_SETUP="/opt/ros/noetic/setup.bash"

# ── 1. Check ROS Noetic ──────────────────────────────────────────────────────
if [ ! -f "$ROS_SETUP" ]; then
    echo "[setup] ERROR: ROS Noetic not found at $ROS_SETUP" >&2
    echo "[setup] Install ROS Noetic first: http://wiki.ros.org/noetic/Installation" >&2
    exit 1
fi

# ── 2. Check Python 3.14 and venv support ──────────────────────────────────────
if ! command -v python3.14 &> /dev/null; then
    echo "[setup] ERROR: python3.14 not found." >&2
    echo "[setup] Install with: sudo apt install python3.14 python3.14-venv python3.14-dev" >&2
    exit 1
fi

if ! python3.14 -m venv --help &> /dev/null; then
    echo "[setup] ERROR: python3.14-venv package not installed." >&2
    echo "[setup] Install with: sudo apt install python3.14-venv" >&2
    exit 1
fi

PYTHON=$(command -v python3.14)

# ── 3. Install ROS dependencies for the workspace ─────────────────────────────
if [ -d "$CATKIN_WS" ]; then
    echo "[setup] Installing ROS dependencies..."
    source "$ROS_SETUP"
    
    # Check if rosdep is installed
    if ! command -v rosdep &> /dev/null; then
        echo "[setup] Installing rosdep..."
        sudo apt update
        sudo apt install python3-rosdep
        sudo rosdep init
        rosdep update
    fi
    
    # Install missing ROS packages that rosdep can't resolve
    echo "[setup] Installing MAVROS packages..."
    sudo apt install -y ros-noetic-mavros ros-noetic-mavros-msgs ros-noetic-libmavconn ros-noetic-mavlink
    
    echo "[setup] Installing Gazebo packages..."
    sudo apt install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-gazebo-plugins
    
    # Install dependencies from the workspace (for packages that are resolvable)
    cd "$CATKIN_WS"
    rosdep install --from-paths src --ignore-src -r -y || true
    
    cd "$PROJECT_ROOT"
fi

# ── 4. Create virtual environment ─────────────────────────────────────────────
if [ -d "$VENV_DIR" ]; then
    rm -rf "$VENV_DIR"
fi

$PYTHON -m venv "$VENV_DIR"

if [ $? -ne 0 ] || [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo "[setup] ERROR: Failed to create virtual environment" >&2
    exit 1
fi

# ── 5. Install Python packages ────────────────────────────────────────────────
# Skip pip upgrade to avoid network issues
if [ ! -f "$REQUIREMENTS_FILE" ]; then
    echo "[setup] ERROR: requirements.txt not found" >&2
    exit 1
fi

"$VENV_DIR/bin/pip" install --timeout 120 --retries 5 -r "$REQUIREMENTS_FILE"
if [ $? -ne 0 ]; then
    echo "[setup] ERROR: Failed to install requirements" >&2
    exit 1
fi

# ── 6. Build catkin workspace ─────────────────────────────────────────────────
if [ -d "$CATKIN_WS" ]; then
    source "$ROS_SETUP"
    
    if [ -z "$ROS_DISTRO" ]; then
        echo "[setup] ERROR: ROS environment not properly sourced" >&2
        exit 1
    fi
    
    export PATH="$VENV_DIR/bin:$PATH"
    export PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH"
    
    cd "$CATKIN_WS"
    
    if [ -f "build/CMakeCache.txt" ]; then
        rm -rf build devel
    fi
    
    catkin_make -DPYTHON_EXECUTABLE="$VENV_DIR/bin/python"
    if [ $? -ne 0 ]; then
        echo "[setup] ERROR: catkin_make failed" >&2
        exit 1
    fi
    
    cd "$PROJECT_ROOT"
fi

# ── Done ──────────────────────────────────────────────────────────────────────
echo ""
echo "Setup complete"
echo ""
echo "Activate environment:"
echo "    source venv/bin/activate"
echo "Start program:"
echo "    python3 __main__.py"