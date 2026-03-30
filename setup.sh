#!/usr/bin/env bash
# setup.sh — Python 3.14 with ROS Noetic support
# Run with: ./setup.sh

set -e  # Exit on any error

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
echo "[setup] ROS Noetic found at $ROS_SETUP"

# ── 2. Check Python 3.14 and venv support ────────────────────────────────────
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
echo "[setup] Python: $PYTHON ($(python3.14 --version))"

# ── 3. Install ROS dependencies for the workspace ────────────────────────────
if [ -d "$CATKIN_WS" ]; then
    echo "[setup] Installing ROS dependencies..."
    source "$ROS_SETUP"

    # Update package lists first — without this apt may not find ROS packages
    echo "[setup] Updating apt package lists..."
    sudo apt update

    # Check if rosdep is installed and initialized
    if ! command -v rosdep &> /dev/null; then
        echo "[setup] Installing rosdep..."
        sudo apt install -y python3-rosdep
    fi

    # rosdep init fails if already initialized — this is normal, ignore error
    sudo rosdep init 2>/dev/null || true
    rosdep update

    # Install MAVROS — required by RFID_Sensor_Plugin_gazebo (depends on mavros_msgs)
    # Without this, catkin_make will fail with "Could not find ... mavros_msgs"
    echo "[setup] Installing MAVROS packages (required by RFID plugin)..."
    if ! sudo apt install -y ros-noetic-mavros ros-noetic-mavros-msgs ros-noetic-libmavconn ros-noetic-mavlink; then
        echo "[setup] WARNING: Failed to install some MAVROS packages." >&2
        echo "[setup] If catkin_make fails later, install manually:" >&2
        echo "[setup]   sudo apt install ros-noetic-mavros ros-noetic-mavros-msgs" >&2
    fi

    # Install Gazebo ROS packages
    echo "[setup] Installing Gazebo packages..."
    if ! sudo apt install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control ros-noetic-gazebo-plugins; then
        echo "[setup] WARNING: Failed to install some Gazebo packages." >&2
        echo "[setup] If catkin_make fails later, install manually:" >&2
        echo "[setup]   sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control" >&2
    fi

    # Install remaining dependencies from the workspace
    cd "$CATKIN_WS"
    rosdep install --from-paths src --ignore-src -r -y || true

    cd "$PROJECT_ROOT"
fi

# ── 4. Create virtual environment ────────────────────────────────────────────
if [ -d "$VENV_DIR" ]; then
    echo "[setup] Removing old virtual environment..."
    rm -rf "$VENV_DIR"
fi

echo "[setup] Creating virtual environment..."
$PYTHON -m venv "$VENV_DIR"

if [ ! -f "$VENV_DIR/bin/activate" ]; then
    echo "[setup] ERROR: Failed to create virtual environment" >&2
    exit 1
fi

# ── 5. Install Python packages ───────────────────────────────────────────────
echo "[setup] Installing Python dependencies..."
"$VENV_DIR/bin/pip" install --upgrade pip

if [ ! -f "$REQUIREMENTS_FILE" ]; then
    echo "[setup] ERROR: requirements.txt not found" >&2
    exit 1
fi

"$VENV_DIR/bin/pip" install -r "$REQUIREMENTS_FILE"

# ── 6. Build catkin workspace ────────────────────────────────────────────────
if [ -d "$CATKIN_WS" ]; then
    echo "[setup] Building catkin workspace..."
    source "$ROS_SETUP"

    if [ -z "$ROS_DISTRO" ]; then
        echo "[setup] ERROR: ROS environment not properly sourced" >&2
        exit 1
    fi

    export PATH="$VENV_DIR/bin:$PATH"
    export PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages:$PYTHONPATH"

    cd "$CATKIN_WS"

    # Clean previous build if exists — avoids cmake cache conflicts
    if [ -f "build/CMakeCache.txt" ]; then
        echo "[setup] Cleaning old build..."
        rm -rf build devel
    fi

    catkin_make -DPYTHON_EXECUTABLE="$VENV_DIR/bin/python"

    cd "$PROJECT_ROOT"
fi

# ── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo "================================================================"
echo "  Setup complete!"
echo "================================================================"
echo ""
echo "To start the application, run these commands:"
echo ""
echo "    source /opt/ros/noetic/setup.bash"
echo "    source catkin_ws/devel/setup.bash"
echo "    source venv/bin/activate"
echo "    python3 __main__.py"
echo ""
echo "TIP: Add the first two lines to your ~/.bashrc to avoid"
echo "     typing them every time you open a new terminal."
echo ""