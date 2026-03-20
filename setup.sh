#!/usr/bin/env bash
# setup.sh — Python 3.14 enforced version

# Detect if script is being sourced or executed
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    echo -e "\033[1;33m[setup]\033[0m Run with:  source setup.sh  — to also activate the venv in this shell."
    SOURCED=0
else
    SOURCED=1
fi

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$PROJECT_ROOT/venv"
CATKIN_WS="$PROJECT_ROOT/catkin_ws"

# ── colours ───────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info()  { echo -e "${GREEN}[setup]${NC} $*"; }
warn()  { echo -e "${YELLOW}[setup]${NC} $*"; }
error() { echo -e "${RED}[setup]${NC} $*" >&2; }

# ── 1. Force Python 3.14 ──────────────────────────────────────────────────────
if command -v python3.14 &> /dev/null; then
    PYTHON=$(command -v python3.14)
else
    error "python3.14 not found. Install it first."
    if [ "$SOURCED" -eq 1 ]; then
        return 1
    else
        exit 1
    fi
fi

PY_VERSION=$($PYTHON -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
info "Using $PYTHON  (Python $PY_VERSION)"

# ── 2. Create virtual environment ─────────────────────────────────────────────
if [ -d "$VENV_DIR" ]; then
    warn "venv already exists at $VENV_DIR — recreating for Python 3.14"
    rm -rf "$VENV_DIR"
fi

info "Creating venv at $VENV_DIR ..."
$PYTHON -m venv --system-site-packages "$VENV_DIR"
info "venv created"

# Activate
source "$VENV_DIR/bin/activate"
info "venv activated: $(which python)"

# ── 3. Upgrade pip ────────────────────────────────────────────────────────────
info "Upgrading pip ..."
pip install --upgrade pip --quiet

# ── 4. Install Python requirements ────────────────────────────────────────────
info "Installing Python packages ..."
pip install \
    PyQt5 \
    PyQt5-sip \
    opencv-python \
    numpy \
    PyYAML \
    rospkg \
    catkin_pkg \
    defusedxml \
    empy \
    --quiet

info "All Python packages installed"

# ── 5. catkin_make ────────────────────────────────────────────────────────────
if [ ! -d "$CATKIN_WS" ]; then
    warn "catkin_ws not found at $CATKIN_WS — skipping catkin_make"
else
    ROS_SETUP=""
    for candidate in \
        /opt/ros/noetic/setup.bash \
        /opt/ros/melodic/setup.bash \
        /opt/ros/kinetic/setup.bash; do
        if [ -f "$candidate" ]; then
            ROS_SETUP="$candidate"
            break
        fi
    done

    if [ -z "$ROS_SETUP" ]; then
        warn "No ROS installation found — skipping catkin_make"
    else
        info "Sourcing ROS from $ROS_SETUP ..."
        source "$ROS_SETUP"
        info "Running catkin_make in $CATKIN_WS ..."
        cd "$CATKIN_WS"
        catkin_make
        cd "$PROJECT_ROOT"
        info "catkin_make complete"
    fi
fi

# ── Done ──────────────────────────────────────────────────────────────────────
echo ""
info "Setup complete."

# ── 6. Activate in current shell ──────────────────────────────────────────────
if [ "$SOURCED" -eq 1 ]; then
    source "$VENV_DIR/bin/activate"
    info "venv activated in current shell: $(which python)"
    if [ -n "$ROS_SETUP" ]; then
        source "$ROS_SETUP"
        info "ROS sourced: $ROS_SETUP"
    fi
    cd "$PROJECT_ROOT"
else
    info "To activate the venv in this shell run:"
    echo ""
    echo "    source $VENV_DIR/bin/activate"
    echo ""
fi