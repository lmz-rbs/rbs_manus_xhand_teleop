#!/bin/bash
# ============================================================================
# Manus -> GeoRT -> XHand Teleoperation - Environment Setup
# ============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEORT_DIR="$SCRIPT_DIR/../third_party/GeoRT"

echo "============================================"
echo "  Manus-XHand Teleoperation Setup"
echo "============================================"

# 1. Check Python version
echo ""
echo "[1/5] Checking Python..."
python3 --version
if [ $? -ne 0 ]; then
    echo "ERROR: Python 3.8+ required"
    exit 1
fi

# 2. Install Python dependencies
echo ""
echo "[2/5] Installing Python dependencies..."
pip3 install numpy scipy torch --quiet 2>/dev/null || true
pip3 install pyzmq pybind11 --quiet 2>/dev/null || true

# 3. Install SAPIEN (for visualization)
echo ""
echo "[3/5] Installing SAPIEN (for visualization)..."
pip3 install sapien --quiet 2>/dev/null || {
    echo "WARNING: SAPIEN installation failed. Visualization will not be available."
    echo "         You may need: pip3 install sapien==2.2.2"
}

# 4. Install XHand controller SDK
echo ""
echo "[4/5] Installing XHand controller SDK..."
XHAND_SDK_DIR="$(dirname "$SCRIPT_DIR")/../ManusRetargeting/xhand_control_sdk_py"
PY_VER=$(python3 -c "import sys; print(f'cp{sys.version_info.major}{sys.version_info.minor}')")
WHL=$(find "$XHAND_SDK_DIR" -name "xhand_controller*${PY_VER}*.whl" 2>/dev/null | head -1)
if [ -n "$WHL" ]; then
    pip3 install "$WHL" --quiet 2>/dev/null || {
        echo "WARNING: XHand SDK installation failed."
        echo "         Wheel: $WHL"
    }
else
    echo "WARNING: No matching XHand SDK wheel found for $PY_VER"
    echo "         Available: $(ls $XHAND_SDK_DIR/*.whl 2>/dev/null)"
fi

# 5. Check GeoRT
echo ""
echo "[5/5] Checking GeoRT..."
if [ -d "$GEORT_DIR" ]; then
    echo "  GeoRT found at: $GEORT_DIR"
    # Install GeoRT dependencies
    if [ -f "$GEORT_DIR/requirements.txt" ]; then
        pip3 install -r "$GEORT_DIR/requirements.txt" --quiet 2>/dev/null || true
    fi
else
    echo "WARNING: GeoRT not found at $GEORT_DIR"
    echo "         Training and visualization will not work."
fi

echo ""
echo "============================================"
echo "  Setup Complete!"
echo "============================================"
echo ""
echo "Quick start:"
echo "  # Test with simulation:"
echo "  cd $SCRIPT_DIR"
echo "  python3 teleop.py --mocap sim --hand sim"
echo ""
echo "  # With visualization:"
echo "  python3 teleop.py --mocap sim --hand sim --viz"
echo ""
echo "  # Full pipeline (needs trained model):"
echo "  python3 teleop.py --mocap zmq --ckpt xhand_geort_v1 --hand real --hand-port /dev/ttyUSB0 --viz"
