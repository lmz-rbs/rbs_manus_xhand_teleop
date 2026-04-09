#!/bin/bash
# ============================================================================
# Step 3: Run teleoperation
# ============================================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo "============================================"
echo "  Step 3: Manus -> GeoRT -> XHand Teleop"
echo "============================================"

MOCAP_MODE=${1:-"sim"}
HAND_MODE=${2:-"sim"}
CKPT_TAG=${3:-"xhand_geort_v1"}
HAND_PORT=${4:-"/dev/ttyUSB0"}

echo "Config:"
echo "  Mocap:  $MOCAP_MODE"
echo "  Hand:   $HAND_MODE"  
echo "  Model:  $CKPT_TAG"
echo "  Port:   $HAND_PORT"
echo ""

ARGS="--mocap $MOCAP_MODE --hand $HAND_MODE"

if [ "$CKPT_TAG" != "none" ]; then
    ARGS="$ARGS --ckpt $CKPT_TAG"
fi

if [ "$HAND_MODE" = "real" ]; then
    ARGS="$ARGS --hand-port $HAND_PORT"
fi

# Enable visualization by default
ARGS="$ARGS --viz"

echo "Running: python3 teleop.py $ARGS"
echo ""

python3 teleop.py $ARGS
