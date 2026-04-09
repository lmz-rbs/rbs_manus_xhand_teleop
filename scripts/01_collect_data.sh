#!/bin/bash
# ============================================================================
# Step 1: Collect human hand data from Manus glove
# ============================================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

echo "============================================"
echo "  Step 1: Collect Human Hand Data"
echo "============================================"
echo ""
echo "Prerequisites:"
echo "  - Manus glove connected and broadcasting via ZMQ (port 8765)"
echo "  - OR: Run manus_mocap_core.py from GeoRT for ROS2+ZMQ bridge"
echo "  - OR: Use --mocap sim for testing with synthetic data"
echo ""

MOCAP_MODE=${1:-"sim"}
TAG=${2:-"human_xhand"}
DURATION=${3:-30}

echo "Config: mode=$MOCAP_MODE, tag=$TAG, duration=${DURATION}s"
echo "Starting data collection in 3 seconds..."
sleep 3

python3 -c "
import sys
sys.path.insert(0, 'src')
from manus_bridge import ManusGloveBridge
from retargeting import collect_human_data
import logging
logging.basicConfig(level=logging.INFO)

bridge = ManusGloveBridge(mode='$MOCAP_MODE')
collect_human_data(bridge, tag='$TAG', duration_sec=$DURATION, rate_hz=30)
bridge.close()
"

echo ""
echo "Data collection complete!"
echo "Data saved with tag: $TAG"
echo ""
echo "Next step: Run scripts/02_train.sh $TAG"
