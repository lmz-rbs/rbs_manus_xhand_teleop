#!/bin/bash
# ============================================================================
# Step 2: Train GeoRT retargeting model for XHand
# ============================================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$SCRIPT_DIR/.."
cd "$PROJECT_DIR"

echo "============================================"
echo "  Step 2: Train GeoRT Retargeting Model"
echo "============================================"

HUMAN_DATA=${1:-"human_alex"}
CKPT_TAG=${2:-"xhand_v1"}

GEORT_DIR="$PROJECT_DIR/third_party/GeoRT"

# Ensure config + assets are in GeoRT
cp config/xhand_right.json "$GEORT_DIR/geort/config/"
mkdir -p "$GEORT_DIR/assets/xhand_right/meshes/"
cp assets/xhand_right/xhand_right_nocol.urdf "$GEORT_DIR/assets/xhand_right/"
cp assets/xhand_right/meshes/*.STL "$GEORT_DIR/assets/xhand_right/meshes/"

echo "Config: human_data=$HUMAN_DATA, ckpt=$CKPT_TAG"

# Use our wrapper (has SAPIEN compat layer)
.venv/bin/python train_xhand.py --human-data "$HUMAN_DATA" --ckpt-tag "$CKPT_TAG"

# Copy checkpoint back into project
mkdir -p checkpoints/
cp -r "$GEORT_DIR"/checkpoint/*"$CKPT_TAG"* checkpoints/ 2>/dev/null || true
cp "$GEORT_DIR"/checkpoint/fk_model_xhand_right.pth checkpoints/ 2>/dev/null || true

echo ""
echo "Training complete!"
echo "Checkpoints in: $PROJECT_DIR/checkpoints/"
