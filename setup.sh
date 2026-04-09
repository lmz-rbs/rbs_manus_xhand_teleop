#!/bin/bash
# ============================================================================
# Manus -> GeoRT -> XHand Teleoperation - Environment Setup (uv)
# ============================================================================
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GEORT_DIR="$SCRIPT_DIR/third_party/GeoRT"

echo "============================================"
echo "  Manus-XHand Teleoperation Setup (uv)"
echo "============================================"

# 1. Install uv (if not present)
echo ""
echo "[1/5] Checking uv..."
if ! command -v uv &>/dev/null; then
    echo "  Installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    export PATH="$HOME/.local/bin:$PATH"
fi
uv --version

# 2. Create venv
echo ""
echo "[2/5] Creating .venv (Python 3.10)..."
cd "$SCRIPT_DIR"
uv venv --python 3.10

# 3. Install PyTorch + core deps
echo ""
echo "[3/5] Installing PyTorch + core deps..."
uv pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
uv pip install "numpy<2" scipy pyzmq tqdm trimesh open3d sapien

# 4. Install GeoRT (editable) + XHand SDK
echo ""
echo "[4/5] Installing GeoRT + XHand SDK..."
if [ -d "$GEORT_DIR" ]; then
    uv pip install -e "$GEORT_DIR"
    echo "  GeoRT installed (editable) from $GEORT_DIR"
else
    echo "  WARNING: GeoRT not found at $GEORT_DIR"
    echo "  Run first: git clone https://github.com/facebookresearch/GeoRT.git third_party/GeoRT"
fi

# XHand controller wheel (search nearby dirs)
XHAND_WHL=$(find "$SCRIPT_DIR" "$(dirname "$SCRIPT_DIR")" -maxdepth 4 \
    -name "xhand_controller*cp310*.whl" 2>/dev/null | head -1)
if [ -n "$XHAND_WHL" ]; then
    uv pip install "$XHAND_WHL"
    echo "  XHand SDK installed: $XHAND_WHL"
else
    echo "  WARNING: xhand_controller wheel not found."
    echo "  Install manually: uv pip install /path/to/xhand_controller-*.whl"
fi

# 5. Verify
echo ""
echo "[5/5] Verifying installation..."
.venv/bin/python - <<'PYEOF'
import torch; print(f"  torch {torch.__version__}  cuda={torch.cuda.is_available()}")
import sapien; print(f"  sapien {sapien.__version__}")
import zmq; print(f"  zmq {zmq.__version__}")
import numpy; print(f"  numpy {numpy.__version__}")
try:
    import geort; print("  geort OK")
except Exception as e:
    print(f"  geort NOT FOUND: {e}")
PYEOF

echo ""
echo "============================================"
echo "  Setup Complete!"
echo "============================================"
echo ""
echo "Usage:"
echo "  source .venv/bin/activate"
echo "  python teleop.py --mocap sim --hand sim        # simulation test"
echo "  python teleop.py --mocap zmq --ckpt xhand_v1  # with real Manus"
