#!/usr/bin/env python3
"""
GeoRT trainer wrapper with SAPIEN 3 compatibility.
Usage: python3 train_xhand.py [--ckpt-tag TAG] [--human-data TAG] [--epochs N]
"""
import sys
import os

# 1. Load SAPIEN compat shim BEFORE anything else
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))
import sapien_compat

# 2. Add GeoRT to path
GEORT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "third_party", "GeoRT")
sys.path.insert(0, GEORT_ROOT)
os.chdir(GEORT_ROOT)

# 3. Run trainer
from geort.trainer import GeoRTTrainer
from geort.utils.config_utils import get_config
import argparse

parser = argparse.ArgumentParser(description="Train GeoRT retargeting for XHand")
parser.add_argument("--hand", type=str, default="xhand_right")
parser.add_argument("--human-data", type=str, default="human_alex")
parser.add_argument("--ckpt-tag", type=str, default="xhand_v1")
parser.add_argument("--w-chamfer", type=float, default=80.0)
parser.add_argument("--w-curvature", type=float, default=0.1)
parser.add_argument("--w-collision", type=float, default=0.0)
parser.add_argument("--w-pinch", type=float, default=1.0)
args = parser.parse_args()

config = get_config(args.hand)
trainer = GeoRTTrainer(config)

from geort.utils.path import get_human_data
human_data_path = get_human_data(args.human_data)
print(f"Training with human data: {human_data_path}")

trainer.train(
    human_data_path,
    tag=args.ckpt_tag,
    w_chamfer=args.w_chamfer,
    w_curvature=args.w_curvature,
    w_collision=args.w_collision,
    w_pinch=args.w_pinch,
)
