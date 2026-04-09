"""
GeoRT Retargeting Wrapper for XHand.

This module wraps GeoRT's training and inference APIs to work with
the XHand configuration. It handles:
  - Training a retargeting model for XHand (12-DOF)
  - Loading trained checkpoints for inference
  - Converting human hand keypoints (21,3) -> XHand joint angles (12,)
"""

import sys
import os
import json
import numpy as np
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

# Project root
PROJECT_ROOT = Path(__file__).parent.parent.resolve()
GEORT_ROOT = PROJECT_ROOT / "third_party" / "GeoRT"
CONFIG_DIR = PROJECT_ROOT / "config"
ASSETS_DIR = PROJECT_ROOT / "assets"
CHECKPOINT_DIR = PROJECT_ROOT / "checkpoints"


def setup_geort_path():
    """Add GeoRT to Python path."""
    geort_path = str(GEORT_ROOT)
    if geort_path not in sys.path:
        sys.path.insert(0, geort_path)
    logger.info(f"GeoRT root: {GEORT_ROOT}")


def load_xhand_config(config_name="xhand_right"):
    """Load XHand configuration JSON."""
    config_path = CONFIG_DIR / f"{config_name}.json"
    with open(config_path, 'r') as f:
        config = json.load(f)
    
    # Make URDF path absolute
    urdf_path = config["urdf_path"]
    if not os.path.isabs(urdf_path):
        config["urdf_path"] = str(PROJECT_ROOT / urdf_path)
    
    logger.info(f"Loaded config: {config_name}, {len(config['joint_order'])} joints, "
                f"{len(config['fingertip_link'])} fingertips")
    return config


class XHandRetargeter:
    """
    GeoRT-based retargeting model for XHand.
    
    Usage:
        # Training
        retargeter = XHandRetargeter()
        retargeter.train(human_data_tag="human_data", ckpt_tag="xhand_v1", epochs=3000)
        
        # Inference
        retargeter = XHandRetargeter.load("xhand_v1")
        qpos = retargeter.forward(keypoints)  # (21,3) -> (12,)
    """

    def __init__(self, config=None):
        setup_geort_path()
        self.config = config or load_xhand_config()

    def train(self, human_data_tag="human_alex", ckpt_tag="xhand_geort_v1", 
              epochs=3000, lr=1e-3, batch_size=128, save_interval=500):
        """
        Train a GeoRT retargeting model for XHand.
        
        Args:
            human_data_tag: Tag for human hand data (collected via mocap)
            ckpt_tag: Checkpoint tag for saving
            epochs: Number of training epochs
            lr: Learning rate
            batch_size: Training batch size
            save_interval: Save checkpoint every N epochs
        """
        from geort.trainer import GeoRTTrainer
        from geort.utils.config_utils import save_json
        
        # Override GeoRT's config loading to use our config
        trainer = GeoRTTrainer(self.config)
        
        # Save config alongside checkpoint
        ckpt_dir = CHECKPOINT_DIR / ckpt_tag
        ckpt_dir.mkdir(parents=True, exist_ok=True)
        save_json(self.config, str(ckpt_dir / "config.json"))
        
        logger.info(f"Training GeoRT for XHand: epochs={epochs}, lr={lr}, ckpt={ckpt_tag}")
        logger.info(f"Human data: {human_data_tag}")
        
        # Train using GeoRT's pipeline
        trainer.run(
            human_data=human_data_tag,
            ckpt_tag=ckpt_tag,
            epochs=epochs,
            lr=lr,
            batch_size=batch_size,
            save_interval=save_interval,
        )
        
        logger.info(f"Training complete! Checkpoint saved to {ckpt_dir}")

    @classmethod
    def load(cls, ckpt_tag="xhand_geort_v1", ckpt_dir=None, epoch=0):
        """
        Load a trained retargeting model.
        
        Args:
            ckpt_tag: Checkpoint tag
            ckpt_dir: Override checkpoint directory (default: project checkpoints/)
            epoch: Specific epoch to load (0 = last)
        Returns:
            XHandRetargeter with loaded model
        """
        setup_geort_path()
        from geort.export import GeoRTRetargetingModel
        
        if ckpt_dir is None:
            ckpt_dir = CHECKPOINT_DIR
        
        # Find checkpoint
        ckpt_path = Path(ckpt_dir)
        
        # Try project checkpoints first, then GeoRT checkpoints
        search_dirs = [ckpt_path, PROJECT_ROOT / "checkpoints", GEORT_ROOT / "checkpoint"]
        
        model_dir = None
        for search_dir in search_dirs:
            if not search_dir.exists():
                continue
            for d in search_dir.iterdir():
                if d.is_dir() and ckpt_tag in d.name:
                    model_dir = d
                    break
            if model_dir:
                break
        
        if model_dir is None:
            raise FileNotFoundError(
                f"Checkpoint '{ckpt_tag}' not found in {[str(d) for d in search_dirs]}")
        
        if epoch > 0:
            model_path = model_dir / f"epoch_{epoch}.pth"
        else:
            model_path = model_dir / "last.pth"
        config_path = model_dir / "config.json"
        
        logger.info(f"Loading model from {model_dir}")
        
        instance = cls.__new__(cls)
        with open(config_path, 'r') as f:
            instance.config = json.load(f)
        instance._model = GeoRTRetargetingModel(
            model_path=str(model_path), 
            config_path=str(config_path)
        )
        return instance

    def forward(self, keypoints):
        """
        Retarget human hand keypoints to XHand joint angles.
        
        Args:
            keypoints: numpy array (21, 3) - human hand keypoints in canonical frame
        Returns:
            qpos: numpy array (12,) - XHand joint angles in radians
        """
        if not hasattr(self, '_model'):
            raise RuntimeError("Model not loaded. Use XHandRetargeter.load() or train() first.")
        
        qpos = self._model.forward(keypoints)
        return qpos

    def visualize(self, keypoints=None, qpos=None):
        """
        Visualize the retargeting result in SAPIEN viewer.
        
        Args:
            keypoints: (21, 3) human keypoints (will be retargeted)
            qpos: (12,) direct joint angles to visualize
        """
        from geort.env.hand import HandKinematicModel
        
        hand = HandKinematicModel.build_from_config(self.config, render=True)
        viewer_env = hand.get_viewer_env()
        
        if qpos is not None:
            hand.set_qpos_target(qpos)
        
        while True:
            viewer_env.update()
            
            if keypoints is not None and hasattr(self, '_model'):
                qpos = self.forward(keypoints)
                hand.set_qpos_target(qpos)


def collect_human_data(mocap_bridge, tag="human_xhand", duration_sec=30, rate_hz=30):
    """
    Collect human hand data from Manus glove for training.
    
    Args:
        mocap_bridge: ManusGloveBridge instance
        tag: Data tag for saving
        duration_sec: Collection duration
        rate_hz: Sampling rate
    Returns:
        Path to saved data file
    """
    setup_geort_path()
    from geort import save_human_data
    
    logger.info(f"Collecting human data for {duration_sec}s at {rate_hz}Hz...")
    logger.info("Move your hand through various poses!")
    
    import time
    data = []
    dt = 1.0 / rate_hz
    start = time.time()
    
    while time.time() - start < duration_sec:
        result = mocap_bridge.get()
        if result["status"] == "recording" and result["result"] is not None:
            data.append(result["result"].copy())
        time.sleep(dt)
    
    if len(data) == 0:
        logger.error("No data collected!")
        return None
    
    data = np.array(data)
    save_path = save_human_data(data, tag)
    logger.info(f"Collected {len(data)} frames, saved to {save_path}")
    return save_path


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="XHand GeoRT Retargeting")
    sub = parser.add_subparsers(dest="command")
    
    # Train
    train_p = sub.add_parser("train", help="Train retargeting model")
    train_p.add_argument("--human-data", type=str, default="human_alex")
    train_p.add_argument("--ckpt-tag", type=str, default="xhand_geort_v1")
    train_p.add_argument("--epochs", type=int, default=3000)
    train_p.add_argument("--lr", type=float, default=1e-3)
    
    # Visualize
    vis_p = sub.add_parser("visualize", help="Visualize hand model")
    vis_p.add_argument("--ckpt-tag", type=str, default=None)
    
    # Collect
    col_p = sub.add_parser("collect", help="Collect human data")
    col_p.add_argument("--tag", type=str, default="human_xhand")
    col_p.add_argument("--duration", type=int, default=30)
    col_p.add_argument("--mocap-mode", type=str, default="zmq")
    
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    
    if args.command == "train":
        retargeter = XHandRetargeter()
        retargeter.train(
            human_data_tag=args.human_data,
            ckpt_tag=args.ckpt_tag,
            epochs=args.epochs,
            lr=args.lr,
        )
    elif args.command == "visualize":
        if args.ckpt_tag:
            retargeter = XHandRetargeter.load(args.ckpt_tag)
        else:
            retargeter = XHandRetargeter()
        retargeter.visualize()
    elif args.command == "collect":
        from manus_bridge import ManusGloveBridge
        bridge = ManusGloveBridge(mode=args.mocap_mode)
        collect_human_data(bridge, tag=args.tag, duration_sec=args.duration)
        bridge.close()
    else:
        parser.print_help()
